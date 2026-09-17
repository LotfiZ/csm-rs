//! Local browser demo for csm-rs: serves a single page and runs the real Rust
//! matcher for every frame request.

mod scene;
mod simulation;

use axum::{
    routing::{get, post},
    Json, Router,
};
use csm_rs::{Matcher, Params, PolarScan, Pose};
use serde::{Deserialize, Serialize};

use scene::Scene;
use simulation::{
    guess_rng, initial_guess, pose_at, relative_pose, scan_for, ScanFrame, SimConfig,
};

const INDEX_HTML: &str = include_str!("../web/index.html");

/// Build the demo router (shared by the server and the workflow tests).
pub fn app() -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/frame", post(frame))
}

pub async fn index() -> axum::response::Html<&'static str> {
    axum::response::Html(INDEX_HTML)
}

#[derive(Serialize, Deserialize)]
pub struct FrameResponse {
    pub request_id: u64,
    pub scenario: String,
    pub reference_mode: String,
    pub step: u64,
    /// Total true sensor-to-reference-frame motion.
    pub truth_pose: [f64; 3],
    /// The initial relative guess supplied to the matcher.
    pub initial_pose: [f64; 3],
    /// Estimated total motion (accumulated across frames in previous-frame mode).
    pub estimated_pose: [f64; 3],
    /// True motion for the last matched pair (relative in previous-frame mode).
    pub relative_truth_pose: [f64; 3],
    /// Estimated motion for the last matched pair.
    pub relative_estimated_pose: [f64; 3],
    /// Accumulated estimation error, mapping truth into estimate coordinates.
    pub drift_pose: [f64; 3],
    pub valid: bool,
    /// True only for accepted convergence, never for an unsuccessful candidate.
    pub accepted: bool,
    pub termination: String,
    pub iterations: i32,
    pub nvalid: i32,
    pub error: f64,
    pub covariance_status: String,
    pub reference: Vec<[f64; 2]>,
    pub sensor_unaligned: Vec<[f64; 2]>,
    pub sensor_aligned: Vec<[f64; 2]>,
    pub sensor_true: Vec<[f64; 2]>,
    pub extent: f64,
    pub segments: Vec<[[f64; 2]; 2]>,
}

fn world_points(scan: &ScanFrame, transform: Pose) -> Vec<[f64; 2]> {
    let mut points = Vec::new();
    for i in 0..scan.angles.len() {
        if !scan.valid[i] {
            continue;
        }
        let local = [
            scan.readings[i] * scan.angles[i].cos(),
            scan.readings[i] * scan.angles[i].sin(),
        ];
        points.push(transform.transform_point(local));
    }
    points
}

fn params_from(config: &SimConfig) -> Params {
    let mut params = Params::default();
    params.restart.enabled = config.restart;
    params.correspondence.search = match config.search.as_str() {
        "naive" => csm_rs::CorrespondenceSearch::Naive,
        _ => csm_rs::CorrespondenceSearch::Tricks,
    };
    params.correspondence.metric = match config.metric.as_str() {
        "point_to_point" => csm_rs::DistanceMetric::PointToPoint,
        _ => csm_rs::DistanceMetric::PointToLine,
    };
    params.correspondence.max_dist = config.max_correspondence_dist;
    params.correspondence.do_alpha_test = config.do_alpha_test;
    params.correspondence.do_visibility_test = config.do_visibility_test;
    params.stopping.max_iterations = config.max_iterations;
    params.outliers.remove_doubles = config.remove_doubles;
    params.outliers.max_perc = config.outliers_max_perc;
    params.do_compute_covariance = config.do_compute_covariance;
    params
}

fn match_frames(
    matcher: &Matcher,
    reference: &ScanFrame,
    sensor: &ScanFrame,
    guess: Pose,
) -> Result<csm_rs::MatchOutcome, String> {
    let reference_scan = PolarScan::new(&reference.angles, &reference.readings, &reference.valid)
        .map_err(|error| error.to_string())?;
    let sensor_scan = PolarScan::new(&sensor.angles, &sensor.readings, &sensor.valid)
        .map_err(|error| error.to_string())?;
    matcher
        .match_polar_from(reference_scan, sensor_scan, guess)
        .map_err(|error| error.to_string())
}

/// State shared by both reference policies.
struct PolicyState {
    reference: ScanFrame,
    sensor: ScanFrame,
    guess: Pose,
    estimate: Pose,
    relative_truth: Pose,
    relative_estimate: Pose,
    outcome: csm_rs::MatchOutcome,
}

fn fixed_policy(
    scene: &Scene,
    config: &SimConfig,
    matcher: &Matcher,
) -> Result<PolicyState, String> {
    let reference_pose = pose_at(&config.scenario, 0, config.motion);
    let sensor_pose = pose_at(&config.scenario, config.step, config.motion);
    let reference = scan_for(scene, reference_pose, config, 0);
    let sensor = scan_for(scene, sensor_pose, config, config.step);
    let truth = relative_pose(reference_pose, sensor_pose);
    let mut rng = guess_rng(config.seed, config.step);
    let guess = initial_guess(truth, config.initial_error, &mut rng);
    let outcome = match_frames(matcher, &reference, &sensor, guess)?;
    Ok(PolicyState {
        reference,
        sensor,
        guess,
        estimate: outcome.pose,
        relative_truth: truth,
        relative_estimate: outcome.pose,
        outcome,
    })
}

fn previous_frame_policy(
    scene: &Scene,
    config: &SimConfig,
    matcher: &Matcher,
) -> Result<PolicyState, String> {
    let mut accumulated = Pose::IDENTITY;
    let mut state = None;
    let steps = config.step.max(1);
    for s in 1..=steps {
        let previous_pose = pose_at(&config.scenario, s - 1, config.motion);
        let current_pose = pose_at(&config.scenario, s, config.motion);
        let reference = scan_for(scene, previous_pose, config, s - 1);
        let sensor = scan_for(scene, current_pose, config, s);
        let relative_truth = relative_pose(previous_pose, current_pose);
        let mut rng = guess_rng(config.seed, s);
        let guess = initial_guess(relative_truth, config.initial_error, &mut rng);
        let outcome = match_frames(matcher, &reference, &sensor, guess)?;
        accumulated = accumulated.compose(outcome.pose);
        state = Some(PolicyState {
            reference,
            sensor,
            guess,
            estimate: accumulated,
            relative_truth,
            relative_estimate: outcome.pose,
            outcome,
        });
    }
    state.ok_or_else(|| "previous-frame policy requires a step".to_owned())
}

pub fn run_frame(config: &SimConfig) -> Result<FrameResponse, String> {
    let scene = Scene::by_name(&config.scenario);
    let matcher = Matcher::new(params_from(config)).map_err(|error| error.to_string())?;
    let state = if config.reference_mode == "previous_frame" {
        previous_frame_policy(&scene, config, &matcher)?
    } else {
        fixed_policy(&scene, config, &matcher)?
    };
    // Total truth depends only on the scenario motion.
    let total_truth = relative_pose(
        pose_at(&config.scenario, 0, config.motion),
        pose_at(&config.scenario, config.step, config.motion),
    );
    let drift = total_truth.inverse().compose(state.estimate);

    Ok(FrameResponse {
        request_id: config.request_id,
        scenario: scene.name.to_owned(),
        reference_mode: config.reference_mode.clone(),
        step: config.step,
        truth_pose: total_truth.to_array(),
        initial_pose: state.guess.to_array(),
        estimated_pose: state.estimate.to_array(),
        relative_truth_pose: state.relative_truth.to_array(),
        relative_estimated_pose: state.relative_estimate.to_array(),
        drift_pose: drift.to_array(),
        valid: state.outcome.valid,
        accepted: state.outcome.accepted(),
        termination: format!("{:?}", state.outcome.termination),
        iterations: state.outcome.iterations,
        nvalid: state.outcome.nvalid,
        error: state.outcome.error,
        covariance_status: format!("{:?}", state.outcome.covariance_status),
        reference: world_points(&state.reference, Pose::IDENTITY),
        sensor_unaligned: world_points(&state.sensor, state.guess),
        sensor_aligned: world_points(&state.sensor, state.relative_estimate),
        sensor_true: world_points(&state.sensor, state.relative_truth),
        extent: scene.extent,
        segments: scene.segments.iter().map(|s| [s.a, s.b]).collect(),
    })
}

async fn frame(Json(config): Json<SimConfig>) -> Result<Json<FrameResponse>, String> {
    run_frame(&config).map(Json)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_runs_the_real_matcher_and_tracks_truth() {
        let config = SimConfig {
            step: 4,
            initial_error: 0.05,
            ..SimConfig::default()
        };
        let response = run_frame(&config).expect("frame runs");
        assert!(response.valid, "a well-separated frame should match");
        assert!(response.reference.len() > 100);
        assert!(!response.sensor_aligned.is_empty());
        let dx = response.truth_pose[0] - response.estimated_pose[0];
        let dy = response.truth_pose[1] - response.estimated_pose[1];
        assert!(dx.hypot(dy) < 0.1, "estimated pose should track truth");
    }

    #[test]
    fn frame_is_reproducible_for_a_seed() {
        let config = SimConfig {
            step: 5,
            seed: 99,
            ..SimConfig::default()
        };
        let first = run_frame(&config).expect("frame runs");
        let second = run_frame(&config).expect("frame runs");
        assert_eq!(first.estimated_pose, second.estimated_pose);
        assert_eq!(first.reference, second.reference);
    }
}
