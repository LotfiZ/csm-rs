//! Local browser demo for csm-rs: serves a single page and runs the real Rust
//! matcher for every frame request.

mod scene;
mod simulation;

use axum::{
    routing::{get, post},
    Json, Router,
};
use csm_rs::{Matcher, Params, PolarScan, Pose};
use serde::Serialize;

use scene::Scene;
use simulation::{initial_guess, pose_at, relative_pose, scan_at, Rng, ScanFrame, SimConfig};

const INDEX_HTML: &str = include_str!("../web/index.html");

#[tokio::main]
async fn main() {
    let address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "127.0.0.1:7878".to_owned());
    let app = app();
    let listener = tokio::net::TcpListener::bind(&address)
        .await
        .expect("bind demo address");
    println!("csm-rs demo listening on http://{address}");
    axum::serve(listener, app).await.expect("serve demo");
}

/// Build the demo router (shared by the server and the workflow tests).
pub fn app() -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/frame", post(frame))
}

async fn index() -> axum::response::Html<&'static str> {
    axum::response::Html(INDEX_HTML)
}

#[derive(Serialize)]
struct FrameResponse {
    request_id: u64,
    scenario: String,
    reference_mode: String,
    step: u64,
    truth_pose: [f64; 3],
    initial_pose: [f64; 3],
    estimated_pose: [f64; 3],
    valid: bool,
    termination: String,
    iterations: i32,
    nvalid: i32,
    error: f64,
    covariance_status: String,
    reference: Vec<[f64; 2]>,
    sensor_unaligned: Vec<[f64; 2]>,
    sensor_aligned: Vec<[f64; 2]>,
    sensor_true: Vec<[f64; 2]>,
    extent: f64,
    segments: Vec<[[f64; 2]; 2]>,
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

fn run_frame(config: &SimConfig) -> Result<FrameResponse, String> {
    let scene = Scene::by_name(&config.scenario);
    let mut rng = Rng::new(config.seed);
    let sensor_pose = pose_at(&config.scenario, config.step, config.motion);
    let (reference_frame, truth) = if config.reference_mode == "previous_frame" {
        let reference_pose = pose_at(&config.scenario, config.step.saturating_sub(1), config.motion);
        let reference = scan_at(&scene, reference_pose, config, &mut rng);
        let truth = relative_pose(reference_pose, sensor_pose);
        (reference, truth)
    } else {
        let reference_pose = pose_at(&config.scenario, 0, config.motion);
        let reference = scan_at(&scene, reference_pose, config, &mut rng);
        let truth = relative_pose(reference_pose, sensor_pose);
        (reference, truth)
    };
    let sensor_frame = scan_at(&scene, sensor_pose, config, &mut rng);
    let guess = initial_guess(truth, config.initial_error, &mut rng);

    let reference_scan = PolarScan::new(
        &reference_frame.angles,
        &reference_frame.readings,
        &reference_frame.valid,
    )
    .map_err(|error| error.to_string())?;
    let sensor_scan = PolarScan::new(
        &sensor_frame.angles,
        &sensor_frame.readings,
        &sensor_frame.valid,
    )
    .map_err(|error| error.to_string())?;
    let matcher = Matcher::new(params_from(config)).map_err(|error| error.to_string())?;
    let outcome = matcher
        .match_polar_from(reference_scan, sensor_scan, guess)
        .map_err(|error| error.to_string())?;

    Ok(FrameResponse {
        request_id: config.request_id,
        scenario: scene.name.to_owned(),
        reference_mode: config.reference_mode.clone(),
        step: config.step,
        truth_pose: truth.to_array(),
        initial_pose: guess.to_array(),
        estimated_pose: outcome.pose.to_array(),
        valid: outcome.valid,
        termination: format!("{:?}", outcome.termination),
        iterations: outcome.iterations,
        nvalid: outcome.nvalid,
        error: outcome.error,
        covariance_status: format!("{:?}", outcome.covariance_status),
        reference: world_points(&reference_frame, Pose::IDENTITY),
        sensor_unaligned: world_points(&sensor_frame, guess),
        sensor_aligned: world_points(&sensor_frame, outcome.pose),
        sensor_true: world_points(&sensor_frame, truth),
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
