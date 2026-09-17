//! Scan-pair import, versioned session export, and replay.
//!
//! ## Import format (`csm-rs-scan-pair`, version 1)
//!
//! ```json
//! {
//!   "format": "csm-rs-scan-pair",
//!   "version": 1,
//!   "initial_guess": [0.0, 0.0, 0.0],
//!   "reference": { "kind": "polar", "angles": [...], "readings": [5.0, null, ...], "valid": [...] },
//!   "sensor":    { "kind": "cartesian", "points": [[8.0, 0.0], ...], "valid": [...] },
//!   "config": { }
//! }
//! ```
//!
//! - Polar rays are ordered by bearing, in radians; readings are metres and
//!   `null` marks a missing return (`valid` must be false there).
//! - Cartesian points are ordered by bearing; `angles` may supply explicit
//!   bearings, otherwise `atan2(y, x)` is derived.
//! - `sigma` and `true_alpha` are optional per-ray arrays for the weighting
//!   paths.
//! - `config` is an optional matching configuration (any [`SimConfig`] field);
//!   simulation-only fields are ignored.
//!
//! Imported data has no ground truth, so responses never fabricate one.

use csm_rs::{Matcher, Pose, PreparedPolarScan};
use serde::{Deserialize, Serialize};

use crate::scene::Scene;
use crate::simulation::{
    guess_rng, initial_guess, pose_at, relative_pose, scan_for, SessionRecord, SessionResult,
    SimConfig,
};

/// One scan in an imported pair.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ScanInput {
    Polar {
        angles: Vec<f64>,
        readings: Vec<Option<f64>>,
        valid: Vec<bool>,
        #[serde(default)]
        sigma: Option<Vec<f64>>,
        #[serde(default)]
        true_alpha: Option<Vec<f64>>,
    },
    Cartesian {
        points: Vec<[f64; 2]>,
        valid: Vec<bool>,
        #[serde(default)]
        angles: Option<Vec<f64>>,
    },
}

impl ScanInput {
    fn to_prepared(&self) -> Result<PreparedPolarScan, String> {
        match self {
            ScanInput::Polar {
                angles,
                readings,
                valid,
                sigma,
                true_alpha,
            } => {
                if readings.len() != angles.len() || valid.len() != angles.len() {
                    return Err("polar scan arrays must have equal length".to_owned());
                }
                let readings: Vec<f64> = readings
                    .iter()
                    .zip(valid)
                    .enumerate()
                    .map(|(i, (value, ok))| match value {
                        Some(value) => Ok(*value),
                        None if !ok => Ok(f64::NAN),
                        None => Err(format!("ray {i} is valid but has a missing reading")),
                    })
                    .collect::<Result<_, _>>()?;
                // Validate via the public contract before building storage.
                let _ = csm_rs::PolarScan::with_inputs(
                    angles,
                    &readings,
                    valid,
                    sigma.as_deref(),
                    true_alpha.as_deref(),
                )
                .map_err(|error| error.to_string())?;
                PreparedPolarScan::from_polar_with_inputs(
                    angles.clone(),
                    readings,
                    valid.clone(),
                    sigma.clone(),
                    true_alpha.clone(),
                )
                .map_err(|error| error.to_string())
            }
            ScanInput::Cartesian {
                points,
                valid,
                angles,
            } => {
                if let Some(angles) = angles {
                    PreparedPolarScan::from_cartesian_with_angles(points, angles, valid)
                        .map_err(|error| error.to_string())
                } else {
                    PreparedPolarScan::from_cartesian(points, valid)
                        .map_err(|error| error.to_string())
                }
            }
        }
    }
}

/// A complete scan-pair import document.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScanPair {
    pub format: String,
    #[serde(default = "default_format_version")]
    pub version: u32,
    pub reference: ScanInput,
    pub sensor: ScanInput,
    #[serde(default)]
    pub initial_guess: Option<[f64; 3]>,
    #[serde(default)]
    pub config: Option<SimConfig>,
}

fn default_format_version() -> u32 {
    1
}

impl ScanPair {
    fn validate(&self) -> Result<(), String> {
        if self.format != "csm-rs-scan-pair" {
            return Err(format!("unsupported import format '{}'", self.format));
        }
        if self.version != 1 {
            return Err(format!("unsupported import version {}", self.version));
        }
        Ok(())
    }
}

/// Result of matching an imported or replayed pair. There is no ground truth.
#[derive(Serialize, Deserialize)]
pub struct ImportResponse {
    pub reference: Vec<[f64; 2]>,
    pub sensor_unaligned: Vec<[f64; 2]>,
    pub sensor_aligned: Vec<[f64; 2]>,
    pub initial_pose: [f64; 3],
    pub estimated_pose: [f64; 3],
    pub valid: bool,
    pub accepted: bool,
    pub termination: String,
    pub iterations: i32,
    pub nvalid: i32,
    pub error: f64,
    pub covariance_status: String,
    pub extent: f64,
}

fn params_from(config: &SimConfig) -> csm_rs::Params {
    let mut params = csm_rs::Params::default();
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
    params.stopping.max_iterations = config.max_iterations;
    params.outliers.remove_doubles = config.remove_doubles;
    params.outliers.max_perc = config.outliers_max_perc;
    params.do_compute_covariance = config.do_compute_covariance;
    params
}

fn scan_points(scan: &PreparedPolarScan, transform: Pose) -> Vec<[f64; 2]> {
    let mut points = Vec::new();
    for i in 0..scan.len() {
        if !scan.valid()[i] {
            continue;
        }
        let local = [
            scan.readings()[i] * scan.angles()[i].cos(),
            scan.readings()[i] * scan.angles()[i].sin(),
        ];
        points.push(transform.transform_point(local));
    }
    points
}

/// Match an imported pair. Ground truth is never invented.
pub fn run_import(pair: &ScanPair) -> Result<ImportResponse, String> {
    pair.validate()?;
    let config = pair.config.clone().unwrap_or_default();
    let mut reference = pair.reference.to_prepared()?;
    let mut sensor = pair.sensor.to_prepared()?;
    let guess = pair
        .initial_guess
        .map(Pose::from_array)
        .unwrap_or(Pose::IDENTITY);
    let matcher = Matcher::new(params_from(&config)).map_err(|error| error.to_string())?;
    let outcome = matcher
        .match_prepared_from(&mut reference, &mut sensor, guess)
        .map_err(|error| error.to_string())?;

    Ok(ImportResponse {
        reference: scan_points(&reference, Pose::IDENTITY),
        sensor_unaligned: scan_points(&sensor, guess),
        sensor_aligned: scan_points(&sensor, outcome.pose),
        initial_pose: guess.to_array(),
        estimated_pose: outcome.pose.to_array(),
        valid: outcome.valid,
        accepted: outcome.accepted(),
        termination: format!("{:?}", outcome.termination),
        iterations: outcome.iterations,
        nvalid: outcome.nvalid,
        error: outcome.error,
        covariance_status: format!("{:?}", outcome.covariance_status),
        extent: bounding_extent(&reference, &sensor, guess, outcome.pose),
    })
}

fn bounding_extent(
    reference: &PreparedPolarScan,
    sensor: &PreparedPolarScan,
    guess: Pose,
    estimate: Pose,
) -> f64 {
    let mut largest: f64 = 1.0;
    for (scan, transform) in [
        (reference, Pose::IDENTITY),
        (sensor, guess),
        (sensor, estimate),
    ] {
        for point in scan_points(scan, transform) {
            largest = largest.max(point[0].abs()).max(point[1].abs());
        }
    }
    largest * 1.1
}

fn export_result(outcome: &csm_rs::MatchOutcome) -> SessionResult {
    SessionResult {
        estimated_pose: outcome.pose.to_array(),
        valid: outcome.valid,
        termination: format!("{:?}", outcome.termination),
        iterations: outcome.iterations,
        nvalid: outcome.nvalid,
        error: outcome.error,
    }
}

/// Generate the simulated frame for `config` and package a versioned session.
pub fn export_session(config: &SimConfig) -> Result<SessionRecord, String> {
    let scene = Scene::by_name(&config.scenario);
    let reference_pose = pose_at(&config.scenario, 0, config.motion);
    let sensor_pose = pose_at(&config.scenario, config.step, config.motion);
    let reference = scan_for(&scene, reference_pose, config, 0);
    let sensor = scan_for(&scene, sensor_pose, config, config.step);
    let truth = relative_pose(reference_pose, sensor_pose);
    let mut rng = guess_rng(config.seed, config.step);
    let guess = initial_guess(truth, config.initial_error, &mut rng);

    let mut reference_prepared = PreparedPolarScan::from_polar(
        reference.angles.clone(),
        reference.readings.clone(),
        reference.valid.clone(),
    )
    .map_err(|error| error.to_string())?;
    let mut sensor_prepared = PreparedPolarScan::from_polar(
        sensor.angles.clone(),
        sensor.readings.clone(),
        sensor.valid.clone(),
    )
    .map_err(|error| error.to_string())?;
    let matcher = Matcher::new(params_from(config)).map_err(|error| error.to_string())?;
    let outcome = matcher
        .match_prepared_from(&mut reference_prepared, &mut sensor_prepared, guess)
        .map_err(|error| error.to_string())?;

    Ok(SessionRecord {
        version: 1,
        scenario: config.scenario.clone(),
        reference_mode: config.reference_mode.clone(),
        seed: config.seed,
        step: config.step,
        config: config.clone(),
        reference_angles: reference.angles,
        reference_readings: option_readings(&reference.readings),
        reference_valid: reference.valid,
        sensor_angles: sensor.angles,
        sensor_readings: option_readings(&sensor.readings),
        sensor_valid: sensor.valid,
        initial_guess: guess.to_array(),
        result: export_result(&outcome),
    })
}

fn option_readings(readings: &[f64]) -> Vec<Option<f64>> {
    readings
        .iter()
        .map(|value| value.is_finite().then_some(*value))
        .collect()
}

/// Rerun the matcher on an exported session and compare with its stored result.
pub fn replay_session(record: &SessionRecord) -> Result<ImportResponse, String> {
    if record.version != 1 {
        return Err(format!("unsupported session version {}", record.version));
    }
    let pair = ScanPair {
        format: "csm-rs-scan-pair".to_owned(),
        version: 1,
        reference: ScanInput::Polar {
            angles: record.reference_angles.clone(),
            readings: record.reference_readings.clone(),
            valid: record.reference_valid.clone(),
            sigma: None,
            true_alpha: None,
        },
        sensor: ScanInput::Polar {
            angles: record.sensor_angles.clone(),
            readings: record.sensor_readings.clone(),
            valid: record.sensor_valid.clone(),
            sigma: None,
            true_alpha: None,
        },
        initial_guess: Some(record.initial_guess),
        config: Some(record.config.clone()),
    };
    let response = run_import(&pair)?;
    let stored = &record.result;
    if response.valid != stored.valid || response.termination != stored.termination {
        return Err(format!(
            "replay diverged: {} != {}",
            response.termination, stored.termination
        ));
    }
    for (a, b) in response.estimated_pose.iter().zip(stored.estimated_pose) {
        if (a - b).abs() > 1e-9 {
            return Err(format!("replay pose diverged: {a} != {b}"));
        }
    }
    Ok(response)
}
