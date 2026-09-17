//! Seeded sensor simulation: motion, noise, dropout, and scan generation.

use csm_rs::Pose;
use serde::{Deserialize, Serialize};

use crate::scene::{cast_ray, Scene};

/// Number of rays per scan. Fixed so prepared workspaces stay reusable.
pub const RAY_COUNT: usize = 181;
/// Angular span of the sensor, in radians, centered on the sensor heading.
pub const HALF_SPAN: f64 = 2.2;

/// A generated scan in sensor coordinates plus its world pose.
#[allow(dead_code)] // pose is used by later playback/reference-policy stages
pub struct ScanFrame {
    pub angles: Vec<f64>,
    pub readings: Vec<f64>,
    pub valid: Vec<bool>,
    pub pose: Pose,
}

/// Run-time controls. All fields affect actual matcher inputs.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct SimConfig {
    pub scenario: String,
    pub reference_mode: String,
    pub seed: u64,
    /// Motion scale: multiplies the per-step displacement and rotation.
    pub motion: f64,
    /// Standard deviation added to each valid range reading.
    pub noise: f64,
    /// Probability that a ray is dropped (marked invalid).
    pub dropout: f64,
    /// Magnitude of the initial-pose estimate error.
    pub initial_error: f64,
    /// Whether the restart shell is enabled.
    pub restart: bool,
    /// Correspondence search: "tricks" or "naive".
    pub search: String,
    /// Distance metric: "point_to_line" or "point_to_point".
    pub metric: String,
    pub max_correspondence_dist: f64,
    pub max_iterations: i32,
    pub do_compute_covariance: bool,
    /// Simulation step index (0 is the reference pose in fixed mode).
    pub step: u64,
    pub do_alpha_test: bool,
    pub do_visibility_test: bool,
    /// Whether to collect per-iteration instrumentation.
    pub trace: bool,
    pub remove_doubles: bool,
    pub outliers_max_perc: f64,
    pub request_id: u64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            scenario: "asymmetric_room".to_owned(),
            reference_mode: "fixed".to_owned(),
            seed: 7,
            motion: 1.0,
            noise: 0.01,
            dropout: 0.0,
            initial_error: 0.05,
            restart: true,
            search: "tricks".to_owned(),
            metric: "point_to_line".to_owned(),
            max_correspondence_dist: 2.0,
            max_iterations: 1000,
            do_compute_covariance: false,
            step: 0,
            do_alpha_test: false,
            do_visibility_test: false,
            trace: false,
            remove_doubles: true,
            outliers_max_perc: 0.95,
            request_id: 0,
        }
    }
}

/// Deterministic xorshift64* generator so recorded seeds replay exactly.
#[derive(Clone)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).max(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in [0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Approximately standard-normal via the sum of four uniforms.
    pub fn normal(&mut self) -> f64 {
        (self.unit() + self.unit() + self.unit() + self.unit() - 2.0) * 1.1547
    }
}

/// Sensor world pose for a scenario at `step`.
pub fn pose_at(scenario: &str, step: u64, motion: f64) -> Pose {
    let s = step as f64 * motion;
    match scenario {
        "ambiguous_corridor" => Pose::new(0.15 * s, 0.0, 0.02 * s),
        "partial_overlap" => Pose::new(-3.0 + 0.15 * s, 0.0, 0.01 * s),
        _ => {
            let angle = 0.12 * s;
            Pose::new(
                2.5 * angle.cos(),
                2.5 * angle.sin(),
                angle + std::f64::consts::FRAC_PI_2,
            )
        }
    }
}

/// Generate the scan for a specific simulation frame. The per-frame generator
/// is derived from the seed and frame index so frames are independent and
/// replay exactly regardless of request order.
pub fn scan_for(scene: &Scene, pose: Pose, config: &SimConfig, frame: u64) -> ScanFrame {
    let mut rng = Rng::new(
        config.seed
            ^ frame
                .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                .wrapping_add(0x1234_5678_9ABC_DEF1),
    );
    scan_at(scene, pose, config, &mut rng)
}

/// A per-frame generator for the initial-guess perturbation.
pub fn guess_rng(seed: u64, frame: u64) -> Rng {
    Rng::new(seed ^ frame.wrapping_mul(0x2545_F491_4F6C_DD1D))
}

/// Generate one scan at the given world pose.
pub fn scan_at(scene: &Scene, pose: Pose, config: &SimConfig, rng: &mut Rng) -> ScanFrame {
    let mut angles = Vec::with_capacity(RAY_COUNT);
    let mut readings = Vec::with_capacity(RAY_COUNT);
    let mut valid = Vec::with_capacity(RAY_COUNT);
    for i in 0..RAY_COUNT {
        let local = -HALF_SPAN + 2.0 * HALF_SPAN * i as f64 / (RAY_COUNT - 1) as f64;
        angles.push(local);
        let world_angle = local + pose.theta;
        let direction = [world_angle.cos(), world_angle.sin()];
        let hit = cast_ray([pose.x, pose.y], direction, &scene.segments);
        let dropped = config.dropout > 0.0 && rng.unit() < config.dropout;
        match hit {
            Some(distance) if !dropped => {
                let noisy = distance + config.noise * rng.normal();
                if noisy > 0.05 {
                    readings.push(noisy);
                    valid.push(true);
                } else {
                    readings.push(f64::NAN);
                    valid.push(false);
                }
            }
            _ => {
                readings.push(f64::NAN);
                valid.push(false);
            }
        }
    }
    ScanFrame {
        angles,
        readings,
        valid,
        pose,
    }
}

/// The true sensor-to-reference transform for a reference pose.
pub fn relative_pose(reference: Pose, sensor: Pose) -> Pose {
    reference.inverse().compose(sensor)
}

/// A perturbed initial guess for the matcher.
pub fn initial_guess(truth: Pose, error: f64, rng: &mut Rng) -> Pose {
    if error <= 0.0 {
        return truth;
    }
    Pose::new(
        truth.x + error * rng.normal(),
        truth.y + error * rng.normal(),
        truth.theta + 0.5 * error * rng.normal(),
    )
}

/// Matching result stored in an exported session.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SessionResult {
    pub estimated_pose: [f64; 3],
    pub valid: bool,
    pub termination: String,
    pub iterations: i32,
    pub nvalid: i32,
    pub error: f64,
}

/// A versioned export of a frame request for replay.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[allow(dead_code)] // used by the import/export/replay stage
pub struct SessionRecord {
    pub version: u32,
    pub scenario: String,
    pub reference_mode: String,
    pub seed: u64,
    pub step: u64,
    pub config: SimConfig,
    pub reference_angles: Vec<f64>,
    pub reference_readings: Vec<Option<f64>>,
    pub reference_valid: Vec<bool>,
    pub sensor_angles: Vec<f64>,
    pub sensor_readings: Vec<Option<f64>>,
    pub sensor_valid: Vec<bool>,
    pub initial_guess: [f64; 3],
    pub result: SessionResult,
}
