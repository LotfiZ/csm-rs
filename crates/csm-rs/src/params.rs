//! Match parameters: fully idiomatic config surface (grilling Q7).
//!
//! C: `struct sm_params` in `sm/csm/algos.h`; defaults in `sm/csm/sm_options.c`
//!
//! Every field documents its C counterpart. Grouped into sub-structs with
//! `Default` impls transcribed from `sm_options.c`. Strategy enums replace
//! C's boolean flags; phase-2 enhancements arrive as new enum variants
//! (grilling Q15).

/// Correspondence search strategy.
///
/// C: `sm_params.use_corr_tricks` (bool)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CorrespondenceSearch {
    /// Jump-table search, O(1)-ish per ray. C: `find_correspondences_tricks()`
    #[default]
    Tricks,
    /// Naive full scan. C: `find_correspondences()`
    Naive,
}

/// Distance metric minimized by the solver.
///
/// C: `sm_params.use_point_to_line_distance` (bool)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DistanceMetric {
    /// Point-to-line (PlICP). C: `use_point_to_line_distance = 1`
    #[default]
    PointToLine,
    /// Point-to-point. C: `use_point_to_line_distance = 0`
    PointToPoint,
}

/// Sensor reading validity interval.
///
/// C: `sm_params.min_reading`, `sm_params.max_reading`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReadingBounds {
    /// C: `min_reading` (default 0.0)
    pub min: f64,
    /// C: `max_reading` (default 1000.0)
    pub max: f64,
}

impl Default for ReadingBounds {
    fn default() -> Self {
        Self {
            min: 0.0,
            max: 1000.0,
        }
    }
}

/// Maximum per-iteration corrections.
///
/// C: `sm_params.max_angular_correction_deg`, `sm_params.max_linear_correction`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrectionLimits {
    /// C: `max_angular_correction_deg` (default 90.0)
    pub max_angular_deg: f64,
    /// C: `max_linear_correction` (default 2.0)
    pub max_linear: f64,
}

impl Default for CorrectionLimits {
    fn default() -> Self {
        Self {
            max_angular_deg: 90.0,
            max_linear: 2.0,
        }
    }
}

/// Stopping criteria.
///
/// C: `sm_params.max_iterations`, `sm_params.epsilon_xy`, `sm_params.epsilon_theta`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoppingCriteria {
    /// C: `max_iterations` (default 1000)
    pub max_iterations: i32,
    /// C: `epsilon_xy` (default 0.0001)
    pub epsilon_xy: f64,
    /// C: `epsilon_theta` (default 0.0001)
    pub epsilon_theta: f64,
}

impl Default for StoppingCriteria {
    fn default() -> Self {
        Self {
            max_iterations: 1000,
            epsilon_xy: 0.0001,
            epsilon_theta: 0.0001,
        }
    }
}

/// Correspondence finding configuration.
///
/// C: various `sm_params` fields
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrespondenceParams {
    /// C: `use_corr_tricks`
    pub search: CorrespondenceSearch,
    /// C: `use_point_to_line_distance`
    pub metric: DistanceMetric,
    /// C: `max_correspondence_dist` (default 2.0)
    pub max_dist: f64,
    /// C: `sigma` (default 0.01) — "dubious parameter (m)"
    pub sigma: f64,
    /// C: `do_alpha_test` (default 0)
    pub do_alpha_test: bool,
    /// C: `do_alpha_test_thresholdDeg` (default 20.0)
    pub alpha_test_threshold_deg: f64,
    /// C: `do_visibility_test` (default 0)
    pub do_visibility_test: bool,
    /// C: `clustering_threshold` (default 0.05)
    pub clustering_threshold: f64,
    /// C: `orientation_neighbourhood` (default 3)
    pub orientation_neighbourhood: i32,
}

impl Default for CorrespondenceParams {
    fn default() -> Self {
        Self {
            search: CorrespondenceSearch::Tricks,
            metric: DistanceMetric::PointToLine,
            max_dist: 2.0,
            sigma: 0.01,
            do_alpha_test: false,
            alpha_test_threshold_deg: 20.0,
            do_visibility_test: false,
            clustering_threshold: 0.05,
            orientation_neighbourhood: 3,
        }
    }
}

/// Outlier rejection parameters.
///
/// C: `sm_params.outliers_*`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutlierParams {
    /// Keep at most this fraction of correspondences (discard highest-error).
    /// C: `outliers_maxPerc` (default 0.95)
    pub max_perc: f64,
    /// Percentile for the adaptive threshold. C: `outliers_adaptive_order` (0.7)
    pub adaptive_order: f64,
    /// Multiplier over the percentile error. C: `outliers_adaptive_mult` (2.0)
    pub adaptive_mult: f64,
    /// Forbid two correspondences sharing a reference point.
    /// C: `outliers_remove_doubles` (1)
    pub remove_doubles: bool,
}

impl Default for OutlierParams {
    fn default() -> Self {
        // C: sm_options.c defaults
        Self {
            max_perc: 0.95,
            adaptive_order: 0.7,
            adaptive_mult: 2.0,
            remove_doubles: true,
        }
    }
}

/// Restart-from-perturbation configuration.
///
/// C: `sm_params.restart*` — note C's `restart` defaults to **1** (enabled).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RestartParams {
    /// C: `restart` (default 1)
    pub enabled: bool,
    /// C: `restart_threshold_mean_error` (default 0.01)
    pub threshold_mean_error: f64,
    /// C: `restart_dt` (default 0.01)
    pub dt: f64,
    /// C: `restart_dtheta` (default deg2rad(1.5))
    pub dtheta: f64,
}

impl Default for RestartParams {
    fn default() -> Self {
        Self {
            enabled: true,
            threshold_mean_error: 0.01,
            dt: 0.01,
            dtheta: 1.5f64.to_radians(),
        }
    }
}

/// Correspondence weighting schemes.
///
/// C: `sm_params.use_ml_weights`, `sm_params.use_sigma_weights` (both default 0)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeightParams {
    /// C: `use_ml_weights`
    pub ml: bool,
    /// C: `use_sigma_weights`
    pub sigma: bool,
}

/// Top-level match parameters.
///
/// C: `struct sm_params` in `sm/csm/algos.h`
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Params {
    /// First guess for the displacement. C: `first_guess`
    pub first_guess: [f64; 3],
    /// Robot's laser pose (used by `sm_icp_xy`, not `sm_icp`).
    /// C: `laser[3]` = laser_x, laser_y, laser_theta (all default 0.0)
    pub laser_pose: [f64; 3],
    /// Sensor reading interval applied before correspondence search.
    /// C: `min_reading`, `max_reading`
    pub reading_bounds: ReadingBounds,
    /// Maximum translation and rotation allowed for one correction.
    /// C: `max_angular_correction_deg`, `max_linear_correction`
    pub correction_limits: CorrectionLimits,
    /// Iteration limit and pose-delta stopping thresholds.
    /// C: `max_iterations`, `epsilon_xy`, `epsilon_theta`
    pub stopping: StoppingCriteria,
    /// Correspondence strategy, metric, alpha, visibility, and orientation
    /// settings. C: the corresponding fields of `struct sm_params`.
    pub correspondence: CorrespondenceParams,
    /// Duplicate and percentile/adaptive correspondence rejection settings.
    /// C: `outliers_maxPerc`, `outliers_adaptive_order`,
    /// `outliers_adaptive_mult`, `outliers_remove_doubles`
    pub outliers: OutlierParams,
    /// Six-perturbation restart settings. C: `restart*`
    pub restart: RestartParams,
    /// ML and per-ray sigma weighting switches.
    /// C: `use_ml_weights`, `use_sigma_weights`
    pub weights: WeightParams,
    /// C: `do_compute_covariance` (default 0)
    pub do_compute_covariance: bool,
    /// C: `debug_verify_tricks` (default 0)
    pub debug_verify_tricks: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamsError {
    NonFinite,
    InvalidRange,
    InvalidIterationLimit,
}

impl core::fmt::Display for ParamsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::NonFinite => "matcher parameters contain a non-finite value",
            Self::InvalidRange => "matcher parameters contain an invalid range",
            Self::InvalidIterationLimit => "matcher iteration limit must be non-negative",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ParamsError {}

impl Params {
    /// Validate numeric settings before constructing a matcher.
    pub fn validate(&self) -> Result<(), ParamsError> {
        let finite = self
            .first_guess
            .iter()
            .chain(self.laser_pose.iter())
            .all(|v| v.is_finite())
            && self.reading_bounds.min.is_finite()
            && self.reading_bounds.max.is_finite()
            && self.correction_limits.max_angular_deg.is_finite()
            && self.correction_limits.max_linear.is_finite()
            && self.stopping.epsilon_xy.is_finite()
            && self.stopping.epsilon_theta.is_finite();
        if !finite {
            return Err(ParamsError::NonFinite);
        }
        if self.reading_bounds.min < 0.0
            || self.reading_bounds.max <= self.reading_bounds.min
            || self.correction_limits.max_linear < 0.0
            || self.stopping.epsilon_xy < 0.0
            || self.stopping.epsilon_theta < 0.0
        {
            return Err(ParamsError::InvalidRange);
        }
        if self.stopping.max_iterations < 0 {
            return Err(ParamsError::InvalidIterationLimit);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_sm_options_c() {
        // Transcribed from sm/csm/sm_options.c — this test pins the
        // transcription against accidental edits.
        let p = Params::default();
        assert_eq!(p.first_guess, [0.0, 0.0, 0.0]);
        assert_eq!(p.laser_pose, [0.0, 0.0, 0.0]);
        assert_eq!(p.reading_bounds.min, 0.0);
        assert_eq!(p.reading_bounds.max, 1000.0);
        assert_eq!(p.correction_limits.max_angular_deg, 90.0);
        assert_eq!(p.correction_limits.max_linear, 2.0);
        assert_eq!(p.stopping.max_iterations, 1000);
        assert_eq!(p.stopping.epsilon_xy, 0.0001);
        assert_eq!(p.stopping.epsilon_theta, 0.0001);
        assert_eq!(p.correspondence.search, CorrespondenceSearch::Tricks);
        assert_eq!(p.correspondence.metric, DistanceMetric::PointToLine);
        assert_eq!(p.correspondence.max_dist, 2.0);
        assert_eq!(p.correspondence.sigma, 0.01);
        assert!(!p.correspondence.do_alpha_test);
        assert_eq!(p.correspondence.alpha_test_threshold_deg, 20.0);
        assert!(!p.correspondence.do_visibility_test);
        assert_eq!(p.correspondence.clustering_threshold, 0.05);
        assert_eq!(p.correspondence.orientation_neighbourhood, 3);
        assert_eq!(p.outliers.max_perc, 0.95);
        assert_eq!(p.outliers.adaptive_order, 0.7);
        assert_eq!(p.outliers.adaptive_mult, 2.0);
        assert!(p.outliers.remove_doubles);
        assert!(p.restart.enabled);
        assert_eq!(p.restart.threshold_mean_error, 0.01);
        assert_eq!(p.restart.dt, 0.01);
        assert!((p.restart.dtheta - 1.5f64.to_radians()).abs() < 1e-15);
        assert!(!p.weights.ml);
        assert!(!p.weights.sigma);
        assert!(!p.do_compute_covariance);
        assert!(!p.debug_verify_tricks);
    }

    #[test]
    fn validation_rejects_invalid_limits() {
        let mut p = Params::default();
        p.stopping.max_iterations = -1;
        assert_eq!(p.validate(), Err(ParamsError::InvalidIterationLimit));
        p = Params::default();
        p.reading_bounds.max = -1.0;
        assert_eq!(p.validate(), Err(ParamsError::InvalidRange));
    }
}
