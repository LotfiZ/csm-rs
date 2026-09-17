//! Match parameters.
//!
//! Grouped into sub-structs with `Default` impls. Boolean strategy switches
//! are represented as enums.

/// Correspondence search strategy.
///
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CorrespondenceSearch {
    /// Jump-table search, O(1)-ish per ray.
    #[default]
    Tricks,
    /// Naive full scan.
    Naive,
}

/// Distance metric minimized by the solver.
///
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DistanceMetric {
    /// Point-to-line (PlICP).
    #[default]
    PointToLine,
    /// Point-to-point.
    PointToPoint,
}

/// Sensor reading validity interval.
///
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReadingBounds {
    /// Minimum valid reading in metres (default 0.0).
    pub min: f64,
    /// Maximum valid reading in metres (default 1000.0).
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrectionLimits {
    /// Maximum rotation correction per iteration, in degrees (default 90.0).
    pub max_angular_deg: f64,
    /// Maximum translation correction per iteration, in metres (default 2.0).
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoppingCriteria {
    /// Maximum ICP iterations (default 1000).
    pub max_iterations: i32,
    /// Translation change below which ICP stops, in metres (default 0.0001).
    pub epsilon_xy: f64,
    /// Rotation change below which ICP stops, in radians (default 0.0001).
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrespondenceParams {
    /// Correspondence search strategy.
    pub search: CorrespondenceSearch,
    /// Distance metric minimized by the solver.
    pub metric: DistanceMetric,
    /// Maximum correspondence distance in metres (default 2.0).
    pub max_dist: f64,
    /// Assumed correspondence noise sigma, in metres (default 0.01).
    pub sigma: f64,
    /// Enable the surface-orientation compatibility test (default off).
    pub do_alpha_test: bool,
    /// Orientation difference allowed by the alpha test, in degrees (default 20.0).
    pub alpha_test_threshold_deg: f64,
    /// Enable the visibility test (default off).
    pub do_visibility_test: bool,
    /// Distance threshold for clustering adjacent rays, in metres (default 0.05).
    pub clustering_threshold: f64,
    /// Neighbours used to estimate surface orientation (default 3).
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutlierParams {
    /// Keep at most this fraction of correspondences, discarding the
    /// highest-error ones (default 0.95).
    pub max_perc: f64,
    /// Percentile used for the adaptive error threshold (default 0.7).
    pub adaptive_order: f64,
    /// Multiplier applied to the percentile error (default 2.0).
    pub adaptive_mult: f64,
    /// Forbid two correspondences sharing a reference point (default on).
    pub remove_doubles: bool,
}

impl Default for OutlierParams {
    fn default() -> Self {
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RestartParams {
    /// Enable restart from local perturbations (default on).
    pub enabled: bool,
    /// Mean correspondence error above which a restart is attempted
    /// (default 0.01).
    pub threshold_mean_error: f64,
    /// Translation perturbation for restarts, in metres (default 0.01).
    pub dt: f64,
    /// Rotation perturbation for restarts, in radians (default 1.5°).
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
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeightParams {
    /// Maximum-likelihood correspondence weighting.
    pub ml: bool,
    /// Per-ray sigma weighting.
    pub sigma: bool,
}

/// Top-level match parameters.
///
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Params {
    /// Sensor reading interval applied before correspondence search.
    pub reading_bounds: ReadingBounds,
    /// Maximum translation and rotation allowed for one correction.
    pub correction_limits: CorrectionLimits,
    /// Iteration limit and pose-delta stopping thresholds.
    pub stopping: StoppingCriteria,
    /// Correspondence strategy, metric, alpha, visibility, and orientation
    /// settings.
    pub correspondence: CorrespondenceParams,
    /// Duplicate and percentile/adaptive correspondence rejection settings.
    pub outliers: OutlierParams,
    /// Six-perturbation restart settings.
    pub restart: RestartParams,
    /// ML and per-ray sigma weighting switches.
    pub weights: WeightParams,
    /// Compute covariance and derivative outputs (default off).
    pub do_compute_covariance: bool,
    /// Run the smart and naive searches together as a debug check (default off).
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
        let finite = self.reading_bounds.min.is_finite()
            && self.reading_bounds.max.is_finite()
            && self.correction_limits.max_angular_deg.is_finite()
            && self.correction_limits.max_linear.is_finite()
            && self.stopping.epsilon_xy.is_finite()
            && self.stopping.epsilon_theta.is_finite()
            && self.correspondence.max_dist.is_finite()
            && self.correspondence.sigma.is_finite()
            && self.correspondence.alpha_test_threshold_deg.is_finite()
            && self.correspondence.clustering_threshold.is_finite()
            && self.outliers.max_perc.is_finite()
            && self.outliers.adaptive_order.is_finite()
            && self.outliers.adaptive_mult.is_finite()
            && self.restart.threshold_mean_error.is_finite()
            && self.restart.dt.is_finite()
            && self.restart.dtheta.is_finite();
        if !finite {
            return Err(ParamsError::NonFinite);
        }
        if self.reading_bounds.min < 0.0
            || self.reading_bounds.max <= self.reading_bounds.min
            || self.correction_limits.max_angular_deg < 0.0
            || self.correction_limits.max_linear < 0.0
            || self.stopping.epsilon_xy < 0.0
            || self.stopping.epsilon_theta < 0.0
        {
            return Err(ParamsError::InvalidRange);
        }
        if self.correspondence.max_dist <= 0.0
            || self.correspondence.sigma <= 0.0
            || self.correspondence.alpha_test_threshold_deg < 0.0
            || self.correspondence.clustering_threshold < 0.0
            || self.correspondence.orientation_neighbourhood < 1
            || !(0.0..=1.0).contains(&self.outliers.max_perc)
            || self.outliers.adaptive_order < 0.0
            || self.outliers.adaptive_mult <= 0.0
            || self.restart.threshold_mean_error < 0.0
            || self.restart.dt < 0.0
            || self.restart.dtheta < 0.0
        {
            return Err(ParamsError::InvalidRange);
        }
        if self.stopping.max_iterations < 0 {
            return Err(ParamsError::InvalidIterationLimit);
        }
        Ok(())
    }

    /// Validate and return the configuration for fluent construction.
    pub fn validated(self) -> Result<Self, ParamsError> {
        self.validate()?;
        Ok(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_documented_values() {
        // Pin the documented defaults against accidental edits.
        let p = Params::default();
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
        p = Params::default();
        p.correction_limits.max_angular_deg = -1.0;
        assert_eq!(p.validate(), Err(ParamsError::InvalidRange));
    }

    #[test]
    fn validation_error_has_stable_display_text() {
        assert_eq!(
            ParamsError::NonFinite.to_string(),
            "matcher parameters contain a non-finite value"
        );
    }
}
