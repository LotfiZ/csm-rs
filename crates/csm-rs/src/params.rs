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

/// Outlier rejection parameters.
///
/// C: `sm_params.outliers_*`
#[derive(Clone, Copy, Debug)]
pub struct OutlierParams {
    /// Keep at most this fraction of correspondences (discard highest-error).
    /// C: `outliers_maxPerc` (default 0.9)
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
            max_perc: 0.9,
            adaptive_order: 0.7,
            adaptive_mult: 2.0,
            remove_doubles: true,
        }
    }
}

/// Top-level match parameters.
///
/// C: `struct sm_params`
// TODO(port): remaining groups — corrections limits, stopping criteria,
// restart, orientation/alpha, visibility, weights, covariance, sensor noise,
// reading bounds. Each with C cross-references and sm_options.c defaults.
#[derive(Clone, Debug, Default)]
pub struct Params {
    pub correspondence_search: CorrespondenceSearch,
    pub outliers: OutlierParams,
}
