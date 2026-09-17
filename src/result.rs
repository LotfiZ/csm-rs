//! Match result.
//!
//! A failed match is a *result*, not an error: check [`MatchResult::valid`].
//! `Result` is reserved for malformed input.

use crate::math::{Mat3, Matrix};

/// Outcome of one scan match.
///
#[derive(Clone, Debug, Default)]
pub struct MatchResult {
    /// False when the match failed (too few correspondences, NaN first guess).
    /// Not an error condition.
    pub valid: bool,
    /// Scan-matching result `(x, y, theta)`.
    pub x: [f64; 3],
    /// Iterations performed.
    pub iterations: i32,
    /// Valid correspondences in the final iteration.
    pub nvalid: i32,
    /// Total correspondence error.
    pub error: f64,
    /// Closed-form covariance of `x`, present only when
    /// [`crate::Params::do_compute_covariance`].
    pub cov_x: Option<Mat3>,
    /// d(x)/d(y1), for covariance propagation. One column per reference ray.
    pub dx_dy1: Option<Matrix>,
    /// d(x)/d(y2), for covariance propagation. One column per sensor ray.
    pub dx_dy2: Option<Matrix>,
    /// Fisher information (Hessian of the point-to-line objective).
    pub fisher: Option<Mat3>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_result_is_invalid_match_without_covariance() {
        // allocated on request.
        let r = MatchResult::default();
        assert!(!r.valid);
        assert_eq!(r.x, [0.0, 0.0, 0.0]);
        assert_eq!(r.iterations, 0);
        assert_eq!(r.nvalid, 0);
        assert!(r.cov_x.is_none() && r.dx_dy1.is_none() && r.dx_dy2.is_none());
    }
}
