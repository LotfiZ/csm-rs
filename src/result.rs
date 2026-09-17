//! Match result.
//!
//! C: `struct sm_result` in `sm/csm/algos.h`
//!
//! A failed match is a *result*, not an error (grilling Q9): check
//! [`SmResult::valid`]. `Result` is reserved for malformed input.

use crate::math::{Mat3, Matrix};

/// Outcome of one scan match.
///
/// C: `struct sm_result`
#[derive(Clone, Debug, Default)]
pub struct SmResult {
    /// C: `valid` — false when the match failed (too few correspondences,
    /// NaN first guess, …). Not an error condition.
    pub valid: bool,
    /// Scan-matching result `(x, y, theta)`. C: `x[3]`
    pub x: [f64; 3],
    /// Iterations performed. C: `iterations`
    pub iterations: i32,
    /// Valid correspondences in the final iteration. C: `nvalid`
    pub nvalid: i32,
    /// Total correspondence error. C: `error`
    pub error: f64,
    /// Closed-form covariance of `x`, present only when
    /// [`crate::Params::do_compute_covariance`]. C: `cov_x_m`
    pub cov_x: Option<Mat3>,
    /// d(x)/d(y1), for covariance propagation. One column per reference ray.
    /// C: `dx_dy1_m`
    pub dx_dy1: Option<Matrix>,
    /// d(x)/d(y2), for covariance propagation. One column per sensor ray.
    /// C: `dx_dy2_m`
    pub dx_dy2: Option<Matrix>,
    /// Fisher information (Hessian of the point-to-line objective).
    pub fisher: Option<Mat3>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_result_is_invalid_match_without_covariance() {
        // C: sm_result starts with valid = 0; covariance matrices are only
        // allocated on request.
        let r = SmResult::default();
        assert!(!r.valid);
        assert_eq!(r.x, [0.0, 0.0, 0.0]);
        assert_eq!(r.iterations, 0);
        assert_eq!(r.nvalid, 0);
        assert!(r.cov_x.is_none() && r.dx_dy1.is_none() && r.dx_dy2.is_none());
    }
}
