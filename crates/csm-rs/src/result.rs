//! Match result.
//!
//! C: `struct sm_result` in `sm/csm/algos.h`
//!
//! A failed match is a *result*, not an error (grilling Q9): check
//! [`SmResult::valid`]. `Result` is reserved for malformed input.

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
    // TODO(port): cov_x_m, dx_dy1_m, dx_dy2_m (3×3 matrices, present when
    // Params::do_compute_covariance) — C: gsl_matrix fields.
}
