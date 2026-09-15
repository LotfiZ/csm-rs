//! Closed-form estimate of ICP's matching covariance.
//!
//! C: `sm/csm/icp/icp_covariance.c` (`compute_covariance_exact`),
//!     `sm/csm/laser_data_fisher.c` (Fisher information helpers)
//!
//! Method: Censi, "An accurate closed-form estimate of ICP's covariance"
//! (ICRA 2007). Produces cov(x), dx_dy1, dx_dy2 as 3×3 matrices.

// TODO(port): compute_covariance_exact, ld_fisher0.
