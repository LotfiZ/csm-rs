//! Closed-form weighted point-correspondence solver.
//!
//! C: `sm/lib/gpc/gpc.c` (`gpc_solve`, `gpc_total_error`),
//!     `sm/lib/gpc/gpc_utils.c`,
//!     `sm/csm/icp/icp_loop.c` (`compute_next_estimate`)
//!
//! Solves the general point-correspondence problem: find translation `t` and
//! rotation `θ` minimizing Σ (R(θ)p + t − q)′ C (R(θ)p + t − q) over the
//! valid correspondences. Closed form via 4×4 normal equations with 2×2
//! block inverses — hand-rolled small matrices per grilling Q3.
//!
//! Note: upstream `gpc.c` is GPLv2+ (the rest of CSM is LGPLv3); this module
//! is the GPL contaminant and the candidate for a future clean-room rewrite
//! if copyleft ever blocks a use case (grilling Q13).

// TODO(port): gpc_solve, gpc_total_error, compute_next_estimate.
