//! The ICP loop: iteration, convergence, oscillation detection, restart.
//!
//! C: `sm/csm/icp/icp.c` (`sm_icp`, `sm_icp_xy`),
//!     `sm/csm/icp/icp_loop.c` (`icp_loop`)
//!
//! Pipeline per iteration: world-coord transform → correspondence search →
//! outlier rejection (doubles, trim) → closed-form next estimate →
//! convergence test (epsilon_xy, epsilon_theta) → oscillation detection via
//! correspondence hash. Outer layer: validity gating, alpha test setup,
//! visibility test, 6-perturbation restart, optional covariance.

use crate::{Params, SmResult};

/// C: `sm/csm/icp/icp.c:sm_icp()`
pub(crate) fn sm_icp(_params: &Params, result: &mut SmResult) {
    // TODO(port)
    result.valid = false;
}

// TODO(port): icp_loop, ld_invalid_if_outside, restart perturbations.
