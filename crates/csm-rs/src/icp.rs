//! The ICP loop: iteration and convergence.
//!
//! C: `sm/csm/icp/icp.c` (`sm_icp`),
//!     `sm/csm/icp/icp_loop.c` (`icp_loop`, `termination_criterion`)
//!
//! The loop transforms the sensor scan, finds correspondences with the
//! selected strategy, rejects outliers, solves the closed-form PlICP update,
//! and stops when the pose correction is below both configured thresholds.
//! Correspondence-hash oscillation detection and the six-perturbation restart
//! shell follow the C control flow; covariance is added by a later ticket.

use crate::correspondence::{find_correspondences, kill_outliers_double, kill_outliers_trim};
use crate::laser_data::LaserData;
use crate::math::{corr_hash, pose_diff};
use crate::params::DistanceMetric;
use crate::params::Params;
use crate::result::SmResult;
use crate::solver::compute_next_estimate;

/// Run point-to-line ICP through the CSM shell.
///
/// C: `sm/csm/icp/icp.c:sm_icp()`
pub(crate) fn sm_icp(
    params: &Params,
    laser_ref: &mut LaserData,
    laser_sens: &mut LaserData,
    result: &mut SmResult,
) {
    *result = SmResult::default();

    // C: `ld_valid_fields()` is checked before any input mutation.
    if laser_ref.validate().is_err() || laser_sens.validate().is_err() {
        return;
    }

    // C: `ld_invalid_if_outside()` in `icp.c`.
    laser_ref.invalid_if_outside(params.reading_bounds.min, params.reading_bounds.max);
    laser_sens.invalid_if_outside(params.reading_bounds.min, params.reading_bounds.max);

    // C: `ld_create_jump_tables()` in `icp.c`. The table is needed by the
    // tricks search and is also built for the debug equivalence check.
    if matches!(
        params.correspondence.search,
        crate::params::CorrespondenceSearch::Tricks
    ) || params.debug_verify_tricks
    {
        laser_ref.create_jump_tables();
    }

    laser_ref.compute_cartesian();
    laser_sens.compute_cartesian();

    // These fields are already part of the scan model. Computing them here
    // keeps the alpha-gated correspondence path usable while its other
    // optional features remain outside this tracer ticket.
    if params.correspondence.do_alpha_test {
        laser_ref.simple_clustering(params.correspondence.clustering_threshold);
        laser_ref.compute_orientation(
            params.correspondence.orientation_neighbourhood,
            params.correspondence.sigma,
        );
        laser_sens.simple_clustering(params.correspondence.clustering_threshold);
        laser_sens.compute_orientation(
            params.correspondence.orientation_neighbourhood,
            params.correspondence.sigma,
        );
    }

    let outcome = icp_loop_with_restart(params, laser_ref, laser_sens);
    result.valid = outcome.success;
    result.x = outcome.x;
    result.error = outcome.error;
    result.iterations = outcome.iterations;
    result.nvalid = if outcome.success { outcome.nvalid } else { 0 };
}

#[derive(Clone, Copy, Debug)]
struct IcpOutcome {
    success: bool,
    x: [f64; 3],
    error: f64,
    iterations: i32,
    nvalid: i32,
}

/// Run ICP once, then try CSM's six local perturbations when the mean error is
/// above the configured restart threshold.
///
/// C: `sm/csm/icp/icp.c:sm_icp()`
fn icp_loop_with_restart(
    params: &Params,
    laser_ref: &LaserData,
    laser_sens: &mut LaserData,
) -> IcpOutcome {
    let initial = icp_loop(params, params.first_guess, laser_ref, laser_sens);
    if !initial.success {
        return initial;
    }

    let mut best = initial;
    let mut iterations = initial.iterations;
    let should_restart = params.restart.enabled
        && (initial.error / initial.nvalid as f64) > params.restart.threshold_mean_error;

    if should_restart {
        for perturbation in restart_perturbations(params) {
            let start = [
                initial.x[0] + perturbation[0],
                initial.x[1] + perturbation[1],
                initial.x[2] + perturbation[2],
            ];
            let candidate = icp_loop(params, start, laser_ref, laser_sens);
            if !candidate.success {
                // C stops trying perturbations after the first failed restart,
                // but still returns the best successful result so far.
                break;
            }
            iterations += candidate.iterations;
            if candidate.error < best.error {
                best.x = candidate.x;
                best.error = candidate.error;
            }
        }

        // C recomputes the public correspondence state after any restart,
        // using the pose selected by the total-error comparison.
        laser_sens.compute_world_coords(&best.x);
        find_correspondences(params, laser_ref, laser_sens);
    }

    best.iterations = iterations;
    // C keeps the correspondence count from the initial loop even if a
    // restart has a different surviving set.
    best.nvalid = initial.nvalid;
    best
}

/// The six restart offsets used by CSM: ±x, ±y, and ±theta.
///
/// C: `perturb[6][3]` in `sm/csm/icp/icp.c`
fn restart_perturbations(params: &Params) -> [[f64; 3]; 6] {
    let dt = params.restart.dt;
    let dtheta = params.restart.dtheta;
    [
        [dt, 0.0, 0.0],
        [-dt, 0.0, 0.0],
        [0.0, dt, 0.0],
        [0.0, -dt, 0.0],
        [0.0, 0.0, dtheta],
        [0.0, 0.0, -dtheta],
    ]
}

/// Perform one convergence/oscillation-aware ICP loop.
///
/// C: `sm/csm/icp/icp_loop.c:icp_loop()`
fn icp_loop(
    params: &Params,
    initial_guess: [f64; 3],
    laser_ref: &LaserData,
    laser_sens: &mut LaserData,
) -> IcpOutcome {
    let mut x_old = initial_guess;
    let mut x_new = x_old;
    let mut last_error = 0.0;
    let mut last_nvalid = 0;
    let mut best_x = x_new;
    let mut best_error = f64::INFINITY;
    let mut best_nvalid = 0;

    if x_old.iter().any(|value| value.is_nan()) {
        return IcpOutcome {
            success: false,
            x: x_new,
            error: 0.0,
            iterations: 0,
            nvalid: 0,
        };
    }

    let max_iterations = params.stopping.max_iterations.max(0) as usize;
    let mut hashes = Vec::with_capacity(max_iterations);
    for iteration in 0..max_iterations {
        // C: `ld_compute_world_coords(laser_sens, x_old)`.
        laser_sens.compute_world_coords(&x_old);
        find_correspondences(params, laser_ref, laser_sens);

        let nvalid_before = laser_sens.corr.iter().filter(|corr| corr.valid).count();
        if (nvalid_before as f64) < laser_sens.nrays as f64 * 0.05 {
            return IcpOutcome {
                success: false,
                x: x_new,
                error: 0.0,
                iterations: iteration as i32 + 1,
                nvalid: 0,
            };
        }

        // C: `kill_outliers_double()` followed by `kill_outliers_trim()` in
        // `sm/csm/icp/icp_loop.c`.
        if params.outliers.remove_doubles {
            kill_outliers_double(laser_ref, laser_sens);
        }
        let trimmed = kill_outliers_trim(&params.outliers, laser_ref, laser_sens);
        let nvalid = trimmed.nvalid;
        if (nvalid as f64) < laser_sens.nrays as f64 * 0.05 {
            return IcpOutcome {
                success: false,
                x: x_new,
                error: 0.0,
                iterations: iteration as i32 + 1,
                nvalid: 0,
            };
        }

        let Some(next) = compute_next_estimate(params, laser_ref, laser_sens, x_old) else {
            return IcpOutcome {
                success: false,
                x: x_new,
                error: 0.0,
                iterations: iteration as i32 + 1,
                nvalid: 0,
            };
        };
        x_new = next;

        let error = trimmed.total_error;
        last_error = error;
        last_nvalid = nvalid as i32;
        if error < best_error {
            best_x = x_new;
            best_error = error;
            best_nvalid = nvalid as i32;
        }
        let delta = pose_diff(x_new, x_old);

        // C records the post-trim correspondence set and stops PlICP as soon
        // as that set has appeared in an earlier iteration. Return the best
        // pose/error seen before the cycle, as required by the public ICP
        // contract. With C's unweighted solver, the repeated hash normally
        // produces the same pose as the preceding occurrence.
        let correspondence_keys = laser_sens
            .corr
            .iter()
            .map(|corr| corr.valid.then_some((corr.j1, corr.j2)))
            .collect::<Vec<_>>();
        let hash = corr_hash(&correspondence_keys);
        let oscillating =
            params.correspondence.metric == DistanceMetric::PointToLine && hashes.contains(&hash);
        hashes.push(hash);
        if oscillating {
            return IcpOutcome {
                success: true,
                x: best_x,
                error: best_error,
                iterations: iteration as i32 + 1,
                nvalid: best_nvalid,
            };
        }

        if termination_criterion(params, delta) {
            return IcpOutcome {
                success: true,
                x: x_new,
                error,
                iterations: iteration as i32 + 1,
                nvalid: nvalid as i32,
            };
        }
        x_old = x_new;
    }

    // C stores `iteration + 1` after the for-loop, so exhausting N iterations
    // reports N+1. Preserve that observable result for the conformance path.
    IcpOutcome {
        success: max_iterations > 0,
        x: x_new,
        error: last_error,
        iterations: max_iterations as i32 + 1,
        nvalid: last_nvalid,
    }
}

/// Check the two configured pose-delta thresholds.
///
/// C: `termination_criterion()` in `sm/csm/icp/icp_loop.c`
fn termination_criterion(params: &Params, delta: [f64; 3]) -> bool {
    let norm = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
    norm < params.stopping.epsilon_xy && delta[2].abs() < params.stopping.epsilon_theta
}
