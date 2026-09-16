//! The ICP loop: iteration and convergence.
//!
//! C: `sm/csm/icp/icp.c` (`sm_icp`),
//!     `sm/csm/icp/icp_loop.c` (`icp_loop`, `termination_criterion`)
//!
//! The loop transforms the sensor scan, finds correspondences with the
//! selected strategy, rejects outliers, solves the closed-form PlICP update,
//! and stops when the pose correction is below both configured thresholds.
//! Correspondence-hash oscillation detection and the six-perturbation restart
//! shell follow the C control flow. Optional Censi covariance is evaluated
//! after the final correspondence set has been selected.

use crate::correspondence::{
    find_correspondences, kill_outliers_double_with_scratch, kill_outliers_trim_with_scratch,
};
use crate::covariance::compute_covariance_exact;
use crate::laser_data::{LaserData, LaserDataError};
use crate::math::{corr_hash_iter, ominus, pose_diff};
use crate::params::DistanceMetric;
use crate::params::Params;
use crate::result::SmResult;
use crate::solver::{compute_next_estimate_with_scratch, GpcCorrespondence};

/// Run point-to-line ICP through the CSM shell.
///
/// C: `sm/csm/icp/icp.c:sm_icp()`
pub(crate) fn sm_icp(
    params: &Params,
    laser_ref: &mut LaserData,
    laser_sens: &mut LaserData,
    result: &mut SmResult,
) -> Result<(), LaserDataError> {
    let mut scratch = IcpScratch::new(
        laser_ref.nrays,
        laser_sens.nrays,
        params.stopping.max_iterations.max(0) as usize,
    );
    sm_icp_with_scratch(params, laser_ref, laser_sens, result, &mut scratch)
}

pub(crate) fn sm_icp_with_scratch(
    params: &Params,
    laser_ref: &mut LaserData,
    laser_sens: &mut LaserData,
    result: &mut SmResult,
    scratch: &mut IcpScratch,
) -> Result<(), LaserDataError> {
    *result = SmResult::default();

    // C: `ld_valid_fields()` is checked before any input mutation.
    laser_ref.validate()?;
    laser_sens.validate()?;

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

    // C computes alpha before visibility invalidates rays. Keep that order so
    // the derived fields on surviving rays have the same values as CSM.
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

    if params.correspondence.do_visibility_test {
        // C: `visibilityTest(laser_ref, x_old)` and
        // `visibilityTest(laser_sens, ominus(x_old))` in `icp.c`.
        laser_ref.visibility_test(&params.first_guess);
        let sensor_viewpoint = ominus(params.first_guess);
        laser_sens.visibility_test(&sensor_viewpoint);
    }

    let outcome = icp_loop_with_restart(params, laser_ref, laser_sens, scratch);
    result.valid = outcome.success;
    result.x = outcome.x;
    result.error = outcome.error;
    result.iterations = outcome.iterations;
    result.nvalid = if outcome.success { outcome.nvalid } else { 0 };

    if outcome.success && params.do_compute_covariance {
        if let Some(covariance) = compute_covariance_exact(laser_ref, laser_sens, outcome.x) {
            result.cov_x = Some(
                covariance
                    .cov0_x
                    .scale(params.correspondence.sigma * params.correspondence.sigma),
            );
            result.dx_dy1 = Some(covariance.dx_dy1);
            result.dx_dy2 = Some(covariance.dx_dy2);
        }
    }

    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct IcpOutcome {
    success: bool,
    x: [f64; 3],
    error: f64,
    iterations: i32,
    nvalid: i32,
}

/// Reusable buffers for the correspondence and outlier stages of ICP.
#[derive(Debug)]
pub(crate) struct IcpScratch {
    hashes: Vec<u32>,
    nearest_distances: Vec<f64>,
    distances_by_sensor: Vec<f64>,
    distances: Vec<f64>,
    correspondences: Vec<GpcCorrespondence>,
}

impl IcpScratch {
    pub(crate) fn new(reference_rays: usize, sensor_rays: usize, max_iterations: usize) -> Self {
        Self {
            hashes: Vec::with_capacity(max_iterations),
            nearest_distances: vec![0.0; reference_rays],
            distances_by_sensor: vec![0.0; sensor_rays],
            distances: Vec::with_capacity(sensor_rays),
            correspondences: Vec::with_capacity(sensor_rays),
        }
    }
}

/// Run ICP once, then try CSM's six local perturbations when the mean error is
/// above the configured restart threshold.
///
/// C: `sm/csm/icp/icp.c:sm_icp()`
fn icp_loop_with_restart(
    params: &Params,
    laser_ref: &LaserData,
    laser_sens: &mut LaserData,
    scratch: &mut IcpScratch,
) -> IcpOutcome {
    let initial = icp_loop(params, params.first_guess, laser_ref, laser_sens, scratch);
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
            let candidate = icp_loop(params, start, laser_ref, laser_sens, scratch);
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
    scratch: &mut IcpScratch,
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
    scratch.hashes.clear();
    scratch.distances.clear();
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
            kill_outliers_double_with_scratch(
                laser_ref,
                laser_sens,
                &mut scratch.nearest_distances,
            );
        }
        let trimmed = kill_outliers_trim_with_scratch(
            &params.outliers,
            laser_ref,
            laser_sens,
            &mut scratch.distances_by_sensor,
            &mut scratch.distances,
        );
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

        let Some(next) = compute_next_estimate_with_scratch(
            params,
            laser_ref,
            laser_sens,
            x_old,
            &mut scratch.correspondences,
        ) else {
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
            .map(|corr| corr.valid.then_some((corr.j1, corr.j2)));
        let hash = corr_hash_iter(correspondence_keys);
        let oscillating = params.correspondence.metric == DistanceMetric::PointToLine
            && scratch.hashes.contains(&hash);
        scratch.hashes.push(hash);
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
