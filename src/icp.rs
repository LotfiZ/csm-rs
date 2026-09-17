//! The ICP loop: iteration and convergence.
//!
//! The loop transforms the sensor scan, finds correspondences with the
//! selected strategy, rejects outliers, solves the closed-form PlICP update,
//! and stops when the pose correction is below both configured thresholds.
//! Correspondence-hash oscillation detection and the six-perturbation restart
//! shell run before the optional Censi covariance is evaluated on the final
//! correspondence set.

use crate::correspondence::{
    find_correspondences, kill_outliers_double_with_scratch, kill_outliers_trim_with_scratch,
};
use crate::covariance::compute_covariance_exact_into;
use crate::matching::TerminationReason;
use crate::math::{corr_hash_iter, ominus, pose_diff, Mat3, Matrix};
use crate::params::DistanceMetric;
use crate::params::Params;
use crate::result::MatchResult;
use crate::scan_data::{OrientationScratch, ScanData, ScanError};
use crate::solver::{compute_next_estimate_with_scratch, PointCorrespondence};

/// Run point-to-line ICP.
///
pub(crate) fn run_icp(
    params: &Params,
    guess: [f64; 3],
    laser_ref: &mut ScanData,
    laser_sens: &mut ScanData,
    result: &mut MatchResult,
) -> Result<TerminationReason, ScanError> {
    let mut scratch = IcpScratch::new(
        laser_ref.nrays,
        laser_sens.nrays,
        params.stopping.max_iterations.max(0) as usize,
        params.correspondence.orientation_neighbourhood,
    );
    let termination =
        run_icp_with_scratch(params, guess, laser_ref, laser_sens, result, &mut scratch)?;
    // The convenience path owns its result, so copy the reusable derivative
    // buffers out before the scratch is dropped.
    if scratch.covariance.is_some() {
        result.dx_dy1 = Some(scratch.cov_dx_dy1.clone());
        result.dx_dy2 = Some(scratch.cov_dx_dy2.clone());
        result.fisher = scratch.fisher;
    }
    Ok(termination)
}

pub(crate) fn run_icp_with_scratch(
    params: &Params,
    guess: [f64; 3],
    laser_ref: &mut ScanData,
    laser_sens: &mut ScanData,
    result: &mut MatchResult,
    scratch: &mut IcpScratch,
) -> Result<TerminationReason, ScanError> {
    *result = MatchResult::default();

    laser_ref.validate()?;
    laser_sens.validate()?;

    laser_ref.invalid_if_outside(params.reading_bounds.min, params.reading_bounds.max);
    laser_sens.invalid_if_outside(params.reading_bounds.min, params.reading_bounds.max);

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

    // Compute alpha before visibility invalidates rays. Keep that order so
    // the derived fields on surviving rays stay consistent.
    if params.correspondence.do_alpha_test {
        laser_ref.simple_clustering(params.correspondence.clustering_threshold);
        laser_ref.compute_orientation_with_scratch(
            params.correspondence.orientation_neighbourhood,
            params.correspondence.sigma,
            &mut scratch.orientation,
        );
        laser_sens.simple_clustering(params.correspondence.clustering_threshold);
        laser_sens.compute_orientation_with_scratch(
            params.correspondence.orientation_neighbourhood,
            params.correspondence.sigma,
            &mut scratch.orientation,
        );
    }

    if params.correspondence.do_visibility_test {
        // `visibilityTest(laser_sens, ominus(x_old))` in `icp.c`.
        laser_ref.visibility_test_with_scratch(&guess, &mut scratch.visibility_thetas);
        let sensor_viewpoint = ominus(guess);
        laser_sens.visibility_test_with_scratch(&sensor_viewpoint, &mut scratch.visibility_thetas);
    }

    let outcome = icp_loop_with_restart(params, guess, laser_ref, laser_sens, scratch);
    result.valid = outcome.success;
    result.x = outcome.x;
    result.error = outcome.error;
    result.iterations = outcome.iterations;
    // Preserve the correspondence count even after an unsuccessful
    // termination; callers use it as a diagnostic.
    result.nvalid = outcome.nvalid;

    if outcome.success && params.do_compute_covariance {
        if let Some(covariance) = compute_covariance_exact_into(
            laser_ref,
            laser_sens,
            outcome.x,
            &mut scratch.cov_dx_dy1,
            &mut scratch.cov_dx_dy2,
        ) {
            let scaled = covariance
                .cov0_x
                .scale(params.correspondence.sigma * params.correspondence.sigma);
            result.cov_x = Some(scaled);
            scratch.covariance = Some(scaled);
            scratch.fisher = Some(covariance.fisher);
        } else {
            scratch.covariance = None;
            scratch.fisher = None;
        }
    } else {
        scratch.covariance = None;
        scratch.fisher = None;
    }

    Ok(outcome.termination)
}

#[derive(Clone, Copy, Debug)]
struct IcpOutcome {
    success: bool,
    x: [f64; 3],
    error: f64,
    iterations: i32,
    nvalid: i32,
    termination: TerminationReason,
}

/// Reusable buffers for the correspondence and outlier stages of ICP.
pub(crate) struct IcpScratch {
    hashes: Vec<u32>,
    nearest_distances: Vec<f64>,
    distances_by_sensor: Vec<f64>,
    distances: Vec<f64>,
    correspondences: Vec<PointCorrespondence>,
    visibility_thetas: Vec<f64>,
    orientation: OrientationScratch,
    /// Reusable derivative-matrix storage for the optional covariance path.
    pub(crate) cov_dx_dy1: Matrix,
    pub(crate) cov_dx_dy2: Matrix,
    /// Covariance and Fisher information produced by the last match.
    pub(crate) covariance: Option<Mat3>,
    pub(crate) fisher: Option<Mat3>,
    pub(crate) trace_events: Vec<TraceEvent>,
    pub(crate) trace_enabled: bool,
}

/// One instrumented ICP iteration with its contributing correspondences.
pub(crate) struct TraceEvent {
    pub iteration: usize,
    pub pose: [f64; 3],
    pub error: f64,
    pub nvalid: usize,
    pub restart: bool,
    pub correspondences: Vec<TraceCorrespondence>,
}

/// A single correspondence in an instrumented iteration.
pub(crate) struct TraceCorrespondence {
    pub sensor: usize,
    pub reference_j1: i32,
    pub reference_j2: i32,
    pub distance: f64,
    /// Sensor point in the reference frame at correspondence time.
    pub sensor_point: [f64; 2],
    /// Matching reference point in the reference frame.
    pub reference_point: [f64; 2],
}

impl IcpScratch {
    pub(crate) fn new(
        reference_rays: usize,
        sensor_rays: usize,
        max_iterations: usize,
        orientation_neighbourhood: i32,
    ) -> Self {
        Self {
            hashes: Vec::with_capacity(max_iterations),
            nearest_distances: vec![0.0; reference_rays],
            distances_by_sensor: vec![0.0; sensor_rays],
            distances: Vec::with_capacity(sensor_rays),
            correspondences: Vec::with_capacity(sensor_rays),
            visibility_thetas: vec![f64::NAN; reference_rays.max(sensor_rays)],
            orientation: OrientationScratch::new(orientation_neighbourhood),
            cov_dx_dy1: Matrix::zeros(3, reference_rays),
            cov_dx_dy2: Matrix::zeros(3, sensor_rays),
            covariance: None,
            fisher: None,
            trace_events: Vec::new(),
            trace_enabled: false,
        }
    }

    pub(crate) fn memory_bytes(&self) -> usize {
        self.hashes.capacity() * std::mem::size_of::<u32>()
            + self.nearest_distances.capacity() * std::mem::size_of::<f64>()
            + self.distances_by_sensor.capacity() * std::mem::size_of::<f64>()
            + self.distances.capacity() * std::mem::size_of::<f64>()
            + self.correspondences.capacity() * std::mem::size_of::<PointCorrespondence>()
    }
}

/// Run ICP once, then try six local perturbations when the mean error is
/// above the configured restart threshold.
///
fn icp_loop_with_restart(
    params: &Params,
    guess: [f64; 3],
    laser_ref: &ScanData,
    laser_sens: &mut ScanData,
    scratch: &mut IcpScratch,
) -> IcpOutcome {
    let initial = icp_loop(params, guess, laser_ref, laser_sens, scratch, false);
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
            let candidate = icp_loop(params, start, laser_ref, laser_sens, scratch, true);
            if !candidate.success {
                // Stop trying perturbations after the first failed restart,
                // but still returns the best successful result so far.
                break;
            }
            iterations += candidate.iterations;
            if candidate.error < best.error {
                best.x = candidate.x;
                best.error = candidate.error;
            }
        }

        // Recompute the public correspondence state after any restart,
        // using the pose selected by the total-error comparison.
        laser_sens.compute_world_coords(&best.x);
        find_correspondences(params, laser_ref, laser_sens);
    }

    best.iterations = iterations;
    // Keep the correspondence count from the initial loop even if a
    // restart has a different surviving set.
    best.nvalid = initial.nvalid;
    best
}

/// The six restart offsets: ±x, ±y, and ±theta.
///
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
fn icp_loop(
    params: &Params,
    initial_guess: [f64; 3],
    laser_ref: &ScanData,
    laser_sens: &mut ScanData,
    scratch: &mut IcpScratch,
    restart: bool,
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
            termination: TerminationReason::NumericalFailure,
        };
    }

    let max_iterations = params.stopping.max_iterations.max(0) as usize;
    scratch.hashes.clear();
    scratch.distances.clear();
    for iteration in 0..max_iterations {
        laser_sens.compute_world_coords(&x_old);
        find_correspondences(params, laser_ref, laser_sens);

        let nvalid_before = laser_sens.corr.iter().filter(|corr| corr.valid).count();
        if (nvalid_before as f64) < laser_sens.nrays as f64 * 0.05 {
            return IcpOutcome {
                success: false,
                x: x_new,
                error: 0.0,
                iterations: iteration as i32 + 1,
                nvalid: nvalid_before as i32,
                termination: if nvalid_before == 0 {
                    TerminationReason::NoCorrespondences
                } else {
                    TerminationReason::InsufficientGeometry
                },
            };
        }

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
                error: trimmed.total_error,
                iterations: iteration as i32 + 1,
                nvalid: nvalid as i32,
                termination: if nvalid == 0 {
                    TerminationReason::NoCorrespondences
                } else {
                    TerminationReason::InsufficientGeometry
                },
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
                error: trimmed.total_error,
                iterations: iteration as i32 + 1,
                nvalid: nvalid as i32,
                termination: TerminationReason::NumericalFailure,
            };
        };
        x_new = next;

        let error = trimmed.total_error;
        if scratch.trace_enabled {
            let correspondences = laser_sens
                .corr
                .iter()
                .enumerate()
                .filter(|(_, corr)| corr.valid)
                .map(|(sensor, corr)| TraceCorrespondence {
                    sensor,
                    reference_j1: corr.j1,
                    reference_j2: corr.j2,
                    distance: corr.dist2_j1.max(0.0).sqrt(),
                    sensor_point: laser_sens.points_w[sensor].p,
                    reference_point: usize::try_from(corr.j1)
                        .ok()
                        .filter(|&j1| j1 < laser_ref.nrays)
                        .map(|j1| laser_ref.points[j1].p)
                        .unwrap_or([f64::NAN, f64::NAN]),
                })
                .collect();
            scratch.trace_events.push(TraceEvent {
                iteration,
                pose: x_new,
                error,
                nvalid,
                restart,
                correspondences,
            });
        }
        last_error = error;
        last_nvalid = nvalid as i32;
        if error < best_error {
            best_x = x_new;
            best_error = error;
            best_nvalid = nvalid as i32;
        }
        let delta = pose_diff(x_new, x_old);

        // Record the post-trim correspondence set and stop PlICP as soon
        // as that set has appeared in an earlier iteration. Return the best
        // pose/error seen before the cycle, as required by the public ICP
        // contract. With the unweighted solver, the repeated hash normally
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
                termination: TerminationReason::CycleDetected,
            };
        }

        if termination_criterion(params, delta) {
            return IcpOutcome {
                success: true,
                x: x_new,
                error,
                iterations: iteration as i32 + 1,
                nvalid: nvalid as i32,
                termination: TerminationReason::Converged,
            };
        }
        x_old = x_new;
    }

    // Store `iteration + 1` after the loop, so exhausting N iterations
    // reports N+1. Preserve that observable result for the conformance path.
    IcpOutcome {
        success: max_iterations > 0,
        x: x_new,
        error: last_error,
        iterations: max_iterations as i32 + 1,
        nvalid: last_nvalid,
        termination: TerminationReason::IterationLimit,
    }
}

/// Check the two configured pose-delta thresholds.
///
fn termination_criterion(params: &Params, delta: [f64; 3]) -> bool {
    let norm = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
    norm < params.stopping.epsilon_xy && delta[2].abs() < params.stopping.epsilon_theta
}

#[cfg(test)]
mod termination_tests {
    use super::*;
    use crate::params::Params;
    use crate::result::MatchResult;

    fn polar_scan(n: usize, readings: impl Fn(usize) -> f64) -> ScanData {
        let theta: Vec<f64> = (0..n)
            .map(|i| -0.2 + 0.4 * i as f64 / (n - 1) as f64)
            .collect();
        let readings: Vec<f64> = (0..n).map(&readings).collect();
        ScanData::from_polar(theta, readings, vec![true; n]).unwrap()
    }

    #[test]
    fn insufficient_geometry_is_an_outcome_not_an_error() {
        // Four of a hundred rays land within the correspondence distance.
        let reference = polar_scan(100, |_| 5.0);
        let mut reference = reference;
        let mut sensor = polar_scan(100, |i| if i < 4 { 5.0 } else { 50.0 });
        let mut params = Params::default();
        params.correspondence.max_dist = 1.0;
        let mut result = MatchResult::default();
        let termination =
            run_icp(&params, [0.0; 3], &mut reference, &mut sensor, &mut result).unwrap();
        assert_eq!(termination, TerminationReason::InsufficientGeometry);
        assert!(!result.valid);
        assert!(result.nvalid > 0, "candidate diagnostics are preserved");
    }
}
