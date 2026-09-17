//! Matching configuration, workspaces, and results.

use crate::{
    icp,
    math::{Mat3, Matrix},
    params::{Params, ParamsError},
    pose::Pose,
    result::MatchResult,
    scan::{CartesianScan, PolarScan},
    scan_data::{ScanData, ScanError},
};

/// Owned, reusable polar scan storage for steady-state matching.
#[derive(Clone, Debug)]
pub struct PreparedPolarScan {
    data: ScanData,
    /// The caller's validity flags. Matching temporarily invalidates rays
    /// (reading bounds, visibility), so the input is restored before every
    /// match to avoid stale state across frames.
    input_valid: Vec<bool>,
}

impl PreparedPolarScan {
    pub fn from_polar(
        angles: Vec<f64>,
        readings: Vec<f64>,
        valid: Vec<bool>,
    ) -> Result<Self, ScanError> {
        let data = ScanData::from_polar(angles, readings, valid.clone())?;
        Ok(Self {
            data,
            input_valid: valid,
        })
    }

    /// Owned scan with optional per-ray sigma and known surface orientation
    /// for the weighting paths.
    pub fn from_polar_with_inputs(
        angles: Vec<f64>,
        readings: Vec<f64>,
        valid: Vec<bool>,
        sigma: Option<Vec<f64>>,
        true_alpha: Option<Vec<f64>>,
    ) -> Result<Self, ScanError> {
        let mut scan = Self::from_polar(angles, readings, valid)?;
        if let Some(sigma) = sigma {
            if sigma.len() != scan.len() {
                return Err(ScanError::InconsistentLengths {
                    field: "sigma",
                    expected: scan.len(),
                    actual: sigma.len(),
                });
            }
            scan.data.readings_sigma.copy_from_slice(&sigma);
        }
        if let Some(true_alpha) = true_alpha {
            if true_alpha.len() != scan.len() {
                return Err(ScanError::InconsistentLengths {
                    field: "true_alpha",
                    expected: scan.len(),
                    actual: true_alpha.len(),
                });
            }
            scan.data.true_alpha.copy_from_slice(&true_alpha);
        }
        Ok(scan)
    }

    pub fn from_cartesian(points: &[[f64; 2]], valid: &[bool]) -> Result<Self, ScanError> {
        let scan = CartesianScan::new(points, valid)?;
        let (angles, readings) = scan.to_polar_parts();
        Self::from_polar(angles, readings, valid.to_vec())
    }

    pub fn from_cartesian_with_angles(
        points: &[[f64; 2]],
        angles: &[f64],
        valid: &[bool],
    ) -> Result<Self, ScanError> {
        let scan = CartesianScan::with_angles(points, angles, valid)?;
        let (angles, readings) = scan.to_polar_parts();
        Self::from_polar(angles, readings, valid.to_vec())
    }

    /// Reserve storage for `capacity` rays without choosing a scan yet.
    /// Use [`Self::set_polar`] or [`Self::set_cartesian`] to fill it without
    /// further allocation.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            data: ScanData::with_capacity(capacity),
            input_valid: Vec::with_capacity(capacity),
        }
    }

    /// Reserve room for at least `capacity` rays. This is the explicit growth
    /// operation; matching never grows storage on its own.
    pub fn reserve(&mut self, capacity: usize) {
        self.data.reserve_rays(capacity);
        self.input_valid
            .reserve(capacity.saturating_sub(self.input_valid.len()));
    }

    /// Restore the caller's input validity and clear derived state before a
    /// match. Allocation-free and idempotent.
    pub(crate) fn prepare_for_match(&mut self) {
        if self.data.valid.len() == self.input_valid.len() {
            self.data.valid.copy_from_slice(&self.input_valid);
        }
        self.data.reset_derived();
    }

    /// Replace the scan contents, growing only up to the reserved capacity.
    ///
    /// Angles, readings, and validity must have equal length. The scan is
    /// revalidated and all derived fields are reset, so no stale state from a
    /// previous frame survives.
    pub fn set_polar(
        &mut self,
        angles: &[f64],
        readings: &[f64],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        if readings.len() != angles.len() {
            return Err(ScanError::InconsistentLengths {
                field: "readings",
                expected: angles.len(),
                actual: readings.len(),
            });
        }
        if valid.len() != angles.len() {
            return Err(ScanError::InconsistentLengths {
                field: "valid",
                expected: angles.len(),
                actual: valid.len(),
            });
        }
        if angles.len() > self.data.capacity() {
            return Err(ScanError::CapacityExceeded {
                capacity: self.data.capacity(),
                requested: angles.len(),
            });
        }
        self.data.resize_rays(angles.len());
        self.data.theta.copy_from_slice(angles);
        self.data.readings.copy_from_slice(readings);
        self.data.valid.copy_from_slice(valid);
        // A new frame invalidates previously supplied per-ray orientation.
        self.data.true_alpha.fill(f64::NAN);
        self.input_valid.clear();
        self.input_valid.extend_from_slice(valid);
        self.data.min_theta = angles.first().copied().unwrap_or(f64::NAN);
        self.data.max_theta = angles.last().copied().unwrap_or(f64::NAN);
        self.data.validate()
    }

    /// Replace the scan with ordered Cartesian points.
    pub fn set_cartesian(&mut self, points: &[[f64; 2]], valid: &[bool]) -> Result<(), ScanError> {
        let scan = CartesianScan::new(points, valid)?;
        let (angles, readings) = scan.to_polar_parts();
        self.set_polar(&angles, &readings, valid)
    }

    /// Replace the scan with ordered Cartesian points and explicit bearings.
    pub fn set_cartesian_with_angles(
        &mut self,
        points: &[[f64; 2]],
        angles: &[f64],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        let scan = CartesianScan::with_angles(points, angles, valid)?;
        let (angles, readings) = scan.to_polar_parts();
        self.set_polar(&angles, &readings, valid)
    }

    pub fn len(&self) -> usize {
        self.data.nrays
    }

    pub fn is_empty(&self) -> bool {
        self.data.nrays == 0
    }

    pub fn capacity(&self) -> usize {
        self.data.capacity()
    }

    pub fn angles(&self) -> &[f64] {
        &self.data.theta
    }

    pub fn readings(&self) -> &[f64] {
        &self.data.readings
    }

    pub fn valid(&self) -> &[bool] {
        &self.data.valid
    }

    /// Per-ray range standard deviation, or `NaN` when unset.
    pub fn sigma(&self) -> &[f64] {
        &self.data.readings_sigma
    }

    /// Per-ray known surface orientation, or `NaN` when unset.
    pub fn true_alpha(&self) -> &[f64] {
        &self.data.true_alpha
    }

    /// Refresh readings in place while retaining all allocated storage.
    pub fn update(&mut self, readings: &[f64], valid: &[bool]) -> Result<(), ScanError> {
        if readings.len() != self.data.nrays {
            if readings.len() > self.data.nrays {
                return Err(ScanError::CapacityExceeded {
                    capacity: self.data.nrays,
                    requested: readings.len(),
                });
            }
            return Err(ScanError::InconsistentLengths {
                field: "readings",
                expected: self.data.nrays,
                actual: readings.len(),
            });
        }
        if valid.len() != self.data.nrays {
            return Err(ScanError::InconsistentLengths {
                field: "valid",
                expected: self.data.nrays,
                actual: valid.len(),
            });
        }
        self.data.readings.copy_from_slice(readings);
        self.data.valid.copy_from_slice(valid);
        self.input_valid.copy_from_slice(valid);
        self.data.true_alpha.fill(f64::NAN);
        self.data.reset_derived();
        self.data.validate()
    }

    /// Refresh an ordered Cartesian frame in place.
    pub fn update_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        if points.len() != self.data.nrays {
            if points.len() > self.data.nrays {
                return Err(ScanError::CapacityExceeded {
                    capacity: self.data.nrays,
                    requested: points.len(),
                });
            }
            return Err(ScanError::InconsistentLengths {
                field: "points",
                expected: self.data.nrays,
                actual: points.len(),
            });
        }
        let scan = CartesianScan::new(points, valid)?;
        for (reading, point) in self.data.readings.iter_mut().zip(scan.points()) {
            *reading = point[0].hypot(point[1]);
        }
        self.data.valid.copy_from_slice(valid);
        self.input_valid.copy_from_slice(valid);
        self.data.true_alpha.fill(f64::NAN);
        self.data.reset_derived();
        self.data.validate()
    }
}

/// Reusable matching workspace for fixed-shape streams.
///
/// The scan buffers are owned by the workspace and can be updated in place
/// between calls. Constructing it up front makes the intended steady-state
/// ownership explicit for real-time and embedded callers.
pub struct PreparedMatcher {
    matcher: Matcher,
    reference: PreparedPolarScan,
    sensor: PreparedPolarScan,
    scratch: icp::IcpScratch,
}

impl PreparedMatcher {
    pub fn new(
        matcher: Matcher,
        reference: PreparedPolarScan,
        sensor: PreparedPolarScan,
    ) -> Result<Self, ScanError> {
        if reference.capacity() == 0 || sensor.capacity() == 0 {
            return Err(ScanError::NraysOutOfRange);
        }
        let scratch = icp::IcpScratch::new(
            reference.capacity().max(reference.len()),
            sensor.capacity().max(sensor.len()),
            matcher.params.stopping.max_iterations.max(0) as usize,
            matcher.params.correspondence.orientation_neighbourhood,
        );
        Ok(Self {
            matcher,
            reference,
            sensor,
            scratch,
        })
    }

    /// Reserve reference and sensor capacity, then rebuild the workspace. This
    /// is the explicit growth operation: matching itself never grows storage.
    pub fn reserve(&mut self, reference_capacity: usize, sensor_capacity: usize) {
        self.reference.reserve(reference_capacity);
        self.sensor.reserve(sensor_capacity);
        self.scratch = icp::IcpScratch::new(
            self.reference.capacity().max(self.reference.len()),
            self.sensor.capacity().max(self.sensor.len()),
            self.matcher.params.stopping.max_iterations.max(0) as usize,
            self.matcher.params.correspondence.orientation_neighbourhood,
        );
    }

    /// Replace the reference frame, growing only up to reserved capacity.
    pub fn set_reference_polar(
        &mut self,
        angles: &[f64],
        readings: &[f64],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        self.reference.set_polar(angles, readings, valid)
    }

    /// Replace the sensor frame, growing only up to reserved capacity.
    pub fn set_sensor_polar(
        &mut self,
        angles: &[f64],
        readings: &[f64],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        self.sensor.set_polar(angles, readings, valid)
    }

    /// Replace the reference frame with ordered Cartesian points.
    pub fn set_reference_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        self.reference.set_cartesian(points, valid)
    }

    /// Replace the sensor frame with ordered Cartesian points.
    pub fn set_sensor_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        self.sensor.set_cartesian(points, valid)
    }

    pub fn reference(&self) -> &PreparedPolarScan {
        &self.reference
    }

    pub fn reference_mut(&mut self) -> &mut PreparedPolarScan {
        &mut self.reference
    }

    pub fn sensor(&self) -> &PreparedPolarScan {
        &self.sensor
    }

    pub fn sensor_mut(&mut self) -> &mut PreparedPolarScan {
        &mut self.sensor
    }

    /// Return `(reference_capacity, sensor_capacity)` in rays.
    pub fn capacities(&self) -> (usize, usize) {
        (self.reference.capacity(), self.sensor.capacity())
    }

    /// Bytes currently reserved by the reusable ICP workspace.
    pub fn workspace_bytes(&self) -> usize {
        self.scratch.memory_bytes()
    }

    /// Update the streamed sensor frame while preserving its allocation.
    pub fn update_sensor(&mut self, readings: &[f64], valid: &[bool]) -> Result<(), ScanError> {
        self.sensor.update(readings, valid)
    }

    /// Update an ordered Cartesian sensor frame in place.
    pub fn update_sensor_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        self.sensor.update_cartesian(points, valid)
    }

    /// Update an ordered Cartesian reference frame in place.
    pub fn update_reference_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), ScanError> {
        self.reference.update_cartesian(points, valid)
    }

    /// Update the reference frame while preserving its allocation.
    pub fn update_reference(&mut self, readings: &[f64], valid: &[bool]) -> Result<(), ScanError> {
        self.reference.update(readings, valid)
    }

    /// Match the current frames with the identity initial pose.
    pub fn match_once(&mut self) -> Result<MatchOutcome, ScanError> {
        self.match_once_from(Pose::IDENTITY)
    }

    /// Match the current frames with an explicit initial pose.
    pub fn match_once_from(&mut self, guess: Pose) -> Result<MatchOutcome, ScanError> {
        let mut outcome = MatchOutcome::default();
        self.match_once_from_into(guess, &mut outcome)?;
        Ok(outcome)
    }

    /// Match into caller-owned outcome storage, reusing its uncertainty
    /// buffers so repeated matching performs no heap allocation after the
    /// caller has reserved them with [`MatchOutcome::reserve_uncertainty`].
    pub fn match_once_from_into(
        &mut self,
        guess: Pose,
        outcome: &mut MatchOutcome,
    ) -> Result<(), ScanError> {
        self.reference.prepare_for_match();
        self.sensor.prepare_for_match();
        let mut result = MatchResult::default();
        let termination = icp::run_icp_with_scratch(
            &self.matcher.params,
            guess.to_array(),
            &mut self.reference.data,
            &mut self.sensor.data,
            &mut result,
            &mut self.scratch,
        )?;
        let params = &self.matcher.params;
        let covariance = self.scratch.covariance;
        let fisher = self.scratch.fisher;
        let mut filled = MatchOutcome::from(result).with_termination(termination);
        filled.covariance = covariance;
        filled.fisher_information = fisher;
        filled.covariance_status = if !params.do_compute_covariance {
            CovarianceStatus::Disabled
        } else if covariance.is_some() {
            CovarianceStatus::Computed
        } else {
            CovarianceStatus::Failed
        };
        if covariance.is_some() {
            copy_matrix_into(&mut outcome.dx_dy_reference, &self.scratch.cov_dx_dy1);
            copy_matrix_into(&mut outcome.dx_dy_sensor, &self.scratch.cov_dx_dy2);
        } else {
            outcome.dx_dy_reference = None;
            outcome.dx_dy_sensor = None;
        }
        filled.dx_dy_reference = outcome.dx_dy_reference.take();
        filled.dx_dy_sensor = outcome.dx_dy_sensor.take();
        *outcome = filled;
        Ok(())
    }

    /// Match with the identity initial pose into caller-owned storage.
    pub fn match_into(&mut self, outcome: &mut MatchOutcome) -> Result<(), ScanError> {
        self.match_once_from_into(Pose::IDENTITY, outcome)
    }

    /// Match once and report the accepted result as a final snapshot.
    ///
    /// Tracing is opt-in and runs the same matching path; the observer is
    /// called for every real iteration. Traced matches allocate per-iteration
    /// correspondence storage, so instrumented timing must be reported
    /// separately from ordinary matching timing.
    pub fn match_once_traced<F: FnMut(IterationSnapshot)>(
        &mut self,
        mut observe: F,
    ) -> Result<MatchOutcome, ScanError> {
        self.scratch.trace_events.clear();
        self.scratch.trace_enabled = true;
        let outcome = self.match_once();
        self.scratch.trace_enabled = false;
        let outcome = outcome?;
        for event in self.scratch.trace_events.drain(..) {
            observe(IterationSnapshot {
                iteration: event.iteration,
                pose: event.pose,
                error: event.error,
                valid_correspondences: event.nvalid,
                restart: event.restart,
                correspondences: event
                    .correspondences
                    .into_iter()
                    .map(|corr| CorrespondenceSnapshot {
                        sensor_ray: corr.sensor,
                        reference_j1: corr.reference_j1,
                        reference_j2: corr.reference_j2,
                        distance: corr.distance,
                        sensor_point: corr.sensor_point,
                        reference_point: corr.reference_point,
                    })
                    .collect(),
            });
        }
        Ok(outcome)
    }
}

/// Result of a match.
///
/// A well-formed input can produce an unsuccessful result; inspect
/// [`Self::valid`] to distinguish that case from input errors. A candidate
/// pose is always present, but only [`Self::valid`] results are accepted
/// convergence; unsuccessful candidates are diagnostics.
#[derive(Clone, Debug, Default)]
pub struct MatchOutcome {
    pub status: MatchStatus,
    pub valid: bool,
    /// Sensor-to-reference transform of the (candidate or accepted) match.
    pub pose: Pose,
    pub iterations: i32,
    pub nvalid: i32,
    pub error: f64,
    pub covariance: Option<Mat3>,
    pub dx_dy_reference: Option<Matrix>,
    pub dx_dy_sensor: Option<Matrix>,
    /// Hessian of the point-to-line objective (Fisher information).
    pub fisher_information: Option<Mat3>,
    pub covariance_status: CovarianceStatus,
    pub termination: TerminationReason,
}

/// Whether optional uncertainty was requested and produced.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CovarianceStatus {
    /// The matcher was configured pose-only.
    #[default]
    Disabled,
    /// All requested uncertainty outputs were produced.
    Computed,
    /// Uncertainty was requested but could not be computed; the pose remains
    /// usable when present.
    Failed,
}

/// Why matching stopped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TerminationReason {
    /// Matching failed without a more specific reason.
    #[default]
    Failed,
    /// The pose correction fell below the configured thresholds.
    Converged,
    /// No usable correspondences were found.
    NoCorrespondences,
    /// Some correspondences existed but fewer than the usable-geometry
    /// threshold.
    InsufficientGeometry,
    /// The configured iteration limit was reached.
    IterationLimit,
    /// The correspondence set repeated, so the loop stopped on a cycle.
    CycleDetected,
    /// The linear solve failed (singular or non-finite system).
    NumericalFailure,
}

impl MatchOutcome {
    /// Whether covariance and both derivative matrices were produced.
    pub fn has_uncertainty(&self) -> bool {
        self.covariance.is_some()
            && self.dx_dy_reference.is_some()
            && self.dx_dy_sensor.is_some()
            && self.fisher_information.is_some()
    }

    /// Reserve derivative-matrix storage so [`PreparedMatcher::match_into`]
    /// can fill uncertainty outputs without allocating.
    pub fn reserve_uncertainty(&mut self, reference_rays: usize, sensor_rays: usize) {
        match &mut self.dx_dy_reference {
            Some(matrix) => matrix.reset(3, reference_rays),
            None => self.dx_dy_reference = Some(Matrix::zeros(3, reference_rays)),
        }
        match &mut self.dx_dy_sensor {
            Some(matrix) => matrix.reset(3, sensor_rays),
            None => self.dx_dy_sensor = Some(Matrix::zeros(3, sensor_rays)),
        }
    }

    fn with_diagnostics(mut self, params: &Params) -> Self {
        self.covariance_status = if !params.do_compute_covariance {
            CovarianceStatus::Disabled
        } else if self.has_uncertainty() {
            CovarianceStatus::Computed
        } else {
            CovarianceStatus::Failed
        };
        self
    }

    fn with_termination(mut self, termination: TerminationReason) -> Self {
        self.termination = termination;
        self.status = if self.valid && termination == TerminationReason::Converged {
            MatchStatus::Converged
        } else {
            MatchStatus::Failed
        };
        self
    }
}

/// One instrumented ICP iteration.
#[derive(Clone, Debug, PartialEq)]
pub struct IterationSnapshot {
    pub iteration: usize,
    pub pose: [f64; 3],
    pub error: f64,
    pub valid_correspondences: usize,
    /// Whether this iteration came from a restart perturbation.
    pub restart: bool,
    /// Correspondences that contributed to this iteration's solve.
    pub correspondences: Vec<CorrespondenceSnapshot>,
}

/// A correspondence contributing to an instrumented iteration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrespondenceSnapshot {
    pub sensor_ray: usize,
    pub reference_j1: i32,
    pub reference_j2: i32,
    /// Point-to-line distance for this correspondence, in metres.
    pub distance: f64,
    /// Sensor point in the reference frame when the correspondence was found.
    pub sensor_point: [f64; 2],
    /// Matching reference point in the reference frame.
    pub reference_point: [f64; 2],
}

/// Coarse match status.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MatchStatus {
    #[default]
    Failed,
    Converged,
}

impl MatchOutcome {
    pub fn converged(&self) -> bool {
        self.accepted()
    }

    /// Whether the result is an accepted convergence rather than a candidate
    /// from an unsuccessful termination.
    pub fn accepted(&self) -> bool {
        self.valid && self.termination == TerminationReason::Converged
    }

    /// The candidate pose, if the matcher produced one.
    ///
    /// A candidate is always present for a well-formed request, including
    /// unsuccessful terminations; `None` means no candidate was available
    /// (for example a non-finite initial pose reaching the engine).
    pub fn candidate(&self) -> Option<Pose> {
        self.pose.is_finite().then_some(self.pose)
    }
}

impl From<MatchResult> for MatchOutcome {
    fn from(result: MatchResult) -> Self {
        Self {
            status: if result.valid {
                MatchStatus::Converged
            } else {
                MatchStatus::Failed
            },
            valid: result.valid,
            pose: Pose::from_array(result.x),
            iterations: result.iterations,
            nvalid: result.nvalid,
            error: result.error,
            covariance: result.cov_x,
            dx_dy_reference: result.dx_dy1,
            dx_dy_sensor: result.dx_dy2,
            fisher_information: result.fisher,
            covariance_status: CovarianceStatus::Disabled,
            termination: TerminationReason::Failed,
        }
    }
}

/// Validated scan matcher configuration.
///
/// Construct one with [`Matcher::default`] for useful defaults, or with
/// [`Matcher::new`] to validate a custom [`Params`]. All matching methods take
/// the reference scan explicitly; the reference is never replaced implicitly.
#[derive(Clone, Debug, Default)]
pub struct Matcher {
    params: Params,
}

impl Matcher {
    /// Construct a matcher after validating all numeric parameters.
    pub fn new(params: Params) -> Result<Self, ParamsError> {
        params.validate()?;
        Ok(Self { params })
    }

    /// A matcher using the documented defaults.
    ///
    /// The defaults are pose-only: uncertainty must be requested explicitly
    /// through [`Params::do_compute_covariance`].
    pub fn default_pose_only() -> Self {
        Self {
            params: Params::default(),
        }
    }

    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Build a reusable workspace from two owned, fixed-shape scans.
    pub fn prepare(
        &self,
        reference: PreparedPolarScan,
        sensor: PreparedPolarScan,
    ) -> Result<PreparedMatcher, ScanError> {
        PreparedMatcher::new(self.clone(), reference, sensor)
    }

    /// Build a reusable workspace directly from ordered Cartesian frames.
    pub fn prepare_cartesian(
        &self,
        reference_points: &[[f64; 2]],
        reference_valid: &[bool],
        sensor_points: &[[f64; 2]],
        sensor_valid: &[bool],
    ) -> Result<PreparedMatcher, ScanError> {
        let reference = PreparedPolarScan::from_cartesian(reference_points, reference_valid)?;
        let sensor = PreparedPolarScan::from_cartesian(sensor_points, sensor_valid)?;
        self.prepare(reference, sensor)
    }

    /// Build a reusable workspace directly from borrowed polar frames.
    pub fn prepare_polar(
        &self,
        reference: PolarScan<'_>,
        sensor: PolarScan<'_>,
    ) -> Result<PreparedMatcher, ScanError> {
        let reference = PreparedPolarScan::from_polar(
            reference.angles().to_vec(),
            reference.readings().to_vec(),
            reference.valid().to_vec(),
        )?;
        let sensor = PreparedPolarScan::from_polar(
            sensor.angles().to_vec(),
            sensor.readings().to_vec(),
            sensor.valid().to_vec(),
        )?;
        self.prepare(reference, sensor)
    }

    /// Match two borrowed ordered polar scans with the identity initial pose.
    pub fn match_polar(
        &self,
        reference: PolarScan<'_>,
        sensor: PolarScan<'_>,
    ) -> Result<MatchOutcome, ScanError> {
        self.match_polar_from(reference, sensor, Pose::IDENTITY)
    }

    /// Match two borrowed ordered polar scans with an explicit initial pose.
    ///
    /// The returned transform maps sensor-scan coordinates into reference-scan
    /// coordinates.
    pub fn match_polar_from(
        &self,
        reference: PolarScan<'_>,
        sensor: PolarScan<'_>,
        guess: Pose,
    ) -> Result<MatchOutcome, ScanError> {
        let reference_sigma = reference.sigma();
        let reference_alpha = reference.true_alpha();
        let sensor_sigma = sensor.sigma();
        let sensor_alpha = sensor.true_alpha();
        let mut laser_ref = ScanData::from_polar(
            reference.angles().to_vec(),
            reference.readings().to_vec(),
            reference.valid().to_vec(),
        )?;
        apply_weight_inputs(&mut laser_ref, reference_sigma, reference_alpha);
        let mut laser_sens = ScanData::from_polar(
            sensor.angles().to_vec(),
            sensor.readings().to_vec(),
            sensor.valid().to_vec(),
        )?;
        apply_weight_inputs(&mut laser_sens, sensor_sigma, sensor_alpha);
        self.match_laser(&mut laser_ref, &mut laser_sens, guess)
    }

    /// Match ordered Cartesian scans with the identity initial pose.
    pub fn match_cartesian(
        &self,
        reference: CartesianScan<'_>,
        sensor: CartesianScan<'_>,
    ) -> Result<MatchOutcome, ScanError> {
        self.match_cartesian_from(reference, sensor, Pose::IDENTITY)
    }

    /// Match ordered Cartesian scans with an explicit initial pose.
    pub fn match_cartesian_from(
        &self,
        reference: CartesianScan<'_>,
        sensor: CartesianScan<'_>,
        guess: Pose,
    ) -> Result<MatchOutcome, ScanError> {
        let mut reference = cartesian_to_laser(reference)?;
        let mut sensor = cartesian_to_laser(sensor)?;
        self.match_laser(&mut reference, &mut sensor, guess)
    }

    /// Match reusable scan storage. The scans are mutated only in their
    /// private derived fields; their polar inputs remain unchanged.
    pub fn match_prepared(
        &self,
        reference: &mut PreparedPolarScan,
        sensor: &mut PreparedPolarScan,
    ) -> Result<MatchOutcome, ScanError> {
        self.match_prepared_from(reference, sensor, Pose::IDENTITY)
    }

    /// Match reusable scan storage with an explicit initial pose.
    pub fn match_prepared_from(
        &self,
        reference: &mut PreparedPolarScan,
        sensor: &mut PreparedPolarScan,
        guess: Pose,
    ) -> Result<MatchOutcome, ScanError> {
        reference.prepare_for_match();
        sensor.prepare_for_match();
        self.match_laser(&mut reference.data, &mut sensor.data, guess)
    }

    fn match_laser(
        &self,
        reference: &mut ScanData,
        sensor: &mut ScanData,
        guess: Pose,
    ) -> Result<MatchOutcome, ScanError> {
        if !guess.is_finite() {
            return Err(ScanError::NonFiniteGuess);
        }
        let mut result = MatchResult::default();
        let termination = icp::run_icp(
            &self.params,
            guess.to_array(),
            reference,
            sensor,
            &mut result,
        )?;
        Ok(MatchOutcome::from(result)
            .with_diagnostics(&self.params)
            .with_termination(termination))
    }
}

/// Copy optional weighting inputs onto a scan.
fn apply_weight_inputs(scan: &mut ScanData, sigma: Option<&[f64]>, true_alpha: Option<&[f64]>) {
    if let Some(sigma) = sigma {
        if sigma.len() == scan.nrays {
            scan.readings_sigma.copy_from_slice(sigma);
        }
    }
    if let Some(true_alpha) = true_alpha {
        if true_alpha.len() == scan.nrays {
            scan.true_alpha.copy_from_slice(true_alpha);
        }
    }
}

/// Convert Cartesian input to the engine's polar representation.
///
/// Explicit bearings are preserved; otherwise the bearing is `atan2(y, x)`.
fn cartesian_to_laser(scan: CartesianScan<'_>) -> Result<ScanData, ScanError> {
    let (angles, readings) = scan.to_polar_parts();
    ScanData::from_polar(angles, readings, scan.valid().to_vec())
}

/// Copy `source` into a reusable matrix slot, allocating only when the slot
/// was never reserved.
fn copy_matrix_into(slot: &mut Option<Matrix>, source: &Matrix) {
    match slot {
        Some(matrix) => matrix.copy_from(source),
        None => *slot = Some(source.clone()),
    }
}
