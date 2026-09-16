//! Matching configuration, workspaces, and results.
//!
//! The supported interface is organized around four ideas: ordered scans
//! ([`crate::PolarScan`], [`crate::CartesianScan`]), validated configuration
//! ([`crate::Params`]), matching ([`Matcher`]), and results
//! ([`MatchOutcome`]).

use crate::{
    icp,
    laser_data::{LaserData, ScanError},
    math::{Mat3, Matrix},
    params::{Params, ParamsError},
    pose::Pose,
    result::SmResult,
    scan::{CartesianScan, PolarScan},
};

/// Owned, reusable polar scan storage for steady-state matching.
#[derive(Clone, Debug)]
pub struct PreparedPolarScan {
    data: LaserData,
}

impl PreparedPolarScan {
    pub fn from_polar(
        angles: Vec<f64>,
        readings: Vec<f64>,
        valid: Vec<bool>,
    ) -> Result<Self, ScanError> {
        Ok(Self {
            data: LaserData::from_polar(angles, readings, valid)?,
        })
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

    pub fn len(&self) -> usize {
        self.data.nrays
    }

    pub fn is_empty(&self) -> bool {
        self.data.nrays == 0
    }

    pub fn capacity(&self) -> usize {
        self.data.theta.capacity()
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
        self.reset_derived();
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
        self.reset_derived();
        self.data.validate()
    }

    fn reset_derived(&mut self) {
        self.data.cluster.fill(-1);
        self.data.alpha.fill(f64::NAN);
        self.data.cov_alpha.fill(f64::NAN);
        self.data.alpha_valid.fill(false);
        self.data.true_alpha.fill(f64::NAN);
        self.data.corr.fill(Default::default());
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
        if reference.is_empty() || sensor.is_empty() {
            return Err(ScanError::NraysOutOfRange);
        }
        let scratch = icp::IcpScratch::new(
            reference.len(),
            sensor.len(),
            matcher.params.stopping.max_iterations.max(0) as usize,
        );
        Ok(Self {
            matcher,
            reference,
            sensor,
            scratch,
        })
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
        let mut result = SmResult::default();
        icp::sm_icp_with_scratch(
            &self.matcher.params,
            guess.to_array(),
            &mut self.reference.data,
            &mut self.sensor.data,
            &mut result,
            &mut self.scratch,
        )?;
        Ok(MatchOutcome::from(result).with_diagnostics(&self.matcher.params))
    }

    /// Match once and report the accepted result as a final snapshot.
    pub fn match_once_traced<F: FnMut(IterationSnapshot)>(
        &mut self,
        mut observe: F,
    ) -> Result<MatchOutcome, ScanError> {
        self.scratch.trace_events.clear();
        self.scratch.trace_enabled = true;
        let outcome = self.match_once();
        self.scratch.trace_enabled = false;
        let outcome = outcome?;
        for (iteration, pose, error, nvalid) in self.scratch.trace_events.drain(..) {
            observe(IterationSnapshot {
                iteration,
                pose,
                error,
                valid_correspondences: nvalid,
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
    /// Too few usable correspondences remained.
    NoCorrespondences,
    /// The configured iteration limit was reached.
    IterationLimit,
}

impl MatchOutcome {
    /// Whether covariance and both derivative matrices were produced.
    pub fn has_uncertainty(&self) -> bool {
        self.covariance.is_some() && self.dx_dy_reference.is_some() && self.dx_dy_sensor.is_some()
    }

    fn with_diagnostics(mut self, params: &Params) -> Self {
        self.covariance_status = if !params.do_compute_covariance {
            CovarianceStatus::Disabled
        } else if self.has_uncertainty() {
            CovarianceStatus::Computed
        } else {
            CovarianceStatus::Failed
        };
        self.termination = if self.valid {
            TerminationReason::Converged
        } else if self.nvalid == 0 {
            TerminationReason::NoCorrespondences
        } else if self.iterations >= params.stopping.max_iterations {
            TerminationReason::IterationLimit
        } else {
            TerminationReason::Failed
        };
        self
    }
}

/// One instrumented ICP iteration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IterationSnapshot {
    pub iteration: usize,
    pub pose: [f64; 3],
    pub error: f64,
    pub valid_correspondences: usize,
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
        self.status == MatchStatus::Converged
    }
}

impl From<SmResult> for MatchOutcome {
    fn from(result: SmResult) -> Self {
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
        let mut reference = LaserData::from_polar(
            reference.angles().to_vec(),
            reference.readings().to_vec(),
            reference.valid().to_vec(),
        )?;
        let mut sensor = LaserData::from_polar(
            sensor.angles().to_vec(),
            sensor.readings().to_vec(),
            sensor.valid().to_vec(),
        )?;
        self.match_laser(&mut reference, &mut sensor, guess)
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
        self.match_laser(&mut reference.data, &mut sensor.data, guess)
    }

    fn match_laser(
        &self,
        reference: &mut LaserData,
        sensor: &mut LaserData,
        guess: Pose,
    ) -> Result<MatchOutcome, ScanError> {
        if !guess.is_finite() {
            return Err(ScanError::NonFiniteGuess);
        }
        let mut result = SmResult::default();
        icp::sm_icp(
            &self.params,
            guess.to_array(),
            reference,
            sensor,
            &mut result,
        )?;
        Ok(MatchOutcome::from(result).with_diagnostics(&self.params))
    }
}

/// Convert Cartesian input to the engine's polar representation.
///
/// Explicit bearings are preserved; otherwise the bearing is `atan2(y, x)`.
fn cartesian_to_laser(scan: CartesianScan<'_>) -> Result<LaserData, ScanError> {
    let (angles, readings) = scan.to_polar_parts();
    LaserData::from_polar(angles, readings, scan.valid().to_vec())
}
