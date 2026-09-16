//! Borrowed, immutable scan-matching API.

use crate::{
    icp,
    math::{Mat3, Matrix},
    LaserData, LaserDataError, Params, SmResult,
};

/// A validated polar scan borrowed from caller-owned buffers.
#[derive(Clone, Copy, Debug)]
pub struct PolarScan<'a> {
    angles: &'a [f64],
    readings: &'a [f64],
    valid: &'a [bool],
}

/// Ordered Cartesian scan borrowed from caller-owned `(x, y)` points.
#[derive(Clone, Copy, Debug)]
pub struct CartesianScan<'a> {
    points: &'a [[f64; 2]],
    angles: Option<&'a [f64]>,
    valid: &'a [bool],
}

impl<'a> CartesianScan<'a> {
    pub fn new(points: &'a [[f64; 2]], valid: &'a [bool]) -> Result<Self, LaserDataError> {
        if valid.len() != points.len() {
            return Err(LaserDataError::InconsistentLengths {
                field: "valid",
                expected: points.len(),
                actual: valid.len(),
            });
        }
        if let Some((index, _)) = points
            .iter()
            .zip(valid)
            .enumerate()
            .find(|(_, (p, ok))| **ok && (!p[0].is_finite() || !p[1].is_finite()))
        {
            return Err(LaserDataError::BadValidRay(index));
        }
        if points.len() < 10 {
            return Err(LaserDataError::NraysOutOfRange);
        }
        Ok(Self {
            points,
            angles: None,
            valid,
        })
    }

    /// Construct Cartesian input with caller-supplied per-beam bearings.
    pub fn with_angles(
        points: &'a [[f64; 2]],
        angles: &'a [f64],
        valid: &'a [bool],
    ) -> Result<Self, LaserDataError> {
        if angles.len() != points.len() {
            return Err(LaserDataError::InconsistentLengths {
                field: "angles",
                expected: points.len(),
                actual: angles.len(),
            });
        }
        let mut scan = Self::new(points, valid)?;
        if angles.iter().any(|angle| !angle.is_finite()) {
            return Err(LaserDataError::BadValidRay(0));
        }
        for i in 1..angles.len() {
            if valid[i] && valid[i - 1] && angles[i] == angles[i - 1] {
                return Err(LaserDataError::DuplicateBearing(i));
            }
        }
        scan.angles = Some(angles);
        Ok(scan)
    }

    pub fn points(&self) -> &'a [[f64; 2]] {
        self.points
    }
    pub fn valid(&self) -> &'a [bool] {
        self.valid
    }
    pub fn angles(&self) -> Option<&'a [f64]> {
        self.angles
    }
    pub fn len(&self) -> usize {
        self.points.len()
    }
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// Owned, reusable polar scan storage for steady-state matching.
#[derive(Clone, Debug)]
pub struct PreparedPolarScan {
    data: LaserData,
}

/// Reusable matching workspace for fixed-size streams.
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
    ) -> Result<Self, LaserDataError> {
        if reference.is_empty() || sensor.is_empty() {
            return Err(LaserDataError::NraysOutOfRange);
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

    /// Return `(reference_capacity, sensor_capacity)` in rays.
    pub fn capacities(&self) -> (usize, usize) {
        (self.reference.capacity(), self.sensor.capacity())
    }
    pub fn sensor_mut(&mut self) -> &mut PreparedPolarScan {
        &mut self.sensor
    }

    /// Update the streamed sensor frame while preserving its allocation.
    pub fn update_sensor(
        &mut self,
        readings: &[f64],
        valid: &[bool],
    ) -> Result<(), LaserDataError> {
        self.sensor.update(readings, valid)
    }

    /// Update an ordered Cartesian sensor frame in place.
    pub fn update_sensor_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), LaserDataError> {
        self.sensor.update_cartesian(points, valid)
    }

    /// Update an ordered Cartesian reference frame in place.
    pub fn update_reference_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), LaserDataError> {
        self.reference.update_cartesian(points, valid)
    }

    /// Update the reference frame while preserving its allocation.
    pub fn update_reference(
        &mut self,
        readings: &[f64],
        valid: &[bool],
    ) -> Result<(), LaserDataError> {
        self.reference.update(readings, valid)
    }

    pub fn match_once(&mut self) -> Result<MatchOutcome, LaserDataError> {
        let mut result = SmResult::default();
        icp::sm_icp_with_scratch(
            &self.matcher.params,
            &mut self.reference.data,
            &mut self.sensor.data,
            &mut result,
            &mut self.scratch,
        )?;
        let mut outcome: MatchOutcome = result.into();
        outcome.covariance_status = if !self.matcher.params.do_compute_covariance {
            CovarianceStatus::Disabled
        } else if outcome.has_uncertainty() {
            CovarianceStatus::Computed
        } else {
            CovarianceStatus::Failed
        };
        outcome.termination = if outcome.valid {
            TerminationReason::Converged
        } else if outcome.nvalid == 0 {
            TerminationReason::NoCorrespondences
        } else if outcome.iterations >= self.matcher.params.stopping.max_iterations {
            TerminationReason::IterationLimit
        } else {
            TerminationReason::Failed
        };
        Ok(outcome)
    }

    /// Match into caller-owned result storage for allocation-free result reuse.
    pub fn match_once_into(&mut self, result: &mut SmResult) -> Result<(), LaserDataError> {
        icp::sm_icp_with_scratch(
            &self.matcher.params,
            &mut self.reference.data,
            &mut self.sensor.data,
            result,
            &mut self.scratch,
        )
    }

    /// Match once and report the accepted result as a final snapshot.
    pub fn match_once_traced<F: FnMut(IterationSnapshot)>(
        &mut self,
        mut observe: F,
    ) -> Result<MatchOutcome, LaserDataError> {
        self.scratch.observer = None;
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

impl PreparedPolarScan {
    pub fn from_polar(
        angles: Vec<f64>,
        readings: Vec<f64>,
        valid: Vec<bool>,
    ) -> Result<Self, LaserDataError> {
        Ok(Self {
            data: LaserData::from_polar(angles, readings, valid)?,
        })
    }

    pub fn from_cartesian(points: &[[f64; 2]], valid: &[bool]) -> Result<Self, LaserDataError> {
        let scan = CartesianScan::new(points, valid)?;
        let angles = scan.points.iter().map(|p| p[1].atan2(p[0])).collect();
        let readings = scan.points.iter().map(|p| p[0].hypot(p[1])).collect();
        Self::from_polar(angles, readings, valid.to_vec())
    }

    pub fn from_cartesian_with_angles(
        points: &[[f64; 2]],
        angles: &[f64],
        valid: &[bool],
    ) -> Result<Self, LaserDataError> {
        let scan = CartesianScan::with_angles(points, angles, valid)?;
        let readings = scan.points.iter().map(|p| p[0].hypot(p[1])).collect();
        Self::from_polar(angles.to_vec(), readings, valid.to_vec())
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
    pub fn update(&mut self, readings: &[f64], valid: &[bool]) -> Result<(), LaserDataError> {
        if readings.len() != self.data.nrays {
            return Err(LaserDataError::InconsistentLengths {
                field: "readings",
                expected: self.data.nrays,
                actual: readings.len(),
            });
        }
        if valid.len() != self.data.nrays {
            return Err(LaserDataError::InconsistentLengths {
                field: "valid",
                expected: self.data.nrays,
                actual: valid.len(),
            });
        }
        self.data.readings.copy_from_slice(readings);
        self.data.valid.copy_from_slice(valid);
        self.data.cluster.fill(-1);
        self.data.alpha.fill(f64::NAN);
        self.data.cov_alpha.fill(f64::NAN);
        self.data.alpha_valid.fill(false);
        self.data.true_alpha.fill(f64::NAN);
        self.data.corr.fill(Default::default());
        self.data.validate()
    }

    /// Refresh an ordered Cartesian frame in place.
    pub fn update_cartesian(
        &mut self,
        points: &[[f64; 2]],
        valid: &[bool],
    ) -> Result<(), LaserDataError> {
        if points.len() != self.data.nrays {
            return Err(LaserDataError::InconsistentLengths {
                field: "points",
                expected: self.data.nrays,
                actual: points.len(),
            });
        }
        let scan = CartesianScan::new(points, valid)?;
        for (reading, point) in self.data.readings.iter_mut().zip(scan.points) {
            *reading = point[0].hypot(point[1]);
        }
        self.data.valid.copy_from_slice(valid);
        self.data.cluster.fill(-1);
        self.data.alpha.fill(f64::NAN);
        self.data.cov_alpha.fill(f64::NAN);
        self.data.alpha_valid.fill(false);
        self.data.true_alpha.fill(f64::NAN);
        self.data.corr.fill(Default::default());
        self.data.validate()
    }
}

impl<'a> PolarScan<'a> {
    /// Validate and borrow polar scan data. Inputs use metres and radians.
    pub fn new(
        angles: &'a [f64],
        readings: &'a [f64],
        valid: &'a [bool],
    ) -> Result<Self, LaserDataError> {
        if readings.len() != angles.len() {
            return Err(LaserDataError::InconsistentLengths {
                field: "readings",
                expected: angles.len(),
                actual: readings.len(),
            });
        }
        if valid.len() != angles.len() {
            return Err(LaserDataError::InconsistentLengths {
                field: "valid",
                expected: angles.len(),
                actual: valid.len(),
            });
        }
        // Reuse the established validation rules while retaining borrowed ownership.
        LaserData::from_polar(angles.to_vec(), readings.to_vec(), valid.to_vec())?;
        Ok(Self {
            angles,
            readings,
            valid,
        })
    }

    pub fn angles(&self) -> &'a [f64] {
        self.angles
    }
    pub fn readings(&self) -> &'a [f64] {
        self.readings
    }
    pub fn valid(&self) -> &'a [bool] {
        self.valid
    }
    pub fn len(&self) -> usize {
        self.angles.len()
    }
    pub fn is_empty(&self) -> bool {
        self.angles.is_empty()
    }
}

/// Result of a match. A well-formed input can produce an unsuccessful result;
/// inspect [`Self::valid`] to distinguish that case from input errors.
#[derive(Clone, Debug, Default)]
pub struct MatchOutcome {
    pub status: MatchStatus,
    pub valid: bool,
    pub pose: [f64; 3],
    pub iterations: i32,
    pub nvalid: i32,
    pub error: f64,
    pub covariance: Option<Mat3>,
    pub dx_dy_reference: Option<Matrix>,
    pub dx_dy_sensor: Option<Matrix>,
    pub covariance_status: CovarianceStatus,
    pub termination: TerminationReason,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CovarianceStatus {
    #[default]
    Disabled,
    Computed,
    Failed,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TerminationReason {
    #[default]
    Failed,
    Converged,
    NoCorrespondences,
    IterationLimit,
}

impl MatchOutcome {
    /// Whether covariance and both derivative matrices were produced.
    pub fn has_uncertainty(&self) -> bool {
        self.covariance.is_some() && self.dx_dy_reference.is_some() && self.dx_dy_sensor.is_some()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IterationSnapshot {
    pub iteration: usize,
    pub pose: [f64; 3],
    pub error: f64,
    pub valid_correspondences: usize,
}

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
            pose: result.x,
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

/// Immutable scan matcher configuration.
#[derive(Clone, Debug, Default)]
pub struct Matcher {
    params: Params,
}

impl Matcher {
    /// Construct a matcher after validating all numeric parameters.
    pub fn try_new(params: Params) -> Result<Self, crate::params::ParamsError> {
        params.validate()?;
        Ok(Self { params })
    }

    /// Construct a validated pose-only matcher for constrained deployments.
    pub fn try_pose_only(mut params: Params) -> Result<Self, crate::params::ParamsError> {
        params.do_compute_covariance = false;
        Self::try_new(params)
    }

    #[must_use]
    pub fn new(params: Params) -> Self {
        Self { params }
    }
    /// Create a pose-only matcher for constrained or embedded deployments.
    pub fn pose_only(mut params: Params) -> Self {
        params.do_compute_covariance = false;
        Self { params }
    }
    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Build a reusable workspace from two owned, fixed-shape scans.
    pub fn prepare(
        &self,
        reference: PreparedPolarScan,
        sensor: PreparedPolarScan,
    ) -> Result<PreparedMatcher, LaserDataError> {
        PreparedMatcher::new(self.clone(), reference, sensor)
    }

    /// Build a reusable workspace directly from ordered Cartesian frames.
    pub fn prepare_cartesian(
        &self,
        reference_points: &[[f64; 2]],
        reference_valid: &[bool],
        sensor_points: &[[f64; 2]],
        sensor_valid: &[bool],
    ) -> Result<PreparedMatcher, LaserDataError> {
        let reference = PreparedPolarScan::from_cartesian(reference_points, reference_valid)?;
        let sensor = PreparedPolarScan::from_cartesian(sensor_points, sensor_valid)?;
        self.prepare(reference, sensor)
    }

    /// Build a reusable workspace directly from borrowed polar frame data.
    pub fn prepare_polar(
        &self,
        reference: PolarScan<'_>,
        sensor: PolarScan<'_>,
    ) -> Result<PreparedMatcher, LaserDataError> {
        let reference = PreparedPolarScan::from_polar(
            reference.angles.to_vec(),
            reference.readings.to_vec(),
            reference.valid.to_vec(),
        )?;
        let sensor = PreparedPolarScan::from_polar(
            sensor.angles.to_vec(),
            sensor.readings.to_vec(),
            sensor.valid.to_vec(),
        )?;
        self.prepare(reference, sensor)
    }

    /// Match two borrowed polar scans without modifying caller buffers.
    pub fn match_polar(
        &self,
        reference: PolarScan<'_>,
        sensor: PolarScan<'_>,
    ) -> Result<MatchOutcome, LaserDataError> {
        let mut reference = LaserData::from_polar(
            reference.angles.to_vec(),
            reference.readings.to_vec(),
            reference.valid.to_vec(),
        )?;
        let mut sensor = LaserData::from_polar(
            sensor.angles.to_vec(),
            sensor.readings.to_vec(),
            sensor.valid.to_vec(),
        )?;
        let mut result = SmResult::default();
        icp::sm_icp(&self.params, &mut reference, &mut sensor, &mut result)?;
        Ok(result.into())
    }

    /// Match ordered Cartesian scans by converting them once to the engine's
    /// polar representation. Point buffers are borrowed and never modified.
    pub fn match_cartesian(
        &self,
        reference: CartesianScan<'_>,
        sensor: CartesianScan<'_>,
    ) -> Result<MatchOutcome, LaserDataError> {
        let to_polar = |scan: CartesianScan<'_>| {
            let angles: Vec<f64> = scan.points.iter().map(|p| p[1].atan2(p[0])).collect();
            let readings: Vec<f64> = scan.points.iter().map(|p| p[0].hypot(p[1])).collect();
            LaserData::from_polar(angles, readings, scan.valid.to_vec())
        };
        let mut reference = to_polar(reference)?;
        let mut sensor = to_polar(sensor)?;
        let mut result = SmResult::default();
        icp::sm_icp(&self.params, &mut reference, &mut sensor, &mut result)?;
        Ok(result.into())
    }

    /// Match reusable scan storage. The scans are mutated only in their
    /// private derived fields; their polar inputs remain unchanged.
    pub fn match_prepared(
        &self,
        reference: &mut PreparedPolarScan,
        sensor: &mut PreparedPolarScan,
    ) -> Result<MatchOutcome, LaserDataError> {
        let mut result = SmResult::default();
        icp::sm_icp(
            &self.params,
            &mut reference.data,
            &mut sensor.data,
            &mut result,
        )?;
        Ok(result.into())
    }

    /// Reuse a caller-owned legacy result buffer for a prepared match.
    ///
    /// This is useful in fixed-rate loops where the result storage is kept
    /// alongside the scan workspace.
    pub fn match_prepared_into(
        &self,
        reference: &mut PreparedPolarScan,
        sensor: &mut PreparedPolarScan,
        result: &mut SmResult,
    ) -> Result<(), LaserDataError> {
        *result = SmResult::default();
        icp::sm_icp(&self.params, &mut reference.data, &mut sensor.data, result)
    }

    /// Match and notify an application-owned observer after each result.
    pub fn match_prepared_observed<F: FnMut(&MatchOutcome)>(
        &self,
        reference: &mut PreparedPolarScan,
        sensor: &mut PreparedPolarScan,
        mut observe: F,
    ) -> Result<MatchOutcome, LaserDataError> {
        let outcome = self.match_prepared(reference, sensor)?;
        observe(&outcome);
        Ok(outcome)
    }
}
