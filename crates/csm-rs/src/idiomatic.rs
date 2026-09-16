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
        Ok(Self { points, valid })
    }

    pub fn points(&self) -> &'a [[f64; 2]] {
        self.points
    }
    pub fn valid(&self) -> &'a [bool] {
        self.valid
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

    pub fn len(&self) -> usize {
        self.data.nrays
    }
    pub fn is_empty(&self) -> bool {
        self.data.nrays == 0
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
        }
    }
}

/// Immutable scan matcher configuration.
#[derive(Clone, Debug, Default)]
pub struct Matcher {
    params: Params,
}

impl Matcher {
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
