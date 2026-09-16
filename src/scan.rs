//! Ordered scan inputs.
//!
//! Both scan types borrow caller-owned buffers and never reorder, compact, or
//! discard rays. Explicitly invalid rays keep their position in the ordering,
//! which preserves the neighbourhood relationships used by the matcher.

use crate::laser_data::LaserData;

pub use crate::laser_data::ScanError;

/// A validated ordered polar scan borrowed from caller-owned buffers.
///
/// Rays are ordered by bearing. All angles and readings are in radians and
/// metres. The sensor sits at the origin of its own frame; a ray at angle
/// `theta` with valid reading `r` contributes the sensor-frame point
/// `[r cos(theta), r sin(theta)]`. A [missing return](Self::new) is marked by
/// `valid[i] == false` and keeps its position.
#[derive(Clone, Copy, Debug)]
pub struct PolarScan<'a> {
    angles: &'a [f64],
    readings: &'a [f64],
    valid: &'a [bool],
}

impl<'a> PolarScan<'a> {
    /// Validate and borrow ordered polar scan data.
    ///
    /// `angles`, `readings`, and `valid` must have equal length. Readings for
    /// rays marked valid must be finite. Missing returns keep their order and
    /// may carry any reading value.
    pub fn new(
        angles: &'a [f64],
        readings: &'a [f64],
        valid: &'a [bool],
    ) -> Result<Self, ScanError> {
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
        // Reuse the established validation rules while retaining borrowed
        // ownership. The input order is preserved.
        LaserData::from_polar(angles.to_vec(), readings.to_vec(), valid.to_vec())?;
        Ok(Self {
            angles,
            readings,
            valid,
        })
    }

    /// Bearings in radians, one per ray, in scan order.
    pub fn angles(&self) -> &'a [f64] {
        self.angles
    }

    /// Distances in metres, one per ray, in scan order.
    pub fn readings(&self) -> &'a [f64] {
        self.readings
    }

    /// Explicit per-ray validity, one entry per ray, in scan order.
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

/// A validated ordered Cartesian scan borrowed from caller-owned `(x, y)`
/// points.
///
/// Points are ordered by bearing around the sensor origin. The matcher needs
/// a bearing per ray for its neighbourhood search: either supply explicit
/// angles with [`CartesianScan::with_angles`], or let the matcher derive them
/// as `atan2(y, x)`. Points are never sorted or reordered.
#[derive(Clone, Copy, Debug)]
pub struct CartesianScan<'a> {
    points: &'a [[f64; 2]],
    angles: Option<&'a [f64]>,
    valid: &'a [bool],
}

impl<'a> CartesianScan<'a> {
    /// Borrow ordered points and derive bearings as `atan2(y, x)`.
    pub fn new(points: &'a [[f64; 2]], valid: &'a [bool]) -> Result<Self, ScanError> {
        if valid.len() != points.len() {
            return Err(ScanError::InconsistentLengths {
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
            return Err(ScanError::BadValidRay(index));
        }
        if points.len() < 10 {
            return Err(ScanError::NraysOutOfRange);
        }
        Ok(Self {
            points,
            angles: None,
            valid,
        })
    }

    /// Borrow ordered points with caller-supplied per-beam bearings.
    ///
    /// This is required when the ordering's neighbourhood is defined by
    /// something other than `atan2(y, x)` (for example a differently mounted
    /// scanner). Bearing order must match the point order.
    pub fn with_angles(
        points: &'a [[f64; 2]],
        angles: &'a [f64],
        valid: &'a [bool],
    ) -> Result<Self, ScanError> {
        if angles.len() != points.len() {
            return Err(ScanError::InconsistentLengths {
                field: "angles",
                expected: points.len(),
                actual: angles.len(),
            });
        }
        let mut scan = Self::new(points, valid)?;
        if angles.iter().any(|angle| !angle.is_finite()) {
            return Err(ScanError::BadValidRay(0));
        }
        for i in 1..angles.len() {
            if valid[i] && valid[i - 1] && angles[i] == angles[i - 1] {
                return Err(ScanError::DuplicateBearing(i));
            }
        }
        scan.angles = Some(angles);
        Ok(scan)
    }

    /// Sensor-frame `(x, y)` points in metres, in scan order.
    pub fn points(&self) -> &'a [[f64; 2]] {
        self.points
    }

    /// Explicit per-point validity, in scan order.
    pub fn valid(&self) -> &'a [bool] {
        self.valid
    }

    /// Caller-supplied bearings, if any. When `None`, the matcher derives
    /// `atan2(y, x)`.
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
