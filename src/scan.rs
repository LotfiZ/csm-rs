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
    sigma: Option<&'a [f64]>,
    true_alpha: Option<&'a [f64]>,
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
            sigma: None,
            true_alpha: None,
        })
    }

    /// Validate and borrow polar scan data with optional per-ray uncertainty
    /// inputs used by the sigma/ML weighting paths.
    ///
    /// `sigma` is the per-ray range standard deviation in metres (finite and
    /// non-negative, or `NaN` when unknown); `true_alpha` is a known surface
    /// orientation in radians used by ML weighting (`NaN` when unknown).
    pub fn with_inputs(
        angles: &'a [f64],
        readings: &'a [f64],
        valid: &'a [bool],
        sigma: Option<&'a [f64]>,
        true_alpha: Option<&'a [f64]>,
    ) -> Result<Self, ScanError> {
        let mut scan = Self::new(angles, readings, valid)?;
        if let Some(sigma) = sigma {
            if sigma.len() != angles.len() {
                return Err(ScanError::InconsistentLengths {
                    field: "sigma",
                    expected: angles.len(),
                    actual: sigma.len(),
                });
            }
            if sigma.iter().any(|value| *value < 0.0) {
                return Err(ScanError::NegativeSigma(0));
            }
        }
        if let Some(true_alpha) = true_alpha {
            if true_alpha.len() != angles.len() {
                return Err(ScanError::InconsistentLengths {
                    field: "true_alpha",
                    expected: angles.len(),
                    actual: true_alpha.len(),
                });
            }
        }
        scan.sigma = sigma;
        scan.true_alpha = true_alpha;
        Ok(scan)
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

    /// Optional per-ray range standard deviation in metres.
    pub fn sigma(&self) -> Option<&'a [f64]> {
        self.sigma
    }

    /// Optional per-ray known surface orientation in radians.
    pub fn true_alpha(&self) -> Option<&'a [f64]> {
        self.true_alpha
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
    /// scanner). Bearing order must match the point order. Bearings of valid
    /// points must be finite; a non-finite bearing on a missing return is
    /// tolerated because that ray is never matched.
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
        for (i, angle) in angles.iter().enumerate() {
            if valid[i] && !angle.is_finite() {
                return Err(ScanError::NonFiniteBearing(i));
            }
        }
        for i in 1..angles.len() {
            if valid[i] && valid[i - 1] && angles[i] == angles[i - 1] {
                return Err(ScanError::DuplicateBearing(i));
            }
        }
        scan.angles = Some(angles);
        Ok(scan)
    }

    /// Convert this scan to the polar `(angles, readings)` pair used by the
    /// engine. Bearings are preserved when supplied; otherwise they are
    /// derived as `atan2(y, x)`. Non-finite derived bearings (only possible on
    /// missing returns) are filled from neighbouring finite bearings so scan
    /// ordering metadata remains usable.
    pub(crate) fn to_polar_parts(self) -> (Vec<f64>, Vec<f64>) {
        let mut angles: Vec<f64> = match self.angles {
            Some(angles) => angles.to_vec(),
            None => self.points.iter().map(|p| p[1].atan2(p[0])).collect(),
        };
        if angles.iter().any(|angle| !angle.is_finite()) {
            fill_nonfinite(&mut angles);
        }
        let readings = self.points.iter().map(|p| p[0].hypot(p[1])).collect();
        (angles, readings)
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

    /// The effective bearings the matcher uses, in scan order.
    ///
    /// Returns the caller-supplied bearings when present, otherwise
    /// `atan2(y, x)` for each point. Bearings of missing returns whose
    /// coordinates are absent are filled from their nearest finite
    /// neighbours. This never reorders the scan.
    pub fn bearings(&self) -> Vec<f64> {
        self.to_polar_parts().0
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// Replace non-finite entries with the nearest finite neighbour, filling
/// forward then backward. Used for missing returns whose coordinates are
/// absent and whose derived bearing is therefore undefined.
fn fill_nonfinite(values: &mut [f64]) {
    let mut last = f64::NAN;
    for value in values.iter_mut() {
        if value.is_finite() {
            last = *value;
        } else if last.is_finite() {
            *value = last;
        }
    }
    let mut next = f64::NAN;
    for value in values.iter_mut().rev() {
        if value.is_finite() {
            next = *value;
        } else if next.is_finite() {
            *value = next;
        } else {
            // No finite bearing anywhere in the scan: use a neutral value so
            // the ordering metadata stays well-formed.
            *value = 0.0;
        }
    }
}
