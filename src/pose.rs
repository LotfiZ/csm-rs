//! Rigid 2D pose used for initial guesses and match results.

use crate::math;

/// A rigid 2D transform mapping sensor-scan coordinates into reference-scan
/// coordinates.
///
/// The pose is `(x, y, theta)`: a counter-clockwise rotation by `theta`
/// (radians) followed by a translation `(x, y)` in metres. Applying it to a
/// point `p` in the sensor frame yields `R(theta) * p + (x, y)`.
///
/// ```
/// use csm_rs::Pose;
///
/// let sensor_point = [1.0, 0.0];
/// let pose = Pose::new(0.5, 0.0, std::f64::consts::FRAC_PI_2);
/// let [x, y] = pose.transform_point(sensor_point);
/// assert!((x - 0.5).abs() < 1e-12 && (y - 1.0).abs() < 1e-12);
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Translation along the reference x axis, in metres.
    pub x: f64,
    /// Translation along the reference y axis, in metres.
    pub y: f64,
    /// Counter-clockwise rotation, in radians.
    pub theta: f64,
}

impl Pose {
    /// The identity transform: no rotation or translation.
    pub const IDENTITY: Pose = Pose {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    };

    /// Construct a pose from metres and radians.
    pub const fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    /// Build a pose from a raw `(x, y, theta)` array.
    pub const fn from_array(values: [f64; 3]) -> Self {
        Self::new(values[0], values[1], values[2])
    }

    /// Return the pose as a raw `(x, y, theta)` array.
    pub const fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.theta]
    }

    /// Whether every component is finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.theta.is_finite()
    }

    /// Apply this pose to a point in the sensor frame, returning a point in
    /// the reference frame.
    pub fn transform_point(self, point: [f64; 2]) -> [f64; 2] {
        math::transform(point, self.to_array())
    }

    /// Compose two poses: `self ∘ other`. The result applies `other` first,
    /// then `self`.
    pub fn compose(self, other: Pose) -> Pose {
        Pose::from_array(math::oplus(self.to_array(), other.to_array()))
    }

    /// The inverse transform.
    pub fn inverse(self) -> Pose {
        Pose::from_array(math::ominus(self.to_array()))
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_is_neutral_for_composition() {
        let pose = Pose::new(0.4, -0.2, 0.3);
        assert_eq!(Pose::IDENTITY.compose(pose), pose);
        assert_eq!(pose.compose(Pose::IDENTITY), pose);
    }

    #[test]
    fn inverse_round_trips() {
        let pose = Pose::new(0.4, -0.2, 0.3);
        let round_trip = pose.compose(pose.inverse());
        assert!(round_trip.x.abs() < 1e-12);
        assert!(round_trip.y.abs() < 1e-12);
        assert!(round_trip.theta.abs() < 1e-12);
    }

    #[test]
    fn composition_applies_other_first() {
        let rotate = Pose::new(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let translate = Pose::new(1.0, 0.0, 0.0);
        let point = [1.0, 0.0];
        let composed = translate.compose(rotate).transform_point(point);
        assert!((composed[0] - 1.0).abs() < 1e-12);
        assert!((composed[1] - 1.0).abs() < 1e-12);
    }
}
