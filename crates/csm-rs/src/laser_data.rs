//! Laser scan data structure and derived quantities.
//!
//! C: `sm/csm/laser_data.h`, `sm/csm/laser_data.c`, `sm/csm/laser_data_inline.h`,
//!     `sm/csm/laser_data_bbox.c`, `sm/csm/clustering.c`, `sm/csm/orientation.c`
//!
//! Ports: polar→cartesian conversion, world-coordinate transform, jump tables
//! for the smart correspondence search, simple clustering, orientation
//! estimation, and the per-ray validity model (`valid` flags + NaN readings,
//! kept C-faithful per grilling Q9).

/// A single 2D point with its polar representation.
///
/// C: `struct point2d` in `laser_data.h`
#[derive(Clone, Copy, Debug, Default)]
pub struct Point2d {
    pub p: [f64; 2],
    pub rho: f64,
    pub phi: f64,
}

/// One laser scan: `nrays` rays with polar + cartesian representations.
///
/// C: `struct laser_data` in `laser_data.h`
// TODO(port): fields — theta, valid, readings, cluster, alpha, cov_alpha,
// alpha_valid, readings_sigma, true_alpha, corr, true_pose, odometry,
// estimate, points, points_w, jump tables (up/down × bigger/smaller).
pub struct LaserData {
    pub nrays: usize,
    pub min_theta: f64,
    pub max_theta: f64,
}

/// One correspondence between a ray of the sensor scan and the reference scan.
///
/// C: `struct correspondence` in `laser_data.h`
#[derive(Clone, Copy, Debug, Default)]
pub struct Correspondence {
    pub valid: bool,
    /// Closest point in the other scan.
    pub j1: i32,
    /// Second closest point in the other scan.
    pub j2: i32,
    /// Point-to-point or point-to-line.
    pub corr_type: CorrespondenceType,
    /// Squared distance from p(i) to point j1.
    pub dist2_j1: f64,
}

/// C: `enum { corr_pp, corr_pl }` in `laser_data.h`
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CorrespondenceType {
    #[default]
    PointToPoint,
    PointToLine,
}
