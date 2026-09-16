//! Laser scan data structure and derived quantities.
//!
//! C: `sm/csm/laser_data.h`, `sm/csm/laser_data.c`, `sm/csm/laser_data_inline.h`,
//!     `sm/csm/laser_data_bbox.c`, `sm/csm/clustering.c`, `sm/csm/orientation.c`,
//!     visibility in `sm/csm/icp/icp_outliers.c`,
//!     jump tables in `sm/csm/icp/icp_corr_tricks.c:ld_create_jump_tables()`
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

/// One correspondence between a ray of the sensor scan and the reference scan.
///
/// C: `struct correspondence` in `laser_data.h`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Correspondence {
    pub valid: bool,
    /// Closest point in the other scan (-1 when unset).
    pub j1: i32,
    /// Second closest point in the other scan (-1 when unset).
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

impl Default for Correspondence {
    /// C: `ld_alloc()` initializes `valid=0, j1=-1, j2=-1`.
    fn default() -> Self {
        Self {
            valid: false,
            j1: -1,
            j2: -1,
            corr_type: CorrespondenceType::PointToPoint,
            dist2_j1: 0.0,
        }
    }
}

/// One laser scan: `nrays` rays with polar + cartesian representations.
///
/// C: `struct laser_data` in `laser_data.h`
#[derive(Clone, Debug)]
pub struct LaserData {
    pub nrays: usize,
    pub min_theta: f64,
    pub max_theta: f64,

    pub theta: Vec<f64>,
    pub valid: Vec<bool>,
    pub readings: Vec<f64>,
    pub readings_sigma: Vec<f64>,

    /// -1 = no cluster. C: `cluster`
    pub cluster: Vec<i32>,
    /// Estimated orientation (NaN if not computed). C: `alpha`
    pub alpha: Vec<f64>,
    pub cov_alpha: Vec<f64>,
    pub alpha_valid: Vec<bool>,
    pub true_alpha: Vec<f64>,

    pub corr: Vec<Correspondence>,

    pub true_pose: [f64; 3],
    pub odometry: [f64; 3],
    pub estimate: [f64; 3],

    /// Cartesian points in the scan frame. C: `points`
    pub points: Vec<Point2d>,
    /// Cartesian points in the world frame. C: `points_w`
    pub points_w: Vec<Point2d>,

    /// Jump tables for the tricks correspondence search.
    /// C: `up_bigger`, `up_smaller`, `down_bigger`, `down_smaller`
    pub up_bigger: Vec<i32>,
    pub up_smaller: Vec<i32>,
    pub down_bigger: Vec<i32>,
    pub down_smaller: Vec<i32>,
}

impl LaserData {
    /// Allocate a scan with `nrays` rays spanning `[min_theta, max_theta]`;
    /// `theta` is linearly spaced (so the C invariant `min_theta == theta[0]`
    /// holds). Everything else mirrors `ld_alloc()`: rays invalid, readings
    /// NaN, cluster -1, correspondences unset, points NaN.
    ///
    /// C: `ld_alloc_new()` / `ld_alloc()` in `laser_data.c`
    pub fn new(nrays: usize, min_theta: f64, max_theta: f64) -> Self {
        let theta: Vec<f64> = (0..nrays)
            .map(|i| min_theta + (max_theta - min_theta) * i as f64 / (nrays - 1) as f64)
            .collect();
        let nan_point = Point2d {
            p: [f64::NAN, f64::NAN],
            rho: f64::NAN,
            phi: f64::NAN,
        };
        Self {
            nrays,
            min_theta,
            max_theta,
            theta,
            valid: vec![false; nrays],
            readings: vec![f64::NAN; nrays],
            readings_sigma: vec![f64::NAN; nrays],
            cluster: vec![-1; nrays],
            alpha: vec![f64::NAN; nrays],
            cov_alpha: vec![f64::NAN; nrays],
            alpha_valid: vec![false; nrays],
            true_alpha: vec![f64::NAN; nrays],
            corr: vec![Correspondence::default(); nrays],
            true_pose: [f64::NAN; 3],
            odometry: [f64::NAN; 3],
            estimate: [f64::NAN; 3],
            points: vec![nan_point; nrays],
            points_w: vec![nan_point; nrays],
            up_bigger: vec![0; nrays],
            up_smaller: vec![0; nrays],
            down_bigger: vec![0; nrays],
            down_smaller: vec![0; nrays],
        }
    }

    /// Mark valid rays outside `(min_reading, max_reading]` as invalid.
    ///
    /// C: `ld_invalid_if_outside()` in `icp/icp.c`
    pub fn invalid_if_outside(&mut self, min_reading: f64, max_reading: f64) {
        for i in 0..self.nrays {
            if !self.valid[i] {
                continue;
            }
            let r = self.readings[i];
            if r <= min_reading || r > max_reading {
                self.valid[i] = false;
            }
        }
    }

    /// Polar → cartesian in the scan frame, for *all* rays (C-faithful: no
    /// validity check, so NaN readings yield NaN points). `rho`/`phi` of
    /// `points` are set to NaN.
    ///
    /// C: `ld_compute_cartesian()` in `laser_data.c`
    pub fn compute_cartesian(&mut self) {
        for i in 0..self.nrays {
            self.points[i].p[0] = self.theta[i].cos() * self.readings[i];
            self.points[i].p[1] = self.theta[i].sin() * self.readings[i];
            self.points[i].rho = f64::NAN;
            self.points[i].phi = f64::NAN;
        }
    }

    /// Transform `points` into the world frame under `pose` (valid rays
    /// only), then compute `rho`/`phi` of `points_w` for *all* rays.
    ///
    /// C: `ld_compute_world_coords()` in `laser_data.c`
    pub fn compute_world_coords(&mut self, pose: &[f64; 3]) {
        let (px, py, theta) = (pose[0], pose[1], pose[2]);
        let (c, s) = (theta.cos(), theta.sin());
        for i in 0..self.nrays {
            if !self.valid[i] {
                continue;
            }
            let (x0, y0) = (self.points[i].p[0], self.points[i].p[1]);
            self.points_w[i].p[0] = c * x0 - s * y0 + px;
            self.points_w[i].p[1] = s * x0 + c * y0 + py;
        }
        for i in 0..self.nrays {
            let (x, y) = (self.points_w[i].p[0], self.points_w[i].p[1]);
            self.points_w[i].rho = (x * x + y * y).sqrt();
            self.points_w[i].phi = y.atan2(x);
        }
    }

    /// Invalidate points hidden from a viewpoint when their bearing reverses
    /// along the scan order.
    ///
    /// C: `visibilityTest()` in `sm/csm/icp/icp_outliers.c`. The C routine
    /// uses only the viewpoint translation; the pose angle is intentionally
    /// ignored.
    pub fn visibility_test(&mut self, viewpoint: &[f64; 3]) {
        let mut theta_from_viewpoint = vec![f64::NAN; self.nrays];
        for (i, angle) in theta_from_viewpoint.iter_mut().enumerate() {
            if !self.valid[i] {
                continue;
            }
            *angle = (viewpoint[1] - self.points[i].p[1]).atan2(viewpoint[0] - self.points[i].p[0]);
        }

        for i in 1..self.nrays {
            if !self.valid[i] || !self.valid[i - 1] {
                continue;
            }
            if theta_from_viewpoint[i] < theta_from_viewpoint[i - 1] {
                self.valid[i] = false;
            }
        }
    }

    /// Build the four jump tables used by the tricks correspondence search.
    ///
    /// C: `ld_create_jump_tables()` in `icp/icp_corr_tricks.c`
    pub fn create_jump_tables(&mut self) {
        for i in 0..self.nrays {
            let mut j = i as i32 + 1;
            while (j as usize) < self.nrays
                && self.valid[j as usize]
                && self.readings[j as usize] <= self.readings[i]
            {
                j += 1;
            }
            self.up_bigger[i] = j - i as i32;

            let mut j = i as i32 + 1;
            while (j as usize) < self.nrays
                && self.valid[j as usize]
                && self.readings[j as usize] >= self.readings[i]
            {
                j += 1;
            }
            self.up_smaller[i] = j - i as i32;

            let mut j = i as i32 - 1;
            while j >= 0 && self.valid[j as usize] && self.readings[j as usize] >= self.readings[i]
            {
                j -= 1;
            }
            self.down_smaller[i] = j - i as i32;

            let mut j = i as i32 - 1;
            while j >= 0 && self.valid[j as usize] && self.readings[j as usize] <= self.readings[i]
            {
                j -= 1;
            }
            self.down_bigger[i] = j - i as i32;
        }
    }

    /// Cluster consecutive valid rays whose readings jump by less than
    /// `threshold`. Invalid rays get cluster -1; `last_reading` survives
    /// invalid rays (C-faithful).
    ///
    /// C: `ld_simple_clustering()` in `clustering.c`
    pub fn simple_clustering(&mut self, threshold: f64) {
        let mut cluster: i32 = -1;
        let mut last_reading = 0.0;
        for i in 0..self.nrays {
            if !self.valid[i] {
                self.cluster[i] = -1;
                continue;
            }
            if cluster == -1 {
                cluster = 0;
            } else if (last_reading - self.readings[i]).abs() > threshold {
                cluster += 1;
            }
            self.cluster[i] = cluster;
            last_reading = self.readings[i];
        }
    }

    /// Neighbour ray indexes for orientation estimation: up to `max_num`
    /// valid same-cluster rays ascending, then descending (order preserved
    /// for floating-point fidelity).
    ///
    /// C: `find_neighbours()` in `orientation.c` — note the asymmetric
    /// bounds (`up+1 <= i+max_num` vs `down >= i-max_num`), ported as-is.
    fn find_neighbours(&self, i: usize, max_num: i32) -> Vec<usize> {
        let i32_i = i as i32;
        let mut indexes = Vec::new();
        let mut up = i32_i;
        while up < i32_i + max_num
            && (up + 1) < self.nrays as i32
            && self.valid[(up + 1) as usize]
            && self.cluster[(up + 1) as usize] == self.cluster[i]
        {
            up += 1;
            indexes.push(up as usize);
        }
        let mut down = i32_i;
        while down >= i32_i - max_num
            && down > 0
            && self.valid[(down - 1) as usize]
            && self.cluster[(down - 1) as usize] == self.cluster[i]
        {
            down -= 1;
            indexes.push(down as usize);
        }
        indexes
    }

    /// Estimate the local surface orientation at each valid, clustered ray.
    /// Requires `cluster` to be set (see [`Self::simple_clustering`]).
    ///
    /// C: `ld_compute_orientation()` in `orientation.c`
    pub fn compute_orientation(&mut self, size_neighbourhood: i32, sigma: f64) {
        for i in 0..self.nrays {
            if !self.valid[i] || self.cluster[i] == -1 {
                self.alpha[i] = f64::NAN;
                self.cov_alpha[i] = f64::NAN;
                self.alpha_valid[i] = false;
                continue;
            }
            let neighbours = self.find_neighbours(i, size_neighbourhood);
            if neighbours.is_empty() {
                self.alpha[i] = f64::NAN;
                self.cov_alpha[i] = f64::NAN;
                self.alpha_valid[i] = false;
                continue;
            }
            let thetas: Vec<f64> = neighbours.iter().map(|&j| self.theta[j]).collect();
            let readings: Vec<f64> = neighbours.iter().map(|&j| self.readings[j]).collect();
            let (alpha, cov0_alpha) =
                filter_orientation(self.theta[i], self.readings[i], &thetas, &readings);
            if alpha.is_nan() {
                self.alpha[i] = f64::NAN;
                self.cov_alpha[i] = f64::NAN;
                self.alpha_valid[i] = false;
            } else {
                self.alpha[i] = alpha;
                self.cov_alpha[i] = cov0_alpha * sigma * sigma;
                self.alpha_valid[i] = true;
            }
        }
    }

    /// Structural validation of the scan, mirroring the C checks.
    ///
    /// C: `ld_valid_fields()` in `laser_data.c`
    pub fn validate(&self) -> Result<(), LaserDataError> {
        use LaserDataError as E;
        if self.nrays < 10 || self.nrays > 10000 {
            return Err(E::NraysOutOfRange);
        }
        if self.min_theta.is_nan() || self.max_theta.is_nan() {
            return Err(E::NanThetaBounds);
        }
        let fov = self.max_theta - self.min_theta;
        if fov < 20.0f64.to_radians() || fov > 2.01 * std::f64::consts::PI {
            return Err(E::FovOutOfRange);
        }
        if (self.min_theta - self.theta[0]).abs() > 1e-8
            || (self.max_theta - self.theta[self.nrays - 1]).abs() > 1e-8
        {
            return Err(E::ThetaBoundsMismatch);
        }
        for i in 0..self.nrays {
            if self.valid[i] {
                let r = self.readings[i];
                if r.is_nan() || self.theta[i].is_nan() || !(0.0 < r && r < 100.0) {
                    return Err(E::BadValidRay(i));
                }
            } else {
                if self.theta[i].is_nan() {
                    return Err(E::BadValidRay(i));
                }
                if self.cluster[i] != -1 {
                    return Err(E::BadCluster(i));
                }
            }
            if self.cluster[i] < -1 {
                return Err(E::BadCluster(i));
            }
            if !self.readings_sigma[i].is_nan() && self.readings_sigma[i] < 0.0 {
                return Err(E::NegativeSigma(i));
            }
        }
        let num_valid = self.valid.iter().filter(|&&v| v).count();
        // C compares in floating point: num_valid < nrays * 0.10
        if (num_valid as f64) < self.nrays as f64 * 0.10 {
            return Err(E::TooFewValidRays);
        }
        Ok(())
    }
}

/// Input validation failures, mirroring the `sm_error` checks in
/// `ld_valid_fields()`. Malformed input is an error; a failed *match* is not
/// (grilling Q9).
///
/// C: `ld_valid_fields()` in `laser_data.c`
#[derive(Clone, Debug, PartialEq)]
pub enum LaserDataError {
    /// C: nrays must be in [10, 10000]
    NraysOutOfRange,
    /// C: min/max theta must not be NaN
    NanThetaBounds,
    /// C: FOV must be in [20 deg, 2.01 pi]
    FovOutOfRange,
    /// C: min_theta must equal theta[0], max_theta must equal theta[last]
    ThetaBoundsMismatch,
    /// C: a valid ray must have non-NaN reading/theta and reading in (0, 100)
    BadValidRay(usize),
    /// C: invalid rays must have cluster == -1, and cluster must be >= -1
    BadCluster(usize),
    /// C: readings_sigma must be non-negative when set
    NegativeSigma(usize),
    /// C: at least 10% of rays must be valid
    TooFewValidRays,
}

impl std::fmt::Display for LaserDataError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NraysOutOfRange => write!(f, "invalid number of rays (need 10..=10000)"),
            Self::NanThetaBounds => write!(f, "NaN min/max theta"),
            Self::FovOutOfRange => write!(f, "FOV outside [20 deg, 2.01 pi]"),
            Self::ThetaBoundsMismatch => {
                write!(f, "min/max theta do not match theta[0]/theta[last]")
            }
            Self::BadValidRay(i) => {
                write!(f, "ray #{i}: NaN or out-of-(0,100) reading on valid ray")
            }
            Self::BadCluster(i) => write!(f, "ray #{i}: bad cluster value"),
            Self::NegativeSigma(i) => write!(f, "ray #{i}: negative readings_sigma"),
            Self::TooFewValidRays => write!(f, "fewer than 10% valid rays"),
        }
    }
}

impl std::error::Error for LaserDataError {}

/// Weighted least-squares orientation filter. Returns `(alpha, cov0_alpha)`;
/// `alpha` is NaN when the system is singular.
///
/// C: `filter_orientation()` in `orientation.c`. The model is
/// `Y = L·f1 + R·ε` with `L = ones(n)`, solved via the n×n inverse of
/// `R·Rᵀ` (egsl in C; [`crate::math::invert_dyn`] here).
fn filter_orientation(theta0: f64, rho0: f64, thetas: &[f64], rhos: &[f64]) -> (f64, f64) {
    let n = thetas.len();
    // Y[i] = (rho_i - rho0) / (theta_i - theta0)
    // R[i,0] = -1/(theta_i - theta0), R[i,i+1] = +1/(theta_i - theta0)
    let mut y = vec![0.0; n];
    let mut rrt = vec![vec![0.0; n]; n];
    let mut r: Vec<Vec<f64>> = vec![vec![0.0; n + 1]; n];
    for i in 0..n {
        let dt = thetas[i] - theta0;
        y[i] = (rhos[i] - rho0) / dt;
        r[i][0] = -1.0 / dt;
        r[i][i + 1] = 1.0 / dt;
    }
    for i in 0..n {
        for j in 0..n {
            rrt[i][j] = (0..=n).map(|k| r[i][k] * r[j][k]).sum();
        }
    }
    let Some(erinv) = crate::math::invert_dyn(&rrt) else {
        return (f64::NAN, f64::NAN);
    };
    // L = ones(n): Lᵀ·eRinv·L is the sum of all entries (scalar);
    // Lᵀ·eRinv·Y is the sum of entries of eRinv·Y (scalar).
    let mut sum_all = 0.0;
    let mut erinv_y = vec![0.0; n];
    for (i, row) in erinv.iter().enumerate() {
        for (j, &e) in row.iter().enumerate() {
            sum_all += e;
            erinv_y[i] += e * y[j];
        }
    }
    if sum_all == 0.0 {
        return (f64::NAN, f64::NAN);
    }
    let cov_f1 = 1.0 / sum_all;
    let f1 = cov_f1 * erinv_y.iter().sum::<f64>();

    let mut alpha = theta0 - (f1 / rho0).atan();
    if alpha.cos() * theta0.cos() + alpha.sin() * theta0.sin() > 0.0 {
        alpha += std::f64::consts::PI;
    }

    let denom = rho0 * rho0 + f1 * f1;
    let dalpha_df1 = rho0 / denom;
    let dalpha_drho = -f1 / denom;
    let cov0_alpha = dalpha_df1 * dalpha_df1 * cov_f1 + dalpha_drho * dalpha_drho;

    (alpha, cov0_alpha)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A scan that passes ld_valid_fields: 100 rays over 180°, all valid,
    /// readings in (0, 100).
    fn valid_scan() -> LaserData {
        let mut ld = LaserData::new(100, -1.5, 1.5);
        for i in 0..100 {
            ld.valid[i] = true;
            ld.readings[i] = 5.0;
        }
        ld
    }

    #[test]
    fn validate_accepts_well_formed_scan() {
        assert!(valid_scan().validate().is_ok());
    }

    #[test]
    fn validate_rejects_too_few_rays() {
        // C: nrays < 10 rejected
        let ld = LaserData::new(5, -1.0, 1.0);
        assert!(matches!(
            ld.validate(),
            Err(LaserDataError::NraysOutOfRange)
        ));
    }

    #[test]
    fn validate_rejects_narrow_fov() {
        // C: FOV < 20 deg rejected (100 rays over 0.1 rad)
        let mut ld = LaserData::new(100, 0.0, 0.1);
        for i in 0..100 {
            ld.valid[i] = true;
            ld.readings[i] = 5.0;
        }
        assert!(matches!(ld.validate(), Err(LaserDataError::FovOutOfRange)));
    }

    #[test]
    fn validate_rejects_theta_bounds_mismatch() {
        // C: |min_theta - theta[0]| > 1e-8 rejected
        let mut ld = valid_scan();
        ld.min_theta = ld.theta[0] + 1e-6;
        assert!(matches!(
            ld.validate(),
            Err(LaserDataError::ThetaBoundsMismatch)
        ));
    }

    #[test]
    fn validate_rejects_nan_reading_on_valid_ray() {
        let mut ld = valid_scan();
        ld.readings[42] = f64::NAN;
        assert!(matches!(
            ld.validate(),
            Err(LaserDataError::BadValidRay(42))
        ));
    }

    #[test]
    fn validate_rejects_reading_outside_sensor_range() {
        // C: valid readings must be in (0, 100)
        let mut ld = valid_scan();
        ld.readings[7] = 150.0;
        assert!(matches!(ld.validate(), Err(LaserDataError::BadValidRay(7))));
    }

    #[test]
    fn validate_rejects_cluster_on_invalid_ray() {
        let mut ld = valid_scan();
        ld.valid[3] = false;
        ld.cluster[3] = 2;
        assert!(matches!(ld.validate(), Err(LaserDataError::BadCluster(3))));
    }

    #[test]
    fn validate_rejects_negative_sigma() {
        let mut ld = valid_scan();
        ld.readings_sigma[9] = -1.0;
        assert!(matches!(
            ld.validate(),
            Err(LaserDataError::NegativeSigma(9))
        ));
    }

    #[test]
    fn invalid_if_outside_marks_rays() {
        // C: ld_invalid_if_outside — valid rays with r <= min or r > max become invalid
        let mut ld = valid_scan();
        ld.readings[0] = 0.5; // below min
        ld.readings[1] = 5.0; // inside
        ld.readings[2] = 10.0; // above max (strictly greater)
        ld.readings[3] = 2.0; // boundary: equal to min -> invalid (r <= min)
        ld.readings[4] = 7.0; // boundary: equal to max -> stays valid
        ld.invalid_if_outside(2.0, 7.0);
        assert!(!ld.valid[0]);
        assert!(ld.valid[1]);
        assert!(!ld.valid[2]);
        assert!(!ld.valid[3]);
        assert!(ld.valid[4]);
    }

    #[test]
    fn compute_cartesian_fills_all_rays_polar_nan() {
        // C: ld_compute_cartesian — every ray, rho/phi set to NaN
        let mut ld = LaserData::new(10, 0.0, 1.0);
        for i in 0..10 {
            ld.valid[i] = true;
        }
        ld.readings[0] = 2.0;
        ld.theta[0] = 0.0;
        ld.compute_cartesian();
        assert!((ld.points[0].p[0] - 2.0).abs() < 1e-15);
        assert!((ld.points[0].p[1] - 0.0).abs() < 1e-15);
        assert!(ld.points[0].rho.is_nan() && ld.points[0].phi.is_nan());
        // NaN readings propagate to NaN points (C-faithful: no valid check)
        assert!(ld.points[5].p[0].is_nan());
    }

    #[test]
    fn compute_world_coords_transforms_valid_rays_only() {
        // C: ld_compute_world_coords — p transformed for valid rays only,
        // rho/phi computed for ALL rays
        let mut ld = LaserData::new(10, -1.0, 1.0);
        ld.valid[0] = true;
        ld.readings[0] = 2.0;
        ld.theta[0] = 0.0;
        ld.min_theta = ld.theta[0];
        ld.compute_cartesian();
        let pose = [1.0, 1.0, std::f64::consts::FRAC_PI_2];
        ld.compute_world_coords(&pose);
        // point (2,0) rotated by pi/2 then translated by (1,1) -> (1, 3)
        assert!((ld.points_w[0].p[0] - 1.0).abs() < 1e-12);
        assert!((ld.points_w[0].p[1] - 3.0).abs() < 1e-12);
        assert!((ld.points_w[0].rho - (10.0f64).sqrt()).abs() < 1e-12);
        assert!((ld.points_w[0].phi - 3.0f64.atan2(1.0)).abs() < 1e-12);
        // invalid ray: p untouched (stays NaN from alloc)
        assert!(ld.points_w[5].p[0].is_nan());
    }

    #[test]
    fn visibility_test_invalidates_bearing_reversals() {
        let mut ld = LaserData::new(10, -1.0, 1.0);
        for i in 0..ld.nrays {
            ld.valid[i] = true;
            let angle = if i == 5 { -1.0 } else { -1.0 + 0.2 * i as f64 };
            ld.points[i].p = [-angle.cos(), -angle.sin()];
        }

        ld.visibility_test(&[0.0, 0.0, 7.0]);

        assert!((0..10).filter(|&i| !ld.valid[i]).eq([5]));
    }

    #[test]
    fn jump_tables_hand_computed_example() {
        // Worked example, computed by hand from the C loops:
        // readings = [3, 1, 2, 2, 5, 4], all valid
        let mut ld = LaserData::new(6, -0.5, 0.5);
        for (i, &r) in [3.0, 1.0, 2.0, 2.0, 5.0, 4.0].iter().enumerate() {
            ld.valid[i] = true;
            ld.readings[i] = r;
        }
        ld.create_jump_tables();
        assert_eq!(ld.up_bigger, [4, 1, 2, 1, 2, 1]);
        assert_eq!(ld.up_smaller, [1, 5, 4, 3, 1, 1]);
        assert_eq!(ld.down_smaller, [-1, -2, -1, -2, -1, -2]);
        assert_eq!(ld.down_bigger, [-1, -1, -2, -3, -5, -1]);
    }

    #[test]
    fn jump_tables_stop_at_invalid_rays() {
        // C: the while conditions test ld->valid[j] first
        let mut ld = LaserData::new(10, -1.0, 1.0);
        for i in 0..10 {
            ld.valid[i] = true;
            ld.readings[i] = 5.0;
        }
        ld.valid[4] = false;
        ld.create_jump_tables();
        // From ray 0, up_bigger walks while readings[j] <= 5.0 — stops at j=4 (invalid)
        assert_eq!(ld.up_bigger[0], 4);
        // From ray 9, down_bigger walks while readings[j] <= 4.0+... stops at j=4
        assert_eq!(ld.down_bigger[9], 4 - 9);
    }

    #[test]
    fn simple_clustering_splits_on_jumps() {
        // Worked: readings [1, 1.01, 3, 3.02, 1], threshold 0.05 -> [0,0,1,1,2]
        let mut ld = LaserData::new(10, -1.0, 1.0);
        for (i, &r) in [1.0, 1.01, 3.0, 3.02, 1.0].iter().enumerate() {
            ld.valid[i] = true;
            ld.readings[i] = r;
        }
        ld.simple_clustering(0.05);
        assert_eq!(&ld.cluster[..5], &[0, 0, 1, 1, 2]);
        // untouched (invalid) rays get cluster -1
        assert!(ld.cluster[5..].iter().all(|&c| c == -1));
    }

    #[test]
    fn simple_clustering_bridges_invalid_rays() {
        // C-faithful: last_reading survives invalid rays, so [1, X, 1.01]
        // with threshold 0.05 stays one cluster: [0, -1, 0]
        let mut ld = LaserData::new(10, -1.0, 1.0);
        ld.valid[0] = true;
        ld.readings[0] = 1.0;
        ld.valid[2] = true;
        ld.readings[2] = 1.01;
        ld.simple_clustering(0.05);
        assert_eq!(&ld.cluster[..3], &[0, -1, 0]);
    }

    #[test]
    fn find_neighbours_walks_cluster_both_ways() {
        // Worked: 10 valid rays, one cluster, max_num=2, i=5
        // ups first (ascending), then downs (descending); the C down-walk
        // condition is checked before decrementing, so it yields max_num+1
        // downs: [6,7,4,3,2]
        let mut ld = LaserData::new(10, -1.0, 1.0);
        for i in 0..10 {
            ld.valid[i] = true;
            ld.readings[i] = 5.0;
            ld.cluster[i] = 0;
        }
        assert_eq!(ld.find_neighbours(5, 2), vec![6, 7, 4, 3, 2]);
        // boundary: i=0 has no downs
        assert_eq!(ld.find_neighbours(0, 2), vec![1, 2]);
    }

    #[test]
    fn find_neighbours_stops_at_cluster_change() {
        let mut ld = LaserData::new(10, -1.0, 1.0);
        for i in 0..10 {
            ld.valid[i] = true;
            ld.readings[i] = 5.0;
        }
        for i in 0..6 {
            ld.cluster[i] = 0;
        }
        for i in 6..10 {
            ld.cluster[i] = 1;
        }
        assert_eq!(ld.find_neighbours(5, 2), vec![4, 3, 2]);
    }

    #[test]
    fn compute_orientation_matches_c_reference() {
        // Ground truth from the C library: wall y=2, 181 rays over
        // [0.2, pi-0.2], one cluster, neighbourhood 3, sigma 0.01.
        let nrays = 181;
        let (tmin, tmax) = (0.2, std::f64::consts::PI - 0.2);
        let mut ld = LaserData::new(nrays, tmin, tmax);
        for i in 0..nrays {
            ld.valid[i] = true;
            ld.readings[i] = 2.0 / ld.theta[i].sin();
        }
        ld.simple_clustering(1000.0);
        ld.compute_orientation(3, 0.01);
        // (ray, alpha, cov_alpha) as printed by the C reference
        let golden: [(usize, f64, f64); 7] = [
            (0, 4.667_736_996_433_337, 2.982_699_741_012_538e-6),
            (30, 4.72744521612307, 0.00012508773117607194),
            (60, 4.7224561370844995, 0.00131686988590062),
            (90, 4.720_018_376_251_946, 0.002_565_544_549_570_049),
            (120, 4.720559304653337, 0.001_364_778_643_113_702),
            (150, 4.722188768107566, 0.00014216371217088818),
            (180, 4.772_113_047_180_021, 1.9109700429176555e-06),
        ];
        for (i, alpha, cov) in golden {
            assert!(ld.alpha_valid[i], "ray {i} should have valid alpha");
            assert!(
                (ld.alpha[i] - alpha).abs() < 1e-9,
                "ray {i}: alpha {} vs golden {}",
                ld.alpha[i],
                alpha
            );
            assert!(
                (ld.cov_alpha[i] - cov).abs() / cov < 1e-6,
                "ray {i}: cov_alpha {} vs golden {}",
                ld.cov_alpha[i],
                cov
            );
        }
    }

    #[test]
    fn validate_rejects_too_few_valid_rays() {
        // C: needs >= 10% valid rays
        let mut ld = LaserData::new(100, -1.5, 1.5);
        for i in 0..5 {
            ld.valid[i] = true;
            ld.readings[i] = 5.0;
        }
        assert!(matches!(
            ld.validate(),
            Err(LaserDataError::TooFewValidRays)
        ));
    }

    #[test]
    fn new_mirrors_ld_alloc_initialization() {
        let ld = LaserData::new(11, -1.0, 1.0);
        assert_eq!(ld.nrays, 11);
        // theta linearly spaced, endpoints exact (C invariant: theta[0] == min_theta)
        assert_eq!(ld.theta[0], -1.0);
        assert_eq!(ld.theta[10], 1.0);
        assert!((ld.theta[5] - 0.0).abs() < 1e-15);
        assert!(ld.valid.iter().all(|&v| !v));
        assert!(ld.readings.iter().all(|r| r.is_nan()));
        assert!(ld.cluster.iter().all(|&c| c == -1));
        assert!(ld.corr.iter().all(|c| !c.valid && c.j1 == -1 && c.j2 == -1));
        assert!(ld.points.iter().all(|p| p.p[0].is_nan() && p.rho.is_nan()));
        assert!(ld.odometry.iter().all(|v| v.is_nan()));
    }
}
