//! Closed-form estimate of ICP's matching covariance.
//!
//! C: `sm/csm/icp/icp_covariance.c` (`compute_covariance_exact`),
//!     `sm/csm/laser_data_fisher.c` (`ld_fisher0`)
//!
//! Method: Censi, "An accurate closed-form estimate of ICP's covariance"
//! (ICRA 2007). The exact calculation returns the unscaled covariance and
//! the two input-derivative matrices. CSM scales only the covariance by
//! `params.sigma²` before storing the public result.

use crate::laser_data::LaserData;
use crate::math::{Mat3, Matrix};

/// The unscaled outputs of CSM's exact covariance calculation.
///
/// C: the `cov0_x`, `dx_dy1`, and `dx_dy2` outputs of
/// `compute_covariance_exact()`.
#[derive(Clone, Debug)]
pub(crate) struct ExactCovariance {
    pub cov0_x: Mat3,
    pub dx_dy1: Matrix,
    pub dx_dy2: Matrix,
}

/// Compute the Fisher information matrix for one scan.
///
/// The matrix is expressed in robot coordinates and uses `true_alpha` only;
/// rays with a NaN `true_alpha` are skipped, matching CSM's
/// `ld_fisher0()`. This helper is public because it is useful to callers that
/// want the scan-only information matrix alongside ICP's matching covariance.
///
/// C: `sm/csm/laser_data_fisher.c:ld_fisher0()`
#[allow(dead_code)] // Exposed by the optional-uncertainty public API (see epic #35).
pub fn fisher0(laser: &LaserData) -> Mat3 {
    let mut fim = [[0.0; 3]; 3];
    for i in 0..laser.nrays {
        let alpha = laser.true_alpha[i];
        if alpha.is_nan() {
            continue;
        }

        let theta = laser.theta[i];
        let beta = alpha - theta;
        let reading = laser.readings[i];
        let c = alpha.cos();
        let s = alpha.sin();
        let z = 1.0 / beta.cos();
        let tangent = beta.tan();

        let values = [
            c * c * z * z,
            c * s * z * z,
            c * z * tangent * reading,
            c * s * z * z,
            s * s * z * z,
            s * z * tangent * reading,
            c * z * tangent * reading,
            s * z * tangent * reading,
            tangent * reading * tangent * reading,
        ];
        for row in 0..3 {
            for col in 0..3 {
                fim[row][col] += values[row * 3 + col];
            }
        }
    }
    Mat3::new(fim)
}

/// Compute CSM's exact closed-form covariance before the final `sigma²`
/// scaling.
///
/// The calculation uses the correspondence set currently stored on
/// `laser_sens`. This is deliberate: after a restart the C implementation
/// recomputes that set at the selected pose and then passes it directly to
/// `compute_covariance_exact()`.
///
/// C: `sm/csm/icp/icp_covariance.c:compute_covariance_exact()`
pub(crate) fn compute_covariance_exact(
    laser_ref: &LaserData,
    laser_sens: &LaserData,
    x: [f64; 3],
) -> Option<ExactCovariance> {
    let mut d2j_dxdy1 = Matrix::zeros(3, laser_ref.nrays);
    let mut d2j_dxdy2 = Matrix::zeros(3, laser_sens.nrays);

    // The Hessian d²J/dx², accumulated as the three pieces used by C.
    let mut d2j_dt2 = [[0.0; 2]; 2];
    let mut d2j_dt_dtheta = [0.0; 2];
    let mut d2j_dtheta2 = 0.0;

    let theta = x[2];
    let translation = [x[0], x[1]];

    for i in 0..laser_sens.nrays {
        if !laser_sens.corr[i].valid {
            continue;
        }

        let j1 = usize::try_from(laser_sens.corr[i].j1).ok()?;
        let j2 = usize::try_from(laser_sens.corr[i].j2).ok()?;
        if j1 >= laser_ref.nrays || j2 >= laser_ref.nrays {
            return None;
        }

        let p_i = laser_sens.points[i].p;
        let p_j1 = laser_ref.points[j1].p;
        let p_j2 = laser_ref.points[j2].p;

        // C: v1 := rot(theta + M_PI/2) * p_i.
        let v1 = rotate(p_i, theta + std::f64::consts::FRAC_PI_2);
        // C: v2 := rot(theta) * p_i + t - p_j1.
        let rotated_p_i = rotate(p_i, theta);
        let v2 = [
            rotated_p_i[0] + translation[0] - p_j1[0],
            rotated_p_i[1] + translation[1] - p_j1[1],
        ];
        // C: v3 := vers(theta + laser_sens->theta[i]).
        let v3 = vers(theta + laser_sens.theta[i]);
        // C: v4 := vers(theta + laser_sens->theta[i] + M_PI/2).
        let v4 = vers(theta + laser_sens.theta[i] + std::f64::consts::FRAC_PI_2);

        let c_k = compute_c_k(p_j1, p_j2);

        let c_v1 = mat2_vec(c_k, v1);
        for row in 0..2 {
            for col in 0..2 {
                d2j_dt2[row][col] += 2.0 * c_k[row][col];
            }
        }
        d2j_dt_dtheta[0] += 2.0 * c_v1[0];
        d2j_dt_dtheta[1] += 2.0 * c_v1[1];

        // C: v_new := rot(theta + M_PI) * p_i.
        let v_new = rotate(p_i, theta + std::f64::consts::PI);
        let d2j_dtheta2_k = 2.0 * (quadratic(v2, c_k, v_new) + quadratic(v1, c_k, v1));
        d2j_dtheta2 += d2j_dtheta2_k;

        // Derivatives with respect to the sensor range rho_i.
        let d2jk_dtdrho_i = scale_vec2(mat2_vec(c_k, v3), 2.0);
        let d2jk_dtheta_drho_i = 2.0 * (quadratic(v2, c_k, v4) + quadratic(v3, c_k, v1));
        add_column(
            &mut d2j_dxdy2,
            i,
            [d2jk_dtdrho_i[0], d2jk_dtdrho_i[1], d2jk_dtheta_drho_i],
        );

        // Derivatives with respect to the two reference ranges.
        let d_c_drho_j1 = d_c_drho(p_j1, p_j2);
        let d_c_drho_j2 = d_c_drho(p_j2, p_j1);
        let v_j1 = vers(laser_ref.theta[j1]);

        let d2jk_dt_drho_j1 = add_vec2(
            scale_vec2(mat2_vec(c_k, v_j1), -2.0),
            scale_vec2(mat2_vec(d_c_drho_j1, v2), 2.0),
        );
        let d2jk_dtheta_drho_j1 = -2.0 * quadratic(v_j1, c_k, v1) + quadratic(v2, d_c_drho_j1, v1);
        add_column(
            &mut d2j_dxdy1,
            j1,
            [d2jk_dt_drho_j1[0], d2jk_dt_drho_j1[1], d2jk_dtheta_drho_j1],
        );

        let d2jk_dt_drho_j2 = scale_vec2(mat2_vec(d_c_drho_j2, v2), 2.0);
        let d2jk_dtheta_drho_j2 = 2.0 * quadratic(v2, d_c_drho_j2, v1);
        add_column(
            &mut d2j_dxdy1,
            j2,
            [d2jk_dt_drho_j2[0], d2jk_dt_drho_j2[1], d2jk_dtheta_drho_j2],
        );
    }

    // C: compose d²J/dx² from its translation and rotation blocks.
    let d2j_dx2 = Mat3::new([
        [d2j_dt2[0][0], d2j_dt2[0][1], d2j_dt_dtheta[0]],
        [d2j_dt2[1][0], d2j_dt2[1][1], d2j_dt_dtheta[1]],
        [d2j_dt_dtheta[0], d2j_dt_dtheta[1], d2j_dtheta2],
    ]);
    let inverse = d2j_dx2.inv()?;
    let edx_dy1 = negative_left_multiply(&inverse, &d2j_dxdy1);
    let edx_dy2 = negative_left_multiply(&inverse, &d2j_dxdy2);

    let mut cov0_x = [[0.0; 3]; 3];
    for (row, values) in cov0_x.iter_mut().enumerate() {
        for (col, value) in values.iter_mut().enumerate() {
            let dy1: f64 = (0..edx_dy1.cols())
                .map(|k| edx_dy1.data[row][k] * edx_dy1.data[col][k])
                .sum();
            let dy2: f64 = (0..edx_dy2.cols())
                .map(|k| edx_dy2.data[row][k] * edx_dy2.data[col][k])
                .sum();
            *value = dy1 + dy2;
        }
    }

    Some(ExactCovariance {
        cov0_x: Mat3::new(cov0_x),
        dx_dy1: edx_dy1,
        dx_dy2: edx_dy2,
    })
}

fn rotate(point: [f64; 2], angle: f64) -> [f64; 2] {
    let c = angle.cos();
    let s = angle.sin();
    [c * point[0] - s * point[1], s * point[0] + c * point[1]]
}

fn vers(angle: f64) -> [f64; 2] {
    [angle.cos(), angle.sin()]
}

fn compute_c_k(p1: [f64; 2], p2: [f64; 2]) -> [[f64; 2]; 2] {
    let d = [p1[0] - p2[0], p1[1] - p2[1]];
    let alpha = std::f64::consts::FRAC_PI_2 + d[1].atan2(d[0]);
    let c = alpha.cos();
    let s = alpha.sin();
    [[c * c, c * s], [c * s, s * s]]
}

/// C: `dC_drho()` in `sm/csm/icp/icp_covariance.c`.
fn d_c_drho(p1: [f64; 2], p2: [f64; 2]) -> [[f64; 2]; 2] {
    let eps = 0.001;
    let c_k = compute_c_k(p1, p2);
    let norm = (p1[0] * p1[0] + p1[1] * p1[1]).sqrt();
    let scale = eps / norm;
    let p1b = [p1[0] + scale * p1[0], p1[1] + scale * p1[1]];
    let c_k_eps = compute_c_k(p1b, p2);
    [
        [
            (c_k_eps[0][0] - c_k[0][0]) / eps,
            (c_k_eps[0][1] - c_k[0][1]) / eps,
        ],
        [
            (c_k_eps[1][0] - c_k[1][0]) / eps,
            (c_k_eps[1][1] - c_k[1][1]) / eps,
        ],
    ]
}

fn mat2_vec(matrix: [[f64; 2]; 2], vector: [f64; 2]) -> [f64; 2] {
    [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1],
    ]
}

fn quadratic(left: [f64; 2], matrix: [[f64; 2]; 2], right: [f64; 2]) -> f64 {
    let matrix_right = mat2_vec(matrix, right);
    left[0] * matrix_right[0] + left[1] * matrix_right[1]
}

fn scale_vec2(vector: [f64; 2], scale: f64) -> [f64; 2] {
    [scale * vector[0], scale * vector[1]]
}

fn add_vec2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] + b[0], a[1] + b[1]]
}

fn add_column(matrix: &mut Matrix, column: usize, values: [f64; 3]) {
    for (row, value) in values.into_iter().enumerate() {
        matrix.data[row][column] += value;
    }
}

fn negative_left_multiply(left: &Mat3, right: &Matrix) -> Matrix {
    let mut result = Matrix::zeros(3, right.cols());
    for row in 0..3 {
        for col in 0..right.cols() {
            let value = left.data[row][0] * right.data[0][col]
                + left.data[row][1] * right.data[1][col]
                + left.data[row][2] * right.data[2][col];
            result.data[row][col] = -value;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fisher0_matches_one_ray_information_contribution() {
        let mut laser = LaserData::new(10, -1.0, 1.0);
        laser.theta[0] = 0.1;
        laser.readings[0] = 2.0;
        laser.true_alpha[0] = 0.4;

        let beta: f64 = 0.3;
        let c = 0.4f64.cos();
        let s = 0.4f64.sin();
        let z = 1.0 / beta.cos();
        let tangent = beta.tan();
        let expected = [
            [c * c * z * z, c * s * z * z, c * z * tangent * 2.0],
            [c * s * z * z, s * s * z * z, s * z * tangent * 2.0],
            [
                c * z * tangent * 2.0,
                s * z * tangent * 2.0,
                tangent * 2.0 * tangent * 2.0,
            ],
        ];
        let actual = fisher0(&laser);
        for (row, values) in expected.iter().enumerate() {
            for (col, expected) in values.iter().enumerate() {
                assert!((actual.data[row][col] - expected).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn fisher0_skips_rays_without_true_alpha() {
        let mut laser = LaserData::new(10, -1.0, 1.0);
        laser.readings[0] = 2.0;
        assert_eq!(fisher0(&laser), Mat3::new([[0.0; 3]; 3]));
    }
}
