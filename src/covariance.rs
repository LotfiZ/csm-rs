//! Closed-form estimate of ICP's matching covariance.
//!
//! Method: Censi, "An accurate closed-form estimate of ICP's covariance"
//! (ICRA 2007). The exact calculation returns the unscaled covariance and the
//! two input-derivative matrices. Only the covariance is scaled by
//! `params.sigma²` before storing the public result.

use crate::math::{Mat3, Matrix};
use crate::scan_data::ScanData;

/// The unscaled covariance and the Hessian of the point-to-line objective,
/// which is the match's Fisher information.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ExactCovariance {
    pub cov0_x: Mat3,
    pub fisher: Mat3,
}

/// Compute the exact closed-form covariance before the final `sigma²`
/// scaling.
///
/// The calculation uses the correspondence set currently stored on
/// `laser_sens`. This is deliberate: after a restart the implementation
/// recomputes that set at the selected pose and then passes it directly to
/// `compute_covariance_exact()`.
///
pub(crate) fn compute_covariance_exact_into(
    laser_ref: &ScanData,
    laser_sens: &ScanData,
    x: [f64; 3],
    d2j_dxdy1: &mut Matrix,
    d2j_dxdy2: &mut Matrix,
) -> Option<ExactCovariance> {
    d2j_dxdy1.reset(3, laser_ref.nrays);
    d2j_dxdy2.reset(3, laser_sens.nrays);

    // The Hessian d²J/dx², accumulated in three blocks.
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

        let v1 = rotate(p_i, theta + std::f64::consts::FRAC_PI_2);
        let rotated_p_i = rotate(p_i, theta);
        let v2 = [
            rotated_p_i[0] + translation[0] - p_j1[0],
            rotated_p_i[1] + translation[1] - p_j1[1],
        ];
        let v3 = vers(theta + laser_sens.theta[i]);
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

        let v_new = rotate(p_i, theta + std::f64::consts::PI);
        let d2j_dtheta2_k = 2.0 * (quadratic(v2, c_k, v_new) + quadratic(v1, c_k, v1));
        d2j_dtheta2 += d2j_dtheta2_k;

        // Derivatives with respect to the sensor range rho_i.
        let d2jk_dtdrho_i = scale_vec2(mat2_vec(c_k, v3), 2.0);
        let d2jk_dtheta_drho_i = 2.0 * (quadratic(v2, c_k, v4) + quadratic(v3, c_k, v1));
        add_column(
            d2j_dxdy2,
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
            d2j_dxdy1,
            j1,
            [d2jk_dt_drho_j1[0], d2jk_dt_drho_j1[1], d2jk_dtheta_drho_j1],
        );

        let d2jk_dt_drho_j2 = scale_vec2(mat2_vec(d_c_drho_j2, v2), 2.0);
        let d2jk_dtheta_drho_j2 = 2.0 * quadratic(v2, d_c_drho_j2, v1);
        add_column(
            d2j_dxdy1,
            j2,
            [d2jk_dt_drho_j2[0], d2jk_dt_drho_j2[1], d2jk_dtheta_drho_j2],
        );
    }

    let d2j_dx2 = Mat3::new([
        [d2j_dt2[0][0], d2j_dt2[0][1], d2j_dt_dtheta[0]],
        [d2j_dt2[1][0], d2j_dt2[1][1], d2j_dt_dtheta[1]],
        [d2j_dt_dtheta[0], d2j_dt_dtheta[1], d2j_dtheta2],
    ]);
    let inverse = d2j_dx2.inv()?;
    negative_left_multiply_in_place(&inverse, d2j_dxdy1);
    negative_left_multiply_in_place(&inverse, d2j_dxdy2);

    let mut cov0_x = [[0.0; 3]; 3];
    for (row, values) in cov0_x.iter_mut().enumerate() {
        for (col, value) in values.iter_mut().enumerate() {
            let dy1: f64 = (0..d2j_dxdy1.cols())
                .map(|k| d2j_dxdy1.data[row][k] * d2j_dxdy1.data[col][k])
                .sum();
            let dy2: f64 = (0..d2j_dxdy2.cols())
                .map(|k| d2j_dxdy2.data[row][k] * d2j_dxdy2.data[col][k])
                .sum();
            *value = dy1 + dy2;
        }
    }

    Some(ExactCovariance {
        cov0_x: Mat3::new(cov0_x),
        fisher: d2j_dx2,
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

fn negative_left_multiply_in_place(left: &Mat3, matrix: &mut Matrix) {
    for col in 0..matrix.cols() {
        let v = [
            matrix.data[0][col],
            matrix.data[1][col],
            matrix.data[2][col],
        ];
        for row in 0..3 {
            matrix.data[row][col] =
                -(left.data[row][0] * v[0] + left.data[row][1] * v[1] + left.data[row][2] * v[2]);
        }
    }
}

#[cfg(test)]
mod failure_tests {
    use super::*;
    use crate::scan_data::{Correspondence, CorrespondenceType, Point};

    fn point(x: f64, y: f64) -> Point {
        Point {
            p: [x, y],
            rho: f64::NAN,
            phi: f64::NAN,
        }
    }

    #[test]
    fn singular_correspondence_geometry_returns_no_covariance() {
        // All points lie on the vertical line x = 8, so every point-to-line
        // normal is horizontal and the translation information along x is
        // empty; the Hessian is singular.
        let n = 3;
        let mut scan = ScanData::new(n, -1.0, 1.0);
        for i in 0..n {
            let y = i as f64 - 1.0;
            scan.valid[i] = true;
            scan.theta[i] = 0.0;
            scan.readings[i] = 8.0;
            scan.points[i] = point(8.0, y);
            scan.corr[i] = Correspondence {
                valid: true,
                j1: i as i32,
                j2: i as i32,
                corr_type: CorrespondenceType::PointToLine,
                dist2_j1: 0.0,
            };
        }
        let mut dx_dy1 = Matrix::zeros(0, 0);
        let mut dx_dy2 = Matrix::zeros(0, 0);
        let result =
            compute_covariance_exact_into(&scan, &scan, [0.0; 3], &mut dx_dy1, &mut dx_dy2);
        assert!(
            result.is_none(),
            "singular geometry must not yield a covariance"
        );
    }
}
