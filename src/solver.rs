//! Closed-form weighted point-correspondence solver.
//!
//! C: `sm/lib/gpc/gpc.c` (`gpc_solve`, `gpc_total_error`),
//!     `sm/lib/gpc/gpc_utils.c`,
//!     `sm/csm/icp/icp_loop.c` (`compute_next_estimate`)
//!
//! Solves the general point-correspondence problem: find translation `t` and
//! rotation `θ` minimizing Σ (R(θ)p + t − q)′ C (R(θ)p + t − q) over the
//! valid correspondences. Closed form via 4×4 normal equations with 2×2
//! block inverses — hand-rolled small matrices per grilling Q3.
//!
//! Note: upstream `gpc.c` is GPLv2+ (the rest of CSM is LGPLv3); this module
//! is the GPL contaminant and the candidate for a future clean-room rewrite
//! if copyleft ever blocks a use case (grilling Q13).

use crate::laser_data::{CorrespondenceType, LaserData};
use crate::math::{projection_on_segment, Mat2, Mat4};
use crate::params::Params;

/// One weighted point correspondence in the form consumed by GPC.
///
/// C: `struct gpc_corr` in `sm/lib/gpc/gpc.h`
#[derive(Clone, Copy, Debug)]
pub(crate) struct GpcCorrespondence {
    pub p: [f64; 2],
    pub q: [f64; 2],
    pub c: [[f64; 2]; 2],
    pub valid: bool,
}

/// Build GPC correspondences and solve for the next ICP pose.
///
/// C: `compute_next_estimate()` in `sm/csm/icp/icp_loop.c`
#[allow(dead_code)]
pub(crate) fn compute_next_estimate(
    params: &Params,
    laser_ref: &LaserData,
    laser_sens: &LaserData,
    x_old: [f64; 3],
) -> Option<[f64; 3]> {
    let mut correspondences = Vec::new();
    compute_next_estimate_with_scratch(params, laser_ref, laser_sens, x_old, &mut correspondences)
}

pub(crate) fn compute_next_estimate_with_scratch(
    params: &Params,
    laser_ref: &LaserData,
    laser_sens: &LaserData,
    x_old: [f64; 3],
    correspondences: &mut Vec<GpcCorrespondence>,
) -> Option<[f64; 3]> {
    correspondences.clear();

    for i in 0..laser_sens.nrays {
        if !laser_sens.valid[i] || !laser_sens.corr[i].valid {
            continue;
        }

        let j1 = usize::try_from(laser_sens.corr[i].j1).ok()?;
        let j2 = usize::try_from(laser_sens.corr[i].j2).ok()?;
        if j1 >= laser_ref.nrays || j2 >= laser_ref.nrays {
            return None;
        }

        let mut correspondence = match laser_sens.corr[i].corr_type {
            CorrespondenceType::PointToLine => {
                let p = laser_sens.points[i].p;
                let q = laser_ref.points[j1].p;
                let diff = [
                    laser_ref.points[j1].p[0] - laser_ref.points[j2].p[0],
                    laser_ref.points[j1].p[1] - laser_ref.points[j2].p[1],
                ];
                let one_on_norm = 1.0 / (diff[0] * diff[0] + diff[1] * diff[1]).sqrt();
                let cos_alpha = diff[1] * one_on_norm;
                let sin_alpha = -diff[0] * one_on_norm;
                GpcCorrespondence {
                    p,
                    q,
                    c: [
                        [cos_alpha * cos_alpha, cos_alpha * sin_alpha],
                        [cos_alpha * sin_alpha, sin_alpha * sin_alpha],
                    ],
                    valid: true,
                }
            }
            CorrespondenceType::PointToPoint => GpcCorrespondence {
                p: laser_sens.points[i].p,
                q: projection_on_segment(
                    laser_ref.points[j1].p,
                    laser_ref.points[j2].p,
                    laser_sens.points_w[i].p,
                ),
                c: [[1.0, 0.0], [0.0, 1.0]],
                valid: true,
            },
        };

        let mut factor = 1.0;

        // C: `use_ml_weights` branch in `compute_next_estimate()`.
        if params.weights.ml {
            let alpha = if !laser_ref.true_alpha[j1].is_nan() {
                Some(laser_ref.true_alpha[j1])
            } else if laser_ref.alpha_valid[j1] {
                Some(laser_ref.alpha[j1])
            } else {
                None
            };
            if let Some(alpha) = alpha {
                let beta = alpha - (x_old[2] + laser_sens.theta[i]);
                let cosine = beta.cos();
                factor *= 1.0 / (cosine * cosine);
            }
        }

        // C: `use_sigma_weights` branch in `compute_next_estimate()`.
        if params.weights.sigma {
            let sigma = laser_sens.readings_sigma[i];
            if !sigma.is_nan() {
                factor *= 1.0 / (sigma * sigma);
            }
        }

        for row in &mut correspondence.c {
            for value in row {
                *value *= factor;
            }
        }
        correspondences.push(correspondence);
    }

    let x_new = gpc_solve(correspondences)?;
    // C computes both values for its diagnostic check. Keep the same
    // observable arithmetic even though this tracer has no logging surface.
    let _old_error = gpc_total_error(correspondences, x_old);
    let _new_error = gpc_total_error(correspondences, x_new);
    Some(x_new)
}

/// Solve the weighted point-correspondence problem.
///
/// C: `gpc_solve()` in `sm/lib/gpc/gpc.c`
pub(crate) fn gpc_solve(correspondences: &[GpcCorrespondence]) -> Option<[f64; 3]> {
    let (big_m, g) = normal_equations(correspondences);

    let m_a = Mat2::new([[big_m[0][0], big_m[0][1]], [big_m[1][0], big_m[1][1]]]);
    let m_b = Mat2::new([[big_m[0][2], big_m[0][3]], [big_m[1][2], big_m[1][3]]]);
    let m_d = Mat2::new([[big_m[2][2], big_m[2][3]], [big_m[3][2], big_m[3][3]]]);

    let m_ai = m_a.inv()?;
    // Keep the multiplication grouping used by GPC: inv(A)·B is formed
    // first, then Bᵀ·(inv(A)·B). The weighted path is sensitive to changing
    // this floating-point evaluation order.
    let m_ai_b = m_ai.mul(&m_b);
    let m_b_transpose_ai_b = m_b.transpose().mul(&m_ai_b);
    let m_s = mat2_sub(&m_d, &m_b_transpose_ai_b);
    let m_s_det = mat2_det(&m_s);
    let m_sa = mat2_scale(&m_s.inv()?, m_s_det);

    let g1 = [g[0], g[1]];
    let g2 = [g[2], g[3]];
    // C: g1ᵀ·inv(A), followed by ·B (rather than g1ᵀ·(inv(A)·B)).
    let g1t_m_ai = row_mul_mat2(&g1, &m_ai);
    let m1t = row_mul_mat2(&g1t_m_ai, &m_b);
    let m2t = row_mul_mat2(&m1t, &m_sa);
    let m3t = row_mul_mat2(&g2, &m_sa);

    let p = [
        dot2(&m2t, &m2t) - 2.0 * dot2(&m2t, &m3t) + dot2(&m3t, &m3t),
        4.0 * dot2(&m2t, &m1t) - 8.0 * dot2(&m2t, &g2) + 4.0 * dot2(&g2, &m3t),
        4.0 * dot2(&m1t, &m1t) - 8.0 * dot2(&m1t, &g2) + 4.0 * dot2(&g2, &g2),
    ];
    let l = [m_s_det, 2.0 * m_s.data[0][0] + 2.0 * m_s.data[1][1], 4.0];
    let q = [
        p[0] - l[0] * l[0],
        p[1] - 2.0 * l[1] * l[0],
        p[2] - (l[1] * l[1] + 2.0 * l[0] * l[2]),
        -(2.0 * l[2] * l[1]),
        -(l[2] * l[2]),
    ];

    let lambda = greatest_real_root(&q)?;
    let mut system = big_m;
    system[2][2] += 2.0 * lambda;
    system[3][3] += 2.0 * lambda;
    let mut x = Mat4::new(system).solve(g)?;
    for value in &mut x {
        *value = -*value;
    }
    Some([x[0], x[1], x[3].atan2(x[2])])
}

/// Assemble the normal equations used by GPC.
///
/// C: the `d_bigM` and `d_g` accumulation in `sm/lib/gpc/gpc.c:gpc_solve()`
fn normal_equations(correspondences: &[GpcCorrespondence]) -> ([[f64; 4]; 4], [f64; 4]) {
    let mut d_big_m = [[0.0; 4]; 4];
    let mut d_g = [0.0; 4];

    for correspondence in correspondences {
        if !correspondence.valid {
            continue;
        }
        let c00 = correspondence.c[0][0];
        let c01 = correspondence.c[0][1];
        let c10 = correspondence.c[1][0];
        let c11 = correspondence.c[1][1];
        let qx = correspondence.q[0];
        let qy = correspondence.q[1];
        let px = correspondence.p[0];
        let py = correspondence.p[1];

        d_big_m[0][0] += c00;
        d_big_m[0][1] += c01;
        d_big_m[0][2] += px * c00 + py * c01;
        d_big_m[0][3] += -py * c00 + px * c01;

        d_big_m[1][0] += c10;
        d_big_m[1][1] += c11;
        d_big_m[1][2] += px * c10 + py * c11;
        d_big_m[1][3] += px * c11 - py * c10;

        d_big_m[2][0] += px * c00 + py * c10;
        d_big_m[2][1] += px * c01 + py * c11;
        d_big_m[2][2] += px * px * c00 + px * py * (c10 + c01) + py * py * c11;
        d_big_m[2][3] += px * px * c01 + px * py * (-c00 + c11) - py * py * c10;

        d_big_m[3][0] += -py * c00 + px * c10;
        d_big_m[3][1] += -py * c01 + px * c11;
        d_big_m[3][2] += px * px * c10 + px * py * (-c00 + c11) - py * py * c01;
        d_big_m[3][3] += px * px * c11 + px * py * (-c10 - c01) + py * py * c00;

        d_g[0] += c00 * qx + c10 * qy;
        d_g[1] += c01 * qx + c11 * qy;
        d_g[2] += qx * (c00 * px + c01 * py) + qy * (c10 * px + c11 * py);
        d_g[3] += qx * (-c00 * py + c01 * px) + qy * (-c10 * py + c11 * px);
    }

    let big_m = d_big_m.map(|row| row.map(|value| 2.0 * value));
    let g = d_g.map(|value| -2.0 * value);
    (big_m, g)
}

/// Weighted squared error for one GPC correspondence.
///
/// C: `gpc_error()` in `sm/lib/gpc/gpc.c`
pub(crate) fn gpc_error(correspondence: &GpcCorrespondence, pose: [f64; 3]) -> f64 {
    let cosine = pose[2].cos();
    let sine = pose[2].sin();
    let e0 =
        cosine * correspondence.p[0] - sine * correspondence.p[1] + pose[0] - correspondence.q[0];
    let e1 =
        sine * correspondence.p[0] + cosine * correspondence.p[1] + pose[1] - correspondence.q[1];
    e0 * e0 * correspondence.c[0][0]
        + 2.0 * e0 * e1 * correspondence.c[0][1]
        + e1 * e1 * correspondence.c[1][1]
}

/// Total weighted squared error for a GPC correspondence list.
///
/// C: `gpc_total_error()` in `sm/lib/gpc/gpc.c`
pub(crate) fn gpc_total_error(correspondences: &[GpcCorrespondence], pose: [f64; 3]) -> f64 {
    correspondences
        .iter()
        .filter(|correspondence| correspondence.valid)
        .map(|correspondence| gpc_error(correspondence, pose))
        .sum()
}

fn mat2_det(matrix: &Mat2) -> f64 {
    matrix.data[0][0] * matrix.data[1][1] - matrix.data[0][1] * matrix.data[1][0]
}

fn mat2_sub(a: &Mat2, b: &Mat2) -> Mat2 {
    Mat2::new([
        [a.data[0][0] - b.data[0][0], a.data[0][1] - b.data[0][1]],
        [a.data[1][0] - b.data[1][0], a.data[1][1] - b.data[1][1]],
    ])
}

fn mat2_scale(matrix: &Mat2, scale: f64) -> Mat2 {
    Mat2::new([
        [matrix.data[0][0] * scale, matrix.data[0][1] * scale],
        [matrix.data[1][0] * scale, matrix.data[1][1] * scale],
    ])
}

fn row_mul_mat2(row: &[f64; 2], matrix: &Mat2) -> [f64; 2] {
    [
        row[0] * matrix.data[0][0] + row[1] * matrix.data[1][0],
        row[0] * matrix.data[0][1] + row[1] * matrix.data[1][1],
    ]
}

fn dot2(a: &[f64; 2], b: &[f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

/// Greatest real root of the polynomial whose coefficients are ascending in
/// degree. This replaces GSL's `gsl_poly_complex_solve()` for the quartic
/// generated by GPC.
///
/// C: `poly_greatest_real_root()` in `sm/lib/gpc/gpc_utils.c`
fn greatest_real_root(coefficients: &[f64]) -> Option<f64> {
    let roots = real_roots(coefficients);
    roots.values[..roots.len]
        .iter()
        .copied()
        .max_by(|a, b| a.partial_cmp(b).unwrap())
}

/// Fixed-capacity real-root set. GPC's polynomial is a quartic, so eight slots
/// cover every root of the polynomial and of its recursion derivatives without
/// heap allocation in the prepared matching path.
const MAX_ROOTS: usize = 64;

#[derive(Clone, Copy)]
struct Roots {
    values: [f64; MAX_ROOTS],
    len: usize,
}

impl Roots {
    fn new() -> Self {
        Self {
            values: [0.0; MAX_ROOTS],
            len: 0,
        }
    }

    fn push(&mut self, value: f64) {
        if self.len < MAX_ROOTS {
            self.values[self.len] = value;
            self.len += 1;
        }
    }

    /// Sort ascending and drop near-duplicates within `tolerance`.
    fn sort_and_dedup(&mut self, tolerance: f64) {
        let slice = &mut self.values[..self.len];
        slice.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut write = 0;
        for read in 0..self.len {
            let distinct = write == 0
                || (slice[read] - slice[write - 1]).abs()
                    > tolerance * slice[read].abs().max(slice[write - 1].abs()).max(1.0);
            if distinct {
                slice[write] = slice[read];
                write += 1;
            }
        }
        self.len = write;
    }
}

/// Real roots of a polynomial with coefficients ascending in degree.
///
/// Allocation-free: the quartic GPC polynomial bounds the recursion depth, so
/// stack arrays cover every intermediate polynomial.
fn real_roots(coefficients: &[f64]) -> Roots {
    roots_impl(coefficients)
}

/// Recursive root finder using stack-only storage.
fn roots_impl(coefficients: &[f64]) -> Roots {
    let mut coeffs = [0.0f64; MAX_ROOTS];
    let mut n = coefficients.len().min(MAX_ROOTS);
    coeffs[..n].copy_from_slice(&coefficients[..n]);
    while n > 1 && coeffs[n - 1] == 0.0 {
        n -= 1;
    }
    let degree = n.saturating_sub(1);
    if degree == 0 {
        return Roots::new();
    }
    if degree == 1 {
        let mut result = Roots::new();
        if coeffs[1] != 0.0 {
            result.push(-coeffs[0] / coeffs[1]);
        }
        return result;
    }

    let mut derivative = [0.0f64; MAX_ROOTS];
    for i in 1..n {
        derivative[i - 1] = i as f64 * coeffs[i];
    }
    let bound = cauchy_bound(&coeffs[..n]);
    let derivative_roots = roots_impl(&derivative[..n - 1]);

    let mut critical = Roots::new();
    for root in derivative_roots.values[..derivative_roots.len].iter().copied() {
        if root.is_finite() && root > -bound && root < bound {
            critical.push(root);
        }
    }
    critical.sort_and_dedup(1e-14);

    let mut points = [0.0f64; MAX_ROOTS + 2];
    let mut point_count = 0;
    points[point_count] = -bound;
    point_count += 1;
    for root in critical.values[..critical.len].iter().copied() {
        points[point_count] = root;
        point_count += 1;
    }
    points[point_count] = bound;
    point_count += 1;

    let function_scale = coeffs[..n]
        .iter()
        .enumerate()
        .map(|(degree, coefficient)| coefficient.abs() * bound.powi(degree as i32))
        .sum::<f64>()
        .max(1.0);
    let function_tolerance = 1e-12 * function_scale;

    let mut roots = Roots::new();
    for point in critical.values[..critical.len].iter().copied() {
        if poly_eval(&coeffs[..n], point).abs() <= function_tolerance {
            roots.push(point);
        }
    }

    for window in points[..point_count].windows(2) {
        let mut left = window[0];
        let mut right = window[1];
        let mut left_value = poly_eval(&coeffs[..n], left);
        let right_value = poly_eval(&coeffs[..n], right);
        if left_value == 0.0 {
            roots.push(left);
            continue;
        }
        if right_value == 0.0 {
            roots.push(right);
            continue;
        }
        if !left_value.is_finite()
            || !right_value.is_finite()
            || left_value.signum() == right_value.signum()
        {
            continue;
        }

        for _ in 0..120 {
            let middle = 0.5 * (left + right);
            if middle == left || middle == right {
                break;
            }
            let middle_value = poly_eval(&coeffs[..n], middle);
            if middle_value == 0.0 {
                left = middle;
                right = middle;
                break;
            }
            if middle_value.signum() == left_value.signum() {
                left = middle;
                left_value = middle_value;
            } else {
                right = middle;
            }
            if (right - left).abs() <= 1e-15 * middle.abs().max(1.0) {
                break;
            }
        }
        roots.push(0.5 * (left + right));
    }

    roots.sort_and_dedup(1e-12);
    roots
}

fn cauchy_bound(coefficients: &[f64]) -> f64 {
    let leading = coefficients.last().copied().unwrap_or(0.0).abs();
    if leading == 0.0 {
        return 1.0;
    }
    1.0 + coefficients[..coefficients.len() - 1]
        .iter()
        .map(|coefficient| coefficient.abs() / leading)
        .fold(0.0, f64::max)
}

fn poly_eval(coefficients: &[f64], x: f64) -> f64 {
    coefficients
        .iter()
        .rev()
        .fold(0.0, |value, coefficient| value * x + coefficient)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpc_solves_known_rigid_motion() {
        // The expected pose and four transformed points are the independent
        // worked example used to sanity-check the C GPC implementation.
        let expected: [f64; 3] = [0.4, -0.2, 0.3];
        let cosine = expected[2].cos();
        let sine = expected[2].sin();
        let points = [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]];
        let correspondences = points.map(|p| GpcCorrespondence {
            p,
            q: [
                cosine * p[0] - sine * p[1] + expected[0],
                sine * p[0] + cosine * p[1] + expected[1],
            ],
            c: [[1.0, 0.0], [0.0, 1.0]],
            valid: true,
        });

        let actual = gpc_solve(&correspondences).expect("well-conditioned GPC system");
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
        }
    }
}
