//! Small-matrix and 2D-pose algebra, hand-rolled (grilling Q3).
//!
//! C: `sm/csm/math_utils.c`, `sm/csm/math_utils_gsl.c`,
//!     `sm/csm/icp/fast_math.h`
//!
//! Scope: 2×2/3×3/4×4 matrix ops actually used by the port — multiply,
//! transpose, closed-form 2×2/3×3 inverse, 4×4 solve for the gpc normal
//! equations — plus 2D pose composition/difference and the correspondence
//! hash. Zero external math dependencies.

/// 2×2 matrix, row-major.
///
/// C: raw `gsl_matrix` 2×2 usage in `sm/lib/gpc/gpc.c`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat2 {
    /// Row-major matrix entries. C: `gsl_matrix` elements in `gpc.c`
    pub data: [[f64; 2]; 2],
}

impl Mat2 {
    /// Construct a 2×2 matrix from row-major entries.
    ///
    /// C: 2×2 `gsl_matrix` initialization in `sm/lib/gpc/gpc.c`
    pub fn new(data: [[f64; 2]; 2]) -> Self {
        Self { data }
    }

    /// Return the transpose of this matrix.
    ///
    /// C: `gsl_matrix_transpose_memcpy()` usage in `sm/lib/gpc/gpc.c`
    pub fn transpose(&self) -> Self {
        Self::new(std::array::from_fn(|r| {
            std::array::from_fn(|c| self.data[c][r])
        }))
    }

    /// Multiply this matrix by `other`.
    ///
    /// C: `gsl_blas_dgemm()` usage in `sm/lib/gpc/gpc.c`
    pub fn mul(&self, other: &Mat2) -> Mat2 {
        let (a, b) = (self.data, other.data);
        Mat2::new([
            [
                a[0][0] * b[0][0] + a[0][1] * b[1][0],
                a[0][0] * b[0][1] + a[0][1] * b[1][1],
            ],
            [
                a[1][0] * b[0][0] + a[1][1] * b[1][0],
                a[1][0] * b[0][1] + a[1][1] * b[1][1],
            ],
        ])
    }

    /// Closed-form inverse; `None` if singular.
    ///
    /// C: `m_inv()` in `sm/lib/gpc/gpc.c`
    pub fn inv(&self) -> Option<Mat2> {
        let a = self.data;
        let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        if det == 0.0 {
            return None;
        }
        let s = 1.0 / det;
        Some(Mat2::new([
            [a[1][1] * s, -a[0][1] * s],
            [-a[1][0] * s, a[0][0] * s],
        ]))
    }
}

/// 3×3 matrix, row-major.
///
/// C: egsl 3×3 usage in `sm/csm/icp/icp_covariance.c`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    /// Row-major matrix entries. C: 3×3 `gsl_matrix` elements in CSM
    pub data: [[f64; 3]; 3],
}

impl Mat3 {
    /// Construct a 3×3 matrix from row-major entries.
    ///
    /// C: 3×3 matrix initialization in `sm/csm/icp/icp_covariance.c`
    pub fn new(data: [[f64; 3]; 3]) -> Self {
        Self { data }
    }

    /// Scale every entry by a scalar.
    ///
    /// C: `sc()` in `sm/lib/egsl/egsl_ops.c`.
    pub fn scale(&self, scale: f64) -> Self {
        Self::new(std::array::from_fn(|row| {
            std::array::from_fn(|col| self.data[row][col] * scale)
        }))
    }

    /// Return the transpose of this matrix.
    ///
    /// C: matrix transpose operations in `sm/csm/icp/icp_covariance.c`
    pub fn transpose(&self) -> Self {
        Self::new(std::array::from_fn(|r| {
            std::array::from_fn(|c| self.data[c][r])
        }))
    }

    /// Multiply this matrix by `other`.
    ///
    /// C: matrix multiplication operations in `sm/csm/icp/icp_covariance.c`
    pub fn mul(&self, other: &Mat3) -> Mat3 {
        let (a, b) = (self.data, other.data);
        Mat3::new(std::array::from_fn(|r| {
            std::array::from_fn(|c| a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c])
        }))
    }

    /// Closed-form inverse via cofactor expansion; `None` if singular.
    ///
    /// C: 3×3 inverse operations in `sm/csm/icp/icp_covariance.c`
    pub fn inv(&self) -> Option<Mat3> {
        let a = self.data;
        let c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
        let c01 = -(a[1][0] * a[2][2] - a[1][2] * a[2][0]);
        let c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
        let det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
        if det == 0.0 {
            return None;
        }
        let s = 1.0 / det;
        let c10 = -(a[0][1] * a[2][2] - a[0][2] * a[2][1]);
        let c11 = a[0][0] * a[2][2] - a[0][2] * a[2][0];
        let c12 = -(a[0][0] * a[2][1] - a[0][1] * a[2][0]);
        let c20 = a[0][1] * a[1][2] - a[0][2] * a[1][1];
        let c21 = -(a[0][0] * a[1][2] - a[0][2] * a[1][0]);
        let c22 = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        // Inverse is the transpose of the cofactor matrix, scaled.
        Some(Mat3::new([
            [c00 * s, c10 * s, c20 * s],
            [c01 * s, c11 * s, c21 * s],
            [c02 * s, c12 * s, c22 * s],
        ]))
    }
}

/// Dynamically sized dense matrix with row-major rows.
///
/// C: `gsl_matrix` in `sm/csm/icp/icp_covariance.c`. The covariance itself
/// is always 3×3, but its input-derivative matrices are 3×N, where N is the
/// number of rays in the corresponding scan.
#[derive(Clone, Debug, PartialEq)]
pub struct Matrix {
    /// Matrix entries grouped by row.
    ///
    /// C: `gsl_matrix` elements accessed by `gsl_matrix_get()`.
    pub data: Vec<Vec<f64>>,
}

impl Matrix {
    /// Construct a zero-filled matrix.
    ///
    /// C: `zeros(rows, cols)` in `sm/lib/egsl/egsl_ops.c`.
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            data: vec![vec![0.0; cols]; rows],
        }
    }

    /// Number of rows.
    ///
    /// C: `gsl_matrix::size1`.
    pub fn rows(&self) -> usize {
        self.data.len()
    }

    /// Number of columns, or zero for a matrix without rows.
    ///
    /// C: `gsl_matrix::size2`.
    pub fn cols(&self) -> usize {
        self.data.first().map_or(0, Vec::len)
    }
}

/// 4×4 matrix, row-major — the gpc normal-equations size.
///
/// C: raw `gsl_matrix` 4×4 usage in `sm/lib/gpc/gpc.c`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat4 {
    /// Row-major matrix entries. C: 4×4 `gsl_matrix` elements in `gpc.c`
    pub data: [[f64; 4]; 4],
}

impl Mat4 {
    /// Construct a 4×4 matrix from row-major entries.
    ///
    /// C: 4×4 normal-equation matrix initialization in `sm/lib/gpc/gpc.c`
    pub fn new(data: [[f64; 4]; 4]) -> Self {
        Self { data }
    }

    /// Return the transpose of this matrix.
    ///
    /// C: transpose operations in `sm/lib/gpc/gpc.c`
    pub fn transpose(&self) -> Self {
        Self::new(std::array::from_fn(|r| {
            std::array::from_fn(|c| self.data[c][r])
        }))
    }

    /// Multiply this matrix by `other`.
    ///
    /// C: `gsl_blas_dgemm()` usage in `sm/lib/gpc/gpc.c`
    pub fn mul(&self, other: &Mat4) -> Mat4 {
        let (a, b) = (self.data, other.data);
        Mat4::new(std::array::from_fn(|r| {
            std::array::from_fn(|c| (0..4).map(|k| a[r][k] * b[k][c]).sum())
        }))
    }

    /// Solve `A·x = b` via LU decomposition with partial pivoting;
    /// `None` if singular.
    ///
    /// C: the 4×4 normal-equation solve in `sm/lib/gpc/gpc.c`
    pub fn solve(&self, b: [f64; 4]) -> Option<[f64; 4]> {
        let mut a = self.data;
        let mut x = b;
        // Forward elimination with partial pivoting.
        for col in 0..4 {
            let pivot =
                (col..4).max_by(|&r, &s| a[r][col].abs().partial_cmp(&a[s][col].abs()).unwrap())?;
            if a[pivot][col] == 0.0 {
                return None;
            }
            a.swap(col, pivot);
            x.swap(col, pivot);
            for r in (col + 1)..4 {
                let f = a[r][col] / a[col][col];
                let pivot_row = a[col]; // [f64; 4] is Copy: sidesteps the double borrow
                for (arc, &acc) in a[r].iter_mut().zip(pivot_row.iter()).skip(col) {
                    *arc -= f * acc;
                }
                x[r] -= f * x[col];
            }
        }
        // Back substitution.
        for r in (0..4).rev() {
            for c in (r + 1)..4 {
                x[r] -= a[r][c] * x[c];
            }
            x[r] /= a[r][r];
        }
        Some(x)
    }
}

/// Pose inverse: ⊖x.
///
/// C: `ominus_d()` in `sm/csm/math_utils.c`
pub fn ominus(x: [f64; 3]) -> [f64; 3] {
    let c = x[2].cos();
    let s = x[2].sin();
    [-c * x[0] - s * x[1], s * x[0] - c * x[1], -x[2]]
}

/// Pose composition: x1 ⊕ x2.
///
/// C: `oplus_d()` in `sm/csm/math_utils.c`
pub fn oplus(x1: [f64; 3], x2: [f64; 3]) -> [f64; 3] {
    let c = x1[2].cos();
    let s = x1[2].sin();
    [
        x1[0] + c * x2[0] - s * x2[1],
        x1[1] + s * x2[0] + c * x2[1],
        x1[2] + x2[2],
    ]
}

/// Pose difference `pose2 ⊖ pose1`, angle wrapped to (−π, π].
///
/// C: `pose_diff_d()` in `sm/csm/math_utils.c`
pub fn pose_diff(pose2: [f64; 3], pose1: [f64; 3]) -> [f64; 3] {
    let mut res = oplus(ominus(pose1), pose2);
    while res[2] > std::f64::consts::PI {
        res[2] -= 2.0 * std::f64::consts::PI;
    }
    while res[2] < -std::f64::consts::PI {
        res[2] += 2.0 * std::f64::consts::PI;
    }
    res
}

/// Transform a 2D point by a pose.
///
/// C: `transform_d()` in `sm/csm/math_utils.c`
pub fn transform(point: [f64; 2], pose: [f64; 3]) -> [f64; 2] {
    let c = pose[2].cos();
    let s = pose[2].sin();
    [
        pose[0] + c * point[0] - s * point[1],
        pose[1] + s * point[0] + c * point[1],
    ]
}

/// Smallest signed difference `a − b`, wrapped to (−π, π].
///
/// C: `angleDiff()` in `sm/csm/math_utils.c`
pub fn angle_diff(a: f64, b: f64) -> f64 {
    let mut t = a - b;
    while t < -std::f64::consts::PI {
        t += 2.0 * std::f64::consts::PI;
    }
    while t > std::f64::consts::PI {
        t -= 2.0 * std::f64::consts::PI;
    }
    t
}

/// Squared Euclidean distance between two 2D points.
///
/// C: `distance_squared_d()` in `sm/csm/math_utils.c`
pub(crate) fn distance_squared(a: [f64; 2], b: [f64; 2]) -> f64 {
    let x = a[0] - b[0];
    let y = a[1] - b[1];
    x * x + y * y
}

/// Euclidean norm of a 2D vector.
///
/// C: `norm_d()` in `sm/csm/math_utils.c`
pub(crate) fn norm(point: [f64; 2]) -> f64 {
    (point[0] * point[0] + point[1] * point[1]).sqrt()
}

/// Project a point onto the line through two points.
///
/// C: `projection_on_line_d()` in `sm/csm/math_utils.c`
pub(crate) fn projection_on_line(a: [f64; 2], b: [f64; 2], point: [f64; 2]) -> [f64; 2] {
    projection_on_line_with_distance(a, b, point).0
}

fn projection_on_line_with_distance(a: [f64; 2], b: [f64; 2], point: [f64; 2]) -> ([f64; 2], f64) {
    let t0 = a[0] - b[0];
    let t1 = a[1] - b[1];
    let one_on_r = 1.0 / (t0 * t0 + t1 * t1).sqrt();
    let c = t1 * one_on_r;
    let s = -t0 * one_on_r;
    let rho = c * a[0] + s * a[1];
    let projection = [
        c * rho + s * s * point[0] - c * s * point[1],
        s * rho - c * s * point[0] + c * c * point[1],
    ];
    let distance = (rho - (c * point[0] + s * point[1])).abs();
    (projection, distance)
}

/// Project a point onto a segment, clamping to the nearer endpoint.
///
/// C: `projection_on_segment_d()` in `sm/csm/math_utils.c`
pub(crate) fn projection_on_segment(a: [f64; 2], b: [f64; 2], point: [f64; 2]) -> [f64; 2] {
    let projection = projection_on_line(a, b, point);
    let inside = (projection[0] - a[0]) * (projection[0] - b[0])
        + (projection[1] - a[1]) * (projection[1] - b[1])
        < 0.0;
    if inside {
        projection
    } else if distance_squared(a, point) < distance_squared(b, point) {
        a
    } else {
        b
    }
}

/// Distance from a point to a segment.
///
/// C: `dist_to_segment_d()` in `sm/csm/math_utils.c`
pub(crate) fn distance_to_segment(a: [f64; 2], b: [f64; 2], point: [f64; 2]) -> f64 {
    let (projection, distance) = projection_on_line_with_distance(a, b, point);
    let inside = (projection[0] - a[0]) * (projection[0] - b[0])
        + (projection[1] - a[1]) * (projection[1] - b[1])
        < 0.0;
    if inside {
        distance
    } else {
        distance_squared(a, point)
            .min(distance_squared(b, point))
            .sqrt()
    }
}

/// Hash of a correspondence set, used for oscillation detection.
/// Each entry is `Some((j1, j2))` for a valid correspondence, `None` otherwise.
///
/// C: `ld_corr_hash()` in `sm/csm/laser_data.c` — reproduces C's unsigned
/// wraparound arithmetic exactly (golden-master fidelity, grilling Q2).
pub fn corr_hash(entries: &[Option<(i32, i32)>]) -> u32 {
    let mut hash: u32 = 0;
    for (i, entry) in entries.iter().enumerate() {
        let str_val: u32 = match entry {
            Some((j1, j2)) => (j1 + 1000 * j2) as u32,
            None => -1i32 as u32,
        };
        hash ^= if (i & 1) == 0 {
            (hash << 7) ^ str_val ^ (hash >> 3)
        } else {
            !((hash << 11) ^ str_val ^ (hash >> 5))
        };
    }
    hash & 0x7FFFFFFF
}

/// Invert a square dynamic matrix via LU decomposition with partial
/// pivoting. Used by `filter_orientation` (C: egsl `inv()` on the n×n
/// `R·Rᵀ` matrix in `orientation.c`).
pub(crate) fn invert_dyn(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    let mut lu = a.to_vec();
    let mut perm: Vec<usize> = (0..n).collect();
    // LU in place, tracking row permutations.
    for col in 0..n {
        let pivot =
            (col..n).max_by(|&r, &s| lu[r][col].abs().partial_cmp(&lu[s][col].abs()).unwrap())?;
        if lu[pivot][col] == 0.0 {
            return None;
        }
        lu.swap(col, pivot);
        perm.swap(col, pivot);
        for r in (col + 1)..n {
            let f = lu[r][col] / lu[col][col];
            let pivot_row = lu[col].clone();
            for (arc, &acc) in lu[r].iter_mut().zip(pivot_row.iter()).skip(col) {
                *arc -= f * acc;
            }
            lu[r][col] = f;
        }
    }
    // Solve A·X = P·I column by column.
    let mut inv = vec![vec![0.0; n]; n];
    for k in 0..n {
        // Unit vector of the permuted system: row r holds e_{perm[r]}.
        let mut b: Vec<f64> = (0..n)
            .map(|r| if perm[r] == k { 1.0 } else { 0.0 })
            .collect();
        // Forward substitution (unit lower).
        for r in 0..n {
            for c in 0..r {
                b[r] -= lu[r][c] * b[c];
            }
        }
        // Back substitution.
        for r in (0..n).rev() {
            for c in (r + 1)..n {
                b[r] -= lu[r][c] * b[c];
            }
            b[r] /= lu[r][r];
        }
        for (r, row) in inv.iter_mut().enumerate() {
            row[k] = b[r];
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn assert_mat2_approx(a: &Mat2, expected: [[f64; 2]; 2]) {
        for (r, row) in expected.iter().enumerate() {
            for (c, &want) in row.iter().enumerate() {
                assert!(
                    (a.data[r][c] - want).abs() < 1e-15,
                    "mismatch at ({r},{c}): got {}, expected {}",
                    a.data[r][c],
                    want
                );
            }
        }
    }

    #[test]
    fn mat2_mul_known_product() {
        // Worked example: [[1,2],[3,4]] * [[5,6],[7,8]] = [[19,22],[43,50]]
        let a = Mat2::new([[1.0, 2.0], [3.0, 4.0]]);
        let b = Mat2::new([[5.0, 6.0], [7.0, 8.0]]);
        let c = a.mul(&b);
        assert_eq!(c.data, [[19.0, 22.0], [43.0, 50.0]]);
    }

    #[test]
    fn mat2_inv_textbook_example() {
        // Textbook: [[4,7],[2,6]] has det 10, inv = [[0.6,-0.7],[-0.2,0.4]]
        let a = Mat2::new([[4.0, 7.0], [2.0, 6.0]]);
        let inv = a.inv().expect("invertible");
        assert_mat2_approx(&inv, [[0.6, -0.7], [-0.2, 0.4]]);
    }

    #[test]
    fn mat3_mul_known_product() {
        // Worked example: A * I = A, and a rotation-like sanity product.
        let a = Mat3::new([[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]]);
        let id = Mat3::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(a.mul(&id).data, a.data);
        assert_eq!(id.mul(&a).data, a.data);
    }

    #[test]
    fn mat3_scale_known_product() {
        let a = Mat3::new([[1.0, -2.0, 3.0], [4.0, 5.0, -6.0], [7.0, 8.0, 9.0]]);
        assert_eq!(
            a.scale(0.5).data,
            [[0.5, -1.0, 1.5], [2.0, 2.5, -3.0], [3.5, 4.0, 4.5]]
        );
    }

    #[test]
    fn mat3_inv_det1_example() {
        // Classic: det = 1, inverse is the known integer adjugate.
        let a = Mat3::new([[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]]);
        let inv = a.inv().expect("invertible");
        let expected = [[-24.0, 18.0, 5.0], [20.0, -15.0, -4.0], [-5.0, 4.0, 1.0]];
        for (r, row) in expected.iter().enumerate() {
            for (c, &want) in row.iter().enumerate() {
                assert!((inv.data[r][c] - want).abs() < 1e-12);
            }
        }
        // Round-trip: A * A^-1 = I
        let prod = a.mul(&inv);
        for (r, row) in prod.data.iter().enumerate() {
            for (c, &got) in row.iter().enumerate() {
                let want = if r == c { 1.0 } else { 0.0 };
                assert!((got - want).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn mat3_inv_singular_returns_none() {
        let a = Mat3::new([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert!(a.inv().is_none());
    }

    #[test]
    fn mat4_mul_diagonal_scales_columns() {
        // Worked example: multiplying by diag(1,2,3,4) scales each column.
        let a = Mat4::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let d = Mat4::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0, 0.0],
            [0.0, 0.0, 3.0, 0.0],
            [0.0, 0.0, 0.0, 4.0],
        ]);
        let c = a.mul(&d);
        assert_eq!(c.data[0], [1.0, 4.0, 9.0, 16.0]);
        assert_eq!(c.data[3], [13.0, 28.0, 45.0, 64.0]);
    }

    #[test]
    fn mat4_solve_recovers_known_solution() {
        // A = [[4,3,2,1],[3,4,3,2],[2,3,4,3],[1,2,3,4]], x = [1,1,1,1]
        // => b = A·x = row sums = [10, 12, 12, 10]
        let a = Mat4::new([
            [4.0, 3.0, 2.0, 1.0],
            [3.0, 4.0, 3.0, 2.0],
            [2.0, 3.0, 4.0, 3.0],
            [1.0, 2.0, 3.0, 4.0],
        ]);
        let x = a.solve([10.0, 12.0, 12.0, 10.0]).expect("nonsingular");
        for v in x {
            assert!((v - 1.0).abs() < 1e-12, "got {v}");
        }
    }

    #[test]
    fn mat4_solve_singular_returns_none() {
        let a = Mat4::new([
            [1.0, 2.0, 3.0, 4.0],
            [2.0, 4.0, 6.0, 8.0],
            [0.0, 1.0, 0.0, 1.0],
            [1.0, 0.0, 1.0, 0.0],
        ]);
        assert!(a.solve([1.0, 1.0, 1.0, 1.0]).is_none());
    }

    fn assert_pose_approx(p: [f64; 3], expected: [f64; 3]) {
        for i in 0..3 {
            assert!(
                (p[i] - expected[i]).abs() < 1e-12,
                "pose[{i}]: got {}, expected {}",
                p[i],
                expected[i]
            );
        }
    }

    #[test]
    fn oplus_rotates_then_translates() {
        // Worked: pose (1,2,π/2) ⊕ (1,0,0): c=0, s=1 => (1, 3, π/2)
        let r = oplus([1.0, 2.0, FRAC_PI_2], [1.0, 0.0, 0.0]);
        assert_pose_approx(r, [1.0, 3.0, FRAC_PI_2]);
    }

    #[test]
    fn ominus_inverts_pose() {
        // Worked: ⊖(1,2,π/2): c=0, s=1 => (-2, 1, -π/2)
        let r = ominus([1.0, 2.0, FRAC_PI_2]);
        assert_pose_approx(r, [-2.0, 1.0, -FRAC_PI_2]);
        // Round-trip: x ⊕ (⊖x) = identity
        let id = oplus([1.0, 2.0, 0.7], ominus([1.0, 2.0, 0.7]));
        assert_pose_approx(id, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn pose_diff_basic_and_wrapped() {
        // Worked: diff((2,2,0),(1,0,0)) = (1,2,0)
        let r = pose_diff([2.0, 2.0, 0.0], [1.0, 0.0, 0.0]);
        assert_pose_approx(r, [1.0, 2.0, 0.0]);
        // Angle wraps into (-π, π]: θ = -π - π = -2π => 0
        let w = pose_diff([0.0, 0.0, -PI], [0.0, 0.0, PI]);
        assert_pose_approx(w, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn transform_point_by_pose() {
        // Worked: transform (1,0) by (1,1,π/2) => (1,2)
        let r = transform([1.0, 0.0], [1.0, 1.0, FRAC_PI_2]);
        assert!((r[0] - 1.0).abs() < 1e-12);
        assert!((r[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn angle_diff_wraps_to_pi_range() {
        assert!((angle_diff(0.2, 2.0 * PI) - 0.2).abs() < 1e-12);
        assert!((angle_diff(PI + 0.5, 0.0) - (0.5 - PI)).abs() < 1e-12);
        assert!((angle_diff(0.3, 0.1) - 0.2).abs() < 1e-12);
    }

    #[test]
    fn invert_dyn_matches_known_3x3() {
        // Same worked example as mat3_inv_det1_example, through the
        // dynamic path used by filter_orientation.
        let a = vec![
            vec![1.0, 2.0, 3.0],
            vec![0.0, 1.0, 4.0],
            vec![5.0, 6.0, 0.0],
        ];
        let inv = invert_dyn(&a).expect("invertible");
        let expected = [[-24.0, 18.0, 5.0], [20.0, -15.0, -4.0], [-5.0, 4.0, 1.0]];
        for (r, row) in expected.iter().enumerate() {
            for (c, &want) in row.iter().enumerate() {
                assert!((inv[r][c] - want).abs() < 1e-12, "({r},{c}): {}", inv[r][c]);
            }
        }
    }

    #[test]
    fn invert_dyn_singular_returns_none() {
        let a = vec![vec![1.0, 2.0], vec![2.0, 4.0]];
        assert!(invert_dyn(&a).is_none());
    }

    #[test]
    fn corr_hash_matches_c_reference() {
        // Ground truth generated by compiling CSM's ld_corr_hash verbatim.
        // case1: [Some((1,2)), None]
        assert_eq!(corr_hash(&[Some((1, 2)), None]), 4100079);
        // case2: [Some((3,4)), Some((5,6)), Some((7,8))]
        assert_eq!(
            corr_hash(&[Some((3, 4)), Some((5, 6)), Some((7, 8))]),
            554505721
        );
        // case3: four invalid correspondences
        assert_eq!(corr_hash(&[None, None, None, None]), 419176248);
        // case4: single valid correspondence at origin
        assert_eq!(corr_hash(&[Some((0, 0))]), 0);
    }

    #[test]
    fn transpose_swaps_off_diagonals() {
        let m2 = Mat2::new([[1.0, 2.0], [3.0, 4.0]]).transpose();
        assert_eq!(m2.data, [[1.0, 3.0], [2.0, 4.0]]);
        let m3 = Mat3::new([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]).transpose();
        assert_eq!(m3.data[0], [1.0, 4.0, 7.0]);
        assert_eq!(m3.data[2], [3.0, 6.0, 9.0]);
        let m4 = Mat4::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ])
        .transpose();
        assert_eq!(m4.data[0], [1.0, 5.0, 9.0, 13.0]);
        assert_eq!(m4.data[3], [4.0, 8.0, 12.0, 16.0]);
    }

    #[test]
    fn mat2_inv_singular_returns_none() {
        let a = Mat2::new([[1.0, 2.0], [2.0, 4.0]]);
        assert!(a.inv().is_none());
    }
}
