//! Small-matrix and 2D-pose algebra, hand-rolled (grilling Q3).
//!
//! C: `sm/csm/math_utils.c`, `sm/csm/math_utils_gsl.c`,
//!     `sm/csm/icp/fast_math.h`
//!
//! Scope: 2×2/3×3/4×4 matrix ops actually used by the port — multiply,
//! transpose, closed-form 2×2/3×3 inverse, 4×4 solve for the gpc normal
//! equations — plus 2D pose composition/difference and the correspondence
//! hash. Zero external math dependencies.

// TODO(port): mat2/mat3/mat4 ops, pose_diff_d, oplus, ld_corr_hash,
// count_equal, any_nan, friendly_pose (debug formatting).
