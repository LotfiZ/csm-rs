//! Correspondence finding and outlier rejection.
//!
//! C: `sm/csm/icp/icp_corr_tricks.c` (jump-table search),
//!     `sm/csm/icp/icp_corr_dumb.c` (naive search),
//!     `sm/csm/icp/icp_outliers.c` (trim + doubles rejection),
//!     `sm/csm/laser_data.c` (`possible_interval`, visibility test)
//!
//! Property-test invariant (grilling Q4): `Tricks` and `Naive` strategies
//! must produce identical correspondence sets — the same invariant C checks
//! with `debug_verify_tricks`.

// TODO(port): find_correspondences (naive), find_correspondences_tricks
// (jump tables), kill_outliers_trim, kill_outliers_double, visibility_test.
