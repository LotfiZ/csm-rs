//! # csm-rs
//!
//! Rust port of the **Canonical Scan Matcher** (Andrea Censi, 2007):
//! point-to-line ICP with smart correspondence search, outlier rejection,
//! and a closed-form estimate of the matching covariance.
//!
//! This crate is a *faithful* port of the C reference implementation
//! (<https://github.com/AndreaCensi/csm>, branch `master`, GSL flavor).
//! Fidelity binds the arithmetic; the API surface is idiomatic Rust.
//! Each module documents the C files it ports.
//!
//! ## Port scope (grilling Q1)
//!
//! Ported: `sm_icp` — ICP/PlICP + covariance, including alpha test,
//! visibility test, ML/sigma weights, and restart logic.
//! Skipped: GPM, HSM, MbICP (unfinished upstream), Cairo drawing, CLI apps.
//!
//! ## Licensing
//!
//! Derivative work of CSM (LGPLv3) and its vendored `gpc` solver (GPLv2+).
//! Because the GPL-derived solver is part of this combined crate, distribution
//! is currently under GPL-2.0-or-later; see the repository NOTICE.md.

#![deny(unsafe_code)]
#![doc(test(attr(deny(warnings))))]

pub mod correspondence;
pub mod covariance;
pub mod icp;
pub mod idiomatic;
pub mod laser_data;
pub mod math;
pub mod params;
pub mod result;
pub mod solver;

pub use idiomatic::{
    CartesianScan, MatchOutcome, MatchStatus, Matcher, PolarScan, PreparedMatcher,
    PreparedPolarScan,
};
pub use laser_data::{LaserData, LaserDataError};
pub use params::{Params, ParamsError};
pub use result::SmResult;

/// Run point-to-line ICP scan matching.
///
/// The scans are mutable because CSM fills their cartesian, world-coordinate,
/// and correspondence fields in place. A failed match is reported through
/// `result.valid`; malformed scan data is returned as [`LaserDataError`].
/// This keeps malformed input separate from a valid scan pair that simply does
/// not converge.
///
/// C: `sm/csm/icp/icp.c:sm_icp()`
pub fn sm_icp(
    params: &Params,
    laser_ref: &mut LaserData,
    laser_sens: &mut LaserData,
    result: &mut SmResult,
) -> Result<(), LaserDataError> {
    icp::sm_icp(params, laser_ref, laser_sens, result)
}
