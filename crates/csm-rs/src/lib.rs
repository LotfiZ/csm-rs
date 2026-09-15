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
//! Derivative work of CSM (LGPLv3) and its vendored `gpc` solver (GPLv2+);
//! this crate is therefore `LGPL-3.0-only OR GPL-2.0-or-later`.

pub mod correspondence;
pub mod covariance;
pub mod icp;
pub mod laser_data;
pub mod math;
pub mod params;
pub mod result;
pub mod solver;

pub use laser_data::LaserData;
pub use params::Params;
pub use result::SmResult;

/// Run point-to-line ICP scan matching.
///
/// C: `sm/csm/icp/icp.c:sm_icp()`
pub fn sm_icp(params: &Params, result: &mut SmResult) {
    icp::sm_icp(params, result)
}
