//! # csm-rs
//!
//! Point-to-line ICP scan matching (Censi, 2007) with smart correspondence
//! search, outlier rejection, restart handling, and an optional closed-form
//! estimate of the matching covariance.
//!
//! The supported interface is organized around four ideas:
//!
//! - **Scans**: [`PolarScan`] (ordered polar returns) and [`CartesianScan`]
//!   (ordered points).
//! - **Configuration**: [`Params`] and its validated sub-configuration.
//! - **Matching**: [`Matcher`], plus [`PreparedMatcher`] for reusable storage.
//! - **Results**: [`MatchOutcome`], with explicit [`TerminationReason`].
//!
//! ## Coordinate contract
//!
//! Inputs use **metres and radians**. A scan is centered on its own sensor
//! origin. Rays are **ordered by bearing**: a polar ray at angle `theta` with
//! reading `r` is the sensor-frame point `[r cos(theta), r sin(theta)]`, and a
//! Cartesian ray is its own `[x, y]`. The matcher relies on that ordering for
//! its neighbourhood search and never sorts or drops points. Unordered
//! point-cloud registration is out of scope.
//!
//! The match result is the rigid transform that maps **sensor-scan
//! coordinates into reference-scan coordinates**: `R(theta) * p + (x, y)`,
//! with `theta` counter-clockwise in the reference frame. [`Pose`] documents
//! the composition convention.
//!
//! ## Quick start
//!
//! ```
//! use csm_rs::{Matcher, Params, PolarScan};
//!
//! let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
//! let readings: Vec<f64> = angles.iter().map(|a| 8.0 + 0.2 * a.cos()).collect();
//! let valid = vec![true; angles.len()];
//! let reference = PolarScan::new(&angles, &readings, &valid)?;
//! let sensor = PolarScan::new(&angles, &readings, &valid)?;
//!
//! let matcher = Matcher::new(Params::default())?;
//! let outcome = matcher.match_polar(reference, sensor)?;
//! assert!(outcome.valid);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ## Errors and outcomes
//!
//! Malformed input returns a [`ScanError`]. A well-formed pair that cannot be
//! matched is a normal [`MatchOutcome`] with `valid == false`; inspect
//! [`MatchOutcome::termination`] for the reason.
//!
//! ## Licensing
//!
//! Derivative work of the Canonical Scan Matcher (LGPLv3) and its vendored
//! solver (GPLv2+). Because the GPL-derived solver is part of this combined
//! crate, distribution is under GPL-2.0-or-later; see the repository NOTICE.md.

#![deny(unsafe_code)]
#![doc(test(attr(deny(warnings))))]

mod correspondence;
mod covariance;
mod icp;
mod matching;
mod math;
mod params;
mod pose;
mod result;
mod scan;
mod scan_data;
mod solver;

pub use matching::{
    CorrespondenceSnapshot, CovarianceStatus, IterationSnapshot, MatchOutcome, MatchStatus,
    Matcher, PreparedMatcher, PreparedPolarScan, TerminationReason,
};
pub use params::{
    CorrectionLimits, CorrespondenceParams, CorrespondenceSearch, DistanceMetric, OutlierParams,
    Params, ParamsError, ReadingBounds, RestartParams, StoppingCriteria, WeightParams,
};
pub use pose::Pose;
pub use scan::{CartesianScan, PolarScan, ScanError};
