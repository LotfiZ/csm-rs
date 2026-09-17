# Changelog

## Unreleased — remaster

- Moved the library package to the repository root and made it define the
  workspace ([#36](https://github.com/LotfiZ/csm-rs/issues/36)).
- Replaced the legacy/idiomatic API split with one coherent public interface
  organized around scans, configuration, matching, and results. Implementation
  modules are private again ([#37](https://github.com/LotfiZ/csm-rs/issues/37)).
- Added `Pose` with an identity convenience and explicit `match_*_from`
  initial-guess matching.
- Support ordered Cartesian scans through the same matching and configuration
  contracts, with supplied or derived (`atan2`) bearings, preserved missing
  returns, and no silent sorting ([#38](https://github.com/LotfiZ/csm-rs/issues/38)).
- Report actual termination reasons (convergence, iteration exhaustion,
  insufficient geometry, no correspondences, cycles, numerical failure) instead
  of inferring from counters, and preserve candidate poses, counts, and
  residuals after unsuccessful termination
  ([#39](https://github.com/LotfiZ/csm-rs/issues/39)).
- Provide explicitly sized prepared pose workspaces with reference/sensor
  capacity, explicit growth, clear capacity errors, and zero heap allocation
  during repeated matching for the retained options
  ([#40](https://github.com/LotfiZ/csm-rs/issues/40)).
- Provide optional covariance, derivative, and Fisher-information outputs with
  reusable prepared storage and zero allocation after preparation, reporting
  uncertainty failure independently of pose status
  ([#41](https://github.com/LotfiZ/csm-rs/issues/41)).
- Validate the retained numerical capabilities through the public API, with a
  reproducible capability suite, per-ray sigma/known-orientation inputs for the
  weighting paths, and a documented deviations summary
  ([#42](https://github.com/LotfiZ/csm-rs/issues/42)).
- Documented the coordinate contract: metres, radians, scan ordering, sensor
  origin, and the sensor-to-reference transform composition.
- Retained `PreparedPolarScan`/`PreparedMatcher` reusable storage and optional
  covariance diagnostics behind the single API.
- Moved the C golden conformance corpus to a crate-internal test module so it
  can retain fixture-level coverage of borrowed algorithm inputs.

## 0.0.x — legacy port

The following features existed on the pre-remaster API and are preserved (or
superseded) by the remaster above:

- Borrowed `PolarScan` and `CartesianScan` inputs.
- Reusable `PreparedPolarScan` storage for fixed-rate matching.
- `PreparedMatcher` with persistent ICP scratch buffers, in-place polar and
  Cartesian frame updates, and reusable result storage.
- Checked construction through `Matcher::try_new` and `Params::validate`.
- Fluent `Params::validated` configuration construction and complete numeric
  range checks.
- Explicit Cartesian beam angles with duplicate-bearing validation.
- Typed `MatchStatus` and covariance/derivative outputs.
- Explicit covariance and termination diagnostics.
- Removed the inherited 10,000-ray ceiling from scan validation.
- Linux x86_64/aarch64 CI checks and an MSRV check.
- Prepared workspace memory introspection and explicit capacity-overflow errors.
