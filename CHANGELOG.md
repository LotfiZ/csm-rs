# Changelog

## Unreleased

- Added borrowed `PolarScan` and `CartesianScan` inputs.
- Added reusable `PreparedPolarScan` storage for fixed-rate matching.
- Added `PreparedMatcher` with persistent ICP scratch buffers, in-place polar
  and Cartesian frame updates, and reusable result storage.
- Added checked construction through `Matcher::try_new` and `Params::validate`.
- Added fluent `Params::validated` configuration construction and complete
  numeric range checks.
- Added explicit Cartesian beam angles with duplicate-bearing validation.
- Added typed `MatchStatus` and covariance/derivative outputs.
- Added explicit covariance and termination diagnostics, including iteration
  limit and no-correspondence outcomes.
- Removed the inherited 10,000-ray ceiling from idiomatic scan validation.
- Added a dependency-free interactive HTML visual example and idiomatic API examples.
- Added Linux x86_64/aarch64 CI checks and an MSRV check.
- Added prepared workspace memory introspection and explicit capacity-overflow
  errors for fixed-shape streaming updates.

The legacy `sm_icp` API remains available while the idiomatic API evolves.
