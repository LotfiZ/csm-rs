# Changelog

## Unreleased

The library package moved to the repository root and now defines the
workspace. The previous `legacy`/`idiomatic` split is replaced by one public
interface, with the implementation modules private again.

### Added

- `Pose` and explicit `match_*_from` initial-guess matching, with clearly
  defined coordinate and composition conventions.
- Ordered Cartesian scans through the same matching and configuration
  contracts, with supplied or derived (`atan2`) bearings and preserved missing
  returns.
- Real termination reasons and candidate poses, counts, and residuals after an
  unsuccessful termination.
- Explicitly sized prepared pose workspaces with zero heap allocation during
  repeated matching, plus optional covariance, derivative, and
  Fisher-information outputs with reusable storage.
- A separate local browser demo (`csm-rs-demo`) that runs the real matcher,
  with reproducible playback, reference policies, scenarios, iteration
  inspection, and scan-pair import/export/replay.
- A public capability suite, per-ray sigma/known-orientation inputs, and
  reproducible latency, allocation, memory, and build-size measurements.

### Changed

- Match inputs and outcomes are exposed through `PolarScan`/`CartesianScan`,
  `Params`, `Matcher`/`PreparedMatcher`, and `MatchOutcome`.
- The C golden conformance corpus moved to a crate-internal test module that
  keeps fixture-level coverage of borrowed algorithm inputs.
- Known numerical deviations from the C implementation are documented with
  their validation evidence in the README.

### Removed

- The inherited 10,000-ray ceiling on scan validation.
