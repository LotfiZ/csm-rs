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
- Add a separate local browser demo package (axum + Tokio) that ray-casts a
  moving sensor and runs the real matcher, drawing reference, unaligned, and
  aligned scans with true/estimated motion and termination diagnostics
  ([#43](https://github.com/LotfiZ/csm-rs/issues/43)).
- Make demo playback reproducible: play/pause/step/reset state, independent
  motion/noise/dropout/initial-guess controls, seeded replay, and request-id
  stale-response guarding verified by browser-to-Rust tests
  ([#44](https://github.com/LotfiZ/csm-rs/issues/44)).
- Add fixed-reference and previous-frame policies with accumulated-drift
  reporting, asymmetric-room/ambiguous-corridor/partial-overlap scenarios,
  progressive advanced configuration, and candidate-versus-accepted result
  display, verified through the browser workflow
  ([#45](https://github.com/LotfiZ/csm-rs/issues/45)).
- Expose opt-in iteration instrumentation with pose updates, correspondences,
  residuals, and restart context; let the browser inspect iterations, and show
  instrumented timing separately from ordinary matching
  ([#46](https://github.com/LotfiZ/csm-rs/issues/46)).
- Support importing ordered polar/Cartesian scan pairs, exporting versioned
  sessions, and replaying them by rerunning the matcher, with clear malformed
  input and unsupported-version errors
  ([#47](https://github.com/LotfiZ/csm-rs/issues/47)).
- Add reproducible latency (including p99), allocation, memory, alignment, and
  build-size measurements with recorded workloads, environment, and commands
  ([#48](https://github.com/LotfiZ/csm-rs/issues/48)).
- Validate the library on physical Jetson AGX Xavier hardware, recording
  correctness and performance evidence, assessing the provisional p99 target,
  and disclosing unverified targets
  ([#49](https://github.com/LotfiZ/csm-rs/issues/49)).
- Verify release documentation, distribution obligations, package contents,
  runnable examples, dependency separation, and the complete browser workflow,
  and assess every epic release criterion in docs/release-readiness.md
  ([#50](https://github.com/LotfiZ/csm-rs/issues/50)).
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
