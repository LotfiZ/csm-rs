# Release readiness

This document verifies the epic's release criteria against recorded evidence.
It does **not** publish a release; publishing is a separate manual step.

## Criteria and evidence

| Criterion | Evidence |
| --- | --- |
| Validated core behavior and retained capabilities | `src/golden_tests.rs` (exact C corpus) and `tests/capabilities.rs` (public capability suite); `cargo test --all-targets --all-features` |
| Verified resource guarantees | Prepared pose and uncertainty matching allocate zero heap allocations after preparation (`tests/prepared_allocation.rs`); capacity errors instead of silent growth; documented in the README |
| Recorded Xavier measurements | [hardware.md](hardware.md) and [measurements.md](measurements.md); regenerate with `./scripts/measure-release.sh` |
| Accurate public documentation | README leads with installation, examples, coordinate/outcome contracts, configuration, and resource guarantees; `docs/` holds contributing history, measurements, and hardware evidence |
| Verified distribution obligations | `cargo package -p csm-rs --allow-dirty` verifies the packaged crate compiles; `scripts/check-package.sh` confirms fixtures and the generator are excluded; `NOTICE.md` plus both license texts are distributed; the crate remains `GPL-2.0-or-later` because the GPL-derived solver is part of the combined work |
| One complete, trustworthy interactive workflow | The demo package serves the real matcher and covers playback, controls, scenarios, reference policies, iteration inspection, and import/export/replay; verified by 14 browser-to-Rust tests and a manual end-to-end smoke run |
| Explicit disclosure of unverified claims | [hardware.md](hardware.md) marks Raspberry Pi and x86 physical performance unverified and treats CI as build/correctness evidence only; [measurements.md](measurements.md) records the p99 target miss |

## Supported interface

- Ordered polar and Cartesian scans with explicit missing returns and optional
  per-ray sigma/known-orientation inputs.
- Validated configuration with useful defaults, explicit initial poses, and
  caller-selected references.
- Pose-only matching plus optional covariance, derivative, and Fisher
  information; uncertainty failure never invalidates a usable pose.
- Explicit termination reasons and candidate estimates.
- Reusable, explicitly sized, allocation-free pose and uncertainty workspaces.
- Opt-in iteration instrumentation that does not change results.

## Hardware

- **Validated:** Jetson AGX Xavier (physical device).
- **Build/correctness only:** Linux x86_64 and aarch64 CI checks.
- **Unverified:** Raspberry Pi and conventional x86 physical performance.

## Known limitations

- The provisional pose-only p99 target (< 10 ms at 3,000 points) is missed on
  the validated Xavier (p99 ≈ 12–14 ms); 20–30 Hz is still supported for
  typical workloads. See [measurements.md](measurements.md).
- Large scans (10,000 points) scale super-linearly to ≈ 124 ms mean.
- Difficult partial-overlap scans with noise can consume the iteration budget;
  bound `max_iterations`.
- Public hosting, WebAssembly, ROS integration, and microcontroller/`no_std`
  targets are out of scope.

## Reproduce

```sh
cargo test --workspace --all-targets --all-features
cargo test --doc
cargo clippy --workspace --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps
cargo package -p csm-rs --allow-dirty
./scripts/check-package.sh
./scripts/measure-release.sh
```
