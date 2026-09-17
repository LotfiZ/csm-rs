# Measurements

Reproducible resource measurements for the core library. Regenerate the whole
report (environment, build sizes, and latency/allocation/memory/alignment)
with:

```sh
./scripts/measure-release.sh
```

The latency/allocation part alone:

```sh
cargo run --release -p csm-rs --example measure
```

## Method

- **Preparation separated from matching.** `prep_ms` is the time to build the
  reference/sensor scans and the `PreparedMatcher` workspace; matching latency
  is measured afterwards on the prepared workspace.
- **Latency distributions.** `p50`, `p95`, and `p99` are reported from sorted
  per-match samples, not averages alone.
- **Modes measured separately.** Pose-only and covariance/Fisher
  (`do_compute_covariance`) modes are separate workloads.
- **Allocation counting.** A process-global counting allocator records heap
  allocations during the matching loop only; reusable storage is reserved
  before the loop starts, so the reported `allocs` is the steady-state count.
- **Memory.** `prepared_bytes` is an estimate of the prepared scan buffers;
  `workspace_bytes` is the reusable ICP scratch footprint.
- **Alignment quality.** Mean translation and rotation error against the
  synthetic ground-truth transform.
- **Instrumented timing** (`match_once_traced`, the demo's iteration
  inspection) is measured separately from ordinary matching and is not mixed
  into these numbers.
- **Workloads.** Synthetic ordered polar scans generated from a deterministic
  seed, with a known 0.05 rad rotation. The "difficult" workload drops 40% of
  sensor rays and adds 0.02 m noise. The restart workload enables CSM's
  six-perturbation shell.

## Environment

- Device: **Jetson AGX (NVIDIA Jetson-AGX, `t186ref`)**, L4T R35.6.5.
- OS: Ubuntu 20.04 (focal), Linux `5.10.216-tegra`, aarch64, 8 logical CPUs.
- Toolchain: `rustc 1.98.1`, `cargo 1.98.1`.
- Build: `--release`, workspace profile with `lto = true`.
- The exact revision is printed by `scripts/measure-release.sh`.

## Latest recorded run

Revision `2ed9e6c` (see the changelog for the current revision). One run on the
device above; latencies vary by a few percent run to run.

| workload | rays | prep ms | match mean ms | p50 | p95 | p99 | max | allocs | align (m / rad) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| easy pose-only | 720 | 1.45 | 1.38 | 1.35 | 1.76 | 2.86 | 2.96 | 0 | 1.0e-6 / 1.6e-7 |
| typical pose-only | 2048 | 2.47 | 5.42 | 5.24 | 7.09 | 7.26 | 7.32 | 0 | 7.5e-7 / 1.1e-7 |
| typical pose-only | 3000 | 3.62 | 10.17 | 9.86 | 11.68 | 13.56 | 17.28 | 0 | 5.9e-8 / 8.6e-9 |
| large pose-only | 10000 | 14.57 | 124.07 | 123.83 | 127.34 | 128.67 | 128.67 | 0 | 3.6e-9 / 1.3e-9 |
| typical uncertainty | 3000 | 3.52 | 11.67 | 11.53 | 12.48 | 14.43 | 17.25 | 0 | 5.9e-8 / 8.6e-9 |
| difficult partial (40% dropout, 0.02 m noise) | 3000 | 0.65 | 94.02 | 90.11 | 109.44 | 151.15 | 183.35 | 0 | 4.7e-2 / 4.1e-2 |
| restart shell | 3000 | 0.70 | 154.70 | 153.90 | 164.61 | 166.72 | 166.72 | 0 | 4.2e-2 / 2.6e-2 |

Memory: 3000-point prepared scans about 102 KiB, ICP workspace about 292 KiB;
10000-point scans about 340 KiB with a 964 KiB workspace.

Build sizes (bytes): `measure` example 539,016; `scan_matching` 514,568;
`benchmark_prepared` 523,576; the demo release binary is also reported by the
script. Example binaries are dominated by the standard library and debug
metadata; the rlib size is reported alongside them.

## Assessment against the provisional target

The provisional engineering target is **pose-only p99 below 10 ms for
3,000-point scans**. On this Jetson AGX, 3,000-point pose-only measures
**p99 ≈ 12–14 ms** and mean ≈ 10 ms. The target is **not** met on this device
as measured, and this is recorded rather than worked around.

However, the provisional target is not a release blocker. The useful supported
workload is clear:

- 20–30 Hz operation (33–50 ms per cycle) is comfortably supported for
  typical 2,000–3,000-point scans in both pose-only and covariance modes:
  even the p99 is ~14 ms, leaving ample margin.
- Larger scans scale super-linearly: 10,000 points cost ~120–130 ms mean,
  which supports ~8 Hz, not 20–30 Hz.
- Difficult partial-overlap scans with noise can consume the full iteration
  budget (~100 ms); applications should bound `max_iterations` for their
  latency budget.
- The restart shell multiplies cost (~150 ms at 3,000 points); disable it when
  the initial guess is good enough.

## Reproducing

1. Run `./scripts/measure-release.sh` on the target device.
2. Compare the printed revision and environment with this document.
3. Latencies are not comparable across devices or build profiles; see
   [hardware.md](hardware.md) for which targets are physically validated.
