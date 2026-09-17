# Performance and hardware validation

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
- **Latency distributions.** `p50`, `p95`, and `p99` come from sorted per-match
  samples, not averages alone.
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
  inspection) is measured separately and is not mixed into these numbers.
- **Workloads.** Synthetic ordered polar scans from a deterministic seed, with
  a known 0.05 rad rotation. The "difficult" workload drops 40% of sensor rays
  and adds 0.02 m noise. The restart workload enables CSM's six-perturbation
  shell.

## Validated hardware

| Item | Value |
| --- | --- |
| Device | NVIDIA Jetson AGX Xavier developer kit |
| Module / carrier | `p2822-0000` + `p2888-0001` (`nvidia,jetson-xavier`, `nvidia,tegra194`) |
| Device tree model | `Jetson-AGX` |
| BSP | L4T R35.6.5 (Jetson Linux) |
| CPU / memory | 8× ARMv8 (Carmel), aarch64, 30 GiB |
| OS | Ubuntu 20.04, Linux `5.10.216-tegra` |
| Toolchain | `rustc 1.98.1`, `cargo 1.98.1` |
| Build | `--release`, workspace profile with `lto = true` |

`cargo test --workspace --all-targets --all-features` passes 129 tests on this
device, including the C golden corpus, the public capability suite, the
prepared allocation check, and the demo workflow tests.

## Recorded run

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

## Assessment

The provisional target — pose-only **p99 below 10 ms for 3,000 points** — is
**not met on this Xavier** (p99 ≈ 12–14 ms across runs). It is recorded as a
miss, and the supported workload is stated explicitly:

- 20–30 Hz cycles (33–50 ms) are comfortably supported for typical
  2,000–3,000-point scans in both pose-only and covariance modes.
- 10,000-point scans are supported at roughly 8 Hz (≈ 124 ms mean).
- Difficult partial-overlap scans with noise can use the full iteration budget
  (≈ 100 ms); bound `max_iterations` to fit a latency budget.
- The restart shell multiplies cost (≈ 150 ms at 3,000 points); disable it when
  the initial guess is already good.

These are ordinary Linux figures for this device, not a hard real-time
guarantee. 20–30 Hz is the typical application frequency.

**Build/CI evidence only:** Linux `x86_64` and `aarch64` are checked by CI
(`cargo check` / `cargo test`), which is build and correctness evidence, not
measured runtime performance. **Unverified:** Raspberry Pi and conventional x86
physical performance. No timing claims are made for them until measured on real
hardware.

## Reproducing

```sh
./scripts/measure-release.sh
cargo test --workspace --all-targets --all-features
```

Record the printed revision and environment alongside any results. Latencies
are not comparable across devices or build profiles.
