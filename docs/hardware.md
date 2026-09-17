# Hardware validation

Release readiness requires physical-device evidence, not build checks. This
document records what has actually been measured.

## Physically validated: Jetson AGX Xavier

| Item | Value |
| --- | --- |
| Device | NVIDIA Jetson AGX Xavier developer kit |
| Module / carrier | `p2822-0000` + `p2888-0001` (`nvidia,jetson-xavier`, `nvidia,tegra194`) |
| Device tree model | `Jetson-AGX` |
| BSP | L4T R35.6.5 (Jetson Linux) |
| CPU | 8× ARMv8 (Carmel), aarch64 |
| Memory | 30 GiB |
| OS | Ubuntu 20.04 (focal), Linux `5.10.216-tegra` |
| Toolchain | `rustc 1.98.1`, `cargo 1.98.1` |
| Build | `--release`, workspace profile with `lto = true` |

The revision under test is printed by `./scripts/measure-release.sh`; the
latest recorded run is in [measurements.md](measurements.md).

### Correctness evidence

`cargo test --workspace --all-targets --all-features` on the device:

- 129 tests passed, 0 failed, across the library unit tests, the C golden
  corpus, the public capability suite, scan validation, tracing, the prepared
  allocation check, and the demo browser-to-Rust workflow tests.

This includes the exact C golden corpus (poses to 1e-9, covariance to 1e-6
relative), the retained-capability suite, and the full interactive demo
workflow (playback, reference policies, iteration inspection, import/export/
replay).

### Performance evidence

Measured with `./scripts/measure-release.sh`; full table and method in
[measurements.md](measurements.md). Summary, 3,000-point pose-only:
preparation ≈ 3.6 ms, matching mean ≈ 10.2 ms, **p99 ≈ 13.6 ms**. The 3,000-point
covariance/Fisher mode adds ≈ 1.5 ms mean.

### Provisional target assessment

The provisional target — pose-only **p99 < 10 ms for 3,000 points** — is **not
met on this Xavier** (p99 ≈ 12–14 ms across runs). It is recorded as a miss,
and the supported workload is assessed explicitly:

- 20–30 Hz cycles (33–50 ms) are comfortably supported for typical
  2,000–3,000-point scans in both pose-only and covariance modes.
- 10,000-point scans are supported at roughly 8 Hz (≈ 124 ms mean).
- Difficult partial-overlap scans with noise can use the full iteration budget
  (≈ 100 ms); bound `max_iterations` to fit a latency budget.
- The restart shell multiplies cost (≈ 150 ms at 3,000 points); disable it when
  the initial guess is already good.

These numbers are ordinary desktop/Linux figures for this device, not a hard
real-time guarantee. 20–30 Hz is the typical application frequency.

## Build/CI evidence only (not physical performance)

- Linux `x86_64-unknown-linux-gnu` and `aarch64-unknown-linux-gnu` are checked
  by CI (`cargo check`, `cargo test` on an x86 runner). These are build and
  correctness checks, **not** measured runtime performance.
- Raspberry Pi and conventional x86 physical performance are **unverified**.
  No timing claims are made for them until measured on real hardware.

## Reproducing

```sh
./scripts/measure-release.sh
cargo test --workspace --all-targets --all-features
```

Record the printed revision and environment alongside any results. Latencies
are not comparable across devices or build profiles.
