# csm-rs

A Rust implementation of the **Canonical Scan Matcher** ([Andrea Censi's CSM](https://github.com/AndreaCensi/csm)):
point-to-line ICP with smart correspondence search, outlier rejection, restart
handling, and an optional closed-form estimate of the matching covariance
([Censi, ICRA 2007](https://purl.org/censi/2007/icpcov)).

The library has no runtime dependencies and is organized around ordered scans,
validated configuration, matching, and results. It supports ordered polar and
Cartesian inputs, explicit initial poses, reusable allocation-free workspaces,
optional covariance/derivative/Fisher-information outputs, and explicit
termination reasons.

## Installation

Add the crate to a Cargo project:

```toml
[dependencies]
csm-rs = { git = "https://github.com/LotfiZ/csm-rs" }
```

Then run the complete test suite from the repository root:

```sh
cargo test --all-targets --all-features
```

## Scope

Supported: the `sm_icp` path (ICP/PlICP with correspondence search, outlier
rejection, orientation/visibility handling, weighting, restart, covariance,
and Fisher information). Not supported: GPM/HSM/MbICP, Cairo drawing, the CLI
apps, arbitrary unordered point clouds, public hosting/WebAssembly, and
microcontroller or `no_std` targets. Port history and the pre-remaster
numerical baseline are in [docs/contributing.md](docs/contributing.md).

## Quick start

Install Rust, then run the complete test suite from the repository root:

```sh
cargo test --all-targets --all-features
```

The supported interface is arranged around scans, configuration, matching, and
results. A minimal match borrows caller-owned buffers and uses an identity
initial pose:

```rust
use csm_rs::{Matcher, Params, PolarScan};

fn match_scans(
    angles: &[f64],
    reference_readings: &[f64],
    sensor_readings: &[f64],
    valid: &[bool],
) -> Result<(), Box<dyn std::error::Error>> {
    let reference = PolarScan::new(angles, reference_readings, valid)?;
    let sensor = PolarScan::new(angles, sensor_readings, valid)?;
    let outcome = Matcher::new(Params::default())?.match_polar(reference, sensor)?;
    if outcome.valid {
        println!("sensor-to-reference pose = {:?}", outcome.pose);
    }
    Ok(())
}
```

`outcome.valid == true` means the scans matched; `false` means the inputs were
well-formed but ICP did not produce a usable match. A `?` error means the
input arrays or scan values violate the input contract. Missing lidar returns
should have `valid[i] == false`; they keep their position in the scan order,
and their reading can be `NaN`.

### Coordinate contract

Inputs use **metres and radians**. Each scan is centered on its own sensor
origin, and rays are **ordered by bearing**: a polar ray at `theta` with
reading `r` is the sensor-frame point `[r cos(theta), r sin(theta)]`, and a
Cartesian ray is its own `[x, y]`. Cartesian bearings are derived as
`atan2(y, x)` unless supplied with `CartesianScan::with_angles` (for example a
differently mounted scanner). The matcher relies on that ordering and never
sorts or drops points; unordered point-cloud registration is out of scope. The
result is the rigid transform mapping sensor-scan coordinates into
reference-scan coordinates: `R(theta) * p + (x, y)` with counter-clockwise
`theta`. [`Pose`] documents composition.

### Initial pose and reference selection

`match_polar` and `match_cartesian` use the identity initial pose. Pass an
explicit guess (for example from odometry) with `match_polar_from` or
`match_cartesian_from`:

```rust
use csm_rs::{Matcher, Params, PolarScan, Pose};

let matcher = Matcher::new(Params::default())?;
let guess = Pose::new(0.10, -0.05, 0.02);
let outcome = matcher.match_polar_from(reference, sensor, guess)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

The reference scan is always chosen explicitly by the caller; the library
never replaces it implicitly.

### Uncertainty

Matching defaults to pose-only. Request the closed-form covariance and
derivative matrices explicitly:

```rust
use csm_rs::{Matcher, Params};

let params = Params { do_compute_covariance: true, ..Params::default() };
let matcher = Matcher::new(params)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

`MatchOutcome::covariance_status` reports disabled, computed, or failed
uncertainty independently of whether the pose itself is usable. Successful
uncertainty outputs are the closed-form `covariance`, the `dx_dy_reference`
and `dx_dy_sensor` derivative matrices, and the `fisher_information` Hessian of
the point-to-line objective. Uncertainty failure never invalidates an
otherwise usable pose.

For reusable storage, reserve the outcome's derivative matrices once with
`MatchOutcome::reserve_uncertainty` and match into it with
`PreparedMatcher::match_into`; covariance and derivative matching then perform
no heap allocation after preparation. Preparation costs and failure semantics
are documented on [`MatchOutcome`] and [`PreparedMatcher`].

### Reusable storage

For fixed-rate applications, build a `PreparedMatcher` once and update its
frames in place:

```text
let matcher = Matcher::default_pose_only();
let mut workspace = matcher.prepare(reference, sensor)?;
workspace.update_sensor(&next_readings, &next_valid)?;
let estimate = workspace.match_once()?;
# Ok::<(), csm_rs::ScanError>(())
```

`Matcher::prepare_polar`, `Matcher::prepare_cartesian`, and `Matcher::prepare`
make scan ownership and workspace reuse explicit. Preparation allocates the
reference and sensor storage, the ICP scratch buffers, and the optional
orientation/visibility buffers. After that, repeated pose-only matching —
including input updates, restarts, retained search/outlier/weighting options,
and unsuccessful outcomes — performs **zero heap allocations**.

`PreparedMatcher::capacities` reports reserved rays, and `workspace_bytes`
reports the scratch footprint. Grow capacity explicitly with
`PreparedMatcher::reserve`; an input larger than the reserved capacity returns
`ScanError::CapacityExceeded` rather than growing during a match. Reference and
sensor scans may have different sizes, and sizes above 3,000 points are
supported by sizing the workspace accordingly.

### Diagnostics

`MatchOutcome::termination` reports the algorithm's actual stop reason:
convergence, iteration exhaustion, too few correspondences, insufficient usable
geometry, cycle detection, numerical failure, or another failure.
`MatchOutcome::accepted()` distinguishes an accepted convergence from a
candidate produced by an unsuccessful termination, and
`MatchOutcome::candidate()` exposes the candidate pose (or `None` when no
candidate exists). Iteration counts, correspondence counts, and residual error
are preserved after unsuccessful termination.

A well-formed scan pair with insufficient usable geometry is an outcome, not a
malformed-input error. `PreparedMatcher::match_once_traced` reports real ICP
iteration snapshots (pose, error, valid correspondences, restart context, and
the contributing correspondences in reference-frame coordinates) for
inspection; instrumented timing is separate from ordinary matching and tracing
does not change the result.

### Examples

A beginner-friendly example creates laser data, prints every beam, and shows
the estimated movement:

```sh
cargo run -p csm-rs --example scan_matching
```

The example simulates a robot scanning a square room. It needs no input files,
extra dependencies, or C installation. Change `FIRST_SENSOR_POSE` in
`examples/scan_matching.rs` to try another small movement.

The prepared benchmark reports both pose-only and uncertainty modes in release
mode:

```sh
cargo run --release -p csm-rs --example benchmark_prepared
```

For a repeatable release resource report (optimized example sizes plus the
prepared latency benchmark), run `scripts/measure-release.sh`.

## Performance and measurements

Resource measurements are reproducible and separated from correctness tests:

```sh
./scripts/measure-release.sh          # environment, build sizes, latency/allocation/memory/alignment
cargo run --release -p csm-rs --example measure
```

The report measures preparation and matching separately, reports latency
percentiles (including p99), counts steady-state heap allocations, reports
memory and alignment quality, and records the seed, hardware, toolchain, build
settings, revision, and commands. See [docs/measurements.md](docs/measurements.md)
for the method, the latest recorded run, and the assessment against the
provisional target. Physical validation on a Jetson AGX Xavier, the supported
workloads, and the explicit list of unverified hardware are recorded in
[docs/hardware.md](docs/hardware.md). No claim of outperforming the C
implementation is made without comparable measurements.

## Capability coverage

The retained CSM capabilities are covered by two reproducible suites:

- `src/golden_tests.rs` (run by `cargo test --lib`) checks exact agreement with
  the C reference corpus: poses to 1e-9, iteration/correspondence counts,
  correspondence hashes, covariance and derivative matrices to 1e-6 relative
  error, and the known smart/naive divergence on the `stallo2` log.
- `tests/capabilities.rs` (run by `cargo test --test capabilities`) exercises
  the public API for correspondence strategies, point/line metrics, outlier
  rejection, orientation and visibility filtering, ML/sigma weighting, restart,
  covariance/derivatives/Fisher information, degenerate geometry, and partial
  overlap.

Run everything with:

```sh
cargo test --all-targets --all-features
```

### Intentional numerical deviations

The remaster preserves mathematical intent and feature coverage rather than
requiring exact C agreement everywhere. Known deviations are:

- **Smart vs naive on `stallo2`.** CSM's jump-table and naive searches diverge
  on that log's invalid sectors; both configured C paths are retained and
  tested individually instead of hiding the divergence.
- **Deliberate solver changes.** Where a change improves clarity, correctness,
  or resource use, it is validated against the golden corpus, known transforms,
  and degenerate geometry rather than against the C source alone. No claim of
  outperforming C is made without comparable measurements.

Regenerate the corpus with the C reference source checked out at
`/path/to/csm-source`:

```sh
./fixture-generator/build.sh /path/to/csm-source \
  tests/fixtures/identity.json
```

## Workspace layout

- `src/` — the core library (no runtime dependencies)
- `tests/` — integration tests and C reference fixtures
- `examples/` — runnable examples and measurement programs
- `demo/` — local browser demonstration (axum + Tokio, dependencies isolated)
- `fixture-generator/` — C tool producing the reference fixtures
- `docs/contributing.md` — contributor commands and numerical baseline
- `docs/measurements.md` — reproducible resource measurements
- `docs/hardware.md` — physical-device validation evidence
- `docs/release-readiness.md` — release-criteria verification

Launch the demo with `cargo run -p csm-rs-demo --release`; see `demo/README.md`.

## License

The scan, correspondence, ICP, and covariance modules are derivative work of
Andrea Censi's Canonical Scan Matcher (CSM), released under LGPLv3. The
closed-form solver in `solver.rs` is derivative work of Andrea Censi's
vendored `gpc` solver, released under GPLv2-or-later. The checked-in fixture
generator is a separate throwaway C tool and is not part of the Rust crate.

The current Rust crate is distributed under **GPL-2.0-or-later** because it
contains a direct port of the upstream gpc solver, whose source is
GPL-2.0-or-later. The remaining CSM-derived algorithm is LGPL-3.0, and its
license text is included for attribution. See NOTICE.md for component-level
provenance. The crate cannot claim an LGPL-only option while the GPL-derived
solver remains part of the combined work. A future clean-room solver may enable
a different license declaration; that would require a separate provenance
review.
