# csm-rs

Rust port of the **Canonical Scan Matcher** ([Andrea Censi's CSM](https://github.com/AndreaCensi/csm)):
point-to-line ICP with smart correspondence search, outlier rejection, and a
closed-form estimate of the matching covariance
([Censi, ICRA 2007](https://purl.org/censi/2007/icpcov)).

## Status

Port scope is the `sm_icp` path only (ICP/PlICP + covariance); GPM/HSM/MbICP,
Cairo drawing, and the CLI apps are intentionally excluded.

Design decisions were captured in a structured grilling session — see module
docs for the C cross-references and per-decision rationale.

## Validation strategy

The port is validated *golden-master* against the original C library:
a throwaway C generator (in `fixture-generator/`, never built by cargo) links
the reference implementation and emits JSON fixtures (scan pairs + params +
expected results) checked into `tests/fixtures/`. The 16-case
corpus covers the synthetic baseline, covariance, trimming, duplicate,
restart, oscillation, feature, and weighting paths, plus the upstream
`misc/tests` logs, a three-point collinear degenerate geometry case, and an
explicit max-iteration exhaustion case. The public tracer path checks pose to
1e-9, `iterations`/`nvalid` exactly, and the configured first-iteration
correspondence hash exactly. The crate-internal seam checks that tricks and
naive correspondence search produce the same keys and hash on the synthetic
common path. Alpha-enabled fixtures validate the configured C path; CSM's
smart routine intentionally omits the optional alpha filter. The imported
`stallo2` log records another known C behavior: CSM's smart and naive searches
diverge on that scan's invalid sectors, so the Rust port keeps and tests each
C path instead of hiding the divergence. The covariance fixture checks the
closed-form result and its derivative matrices to 1e-6 relative error. Match
errors use 1e-9 for synthetic cases and 2e-9 for imported logs to account for
their different native math paths.

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
iteration snapshots (pose, error, and valid correspondences) for inspection.

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

Regenerate the corpus with the C reference source checked out at
`/path/to/csm-source`:

```sh
./fixture-generator/build.sh /path/to/csm-source \
  tests/fixtures/identity.json
```

## Workspace layout

- `src/` — the core library (no robotics-framework dependencies)
- `tests/` — integration tests and C reference fixtures
- `examples/` — runnable examples and measurement programs
- `fixture-generator/` — C tool producing the reference fixtures
- `docs/contributing.md` — contributor commands and numerical baseline

The separate local demo package will be introduced as part of the remaster.

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
