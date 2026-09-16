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
expected results) checked into `crates/csm-rs/tests/fixtures/`. The 16-case
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

Install Rust, then run the complete fixture suite from the repository root:

```sh
cargo test --all-targets --all-features
```

For an application, construct each scan from angles, readings, and validity
flags. The matcher returns an error for malformed scan storage and puts a
normal convergence failure in `result.valid`:

```rust
use csm_rs::{sm_icp, LaserData, Params, SmResult};

fn match_scans(
    angles: Vec<f64>,
    reference_readings: Vec<f64>,
    sensor_readings: Vec<f64>,
    valid: Vec<bool>,
) -> Result<SmResult, Box<dyn std::error::Error>> {
    let mut reference = LaserData::from_polar(
        angles.clone(),
        reference_readings,
        valid.clone(),
    )?;
    let mut sensor = LaserData::from_polar(angles, sensor_readings, valid)?;
    let mut result = SmResult::default();
    sm_icp(&Params::default(), &mut reference, &mut sensor, &mut result)?;
    Ok(result)
}
```

`result.valid == true` means the scans matched. `false` means the inputs were
well-formed but ICP did not produce a usable match. A `?` error means the
input arrays or scan values violate CSM's input contract. Invalid lidar rays
should have `valid[i] == false`; their reading can be `NaN`.

For a beginner-friendly example that creates laser data, prints every beam in
a readable table, and displays the estimated movement, run:

```sh
cargo run -p csm-rs --example scan_matching
```

The example simulates a robot scanning a square room. It needs no input files,
extra dependencies, or C installation. Change `FIRST_SENSOR_POSE` in
`crates/csm-rs/examples/scan_matching.rs` to try another small movement.

Performance baselines are dependency-free and reproducible in release mode:

```sh
cargo run --release -p csm-rs --example benchmark_baseline
cargo run --release -p csm-rs --example benchmark_prepared
```

The prepared benchmark reports both full covariance mode and the pose-only
mode, allowing deployments to measure the cost of uncertainty outputs on their
own hardware.

The idiomatic API borrows application buffers and returns a typed outcome:

```rust
use csm_rs::{Matcher, Params, PolarScan};

let reference = PolarScan::new(&angles, &reference_readings, &valid)?;
let sensor = PolarScan::new(&angles, &sensor_readings, &valid)?;
let outcome = Matcher::new(Params::default()).match_polar(reference, sensor)?;
if outcome.converged() {
    println!("pose = {:?}", outcome.pose);
}
```

The legacy `sm_icp` function remains available for conformance tooling and
existing callers. New integrations should use `Matcher::prepare_polar`,
`Matcher::prepare_cartesian`, or `Matcher::prepare` so scan ownership and
workspace reuse are explicit.

For fixed-rate applications, create `PreparedPolarScan` values once and reuse
them with `match_prepared` or `match_prepared_into`. Ordered Cartesian points
are also accepted through `CartesianScan`. To generate a browser-viewable SVG
demonstration, run:

```sh
cargo run --release -p csm-rs --example visual_match > match.svg
```

Prepared scans can be refreshed in place with `update` or
`update_cartesian`, retaining their allocation capacity. The
`match_prepared_observed` method invokes a caller-supplied closure with each
outcome, which is suitable for metrics, logging, or a UI adapter without a
runtime logging dependency.

For a fixed-shape stream, `Matcher::prepare` retains the scan and ICP
workspace across frames:

```text
let mut workspace = Matcher::pose_only(Params::default()).prepare(reference, sensor)?;
workspace.update_sensor(&next_readings, &next_valid)?;
let estimate = workspace.match_once()?;
# Ok::<(), csm_rs::LaserDataError>(())
```

When covariance is not needed, `Matcher::pose_only(Params::default())`
disables the optional covariance and derivative calculations for a smaller
embedded runtime path.

Use `Matcher::try_new(params)` when configuration comes from a file or another
runtime source; it validates finite values, ranges, and the iteration limit
before the matcher is constructed.

Regenerate the corpus with the C reference source checked out at
`/home/agx/workspace/csm-src`:

```sh
./fixture-generator/build.sh /home/agx/workspace/csm-src \
  crates/csm-rs/tests/fixtures/identity.json
```

## Workspace layout

- `crates/csm-rs` — the pure library (no robotics-framework dependencies)
- `fixture-generator/` — throwaway C tool producing the JSON fixtures
- *(parked)* `crates/csm-horus-node` — [HORUS](https://horusrobotics.dev) node
  wrapper, to be designed once the library validates

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
