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
and feature paths. The imported `stallo2` log records a known C behavior:
CSM's smart and naive searches diverge on that scan's invalid sectors, so the
Rust port keeps and tests each C path instead of hiding the divergence. The
covariance fixture checks the closed-form result and its derivative matrices
to 1e-6 relative error. Match errors use 1e-9 for synthetic cases and 2e-9
for imported logs to account for their different native math paths.

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

Regenerate the corpus with the C reference source checked out at
`/path/to/csm-source`:

```sh
./fixture-generator/build.sh /path/to/csm-source \
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

The Rust crate is therefore dual-licensed **LGPL-3.0-only OR
GPL-2.0-or-later** — see `LICENSE-LGPL-3.0` and `LICENSE-GPL-2.0`.
