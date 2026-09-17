# csm-rs

[![LGPL-3.0-only Licensed](https://img.shields.io/badge/license-LGPL--3.0--only-brightgreen.svg?style=flat-square)](LICENSE)
![CI](https://github.com/LotfiZ/csm-rs/workflows/CI/badge.svg)

Point-to-line ICP scan matching (Censi, 2007) with smart correspondence search,
outlier rejection, restart handling, and an optional closed-form estimate of the
matching covariance. The library has no runtime dependencies and is built around
ordered scans, validated configuration, matching, and results.

## Requirements

Rust 1.70 or newer. The library itself has no dependencies.

## Installation

The crate is not published on crates.io. Add it as a git dependency:

```toml
[dependencies]
csm-rs = { git = "https://github.com/LotfiZ/csm-rs" }
```

## Quick start

```rust
use csm_rs::{Matcher, Params, PolarScan};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Ordered polar scan: 21 rays from -1 rad to 1 rad.
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 8.0 + 0.2 * a.cos()).collect();
    let valid = vec![true; angles.len()];

    let reference = PolarScan::new(&angles, &readings, &valid)?;
    let sensor = PolarScan::new(&angles, &readings, &valid)?;

    let matcher = Matcher::new(Params::default())?;
    let outcome = matcher.match_polar(reference, sensor)?;
    if outcome.valid {
        println!("sensor-to-reference pose = {:?}", outcome.pose);
    }
    Ok(())
}
```

A self-contained example that simulates a robot in a square room is included:

```sh
cargo run -p csm-rs --example scan_matching
```

## Coordinates and inputs

Inputs use **metres and radians**. Each scan is centered on its own sensor
origin, and rays are **ordered by bearing**: a polar ray at `theta` with reading
`r` is the sensor-frame point `[r cos(theta), r sin(theta)]`. Cartesian scans are
supported as well; their bearings are derived as `atan2(y, x)` unless supplied
with `CartesianScan::with_angles`. The matcher relies on that ordering and never
sorts or drops points, so unordered point-cloud registration is out of scope.

The result maps sensor-scan coordinates into reference-scan coordinates:
`R(theta) * p + (x, y)`, with `theta` counter-clockwise.

A missing lidar return is marked with `valid[i] == false` and keeps its position
in the scan order. Malformed input returns a `ScanError`. A well-formed pair that
cannot be matched is a normal outcome with `valid == false`; inspect
`MatchOutcome::termination` for the reason.

## Initial pose and reference

`match_polar`/`match_cartesian` start from the identity pose. Pass an explicit
guess, for example from odometry, with `match_polar_from`/`match_cartesian_from`:

```rust
use csm_rs::{Matcher, Params, Pose};

let matcher = Matcher::new(Params::default())?;
let guess = Pose::new(0.10, -0.05, 0.02);
let outcome = matcher.match_polar_from(reference, sensor, guess)?;
```

The reference scan is always chosen by the caller.

## Uncertainty and reusable storage

Matching is pose-only by default. Enable the closed-form covariance, derivative
matrices, and Fisher information with `Params::do_compute_covariance`; the result
reports uncertainty status independently of whether the pose is usable.

For fixed-rate applications, build a `PreparedMatcher` once and update its frames
in place:

```rust
let matcher = Matcher::default_pose_only();
let mut workspace = matcher.prepare(reference, sensor)?;
workspace.update_sensor(&next_readings, &next_valid)?;
let outcome = workspace.match_once()?;
```

Preparation allocates all scan and scratch storage; repeated pose-only matching
afterwards performs no heap allocation. Capacity is explicit: inputs larger than
the reserved capacity return `ScanError::CapacityExceeded` instead of growing
mid-match. Grow it with `PreparedMatcher::reserve`.

## Performance

Measured on a Jetson AGX Xavier (8× ARMv8, `--release`), synthetic ordered polar
scans. Latency excludes preparation; measure on your own hardware before
relying on these figures.

| scan | mode | mean | p99 |
| ---: | --- | ---: | ---: |
| 2,048 rays | pose only | 5.4 ms | 7.3 ms |
| 3,000 rays | pose only | 10.2 ms | 13.6 ms |
| 3,000 rays | with covariance | 11.7 ms | 14.4 ms |
| 10,000 rays | pose only | 124 ms | 129 ms |

Typical 2,000–3,000-point scans support 20–30 Hz cycles. Larger scans scale
super-linearly. These are ordinary Linux figures, not a hard real-time
guarantee.

## Interactive demo

A local browser demo runs the real matcher and shows the reference, unaligned,
and aligned scans:

```sh
cargo run -p csm-rs-demo --release
```

Then open <http://127.0.0.1:7878>. See [demo/README.md](demo/README.md).

## Testing

```sh
cargo test
```

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

## License

csm-rs is distributed under the **LGPL-3.0** license, the same as Andrea
Censi's Canonical Scan Matcher, from which it derives. See [LICENSE](LICENSE).
