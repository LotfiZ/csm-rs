<div align="center">

<img src="assets/logo.svg" alt="csm-rs logo" width="320">

[![LGPL-3.0-only Licensed](https://img.shields.io/badge/license-LGPL--3.0--only-brightgreen.svg?style=flat-square)](LICENSE)
![CI](https://github.com/LotfiZ/csm-rs/workflows/CI/badge.svg)

</div>

A Rust port of Andrea Censi's [Canonical Scan Matcher](https://github.com/AndreaCensi/csm)
for point-to-line ICP matching of ordered 2D laser scans. The library provides
validated scan inputs, explicit matching outcomes, and optional uncertainty
estimation, with no runtime dependencies.

Experimental: APIs may change before 1.0. Support is best effort.

## Requirements

Rust 1.70 or newer. The library itself has no dependencies.

## Installation

Add the library as a git dependency:

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

## Features

- Ordered polar and Cartesian scans, with validated inputs and configuration.
- Point-to-line ICP with smart correspondence search, outlier rejection,
  visibility and orientation handling, restarts, and optional weighting.
- Explicit initial poses, caller-selected reference scans, and termination reasons.
- Optional closed-form covariance, derivative matrices, and Fisher information.
- Reusable prepared workspaces with allocation-free repeated pose-only matching.

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

## Interactive demo

Explore generated scans, matching results, and iteration traces in the companion
[csm-rs-demo](https://github.com/LotfiZ/csm-rs-demo). It runs locally in a browser
and has its own dependencies and release cycle.

## Testing

```sh
cargo test
```

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

## Contributing

Issues and small pull requests are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md)
for checks and bug-report details.

## Credits and license

This Rust implementation derives from Andrea Censi's
[Canonical Scan Matcher](https://github.com/AndreaCensi/csm), the original C
implementation of the point-to-line ICP algorithm. Credit for the original
algorithm and implementation belongs to its authors.

Distributed under **LGPL-3.0-only**. See [LICENSE](LICENSE).
