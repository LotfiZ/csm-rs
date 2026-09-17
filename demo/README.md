# csm-rs demo

A local browser demo that runs the real `csm-rs` matcher on every frame. It
ray-casts ordered scans from a moving sensor and draws the reference, unaligned,
and aligned scans with the true and estimated motion.

## Run

From the repository root:

```sh
cargo run -p csm-rs-demo --release
```

Then open <http://127.0.0.1:7878>. Pass an address to use another interface or
port:

```sh
cargo run -p csm-rs-demo --release -- 0.0.0.0:8080
```

## Features

- Play, pause, single-step, and reset, with independent motion, noise, dropout,
  and initial-guess controls. A seed replays a run exactly.
- Fixed-reference and previous-frame matching policies; the previous-frame
  policy reports accumulated drift.
- Three scenarios (asymmetric room, ambiguous corridor, partial overlap) and a
  progressive advanced-configuration panel.
- Opt-in iteration inspection showing real matcher iterations and
  correspondences, with instrumented timing reported separately.
- Import of ordered polar/Cartesian scan pairs, versioned session export, and
  replay.

The server is stateless: every `POST /api/frame` regenerates the seeded scans
for the requested step and runs the matcher. Imported data has no ground truth,
so responses show the matcher result only. Direct sensor connections, ROS
integration, and public hosting are out of scope.
