# csm-rs local demo

A local browser demonstration that runs the **real `csm-rs` matcher** for every
frame request. It ray-casts ordered scans from a moving sensor, matches them
against a reference scan, and draws the reference, unaligned, and aligned
scans together with the true and estimated motion.

## Launch

From the repository root:

```sh
cargo run -p csm-rs-demo --release
```

Then open <http://127.0.0.1:7878>. Pass an address as the first argument to use
another interface or port:

```sh
cargo run -p csm-rs-demo --release -- 0.0.0.0:8080
```

## HTTP framework choice

The demo uses [axum](https://github.com/tokio-rs/axum) 0.8 on Tokio. axum is
actively maintained, targets Linux ARM and x86, and has a small routing/serving
surface that fits a local, single-process demo without a templating or asset
pipeline. Its dependencies live only in this package; the `csm-rs` library
itself has none.

## Design

- The server is **stateless**: every `POST /api/frame` regenerates the seeded
  scans for the requested step and runs the matcher. Playback is client-side.
- Play, pause, single-step, and reset are explicit client states; stepping or
  resetting pauses playback first.
- Motion, noise, dropout, and initial-guess error are independent controls that
  change the actual scan geometry, readings, and initial pose sent to the
  matcher.
- Scan generation uses a deterministic xorshift generator so a recorded seed,
  step, and control set replay exactly.
- The browser drops stale responses using the request id, so an older frame
  cannot overwrite a newer one.
- Public hosting and WebAssembly are intentionally out of scope.

## Out of scope

Direct sensor connections, ROS integration, log-format support, and public
hosting.
