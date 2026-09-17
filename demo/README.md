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

## Reference policies and scenarios

- **Fixed reference** matches every frame against the initial scan, so errors do
  not accumulate.
- **Previous frame** replaces the reference each step and composes the relative
  estimates; per-frame errors accumulate as drift, which the UI reports next to
  the true pose.

Three scenarios exercise different failure modes: an **asymmetric room**, an
**ambiguous corridor** whose straight walls weakly constrain yaw, and a
**partial-overlap** room split by a doorway. Motion, noise, dropout, and
initial-guess error are independent controls, and advanced matching
configuration is exposed progressively behind a details panel and passed to
the real matcher.

## Iteration inspection

Tracing is opt-in. Enable **iteration inspection** in the advanced panel to
receive the real matcher iterations: each carries the pose update, residual
error, valid-correspondence count, whether it came from a restart perturbation,
and the contributing correspondences in reference-frame coordinates. Use the
iteration slider to step through them; correspondence lines are coloured by
distance.

Instrumented timing is reported separately from an uninstrumented prepared
match (`normal / instrumented`), and tracing runs the same matching path, so
traced and untraced poses agree. Per-iteration correspondence storage is
allocated only when tracing is enabled.

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

## Import, export, and replay

Imported data has **no ground truth**: responses show the matcher result and
never fabricate a true pose.

### Import format (`csm-rs-scan-pair`, version 1)

```json
{
  "format": "csm-rs-scan-pair",
  "version": 1,
  "initial_guess": [0.0, 0.0, 0.0],
  "reference": {
    "kind": "polar",
    "angles": [-1.0, -0.9, ...],
    "readings": [5.0, null, ...],
    "valid": [true, false, ...],
    "sigma": null,
    "true_alpha": null
  },
  "sensor": {
    "kind": "cartesian",
    "points": [[8.0, 0.0], ...],
    "valid": [true, ...],
    "angles": null
  },
  "config": { "max_iterations": 200 }
}
```

- Polar rays are ordered by bearing in radians; readings are metres; `null`
  marks a missing return and `valid` must be `false` there. A `null` reading on
  a valid ray is rejected.
- Cartesian points are ordered by bearing; `angles` may supply explicit
  bearings, otherwise `atan2(y, x)` is derived.
- `sigma` and `true_alpha` are optional per-ray arrays for the weighting paths.
- `config` optionally overrides matching settings; simulation-only fields are
  ignored.

Malformed input returns HTTP 400 with a clear message. The **sample** button
loads a minimal valid document.

### Export and replay

**Export** packages a versioned session containing the generated scans, the
initial guess, the matching configuration, the simulation seed, and the
matching result. **Replay** reruns the matcher on the stored scans and compares
its result with the stored one under the same reproducibility rules; an
unsupported version is rejected clearly.

## Out of scope

Direct sensor connections, ROS integration, log-format support, and public
hosting.
