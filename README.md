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
explicit max-iteration exhaustion case. The public tracer path checks pose to 1e-9,
`iterations`/`nvalid` exactly, and the configured first-iteration
correspondence hash exactly. The crate-internal seam checks that tricks and
naive correspondence search produce the same keys and hash on the synthetic
common path; the imported `stallo2` log records the known C smart/naive
divergence on invalid sectors. The covariance fixture checks the closed-form
result and its derivative matrices to 1e-6 relative error. This corpus and
these checks are the reviewed sign-off for the acceptance requirements in
spec #1. Match errors use 1e-9 for synthetic cases and 2e-9 for imported
logs to account for their different native math paths.

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

Derivative work of CSM (LGPLv3) and its vendored `gpc` solver (GPLv2+).
This port is therefore dual-licensed **LGPL-3.0-only OR GPL-2.0-or-later** —
see `LICENSE-LGPL-3.0` and `LICENSE-GPL-2.0`.
