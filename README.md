# csm-rs

Rust port of the **Canonical Scan Matcher** ([Andrea Censi's CSM](https://github.com/AndreaCensi/csm)):
point-to-line ICP with smart correspondence search, outlier rejection, and a
closed-form estimate of the matching covariance
([Censi, ICRA 2007](https://purl.org/censi/2007/icpcov)).

## Status

Early scaffold. Port scope is the `sm_icp` path only (ICP/PlICP + covariance);
GPM/HSM/MbICP, Cairo drawing, and the CLI apps are intentionally excluded.

Design decisions were captured in a structured grilling session — see module
docs for the C cross-references and per-decision rationale.

## Validation strategy

The port is validated *golden-master* against the original C library:
a throwaway C generator (in `fixture-generator/`, never built by cargo) links
the reference implementation and emits JSON fixtures (scan pairs + params +
expected results) checked into `crates/csm-rs/tests/fixtures/`. Tolerances:
pose 1e-9, iterations/nvalid exact, covariance 1e-6 relative, plus
first-iteration correspondence-hash equality.

## Workspace layout

- `crates/csm-rs` — the pure library (no robotics-framework dependencies)
- `fixture-generator/` — throwaway C tool producing the JSON fixtures
- *(parked)* `crates/csm-horus-node` — [HORUS](https://horusrobotics.dev) node
  wrapper, to be designed once the library validates

## License

Derivative work of CSM (LGPLv3) and its vendored `gpc` solver (GPLv2+).
This port is therefore dual-licensed **LGPL-3.0-only OR GPL-2.0-or-later** —
see `LICENSE-LGPL-3.0` and `LICENSE-GPL-2.0`.
