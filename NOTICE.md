# csm-rs attribution and license provenance

csm-rs is a Rust derivative work of Andrea Censi's Canonical Scan Matcher
(CSM), plus its gpc closed-form solver.

## Components

- CSM scan matching, correspondence, ICP, and covariance algorithms derive
  from Andrea Censi's CSM project. The upstream project identifies this
  portion as LGPL-3.0.
- The gpc solver derives from Andrea Censi's sm/lib/gpc sources. The upstream
  source headers identify those files as GPL-2.0-or-later.
- The repository also contains a throwaway C fixture generator and checked-in
  test fixtures. They are development artifacts and are not included in the
  published library package.

## Current distribution license

The combined Rust crate is declared **GPL-2.0-or-later**. This reflects the
GPL-derived solver included in the crate. The accompanying LICENSE-GPL-2.0 and
LICENSE-LGPL-3.0 files are retained for complete upstream attribution and
license notices.

This notice records source provenance; it is not legal advice. Before the first
public release, review the complete source history and any additional upstream
notices, then confirm the package metadata and release contents.
