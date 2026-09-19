# Changelog

## 0.1.0 — Unreleased

First public release of the Rust port of Andrea Censi's Canonical Scan Matcher.

- Point-to-line ICP matching over ordered polar and Cartesian scans, with smart
  correspondence search, outlier rejection, visibility and orientation
  handling, restart, and optional weighting.
- Explicit initial poses, caller-selected references, and explicit termination
  reasons.
- Optional closed-form covariance, derivative matrices, and Fisher information.
- Reusable prepared workspaces with zero-allocation pose-only matching after preparation.
