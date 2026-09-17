# Changelog

## 0.1.0

Initial release.

- Point-to-line ICP matching over ordered polar and Cartesian scans, with smart
  correspondence search, outlier rejection, visibility and orientation
  handling, restart, and optional weighting.
- Explicit initial poses, caller-selected references, and explicit termination
  reasons.
- Optional closed-form covariance, derivative matrices, and Fisher information.
- Reusable prepared workspaces with zero-allocation matching after preparation.
- A local browser demo package (`csm-rs-demo`).
