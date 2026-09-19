# Contributing

Issues and small pull requests are welcome. This project is experimental;
support and review are best effort, without a guaranteed response schedule.
Discuss substantial changes in an issue before starting implementation.

For bug reports, include the revision, toolchain, reproduction steps, expected
result, and actual result. For matching problems, include a minimal scan pair
and matcher configuration when possible. Share only data you can make public.

Keep pull requests focused and explain the behavior they change. Add regression
tests for behavior changes and update relevant documentation.

Run these checks from the repository root:

```sh
cargo fmt --all -- --check
cargo test --workspace --all-targets --all-features
cargo test --doc
cargo clippy --workspace --all-targets --all-features -- -D warnings
```

## Preparing a release

Keep the upcoming version marked unreleased in `CHANGELOG.md` until release day.
Run the checks above, build documentation with warnings denied, and verify the
package before selecting the release commit. Date the changelog when releasing.
The companion demo should then pin that exact library commit in its manifest,
lockfile, and saved-experiment version metadata before its own release.
