# Contributing

Run commands from the repository root:

```sh
cargo check --workspace --all-targets
cargo test --workspace --all-targets --all-features
cargo test --doc
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo doc --workspace --no-deps
cargo package -p csm-rs --allow-dirty
./scripts/check-package.sh
```

## Numerical reference

The pre-remaster Rust implementation is preserved at revision
`27f1c24714ebb717410f0a69e87dff7ec970970c`. Inspect it with
`git show 27f1c24714ebb717410f0a69e87dff7ec970970c:crates/csm-rs/src/lib.rs`
or check it out in a separate worktree for comparisons.

The remaster preserves mathematical intent and supported capabilities;
this revision is numerical evidence, not a permanent API or architecture constraint.
Keep intentional numerical deviations documented with their validation evidence.

The C-generated corpus remains in `tests/fixtures/identity.json`. Tests run
offline without C dependencies. To regenerate using an upstream CSM checkout:

```sh
./fixture-generator/build.sh /path/to/csm tests/fixtures/identity.json
```

The generator is separate from Cargo and excluded from the distributed package.
Its build script documents required tools and upstream build settings. Preserve
module-level C cross-references, algorithm references, NOTICE.md, and both
license texts when changing implementation or packaging.
