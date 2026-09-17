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

The previous Rust implementation is preserved at revision
`27f1c24714ebb717410f0a69e87dff7ec970970c`. Inspect it with
`git show 27f1c24714ebb717410f0a69e87dff7ec970970c:crates/csm-rs/src/lib.rs`
or check it out in a separate worktree for comparison.

The C-generated corpus lives in `tests/fixtures/identity.json`; tests run
offline without C dependencies. Regenerate it against an upstream CSM checkout:

```sh
./fixture-generator/build.sh /path/to/csm tests/fixtures/identity.json
```

The generator is a standalone C tool, separate from Cargo and excluded from the
distributed package; its build script documents the required tools and upstream
build settings. Keep the module-level C cross-references, NOTICE.md, and both
license texts when changing implementation or packaging.
