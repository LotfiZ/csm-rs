#!/usr/bin/env bash
set -euo pipefail

cargo build --release -p csm-rs --examples
echo "release example sizes (bytes):"
find target/release/examples -maxdepth 1 -type f -perm -111 -printf '%f %s\n' | sort
echo "prepared steady-state benchmark:"
cargo run --release -p csm-rs --example benchmark_prepared
