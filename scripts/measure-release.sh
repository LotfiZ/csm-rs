#!/usr/bin/env bash
# Reproducible release resource report: environment, build sizes, and the
# latency/allocation measurements. Run from the repository root.
set -euo pipefail

echo "== environment =="
date -u +"%Y-%m-%dT%H:%M:%SZ"
uname -a
rustc -vV
cargo -vV
echo "logical cpus: $(nproc)"
if git rev-parse --short HEAD >/dev/null 2>&1; then
    echo "revision: $(git rev-parse HEAD)"
fi
if [[ -r /proc/device-tree/model ]]; then
    echo "device: $(tr -d '\0' </proc/device-tree/model)"
fi
if [[ -r /etc/nv_tegra_release ]]; then
    cat /etc/nv_tegra_release
fi

echo
echo "== build =="
# Remove stale artifacts so only current examples are measured.
cargo clean -p csm-rs --release
rm -rf target/release/examples
cargo build --release -p csm-rs --examples
cargo build --release -p csm-rs-demo

echo
echo "== build sizes (bytes) =="
find target/release/examples -maxdepth 1 -type f -perm -111 -printf '%f %s\n' | sort
for artifact in target/release/libcsm_rs.rlib target/release/csm-rs-demo; do
    if [[ -f "$artifact" ]]; then
        printf '%s %s\n' "$(basename "$artifact")" "$(stat -c %s "$artifact")"
    fi
done
for rlib in target/release/deps/libcsm_rs-*.rlib; do
    [[ -f "$rlib" ]] && printf 'libcsm_rs.rlib %s\n' "$(stat -c %s "$rlib")"
done

echo
echo "== latency, allocation, memory, and alignment =="
cargo run --release -p csm-rs --example measure
