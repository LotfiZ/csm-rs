#!/usr/bin/env bash
set -euo pipefail

files="$(cargo package --list --allow-dirty --no-verify)"
for forbidden in 'tests/fixtures/' 'fixture-generator/'; do
    if grep -Fq "$forbidden" <<<"$files"; then
        echo "forbidden package entry: $forbidden" >&2
        exit 1
    fi
done
echo "package contents are release-safe"
