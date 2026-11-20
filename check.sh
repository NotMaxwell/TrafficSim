#!/usr/bin/env bash
# Quality gate: every milestone must pass all three.
set -euo pipefail
cd "$(dirname "$0")"
export RUSTFLAGS="${RUSTFLAGS:-} -D warnings"
cargo build --release --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --release
echo "check.sh: all gates passed"
