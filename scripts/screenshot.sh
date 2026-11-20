#!/usr/bin/env bash
# Capture the app's own framebuffer (not the screen) to a PNG.
# Only the app's own pixels are captured, never other windows.
# Usage: scripts/screenshot.sh <out.png> <seconds> [app args...]
set -euo pipefail
out=$1; wait_s=$2; shift 2
cd "$(dirname "$0")/.."
ppm=$(mktemp --suffix=.ppm)
bin="$PWD/target/release/traffic-sim"
"$bin" "$@" --screenshot "$ppm" --after "$wait_s" >/dev/null 2>&1 &
for _ in $(seq 1 $((wait_s * 2 + 60))); do
  [ -s "$ppm" ] && break; sleep 0.5
done
sleep 0.5
[ -s "$ppm" ] || { echo "no screenshot produced"; pkill -f "$ppm" || true; exit 1; }
magick "$ppm" "$out" && rm -f "$ppm"
echo "saved $out"
