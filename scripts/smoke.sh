#!/usr/bin/env bash
# Real-environment smoke: drives the release binary. PASS/FAIL per phase, exits on first failure.
set -uo pipefail
cd "$(dirname "$0")/.."
BIN=target/release/engine-sim
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
phase() { echo "== $1"; }
fail() { echo "FAIL: $1"; exit 1; }

phase "build"
cargo build --release -p engine-app -q || fail "release build"
echo "PASS build"

phase "version"
"$BIN" --version >/dev/null 2>&1 || "$BIN" >/dev/null 2>&1 || fail "binary does not run"
echo "PASS version"

if "$BIN" --help 2>/dev/null | grep -q -- --headless; then
  phase "headless render"
  for preset in presets/*.yaml; do
    [ -e "$preset" ] || continue
    wav="$OUT/$(basename "$preset" .yaml).wav"
    "$BIN" --engine "$preset" --headless --seconds 3 --wav "$wav" --check || fail "headless $preset"
    echo "PASS headless $preset"
  done
fi

if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && "$BIN" --help 2>/dev/null | grep -q -- --quit-after; then
  phase "gui launch"
  "$BIN" --quit-after 4 >"$OUT/gui.log" 2>&1 || { cat "$OUT/gui.log"; fail "gui exited with error"; }
  echo "PASS gui launch"
fi
echo "ALL PASS"
