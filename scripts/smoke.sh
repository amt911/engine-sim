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
  for preset in $("$BIN" --help | sed -n 's/.*possible values: \(.*\)]/\1/p' | head -1 | tr -d ','); do
    "$BIN" --preset "$preset" --headless --seconds 3 --wav "$OUT/$preset.wav" --check || fail "headless $preset"
    echo "PASS headless $preset"
  done
  for preset in presets/*.yaml presets/*.json; do
    [ -e "$preset" ] || continue
    "$BIN" --engine "$preset" --headless --seconds 3 --wav "$OUT/x.wav" --check || fail "headless $preset"
    echo "PASS headless $preset"
  done
fi

phase "dyno sweep"
for preset in single inline4 lfa-v10; do
  "$BIN" --preset "$preset" --dyno >"$OUT/dyno.csv" 2>/dev/null || fail "dyno $preset"
  [ "$(head -1 "$OUT/dyno.csv")" = "rpm,torque_nm,power_hp" ] || fail "dyno $preset header"
  awk -F, 'NR>1 { n++; if ($2+0 <= 0 || $2 != $2+0 || $3+0 <= 0) bad=1 } END { exit (n >= 5 && !bad) ? 0 : 1 }' "$OUT/dyno.csv" \
    || fail "dyno $preset: too few rows or non-positive/NaN torque"
  echo "PASS dyno $preset ($(($(wc -l <"$OUT/dyno.csv") - 1)) points)"
done

phase "file round trip"
"$BIN" --preset lfa-v10 --save "$OUT/rt.yaml" || fail "save yaml"
"$BIN" --engine "$OUT/rt.yaml" --save "$OUT/rt.json" || fail "yaml -> json"
"$BIN" --engine "$OUT/rt.json" --headless --seconds 1 --check || fail "load json + render"
echo '{"version": 99, "engine": {}}' > "$OUT/bad.json"
if "$BIN" --engine "$OUT/bad.json" --headless --seconds 1 2>/dev/null; then fail "future version accepted"; fi
echo "PASS file round trip"

if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && "$BIN" --help 2>/dev/null | grep -q -- --quit-after; then
  phase "gui launch"
  "$BIN" --quit-after 4 --preset single >"$OUT/gui.log" 2>&1 || { cat "$OUT/gui.log"; fail "gui exited with error"; }
  echo "PASS gui launch"
fi
echo "ALL PASS"
