#!/usr/bin/env bash
# scripts/profile.sh [PRESET] [SAMPLES] — sampling profile of a headless render, without perf or
# valgrind (perf record hangs on this machine; see docs/FINDINGS.md). Builds a release binary with
# line tables, preloads scripts/profile/ptracer.c so eu-stack may attach under Yama
# ptrace_scope = 1, and counts the innermost source line of the project's code in each sample.
# Timing-dependent: run it under scripts/cpu-lock.sh.
set -euo pipefail
preset=${1:-inline4}; samples=${2:-300}
root=$(cd "$(dirname "$0")/.." && pwd)
out=${CARGO_TARGET_DIR:-$root/target}/profile
mkdir -p "$out"
gcc -shared -fPIC -O2 -o "$out/libptracer.so" "$root/scripts/profile/ptracer.c"
CARGO_PROFILE_RELEASE_DEBUG=line-tables-only CARGO_TARGET_DIR="$out/target" \
  cargo build -p engine-app --release -j 8 >&2
bin="$out/target/release/engine-sim"
LD_PRELOAD="$out/libptracer.so" "$bin" --preset "$preset" --headless --seconds 600 \
  --wav "$out/profile.wav" >/dev/null 2>&1 &
pid=$!
trap 'kill $pid 2>/dev/null || true' EXIT
sleep 2
for _ in $(seq 1 "$samples"); do
  eu-stack -s -p "$pid" 2>/dev/null | grep -m1 -E '^ +crates/' || true
  sleep 0.03
done | grep -o -E '[a-z_0-9]+\.rs:[0-9]+' | sort | uniq -c | sort -rn | head -30
