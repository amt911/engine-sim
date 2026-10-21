#!/usr/bin/env bash
# Run a heavy command (builds, test suites, mutation runs — see AGENTS.md, "Heavy jobs"):
# inside the machine's 6 GB memory cgroup, at nice 19, cargo capped to 8 jobs. No CPU lock:
# builds/tests/lint never take it (user rule); timing-dependent jobs use scripts/cpu-lock.sh.
set -euo pipefail
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-8}" RAYON_NUM_THREADS="${RAYON_NUM_THREADS:-8}"
exec nice -n 19 systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- "$@"
