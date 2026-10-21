#!/usr/bin/env bash
# Run a heavy command (see AGENTS.md, "Heavy jobs"):
#  - under the machine's memory ceiling (cgroup scope, 6 GB);
#  - holding the shared CPU lock /tmp/claude-1000/cpu-heavy.lock (other sessions on this machine
#    run real-time gates that break under load); owner recorded in cpu-heavy.owner;
#  - at nice 19 with cargo capped to 8 jobs.
set -euo pipefail
LOCK=/tmp/claude-1000/cpu-heavy.lock
OWNER=/tmp/claude-1000/cpu-heavy.owner
mkdir -p /tmp/claude-1000
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-8}" RAYON_NUM_THREADS="${RAYON_NUM_THREADS:-8}"
# Re-entrant: a heavy job started from inside another (e.g. verify-pr.sh → heavy.sh) already holds it.
if [ "${ENGINE_SIM_HEAVY:-}" = 1 ]; then
  exec systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- "$@"
fi
export ENGINE_SIM_HEAVY=1
exec flock "$LOCK" bash -c '
  echo "$$ engine-sim $(date -Is) $*" > "'"$OWNER"'"
  exec nice -n 19 systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- "$@"
' heavy "$@"
