#!/usr/bin/env bash
# Run a heavy command (see AGENTS.md, "Heavy jobs"):
#  - under the machine's memory ceiling (cgroup scope, 6 GB);
#  - holding the shared CPU lock /tmp/claude-1000/cpu-heavy.lock (other sessions on this machine run
#    real-time gates that break under load), max 30 min per hold (timeout 1800);
#  - owner line v2 in cpu-heavy.owner: "<holder pid> engine-sim <claude pid> <start> <what>";
#  - at nice 19 with cargo capped to 8 jobs.
# Waiting more than 30 min: run anyway at nice 19 without the lock (we never kill other jobs).
set -euo pipefail
LOCK=/tmp/claude-1000/cpu-heavy.lock
OWNER=/tmp/claude-1000/cpu-heavy.owner
mkdir -p /tmp/claude-1000
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-8}" RAYON_NUM_THREADS="${RAYON_NUM_THREADS:-8}"
run() { exec nice -n 19 timeout 1800 systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- "$@"; }
# Re-entrant: a heavy job started from inside another (verify-pr.sh → heavy.sh) already holds it.
if [ "${ENGINE_SIM_HEAVY:-}" = 1 ]; then run "$@"; fi
export ENGINE_SIM_HEAVY=1
claude_pid() {
  local p=$$
  while [ "$p" -gt 1 ]; do
    [ "$(cat /proc/$p/comm 2>/dev/null)" = claude ] && { echo "$p"; return; }
    p=$(awk '{print $4}' /proc/$p/stat 2>/dev/null || echo 1)
  done
  echo 0
}
exec 9>"$LOCK"
waited=0
until flock -w 30 9; do
  waited=$((waited + 30))
  if [ "$waited" -ge 1800 ]; then
    echo "heavy.sh: waited 30 min for the CPU lock; running niced without it" >&2
    run "$@"
  fi
done
echo "$$ engine-sim $(claude_pid) $(date -Is) $*" > "$OWNER"
run "$@"
