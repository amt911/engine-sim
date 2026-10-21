#!/usr/bin/env bash
# Run a TIMING-DEPENDENT job (benchmarks, real-time factor / latency measurements) under the CPU
# lock shared with other agent sessions on this machine (~/.claude/CLAUDE.md, "Sharing the CPU"):
# flock /tmp/claude-<uid>/cpu-heavy.lock, owner "<holder pid> engine-sim <claude pid> <start> <what>",
# hold under timeout 1800. A waiter polls every 30 s, goes ahead at once if the holder or its session
# is gone (orphan), and after 30 min in any case (we go ahead niced; we never kill other jobs).
set -euo pipefail
DIR=/tmp/claude-$(id -u)
LOCK=$DIR/cpu-heavy.lock
OWNER=$DIR/cpu-heavy.owner
mkdir -p "$DIR"
alive() { [[ ! "$1" =~ ^[0-9]+$ ]] || kill -0 "$1" 2>/dev/null; }
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
  read -r holder _ session _ < "$OWNER" 2>/dev/null || true
  if ! alive "${holder:-x}" || ! alive "${session:-x}" || [ "$waited" -ge 1800 ]; then
    echo "cpu-lock: holder gone or 30 min waited; going ahead niced without the lock" >&2
    exec nice -n 19 timeout 1800 "$@"
  fi
done
echo "$$ engine-sim $(claude_pid) $(date -Is) $*" > "$OWNER"
exec timeout 1800 "$@"
