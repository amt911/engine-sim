#!/usr/bin/env bash
# scripts/yield-cpu.sh SCOPE — while another session holds the timing lock
# (/tmp/claude-$UID/cpu-heavy.owner names a session other than engine-sim), cap the given
# systemd --user scope (e.g. the one scripts/heavy.sh started for a mutation run) to
# CPUQuota=200% (2 of the 24 cores, on top of its nice 19); lift the cap when the lock is free.
# Agreed with gran-turismo-decomp (2026-10-22): their gates use ~18 cores, 2 more at nice 19
# leave headroom. Freezing instead turned every mutant into a timeout and stalled the gate
# for hours under back-to-back holds. Exits when the scope is gone.
set -u
scope=$1
owner=${CPU_LOCK_OWNER:-/tmp/claude-$(id -u)/cpu-heavy.owner}
quota=${YIELD_CPU_QUOTA:-200%}
capped=0
cap() { systemctl --user set-property --runtime "$scope" "CPUQuota=$1"; }
# Started alongside the job, so the scope may not exist yet: give it a minute to appear.
for _ in $(seq 60); do systemctl --user is-active --quiet "$scope" && break; sleep 1; done
while systemctl --user is-active --quiet "$scope"; do
  read -r hpid who _ < "$owner" 2>/dev/null || hpid=""
  if [ -n "$hpid" ] && [ "${who:-}" != engine-sim ] && kill -0 "$hpid" 2>/dev/null; then
    [ $capped = 1 ] || { cap "$quota" && capped=1 && echo "$(date +%T) capped to $quota ($who holds the lock)"; }
  elif [ $capped = 1 ]; then
    cap "" && capped=0 && echo "$(date +%T) uncapped"
  fi
  sleep 5
done
