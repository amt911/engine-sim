#!/usr/bin/env bash
# scripts/yield-cpu.sh SCOPE — while another session holds the timing lock
# (/tmp/claude-$UID/cpu-heavy.owner names a session other than engine-sim), freeze the given
# systemd --user scope (e.g. the one scripts/heavy.sh started for a mutation run); thaw it when
# the lock is free. Exits when the scope is gone. Frozen mutants may score as timeouts (excluded
# from the gate score), never as caught or missed.
set -u
scope=$1
owner=/tmp/claude-$(id -u)/cpu-heavy.owner
frozen=0
while systemctl --user is-active --quiet "$scope"; do
  read -r hpid who _ < "$owner" 2>/dev/null || hpid=""
  if [ -n "$hpid" ] && [ "${who:-}" != engine-sim ] && kill -0 "$hpid" 2>/dev/null; then
    [ $frozen = 1 ] || { systemctl --user freeze "$scope" && frozen=1 && echo "$(date +%T) frozen ($who holds the lock)"; }
  elif [ $frozen = 1 ]; then
    systemctl --user thaw "$scope" && frozen=0 && echo "$(date +%T) thawed"
  fi
  sleep 5
done
