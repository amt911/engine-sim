#!/usr/bin/env bash
# The periodic mutation run (AGENTS.md: it does not block PRs). The last run is recorded outside
# the repo, per machine, so a computer that was off or a project left alone for months still
# knows how far behind it is: $XDG_STATE_HOME/engine-sim/mutants-last-run holds
# "<date> <main commit> <score or FAIL>".
#
#   scripts/mutants-periodic.sh due   # prints a line when a run is due (session-start hook), else nothing
#   scripts/mutants-periodic.sh run   # mutates engine-core's diff since the recorded commit, then records main
#
# Due: 7 days or more since the last run with engine-core changed since, or 30 engine-core
# commits whatever the date.
set -uo pipefail
cd "$(dirname "$0")/.."
state="${XDG_STATE_HOME:-$HOME/.local/state}/engine-sim"
record="$state/mutants-last-run"
# no record yet: the last full run measured on main (2026-10-21, 92.4 %)
{ read -r last_date last_sha _ < "$record"; } 2>/dev/null || { last_date=2026-10-21; last_sha=8871f9b68f41edb82cbf05dab4eda13efb327403; }
main=$(git rev-parse --verify -q origin/main || git rev-parse --verify -q main) || exit 0
git cat-file -e "$last_sha^{commit}" 2>/dev/null || last_sha=$(git rev-list --max-parents=0 "$main" | tail -1)
changed=$(git rev-list --count "$last_sha..$main" -- crates/engine-core)
days=$(( ($(date +%s) - $(date -d "$last_date" +%s)) / 86400 ))

case "${1:-due}" in
  due)
    if { [ "$days" -ge 7 ] && [ "$changed" -gt 0 ]; } || [ "$changed" -ge 30 ]; then
      echo "MUTATION RUN DUE: $changed engine-core commits and $days days since the last run ($last_date)." \
        "When the CPU is free, run scripts/mutants-periodic.sh run in the background (two passes," \
        "about an hour per 100 mutants on a quiet box) and pin its misses in a follow-up PR."
    fi
    ;;
  run)
    [ "$changed" -gt 0 ] || { echo "nothing changed in engine-core since $last_date"; exit 0; }
    diff=$(mktemp)
    git diff "$last_sha..$main" -- crates/engine-core > "$diff"
    out=$(scripts/mutants.sh --in-diff "$diff" 2>&1 | tee /dev/stderr)
    status=$?
    rm -f "$diff"
    score=$(printf '%s\n' "$out" | sed -n 's/^mutation score: \([0-9]*%\).*/\1/p' | tail -1)
    mkdir -p "$state"
    echo "$(date +%F) $(git rev-parse "$main") ${score:-FAIL}" > "$record"
    echo "recorded: $(cat "$record") (misses: target/mutants-missed.txt)"
    exit $status
    ;;
  *) echo "usage: $0 due|run" >&2; exit 2 ;;
esac
