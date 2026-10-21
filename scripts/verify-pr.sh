#!/usr/bin/env bash
# Agentic PR verification, steps 1-2 (see docs/agents/pr-verification.md). Usage: scripts/verify-pr.sh <pr>
set -uo pipefail
cd "$(dirname "$0")/.."
PR=${1:?pr number}
LOG=$(mktemp)
run() { echo "\$ $*" >>"$LOG"; "$@" >>"$LOG" 2>&1; local rc=$?; echo "rc=$rc" >>"$LOG"; return $rc; }
ok=1
run cargo fmt --all --check || ok=0
run scripts/heavy.sh cargo clippy --workspace --all-targets -- -D warnings || ok=0
run scripts/heavy.sh cargo test --workspace || ok=0
run scripts/smoke.sh || ok=0
verdict=$([ $ok = 1 ] && echo "PASS" || echo "FAIL")
{
  echo "## Agentic verification: $verdict"
  echo
  echo '<details><summary>Log (tail)</summary>'
  echo
  echo '```text'
  tail -n 60 "$LOG"
  echo '```'
  echo '</details>'
  echo
  echo "${VERIFY_NOTES:-}"
} >"$LOG.md"
gh pr comment "$PR" --body-file "$LOG.md"
[ $ok = 1 ]
