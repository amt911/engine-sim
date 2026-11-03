#!/usr/bin/env bash
# Times every engine-core test once and writes the ones slower than $1 seconds (default 2) to
# .config/mutants-slow-tests.txt, which scripts/mutants.sh leaves out of its fast first pass.
# Rerun when the suite changes a lot; a stale list only costs speed, never a verdict (a mutant
# the fast pass misses is always retried with the whole suite).
set -euo pipefail
cd "$(dirname "$0")/.."
limit=${1:-2}
log=$(mktemp)
scripts/heavy.sh cargo nextest run -p engine-core --status-level pass --final-status-level none \
  --no-fail-fast >"$log" 2>&1 || { tail -20 "$log"; exit 1; }
{
  echo "# engine-core tests over ${limit} s (scripts/mutants-slow-tests.sh, $(date +%F)): left out of"
  echo "# the mutation gate's fast pass; the survivors of that pass run the whole suite."
  sed -nE 's/^ *PASS \[ *([0-9.]+)s\] +\([ 0-9]+\/[0-9]+\) engine-core (.*)$/\1 \2/p' "$log" |
    awk -v t="$limit" '$1 > t { print $2 }' | sort
} >.config/mutants-slow-tests.txt
rm -f "$log"
echo "$(grep -vc '^#' .config/mutants-slow-tests.txt) slow tests listed"
