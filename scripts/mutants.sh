#!/usr/bin/env bash
# Mutation gate over engine-core. Extra args go to cargo-mutants. PR gate: diff-scoped,
#   scripts/mutants.sh --in-diff <(git diff main...HEAD -- crates/engine-core)
# (a full run takes hours: every mutant recompiles at opt-level 2; run it occasionally). Score = caught / (caught + missed); timeouts and
# unviable mutants are excluded from both. Fails under FLOOR (a ratchet: raise it, never lower).
set -uo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
# Each parallel mutant builds its own copy: a shared CARGO_TARGET_DIR makes the jobs overwrite
# each other's binaries and the verdicts meaningless.
unset CARGO_TARGET_DIR
FLOOR=90   # 2026-10-21: measured 92.4% (640/693)
# cargo-mutants copies the tree into $TMPDIR; /tmp is tmpfs here (RAM): 5 GB once. Use disk.
export TMPDIR="$PWD/target/mutants-tmp"
mkdir -p "$TMPDIR"
OUT=$(mktemp -d)
CARGO_BUILD_JOBS=4 scripts/heavy.sh cargo mutants -p engine-core -j 3 --timeout 90 -o "$OUT" "$@" | tail -1
caught=$(wc -l < "$OUT/mutants.out/caught.txt")
missed=$(wc -l < "$OUT/mutants.out/missed.txt")
total=$((caught + missed))
[ "$total" -gt 0 ] || { echo "FAIL: no mutants tested"; exit 1; }
score=$((100 * caught / total))
echo "mutation score: $score% ($caught/$total), floor $FLOOR%"
cp "$OUT/mutants.out/missed.txt" target/mutants-missed.txt 2>/dev/null || true
rm -rf "$OUT"
[ "$score" -ge "$FLOOR" ] || { echo "FAIL: below floor"; exit 1; }
