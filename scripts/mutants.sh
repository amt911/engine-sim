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
# One cargo-mutants pass into $1, in a named scope that scripts/yield-cpu.sh freezes while
# another session holds the timing lock (their real-time gates break under our full load, even at
# nice 19); the cap lifts when the lock frees.
pass() {
  local out=$1; shift
  export HEAVY_UNIT="engine-sim-mutants-$$-$(basename "$out")"
  scripts/yield-cpu.sh "$HEAVY_UNIT.scope" >"$TMPDIR/yield-$$.log" 2>&1 &
  CARGO_BUILD_JOBS=4 scripts/heavy.sh cargo mutants -p engine-core -j 3 --timeout 90 -o "$out" "$@" | tail -1
}
OUT=$(mktemp -d)
pass "$OUT" "$@"
caught=$(wc -l < "$OUT/mutants.out/caught.txt")
missed=$(wc -l < "$OUT/mutants.out/missed.txt")
# Frozen time counts against cargo-mutants' wall-clock timeout, so a run frozen for a long lock
# hold scores its mutants as timeouts. Retry them (exact names) twice: a mutant that really
# hangs times out again in 90 s; one that was only frozen gets its verdict.
timeouts="$OUT/mutants.out/timeout.txt"
for _ in 1 2; do
  [ -s "$timeouts" ] || break
  filters=()
  while IFS= read -r name; do
    filters+=(-F "^$(printf '%s' "$name" | sed 's/[][\\.*+?(){}|^$]/\\&/g')"'$')
  done < "$timeouts"
  echo "retrying $(wc -l < "$timeouts") timed-out mutants"
  RETRY=$(mktemp -d)
  pass "$RETRY" "$@" "${filters[@]}"
  # cargo-mutants 27 adds its field-deletion mutants whatever -F says: count only the retried
  only() { grep -Fxf "$timeouts" "$RETRY/mutants.out/$1.txt" || true; }
  caught=$((caught + $(only caught | grep -c .)))
  missed=$((missed + $(only missed | grep -c .)))
  only missed >> "$OUT/mutants.out/missed.txt"
  only timeout > "$RETRY/still.txt"
  cp "$RETRY/still.txt" "$timeouts"
  rm -rf "$RETRY"
done
echo "timeouts after retries: $(wc -l < "$timeouts")"
total=$((caught + missed))
[ "$total" -gt 0 ] || { echo "FAIL: no mutants tested"; exit 1; }
score=$((100 * caught / total))
echo "mutation score: $score% ($caught/$total), floor $FLOOR%"
cp "$OUT/mutants.out/missed.txt" target/mutants-missed.txt 2>/dev/null || true
rm -rf "$OUT"
[ "$score" -ge "$FLOOR" ] || { echo "FAIL: below floor"; exit 1; }
