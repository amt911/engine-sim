#!/usr/bin/env bash
# Mutation gate over engine-core. Extra args go to cargo-mutants. PR gate: diff-scoped,
#   scripts/mutants.sh --in-diff <(git diff main...HEAD -- crates/engine-core)
# (a full run recompiles every mutant at opt-level 2: run it occasionally). Score = caught / (caught + missed); timeouts and
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
# One cargo-mutants pass into $1 with timeout $2, in a named scope that scripts/yield-cpu.sh caps
# to 2 cores while another session holds the timing lock (their real-time gates break under our
# full load, even at nice 19); the cap lifts when the lock frees. Each mutant's nextest runs 2
# threads: 3 mutants × 2 threads fill the 6 cores heavy.sh gives (6 threads each oversubscribed
# them 3× and turned clean runs into timeouts, 2026-11-03).
pass() {
  local out=$1 timeout=$2; shift 2
  export HEAVY_UNIT="engine-sim-mutants-$$-$(basename "$out")"
  scripts/yield-cpu.sh "$HEAVY_UNIT.scope" >"$TMPDIR/yield-$$.log" 2>&1 &
  NEXTEST_TEST_THREADS=2 CARGO_BUILD_JOBS=4 scripts/heavy.sh \
    cargo mutants -p engine-core -j 3 --timeout "$timeout" -o "$out" "$@" | tail -1
}
# Exact-name filters (-F) for the mutants listed in file $1.
filters_for() {
  while IFS= read -r name; do
    printf '%s\0' -F "^$(printf '%s' "$name" | sed 's/[][\\.*+?(){}|^$]/\\&/g')"'$'
  done < "$1"
}
# Pass 1, fast: every mutant against the tests under 2 s (.config/mutants-slow-tests.txt lists
# the rest: the engine journeys, ~1900 of the suite's ~2000 test-seconds, 2026-11-03). A caught
# mutant is caught. Pass 2, whole suite: only what pass 1 left missed or timed out, with a long
# timeout. So a mutant counts as missed only when the whole suite misses it, as before.
slow=$(grep -v '^#' .config/mutants-slow-tests.txt 2>/dev/null | sed 's/.*/test(=&)/' | paste -sd'|')
OUT=$(mktemp -d)
if [ -n "$slow" ]; then
  pass "$OUT" 300 "$@" -- -E "not ($slow)"
else
  pass "$OUT" 300 "$@"
fi
caught=$(wc -l < "$OUT/mutants.out/caught.txt")
cat "$OUT/mutants.out/missed.txt" "$OUT/mutants.out/timeout.txt" > "$OUT/again.txt"
missed=0
timeouts="$OUT/mutants.out/timeout.txt"
if [ -s "$OUT/again.txt" ]; then
  echo "pass 2: $(wc -l < "$OUT/again.txt") mutants against the whole suite"
  RETRY=$(mktemp -d)
  mapfile -d '' filters < <(filters_for "$OUT/again.txt")
  # by name only: the first pass's arguments (an --in-diff from a pipe) cannot be read twice
  pass "$RETRY" 1800 "${filters[@]}"
  # cargo-mutants 27 adds its field-deletion mutants whatever -F says: count only the retried
  only() { grep -Fxf "$OUT/again.txt" "$RETRY/mutants.out/$1.txt" || true; }
  caught=$((caught + $(only caught | grep -c .)))
  missed=$(only missed | grep -c .)
  only missed > "$OUT/mutants.out/missed.txt"
  only timeout > "$timeouts"
  rm -rf "$RETRY"
else
  : > "$timeouts"
fi
nto=$(wc -l < "$timeouts")
echo "timeouts after the whole-suite pass: $nto"
total=$((caught + missed))
# A timeout is out of the score, so a run starved of CPU could pass on what it never judged:
# more than 5 % timeouts and the score means nothing (2026-11-03: 130 of 280 on a loaded box).
if [ $((20 * nto)) -gt $((total + nto)) ]; then
  echo "FAIL: $nto timeouts of $((total + nto)) mutants: too many for a verdict"
  cp "$timeouts" target/mutants-timeouts.txt 2>/dev/null || true
  exit 1
fi
[ "$total" -gt 0 ] || { echo "FAIL: no mutants tested"; exit 1; }
score=$((100 * caught / total))
echo "mutation score: $score% ($caught/$total), floor $FLOOR%"
cp "$OUT/mutants.out/missed.txt" target/mutants-missed.txt 2>/dev/null || true
rm -rf "$OUT"
[ "$score" -ge "$FLOOR" ] || { echo "FAIL: below floor"; exit 1; }
