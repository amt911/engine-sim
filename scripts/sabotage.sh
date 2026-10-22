#!/usr/bin/env bash
# scripts/sabotage.sh FILE LINE OLD NEW [TEST_FILTER] — hand-apply one mutation (first OLD → NEW on
# LINE), run engine-core's tests, report caught/SURVIVED, restore the file. Proves a new test
# catches the mutant it was written for (docs/agents/debugging.md: every assertion seen failing).
set -u
# Shares the machine: lowest priority, 8 build jobs, and a mutant that hangs a test is killed
# after 120 s (scored caught, as cargo-mutants scores timeouts — but say so).
export CARGO_BUILD_JOBS=8
f=$1; l=$2; old=$3; new=$4; filt=${5:-}
cp "$f" "$f.bak"
python3 - "$f" "$l" "$old" "$new" <<'PY' || { mv "$f.bak" "$f"; touch "$f"; exit 2; }
import sys
f, l, o, n = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
lines = open(f).read().split('\n')
if o not in lines[l - 1]:
    sys.exit(f"'{o}' not on line {l}: {lines[l - 1]}")
lines[l - 1] = lines[l - 1].replace(o, n, 1)
open(f, 'w').write('\n'.join(lines))
PY
if ! nice -n 19 cargo build -q -p engine-core --lib --tests >/dev/null 2>&1; then
  echo "UNBUILDABLE $f:$l $old -> $new"
else
  nice -n 19 timeout -k 5 120 cargo test -q -p engine-core --lib -- $filt >/dev/null 2>&1
  case $? in
    0) echo "SURVIVED $f:$l $old -> $new" ;;
    124|137)
      # timeout kills cargo, not the test binary it spawned
      pkill -9 -f "$PWD/target/debug/deps/engine_core-" 2>/dev/null
      echo "caught (timeout) $f:$l $old -> $new" ;;
    *) echo "caught $f:$l $old -> $new" ;;
  esac
fi
# the backup is older than the mutant build: touch it or cargo keeps testing the mutant
mv "$f.bak" "$f"; touch "$f"
