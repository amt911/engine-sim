#!/usr/bin/env bash
# scripts/sabotage.sh FILE LINE OLD NEW [TEST_FILTER] — hand-apply one mutation (first OLD → NEW on
# LINE), run engine-core's tests, report caught/SURVIVED, restore the file. Proves a new test
# catches the mutant it was written for (docs/agents/debugging.md: every assertion seen failing).
set -u
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
if ! cargo build -q -p engine-core --lib --tests >/dev/null 2>&1; then
  echo "UNBUILDABLE $f:$l $old -> $new"
elif cargo test -q -p engine-core --lib -- $filt >/dev/null 2>&1; then
  echo "SURVIVED $f:$l $old -> $new"
else
  echo "caught $f:$l $old -> $new"
fi
# the backup is older than the mutant build: touch it or cargo keeps testing the mutant
mv "$f.bak" "$f"; touch "$f"
