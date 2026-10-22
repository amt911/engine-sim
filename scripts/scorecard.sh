#!/usr/bin/env bash
# scripts/scorecard.sh [--save LABEL] — measure the sound scorecard (crates/engine-app sim.rs
# `scorecard`) and print it next to the last committed one (docs/sound-scorecard.csv), so a
# change to the sound shows its effect in numbers. --save appends this run under LABEL.
set -euo pipefail
cd "$(dirname "$0")/.."
CARGO_BUILD_JOBS=${CARGO_BUILD_JOBS:-8} nice -n 19 cargo test -q -p engine-app --release scorecard \
  -- --ignored --nocapture > /dev/null
new=target/sound/scorecard.csv
base=docs/sound-scorecard.csv
if [ -f "$base" ]; then
  last=$(grep -E '^# ' "$base" | tail -1)
  echo "baseline: ${last#\# }"
  python3 - "$base" "$new" <<'PY'
import sys, csv
blocks, cur = [], []
for line in open(sys.argv[1]):
    if line.startswith('# '):
        cur = []; blocks.append(cur)
    elif line.strip() and not line.startswith('preset'):
        cur.append(line.strip().split(','))
base = {r[0]: r for r in blocks[-1]} if blocks else {}
rows = list(csv.reader(open(sys.argv[2])))
head = rows[0]
for r in rows[1:]:
    b = base.get(r[0])
    cells = []
    for i, h in enumerate(head[1:], 1):
        cells.append(f"{h}={r[i]}" + (f" ({float(r[i]) - float(b[i]):+.1f})" if b else ""))
    print(r[0].ljust(14), "  ".join(cells))
PY
else
  cat "$new"
fi
if [ "${1:-}" = --save ]; then
  { echo "# ${2:?label}"; cat "$new"; } >> "$base"
  echo "saved under '$2' in $base"
fi
