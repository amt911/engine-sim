#!/usr/bin/env bash
# attribute.sh LABEL [SOURCE ...] — which simulator source makes a gate feature what it is.
#
# For a build the gates already rendered (renders/LABEL/engine.yaml + its held full-load renders), render
# the same held speeds again with one sound source switched off at a time (sound.SOURCE: 0), then print
# `spectral_gate.py physics`: every feature at base and its change per source. A failing feature that
# moves toward the real range when one source is off names that source; one that no source moves is the
# gas dynamics (the exhaust and intake waves themselves). No guessing, no clips (debugging.md).
#
# Default sources: intake block mechanical flow_noise valve_jet duct_flow combustion_jitter.
# Labels: LABEL-off-SOURCE. Binary: $ENGINE_SIM_BIN, else the one gates.sh recorded in LABEL.bin.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
lab=$1; shift
srcs=("$@"); [ $# -eq 0 ] && srcs=(intake block mechanical flow_noise valve_jet duct_flow combustion_jitter)
data=$(cd "$here" && python3 -c 'import rdata; print(rdata.DATA)')
base=$data/renders/$lab
bin=${ENGINE_SIM_BIN:-$(head -1 "$base.bin" 2>/dev/null || true)}
[ -x "$bin" ] || { echo "no binary: set ENGINE_SIM_BIN" >&2; exit 2; }
py() { uv run -q --with numpy --with scipy --with matplotlib python -I "$@"; }

if [ -z "${ATTR_IN_SCOPE:-}" ]; then   # inside the memory ceiling (AGENTS.md: heavy jobs)
  ATTR_IN_SCOPE=1 ENGINE_SIM_BIN=$bin exec systemd-run --user --scope --quiet -p MemoryHigh=5G \
    -p MemoryMax=6G -p MemorySwapMax=0 nice -n19 "$0" "$lab" "${srcs[@]}"
fi

held=$(cd "$base" && ls held_*_1.0_*.wav)
work=()
for s in "${srcs[@]}"; do
  grep -qE "^    $s: " "$base/engine.yaml" || { echo "no sound.$s in $base/engine.yaml" >&2; exit 2; }
  d=$data/renders/$lab-off-$s; mkdir -p "$d"
  sed -E "s/^(    $s): .*/\1: 0.0/" "$base/engine.yaml" > "$d/engine.yaml"
  for h in $held; do
    [ -s "$d/$h" ] && continue
    IFS=_ read -r _ rpm load sec <<< "${h%.wav}"
    work+=("$bin --engine $d/engine.yaml --render held:$rpm:$load:$sec --out $d/$h")
  done
done
printf '%s\n' "${work[@]}" | xargs -r -P "${ATTR_JOBS:-5}" -I{} sh -c '{} > /dev/null 2>&1'
labs=(); for s in "${srcs[@]}"; do labs+=("$lab-off-$s"); done
py "$here/spectral/sp_sim.py" "$lab" "${labs[@]}" > /dev/null 2>&1
py "$here/spectral_gate.py" physics "$lab" "${labs[@]}" ${ATTR_COLS:+--cols=$ATTR_COLS}
