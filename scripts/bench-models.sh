#!/usr/bin/env bash
# scripts/bench-models.sh BINARY OUT.txt [SECONDS] — real-time cost of each preset's exhaust model
# at Ultra: the preset as shipped (its own exhaust model) and with the finite-volume exhaust,
# wall seconds per second of audio, one core, full throttle. Timing-dependent: run it through
# scripts/cpu-lock.sh. Below 1.0 keeps up with real time on this machine.
set -euo pipefail
BIN=$1; OUT=$2; SECS=${3:-4}
T=$(mktemp -d)
trap 'rm -r -- "$T"' EXIT
PRESETS="single inline4 honda-k20a honda-k20a-tuned honda-k20c1 subaru-ej25 vw-sdi vw-tdi mazda-renesis mazda-13b-bridgeport lfa-v10 v8-flatplane v8-crossplane"
echo "preset shipped_s_per_s fv_s_per_s" > "$OUT"
for p in $PRESETS; do
  "$BIN" --preset "$p" --save "$T/$p.yaml"
  # the exhaust's model line (4-space indent under `exhaust:`) to fv
  sed 's/^    model: waveguide$/    model: fv/' "$T/$p.yaml" > "$T/$p-fv.yaml"
  row="$p"
  for f in "$T/$p.yaml" "$T/$p-fv.yaml"; do
    s=$(date +%s.%N)
    taskset -c 5 "$BIN" --engine "$f" --headless --quality ultra --throttle 1 \
      --seconds "$SECS" --wav "$T/out.wav" >/dev/null 2>&1 || { row="$row fail"; continue; }
    e=$(date +%s.%N)
    row="$row $(python3 -c "print(round(($e - $s) / $SECS, 3))")"
  done
  echo "$row" >> "$OUT"
done
