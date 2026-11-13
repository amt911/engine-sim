#!/usr/bin/env bash
# gates.sh BINARY SUFFIX [PRESET ...] — the four realism gates on every preset (or the ones named),
# renders and analysis in parallel, the whole run inside one 6 GB memory cgroup.
#
#   1 physgate.py      physics tier (crescendo, load span, lift-off, comb)      renders the scenes
#   2 spectral_gate.py order structure, texture and wind against real recordings
#   3 expect.py        what the engine's kinematics must and must not make (K1-K3)
#   4 blind2.py        cycle-to-cycle periodicity and discrete echo, ratchet on ear-approved builds
#   5 ear_gate.py      what the ear named: bank orders, wind, lift-off hiss, clip peak (same-family recordings)
#
# Labels are PRESET-SUFFIX (lfa-v10 -> lfa-SUFFIX). Renders and caches live in $REALISM_DATA (see
# rdata.py; never in the repository). Summary: $REALISM_DATA/renders/gates-SUFFIX.txt.
# Physics changes are judged on every preset, never one (AGENTS.md: realism renders).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
bin=$(realpath "$1"); sfx=$2; shift 2
all=(inline4 lfa-v10 honda-k20a honda-k20a-tuned honda-k20c1 v8-flatplane v8-crossplane subaru-ej25
     vw-sdi vw-tdi mazda-renesis mazda-13b-bridgeport single)
presets=("$@"); [ $# -eq 0 ] && presets=("${all[@]}")
data=$(cd "$here" && python3 -c 'import rdata; print(rdata.DATA)')
export REALISM_DATA=$data ENGINE_SIM_BIN=$bin
jobs=${PHYSGATE_JOBS:-2}                # renders at once per preset; 13 presets x 2 on 24 cores
py() { uv run -q --with numpy --with scipy --with matplotlib python -I "$@"; }
label() { local p=$1; [ "$p" = lfa-v10 ] && p=lfa; echo "$p-$sfx"; }

# A label's renders belong to one binary: physgate skips a WAV that exists, so a suffix reused
# with another build would judge that build's old renders as this one's (2026-11-13: six presets
# of "dir" were renders of 10-11 Nov). LABEL.bin holds the binary's path and sha256.
sum=$(sha256sum "$bin" | cut -d' ' -f1)
for p in "${presets[@]}"; do
  l=$(label "$p"); d=$data/renders/$l
  if compgen -G "$d/*.wav" >/dev/null && [ "$(sed -n 2p "$d.bin" 2>/dev/null)" != "$sum" ]; then
    echo "gates.sh: $d holds renders of another binary (or of an unknown one); pick a new" \
      "suffix or move that folder away" >&2
    exit 2
  fi
done

if [ -z "${GATES_IN_SCOPE:-}" ]; then   # re-enter inside the memory ceiling (AGENTS.md: heavy jobs)
  GATES_IN_SCOPE=1 exec systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G \
    -p MemorySwapMax=0 nice -n19 "$0" "$bin" "$sfx" "${presets[@]}"
fi

out=$data/renders/gates-$sfx.txt
labels=()
for p in "${presets[@]}"; do
  l=$(label "$p"); labels+=("$l"); printf '%s\n%s\n' "$bin" "$sum" > "$data/renders/$l.bin"
  PHYSGATE_JOBS=$jobs py "$here/physgate.py" run "$bin" "$p" --out "$data/renders/$l" \
    > "$data/renders/$l.physgate.txt" 2>&1 &
done
wait || true

py "$here/spectral/sp_sim.py" "${labels[@]}" > "$data/renders/sp_sim-$sfx.log" 2>&1 || true
{
  for l in "${labels[@]}"; do
    echo "$l physgate: $(grep -E '^(PASS|FAIL|NO DATA)' "$data/renders/$l.physgate.txt" | tail -1)"
  done
  py "$here/spectral_gate.py" gate -q "${labels[@]}" 2>&1 | grep -E '^[a-z0-9.-]+ \(|low-mid|  FAIL' || true
} > "$out.12"
for l in "${labels[@]}"; do
  ( yaml=$data/renders/$l/engine.yaml
    e=$(py "$here/expect.py" judge "$yaml" "$l" 2>&1 | grep '=>' || echo "=> no result")
    b=$(py "$here/blind2.py" gate "$l" 2>&1 | grep -v WARN || true)
    printf '%s expect: %s\n%s\n' "$l" "$e" "$b" > "$data/renders/$l.gates34.txt" ) &
done
wait
uv run -q --no-project --with numpy,scipy,soundfile python -I "$here/ear_gate.py" gate "${labels[@]}" \
  > "$out.5" 2>&1 || true
{ cat "$out.12"; for l in "${labels[@]}"; do cat "$data/renders/$l.gates34.txt"; done; cat "$out.5"; } > "$out"
rm -f "$out.5"
rm -f "$out.12"
cat "$out"
