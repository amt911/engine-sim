#!/usr/bin/env bash
# Build one part (or all): scripts/run.sh piston [--render]   |   scripts/run.sh all --render
# Blender runs headless inside a 6 GB memory cgroup at the lowest CPU priority.
set -euo pipefail
cd "$(dirname "$0")"
parts=("$1"); [[ "$1" == all ]] && parts=(piston conrod crankshaft valvetrain wankel turbo)
extra=(); [[ "${2:-}" == --render ]] && extra=(-- --render 1)
for p in "${parts[@]}"; do
  nice -n 19 systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- \
    blender -b --factory-startup -P "$p.py" "${extra[@]}" 2>&1 | grep -E "^\[|Error|Traceback|File \"" || true
done
