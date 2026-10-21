#!/usr/bin/env bash
# Run a heavy command under the machine's memory ceiling (see AGENTS.md, "Heavy jobs").
set -euo pipefail
exec systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- "$@"
