#!/usr/bin/env bash
# Whether PRs get their mutation run (AGENTS.md, "Mutation gate"). The user's choice, per machine,
# kept outside the repo in $XDG_STATE_HOME/engine-sim/mutants-mode (on by default).
#
#   scripts/mutants-mode.sh on | off   # set it
#   scripts/mutants-mode.sh            # print it
#   scripts/mutants-mode.sh hook       # session start: a line only when it is off
set -euo pipefail
file="${XDG_STATE_HOME:-$HOME/.local/state}/engine-sim/mutants-mode"
mode=$(cat "$file" 2>/dev/null || echo on)
case "${1:-}" in
  on | off)
    mkdir -p "$(dirname "$file")"
    echo "$1" > "$file"
    echo "mutation runs: $1"
    ;;
  hook) [ "$mode" = off ] && echo "Mutation runs are OFF (user's choice): do not start them for PRs. 'scripts/mutants-mode.sh on' turns them back on." || true ;;
  "") echo "mutation runs: $mode" ;;
  *) echo "usage: $0 [on|off|hook]" >&2; exit 2 ;;
esac
