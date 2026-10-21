#!/usr/bin/env bash
# Coverage gate (80% lines). Excludes code that only a real window/sound card exercises
# (main, view painting, app shell, sim thread, cpal device) — covered by scripts/smoke.sh.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
# Arch's rust package has no rustup llvm-tools; use the system LLVM (same major version).
export LLVM_COV="${LLVM_COV:-$(command -v llvm-cov)}" LLVM_PROFDATA="${LLVM_PROFDATA:-$(command -v llvm-profdata)}"
exec scripts/heavy.sh cargo llvm-cov --workspace -j 8 \
  --ignore-filename-regex '(main|view|app|runner|device)\.rs$' \
  --fail-under-lines 80 --summary-only "$@"
