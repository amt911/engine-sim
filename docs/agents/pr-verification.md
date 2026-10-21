# Agentic PR verification (MANDATORY on every PR)

> Part of `AGENTS.md`, kept in its own file. Its rules bind all the same. Edit the rule here.

## Agentic PR verification (MANDATORY on every PR)

**Every PR is verified by running the real app, and the verdict is posted as a PR comment**
(`gh pr comment`) before it is merged. Its job is to catch what diffs and unit tests miss: a
window that doesn't open, a slider that doesn't change the engine, silence, NaN, a preset that no
longer loads.

The pass, for this desktop app:

1. **Gates** — `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`,
   `cargo test --workspace` (inside the cgroup).
2. **Drive the real binary** — `scripts/smoke.sh`: release build, headless render of every preset to
   WAV (non-silent, finite, RMS in range, dominant firing frequency where the preset's RPM says),
   then launch the GUI for a few seconds and check it stays alive.
3. **Look at it** — take a screenshot of the running window (or a kittest snapshot) for any PR
   that touches the UI, and look at it. Attach what was seen to the verdict.
4. **Read the structure** — name what the diff does to the
   [Design principles](design-principles.md#design-principles--solid-applied-with-judgement) and to the
   real-time audio rules (anything new in the audio callback is a finding).
5. **Verdict** — `gh pr comment <n> --body-file <verdict.md>`: what was run, outputs, what was seen,
   findings. `scripts/verify-pr.sh <n>` runs steps 1–2 and posts the skeleton.

The pass never relaxes a red gate: a red gate blocks the merge (see **Git & GitHub** in
`AGENTS.md`).
