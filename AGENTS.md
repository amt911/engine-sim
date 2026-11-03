# engine-sim — Agent Guide

A real-time engine **sound simulator** written in Rust: you build an engine visually (cylinders,
bore, stroke, layout, cams, intake and exhaust geometry, rotary engines…), watch it run animated,
hear the sound produced by the physics, and load it with a dyno, gears and a vehicle. Engines are
exported/imported as JSON or YAML. For anyone who likes engines; it must run on modest machines.

- **What to build:** [`docs/specs.md`](docs/specs.md) (the user's high-level wishes) and the design
  spec in [`docs/superpowers/specs/`](docs/superpowers/specs/).
- **How to build it (physics, DSP, crates, papers):**
  [`docs/research/simulador-motor-rust.md`](docs/research/simulador-motor-rust.md). It is the
  technical source of truth: models, numerical schemes and references to implement. Read the section
  for the phase you are working on before writing physics or DSP.

## Start here

- **Run `/graphify` before each session** once `graphify-out/` exists (it is optional until the
  codebase is big enough to benefit). It summarizes architecture without re-reading the repo.
- **Before touching UI:** the user explicitly wants the app **usable early, even if it looks ugly**
  (2026-10-21). Function first; the `impeccable` flow (`$impeccable teach` → `PRODUCT.md` /
  `DESIGN.md`, never hand-authored) starts when the UI is polished, not before. Until then: legible,
  consistent, no dead controls.
- **Read `docs/FINDINGS.md` before debugging or touching the build** — non-obvious gotchas. When you
  discover something non-obvious that cost time and isn't deducible from the code, add an entry.
- **`docs/FACTS.md` is the working-memory file, not a second FINDINGS** — verified facts every fresh
  agent would otherwise rediscover. Read it when you start, append to it when you finish. See
  [Agent orchestration](docs/agents/agent-orchestration.md#agent-orchestration--parallel-where-its-free-batched-where-its-yours).

## Project skills

Installed in `.agents/skills/` (Codex) and symlinked in `.claude/skills/` (Claude Code), pinned in
`skills-lock.json` (`npx skills add <owner/repo> --skill <name>`; reviewed before install):

- `rust-skills` (leonardomso/rust-skills) — Rust guidelines; consult when writing or reviewing Rust.
- `egui`, `egui-kittest`, `egui-review`, `eframe-project` (gregorycarnegie/egui-agent-skills) —
  written for egui 0.36; we are on 0.37, so confirm API claims against the registry source
  (see `docs/FINDINGS.md`). Run `egui-review` before merging UI changes.

## ⚡ graphify — use every session

```text
/graphify            # first run (builds graph from scratch)
/graphify --update   # incremental update (only re-extracts changed files)
/graphify query "<question>"    # architecture questions instead of opening multiple files
/graphify explain "<name>"      # locate a concept or symbol
/graphify path "A" "B"          # dependency path between two modules
```

## ⚡ superpowers — use whenever applicable

Always prefer **superpowers** skills over ad-hoc approaches. If there's even a small chance a skill
applies to the task, invoke it via the `Skill` tool before acting (including before clarifying
questions).

- **Process skills first** — `brainstorming` before creative/feature work, `systematic-debugging`
  before fixing bugs, `test-driven-development` before writing implementation.
- **Then implementation skills** — domain-specific skills guide execution.
- **Verify before claiming done** — `verification-before-completion` / `requesting-code-review`
  before merging.

Flow: `brainstorming → spec (you approve) → writing-plans → plan (you approve) →
subagent-driven-development → finishing-a-development-branch`. **Nothing is implemented without an
approved spec.** *Project note (user, 2026-10-21):* the user delegated the roadmap — the phased spec
in `docs/superpowers/specs/` counts as approved for its phases; a change of direction goes back to
the user.

User instructions always take precedence over skills; skills override default behavior. **Skills
refine *how* the work is done; they never override the rules in this file. When a skill and this
`AGENTS.md` conflict, this file wins.**

### Mode switch

- **"lite mode"** — fully disables superpowers: no skill is invoked, not even the applicability
  check, until **"normal mode"** is said.
- **"normal mode"** (default) — standard superpowers behavior, plus: when delegating coding work,
  dispatch at most 1 agent at a time, and never use a model above Sonnet (no Opus). The cap counts
  **implementation** agents: a read-only review agent runs alongside one, and should — see
  [Agent orchestration](docs/agents/agent-orchestration.md#agent-orchestration--parallel-where-its-free-batched-where-its-yours).
- **"modo desatendido"** (unattended mode) — the user is away and delegates autonomy: work without
  waiting for confirmations and make reasonable decisions yourself instead of asking. Push, PRs and
  merges follow the project override in **Git & GitHub** below. **Pace in this mode:** intermediate
  tasks run only the tests of what they touched (`cargo test -p <crate> <filter>`); the full gate
  runs once at the end of the branch.
- **"modo pocos tokens"** (low-token mode) — same quality and gates, fewer tokens, slower is fine. No
  subagents (the work and a separate self-review pass done inline, with the same skills), terse
  messages, no exploratory work beyond the priority the user set, batched tool calls, long output to
  files (read the tail), progress reports only at milestones. Ends on **"normal mode"**.

Confirm the switch briefly when it happens.

---

## Rules by topic — what always binds, and where the detail lives

This file stays ≤ 32 KiB (`wc -c AGENTS.md`); when it grows, move detail to `docs/agents/`. **The
lines below bind even if you never open the document; open it before working on that topic.**

- **Tests** → [docs/agents/testing.md](docs/agents/testing.md): Rust test stack, per-crate table,
  physics-test conventions. Binding rules below, in *Tests — the rules that bind every branch*.
- **Quality beyond coverage** →
  [docs/agents/quality-beyond-coverage.md](docs/agents/quality-beyond-coverage.md). Mutation gate 60%
  over `engine-core` (+ pure DSP), a ratchet that only goes up; expected values written by hand.
- **Real-environment verification** →
  [docs/agents/real-environment-verification.md](docs/agents/real-environment-verification.md).
  Real audio device and real window can't be proven in-process: `scripts/smoke.sh` drives the
  release binary.
- **Debugging** → [docs/agents/debugging.md](docs/agents/debugging.md), before chasing a bug. Measure
  before ablating; more than three reproductions → a shortcut script before the fourth; a review
  finding is not a reproduction; every assertion is seen failing once.
- **Agentic PR verification (mandatory)** → [docs/agents/pr-verification.md](docs/agents/pr-verification.md).
  Every PR gets a verdict from a pass that runs the real app (headless render + GUI launch), as a PR
  comment.
- **Agent orchestration** → [docs/agents/agent-orchestration.md](docs/agents/agent-orchestration.md).
  Review in parallel with the next implementation; shared `docs/FACTS.md`; plans carry contracts.
- **Reuse first** → [docs/agents/reuse-first.md](docs/agents/reuse-first.md). Search before writing;
  extend, don't clone; extract at the third copy.
- **Design principles (SOLID)** → [docs/agents/design-principles.md](docs/agents/design-principles.md).
  No abstraction without a second implementation, an IO boundary or a test seam.
- **Codex and Claude Code** → [docs/agents/agent-compatibility.md](docs/agents/agent-compatibility.md).
  Rules are edited in `AGENTS.md` (or its `docs/agents/` document), never in `CLAUDE.md`.

**Timing-dependent jobs take the CPU for themselves, nothing else does.** Real-time runs with deadlines and
benchmarks or pacing measurements run under a shared lock with any other session on the machine: holds under
30 min and under `timeout`; a waiter goes ahead at once when the holder is orphaned (its process or its session is
gone), and after 30 min in any case — nobody waits forever, not even a session left alone. Builds and test suites
never take it. Protocol and
script: [agent orchestration](docs/agents/agent-orchestration.md#exclusive-cpu-between-sessions--only-for-jobs-whose-result-depends-on-time)
(`scripts/cpu-lock.sh` here).

## 🧠 Heavy jobs run inside a memory cgroup (MANDATORY)

**No exceptions:** any long or parallel job started here — the full test suite, coverage, mutation
testing, a release build, anything that spawns workers — runs under a kernel-enforced memory ceiling:

```bash
systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- <command>
```

**6 GB is the standing ceiling on this machine**; don't exceed it without being told to. Cap the
tool too (`cargo build -j 8`, `cargo mutants -j 4`), never *instead* of the cgroup. Check that the
ceiling reaches the process doing the work (`cat /proc/<pid>/cgroup`). **Why:** a mutation run on
this 24-core box once spawned 23 workers at ~2.3 GB each and took the machine down; with the machine
starving, timeouts were scored as killed mutants and the result meant nothing.
`scripts/heavy.sh <command>` wraps a command in this scope.

---

## Stack

| Area | Crate | Role |
| --- | --- | --- |
| Language | Rust (stable, edition 2024) | everything |
| UI + rendering | `eframe` / `egui` (wgpu backend → Vulkan/Metal/DX12/GL) | editor, animated engine view, gauges, plots |
| UI plots | `egui_plot` | torque/power curves, p–V, pressure traces |
| Audio out | `cpal` | device I/O; callback only pops samples |
| Lock-free | `rtrb` | SPSC ring buffer sim → audio |
| SIMD | `wide` | four cells at once in the 1D pipes' WENO reconstruction (SSE2 baseline, bit-identical) |
| Config | `serde` + `serde_json` + `serde_yaml_ng` | engine import/export (JSON/YAML) |
| CLI | `clap` | `--engine`, `--headless`, `--wav` |
| WAV | `hound` | headless render for tests/smoke |
| Tests | `proptest`, `approx`, `egui_kittest` | invariants, float compare, UI tests |

Planned (add when the phase needs them, then update this table): `fft-convolver` (IR), `rubato`
(resampling), `criterion` (benches), `no_denormals` (FTZ).

## Workspace structure

```text
/
├── crates/
│   ├── engine-core/   # pure physics, no IO: units, geometry (crank-slider), gas (0D volumes,
│   │                  #   orifice + implicit port flow), waveguide + exhaust (pipes, collectors,
│   │                  #   open ends), combustion (Wiebe), real-gas cylinder properties (thermo: NASA c_p),
│   │                  #   friction by components (Sandoval), block radiation (structure response), dynamics, load/dyno,
│   │                  #   gearbox (drivetrain: vehicle, clutch), engine definition (serde) + presets. 100% deterministic, f64.
│   ├── engine-audio/  # pure DSP (DC blocker, filters, radiation, later convolution) + cpal output
│   │                  #   behind an `AudioSink` boundary.
│   └── engine-app/    # binary `engine-sim`: eframe UI, sim thread, CLI, headless renderer.
├── presets/           # built-in engines as YAML (`--save`); a test keeps them equal to the code
├── scripts/           # smoke.sh, heavy.sh, verify-pr.sh
└── docs/              # specs, research, agents/, FINDINGS, FACTS
```

Threads: **sim thread** (fixed step, produces audio samples into `rtrb`), **audio callback**
(`cpal`, pops only — no alloc, no lock, no IO), **UI thread** (egui; sends controls, reads state
snapshots). Audio fill level drives how many sim steps run per block (see research doc).

## Commands

```bash
cargo run -p engine-app --release                       # the app
cargo run -p engine-app --release -- --preset inline4      # single | inline4 | lfa-v10
cargo run -p engine-app --release -- --engine presets/lfa-v10.yaml   # any .yaml/.yml/.json
cargo run -p engine-app --release -- --preset lfa-v10 --save my.yaml # write an engine file
cargo run -p engine-app --release -- --preset lfa-v10 --dyno         # WOT sweep as CSV rpm,torque_nm,power_hp
cargo run -p engine-app --release -- --preset lfa-v10 --render drive --out drive.wav  # offline, Reference quality (drive | sweep | loops → folder)
cargo test -p engine-app render_screens -- --ignored    # off-screen UI renders → target/screens/
cargo run -p engine-app --release -- --headless --seconds 3 --wav /tmp/out.wav --check
cargo test --workspace                                  # all tests
cargo clippy --workspace --all-targets -- -D warnings   # lint (gate)
cargo fmt --all --check                                 # format (gate)
scripts/coverage.sh                                     # coverage gate (80% lines)
scripts/mutants.sh --in-diff <(git diff vX.Y.Z..main -- crates/engine-core)  # periodic run since the last one
scripts/mutants.sh                                      # full run (hours; floor 90%, ratchet)
scripts/smoke.sh                                        # real-environment smoke
```

System deps (Arch): `alsa-lib`, `vulkan-icd-loader`, `llvm` (for coverage). Tools:
`cargo install cargo-llvm-cov cargo-mutants cargo-nextest` (mutants run under nextest: `.cargo/mutants.toml`).

---

## Tests — the rules that bind every branch

Stack and conventions: [docs/agents/testing.md](docs/agents/testing.md).

- **Coverage gate: 80%** lines over the workspace, excluding UI rendering glue and the `cpal`
  device code (justified in `docs/agents/testing.md`). Critical physics ≥ 90%.
- **Mutation gate: 60% minimum** over `engine-core`; current floor **90%** (`scripts/mutants.sh`,
  measured 92.4% on 2026-10-21) — a ratchet, raised with the score, never lowered. See
  [Mutation gate — the 60% floor](docs/agents/quality-beyond-coverage.md#mutation-gate--the-60-floor-and-it-only-goes-up).
  **Cadence (user, 2026-11-03): it does not block PRs.** A PR merges on the fast gates (fmt,
  clippy, tests, coverage, smoke) and its verify-pr verdict; the mutation run goes over
  everything merged since the last one, weekly or when the machine is free, and its misses are
  pinned in a follow-up PR. A PR touching delicate physics may add a sample of its diff
  (`scripts/mutants.sh --in-diff … --shard 0/8`, minutes).
- **The "E2E" of this project is the real binary**: headless render (WAV non-silent, finite, at the
  right firing frequency) + GUI launch smoke in `scripts/smoke.sh`, and `egui_kittest` tests that
  drive the editor widgets (click, drag sliders, read the state they changed).

### The pyramid per feature

**One smoke/journey per feature** (e.g. "load preset → render → hear the firing order"), everything
else one layer down: unit tests of the physics module, `proptest` invariants, `egui_kittest` for UI
logic. A UI bug gets its failing test first, at the lowest layer that reproduces it.

### Hard rules (no exceptions)

- **Never claim done without showing test output.** "It compiles" is not "it works".
- **New public function / type / config field → needs a test.** Serialization changes need a
  round-trip test (JSON and YAML).
- **A bug fix needs a failing regression test first**, then the fix (see `systematic-debugging`).
- **Never delete, `#[ignore]` or comment out a test to get green.** Fix the code or the test on purpose.
- **Physics expected values come from outside the code**: a hand calculation, a closed form, a
  textbook or paper table (cite it in a comment), never the same formula re-typed in the test.
- **Don't lower the 80% gate to ship** — exclude untestable modules in config with a written reason.
- **Test over mock** — exercise real code; the only seams are IO boundaries (`AudioSink`, file IO).

### TDD — required for new logic

All new logic in `engine-core` and `engine-audio`, and logic-bearing UI code (editor model,
validation, undo): **Red** (failing test) → **Green** (minimum code) → **Refactor**. Exceptions:
pure layout/visual changes; spikes — but tests before merging.

## Working rules

- **Heavy or parallel jobs run inside a memory cgroup** — `scripts/heavy.sh` or the `systemd-run`
  line above, plus the tool's own concurrency cap.
- **Use superpowers skills whenever they apply** — invoke via `Skill` before acting; process skills
  before implementation skills.
- **New dependencies: install, then say so** — no prior ask (user, 2026-10-05). Check `Cargo.toml`
  first so one job doesn't get two crates; name every new crate in the PR (which, why, what it
  replaces). Verify the crate exists on crates.io and is the one you think it is.
- **Fast and light by default** — this must run on old machines. Pick the option with less CPU and
  memory, and **measure** (bench or `--headless` timing) when a change touches the hot loop.
- **Real-time audio rules (Bencina):** the `cpal` callback never allocates, locks, does IO or runs
  work of unknown duration. Only `rtrb` pop + gain.
- **Physics in `engine-core` is pure and deterministic** — no IO, no threads, no global RNG (seeded
  RNG passed in), f64 for thermodynamic state, f32 for audio post-processing.
- **SI units internally** (m, kg, s, Pa, K, rad). Conversions (mm, cc, rpm, hp, bar, °) only at the
  UI/config boundary, through `engine-core::units`. Field names carry the unit when ambiguous
  (`bore_m`, `rpm`).
- **Engine definitions are data** (`serde`), never code. Every field has a default so old files keep
  loading; a breaking format change bumps `version` and adds a migration with a test.
- **Cite the source** of every physical model or constant in a doc comment (paper/book + equation or
  section), as the research doc does. Don't copy GPL code (OpenWAM, ensim3): read, then reimplement.
  MIT code (engine-sim, ensim4, enginesound) may be ported with attribution in `NOTICE`.
- **No `unwrap()` / `expect()` outside tests and `main` startup** — return `Result` with `thiserror`
  types in libraries.
- **The topic rules bind from *Rules by topic*** — not repeated here, so they cannot drift.
- **Keep this file's Stack/Workspace sections current** — when a phase adds a crate or module,
  update them in the same change.
- **Commits in English**, Conventional Commits. Scope = crate or folder (`feat(core): …`).
- **Progress tags for the timelapse** — after each merged milestone, tag `main` with an annotated
  tag `vMAJOR.MINOR.PATCH` (`git tag -a v0.2.0 -m "<what is usable now>"`) and push it. Every tag
  must build and run. The list of tags and what each shows lives in `CHANGELOG.md`.

## Git & GitHub

- **Commits and branches OK** — create commits and new branches whenever it makes sense, without asking first.
- **Branches:** `feat/name`, `fix/description`, `chore/task`. Never commit straight to `main`.
- **Project override (user, 2026-10-21): push, PR and merge are authorized.** The agent may push the
  feature branches it creates, open PRs with `gh`, **merge its own PRs** once the gates are green and
  the PR verification verdict is posted, and push the progress tags. Still forbidden in every mode:
  `git push --force` / `--force-with-lease`, rewriting `main`'s history, and merging a PR with a red
  gate. Merge with `gh pr merge --merge --delete-branch` (merge commit, no squash/rebase — keeps the
  timelapse history readable). Don't stack PRs: branch from `main`.
- **GitHub via `gh`** — PRs, issues, comments, labels.
- **Every PR must include a manual test plan** — a **How to test manually** section with the exact
  commands (`cargo run -p engine-app --release -- …`), what to click/drag, and what you should see and
  hear.
- **Agentic PR verification (MANDATORY on every PR)** — see
  [docs/agents/pr-verification.md](docs/agents/pr-verification.md); verdict posted with
  `gh pr comment` before merge.
