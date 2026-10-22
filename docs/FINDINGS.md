# FINDINGS — engine-sim

> This is the repo's **non-obvious gotchas log**. `AGENTS.md` § *Start here* makes it
> mandatory reading before debugging or touching the build, and mandatory writing whenever you
> discover something that cost you time and **is not deducible from the code**.

## What goes here

An entry earns its place only if **all** of these hold:

- It cost real time to discover (a wrong path taken, a silent failure, a misleading error).
- It is **not** deducible by reading the code or the tests — a reader of the source would not
  infer it.
- It will bite again: it's a property of the stack, the environment or the tooling, not a
  one-off typo you already fixed.

Not here: how the architecture works (that's `AGENTS.md`), what a feature does (that's
`user-stories.md`), or a bug you fixed in the same change (that's the commit + its regression test).

## Format

One `###` entry per finding, newest first. Keep each to a few lines — a long entry means it
belongs in `AGENTS.md` instead.

```markdown
### <Short symptom, as you'd search for it>

**Symptom.** <What you actually saw — the error string, the wrong output, the hang.>
**Cause.** <The real mechanism, once known.>
**Fix / workaround.** <The command, flag or code change that resolves it.>
**Why it isn't obvious.** <Why the code doesn't tell you this.>

Found: <YYYY-MM-DD> · <area: build | tests | db | ci | deploy | env>
```

---

## Findings

### cargo-mutants with a shared CARGO_TARGET_DIR gives meaningless verdicts

**Symptom.** Impossible survivors (`white() -> -1.0` survived statistical tests of `white`).
**Cause.** With `CARGO_TARGET_DIR` set, the `-j4` mutant jobs (each a separate copy of the tree)
build into one target dir and overwrite each other's binaries; tests run against another mutant.
**Fix / workaround.** `scripts/mutants.sh` unsets `CARGO_TARGET_DIR`. Gate results of PRs #4 and #5
were taken with it set; a clean re-check over main (exhaust + drivetrain diff) was stopped after 80 of
625 mutants to keep the machine's load low for another session: 77/80 caught (96 %) so far.
Mutation runs now use `-j 3` with `CARGO_BUILD_JOBS=4` (≤ ~12 threads; load must stay < 15).
**Why it isn't obvious.** The run looks normal and most mutants still get a plausible verdict.

Found: 2026-10-22 · tests

### Open (improving): FV exhaust, coast still louder than WOT at the firing order (LFA 7000)

**Symptom.** Source at the tail, firing order 583 Hz: coast vs WOT was 7.4×; a real engine is
5–10 dB quieter on overrun.
**Found and fixed so far.**
- The 0D merge junction was a u = 0 stagnation plenum: it passed 35 % of a straight pipe's tone.
  Characteristic (Benson) junction → transparent.
- Overrun manifold at 0.06 bar (fixed idle air, 8× pumping): ECU dashpot → 0.17 bar. Biggest win.
- Inviscid pipes rang forever: Darcy wall friction (f = 0.025).
- Overrun near-vacuum broke the fixed FV step's CFL (a primary reached 1000 bar): sub-cycling.
**Now.** Coast/WOT = 2.2× at the firing order. Ruled out: reverse-flow Cd (added, ratio 0.75:
no change — the coast tone is not valve-area limited). Next: compare with real recordings; the
1D intake (induction-side reversion through the cylinder).
**Tools.** `examples/{port_probe,sound_breakdown,spectrum_probe,blowup_probe}.rs`.

Found: 2026-10-22 · physics

### Other sessions run real-time gates on this machine: share the CPU lock

**Symptom.** Another project's differential gates (real-time game routes) failed while our cargo
builds/tests ran at ~1400 % CPU.
**Cause.** Load above ~15 breaks their real-time timing.
**Fix / workaround.** Timing-dependent jobs (benchmarks, real-time factor) run through
`scripts/cpu-lock.sh` (flock + owner line v2 + timeout 1800). Builds/tests/lint never take the
lock (user rule); they run via `scripts/heavy.sh` at nice 19 with cargo -j8. Agreed 2026-10-22.
**Why it isn't obvious.** Nothing in this repo tells you another session shares the box.

Found: 2026-10-22 · env
### `--quit-after` never closed the window when it was hidden

**Symptom.** Smoke GUI phase hung for minutes; several `engine-sim --quit-after 4` processes alive.
**Cause.** eframe calls `App::ui` only while the window is visible; the quit timer lived in `ui`.
The smoke window opened behind the user's windows (shared desktop) and never drew.
**Fix / workaround.** Quit timer in `App::logic` (runs even when hidden) + `request_repaint_after`;
the smoke runs the GUI under `timeout 30`.
**Why it isn't obvious.** On an idle desktop the window is visible and the bug never shows.

Found: 2026-10-22 · app

### cargo-mutants filled /tmp (tmpfs = RAM) with a 5 GB copy

**Symptom.** RAM near exhaustion during a mutation run; `shared` memory ~8 GB.
**Cause.** cargo-mutants copies the project (with its build) into `$TMPDIR`, and `/tmp` is tmpfs on
this machine, so the copy lives in RAM (and is not reclaimable without swap).
**Fix / workaround.** `scripts/mutants.sh` sets `TMPDIR=target/mutants-tmp` (disk).
**Why it isn't obvious.** The run itself was inside the 6 GB cgroup; the tmpfs pages showed up as
"shared", not as the job's RSS.

Found: 2026-10-21 · build

### Exhaust port ↔ waveguide: an explicit coupling oscillates at Nyquist

**Symptom.** Radiated sound alternated ±20 kPa every sample (hiss); closed throttle louder than WOT.
**Cause.** Valve flow used last step's port pressure; near Δp ≈ 0 the orifice law (ṁ ∝ √Δp) has a
huge slope, so the loop p⁺ → ṁ → p⁺ had gain > 1 at fs/2.
**Fix / workaround.** Solve the port implicitly: ṁ = f(p_cyl, p_base + (c/A)·ṁ) with regula falsi
(`gas::port_flow`). Regression test: `exhaust_source_has_no_sample_to_sample_oscillation`.
**Why it isn't obvious.** Every unit test of the pieces passes; only the coupled loop is unstable.

Found: 2026-10-21 · physics

### The open pipe end must hand the *outflow velocity* to the radiation stage

**Symptom.** After adding waveguides, everything hissy; sharp reversion edges dominated.
**Cause.** The open end output p + p_reflected (pressure at the end, ≈ 0 at low frequency, i.e.
already a derivative) and the voice differentiates again (monopole). Two derivatives.
**Fix / workaround.** Output p − p_reflected (∝ volume velocity); the voice's single d/dt is the
monopole far field (PASP, point source).
**Why it isn't obvious.** Both look like "the radiated pressure"; the difference is one derivative.

Found: 2026-10-21 · physics

### Known limit: WOT is not clearly louder than coasting (waveguide exhaust, v0.3)

**Symptom.** At the same rpm, closed throttle is about as loud as WOT (LFA at 7000 rpm: louder).
**Cause (hypothesis, not proven).** Linear lossless-ish waveguides without mean flow: reversion
pulses at high vacuum are sharp and the pipes ring; real exhausts damp large amplitudes (mean
flow, non-linear losses, heat). Firing-order structure is right (I4 orders 4/8, V10 order 10).
**Fix / workaround.** Partly fixed: a muffler in the tail (pipes rang without one) and a floor on
the waves (they superposed below vacuum at the port). Single/I4 now louder at WOT at high rpm; the
LFA at 7000 rpm still ~2× louder coasting. Pending: non-linear losses or the 1D FV exhaust.
**Why it isn't obvious.** Torque and the order structure look right; only the level balance is off.

Found: 2026-10-21 · physics

### egui panics on a glyph the default font lacks (e.g. "→")

**Symptom.** A kittest test panics: `No glyph for "→" (U+2192) in Proportional ... Install a font that has it`.
**Cause.** epaint 0.37 panics (instead of drawing tofu) when a label contains a character missing from the bundled fonts; arrows and the like are missing, "·", "°", "²" are fine.
**Fix / workaround.** Use ASCII ("to", "->") in UI text; check labels in a kittest run, not only by eye.
**Why it isn't obvious.** The text is only laid out when that widget is drawn, so it only fails in the state that shows it (here: while a dyno sweep runs).

Found: 2026-10-21 · tests

### `cargo llvm-cov` fails: "failed to find llvm-tools-preview"

**Symptom.** `cargo llvm-cov` asks for `rustup component add llvm-tools-preview`; there is no rustup.
**Cause.** Rust comes from Arch's `rust` package, which ships no llvm-tools component.
**Fix / workaround.** Use the system LLVM (same major as `rustc -vV`): `LLVM_COV=/usr/bin/llvm-cov
LLVM_PROFDATA=/usr/bin/llvm-profdata`. `scripts/coverage.sh` sets them.
**Why it isn't obvious.** The error points at rustup, which is not installed here at all.

Found: 2026-10-21 · build

### egui 0.37: `App::ui` replaces `update`, `Panel::left` replaces `SidePanel`

**Symptom.** Examples online use `SidePanel::left(..).show(ctx, ..)` and `fn update(ctx, frame)`.
**Cause.** egui/eframe 0.37 renamed them: `eframe::App::ui(&mut self, ui, frame)`,
`egui::Panel::{left,right,top,bottom}(id).show(ui, ..)`, `CentralPanel::default().show(ui, ..)`;
`rect_stroke` takes a `StrokeKind`.
**Fix / workaround.** Read the registry source (`~/.cargo/registry/src/*/egui-0.37*/`), not old docs.
**Why it isn't obvious.** Most tutorials and model memory predate the rename.

Found: 2026-10-21 · build

### kittest: a labelled `Slider` matches two nodes

**Symptom.** `get_by_label("Spark advance …")` panics: "Found two or more nodes".
**Cause.** `Slider::text` labels both the slider and its value box.
**Fix / workaround.** `get_by_role_and_label(egui::accesskit::Role::Slider, label)`.
**Why it isn't obvious.** Buttons and checkboxes with the same API match exactly one node.

Found: 2026-10-21 · tests

### Hand sabotage that restores the file with `mv` leaves cargo on the mutant build

**Symptom.** After a sabotage loop (mutate → `cargo test` → `mv file.bak file`), the next test run
fails exactly as the last mutant would, though the source is clean.
**Cause.** The backup's mtime is older than the mutant's, and cargo fingerprints by mtime: the
restored file looks unchanged since the last (mutant) build, so nothing is rebuilt.
**Fix / workaround.** `touch` the file after restoring it (the sabotage script does).
**Why it isn't obvious.** Each mutant does rebuild (it is newer), so every verdict is right; only
the run *after* the loop silently tests stale code.

Found: 2026-10-22 · tests

### A volume's pressure is the stagnation pressure for gas leaving it into a pipe

**Symptom.** Plenum + 1D runners read VE 1.20 at 6000 rpm, above unrestricted ITBs (0.98) on the
same runner. There was no tuning to explain it.
**Cause.** The junction ghost gave inflow `p = p_j, ρ = ρ_j` with u ≠ 0. The entering gas then had
a stagnation pressure above the volume's, so it accelerated without paying the dynamic head.
**Fix / workaround.** Treat inflow as a contact problem. The pipe side follows the outgoing
characteristic on the cell's isentrope; the volume side is adiabatic from rest (stagnation).
Solve for the shared p and u (`junction_ghost`, Illinois root in x = (p/p₀)^((γ−1)/2γ), no `powf` in
the loop). VE is now 1.00.
**Why it isn't obvious.** At u = 0 both forms agree, and every resting or small-signal test
passes. The error grows with u², so it only shows at high flow.

Found: 2026-10-22 · physics

### An explicit orifice flux from the cell pressure chatters at the FV rate

**Symptom.** The LFA with ITBs: output clipped at 1.0, RMS 20× the 0D intake, and almost no energy
in engine orders.
**Cause.** The trumpet's face flux was a quasi-steady orifice evaluated from the first cell's
pressure. Its slope dṁ/dΔp → ∞ at Δp → 0, so each FV step overshoots, and the face flux flips
sign every step. The step-to-step difference was larger than the swing.
**Fix / workaround.** A throttled characteristic end (`throttled_ghost`): the orifice limit is
solved together with the outgoing characteristic. Test
`pulsed_runners_radiate_without_step_to_step_chatter` checks diff-RMS < 0.3 × RMS.
**Why it isn't obvious.** Mass balance, VE and torque all look normal; only the sound shows it.
Look at the step-to-step difference of a source, not its mean.

Found: 2026-10-22 · physics

### `IntakeFv::last_inducted() / dt` is a spike train

**Cause.** With `substeps` > 1, the whole FV step's air arrives on the one physics step that
advances the pipes. That's exact for mass, but a sound source fed with it gets spikes `substeps`×
the flow, and hiss ∝ ṁ·u goes as substeps².
**Fix / workaround.** Sound reads `inducted_rate()` (held mean over the last FV step); mass
bookkeeping keeps `last_inducted()`.

Found: 2026-10-22 · audio

### A frozen mutation run scores its mutants as timeouts

**Symptom.** While another session held the CPU lock, a diff gate tested 6 mutants in 40 minutes,
and all 6 were timeouts.
**Cause.** `yield-cpu.sh` freezes the whole scope, cargo-mutants included. Its timeout is
wall-clock, so on thaw every running test has already used up its 90 s.
**Fix / workaround.** `mutants.sh` retries the timeouts twice by exact name (`-F '^…$'`). A
truly hung mutant times out again cheaply. cargo-mutants 27 adds its field-deletion mutants
whatever `-F` says, so only the retried names are counted.

Found: 2026-10-22 · tests
