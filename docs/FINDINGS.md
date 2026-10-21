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

### Other sessions run real-time gates on this machine: share the CPU lock

**Symptom.** Another project's differential gates (real-time game routes) failed while our cargo
builds/tests ran at ~1400 % CPU.
**Cause.** Load above ~15 breaks their real-time timing.
**Fix / workaround.** `scripts/heavy.sh` takes `flock /tmp/claude-1000/cpu-heavy.lock`, writes
the owner to `cpu-heavy.owner`, runs at nice 19 with cargo -j8. Keep each hold < 30 min (split
mutation runs per file). Agreed between sessions on 2026-10-22.
**Why it isn't obvious.** Nothing in this repo tells you another session shares the box.

Found: 2026-10-22 · env

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
