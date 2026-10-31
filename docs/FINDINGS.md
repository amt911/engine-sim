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

### The V10's "mosquito": its two banks were heard from one point, so their orders cancelled

**Symptom.** The LFA at full load sounds like a pure tone on the firing order (user: "muy agudo y
sintético, como un mosquito"). Held 6700 rpm WOT (`--render held:6700:1.0:3`, `ang.py`): the
half-firing orders (cycle orders 5, 15, 25, 35: 279, 837, 1395, 1953 Hz) at −40.6/−42.8/−42.7/
−52.1 dB re the firing line against a real LFA on a chassis dyno at 6060–7300 rpm at
−14.5/−15.3/−24.0/−18.8; the 250–500 Hz octave −30.6 dB re total against −17.5 real.
**Cause.** Exact symmetry at one point: the block was driven by the mean of all ten cylinders'
pressures and heard from one place (`structure::combustion_drive`), and the two bank exhausts are
identical into one junction. Two banks firing in turn cancel every odd multiple of the bank order
when summed at one point; only the 2 % cylinder imbalance was left.
**Fix / workaround.** `Listener::with_engine_at(ahead, across)`: one source per bank
(`Engine::engine_sources_across`, the head's offset (rod + stroke/2 + bore)·sin(θ/2), 0.157 m for
the LFA, a geometric estimate), the intake mouth on the centre line; each bank's side of the block
radiates 1/n of Anderton's whole-engine response driven by its own cylinders. Held 6700 after:
half orders −38.4/−24.5/−25.2/−34.2 (orders 15 and 25 within 10 dB of real; order 5 still −38:
the block passes little at 279 Hz).
**What was tried and left out, with numbers.** Splitting the ITB trumpets per bank too puts the
5th order at **+1.8 dB** re the firing line (real −14.5) and the 250–500 Hz octave at −2.9 (real
−17.5): without the surge tank between the trumpets and the air (Okamoto, SAE 2011-01-2024, Fig.
18: each bank's ITBs breathe from its own tank, "400 Hz tuned") the bank order of ten open
trumpets is 16 dB too loud, and one point hides that by cancelling it. Per-bank mouths belong
behind the surge tank model (needs `IntakeFv` per-runner drawn rates: `inducted_each` per runner,
~20 lines; the texture test `the_texture_radiates_like_a_monopole_from_the_tails` then needs the
texture per source).
**What remains (measured, same held comparison).** Firing harmonics H3–H8 at −27…−39 dB re H1
against −8…−22 real (2–8 kHz 19–21 dB short): neither the catalyst (plain pipe instead: H3 −19)
nor the merge muffler (pipe instead: H4 −19) nor the listener (ISO 362 position) changes the
trend — the pulse front itself (non-linear steepening, branch feat/wave-steepening) or the valve
pulse. Order 5 (279 Hz) needs the real bank asymmetry of the exhaust (Fig. 19: two separate systems
of different routing, no lengths published).
**Why it isn't obvious.** Each source was placed physically (engine ahead of the tails, 1/r,
delayed) and each cylinder differs by the measured few percent; nothing in the code says the sum
of identical banks at one point is a cancellation. Dyno clips carry roller/tyre noise: the
broadband share (periodic 0.30 at 4–8 kHz) is not all engine; the lines are.

Found: 2026-10-29 · physics

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

### Never run the mutation gate on the tree you are editing

**Symptom.** "The diff might be out of date with this source tree", then "FAIL: no mutants tested".
**Cause.** The run started frozen (another session held the lock). By the time cargo-mutants read
the source, `engine.rs` had changed under it, and `--in-diff` no longer matched.
**Fix / workaround.** Run gates in a detached worktree at the PR commit
(`git worktree add --detach target/wt-gate <branch>`) and keep working in the main checkout.

Found: 2026-10-22 · tests

### A sound source summed over outlets is not one outlet at the sum

**Symptom.** The LFA at 8000 rpm WOT was 87 % broadband "air"; the inline-4 was clean.
**Cause.** Jet noise took u from `radiated`, the c·ρu summed over every tail pipe. Two tails gave
one jet at twice the speed, so 4× the dynamic pressure, where two incoherent jets give √2:
+9 dB.
**Fix / workaround.** `Texture::flow_noise(radiated, outlets, …)`: per-outlet velocity, √n sum.
Any nonlinear source (∝ u², ∝ ṁ·u) has to be evaluated per outlet.

Found: 2026-10-22 · audio

### egui's default slider clamping rewrites values just by drawing them

**Symptom.** Showing the LS3 preset in the editor turned its 103.25 mm bore into 103 and marked the
engine dirty, without anyone touching it.
**Cause.** `Slider` defaults to `SliderClamping::Always`. `add_contents` calls `set_value(old)`
every frame, and `set_value` applies `step_by` rounding (egui 0.37 `slider.rs`).
**Fix / workaround.** Every editor slider goes through `editor::slider()` with
`SliderClamping::Edits`: user edits are clamped and stepped, existing values stay intact. Test:
`showing_an_engine_does_not_round_its_values`.

Found: 2026-10-22 · ui

### A fixed spark advance stalls a light, high-compression engine floored from idle

**Symptom.** The flat-plane V8 at full throttle from 900 rpm stopped within 0.1 s; it idled fine
and pulled from higher rpm.
**Cause.** Advance is fixed at 28° BTDC. At 900 rpm a full 12.5:1 charge burns early enough to
push against the crank, and a 0.08 kg·m² flywheel has too little energy to carry it through.
Real ECUs retard the spark at low rpm and high load.
**Fix / workaround.** `EngineDef::spark_advance_at(rpm)`: the configured advance applies from
4000 rpm and is retarded below that, × (0.35 + 0.65·rpm/4000), as an ECU map does. The flat-plane
is back on its 0.08 kg·m² flywheel. Tests: `every_preset_revs_up_from_idle_at_full_throttle`,
`a_retarded_idle_spark_lets_a_light_flywheel_rev_up`.

Found: 2026-10-22 · physics

### A spark angle that moves between steps misfires or fires twice; it looked like idle hunting

**Symptom.** Under closed-loop idle control, the LFA and the V8s hunted by ±40 % and stalled. Gain
sweeps hardly changed this. The same engines idled smoothly at a fixed spark advance.
**Cause.** A spark was detected as "the crank crossed the spark angle between the previous and
current step". When the ECU advanced the timing past a crank sitting just before the old angle,
the crossing never happened (misfire). Retarding it past a crank that had just fired made it fire
again, restarting the burn. In a run where it jumped one way and crept back, this measured 200
firings for the expected 100.
**Fix / workaround.** One spark per charge. Intake closing arms the cylinder, and it fires on the
first step at or past its spark angle (within the following half cycle). Test:
`a_moving_spark_fires_each_charge_exactly_once`. After the fix, idle holds within a few percent.
Lesson: when gains don't move a control symptom, look for a discontinuity in the plant first.

Found: 2026-10-22 · physics

### A constant wall heat coefficient starves a throttled engine; the idle physics was otherwise sane

**Symptom.** The cross-plane could not idle below ~1000 rpm, and falling from a blip it stalled.
**Cause.** Partly the constant 350 W/(m²·K) stand-in for Woschni: at 26 kPa manifold pressure it
over-cools the thin charge. An energy balance at 700 rpm (gross IMEP 2.07 bar, PMEP 0.76, FMEP
1.09) otherwise matched a hand calculation. Woschni added only ~3 N·m there. The falls were
mostly manifold dynamics: the manifold drains during the overrun cut and refills too late,
which real cars cover with a throttle dashpot and closed-loop idle air and spark.
**Fix / workaround.**
- `woschni_htc` and `woschni_velocity` (heat now scales as p^0.8 with load; test against the
  published correlation, and the heat-vs-load ratio within 15 % of (charge ratio)^0.8).
- A throttle dashpot (2 % hold, τ = 1 s).
- An ECU idle-speed control: PI on the bypass, scaled by the air the engine swallows at idle,
  plus spark P+D from a 10° retarded reserve.
- `idle_rpm` per engine.

Tests: `every_preset_holds_its_idle_speed`, `every_preset_starts_idles_and_survives_a_blip`.

Found: 2026-10-22 · physics

### Overrun sounded as loud as full power: three source errors, one still open

**Symptom.** Downshifts sounded wrong. The acoustic (DC-free) output on overrun with fuel cut
came out at −5 to +6 dB of full throttle at the same rpm, on both exhaust models and at every
speed. Motored tailpipe noise is well below fired. The leveler made it worse by compressing
whatever was left.
**Cause.** Three physics errors, measured one at a time:
- **Mass instead of volume.** An open end radiates volume velocity (p = ρ₀/4πr · dQ/dt). FV
  radiated c·(ρu) and the waveguide injected c·ṁ/A as if all gas were at its hot reference, so
  cold, dense reversion gas counted ~3× (+9.5 dB).
- **A fixed reversion temperature.** The waveguide pushed reversion back at a fixed 800 K, even
  when the primaries held cold pumped air.
- **FV valve chatter.** The FV valve exchanged with a port cell that only updates every pipe
  step, so on overrun its flow flipped direction every pipe step (127 reversals in 7 cycles), a
  27 kHz sawtooth heard as "air".

**Fix / workaround.**
- `exhaust_source_flow`: the volume flow, ṁ·T/T_ref, about C_PRIMARY's gas.
- FV radiates ρ_ref·c·u.
- A 0D primary gas temperature: mixing, plus cooling to a slow wall.
- The FV valve uses the waveguide's implicit port against the first cell's characteristic
  (impedance c/A).

The valve's volume source is now ≥ 8 dB down on overrun.

**Still open.** The *radiated* level on overrun stays close to full power. The tail spectrum is
dominated by the firing fundamental in both cases, so the open question is muffler acoustics
(no absorption, and frequency-independent duct losses). Data requested in the local
PAPERS.md. Tests: `the_exhaust_source_is_the_volume_flow`,
`overrun_pumps_less_volume_through_the_valve_than_full_power`,
`reversion_gas_is_as_hot_as_the_primary_holds`,
`the_fv_exhaust_valve_does_not_chatter_on_overrun`, `the_port_holds_what_the_valve_already_pushed`.

Found: 2026-10-22 · physics

### Revs swept 2.5× too fast, idle hung at the fuel cut-off, limiter fired at random

**Symptom.** Blips and downshifts sounded synthetic ("air", a siren-like smear). The spectrograms
(`drive_render` + `scripts/spectrogram.py`) showed three things:
- The LFA went from idle to 9000 rpm in 0.23 s; Tanahashi measured 0.6 s.
- After a lift-off it hung at ≈ 1500 rpm.
- The rev limiter cut random cylinders, 25–50 % of the energy off the engine orders.

**Cause.**
- **Inertia.** The preset inertias were guesses; published engine inertias are ≈ 0.15–0.25 kg·m².
- **Idle air and gating.** The rpm-proportional "decel" idle air was always on above 1500. The
  idle loop was gated off above 1500, which is the fuel cut-off speed, and the cut had no
  hysteresis.
- **Dashpot units.** `DASHPOT_HOLD = 0.02` was a fraction of plate *area*; a dashpot holds a few
  percent of *travel* (3 % travel = 0.11 % area). After every blip the engine had ~4× the idle
  air at fuel reactivation, so it surged back to the cut-off and hunted.
- **Integral freeze.** The idle integral was frozen whenever the engine was above target and
  falling, so a heavy crank's slow start-flare decay was never trimmed.

**Fix / workaround.**
- The LFA's inertia (0.28) comes from its measured snap: test
  `the_lfa_snaps_from_idle_to_9000_in_six_tenths`. That is at the top of the published range,
  which hints that our transient torque is a little high (no wall-film fuel lag on tip-in).
- Injection cut-offs with hysteresis (overrun: reactivate at idle + 300; limiter: 200 below the
  redline). The decel air applies only while the overrun cut-off holds.
- The dashpot is held in travel.
- The integral holds only if the fall reaches the target within 1 s.

Tests: `released_from_high_revs_every_engine_settles_at_its_idle`,
`the_rev_limiter_bounces_off_the_redline`, `overrun_pulls_the_manifold_to_a_deep_vacuum`.

### Overrun ≈ full power at the tail: the firing orders barely drop (open)

**Measured** (`coast_chain`, an ignored test in engine.rs; held at 0.7 × redline):
- The valve's volume source falls 14 dB on overrun (inline-4; V10 −13.6; V8 −11.4), close to
  the choked-flow estimate (−16 dB).
- At the tail it falls only 1–5 dB.
- None of these explain it:
  - the synthetic layers (±2 dB);
  - the waveguide's wave clamp (unchanged without it);
  - valve chatter (1–2 flow reversals per cycle in both states).
- **Per order**, the source drop is all in the low orders. For the inline-4 at 4900 rpm the
  source changes by −21, −16, −12, −8, −3, −2, 0, +4 dB at orders 1–8. The tail only carries the
  firing orders (4, 8, 12), which drop 2–8 dB.
- Either the full-power blowdown is too smooth (too little high-order energy), or the overrun
  valve event is too sharp. A real tailpipe drops 15–25 dBA at part throttle on the half order
  (US patent 11391195).

**Next.** Compare against a measured source or tail spectrum, and redo this under the
non-linear 1D model (temperature-dependent c, steepening of the blowdown).

## FV junction: still gas blew up to 100 Pa because a root finder gave up (2026-10-23)

**Symptom.** A still FV exhaust stayed exactly still, but only because its state was exact. Add
round-off noise (1e-14 of the energy) and within 0.05 s the junction jumped from 1e-9 Pa to
100–1500 Pa. The jump was a single step, not exponential growth. MUSCL (v0.10.1) did the same,
so WENO was not the cause. The test `quiet_system_stays_quiet` could not see it.

**Cause.** Rounding sent a junction at rest down the *inflow* branch of `junction_ghost`. That
branch's root lies within 1e-16 of x = 1, where the junction-side speed √(1 − x²) has an
infinite slope. The bracket ends had values −655 and 1.2e-26, so Illinois regula falsi
crawled. After its 40 tries `rising_root` returned the middle of the bracket: a boundary
pressure of 74 kPa instead of 101 kPa, which dealt the junction a 27 kPa blow.

**Fix.** `rising_root` takes a bisection step whenever three iterations have not halved the
bracket. It runs until the bracket is tight and returns the end with the smaller |f|.

**What misled.** Four times shorter sub-steps "fixed" it, and running the junction through
the RK3 stages halved it. Both hinted at a time-lag instability; it was not one. Print the
first boundary state that disagrees with a plain characteristic ghost before theorising.

## Measuring renders against recordings: hound writes WAVE_FORMAT_EXTENSIBLE

**Symptom.** Every render looked like flat broadband noise next to real recordings (low
line-to-floor contrast, a treble that rose to 8 kHz) — a whole afternoon chased a "noise floor"
in the physics that did not exist.
**Cause.** `hound` writes 32-bit float WAVs with format tag 0xFFFE (extensible), not 3; a loader
that keys float on tag 3 reads the floats as int16 pairs: noise. The reference recordings
(ffmpeg, tag 3) read fine, so only our side was wrong.
**Fix.** `scripts/realism.py` treats 0xFFFE at 32 bits as float and has `--self-test` (a harmonic
series written as extensible float must read back as itself and as lines). Run it before
trusting any comparison. Free reference recordings (Freesound, CC-BY/CC0) live outside the repo
in `target/refs` — compare like with like: an under-bonnet or open-header recording is not a
road car's tailpipe.

## An idling engine's "background air" was the jet model's leftover filter state (2026-10-23)

**Symptom.** Every preset at idle had a steady hiss above 500 Hz, 17–30 dB over everything else
there; its band envelopes were as flat as white noise (p98/median ≈ 7.5–8.5 dB, `scripts/realism.py`
contrast), where real idles are pulse-modulated (10–19 dB).
**Cause.** At idle the tail flow swings through zero every pulse (median exit velocity −0.7 m/s).
`JetNoise::step` returned the band-pass's raw output when nothing blew out — its state is *unit*
white noise, not pascals (~0.05 Pa ≈ 67 dB), and it re-tuned the filter to 1 kHz on the way. By
the correlation a 20 m/s idle jet is ≈ 18 dB SPL: inaudible.
**Fix.** No outflow → 0 Pa; the filter runs down on silence. Ablate one source at a time at idle
and read the contrast column: a source that is "everywhere" at noise-level contrast is a suspect.

## A cut cycle's residual is air, not burnt gas (2026-10-25)
With fuel cut-offs that leave the cylinder pumping air, the gas left at EVC is unburnt air, and a
residual after a cut is ~3× a fired one's (no blowdown). `dilution_of` counted all of it as burnt
diluent (Metghalchi–Keck 1 − 2.1 f), so the first flames after every limiter cut took 200–330°,
finished after EVO and dumped fuel into the exhaust: a steady false afterburn at the limiter.
**Fix.** Dilution = residual less its unburnt fuel and air (`residual_unburnt_kg`). Probe the
charge at IVC (trapped, residual, fresh air, fuel, carried slip) before trusting any new afterburn.

## Separate cargo workspaces sharing one target dir overwrite each other's binary (2026-10-25)
Two worktrees built with the same `CARGO_TARGET_DIR` uplift to the same `release/engine-sim`;
a build that cargo finds fresh does not re-uplift, so both "builds" can hand you one binary.
**Fix.** A/B binaries get their own target dir (`target/tgt-<name>`); check `md5sum` before
rendering comparisons.

## A worktree's `../../scripts/...` is the main checkout's script (2026-10-26)

Worktrees live under `target/wt-*`, so from one of them `../../scripts/mutants.sh` resolves to the
**main checkout's** `scripts/`. `mutants.sh` starts with `cd "$(dirname "$0")/.."`, so it then
mutates the main checkout's tree, not the worktree you meant. `heavy.sh` is only a wrapper and
does no harm. **Fix.** From a worktree, call its own `scripts/…`.

## FV overrun: two numerical sources, and the open question left (2026-10-25)

**Symptom.** Held at constant speed with the fuel cut and every random source off, a fully
deterministic engine, the FV exhaust's sound did not repeat from cycle to cycle. Cycle-to-cycle
correlation was 0.49 on the inline-4 and 0.81 on the LFA. It also came out 9–10 dB over full
power.

**Measured stage by stage** (LFA and inline-4 at 0.85 × redline):
- Valve flow, cylinder pressure and primary pressures repeat exactly.
- On overrun the tail's outlet flow reverses 120–300 times in half a second; at WOT it never does.
- At 192 kHz physics the output broke even at WOT (0.80). Below 5 kHz it repeated.

**Causes.**
- **The open end's inflow.** It imposed the atmosphere's *static* pressure and density on gas
  entering the pipe. The ghost jumped at every reversal: hot gas at p₀ one step, cold air at p₀
  the next. Fix: `fv::open_end`. Along the characteristic from inside, p + Z·v is the interior's.
  Outflow takes p₀. Inflow is the atmosphere's Bernoulli, p = p₀ − ½ρ₀v². Pressure and velocity
  are continuous through a reversal; only the density jumps (a contact surface).
- **A staircase.** FV held its radiated value between pipe steps (7 physics steps at 192 kHz).
  The listener's radiation filter (∝ s) turned every stair edge into a spike at the pipe rate.
  Fix: the value is interpolated on the line between the last two pipe steps, one pipe step
  late.

**Tests:** `a_motored_fv_exhaust_repeats_every_cycle`, `the_radiated_signal_moves_between_pipe_steps`,
`an_open_end_draws_the_atmosphere_in_by_bernoulli`, `an_open_end_passes_outflow_at_the_outside_pressure`.

**Still open.** Periodic now, FV still radiates about 9 dB more on overrun than at full power in
the audible band. It has the same pulse at the valve in both states: the overrun EVO back-flow
into a 0.3 bar cylinder is as strong as the WOT blowdown. Exhaust noise is generally quoted as
≈ 10 dB *below* full load at no load, a generic figure. No measured overrun-vs-WOT tail data is in
hand. Suspects, untested: the open end without the jet's vortex-shedding loss at high mean
outflow (|R| = (1 − M)/(1 + M), which the waveguide has), and no shell or muffler damping at low
mean flow.

## The FV exhaust had its own geometry and no acoustic wall loss (2026-10-26)

**Symptom.** In offline Reference renders (FV exhaust), idle and the tail after a lift-off came
out 8–12 dB louder than the waveguide's. The idle-to-WOT range was 15 dB against the
waveguide's 23 dB; at a car's tailpipe it is typically 25–30 dB.

**Measured.**
- Jets: no part of the gap.
- Valve flows: the same in both models.
- Propagation: with one exhaust geometry and a small valve flow (the linear regime, where the two
  must agree), the default system matched.

**Causes.**
- **Its own geometry.** FV built its own can and tail from the old fields, ignoring the designed
  path, catalysts included. The catalysts alone explain half of the inline-4's gap.
- **No wall loss on the waves.** FV's only wall friction is Darcy's u|u|, which vanishes for small
  oscillations. Without a mean flow (idle, overrun) its resonances were nearly undamped. The
  waveguide's 0.03/m matches Kirchhoff's boundary-layer loss in a primary at the firing orders.

**Fix.**
- FV ends at the collector. A characteristic node (`duct_node`) hands on to the designed path's
  linear duct and open end.
- `Pipe1d::acoustic_loss` adds the same per-metre loss as a momentum sink on u − ū.
- Gaps now: inline-4 idle 3.9 dB, flat-plane V8 3.6 dB, LFA 8.9 dB (open).

**Also found.** Overrun as loud as WOT is not an FV defect. The waveguide does it too, exhaust
only: inline-4 overrun +11.0 dB against WOT +9.3 dB; flat-plane V8 +19.7 against +14.7. See
"Overrun ≈ full power at the tail" above.

**Tests:** `the_collector_end_passes_pressure_and_volume_flow_to_the_duct`,
`in_the_linear_regime_the_fv_net_sounds_like_the_waveguide`,
`the_acoustic_wall_loss_wears_a_wave_by_its_rate_per_metre`, `a_one_to_one_junction_is_transparent`.

## Reference renders with the FV exhaust stuttered at idle (2026-10-26)

**Symptom.** The user heard v0.22.0's offline renders cut in and out and stutter. The audio has no
dropouts (no silent or repeated blocks, no jumps). The engine itself hunted.

**Measured.** At idle with FV (384 kHz physics, held 3 s first), the K20A ranged 559–1118 rpm
(σ 179); the waveguide holds 815–920 (σ 22). The flat-plane V8 ranged 869–1181 (σ 91). There
were no misfires: firings matched the speed.

**Cause.**
- With FV the K20A makes less torque at idle and sits at ~650 rpm against an 800 target, with the
  spark at full advance. The exhaust's mean backpressure and scavenging differ between the
  models; not yet explained.
- The idle air integral was pinned at its floor (bypass 95 % closed). With the waveguide that
  engine has torque to spare at idle (spark retarded the full 20°), and climbing out of that floor
  takes tens of seconds.

**Fix / workaround.** Reference no longer switches to FV. FV stays a choice in the editor, to
come back once it holds an idle. **Open:** FV's idle torque deficit, and the idle air integral's
slow recovery from its floor.

## Per-step helper threads collapse when cores are shared (2026-10-26)

The exhaust net and the ITB runners on their own threads trade messages every physics step
(spin, then `sched_yield`). On an idle desktop that saved ~24 % (v0.16.1). On cores shared with
anything else the partner thread loses its slot and every step waits: LFA Medium, 4 s of sound
on six shared cores took 19 s threaded vs 4.2 s on one thread (and 34 CPU-s vs 3.8). A 15 W
laptop is always in the shared case (UI, audio, browser, one power budget), which is where the
user heard underruns. They are now opt-in (`--threads`, `exhaust_thread::STEP_THREADS`). Any
future parallelism must hand off per block of steps, never per step.

## A longer cam lost torque until the cam phaser was calibrated (2026-10-26)

**Symptom.** The K20A preset with its VTEC lobes (intake and exhaust) and a guessed VTC map made
*less* torque above 6000 rpm than the old single lobe: 163/169/140 N·m at 6000/7000/8000 rpm
against 194/183/148.

**Measured, one lobe at a time** (dyno, phaser off): the exhaust VTEC lobe alone lost at every
speed (177/167/139). The intake VTEC lobe alone lost at 6000 (178) and only won at 8000 (157). Its
extra lift does little: both lobes reach the port limit (L/D 0.25). Its later closing does the
damage until the phaser moves it.

**What works.** Sweep constant VTC advances (0–50° in 10° steps) for each lobe and take each
speed's best. Then the VTEC intake lobe beats the low lobe from ~6000 rpm (195 vs 194 N·m), and by
10 N·m at 8000. That is the crossover Honda switches at. A longer cam only pays with its timing
optimised, as the real ECU's map does (Niizato & Hayashi 2000, §6.1). A guessed map hides it. To
redo it: write the engine with `--save`, set every `full_load_deg` to one value, and run `--dyno`
once per value.

**Still open.** Above 7000 rpm this model breathes less than the K20A on any cam (157 N·m at 8000
against Honda's ~193). That is a general high-speed breathing deficit, not VVT.

## The torque was right for the wrong reasons: friction ×2, cold-air cycle (2026-10-27)

**Symptom.** Above ~7000 rpm the K20A made far less than Honda's figures (158 N·m at 8000 rpm
against ~193). Its friction was 5.4 bar there.

**Two errors that hid each other.**

- *Friction.* `fmep_bar` was Barnes-Moss's whole-engine fit, 0.97 + 0.15·N + 0.05·N² bar
  (N in krpm; Heywood). That is *motored* friction, so it already holds pumping, which the model
  also simulates in the p–V loop. Its N² term was fit on 1970s engines below ~6000 rpm.
- *Cycle.* With γ = 1.4 for the cylinder's gas, the cycle was an air-standard Otto cycle. Net
  indicated efficiency came out at 45–48 % at WOT; real engines run 35–40 % (Heywood ch. 5).
- *Masking.* The doubled friction had been eating the excess work.

**Fixes.**

- Friction by components: Patton–Nitschke–Heywood as improved by Sandoval (`friction.rs`; MIT
  thesis Appendix A-2; it reproduces Fig. 3-3's components within 1 kPa).
- Real gas properties: NASA polynomials for air, the stoichiometric charge and frozen products
  (`thermo.rs`; Heywood Tables 4.7, 4.10, 4.11; Fig. 4.13's γ within 0.005).
- Result: η_net 0.36–0.41. Peak torque within ±5 % of the published value for the K20A, the LFA
  and the LS3, with no per-engine number. The equations are images in the PDFs: read the
  rendered pages, not `pdftotext`.

## A reversion of air labelled as burned gas made the valve chatter (2026-10-27)

**Symptom.** With real gas properties the FV exhaust valve flipped direction in 1–5-step bursts
on overrun: 43 reversals in 7 cycles; the bursts reached 3.6 % of the peak flow.

**Cause.** Reversion gas came back labelled as products (x_b = 1). On a fuel cut the primary
holds air. Each reversion stepped the gas constant and the internal-energy curve, and with them
the pressure.

**Fix.** The primary's gas carries its composition (`port_xb`, set by what leaves the cylinder,
like `port_t`). The fresh charge carries fuel only when the ECU injects (`Composition`). With
that, overrun reverses as it did before the change.

## High-speed power is lost in breathing, not at the valve's port model (2026-10-27)

**Symptom.** At 8000–8500 rpm, WOT, pumping reaches 1.7–1.8 bar (0.3–0.5 at mid speed). The
K20A's torque at 8000 rpm is 160 N·m against Honda's ≈ 193. At bottom dead centre, 8000 rpm, the
cylinder is still at +3.5 bar: the blowdown has not finished when the piston starts to push.

**What it is not.**

- *The linear valve port* (p = p₀ + 2p⁻ + (c/A)·ṁ). Benson's characteristic boundary
  (`feat/benson-valve`) puts a 3-bar, 1300 K blowdown into a quiet pipe at 2.38 bar where the
  linear port says 2.48: 4 %. A first diagnosis blamed the port; the decomposition behind it
  mixed the waveguide's volume-equivalent source with the valve's mass flow.
- *The exhaust bores.* A 65 mm tail adds 5 N·m at 8000 rpm; no catalyst, 2 N·m; all open, 6 N·m.
  An earlier ablation changed `tail_diameter_m`, which `after_collector` overrides, so it
  measured nothing.

**What moves it** (K20A, 8000 rpm, one change at a time):

| Change | Torque at 8000 rpm |
|---|---|
| none | 160 N·m |
| exhaust opening 20° earlier | 168 N·m |
| exhaust valve 50 mm equivalent | 167 N·m |
| intake valve 58 mm equivalent | 177 N·m |

The valve events pass too little gas at speed. The lift curve is a sin² over the duration: its
time-area is lower than a real cam's, which dwells near full lift. And the presets cap lift at
L/D 0.25, where the K20A runs ≈ 0.33. Next: a cam profile from a cam-design source. The LFA's
torque peak may sit at 5500 rather than 6800 rpm until then (the dyno test allows it).

## Real gas properties cost ≈ 10 % of a step (2026-10-27)

**Measured** under `scripts/cpu-lock.sh`, 4 s headless renders: inline4 1.79 → 2.00 s, single
0.88 → 0.94 s, LFA 5.61 → 6.12 s.

**What helps.**

- One mixture per cylinder, recomputed only when its composition changes.
- One fused c_v/u evaluation.
- A Newton stop at 0.1 % of T: quadratic convergence leaves under 0.01 K.
- Three settles a step.
- The friction's speed terms cached per 1 rpm.

**What does not.** `temperature()` alone is ≈ 12 ns a call (microbenchmark). `perf record` hangs
on this machine; use valgrind when it is installed.

## Profiling without perf or valgrind: eu-stack plus a preloaded PR_SET_PTRACER (2026-10-27)

`perf record` hangs on this machine and valgrind is not installed. `eu-stack -p` fails with
"Operation not permitted": Yama's `ptrace_scope = 1` lets a process trace only its descendants.
gdb as the parent cannot sample from Python either: the stop is never processed while the
script sleeps.

**What works.** A three-line shared library whose constructor calls `prctl(PR_SET_PTRACER,
PR_SET_PTRACER_ANY)`, preloaded into the binary. Then `eu-stack -s` can sample it.
`scripts/profile.sh [preset] [samples]` does it all: line-table build, preload, about 300
samples, a count by source line.

**Limit.** eu-stack does not show inlined frames: most samples land on the caller of
`Engine::step`. Its innermost-frame counts still compare two builds fairly.

## Overrun is a perfect tone: ~100 % of it repeats cycle to cycle (2026-10-26)

Share of each band that repeats from cycle to cycle (`periodic2.py`-style synchronous average over
4 cycles, tracked cycle by cycle): the LFA lifting off at 5000–6000 rpm, model v0.23.0, repeats
1.00 / 1.00 / 0.99 / 0.91 at 250 Hz / 500 Hz / 1 kHz / 2 kHz (4 kHz 0.64–0.78). Real LFA lift-offs
at ~4900–5400 rpm (recording yY0Fx8Thyx4, 0.8 s after the lift): 0.11–0.24 / 0.13–0.39 /
0.00–0.05 / ~0. That is the user's overrun "fly": with no combustion the pumping is identical
every cycle. Caveat: the real clips are from a moving car (road and wind noise are aperiodic too).
Candidates checked and why they do not supply it:
- **Throttle-gap jet:** the gap is choked on overrun, so its noise cannot travel upstream to the
  trumpet (sound does not cross a sonic throat against the flow).
- **Reverse valve jets:** `valve_jet_level` is silent when the port is above the cylinder. At EVO
  on overrun the backflow into a 0.13 bar cylinder is choked too: its noise is in the cylinder,
  reaching us only through the block, whose model takes the 0D cylinder pressure (no turbulence).
Open: what aperiodic source a real overrun has at 250 Hz–2 kHz (cycle-to-cycle gas exchange,
turbulence in the cylinder through the structure, pipe-flow noise).

## FV exhaust at idle, after the designed path (2026-10-26)

Measured with `ExhaustModel::Fv`, 3 s settled then 3 s held at idle (core, 192 kHz): the **LFA is
now steady** (900–966 rpm vs the waveguide's 905–982); the **K20A still hunts** (593–1065 vs
807–904). Not misfires (firings match the speed). Not the idle loop either: with the loop frozen
(pedal a hair open) the waveguide K20A settles near 860 rpm while the FV one drifts 1091 → 592 →
1062 on its own. Per cycle the FV engine scavenges better (residual 7–12 % vs 17–19 %) and does
about the same work for the same fresh charge; its residual follows the speed over ~1.5 s, a lag
in a quantity that slows the flame. A faster idle integral makes it worse (KI 2 → stalls both
engines), so the cure is not in the controller. Torque at a held speed, fixed idle air: FV 4–7
N·m below the waveguide from 550 to 1150 rpm, both falling with speed. Open: what in the FV
coupling removes the open-loop damping.
- **All presets, FV vs waveguide idle (3 s held after 3 s settled, rpm range):** single 654–1371
  vs 765–1291; inline-4 756–945 vs 768–872; K20A 593–1065 vs 807–904; flat-plane V8 859–1142 vs
  985–1040; cross-plane V8 580–760 vs 690–766; LFA 900–966 vs 905–982. FV idles worse on five
  of six. Per cycle (K20A, 800 rpm held, same idle air) pumping work is identical (−31 J) and the
  exhaust-stroke pressure the same; FV's gross work is 8 % lower (152.6 vs 166.5 J) as it traps
  ~5 % less fresh charge. Freezing the residual's effect on the flame halves the open-loop swing;
  retuning the idle spark (reserve 10–15°, KD 10–60) does not help (more KD is worse).

## Benson's valve boundary: three ways to NaN, and a closed form for C (2026-10-27)

**Symptom.** The characteristic boundary (`valve_boundary`) passed its unit tests and then put NaN
into 46 engine tests.

**Causes**, each with its regression test:

1. *The sin² lift starts from nothing.* At ψ ≈ 1e-26 the choked root sits within round-off of
   π_α, where (6.51) divides by zero. Below ψ = 1e-9 (under 1e-10 kg/s) the valve is closed.
2. *A small-ψ lower bound for the sonic bracket* overshot the root at ψ ≈ 0.5. Both ends had the
   same sign and regula falsi divided 0/0. The bound now holds for any ψ, and `root` returns the
   nearer end when there is no sign change.
3. *Eq. (6.47)'s second term has a pole* at B = √((κ−1)/2), which the bracket straddled.

**The closed form.** Benson finds C by successive substitution (6.61), solving (6.47) or (6.51)
each time. It has a closed form in π: C = 1/√(π² + d²/KM), with d = π − π_α. That leaves one
equation with no pole, since 1 − B²/KM = C²π².

**Cost.** A solve cost 322 ns cold. Closed form, a step-size convergence test and a secant from
each cylinder's last root (`valve_hint`) bring it to 71 ns warm. 4 s renders: +80 % → +5–13 %
over the linear port.

**Effect.** Torque within 1 % of the linear port's (K20A, LFA, LS3): the boundary is more correct,
not more powerful.

## A WAV reader keyed on format tag 3 misreads the app's float files (2026-10-28)

**Symptom (2026-10-28).** Third-octave distances between quality levels came out 0.2–0.6 dB and
justified a release (v0.26.7) that was then withdrawn: read correctly, the levels were 0.4–6.3 dB
apart.

**Cause.** `hound` writes 32-bit float as format tag 0xFFFE (extensible) whose subformat GUID
starts with 3; a reader that only checks `tag == 3` falls through to an integer branch and reads
the float bits as int16 or int32. A deterministic per-sample map, so the result still "looks"
like a spectrum and periodicity survives — nothing flags it.

**Fix.** Read the subformat's first two bytes when the tag is 0xFFFE (job script `wavio.py`), and
check a reader's min/max/RMS on a known render (|x| ≤ 1) before trusting any number from it.

## Steepening with the LFA's muffler: WOT right, lift-off still too loud (2026-10-29, branch feat/wave-steepening)

Steepening (both directions, 0.2 m segments) on top of the as-built LFA exhaust (bypass muffler,
engine sources ahead), `--render drive`, levels within one file (offline renders are normalized
per file, so only in-file ratios compare): WOT 6.5–7.4 s, 2–8 kHz re total −8.9 dB (linear
−24.6; real LFA blips −6…−8.5). Lift-off 7.6–8.5 s re WOT: −3.5 dB (linear −15.5; real LFA
−8…−19). The muffler did not cure the overrun shock: at 7000+ rpm its bypass valve is open (rpm
rule), so the shock takes the short way. Whether the real valve also closes on overrun (a vacuum
actuator usually answers to load too) is not published — asked the user (PAPERS.md).

Lead (coast_chain, LFA 6300 rpm held, overrun vs WOT, sound layers off = valve source → tail only):
the valve's volume-flow source falls 14.8 dB but the radiated tail only 3.7 dB even with LINEAR
pipes (orders 9–11 of the tail rise +2…+12 dB while the source's fall). With the layers on, WOT is
carried by the intake and flow noise, which hides it (−14.3 dB). So the exhaust's own overrun
excess is not steepening's: the pipes ring more when the mean flow stops (the open end's jet
reflection (1−M)/(1+M) and the friction α = f·M/D both vanish at M → 0) and the valve flow reverses
~4×/cycle with sharp edges. Steepening then lifts those edges' harmonics. Next: check the reverse
flow's sharpness at the valve (Benson boundary on inflow) and the pipes' damping at zero mean flow
(heat transfer, Kirchhoff walls on the linear pipes too) before touching steepening.

## "Sounds far": the linear waveguide's pulses had no shock front, so 2–8 kHz was 15–24 dB short (2026-10-29)

**Symptom.** The LFA's gear pull (`--render gears`) "suena muy lejos" (user). Held 6700 rpm WOT
against the chassis-dyno clip (`ruler.py`, octave deltas model − real in dB re total, 63 Hz…8 kHz):
+8.9 +8.0 +2.9 +3.7 −1.8 −14.4 −23.4 −21.6, score 12.9 dB (at 91312d5, this branch's base — not the
8.9 quoted from another branch); spectral tilt 250 Hz–8 kHz −2.2 dB/oct against the real clip's
+2.1; 2–8 kHz crest factor 18.8 dB (sparse clicks at the firing rate) against 11.5.
**Not the cause (measured one variable at a time on the same held render).** Listener geometry:
ground off 13.8 dB; on the dyno camera's spot (2.5 m, on axis, 0.5 m high, ground) 17.2; 1 m on
axis in free field 15.4 — none moved 2–8 kHz by more than 1.5 dB. Muffler chambers removed: 4 k /
8 k +5 / +10 dB, still −19. Catalyst removed: +7 / +12 dB (its viscothermal loss, Dokumaci), still
−16. The FV exhaust (`model: fv`, 4 cm cells = 7.5 per wavelength at 2 kHz) is worse: 4 k −33, 8 k
−36 (numerical dissipation of the shock it does form). Exhaust pulses alone (every `sound.*`
texture at 0): 2–8 kHz at −25 / −43 / −55 re total — the exhaust delivered nothing above 2 kHz;
what little there was came from the block and the valve jets.
**Cause.** The waveguide was linear. A 0.3–0.5 bar blowdown pulse in a 900 K primary shocks within
0.2–0.8 m (x_s = ρc³/(βωP), Hamilton & Blackstock, *Nonlinear Acoustics*, ch. 4), and a shock
front's harmonics fall at −6 dB/oct where a smooth pulse's fall at −12 or faster: the real H3…H8
sit at −7.6…−22 dB re H1, the model's at −19…−49. A dull, bass-heavy spectrum is heard as
distance (Zahorik, Brungart & Bronkhorst, *Acta Acustica united with Acustica* 91, 2005: spectral
tilt and high-frequency loss are distance cues; reverberation is the other, and there is none here).
**Fix.** Merged branch `feat/wave-steepening` (simple-wave delay of Msallam et al. 2000, Burgers'
entropy solution for the shocks, Kirchhoff walls; spec
`docs/superpowers/specs/2026-10-26-wave-steepening.md`). Held 6700 after: deltas +5.4 +5.1 +2.6 +0.6
−5.7 +1.6 −4.5 −5.2, score 7.6 dB (bands 13.2 → 4.2); tilt +1.9 dB/oct; 2–8 kHz crest 10.7 dB.
Gear pull at 20–22 s: 2 / 4 / 8 kHz re total −14 / −20 / −33 → −5 / −9 / −14, tilt −0.9 → +4.1
dB/oct. The packed silencers' perforated tubes stay linear (their loss is the packing's, by the 2D
model), the smooth pipes steepen.
**Why it isn't obvious.** "Finite-amplitude waves" was already on main (1089284: waves
*superpose* as finite-amplitude waves) and read as "steepening done"; it is not. And an offline
render is normalized per file, so the missing highs only show as band ratios, never as a level.
**What it cost.** Three of the branch's tests had never been green (round trip 441 ≠ 447, the LFA
refusing to detach, textures < 1.3× over a shocked exhaust) and two newer tests sent loud pulses
(0.2 kg/s, which steepen and arrive early). Arrivals are now timed at the half-rise of the first
lobe of a faint pulse (hand values 448 / 1038 / 68 steps met within a step); the detach test runs
the LFA linear and asserts that a steepening net is never handed to the exhaust thread (by design:
its waves arrive when their own speed says, not a fixed block later) — so a steepening engine
keeps its exhaust on the sim thread, see FACTS for the real-time cost.
**Open.** H3 (1.7 kHz) −19 dB before and after: the notch of the tips' length difference
(c/2ΔL ≈ 1.8 kHz) in the muffler's estimated geometry, not the listener. 63–125 Hz +5 dB: 4.7 dB
of it is the rigid ground's image (physics; free field +4.2 / +3.8), the rest cycle orders 1–3;
the clip is a phone on the dyno deck (video frame at 8 s: vertical, on axis, ~2.5 m behind, ~0.5 m
high, block-walled room) whose low-end response is unknown, so this band cannot be settled from it.

Found: 2026-10-29 · physics

## The LFA's bank order dominates below ~3000 rpm at full load: the 10 cm asymmetry through lossless chambers (2026-10-29)

**Symptom.** `sim::tests::firing_orders_dominate_and_output_is_not_saturated` (LFA held 3000 rpm
WOT, Ultra) went red when steepening was merged: cycle order 5 at 0.199 against the firing order
10 at 0.139. The real car at 4110 rpm has order 5 at −15 dB re the firing order (`exhaust.wav`
2.0–5.5 s).
**It predates steepening.** Held renders (Reference, `ruler.py … RPM`, order 5 re order 10):
2800 rpm linear +11.2 / steepening +10.6; 3000 rpm +5.4 / +8.5; 3200 rpm +0.7 / −1.5; with equal
bank lines (`collector_extra_length_m` all 0) at 3000: −30. The test passed on the base only at
its own point (3000 rpm, Ultra) and by a margin steepening's +3 dB there removes; at 2800 rpm or at
Reference it was already false.
**Mechanism (measured, not fixed).** The bank order survives the banks' meeting only by the 10 cm
difference of the lines (0f38888), and below 3000 rpm the muffler valve is closed, so the sum goes
round the end chambers: empty, lossless 1D chambers of 0.12–0.2 m on 50 mm pipes of 0.3–0.55 m,
Helmholtz-like at 70–120 Hz — exactly order 5 at 1700–2900 rpm — ringing with no packing, no
wall loss and no flow loss. Above 3000 rpm the valve's straight path has none of that and order 5
falls to where the real car keeps it (6700 rpm: −11 against −14.5 real). The test is left red on
purpose rather than weakened: the physics to fix is the chambered path's damping (and whether 10
cm is the right bank asymmetry), not the assertion.

Found: 2026-10-29 · physics

## The LFA's firing order at 6700 rpm WOT was the trumpets', not the exhaust's (2026-10-29)

**Symptom.** Giving the LFA its surge tank (`AirboxDef::lfa_v10_estimate`: 20 L, two Ø90 mm
ducts of 0.40 m, the trumpets drawing from the box and the induction sound radiating from the
duct mouths) sent `the_lfa_at_full_throttle_keeps_the_real_cars_orders` from 11.7 to 20.0 dB RMS,
every order *above* the real car's. Peak-picking the held 6700 rpm render: before, 558 Hz (the
firing order) was the top line; after, it is not in the top ten (2511 Hz = order 45 leads, order 5
at −9 dB). The ruler's f0 tracker then locks on 7538 rpm — read its rpm line before its score.
**Cause.** The runners' half-wave (c/2L ≈ 520 Hz) coincides with the firing frequency at 6700 rpm,
and the ten trumpets open to the air radiated it: the whole firing order of the render was the
intake's. Behind a 20 L box the duct mouths pass 1/|cos kL' − (kV/A_t) sin kL'| ≈ 0.06 (−24 dB)
of it, as a Helmholtz filter must, and the exhaust alone has no firing order at that speed.
**What it says.** The exhaust model's missing line at the firing order at high rpm was masked by
the intake, and the real car's remaining intake path is the one Yamaha built: the tank's walls,
tuned to radiate 250–500 Hz (Yamaha, 2009 release). Until that path exists, the preset keeps the
trumpets in the open air (`lfa_v10`), and the tank is an estimate ready to wire in. Numbers:
`docs/FACTS.md` (surge tank, 2026-10-29).
**Why it isn't obvious.** Every band and order test passed with the trumpets masking it; the
dyno barely moves (−2 % at 6000–8000 rpm, one dynamic head lost at the duct's exit into the box).

Found: 2026-10-29 · physics

## The exhaust net's transfer swings ±20 dB between neighbouring orders; the firing order at 6000–7400 rpm sits in a trough (2026-10-29)

**Symptom.** With the surge tank in the LFA preset (the trumpets no longer open to the air) the
held 6700 rpm WOT render loses the firing order: order 10 (558 Hz) −4.2 → −24.0 dB re total, every
other line within 1 dB; guard `the_lfa_at_full_throttle_keeps_the_real_cars_orders` 20.0 dB RMS
(≤ 12 required), ruler 7.6 → 24.9. The real car on the dyno has H1 as its top line (FACTS).
**Cause (measured, not fixed).** The exhaust net's transfer from the valves' flow to the radiated
tails, by cycle order (`engine::probe::probe_lfa_orders`): at 6700 rpm 65 dB at the firing order
against 74–96 at the others; at 6000 rpm order 20 (1 kHz) is at 52 against 105 for order 25; at
4800 order 15 (600 Hz) at 51 against 66–83. A fixed trough near 560–600 Hz and 1 kHz, i.e. a
nearly lossless 1D line ringing: the merge + bypass muffler takes 38 dB at 558 Hz (12–38 at the
other orders) where the patent measures ≈ 10–11 per order with the valve open (JP 2008-169751 A
Fig. 7A); the bare line (no merge, no catalyst) already has the firing order 13–16 dB under its
neighbours, and goes flat only with 0.5 m primaries. The source is right: both banks' valve flow
has H1 on top (H2 −15, order 5 −19). Blocking the chamber path (baffle 0.01), removing the final
chamber or equalising the tips does not bring H1 back at WOT amplitude (FACTS); only removing the
merge (+9 dB) or the whole line's tuning does. The real line is flat to ±5 dB across the orders
(Fig. 7A dashed: H2/H4/H5 −2/+2/−3 re H1 at the muffler inlet; Fig. 8 radiated +3/+9/+2) — what
damps it (temperature gradient along the pipes, losses at every bend, cone and area change with a
M ≈ 0.2 mean flow, the sub muffler, two catalysts per bank) the net does not have; its junctions
are lossless and its pipes lose 0.03 Np/m.
**What was ruled out (numbers in FACTS).** Any surge-tank geometry: the ducts pass −17…−26 dB at
558 Hz (−42 with the air-cleaner case), the "400 Hz" of Okamoto's Fig. 18 being the duct's own
half-wave; only a 0.25 m duct (half-wave at 555 Hz, half the drawing's length) passes it.
**Status.** The preset keeps the trumpets open to the air; the tank stays an estimate ready to
wire in. The physics to fix before it goes in: the exhaust line's damping (flow losses at area
changes and bends, Munjal ch. 2 / Idelchik, c(T) along the line) and the bank stub that rings at
2.5 kHz (order 45 the top line of every render at 6700) — validated against the patent's Fig. 7A/8
orders at 4800 rpm, not by ear.
**Why it isn't obvious.** Every band and order test passed because the open trumpets, by the
runners' half-wave coinciding with the firing frequency at 6700 rpm, put the firing order back
in the air; the exhaust had been without it all along.

Found: 2026-10-29 · physics

## The gear-change thump was the torque cut blanking single charges out (2026-10-30)

**Symptom.** Every upshift of the gears render had a dull LF thump: 20–200 Hz 8–13 dB over the steady
mid/high bands for ≈ 80 ms, once as the cut ramped in and once as it ramped out (peak 117–234 Hz).
**Cause (measured).** The ECU took a cut past the spark's reach (31 %) by blanking out charges,
an accumulator spreading them evenly: as the share swept 0 → 1 in 70 ms the skip pattern swept a
tone of f_firing × share through 0–333 Hz. The exhaust net carries the sub-firing orders far better
than the firing order (FINDINGS above, 2026-10-29): held at 6000 rpm with a quarter of the charges
blanked out, the 125 Hz line came out 7 dB *over* the whole engine's firing order (an ideal train
gives −12 dB); half blanked out kills one bank (they alternate) and its 250 Hz line is 9 dB over.
Spark retard alone left no thump, but with the pedal down the clutch-open engine flared into the
limiter, whose cut-off made the same thump. Ruled out: nonlinear quasi-steady losses at the duct
area changes, a second catalyst per bank, the Rao–Munjal grazing-flow resistance of the baffle
(each moved the orders < 1.5 dB with the fitted geometry).
**Fix.** The ECU's two channels (Bosch, *Gasoline Engine Management*, torque structure): the air
channel trims the electronic throttle on a charge loop to the charge whose most retarded burn
gives the cut's share; the crank-synchronous channel retards the spark on the charge it has. A
full cut is a zero-torque request — closed throttle at speed — which the overrun cut-off answers by
cutting every injection at once. No single charge is blanked out. Per shift the LF peak over the
steady level went 8.6/8.3/11.2/12.6/12.8 → 1.2/1.6/0.9/3.7/5.4 dB.

## The gear-change thump, second pass: the limiter, the muffler's 1D ripple and a cut that reached every cylinder at once (2026-10-30)

**Symptom.** After the torque cut moved to the air channel and the spark (entry above), the user
still heard a dry thump at every upshift of the LFA gears render (`ecu.wav`). The 20 ms band
metric that had passed was the wrong ruler: a 5 ms one finds a single pulse per shift, 10–60 Hz
+33…+35 dB and 60–200 Hz +21 dB over the 300 ms before it, a negative lobe of −0.4 FS (1 ms
mean) for 6 ms then a positive one — a step in the tail's mean flow radiated as dQ/dt. The real
LFA on a dyno (`real/dynogears.wav`, four upshifts found by their pitch drop) has its < 200 Hz
5 ms peak at −11…−15 dB *under* the steady full-band level (+7…+13 dB over the steady LF level);
the render had it +6…+9 dB *over* (+28…+31 over the steady LF).
**Causes (measured, per-step diagnostic through the first shift).**
1. The pulse starts the moment the engine crosses 9000 rpm: the Shift phase ramped the cut 0 → 1
   open-loop while the clutch opened in the same 0.1 s, the clutch's capacity (∝ clutch²) fell
   faster than the air channel took the torque (plate 10/s), and the engine flared 8760 → 9007 rpm
   into the limiter, whose all-cylinder injection cut silenced ten cylinders within 3 ms. Shifting
   at 93 % of the redline instead (no limiter) kept the thump: the overrun cut-off, reached when the
   air trim hit 0.01, did the same thing 20 ms later.
2. Neither layer ablation moved it (block, intake, flow noise, valve jet: identical pulse): it is
   the exhaust line's DC. The tails' outgoing wave sat at ρc·u ≈ 19 kPa (u ≈ 82 m/s hot) at WOT
   and fell to −3 kPa in 6 ms once the charges stopped burning (the waveguide's source counts a
   charge's volume at its own temperature, and a motored charge is half a burnt one's).
3. The line passes that step whole and buries the firing order: a linear tone sweep of the fitted
   LFA line (one cylinder's flow → radiated sum) gave 50–200 Hz at 55–61 dB and 400–950 Hz at
   15–31 dB; without the bypass muffler 43–66 dB flat. The 1D transfer-matrix of pipe → final
   chamber (Ø0.22 × 0.25 m) → three Ø60 outlets reproduces the ripple (−28…−7 dB), so the net
   was right for its geometry: a 1D expansion chamber is 0 dB at every half-wave and 20 dB between.
   The LFA's final chamber is a sphere precisely so that its loss is flat: JP 2008-169751 A, Fig. 6
   measures ≈ 15 dB at 100 Hz and 20–21 dB from 300 Hz to 1 kHz, against the flat chamber it is
   compared with swinging 10–33 dB. H1..H8 of `ecu.wav` at 7000–9000 rpm: 0/−11/−2/−7/−7/−18/
   −15/−10 dB; the real dyno run 0/−10/−12/−15/−11/−16/−16/−16.
**Fix.** (a) `Segment::Measured`: a silencer element carried as its published transmission loss
(graphic EQ on a straight bore, as the packed silencers' 2D transmission already is); the user's
fitted engine file takes Fig. 6 for its final chamber. The preset does **not**: with it the guard
`the_lfa_at_full_throttle_keeps_the_real_cars_orders` (held 6700 rpm, orders re the firing order
against the dyno clip) reads 14.1 dB RMS against its 12 dB ratchet — H1 up 15 dB, H3–H8 unmoved
at −24…−42 dB (real −8…−22), the half orders −14 dB off; extending Fig. 6 above 1 kHz with Fig. 7A's
shape (14/17/23/28 dB at 1.5/2.5/4/8 kHz) made it 16.6. The exhaust's highs are short by that
much; until they are found the preset keeps its 1D chamber and the ratchet its number. (b) The shift's torque cut is the slipping-opening speed loop of
Glielmo et al. 2006 §III.D (reference held at the request speed; 1 % band, ESTIMATE): the flare
stays at 8920–8990 rpm, under the limiter. (c) Port injection: a cut-off reaches a cylinder at its
next exhaust opening (the injection is made onto the closed valve during the exhaust stroke,
Bosch *Automotive Handbook*), so the cylinders stop one by one over a cycle (3.1 of 4 still burn),
not within a quarter cycle.
**Result (`thump/final.wav`, same engine file with the measured chamber).** H1..H8
0/−11/−11/−9/−14/−21/−22/−15 dB (real 0/−10/−12/−15/−11/−16/−16/−16): the firing order is the top
line again. The thump is **not gone**: the 5 ms LF peak is +9…+13 dB over the steady full-band
level (real −11…−15), +34 dB over the steady LF; the pulse is now a −0.18 FS plateau of ≈ 15 ms
(the tail DC falling 15 kPa → 0 over 20 ms) instead of −0.4 FS for 6 ms.
**Ruled out, with numbers.** Counting the source as mass flow alone (the gas's temperature riding
the path lines, Benson vol. 1 §5.1) took the LF peak to −3…+6 dB over steady and +24…+33 over the
steady LF — but it is wrong: at low Mach the column is incompressible, the outlet's volume flow
follows the inlet's at once, so a colder charge of the same mass *does* slow the tail (and it put
H2–H4 at −4/−5/−3, the orders the real car has at −10…−15). Reverted. A slow line temperature
(τ = 1 s) is the same error spread out. The injection latency alone changed the LF peak < 1 dB
(the plateau lengthens, its height stays: the tail's volume flow still falls by the burnt/motored
temperature ratio within a cycle).
**What is left (not done).** The thermal step itself is physical; what the real car does not have
is the full all-cylinder cut: a 0.2 s ASG shift asks the ECU for a torque *reduction* and Bosch's
fast path is spark retard and blanking individual cylinders, not the overrun cut-off's complete
stop with the throttle closing behind it — the air channel of the entry above is the wrong path
for a 0.2 s cut. And the muffler's total compliance (the shell, ≈ 70 L by Fig. 1) is 26 L in the
fitted file.

Found: 2026-10-30 · physics

### The two candidates after it, measured against the real car's shifts (2026-10-30, later)

Per-band 5 ms peaks at the cut-in, dB re the steady full-band level before the shift (real dyno
clip, four upshifts, in the same brackets): 10–60 Hz / 60–200 Hz / 200–800 Hz.
- As committed (measured final chamber, speed loop, injection latency): +8…+12 / −4…−9 / −7…0.
- Real: −11…−24 / −14…−17 / −7…0.
- (1) The muffler shell at the patent's ≈ 70 L (chambers 7/32/12 L, Ø0.30, short connections
  for the plates' holes and pipes 36) instead of the fitted 16.7 L + sphere: +10…+13 / −5…−18 /
  −5…+1 — no change on the thump, H2/H3 up to −7.7/−8.8 (real −10/−12). Not kept.
- (2) The torque request answered by 40° retard + Bosch's injection blank-out (even accumulator)
  with the throttle at the pedal, the air channel of c856392 removed: −3…+5 / +1…+3 / +2…+13.
  Ten dB less infrasound (the throttle stays open, only the burnt→motored volume step is left),
  but 8–10 dB more at 60–200 Hz and 5–13 at 200–800, the sub-firing beat of the skip pattern that
  c856392 had measured: louder exactly where the ear is, 15–20 dB over the real car. Not kept;
  the air channel stays. (1)+(2): the same as (2). Clips: `thump/shell.wav`, `blank.wav`,
  `blank-shell.wav`; `target/tgt-thump/gears-blanking-rejected.mp3`.
- What would meet the real numbers is a cut whose exhaust volume flow does not step within a
  cycle: neither a complete fuel stop (any path) nor a skip pattern does that. The real 0.2 s
  shift's torque trace (Glielmo Fig. 10c) is the next thing to match, with the exhaust mean
  flow's time constant measured on the real clip at a shift (its 10–60 Hz stays 20 dB under).

## The shift thump was the LFA's limiter at its red zone and an overrun cut on the ECU's own throttle (2026-10-30)

**Measured** (ECU timeline per step through the first upshift): the cut ramps in while the clutch
opens, the plate slews 0.1 s to close, the engine flares 8740 → 9046 rpm. The limiter sat at the
redline (9000) and cut every injection; before it, the overrun cut-off read the ECU's trimmed
throttle (the cut's air channel, c856392), not the pedal, and cut every injection as soon as a full
cut closed the plate. Either way ten cylinders stopped burning within a cycle and the tails' mean
flow stepped (39 → 2 kPa in 50 ms).
**Fix.** `shutoff_rpm` (the ECU's fuel shutoff, 0 = at the redline): the LFA's is 9500 over a 9000
red zone (Okamoto et al., SAE 2011-01-2024, Table 1). Overrun is the driver's lift (Bosch, torque
structure: blank-out "when the engine is overrunning"): the cut-off reads the pedal; a shift's full
cut closes the plate and keeps the charges burning, retarded. LF peak at the cut-in re the steady
level, `shiftlf.py` on the fitted LFA, upshifts: +11.5/+2.8/+5.8/+6.8 → +2.2/−7.8/−9.9/−15.9 dB
(real car on its dyno: −14…−17).

## The car's body as a screen makes the LFA heard from behind worse, not better (2026-10-30)

The LFA's 2nd firing harmonic (cycle order 20) is 6–12 dB over the recordings' through the gears
(−5 dB re H1 against −11/−17), and the block carries part of it (block off: −3 dB). The rear body
as one diffracting edge (ISO 9613-2 eq. (14), D_z = 10·lg(3 + 20z/λ); rear edge 1.0 m, engine
3.2 m ahead, ear 2.5 m behind: z ≈ 0.12 m, 6–15 dB from 125 Hz to 4 kHz) on the block *and* the
intake mouth moved order 20 by 1 dB and took H1 with it: the intake mouth carries much of H1, the
half orders rose −10.7 → −5.9 dB and H3/H4 rose to −3 dB re H1. Screening only the block (−10 dB)
gives order 20 −8 dB and leaves the rest. A real body leaks round its sides (z ≈ 0.31 m) and under
its open floor, which one top edge does not model; the remaining order-20 excess is the exhaust's
own (exhaust alone at 8000 rpm: H2 −11.5 dB, still over). Patch kept out of tree:
`body-screen.patch` (Listener::with_body_edge, VehicleDef::rear_height_m) in the session scratchpad.

## The dyno recordings' level between the LFA's firing orders is the dyno cell, not the engine (2026-10-30)

Fitting orders 1–40 to the two dyno clips left cycle orders 11–29 10–15 dB under the recordings
(median −33 against −19…−23 re H1), and no exhaust geometry moved them. In those clips the level
at the half orders between (o + ½) is only 1.5–3 dB under the integer orders (−25/−20 against
−23/−19): a broadband floor (rollers, tyres, the cell's reverb), not engine orders. The two road
clips (`accel`, `flight`) give −32/−29 at the orders and −36/−33 between, where the render sits
(−33/−39). Fit inter-harmonic orders against road clips only; from the dyno clips take the firing
harmonics alone. Tool: `offord.py` beside `ord40.py` in the session scratchpad.

## The cylinder's exhaust comes out hotter at light throttle than at full load (2026-10-31)

With `exhaust.gas_temperature` on, the inline-4 held at 3000 rpm gives a port outflow temperature
(enthalpy-weighted over the cycle) of 1275 K at 15 % throttle against 1214 K at full throttle;
counting outflow only gives the same order. Measured engines run hotter exhaust at more load at
a fixed speed. Measured on the inline-4 at 3000 rpm (throttle 0.1 / 0.15 / 0.3 / 1.0): spark
20.3 / 16.9 / 13.9 / 13.3° BTDC (not at `MAX_SPARK_ADVANCE_DEG`), cylinder gas at exhaust opening
1281 / 1336 / 1261 / 1252 K, port mean 1383 / 1292 / 1232 / 1225 K. So (1) the cylinder's gas at
opening hardly falls with load, where a real one falls by a few hundred kelvin: its heat loss or
residual at light load is the suspect; (2) at a 0.2–0.4 bar manifold the overlap pulls port gas
back into the cylinder, and a net mean (outflow less backflow) rose over the cylinder's: the
backflow's enthalpy is taken at the port model's `port_t` (≈ 500 K), far under the primary's gas,
so it took too little energy away (LFA idle: port 1657 K). Fixed: the port hands the outflow's
mean alone (LFA idle: port 1032 K, primaries 822 K; full load unchanged).

## With gas temperature on, the LFA refit does not beat the fixed-speed fit (2026-10-31)

Turning `exhaust.gas_temperature` on moves the approved LFA off its fit: the muffler and tail gas
sits at ~900–980 K instead of the 622 K the fixed 500 m/s implied, and the patent's Fig. 8 error
rises from ~2 to 21 dB. A full refit of the same seven unknowns (fit5: geometry + EVO/EVC, same
dyno-power penalty, same J) with the option on reached J 6.55 (harm 1.54, orders 8.09, patent
1.93, 485 hp) against 6.0 for the approved fixed-speed fit. The longer line (1.68 → 1.88 m) it
picks is what hotter gas asks for (c ∝ √T, same delays need longer pipes). The option stays off
for the LFA; its better H2–H4 harmonics (1.54 vs the fixed fit's) are paid for in the
inter-harmonic orders. Next lever if revisited: junction impedance from each pipe's ρc
(the spec's non-goal), which the fixed-speed fit never needed.
