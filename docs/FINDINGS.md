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
