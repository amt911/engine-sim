# FACTS — engine-sim

> This is the repo's **shared working memory between agents**. `AGENTS.md`
> § *Agent orchestration* makes it the file every agent reads in its dispatch and appends to when it
> finishes, so the next one doesn't re-derive what this one already checked.

## What goes here

A fact earns its place if **all** of these hold:

- It was **verified against the repo or the running system** — read in the code, seen in the view
  hierarchy, printed by the command. Not remembered, not inferred, not planned.
- A fresh agent starting cold would otherwise spend real time rediscovering it.
- It is a *fact*, not a decision: what the selector is, which fake exists, what that helper accepts.
  Decisions go in the spec or in `AGENTS.md`.

Not here: **gotchas that aren't deducible from the code** — those are `docs/FINDINGS.md`, and they
outlive the branch. What lives here *is* deducible from the code and merely expensive to look up
again. **This file is allowed to go stale and die with the branch**; nothing should depend on it
being complete, and nothing gets written *only* here.

## Format

Grouped by area, one line per fact, newest first inside its group. If a line needs a paragraph, it
is a finding, not a fact — move it.

```markdown
## <Area: selectors | fixtures & fakes | commands | contracts | env>

- `<name or symbol>` — <the fact, in one line>. *(verified: <how>, <YYYY-MM-DD>)*
```

`verified:` says **how you know** — `read <file>:<line>`, `ran <command>`, `maestro hierarchy`,
`curl`ed the endpoint. A line without it is a rumor.

---

## Commands that work here

_None yet._

## Drivetrain and dyno (feat/drivetrain)

- `engine_core::drivetrain::{VehicleDef, Vehicle, Load}` — `Vehicle::step(dt, omega, brake)` returns torque + reflected inertia; `Simulation` feeds them into `Controls::load_torque_nm` / `extra_inertia_kgm2` every physics substep. Vehicle presets are keyed by `Preset::vehicle()`, not stored in `EngineDef`, so `presets/*.yaml` are unchanged. *(verified: read crates/engine-app/src/{sim,controls}.rs)*
- Clutch torque is `engagement² · capacity` (linear stalled every launch in a 1300 kg car); engagement ramps 1.5 /s up, 10 /s down. *(verified: ran the launch test, engine stalled with the linear map)*
- A gear selected with the clutch up at idle stalls the engine (realistic): tests press the clutch in neutral, select the gear, then release. *(verified: sim.rs `driven()` helper)*
- Dyno (`--dyno`, release build, ~seconds): single peaks 49 N·m @2000 / 27 hp @5000; inline4 198 N·m @2500 / 112 hp @5000; LFA 516 N·m @4000 / 521 hp @8500 (published 480 N·m, 552 hp; real peak torque is at ~7000 rpm, the 0D model peaks too early). Sweep runs 1000 rpm .. below redline in 500 rpm steps. *(verified: ran `target/release/engine-sim --preset <p> --dyno`, 2026-10-21)*
- LFA top speed from ratios: 9000 rpm in 6th = 323.6 km/h vs 325 published. *(verified: hand calc + test)*
- Vehicle numbers marked ESTIMATE in `VehicleDef` doc comments (Cd·A, Crr, efficiency, clutch capacity, motorcycle and hatchback figures) were not checked against a source; LFA ratios/mass/final drive come from the task spec, tyre radius is computed from 305/30 ZR20.

## Idle spectrum vs real recordings (2026-10-23, inline-4 preset, Ultra, `scripts/realism.py`)
- Tilt dB re 250–500 Hz for bands 125/250/500/1k/2k/4k: sim road exhaust 16.9/0/−17.7/−22.6/−30.8/−31.5; no
  catalyst ≈ same; empty chambers (no packing) 9.7/0/5.4/6.5/9.8/2.3; no silencers/cat 16.2/0/−1.8/−2.2/−12.5/−28.5.
  Real idles (Volvo S60, Chrysler, drag car): 0.5–4 kHz between −3 and −15. The packed silencers' lumped lining takes
  20–40 dB off above 500 Hz; plane-wave theory misses the HF "beaming" past the packing above chamber cut-on
  (1.84c/πD ≈ 1.2–2.2 kHz for the 213/120 mm shells). Phone-mic recordings lose the 125–250 band; compare ≥ 500 Hz.
- Contrast (p98/median of the band envelope) ≈ 7.5 dB is stationary noise; real idles 10–19 dB above 500 Hz.
- Block radiation (structure response × cylinder pressure) adds mostly 250 Hz–1 kHz: the Wiebe pressure has a
  cliff above 1 kHz (CPL 184 → 144 dB from 1 to 2 kHz at 5000 rpm WOT).

## The packed silencer against a measured one (2026-10-23)
Selamet, Lee & Huff, JSV 262 (2003), Fig. 7: L = 257.2 mm, perforated tube d1 = 49 mm (8 %), shell d2 = 164.4 mm,
fibre R = 4896 rayl/m. Our lining (`Shunt::lining`, low-frequency limit of a rigid-backed layer, distributed along
the tube; plane waves) gives TL 3.7 / 10.3 / 20.0 / 32.7 / 47.2 / 62.7 dB at 0.5 / 1 / 1.5 / 2 / 2.5 / 3 kHz; measured
21 / 29 / 36 / 30 / 22 / 18. Too little below 1.5 kHz, up to 45 dB too much above (TL rising ~f² instead of the
measured dome). A thick layer is far from its low-frequency limit (t = 58 mm), and above the shell's cut-on the
measured TL falls (beaming) where any 1D model keeps rising. Check: `papers/silencers/lining-study/compare_lining.py`
(local). Plan: the silencer's two-port from a 2D axisymmetric mode-matching model (Selamet, Xu, Lee, JASA 117,
2005), realised in the time domain by a rational fit of its scattering matrix (vector fitting, Gustavsen &
Semlyen 1999) — a few biquads per silencer.
- 2D mode-matching prototype (`papers/silencers/lining-study/modal2d.py`, local; Selamet–Xu–Lee 2005 re-derived in e^{jωt},
  Nice's fibre fit, Sullivan–Crocker/Kirby perforate, fundamental seeded from the bulk-reacting effective medium
  kx² = (k0²A1/ρ0 + k̃²A2/ρ̃)/(A1/ρ0 + A2/ρ̃)) reproduces JSV 262 Figs. 6–7: empty zeros at 680/1350 Hz, 25.6 dB at
  1.75 kHz, 1–3 dB above 2.2 kHz; filled 20.0/29.8/35.4/22.7/16.7/15.7 dB at 0.5/1/1.5/2/2.5/3 kHz (measured
  21/29/36/30/22/18). For the inline-4's two road silencers (hot gas, 600 K) it still gives ~100 dB at 2.5 kHz with
  either 16 or 4.9 kPa·s/m² packing: the right silencer physics alone does not bring the idle's 0.5–4 kHz up to real
  recordings — that band must come mostly from sources we lack (valvetrain/mechanical radiation, flow noise
  regenerated after the silencers).
- `pkill -f <pattern>` kills the shell running it when the pattern is in that command line: use `pgrep -x <process-name>`.

## Real-time factors, waveguide against FV exhaust (2026-10-26, Ryzen 9 5900X, CPU lock held)

`runner::measure` at full throttle and 0.8 × redline, 48 kHz audio. Values are audio s per
compute s; the laptop target (5850U) is about 0.6–0.8× of these.

| Preset | WG Medium | WG High | WG Ultra | FV Medium | FV High | FV Ultra |
|---|---|---|---|---|---|---|
| Single 500 | 9.74 | 6.76 | 5.16 | 4.52 | 2.73 | 2.54 |
| Inline-4 2.0 | 4.06 | 3.05 | 3.03 | 2.19 | 1.18 | 1.02 |
| Honda K20A | 4.18 | 2.68 | 1.57 | 1.90 | 0.97 | 0.88 |
| Lexus LFA V10 | 1.91 | 1.22 | 1.03 | 0.88 | 0.44 | 0.36 |
| Flat-plane V8 | 2.75 | 2.05 | 1.56 | 1.30 | 0.69 | 0.58 |
| Cross-plane V8 | 3.16 | 2.11 | 1.56 | 1.01 | 0.43 | 0.36 |

FV costs 2–3× the waveguide: no real-time level affords it on a multi-cylinder engine. Only the
offline Reference level uses it.

## How much of each band repeats cycle to cycle (2026-10-26, LFA idle)

Synchronous average over 4–8 cycles, each cycle found by cross-correlation and resampled to the
median length (job script `periodic2.py`; a fixed-period fold reads ≈ 0 because idle rpm drifts).
Real LFA idle (recording wR8FwJVFXOQ, 6–14 s, 1155 rpm): 125 Hz 0.52, 250 Hz 0.50, 500 Hz 0.19,
1 kHz and up ≈ 0. Model v0.22.1: 0.99 / 0.98 / 0.90 / 0.70 (1 kHz) / 0.45 (2 kHz). With the
burn-variability branch: 0.97 / 0.79 / 0.72 / 0.05 / 0.09. Autocorrelation picks the wrong
period on a V10 unless the rpm search range is narrow (the sim idles at ≈ 1130 rpm in renders).

- **Real cam lift vs the model's sin²** (Heywood 2nd ed., Fig. 6.16c, digitized 2026-10-27; a
  high-speed 2.2 L four, 36 mm intake valve, 10.6 mm lift → L/D 0.29):
  - The curve is cos^1.16 over ±61° cam (RMS 1.3 %), or sin² over ±74.25° cam (2.9 %).
  - Its duration at 0.15 mm lift (SAE) is 264° crank, so a sin² matching it needs ≈ 1.125 × the
    0.15 mm duration.
  - Time above 90 % lift is about the same for both shapes (0.23–0.245 of the 0.15 mm duration).
    The lever on high-speed breathing is lift (presets cap L/D at 0.25; Heywood: max lift ≈ 12 %
    of the bore), not shape.

## Why it still sounds synthetic: real LFA against the render (2026-10-27, `--render held`)

- **Spectral subtraction** (job script `resta.py`; render resampled to the real cycle rate, level
  matched over 300–4000 Hz, over-subtraction α = 6, which leaves −15 dB of a recording against
  itself): at 7400 rpm full load the render is lines (orders 5, 10, 20, 30) over a floor 10–25 dB
  under the real one's.
  - Missing: 100–250 Hz by 10–14 dB, below the firing frequency (617 Hz). Real engines fill that
    range from cylinder-to-cylinder differences.
  - Missing: 2.5–8 kHz by 15–22 dB.
- **Three real LFA recordings** at 6900–8100 rpm (the 0.8 s clip, `lfa-k3t6V_oY-9Q` 20–23 s,
  `lfa-yY0Fx8Thyx4` 178.8–183 s) agree:
  - Level re the firing line: 2.5–5 kHz at −5 to −12 dB (render −23); 5–10 kHz at −12 to −15 dB
    (render −29).
  - The envelope of that noise is stationary: ≤ 1 % of its modulation at the firing (or half)
    frequency, where the render has 25 %. The missing source is steady broadband noise, not
    stronger pulses.
- **Ablation at 7400 rpm full load** (LFA, one `sound.*` source at 0; dB change per band
  80–160 / 160–315 / 315–630 / 630–1250 / 1250–2500 / 2500–5000 / 5000–10000 Hz):

  | Source off | 80–160 | 160–315 | 315–630 | 630–1250 | 1250–2500 | 2500–5000 | 5000–10000 |
  |---|---|---|---|---|---|---|---|
  | flow_noise (tail jets) | −0.1 | −0.1 | −0.1 | −0.1 | −0.1 | −0.2 | −0.3 |
  | intake | −0.4 | −0.4 | +2.4 | +0.7 | −1.2 | −2.1 | −0.6 |
  | valve_jet | 0 | 0 | 0 | 0 | 0 | −0.8 | −6.2 |
  | block | +0.5 | +0.4 | −3.9 | −11.0 | −11.0 | +0.1 | +0.4 |
  | mechanical | 0 | 0 | +0.1 | 0 | −0.2 | −0.7 | −0.1 |
  | combustion_jitter | −8.8 | +0.1 | +0.7 | +0.5 | −0.9 | 0 | +0.4 |

  The block owns 0.6–2.5 kHz. The tail jets are negligible at the tail's ~55 m/s, and nothing
  supplies steady broadband above 2.5 kHz. The candidate is in-duct flow-generated noise
  (Nelson–Morfey). Yasuda 2010 measures silencer flow noise rivalling the engine's at
  5000–6000 rpm. Papers requested in PAPERS.md.
- **VTEC is nearly inaudible in the model**: the K20A held at 6500 rpm full load, VTEC on minus
  forced off: ≤ 1.7 dB in any band, +0.1 dB overall; torque +3 % at 6500 and +10 % at 7500.
- **K20A2 cam lobe heights** (Acura RSX 02–03 service manual, `papers/`):
  - intake primary / mid (VTEC) / secondary: 32.791 / 35.534 / 32.678 mm;
  - exhaust mid / secondary: 34.768 / 32.661 mm (the primary's figure is illegible in the scan).
  - The exhaust's VTEC lobe is real: +2.1 mm of cam over its secondary, against +2.7 mm on the
    intake.
  - No durations or base circle are given.
- **Converting quoted cam timing to the model's sin²**: from Heywood Fig. 6.16c (digitized), the
  sin² of equal time-area and peak lift spans 149–150° cam, which is robust.
  - Its ratio to the duration quoted at 0.15 mm is 1.14–1.19. This depends on the 0.1 mm offset
    of the figure's axis, so the 1.125 above is the low end.
  - At 1 mm it is 1.27–1.37 and depends on the lobe's peak lift. Lobes are not lift-scaled copies
    of one shape, so a 1 mm timing alone does not fix the equivalent sin².
- **Performance dead ends, measured 2026-10-28 (LFA, 5 s headless, cores 0–5, median of 3)**:
  - SIMD of the ITB runners four at a time (branch `perf/itb-runners-simd`, bit-identical):
    −1.4 to −1.8 % for ~680 lines that duplicate the FV numerics. Not merged.
  - Intake FV step sized by the CFL of the actual flow instead of the worst case (0.8·4 cm/700 m/s):
    the step is bounded by the valve coupling, not by stability. Against the converged answer (an
    FV step every physics step, `--dyno`), main is within ±0.1 % at 4500–7500 rpm (+3.7 % at
    1500); twice-longer steps lose up to 9.7 % torque at high rpm. Making the port cell show what
    the valve already drew this FV step (needed so longer steps do not create mass) costs 5 % by
    itself: the cell is refilled from upstream during the step, which neither the stale cell nor
    the drawn-down one represents.
- **What the quality knobs change** (third-octave distance to Reference in dB, LFA held 1100 rpm
  idle / 4000 rpm 30 % / 7000 rpm WOT, 2026-10-28, read correctly — see FINDINGS on WAV readers):
  High (3 steps) 6.3 / 0.6 / 0.4; 2 steps with WENO-Z 5.7 / 1.2 / 0.9; Medium (2 steps, MUSCL)
  5.7 / 1.3 / 0.9; Ultra 1.9 / 0.3 / 0.2; Low 5.1 / 3.7 / 2.1. At 2 steps the pipe scheme and 2 vs
  4 cm packing segments change nothing measurable; the step count does. Idle is where every level
  below Ultra is far off (max 15–21 dB in one band).
- **Real engines at idle repeat almost nothing above 500 Hz** (share of each octave band that a
  synchronous average over 8 cycles keeps; job scripts `periodic2.py` + `s5/sync.py`, period search
  ±8 % around the known speed; recordings in `real_engine_sounds/`, local only):
  - real: Viper V10 race ext 1197 rpm 125/250/500/1k Hz 0.38/0.48/0.12/0.01 (again at 1120 rpm
    0.32/0.24/0.04/0.00); Camaro Z28 V8 intake mic 882 rpm 0.52/0.27/0.09/0.01; Audi RS4 flat V8
    engine-bay mic 800 rpm 0.78/0.46/0.22/0.02.
  - renders (v0.26.6 High, held at load 0): LFA 1150 rpm 0.94/0.97/0.78/0.63 (2 kHz 0.48, 4 kHz
    0.34); cross-plane V8 880 rpm 0.99/0.98/0.96/0.71 (2 kHz 0.42).
  - The share does not depend on the mic (a linear path scales repeat and residue alike).
    `held:RPM:0` above idle is overrun with fuel cut: perfectly periodic, not comparable. The
    period finder locks on wrong on interior mics.
- **Idle periodicity: hold the speed and you get a tone** (2026-10-28). `--render held:` pins the
  crank; idling free (`--headless --throttle 0`) the LFA repeats 0.48 / 0.42 / 0.15 / 0.02 of its
  125 / 250 / 500 / 1000 Hz octaves, like the real LFA (0.52 / 0.50 / 0.19 / ≈0) and a real Viper
  V10 (0.38 / 0.48 / 0.12 / 0.01). Cross-plane V8 free 0.84 / 0.68 / 0.33 / −0.02 (real Camaro
  0.52 / 0.27 / 0.09 / 0.01); flat-plane V8 free 0.97 / 0.93 / 0.76 / 0.41 (real RS4 0.78 / 0.46 /
  0.22 / 0.02): only the flat-plane stays too regular. Compare idle only free-running.
- **At load the render repeats too much, and five suspects are cleared** (2026-10-28):
  - Real Viper V10, mic at the exhaust, slow ramp under load ~2100 rpm: 125 / 250 / 500 / 1k Hz
    0.40 / 0.36 / 0.02 / −0.01. LFA held 2100 WOT: 1.00 / 0.91 / 0.86 / 0.60.
  - Crank coupled to the car (`--render gears`, 6th gear, 7760 rpm) ≈ held (500 Hz 0.88 vs 0.90):
    not a held artifact at load.
  - Tail-pipe jets (`sound.flow_noise`, ARP876): 0.0 dB at 1–4 kHz when removed.
  - Valve flow noise (Nelson–Morfey): −73 dB short (design doc 2026-10-28, NO-GO).
  - Cycle-to-cycle combustion already matches Soltau's WOT data (COV p_max 7–10 %, IMEP
    1.5–1.9 %).
  - Non-linear FV exhaust is not less periodic (7740 WOT, 1 kHz 0.76 vs waveguide 0.67).
- **`scripts/authenticity.py`, a repeatable measure of it (2026-11-04)**: share of each band's
  energy that does not repeat cycle to cycle (blocks of 8 cycles, rpm and ramp tracked), dB.
  `FILE@T0:DUR~RPM!` for a render, `~RPM` for a recording with a known rpm (±3 %).
  - Real, standing: Audi A4 2.0 TFSI revving ~3230: 63–250/250–1k/1–4k/4–12k = −7.0/−0.8/−0.5/0.0,
    corr 0.83, HNR 7 dB; Unimog steady 2400 at the exhaust: −13.9/−4.3/−4.0/−1.4, corr 0.88,
    HNR 9 dB. Onboard and pass-by clips carry wind and road too: not references.
  - Render held 3200 at 5 % load: K20A −24.5/−20.1/−10.6/−1.2 (corr 0.997, HNR 25), inline-4
    −19.8/−6.4/−3.1/−0.3 (HNR 18), LFA −10.9/−12.8/−7.9/−2.9; TDI 2400 −30.5/−16.7/−5.1/−1.0.
  - **Ablation (K20A and inline-4 at 3200, 5 %)**: with `combustion_jitter` 0 everything
    repeats (K20A −53/−42/−23/−6, HNR 47). Turning off flow_noise, valve_jet, intake, block or
    mechanical changes no band by more than 0.7 dB. Every source of randomness we have is far
    under the orders; the real engines' broadband sits ~8 dB under them (HNR), ours 18–25.
  - Clean references (2026-11-04): Skoda Octavia 1.6 standing, mic at the catalyst, 1300 /
    1550 rpm: −10.2/−3.4/−1.1/−0.3 and −13.5/−4.0/−2.7/−0.1 (inline-4 held there, 3 % load:
    −13.6/−7.5/−3.1/−1.8: at no load we are within 2–4 dB). Porsche 911 SC cruising at constant
    rpm, mic at the tails (`…constant_rpms_exhausts__0123.9…@7:9~2560`, `__0178.2…@4:13~3600`):
    −21.8/−5.1/−1.3/0.0, −19.1/−7.1/−0.5/0.0, −20.0/−5.9/−1.7/−0.3, corr 0.98–0.99, HNR 17–20 —
    under load the gap is 250 Hz–4 kHz only (ours −20…−27 / −6…−14). Viper V10 slow pass-by
    ~2410: −6.7/−3.9/+0.7/+1.3.
  - A random per-firing arrival-time jitter on the K20A render (diagnostic, scratchpad
    `jitter/jitter.py`): σ 80 µs → −21.2/−15.0/−3.8/0.0; 160 µs → −15.7/−9.3/−1.2/0.1. The real
    shape is less random at 63–250 Hz and more at 250 Hz–1 kHz than any jitter: some additive
    broadband too.
  - **`held:` pins the crank and hides part of the randomness.** Each cycle's IMEP spread moves a
    free crank's speed; that random walk is real and `held` removes it. K20A free-running
    (`--headless --throttle 0.1`, 3377 rpm): −17.4/−10.2/−7.3/−0.8 against held 3200 at 5 %:
    −24.5/−20.1/−10.6/−1.2. Compare free revving with free revving (Audi A4 standing:
    −7.0/−0.8/−0.5/0.0): the gap left is ~9 dB at 250 Hz–1 kHz and ~7 dB at 1–4 kHz.
  - Literature search (2026-11-04): no paper splits tail-pipe noise into periodic and random with
    values. Candidates by evidence: speed wander (above), pulse-amplitude spread (real 63–250 Hz
    implies 10–30 %, ours 4–6 %), the tail jet's pulsed vortex rings (Sakurai et al. 2005, Honda
    R&D Tech. Rev. 17(2): measured broadband above the 1D tones over 4 kHz; Powell 1953: a pulse
    jet's broadband 25–30 dB over a steady jet's), the jet's hydrodynamic pressure on a close mic
    (Arndt, Long & Glauser 1997, JFM 340), muffler flow noise at high flow (Davies & Holland 1999).
  - Tail jets ×30 in amplitude (`sound.flow_noise: 30`, +29.5 dB, Powell's pulse-jet excess)
    on the free-running K20A at 3377: no band moves more than 0.1 dB, rms equal to 4 digits. At
    part load the U⁸ jet is far under the orders even boosted: not the missing randomness.
- **Real LFA at load, clean reference (2026-10-29)**: chassis-dyno clip `dyno-gears.wav` 7.0–9.4 s
  (6060→7300 rpm, `scratchpad/opus-overrun/mosq/real/`, local only; `agent2/py/ang.py FILE 7.0 9.4 6600`):
  bands re total 63/125/250/500/1k/2k/4k/8k = −26.3/−22.7/−17.5/−4.7/−8.5/−5.1/−7.9/−16.9 dB, periodic share
  0.18/0.67/0.82/0.97/0.75/0.54/0.30/0.05, firing H1..H8 0/−9.6/−7.6/−12.1/−9.7/−13.9/−20.7/−22.0 dB,
  half orders (5, 15, 25, 35) −14.5/−15.3/−24.0/−18.8, other cycle orders −20.3. The other clips
  (`exhaust.wav`, `dyno-f1.wav`) sweep too fast for the tracker above 1 kHz. *(verified: ran it)*
- **Model held 6700 WOT, Reference, default listener** (`--render held:6700:1.0:3`): before per-bank
  block: bands −23.5/−16.6/−30.6/−1.4/−6.5/−23.9/−28.9/−33.1, H1..H8 0/−5.1/−26.6/−26.9/−29.2/−30.8/
  −37.8/−39.2, half orders −40.6/−42.8/−42.7/−52.1; after (feat/realism-fable): half orders
  −38.4/−24.5/−25.2/−34.2, bands and harmonics within 1 dB of before. *(verified: 2026-10-29)*
- **The drive scene's full-throttle window rides the limiter** (8750–9035 rpm, cycle orders 1–3 at
  −3/−8/−15 dB re firing from the fuel cut): compare load against recordings with `held:`, not with
  `drive`'s 5.8–7.4 s. *(verified: `track.py` on the render, 2026-10-29)*
- **The sub muffler (2026-10-29)**: a packed straight-through can of 0.38 m, Ø120 over a 70 mm pipe, 16 kPa·s/m²,
  hot gas 900 K — our 1D lining TL 2.3/8.4/17/28/51/72 dB at 1/2/3/4/6/8 kHz, the 2D mode-matching model
  (`papers/silencers/lining-study/modal2d.py`, needs scipy: `uv venv` in the scratchpad) 16/38/39/25/13/9. Either
  way it cuts the LFA's H2–H6, which the dyno clip keeps; the patent's sound after the sub muffler is flat to
  5 kHz. The preset carries it as a plain pipe. Held 6700 WOT against the dyno clip (`ruler.py`, RMS dB over
  bands + H2..H8 + half orders): packed 12.3, plain 8.9, plain without catalyst 8.4. What remains: H3 −13,
  H4 −17, half order 5 −20, 63/125 Hz +6. The 1D lining is under 2D below ~3 kHz and far over above
  (beaming): the lower tip's glass wool (Ø100 over 60 mm, 0.12 m) is 1.9 dB at 2 kHz in 1D, 12 dB in 2D, so
  it cancels the upper tips at f = c/(2ΔL) ≈ 2 kHz more than it should.
  **Update 2026-11-20:** with the surge tank and the real cam flank the LFA's H2–H6 at its tips match two recordings
  of the car only with a sub muffler in: a 105 mm can (the road silencers' smallest shell) over 0.38 m reads H2–H6
  −15.3 dB re H1 (real −12.7/−13.3), the orders between −22.6 (−21.7), the floor −26.9 (−27.4/−29.5); without it
  closed throttle at 6800 rpm sounds 1.6 dB(A) over full. At full throttle the flow through it (Rao & Munjal's
  grazing flow, the render's flow-following silencers) takes much of the loss the 120 mm can had in still gas.
- **Packed silencers by the 2D model (2026-10-29)**: `silencer2d.rs` ports `modal2d.py` (complex Bessel from the
  `complex-bessel` crate) and matches it to 0.01 dB (0.4 m can at 1 kHz 17.34, 0.8 m 33.82). Backward chamber waves
  must be referred to x = L or the evanescent modes overflow (the 0.8 m can failed everywhere, and 8 kHz+ on short
  ones). Realised as 17 half-octave peaking biquads per direction (octave bands missed a 13 dB/half-octave slope by
  4 dB); the fit must be kept passive (an overshoot of +1.6 dB at 62 Hz stalled the single) and its target clamped at
  −50 dB. ~18 ms per frequency in release, 33 frequencies per packed silencer when the net is built. LFA held 6700 vs
  the dyno clip: 13.3 → 13.4 dB (only its lower tip is packed).
- **"Sounds far" (2026-10-29, branch fix/sounds-far)**: at 91312d5 the held-6700 yardstick (`agent2/py/ruler.py`)
  is 12.9 dB; listener variants (one change each): ground off 13.8, dyno-camera spot (2.5 m, 0°, ear 0.5 m) 17.2,
  1 m on axis free field 15.4; no muffler chambers 14.4; textures off 19.8; `model: fv` 18.3; no catalyst 10.9
  (the catalyst takes 7 / 12 dB at 4 / 8 kHz). With steepening merged: 7.6 (default listener), 8.5 (dyno spot).
  The 2–8 kHz deficit is the exhaust's own (textures off: −25 / −43 / −55 re total), not the listener's.
  *(verified: ran the renders, scratchpad `far/cmp*.txt`)*
- The dyno clip `dyno-gears.wav` is YouTube WS5vCsgCFB0 ("Lexus LFA Runs Through Gears On A Dyno", 9.5 s, Opus
  48 kHz stereo): a vertical phone on the dyno deck straight behind the tips, ≈ 2.5 m back and ≈ 0.5 m high, in a
  concrete-block room; the screen shows 7919 rpm / 276 km/h. *(verified: ffmpeg frame at 8 s, 2026-10-29)*
- `--render gears` of the LFA takes ≈ 8 min on 6 cores (both binaries, one render each in parallel), `held:…:3`
  ≈ 30 s. *(verified: ran them, 2026-10-29)*
- **Steepening's real-time cost (2026-10-29)**: LFA, `--headless --seconds 5 --throttle 1`, cores 0–5, under the CPU
  lock, audio seconds per wall second including start-up (the 2D silencer fit): Ultra 0.45 linear → 0.28 steepening,
  High 0.57 → 0.32 (−40 %: the +28 % of the spec plus the exhaust thread, which a steepening net never takes).
  Absolute values are far below the table above (start-up and the 6-core pin); the ratio is the fact.
  *(verified: `scratchpad/far/rt.sh`)*
- **Surge tank (air box) on the intake (2026-10-29)**: `IntakeDef.airbox: Option<AirboxDef>` (volume, n ducts,
  duct length and Ø). ITB trumpets draw from the box (`throttled_ghost` on the box's state, held over an FV step),
  the box breathes through its ducts as 1D pipes of L + 0.85a + 0.6a (Kinsler §10.8 end corrections), the sound
  source is c·Σρu at the duct mouths; with a common throttle the throttle draws from the box. Verified against hand
  values: the 20 L / 2 × Ø90 × 0.40 m box rings at 61.6 Hz (x·tan x = A_t L'/V, within 4 %) and passes a 2 f_H
  pulsation at |H| = 0.513 (runner line + duct line + box by transfer matrices, within 3 %). LFA with
  `AirboxDef::lfa_v10_estimate()` (not in the preset — FINDINGS 2026-10-29): `--render drive` blips octave bands
  re total 31/63/125/250/500/1k/2k/4k −15.0 −9.6 −3.3 −15.8 −21.0 −14.2 −8.7 −18.2 (before −15.9 −10.9 −3.8 −15.8
  −8.3 −14.1 −9.8 −19.5; real blips −12.3 −6.4 −3.3 −13.2 −17.2 −11.7 −14.1 −14.8): the 500 Hz band goes from
  +8.9 dB over the real car to −3.8 under it; idle's 31 Hz band −23.8 → −9.3 (real −9.2: the box's 62 Hz mode
  under the 54 Hz idle firing). Held 6700 WOT ruler 7.6 → 24.9 dB (firing order gone, see FINDINGS). Dyno: −0.1 %
  at 1000 rpm to −2.6 % at 8000 (489 → 476 hp), one dynamic head at the ducts' exit. *(verified: scratchpad
  `airbox/`, 2026-10-29)*
- **The surge tank's real-time cost is nil (2026-10-29)**: LFA `--headless --seconds 5 --throttle 1`, cores 0–5,
  under the CPU lock, audio seconds per wall second with and without `lfa_v10_estimate()`: High 0.344 → 0.327,
  Ultra 0.238 → 0.248 (±5 %, noise either way: two 12-cell ducts beside ten 8-cell runners and the exhaust).
  *(verified: scratchpad `airbox/rt.sh`)*
- **Surge tank wall radiation does not carry the LFA's exterior firing order (2026-10-29)**: a (1,1) panel mode
  (Fahy & Gardonio §2.6) at 400 Hz (Okamoto JSAE 2011 Fig. 18), η = 0.3 (Battistoni SAE 2006-01-0705, PA6-GF30,
  ASTM E756), 3 mm (Aydemir SAE 2009-01-0345: 2.4–4 mm), 0.175 m², driven by the 20 L box's pressure and radiated at
  the intake's place: blips' 500 Hz band −21.0 → −20.9 dB, held 6700 unchanged. The box's pulsation is too small;
  Yamaha's tuned walls feed the cabin duct, not the exterior. The exterior firing order at high rpm must come from
  the exhaust, which (with the airbox) has none at 6700: next suspect, the bypass muffler's transfer at the firing
  frequency (its assembly cut the steady pulses ~15 dB at 6000 rpm WOT vs no bypass; isolating the chamber path,
  the baffle's open area or the outlets alone changed nothing). The patent's spherical final chamber has a flat
  TL of 15–22 dB from 100 to 1000 Hz (JP 2008-169751 A Fig. 6); the 1D cylinder model has a zero near c/2L.
- Yamaha YZF-R1 (2009) service manual, "throttle body synchronization": vacuum difference between ITBs ≤ 1.33 kPa
  (10 mmHg) at warm idle 1150–1250 rpm (#2 set 1.6 kPa higher) — papers/engine-data/yamaha-r1/.
- **Bypass muffler, valve open, against the patent (2026-10-29)**: linear IR of the LFA net (one cylinder, valve
  open) re no bypass, third octaves 400/500/630/800/1000 Hz: full −22/−34/−49/−30/−35 dB; chamber path blocked
  −14/−16/−17/−18/−20 (≈ the patent's spherical final chamber, Fig. 6: 15–22 dB flat). But the patent's outlet
  orders at ≈ 4800 rpm (Fig. 8: H2 +4, H4 +8, H5 +1 re H1, from a nearly flat source, Fig. 7A) do not pick
  either: full +5.3/+2.3/−12.7, blocked +8.1/−6.2/−1.1 (both 8.6 dB RMS). Blocking it by ear was not adopted.
  The LFA's H3 at 6700 (−19…−24 dB vs the dyno clip) is not from the tips' length difference (two equal tips
  keep the 1.6 kHz dip).
- **Surge tank, lead "400 Hz tuned by surge tank" (2026-10-29)**: by transfer matrices (`scratchpad/tank/py/tank_tf.py`:
  box compliance + duct line with Kinsler §10.8 end corrections, mouth p = 0; |U_mouth/U_in| = 1/|cos kL' − (kV/A) sin kL'|)
  every tank consistent with the EPC drawing passes the 6700 rpm firing order (558 Hz) at −17…−26 dB: today's 20 L /
  2 × Ø90 × 0.40 m −24 dB (its duct line rings at 379 Hz — Okamoto's "400 Hz" is the duct's half-wave, and it does
  nothing at 558); per bank 10 L / Ø90 × 0.20…0.60 m −17…−24; 4…15 L −16…−27; the air-cleaner case (Fig. 18) plus a
  Ø70 × 0.2–0.3 m snorkel in series −42…−45. Only a Ø90 duct of 0.25 m (half-wave at 555 Hz) passes it (+10 dB): twice
  shorter than the drawing's, off Okamoto's 400 Hz, and a tune by ear — not adopted. The exterior firing order at high
  rpm is the exhaust's. *(verified: ran the script)*
- **The LFA exhaust net's transfer by cycle order at full load (2026-10-29)**: `engine::probe::probe_lfa_orders` (ignored,
  `--release -- --ignored --nocapture`): the engine held WOT, orders of the banks' valve flows (both banks summed), of
  the net's radiated tails and of the output; tails/flows = the net's transfer, dB. Both banks' flow at 6700 rpm:
  order 10 (558 Hz) 0, 20 −15, 5 −19, 15 −23, 25 −27, 30 −25, 40 −31, 50 −26: an H1-rich source. Transfer at 6700:
  orders 5/10/15/20/25/30/40/50 = 96/**65**/86/74/79/75/88/84; at 6000 (500 Hz): 96/80/87/**52**/105/60/57/82; at
  7400: 104/70/83/98/77/67/96/81; at 4800: 116/76/**51**/71/74/83/59/66. Neighbouring orders differ by up to 52 dB
  (6000 rpm, orders 20 vs 25): the net is a high-Q line, and the firing order sits in a trough at 6000–7400. The
  patent's measured line is flat: at ≈ 4800 rpm the sound at the main muffler's inlet has H2/H4/H5 at −2/+2/−3 dB
  re H1 (JP 2008-169751 A Fig. 7A, dashed) and the radiated sum +3/+9/+2 (Fig. 8); ours radiates −16/−32/−42.
  Insertion of the merge + bypass muffler at 6700 (no-merge transfer minus full): 21/38/26/31/35/38/17/12 dB by
  order — 38 at the firing order against the patent's ≈ 10–11 per order (Fig. 7A solid vs dashed). The bare line
  (no merge, no catalyst) still has order 10 at 116 against 128–132 for orders 15–25; tail 1.5 m 113 vs 127–135;
  collector 0.1 m 115 vs 116–128; primaries 0.5 m 114 vs 107–117 (flat); linear (no steepening) 103 vs 122–128.
  *(verified: `scratchpad/tank/probe3.log`, `probe4.log`)*
- **The tank in the preset, measured (2026-10-29)**: held 6700 WOT, lines re total (`scratchpad/tank/py/peaks.py`):
  without the tank order 10 (558 Hz) −4.2 dB, order 45 (2512 Hz) −5.3; with it order 10 −24.0, order 45 −3.5, the rest
  within 1 dB: the trumpets carried 20 dB of H1 and nothing else. Ruler 7.6 → 24.9 (its tracker locks on 7538 rpm);
  guard RMS 20.0. Exhaust variants with the tank (held 6700, H1 re total): baffle 0.01 −23.7, 0.03 −23.5 (blocking the
  chamber path does nothing at WOT amplitude), no final chamber −21.2, equal tips −27.2, no bypass −20.2, no merge
  −15.4, primaries 0.45 m −25.6. Order 45 (2.5 kHz, 9 × the bank order) is the top line of every variant that keeps
  the merge (−3.5…−7.8): bank B's extra 0.10 m collector stub between the collector and the catalyst, c/2L ≈ 2.5–2.8
  kHz. Lead "blips" (`yam/cmp.py`) unchanged from the entry above. *(verified: `scratchpad/tank/r-*.txt`)*

## Gear-change thump and the LFA final chamber (2026-10-30, branch fix/gear-thump)

- **Real LFA upshifts (dyno, `real/dynogears.wav`, 4 shifts at 3.95/5.50/6.35/8.45 s):** the < 200 Hz
  5 ms RMS peak within ±0.1 s of the shift is −11…−15 dB under the median full-band RMS of the
  0.5 s before, and +7…+13 dB over the median LF RMS there. A render's shift must meet those two numbers.
- **Fitted LFA line, linear sweep (one cylinder's 0.01 flow tone → radiated sum, 7000 rpm, dB):**
  full 50–200 Hz 55–61, 400–950 Hz 15–31; no bypass muffler 43–66 (flat ±8); open path only (no
  closed chambers) 26–54 at 400–950; with the final chamber as `Segment::Measured` (Fig. 6) full
  33–51 / 10–24, open path only 37 / 16–24 — the listener's dQ/dt adds +6 dB/oct on top.
- **JP 2008-169751 A read-outs:** Fig. 6 (spherical final chamber TL): 15 dB at 100 Hz, 18.5 at 200,
  20–21 from 300 Hz to 1 kHz; the flat chamber 10–33 dB with dips at ≈ 300/500/800 Hz. Fig. 7A
  (4800 rpm, engine original dashed vs upper tip): 1次/2次/4次/5次 118/122/125/122 → 110/110/112/113 dB.
  Fig. 5A: with the valve open the gas goes straight into the final chamber; the chamber path enters
  through holes 20A/22A and short pipes 36, not long pipes. ¶0035: path A's flow resistance ≫ path B's.
- **Shift flare:** open-loop cut ramp + clutch release at 10/s: 8760 → 9007 rpm (limiter at 9000) at
  every shift; with the speed loop (1 % band) 8920–8990. The LFA's `shift_time_s` 0.2.
- **Thump per build (5 ms LF peak over steady full-band / over steady LF, five shifts):** `ecu.wav`
  +6…+9 / +28…+31; measured chamber +9…+13 / +35; + injection latency +9…+13 / +34…+35; mass-flow
  source (reverted, see FINDINGS) −3…+6 / +24…+33.
- **Real LFA shift, per band (5 ms peak at the cut-in re the steady full-band level, four dyno
  upshifts):** 10–60 Hz −11…−24, 60–200 Hz −14…−17, 200–800 Hz −7…0 dB. Committed render +8…+12 /
  −4…−9 / −7…0; blank-out torque cut −3…+5 / +1…+3 / +2…+13; 70 L shell +10…+13 / −5…−18 / −5…+1.
- **3D view frame cost (2026-11-03, `bench_3d_frame`, LFA V10, 1920×1080, RTX 3080 Ti):** CPU
  (instances + prepare) 0.010–0.012 ms mean, 0.02 ms p99 at every detail; GPU submit-to-done
  0.10–0.11 ms mean, 0.33–0.50 ms p99. No integrated GPU, lavapipe or headless GL on this machine:
  the 5850U target is unmeasured (a ~20× slower GPU would still be ~2 ms mean); the detail
  governor steps down after 30 frames over 18 ms.
- **egui-wgpu 0.37 / wgpu 30:** pipeline layouts take `&[Option<&BindGroupLayout>]`, vertex
  buffers `&[Option<VertexBufferLayout>]`, depth `Option<bool>`/`Option<CompareFunction>`;
  glam 0.34 has no `Mat4::perspective_rh` — use `glam::camera::rh::proj::directx::perspective`
  (z 0..1, y up, what wgpu wants) and `camera::rh::view::look_at_mat4`.
- **3D view with intake, exhaust and turbo (2026-11-03, `bench_3d_frame`, LFA V10, RTX 3080 Ti):**
  CPU 0.014 ms mean / 0.02 ms p99; GPU 0.10 ms mean, p99 0.1–0.2 ms (one 1.9 ms p99 at Low under
  another session's load). Pipes are still parts (built once per definition).

- 2026-11-04: building an `ExhaustNet` designs every silencer's and catalyst's filter: 1.3 s on the LFA, 0.2 s on the K20A (release, loaded machine); the FV net 0.08–0.15 s. An engine edit used to rebuild it on the first step, stalling the sound; `Engine::adopt_exhaust_from` now keeps it when the exhaust is unchanged (LFA first block after a bore edit 1137 → 6 ms). Slider drags sent one rebuild plus one quality-benchmark thread per frame; `runner::EditGate` paces them (≤ 1 rebuild / 250 ms, benchmark after 1 s still).
- **3D view with the realistic parts (2026-11-04, `bench_3d_frame`, LFA V10, RTX 3080 Ti):** CPU
  0.017 ms mean / 0.03 ms p99 at every level; GPU mean Low 0.11 ms (primitives), Medium 0.16 ms,
  High 0.19 ms (the realistic piston, rod, throw, valves: about twice the primitives' 0.10 ms).
  The 5850U iGPU is still unmeasured; Low keeps the primitives for it.
- 2026-11-04: an `ExhaustNet`'s cost was its pipes' wall filters (fitted once per stage: a 1.84 m
  pipe 70 ms) and the 2D silencer's equalizer (the LFA's packed silencer 0.7 s). Wall filters are
  now fitted once per pipe; silencer equalizers are memoized (`silencer2d::EqMemo`, carried across
  rebuilds). First block after an exhaust edit, release, loaded machine: K20A 178 → 26 ms,
  LFA ~1100 → 100 ms; whole K20A net 194 → 23 ms.
- **The LFA's "wind" on fix/duct-jet-density is a missing firing line, not more noise (2026-11-24)**: ear_gate's
  wind = aperiodic 150–500 Hz minus H1 (`scripts/realism/ear/ear_feat.py`), on the held renders at 6800/7500 rpm.
  Aperiodic 150–500 Hz: main (b2) 48.6/49.2 dB, branch (rad1) 43.7/45.4, 0.675 m primaries 43.8/44.8; H1 at the
  ear: 71.5/70.3, 35.7/49.6, 55.0/42.2. Main's H1 came from the open trumpets; with the surge tank only the
  exhaust's is left.
- **LFA designed path, valve open, impulse from an anechoic source vs a straight 70 mm pipe (2026-11-24)**: the
  bank lines + Y alone 0–6 dB; the main muffler 24–25 dB at 300–475 Hz, 30–37 at 550–800. Of that, the spherical
  chamber's published TL ≈ 20 dB flat (equal tips: 25–27 dB at every frequency) and the three unequal tips 5–11 dB
  more at 550–1200 Hz (no chamber: 4–5 dB below 475 Hz, 10–17 above 550). Spike: `scratch-refs/spike-designed-path-tf.patch`
  (needs a 1 s warm-up for the bypass valve's 0.3 s travel, and the zero-input run subtracted: the mean flow's DC).
  **In the running engine it is as big** (held renders at the ear, every synthesized source and the intake off,
  valve open; the main muffler against a 0.5 m × 85 mm pipe in its place): 26.2 / 31.4 / 26.6 / 30.0 / 25.6 dB at
  orders 5 / 10 / 15 / 20 / 25 at 4800 rpm, 21.7–30.3 at 7500. The patent's Fig. 8 against its Fig. 7A dashed "engine
  sound" (sub muffler only) reads ≈ 11 / 5 / – / 5 / 5.5. With the chamber element a plain 0.25 m × 85 mm pipe:
  4.4 / 11.5 / 7.3 / 10.0 / 6.2 (the patent's); an empty 14 L expansion chamber in its place: 22.5 / 20.5 / 19.2 /
  7.1 / 12.7. Inputs: `scratch-refs/wabl/{nomain,nochamber,reactive}`, `wabl/il.py`. **But the dashed curve is
  not a measurement:** ¶0047 says the engine sound is "例示" (illustrated) with it, where the solid outlet curves
  are "示されている" (shown); the in-situ figure above is no target. An empty chamber is reactive (its loss is
  reflection), the `Segment::Measured` element absorbs both ways; at 7500 rpm both give the same H1 at the ear
  (47.7 vs 47.4 dB in the held renders), so the realisation is not what keeps H1 low there.
- **What the LFA's wind is made of (held 7500 rpm, rad1, 2026-11-24)**: aperiodic 150–500 Hz 45.4 dB against H1
  49.6. Each source alone (the rest off): tail jet 43.7, combustion jitter 37.2, piston slap 36.0, main bearings 33.1,
  the others ≤ 14 (they sum to 45.4); every one alone already fails the real bound (−22.4: `lfa-accel` −22.4,
  `lfa-flight` −24.3). All off: 13.4 (wind −34). At the ear the intake's H1 is 5 dB over the exhaust's (−44.5 vs
  −49.9 dBFS at 7500; at 6800 they meet at 168° and cancel to 35.7 dB). With the chamber element a plain pipe the
  H1 rises 12–28 dB but the jitter's spread rides the pulses through and rises with it (aperiodic 58.0, wind −4.0;
  jitter off: −17.3). The real exterior recordings' H2 sits 13–25 dB under H1 at 6400–8000 rpm (`look/ordtab.py`,
  five `lfaref` clips); the model's exhaust alone has H2 1–6 dB over it.   Both corrections together (0.65 m primaries, the merge inflow's H1/H2 balance in `pscan/`, and a plain pipe for
  the chamber element): wind −18.3 / −12.6 at 6800 / 7500 (from +8.0 / −4.1), H2 −28 / −6 re H1, floor −48 / −42.
  Either alone: 0.65 m −12.1 / +3.8, no chamber −6.9 / −4.0. Inputs: `scratch-refs/wabl/`.
- **The LFA's exhaust in pascals, and why it is quiet at the top (2026-11-24)**: exhaust only (intake off), held WOT,
  Reference fidelity, the listener's 3 m (`scratch-refs/wabl/abs`, `abs_probe`): H1 85–88 dB SPL at 4800 rpm but
  68–72 at 7500, the tail jet's broadband between the cycle lines (150–500 Hz) ≈ 70 dB there, so H1 ≈ floor. At
  4800 the third octaves 125–315 Hz sit 23–39 dB under the 400 Hz one, as in the patent's measured Fig. 8
  (24–41): the floor is right where H1 is strong. The fall is upstream of every silencer: with a straight 70 mm
  pipe after the collectors the merge inflow's H1 (`order_probe`, 0.675 m primaries) reads 50 / 46.8 / 46.7 /
  41.9 / 37.8 / 32.5 / 14.5 / 29.1 / 36.4 dB at 4800 / 5600 / 6400 / 6800 / 7200 / 7500 / 7800 / 8200 / 8600 rpm —
  the shut primaries' third zero, 3c/4L with c ≈ 585 m/s, a −30 dB notch with −10 dB skirts 400 rpm wide (0.8 m:
  at 7000–7500). Merge inflow share / H2 re H1 at 6500 / 7000 / 7500 / 8000 rpm: 0.575 m −1.5/−7, −0.9/−8,
  −0.4/−17, −1.3/−20; 0.60 m −1.7/−6, −0.5/−13, −0.7/−21, −2.1/−12; 0.625 m −1.2/−8, −0.6/−19, −1.2/−16,
  −2.7/−7; 0.65 m −0.6/−15, −1.3/−19, −2.3/−6, −3.4/−4 (`scratch-refs/pscan`). The combustion's own spread is
  right where it starts (cylinder pressure at EVO COV 1.4–1.8 %, port peak 1.5–2.4 % at 6800–7500 — Soltau's
  charge spread), but where H1 sits in a zero its per-cycle spread is −16 dB re its line at the merge inflow
  (0.8 m, 7500) against −29…−33 elsewhere (`jitter_probe`).
- **The LFA's exhaust source is flat with rpm (2026-11-24, 0.60 m primaries, Reference, held WOT)**: the ten
  exhaust valves' summed flow at the firing order (cycle order 10) against its mean: −16.6 / −13.6 / −19.0 /
  −19.0 / −24.3 dB at 3600 / 4800 / 6000 / 7200 / 8400 rpm, so its dQ/dt holds at 44–47 dB from 4800 to 8400
  while the mean flow grows 4.8 dB (the pulses spread over more crank angle as the speed rises); valves to the
  merge's inflow −10 / −10 / −8 / −12 / −15 dB (the shut primaries' third zero at 8800). The tail jet grows
  with the mean flow (U^7.5) and the mechanical sources with the speed, so the firing line loses to them at the
  top: the LFA's physgate crescendo −3.5 dB(A)/octave (rad1 +6.1, b2 passed with the open trumpets' intake).
  Spike: `scratch-refs/probe-source-by-rpm.patch`. Gates of 209c393 (`gates-j60`): spectral PASS (low-mid
  −18.6 vs −13.7 on rad1), blind2 hnr 10.7 (rad1 13.5), ear_gate wind −4.3, lift +5.4 (rad1 −0.8), peak +18.1.
  Held renders: exhaust alone −11.8 / −10.5 at 6800 / 7500 (0.8 m: −2.9 / −2.1); with the intake −14.3 / −2.9 —
  at 7500 the intake's H1 meets the exhaust's out of phase at the fixed ear (51.9 → 44.1 dB).
- **Where the LFA's wind sits in rpm, and the lift (2026-11-24)**: the ear_gate's wind per 500 rpm bin, real
  exterior (`lfa-accel`, `lfa-flight`): −19…−25 dB flat from 5500 to 8000. Renders: j60 (0.60 m) −20 at 5500–6000,
  −14 at 6500, −2.5 / −3.1 at 7000 / 7500, −11 at 8000, −22 at 8500; rad1 worse (+7.9 at 6500); b2 (open trumpets)
  −20…−26 throughout (`scratch-refs/wabl/windrpm.py`). In pascals at 3 m (`abs_probe`, `wabl/abs2`), exhaust only:
  H1 87.2 / 81.9 / 74.2 / 77.6 / 75.9 / 72.9 / 75.0 dB at 5500…8500 rpm, the floor between the lines 64→71; with
  the intake 85.7 / 81.1 / 80.2 / 78.2 / 68.0 / 73.5 / 79.9 (it meets the exhaust's out of phase at 7500, adds at
  6500 and 8500 at the fixed free-field ear). Lift at Heywood's 12 % of the bore (630202f): H1 +2.5…+4 dB at
  6000–8500, floor +0.5…+1.8; dyno 530 hp at 8000 (L/D 0.25: 497; 0.30: 525; 0.33: 534). EVO 7–21° earlier: +1…+2 %
  at 8500 only. Gates (`gates-hey`): ear_gate wind −11.3 (j60 −4.3), bank +3.9 (j60 +7.0), lift +5.1, peak +16.5;
  spectral PASS (low-mid −20.1); regress vs b2 PASS; blind2 hnr 13.8 @ bin 3 (b2 11.0, also failing); physgate
  crescendo −3.2 dB(A)/octave (deepest hole 7500: the intake–exhaust cancellation) and load +0.0 (overrun as loud
  as full; b2 +6.3 and +8.1 with the trumpets carrying the level).
- **A merged exhaust's tail jet was counted twice (fixed 2026-11-24)**: the LFA's two tails (its banks' halves of
  the one radiated signal) each blew a jet of all three tips' area at one bank's flow through its 70 mm pipe:
  6 dB over one jet of the tips at the whole flow (65.8 vs 59.6 dB at 6000 rpm, 3 m). ARP876 at 7500 rpm, 3 m:
  68 dB per old tail.
- **JP 2008-169751 A ¶0048–0049 (2026-11-24)**: the long lower tip (52, ≥ 2× the others) cuts the 1st order at
  ≈ 4800 rpm and its wool the ≥ 1 kHz; Fig. 8's combined tail sound has H2, H4, H5 over H1 by design. The model
  at 4800 rpm sends H2/H4/H5 into the muffler at −10/−22/−34 dB re H1 (`order_probe`, the merge's inflow); with
  no main muffler the ear's third octaves at 800 / 1250 / 1600 / 2000 Hz read −15 / −16 / −28 / −30 dB re the
  400 Hz one, where Fig. 7A's dashed engine sound reads ≈ −1.5 / +2 / – / −4.5 (its curves are drawn through
  third-octave points, so a reading, not a measurement; an A-weighting would explain ≤ 6 dB of it). At 0.5 m
  the no-main H1 reads ≈ 126 dB against the dashed 122. The road recordings hold H1 dominant
  (−2…−3 dB of 100 Hz–4 kHz, 400–700 Hz), so the exterior firing line must largely come from elsewhere than
  the tail: the intake (Okamoto 2011 Fig. 18, "surge tank tuned 400 Hz, equal tone tuned intake") — the model's
  tank ducts pass −17…−26 dB at 558 Hz (FINDINGS 2026-11-20).
- **The LFA's treble in main was its doubled tail jet (2026-11-25)**: A-weighted octave bands of the held renders,
  main (`b2`) against the branch after the double count's fix (`hey`): 4 kHz 54–59 vs 31–43, 8 kHz 43–53 vs
  15–30 dB at 4500–9000 rpm (twice the speed through u⁸: ≈ +24 dB). With it gone the LFA lost its crescendo
  (−3.2 dB(A)/octave), its load contrast (+0.0) and its "shrill" top. The exhaust alone barely follows the load:
  the inline-4 at 4900 rpm, d/dt of the tail flow (`order_probe`, THR=0/1), firing order 2 drops 1.4 dB on
  overrun (8.3 vs 9.7 dB), the 8 and 16 kHz octaves rise 4–7 dB, the cycle's lines 51 dB over the floor at 8–16 kHz
  (28 at full load); 0.5 cm cells change nothing (not numerical). Screening the intake mouth with the body
  (patch `scratch-refs/wabl/mouth-screen.patch`) fails physgate load or crescendo on 9 of 13 presets (inline-4
  load +10.4 → +2.8 dB(A)): shelved until the exhaust follows the load. K20C1 (load +1.9, lift-off) and RENESIS
  (crescendo −3.5) fail physgate on this branch before these changes too (`hey`), and pass on main: the ear by
  the tails (3e8fde4) is the likeliest cause, the same lack of contrast.
- **Tail gas temperature (2026-11-25)**: the tail jet left at a fixed 622 K (`C_TAIL` 500 m/s) whatever the load,
  and the FV path (Reference renders) never walked `pipe_heat`; the walls settled on the 300 K the port
  temperature starts from (304 K after 0.4 s at full load; hand balance ≈ 765 K). Now the walls settle once the
  cycle means have run three cycles, the FV path walks the heat on its `jet_net`, and the jet leaves at the
  tail's gas (`ExhaustNet::tail_gas_k`; LFA ≈ 950 K at full load, only presets with `gas_temperature`). Each
  outlet blows its own jet (`ExhaustDef::jets`: the LFA's three 60 mm tips, St·u/d √3 higher than one 104 mm jet).
  LFA gates (`gates-tj`): treble back (4 kHz +7…+16, 8 kHz +6…+21 dB re `hey`), physgate crescendo +0.7 and
  load +1.8 (bounds 3.0), lift-off PASS; spectral low-mid wind −9.4 (bound −18.4) and ear-gate wind −3.4: the tail
  jet carries it (`attribute.sh`: flow_noise off → −21.6 and −16.2). The jet's own level is ARP876 at ≈ 100 m/s;
  the lines it sits under are what is weak. Against the chassis-dyno clip the waveguide's cycle orders 30–80 read
  9–24 dB low, but the five exterior recordings hold H5–H8 at −19…−48 dB and the render inside them: no verdict
  on the high harmonics yet. `tail_mach` still uses `C_TAIL`.
- **Overrun at the port is real physics, not a bug (2026-11-25)**: FV, held, cylinder 0 (`scratch-refs/ovr/port-pulse-spike.patch`).
  The inline-4 at 4900 rpm: full load 514 mg out per cycle, peak 120 g/s, port gauge −7…+71 kPa; overrun
  (MAP 23 kPa, cylinder 21 kPa at EVO: a real fuel cut-off's) 81 mg back in (−34 g/s) and 133 mg out (+66 g/s),
  port gauge −37…+46 kPa: the same ≈ 80 kPa swing (p = c·ṁ/A ≈ 31 kPa for the displacement alone). The summed
  ports' firing harmonics on overrun re full: inline-4 +2.6/+3.8/+4.1 dB (H1–H3), the tail −0.6/+2.7/−0.4 and
  −9…−21 at H4–H6; LFA at 6800 (MAP 13 kPa) the ports −15.9 dB at H1 but each primary's exit −9…−14, the
  collectors −5.5, the merge −2.8 (`ovr/chain-spike.patch`, per expansion site): the cold overrun gas's
  acoustics gain ≈ 13 dB along the chain. Part throttle: the inline-4's tail H1 dips −19.8 dB at 701 mg/cycle
  (where the cylinder at EVO meets the exhaust's pressure: no blowdown, no reverse rush). FV pipes exchange no
  heat with their walls (hand NTU of an LFA primary at full load ≈ 0.2: −60 K, c −2…3 %; not built).
- **Real pull vs decel at matched rpm (2026-11-25, `scratch-refs/ovr/decel.py`, dyno clips)**: S2000 (two
  clips): 1–4 kHz −5…−10 dB above 5000 rpm on the decel, H1 −1…−17, dB(A) −2…−8; Civic K20 ≈ 0 (a coast-down?).
  Five LFA dyno clips (`yt/lfa/`): −11…+7 dB(A), the decel often as loud as the pull (camera gain control,
  rollers): no load contrast to fit. Full-load share of the A total: real I4 dyno H1 −9…−30, 1–4 kHz −3…−8 dB;
  the LFA render H1 −1…−6 (the exterior recordings also hold H1 dominant). JP 2008-169751 A states no
  microphone distance: Fig. 7A gives no absolute level.
- **The LFA's tip bore sets its wind floor (2026-11-25)**: the jet's level at a fixed mass flow goes as n^−6.5·d^−13
  (ARP876: area × u^7.5, u ∝ 1/(n·d²)), and the 60 mm bore of the three tips is an ESTIMATE from the patent's
  proportions (no published figure; the trims are ≈ 105 mm outside). Held WOT at 3 m (`abs_probe`, all sources,
  `scratch-refs/wabl/tip`): 60 → 76 mm (3 in) exits lower the 150–500 Hz floor between the cycle lines by 7.7 / 8.6 /
  8.3 / 8.3 dB at 6000 / 7000 / 7500 / 8000 rpm (not the 13.4 of the law: the peak St·u/d moves down into the band
  and the jitter and mechanical sources stay); H1 moves +4.9 / +4.4 / −1.4 / −3.3 (the tips' acoustics). H1 over the
  floor: 13.0 / 4.6 / 0.9 / 0.3 dB → 25.6 / 17.6 / 7.8 / 5.3. The model's LFA at full load reads 84–86 dB SPL at 3 m
  (H1 78–85); no calibrated level of the real car was found (Japanese proximity and EU stationary figures searched;
  the FK8 Civic's stock proximity level is 82 dB per HKS's catalogue, rpm not stated).
- **The LFA's tips fitted at 70 mm (2026-11-25, user: fit only the ESTIMATE dimensions)**: grid of held WOT renders
  (`scratch-refs/wabl/fit/grid.py`, ear_feat wind/bank at 5500–8500 rpm). Bore at 0.60 m primaries, wind 5500 /
  6000 rpm: 55 mm −14.1 / −8.3, 60 −18.4 / −11.7, 65 −23.0 / −17.5, 70 −26.8 / −21.7, 76 −29.8 / −25.7, 80 −31.0 /
  −27.3 (real −19…−25); 70 mm taken from the bins where the firing line is sound (80 overshoots them while 7500–8500
  stay at −5…−10 for every bore: the tone, not the jet). Primaries at 80 mm tips, 0.60 / 0.65 / 0.70 / 0.80 / 0.90 m:
  J 74 / 115 / 116 / 67 / 69, each with a hole somewhere in 6500–8500 (the zeros move): kept 0.60. Gates (`t70`,
  LFA): spectral treble −27.0 (PASS; `tj` −17.2), low-mid −16.6 (bound −18.4; `tj` −9.4), ear_gate wind −5.9, bank
  +4.3, lift +1.9, peak +16.5, physgate crescendo −3.7 dB(A)/octave and load +0.7 (`tj` +0.7 / +1.8): with the jet
  quieter the tone's own fall with speed shows. Exhaust alone at the ear (held, every other source off, firing
  harmonics): H1 88.5 / 88.9 / 80.1 / 73.8 dB SPL at 4800 / 6000 / 7500 / 8500, H2 −20.7 / −23.8 / −14.0 / −2.3 re
  H1, while the main muffler's inflow keeps H1 flat (`chain_fv_spike`: 132 / 131 at 4800 / 7500). JP 2008-169751 A
  ¶0046–0047: at high speed the final chamber and the short tips keep the engine's own order balance (no order cut
  more than another); ¶0048–0049: only the long tip cuts the 1st order (and its wool ≥ 1 kHz), so the sum has the
  2nd, 4th and 5th over the 1st (Fig. 8). The model's chamber and tips cut 625 Hz 7 dB and 800 Hz 11 dB more than
  400 Hz: a low-pass where the patent describes a flat path.
- **The tips' junction is not the main muffler's low-pass (2026-11-25, by hand)**: lossless transfer matrices of the
  model's tips (85 mm duct into two 0.13 m and one 0.345 m tips of 70 mm, end corrections 0.6133a, c 560 m/s,
  radiated ∝ f·U) against three equal short tips: +0.9 / +1.9 / +3.0 / +4.5 / +4.8 / +0.7 / −9.3 / −11.6 / −8.0 dB
  at 300 / 400 / 475 / 550 / 625 / 700 / 800 / 1000 / 1200 Hz (the long tip's half-wave shorts the junction at
  800–1200 only). A 14 L compliance at the junction (the sphere's volume) makes it worse: with the tips it is a
  Helmholtz low-pass (equal tips −4.4 / −5.8 / −6.2 at 625 / 800 / 1000 re 400 Hz). The model's extra 7 dB at 625 Hz
  (and the in-situ 5–11 dB at 550–1200) therefore comes from another element of the path (the long tip's wool
  segment, the bypass's closed path in parallel, the merge): measure each against the patent's three measured
  curves at 4800 rpm (Fig. 7A solid: short tips alone keep the engine's balance; Fig. 7B: the long tip alone
  with the 1st and ≥ 5th orders cut; Fig. 8: the sum, 2nd/4th/5th over the 1st) before changing any geometry.
- **What each LFA tip radiates (2026-11-25, `scratch-refs/wabl/ord/tipsplit-spike.patch`, held WOT, exhaust only)**:
  firing harmonics, 4800 rpm: the short pair H1 −3.1 dB re the sum's, H2..H4 −11.7 / −8.3 / −15.9 re its own H1
  (the muffler's inflow: H2 −10, so the short tips keep the inflow's balance, as ¶0047 says); the long tip H1 −10.2
  (its own 1st order cut, as Fig. 7B shows), H2 −10.6; the sum H2 −20.1 re H1 (Fig. 8, measured: +4). The long
  tip adds in phase at 400 Hz and against the short pair at 800 Hz (sum −5 dB under the pair). At 7500 rpm the
  pair and the long tip are ≈ 117° apart at H1 (625 Hz): the sum 5–6 dB under their in-phase sum, the "low-pass"
  of the main muffler. The patent's sum cuts the 1st order and keeps the 2nd/4th/5th over it: the long tip
  against the pair at 400 Hz and with it at 800 Hz, a path difference ≈ c/(2·400 Hz) ≈ 0.7 m at 780 K, where the
  model has 0.215 m (tip lengths 0.13 / 0.345 m from Fig. 1's proportions, L₂ ≈ 2.9·L₁, scaled to tips first taken
  as 60 mm: an ESTIMATE).
- **The LFA's tips' lengths fitted (2026-11-25, user: scale of Fig. 1's lengths)**: exhaust alone at 4800 rpm,
  firing H2 / H4 / H5 re H1 against Fig. 8's measured +4 / +8 / +1, by scale of the tips' lengths (0.13 / 0.12 wool +
  0.225 m at ×1): ×1 −20.7 / −21.2 / −23.4 (rms 26), ×1.5 rms 19.4, ×1.75 +0.3 / −9.9 / −10.6 (12.5), ×2 −3.2 /
  −11.3 / −13.0 (14.4), ×2.25 15.6, ×2.5 14.4, ×3 17.3, ×4 22; the 1st order falls 11–14 dB at ×1.75–2 (the long tip
  against the short pair at 400 Hz). Held ear J (wind ≤ −22.4, bank ≥ +13.4, 5500–8500 rpm) at 70 mm: ×1 142, ×1.5
  135, ×1.75 162, ×2 116, ×2.5 130, ×3 126; at ×2 the bore again: 65 mm 153, 70 116, 76 119, 80 113 (5500 / 6000
  wind −22.3 / −19.2, inside the real −19…−25). Taken: ×2 (short tips 0.26 m, long 0.24 wool + 0.45 m), 80 mm (the
  range's top). Dyno 529 hp at 8000 (527 before). Gates (`s2`, LFA, against `t70`): ear wind −10.2 (−5.9), bank
  +4.6 (+4.3), lift +6.8 (+1.9), peak +14.7 (+16.5); physgate crescendo +0.6 (−3.7), load −1.1 (+0.7); spectral
  low-mid −16.9 (bound −18.4), treble −26.7 PASS; blind2 hnr over at bins 2–4 and echo 22.3 > 19.8 at bin 3 (the
  long tip's 1.2 ms against the short pair, physgate's comb ripple 1.7 → 3.8 dB p-p, still a PASS).
- **The LFA's load contrast across speed, and the held overrun's cold pipes (2026-11-25, `scratch-refs/ovr/load`,
  `abs_probe` with THR / WARM, all sources, 3 m)**: full over closed throttle +9.7 / +9.4 / +4.6 / +2.8 / −1.2 /
  −2.7 / −3.8 / +2.3 / +0.3 dB(A) at 5000 / 5500 / 6000 / 6500 / 6800 / 7000 / 7500 / 8000 / 8500 rpm. Full load
  falls from 86.4 dB(A) at 5000 to 78.4 at 7500 (the tone's hole, the crescendo's failure too) while overrun
  rises 72 → 82. Overrun at 7500 is no synthesized source (each off: ±0.1 dB(A)) and no firing (motored peaks
  298–305 kPa, ±2 % between cylinders): the exhaust's own lines at cycle orders 4 / 12 / 20 / 19 / 11 (79.3 /
  77.3 / 74.5 / 74.0 / 72.8 dB) over the firing order (68.4): each cylinder's reverse rush follows the waves at
  its port, so the cylinders differ. A held closed-throttle render settles its pipes on overrun's own cold
  steady state; after 1.5 s at full load (a real decel follows a pull) the span reads +3.2 / +2.3 / −0.7 / +1.0
  at 6000 / 6800 / 7500 / 8000 (cold-settled +4.6 / −1.2 / −3.8 / +2.3).
- **The LFA's full-load "hole" is a flat H1 with weak upper orders, plus an H3 spike at 5000 (2026-11-25,
  `scratch-refs/ovr/hole`, `abs_probe`, held WOT, tips 80 mm ×2, Reference)**: exhaust alone 80.2 / 84.0 / 77.2 /
  76.5 / 76.2 / 79.6 dB(A) at 4800 / 5000 / 6000 / 6800 / 7500 / 8000 rpm, with the intake on ±0.5 (2.0 at 7500): the
  fall is the exhaust's, not intake–exhaust interference at the ear. Unweighted H1 stays 74–79 dB throughout and
  carries the total from 6000 up; H2–H5 sit −8…−40 dB under it (Fig. 8 at 4800: +4 / ? / +8 / +1; ours −2.4 / +4.2 /
  −12.4 / −12.1, rms 14.5 dB). The 5000 rpm peak is H3 (1250 Hz, cycle order 30, phase-locked) at +4 dB over H1:
  primaries 0.66 m or the long tip's pipe 0.55 m take 4 dB off it, short tips 0.32 m nothing. Element by element at
  5000 / 6000 / 7500: the published-loss final chamber (`Measured`) is a flat −20 dB (no shape); the long tip's wool
  holds H1 up 5 dB; the closed (low-speed) path in parallel is a Herschel–Quincke branch notching H2 at 5000 rpm
  (833 Hz: −15.4 re H1, +1.9 with its pipes choked to 8 mm; −2.4 at 4800), and its two inter-chamber pipes cut to
  0.02 m (the patent's perforated partitions 20A / 22A, ¶0034) move nothing at 4800 / 6000 / 7500.
- **Where the LFA's upper orders go (2026-11-25, same renders, 7500 / 4800 rpm, H2..H8 re H1)**: real dyno clip
  at 6600 −9.6 / −7.6 / −12.1 / −9.7 / −13.9 / −20.7 / −22.0; model −7.6 / −15.3 / −22.1 / −27.6 / −42.9 / −42.9 /
  −50.4 at 7500. A straight 70 mm pipe after the collectors (no catalyst, silencers or merge) −9.6 / −4.7 / −10.5 /
  −17.1 / −16.1 / −20.3 / −25.8, H1 35 dB up: the source has them. Without the sub-mufflers little changes at 7500
  (−6 dB at H4/H5 at 4800); without the catalysts −5.1 / −5.0 / −14.9 / −16.7 / −16.7 / −24.3 / −31.0, H1 +5.3
  (+6.6 at 4800), +8.6 dB(A). The catalyst alone (`catalyst_brick`, 900 K, between 70 mm pipes) loses 6.3 / 7.4 /
  6.4 / 8.0 / 9.3 / 8.7 dB at 625…3750 Hz, as narrow-tube theory gives (channels 3.0 → 5.9 dB over 0.13 m); an
  empty chamber of its size in its place takes the same in situ (H1 76.2 vs 76.0, H6 −39.5 vs −42.9): the loss is
  the 153 mm body's reflection against the source, not the monolith. The waveguide model shows the catalyst's
  insertion loss 9–14 dB at H3–H7; the FV path 10–31 dB (FV without catalyst carries more upper orders than the
  waveguide: steepening).
- **The LFA's primaries refit with the crescendo (2026-11-25, tips 80 mm ×2, `scratch-refs/wabl/fit/grid_x2.py`,
  held WOT, Reference, all sources at the ear)**: 0.60 / 0.70 / 0.80 / 0.90 m — ear J 113 / 149 / 81 / 79;
  dB(A) slope 4500–9000 rpm +1.5 / +1.4 / +10.3 / +8.7 per octave, mean fall under the running maximum 5.8 / 8.4 /
  4.7 / 3.4 (physgate: ≥ 3 and ≤ 2; every length is loud at 4500, 84–87 dB(A), and 11 dB down at 5500); dyno peak
  529 / – / 533 / 529 hp; Fig. 8 rms at 4800 14.5 / – / 22.5 / 14.5 dB. The firing line's share of 100 Hz–4 kHz
  at 5500 / 6000 / 6500 / 7000 / 7500 / 8000 (real exterior: −2.4…−3.0, flat): 0.60 m −1.6 / −0.9 / −2.1 / −7.3 /
  −3.5 / −1.4; 0.80 m −4.8 / −10.2 / −10.9 / −26.6 / −8.6 / −1.3; 0.90 m −2.3 / −9.6 / −16.3 / −4.1 / −5.3 / −2.5:
  the shut primaries' third zero walks through the band for every length in the range, so the recordings' steady
  line keeps 0.60 m (Okamoto et al. 2011 call the manifold "long branch", length not given; its catalysts are
  metal-substrate with fewer cells). No approved dimension fixes the crescendo without breaking the firing line.
  The model's zero is −30 dB deep with lossless junctions; the real line shows none from 5500 to 8000.
