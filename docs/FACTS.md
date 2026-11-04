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
