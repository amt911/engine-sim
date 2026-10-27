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
measured TL falls (beaming) where any 1D model keeps rising. Check: `papers/dl-silencer/compare_lining.py`
(local). Plan: the silencer's two-port from a 2D axisymmetric mode-matching model (Selamet, Xu, Lee, JASA 117,
2005), realised in the time domain by a rational fit of its scattering matrix (vector fitting, Gustavsen &
Semlyen 1999) — a few biquads per silencer.
- 2D mode-matching prototype (`papers/dl-silencer/modal2d.py`, local; Selamet–Xu–Lee 2005 re-derived in e^{jωt},
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
