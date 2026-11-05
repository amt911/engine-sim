# Measured Source Levels — Part A Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (the user's choice for this work) to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Each noise source of every preset at the level production engines of its kind measure:
structure (block + mechanical) per Lalor/Raff, intake with its air cleaner, exhaust silenced as
built — no per-preset knob.

**Architecture:** Mechanical impacts (main-bearing load reversals, piston side-thrust reversals)
timed by the sim's own forces enter the existing structure path as pseudo-pressures (the valve
seat precedent, `engine.rs` seat strikes); their amplitude per impact is set so the mechanical
path's A-weighted level at 1 m follows the measured petrol curve. The 0D intake's mouth gets a
lumped air cleaner (box + duct + filter resistance) as a linear filter; the FV intake gets the
filter resistance in its air box. Presets with no silencers get the rule-sized road exhaust.

**Tech Stack:** Rust (engine-core), Python 3 + numpy (scripts/realism).

**Spec:** `docs/superpowers/specs/2026-11-05-measured-source-levels-design.md` (Part A).

## Global Constraints

- Every level from a cited measured curve or physics constant; no per-preset constant.
- Anchors (spec table): structure 1 m, petrol, full load: 88–92 dB(A) at 3000 rpm, 98–105 at
  5000–6000 (Lalor 1980 SAE 800402 Fig. 7; Raff & Perry 1973 Fig. 1), ≈ 75–84 at 2000; slopes
  ≈ 13 dB/decade below the critical speed, 50–60 above (Lalor eqs. 12–13). Exhaust orifice
  0.25 m 45°: 104–110 dB Lin (Davies & Holland 1999 Fig. 1c). Air cleaner insertion ≈ 8.5 dB(A)
  (Raff Fig. 14).
- New sound sources default **on** at their physical level (1.0), with a `sound.*` mix field each
  (serde default, JSON/YAML round trip).
- Petrol idle is not a target; the LFA orders guard (≤ 10.9 dB RMS, `engine-app` sim tests) must
  hold.
- Hot loop: no allocation per step; Ultra stays real time (measure).
- Heavy jobs: `taskset -c 0-5 nice -n19 scripts/heavy.sh … -j4`.

## Review Focus

1. Rotary engines (no pistons, no main-bearing reversals of the same kind): mechanical impacts
   must be skipped as the seat strikes are — test in Task 2.
2. Diesels: Anderton puts them ≈ 15 dB(A) above petrol; the mechanical anchor here is petrol
   only — the diesel presets must not get a petrol curve wrongly (test in Task 2: a diesel keeps
   its combustion-dominated level within ±1 dB of before at 2000 rpm).
3. Engine-off / stall (rpm → 0): no impacts, no NaN — test in Task 2.
4. Turbo engines with the FV intake (K20C1): the air cleaner sits before the compressor — test
   in Task 3.
5. Exhaust edits (adopt path): presets' new road exhausts survive `--save` round trips — the
   preset-equality test in Task 4.

---

### Task 0: Measurement tooling on main

**Files:** Create `scripts/realism/modulation.py`, `scripts/realism/levels.py` (from branch
`feat/duct-flow-u4`, unchanged), `scripts/realism/source_levels.py`.

**Produces:** `source_levels.py BINARY PRESET.yaml RPM LOAD` renders the preset held with each
source alone (exhaust = pulses + valve jets + tail jets; intake; block; mechanical; and each new
source) and prints dB Lin and dB(A) at the listener and converted to 1 m (structure: 20·log(3/1))
and 0.25 m (exhaust orifice: 20·log(3/0.25)), next to the anchors.

- [ ] Step 1: `python3 scripts/realism/modulation.py --self-test` passes; a sabotaged copy (scale
  /2) fails.
- [ ] Step 2: `source_levels.py` self-check: a WAV of a 1 kHz sine of 1 Pa rms at 20 Pa full
  scale reads 94.0 dB Lin ± 0.1 and 94.0 dB(A) ± 0.2 (A at 1 kHz = 0 dB).
- [ ] Step 3: PR "chore(scripts): realism measures", merged on its own (docs/tooling only).

### Task 1: Mechanical structure-noise level law (pure functions)

**Files:** Create `crates/engine-core/src/mechanical.rs` (module doc citing Raff 1973 §2,
Figs 1, 6, 11; Lalor 1980 eqs. 12–13, Fig. 7).

**Produces:**
- `pub fn mechanical_dba_at_1m(rpm: f64, bore_m: f64) -> f64` — the measured petrol curve:
  two power laws meeting at the critical speed `CRITICAL_RPM` (Lalor's, read from Fig. 7),
  slope `LOW_DB_PER_DECADE` below and `HIGH_DB_PER_DECADE` above, scaled by bore as Lalor's
  75/91/98 mm series; constants marked ESTIMATE where read off a figure.
- `pub fn rms_pa_from_dba(dba: f64) -> f64`.

- [ ] Step 1 (tests, values written by hand from the figures): at 3000 rpm, 86 mm bore:
  90 ± 2 dB(A); at 6000: 101 ± 3; at 2000: 80 ± 3; a decade above the critical speed adds
  `HIGH_DB_PER_DECADE` exactly; rpm 0 → −∞ (rms 0).
- [ ] Step 2: RED (`cargo test -p engine-core --lib mechanical`).
- [ ] Step 3: implement. Step 4: GREEN. Step 5: commit.

### Task 2: Mechanical impacts into the structure path

**Files:** Modify `crates/engine-core/src/engine.rs` (per-cylinder loop beside the seat strikes,
`:3150-3165`; the structure output `:3736-3812`), `crates/engine-core/src/mechanical.rs`,
`crates/engine-core/src/sound.rs` (`main_bearings: f64`, `piston_slap: f64`, default 1.0).

**Interfaces:**
- `pub fn piston_side_force(g: &CylinderGeometry, theta: f64, p_gauge: f64, omega: f64, m_recip: f64) -> f64`
  (Ungar & Ross 1965; the prototype `e472fb2`'s formula, its doc-comment defect fixed).
- `pub fn rod_axial_force(g: &CylinderGeometry, theta: f64, p_gauge: f64, omega: f64, m_recip: f64) -> f64`
  — gas minus reciprocating inertia along the rod; its sign changes are the main bearings' load
  reversals (Raff §2.3).
- `pub fn reciprocating_mass_kg(bore_m: f64) -> f64` — `RECIP_MASS_PER_BORE3` (ESTIMATE, the
  prototype's 600 kg/m³ for piston + pin + small end; cite the source used).
- Impact pulse: the seat strikes' half-sine (`seat_pulse`), width `IMPACT_PULSE_S` 2e-4.
- Amplitude: every impact of the mechanical path carries the same peak pseudo-pressure a(rpm),
  set so the path's A-weighted power at 1 m equals `mechanical_dba_at_1m`: a(rpm) =
  √(P_target / (rate · E_unit)), with `E_unit` the A-weighted energy at 1 m of a unit pulse
  through the bank's `StructureResponse` (computed once at build by running the pulse through
  a copy of it) and `rate` the impacts per second counted over the last cycle.

- [ ] Step 1 (tests):
  - `the_side_force_reverses_near_tdc_and_bdc` — inline-4 geometry, motored (p_gauge 0),
    3000 rpm: sign changes within ±20° of 0° and 180°.
  - `an_impact_train_reads_the_curve` — inline-4 held 3000 rpm WOT 0.5 s with only the
    mechanical path on: A-weighted level at 1 m within ±1 dB of `mechanical_dba_at_1m(3000, 0.086)`;
    same at 6000.
  - `rotaries_have_no_piston_impacts` — Renesis preset: 0 impacts.
  - `a_stalled_engine_makes_no_impacts` — rpm 0: output exactly 0, finite.
  - `a_diesel_keeps_its_combustion_level` — TDI at 2000 rpm 30 %: total structure dB(A)
    within ±1 dB of the level with the new sources off.
  - `mixes_round_trip_and_old_files_hear_them` (sound.rs).
- [ ] Step 2: RED. Step 3: implement. Step 4: GREEN + `cargo test -p engine-core`.
- [ ] Step 5: commit.

### Task 3: The intake's air cleaner

**Files:** Modify `crates/engine-core/src/intake_fv.rs` (`AirboxDef.filter_resistance_pa_s_m`,
a linear flow resistance of the filter element at the box inlet; serde default from Task 3's
cited value), `crates/engine-core/src/engine.rs` (0D intake: the mouth radiation through a
lumped air cleaner when `airbox` is set — a Helmholtz box + duct + the filter's resistance as a
linear 2-port; presets with `airbox: null` get a rule-sized box: volume and duct from a cited
sizing source or ESTIMATE flagged).

- [ ] Step 1 (tests):
  - `an_air_cleaner_takes_off_what_raff_measured` — inline-4 at 3000 WOT, intake mouth alone:
    A-weighted level with the rule-sized cleaner 8.5 ± 3 dB below the bare mouth.
  - `a_filter_resistance_round_trips` (JSON/YAML; old files: the default).
  - `the_turbo_cleaner_sits_before_the_compressor` — K20C1: builds, finite, mouth level drops.
- [ ] Step 2: RED. Step 3: implement. Step 4: GREEN. Step 5: commit.

### Task 4: Presets silenced as built

**Files:** Modify `presets/*.yaml` and their constructors in `engine.rs` (the presets test keeps
them equal): every preset with no `muffler`/`measured` segment gets `road_exhaust(swept per
branch, its tail diameter)` after its catalyst (or its maker's silencers where `docs/FACTS.md`
or `papers/engine-data/` has them). Check the catalyst's low-frequency loss (8 dB at 200 Hz in
our model) against a published catalyst transmission loss before relying on it; if it is off,
record the finding (fix is its own task).

- [ ] Step 1 (test): `every_preset_sits_in_the_production_exhaust_range` — each preset held
  at 3000 and 6000 WOT, exhaust alone: orifice level converted to 0.25 m within 104–110 dB ± 4.
- [ ] Step 2: RED (today: the unsilenced presets ≈ 20 dB high). Step 3: edit presets.
  Step 4: GREEN; `cargo test --workspace` (presets equality, LFA guard).
- [ ] Step 5: commit.

### Task 5: Census, clips, PR

- [ ] Step 1: `source_levels.py` on inline-4, K20A, LFA, V8, TDI at 1500/10 %, 3000/30 %,
  6000/100 %: every source within ±3 dB of its anchor; table into `docs/FACTS.md`.
- [ ] Step 2: spectrograms + `modulation.py` of the same renders against the real segments and
  Ange's demos (`docs/agents/debugging.md` → *Sound realism*); clips (loudness-matched) of
  inline-4, K20A, LFA before/after to the user for a go/no-go.
- [ ] Step 3: Ultra real-time check (`rtf_probe`, under `scripts/cpu-lock.sh`); full local gate;
  PR with *How to test manually*; verify-pr comment; background in-diff mutation run.
