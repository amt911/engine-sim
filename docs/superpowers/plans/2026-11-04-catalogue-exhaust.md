# Catalogue, PR 1: exhaust headers and systems — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** choose the exhaust as a complete system with one click — headers (single, 4-1, 4-2-1,
log; per bank) and a system after them (road, sport, race) — sized from the engine, re-sized
when the engine changes, with out-of-range warnings.

**Architecture:** a pure module `engine-core::catalogue`. It holds:

- the sizing rules;
- a survey of the repo's real-engine presets that calibrates the rules' two empirical numbers
  (the mean gas speed in a pipe at the torque peak, and the wave-tuning constant);
- the `Systems` record on `EngineDef`, which remembers each choice and its hand edits.

`EngineDef::set_cylinders` and the editor's geometry edits call a re-size. The editor gets an
"Exhaust system" section.

**Tech Stack:** Rust, `engine-core` (pure, f64), `engine-app` editor (egui). No new crates.

**Spec:** `docs/superpowers/specs/2026-11-04-parts-catalogue-design.md`.

## Global Constraints

- Every rule cited, with the source named in the doc comment.
  - Empirical numbers come from the survey of the presets' real cars (each preset cites its
    car). Never from taste.
  - Class targets with no source (the road silencer's attenuation) are `ESTIMATE:` with their
    reasoning, and confirmed by the user's ear.
- Tuned rpm `N_t`: 0.75 × redline for road and sport, 0.85 × redline for race (spec).
- `systems: Option<Systems>` with `#[serde(default, skip_serializing_if = "Option::is_none")]`:
  - no format version bump;
  - presets keep their measured exhausts and their YAML is unchanged;
  - round-trip test in JSON and YAML.
- A hand edit to a measure of a chosen system is kept on re-size and still range-checked.
  "Re-size all" releases them.
- Warnings never block an edit.
- The measured check, single at 4 cylinders + road, against the inline-4:
  - tolerance: centroid within 50 Hz, level within 1.5 dB;
  - if it misses, do not tune: report the miss to the user.

## Review Focus

1. A single (1 cylinder) offered only "single" headers; a 4-1 on 3 cylinders is a 3-1; a V8 or
   flat four gets headers per bank (`layout.banks`), its collectors = its banks.
2. A turbo engine: the system starts at the turbine outlet, and the headers feed the turbine
   (the existing exhaust/turbo coupling). Sizing must not break `config::validate`.
3. A diesel (redline ~4500): the tuned rpm and the lengths come out long. Clamp each measure to
   the editor's slider range, flagged if clamped.
4. Re-size after loading a file that has `systems` but whose measures were hand-edited in
   another version: the kept set is stored by measure name, so unknown names are ignored, not
   panicked on.
5. Changing cylinders 1 → 4 → 1 with a system chosen returns the same measures as the first
   sizing (idempotent).

---

### Task 1: Survey of the real engines (the two empirical numbers)

**Files:** Create `crates/engine-core/src/catalogue.rs` (+ `pub mod catalogue;`).

**Interfaces:**
- `pub struct Survey { pub v_primary: f64, pub v_tail: f64, pub tuning_k: f64, pub spread: (f64, f64) }`
- `pub fn survey() -> Survey`, from the presets whose exhausts are the real cars':
  - K20A, K20C1, LFA V10, EJ25, flat-plane V8, cross-plane V8 (each preset documents its source);
  - not the single (race, invented), the diesels, nor the rotaries.
- Mean gas speed at the torque peak in a pipe of area A carrying the flow of `n` cylinders:
  `v = n·V_d·η_v·(N_t/120)·(T_exh/T_in) / A`.
  - `V_d`: swept volume per cylinder.
  - `η_v`: the engine's volumetric efficiency at `N_t` from a short `--dyno`-like hold (reuse
    `dyno` code), or 0.9 if that is too heavy for a unit test. In that case it is `ESTIMATE:`,
    cite Heywood Fig. 7-1.
  - `T_exh/T_in`: 1000/300, the model's own `C_PRIMARY` gas (ESTIMATE, the same as the sim).
- Wave-tuning constant: the tuned length `L = k·c·θ_ED/(12·N_t)`, with:
  - `θ_ED` = 180 + exhaust-valve opening before BDC, from the valve's `open_deg`;
  - `c = C_PRIMARY`;
  - the half round trip of the blowdown pulse during θ_ED. Heywood §7.8 and Blair ch. 6 give the
    law's form, not its constant; `k` is that constant.

  Fit `k` as the median of `L·12·N_t/(c·θ_ED)` over the surveyed engines.
- `spread`: the min and max of each ratio over the survey: the "real engines" range used by the
  warnings.

- [ ] **Step 1: failing tests**
  - `the_survey_reads_each_real_engine`: survey over the six presets returns finite numbers, the
    K20A's own primary velocity computed by hand in the comment (its 4 primaries, its bore,
    stroke, diameter, redline 8600 → N_t = 6450), within 1 %.
  - `the_survey_spread_holds_every_surveyed_engine`.
- [ ] **Step 2: FAIL. Step 3: implement. Step 4: PASS**, then sabotage (drop the T ratio): the
  hand-value test fails.
- [ ] **Step 5: commit** `feat(core): survey the real engines for the catalogue's two numbers`.

### Task 2: The sizing rules

**Interfaces:**
- `pub enum Headers { Single, FourOne, FourTwoOne, Log }`;
  `pub enum System { Road, Sport, Race }`.
- `pub fn tuned_rpm(redline: f64, system: System) -> f64`: 0.75 or 0.85 (spec).
- `pub fn size_exhaust(def: &EngineDef, h: Headers, s: System, sv: &Survey) -> ExhaustDef`.
  Starts from `def.exhaust` (keeps `model`, `steepening`, `gas_temperature`) and sets:
  - **primaries:**
    - diameter from `v_primary` with n = 1;
    - length from the tuning law with `k`;
    - FourTwoOne's secondaries as `collector_extra_length_m`, tuned to the second pulse;
  - **collectors:**
    - per bank, `collectors = layout.banks` (spec);
    - diameter from `v_primary` for the bank's cylinder count (the merged flow);
    - length 0.15 m (ESTIMATE: a merge's own length; the secondaries carry the tuning);
    - Log: one short collector of the bank's runners, 0.3 × the primary length;
  - **system:**
    - tail diameter from `v_tail` for the whole engine;
    - Road: `after_collector` = catalyst (400 cpsi, sized to the tail's area × 4) + two
      silencers;
    - Sport: catalyst + one straight-through packed silencer;
    - Race: open pipe;
  - **silencers:** expansion chambers sized by Munjal's TL, ch. 3:
    `TL = 10·log10(1 + ¼(m − 1/m)²·sin²kL)`.
    - Length gives the first TL peak at the firing frequency at `N_t`: `kL = π/2`.
    - The area ratio `m` gives the target TL there: Road 20 dB, Sport 10 dB.
      `ESTIMATE:` road cars meet a drive-by limit about 20 dB under an open pipe; confirm by ear.
- Every measure clamped to the editor's slider range (the editor exposes these ranges; move them
  to `engine-core::config` if they live only in `editor.rs`, and say so in the PR).

- [ ] **Step 1: failing tests (hand values)**
  - `a_two_litre_four_gets_its_primary_by_continuity`: inline-4 2.0 L at N_t; the primary
    diameter from the survey's `v_primary` worked by hand in the comment.
  - `the_tuning_law_lengthens_primaries_for_a_slower_engine`: N_t halved gives the length
    doubled (exact, the law's form).
  - `munjals_chamber_gives_its_target_loss`:
    - the road silencer's TL at the firing frequency is 20 dB (±0.5), from the formula with the
      sized m and L, by hand;
    - its length is λ/4 at that frequency.
  - `every_choice_is_a_valid_engine` (proptest over cylinders 1..=12, layouts inline/V/flat,
    redline 4000..=9000, each Headers × System): `config::validate` ok, every measure inside
    the slider range.
  - `a_single_gets_single_headers` and `a_v8_gets_headers_per_bank` (Review Focus 1).
  - `a_turbo_engine_keeps_its_turbine` (Review Focus 2).
- [ ] **Step 2: FAIL. 3: implement. 4: PASS**, then sabotage each rule once.
- [ ] **Step 5: commit** `feat(core): size an exhaust from the engine (catalogue rules)`.

### Task 3: The remembered choice and the re-size

**Interfaces:**
- `pub struct Systems { pub exhaust: Option<ExhaustChoice> }`;
  `pub struct ExhaustChoice { pub headers: Headers, pub system: System, pub kept: Vec<String> }`.
  Measure names are the `ExhaustDef` field names, e.g. `"collector_diameter_m"`.
- `EngineDef.systems: Option<Systems>` (serde as above).
- `pub fn choose_exhaust(def: &mut EngineDef, h: Headers, s: System)` sizes, stores the choice
  and clears `kept`.
- `pub fn resize(def: &mut EngineDef)` re-sizes each chosen system, keeping the `kept` measures
  as they are.
- `pub fn note_hand_edit(def: &mut EngineDef, before: &EngineDef)` adds to `kept` every exhaust
  measure that differs from what the sizing gives.
- `pub fn out_of_range(def: &EngineDef) -> Vec<Warning>`, each warning carrying the measure,
  its value and the survey range scaled to this engine.
- `EngineDef::set_cylinders` calls `resize` at the end.

- [ ] **Step 1: failing tests**
  - Round trip with and without `systems` in JSON and YAML; every preset YAML unchanged.
  - `changing_cylinders_resizes_and_keeps_hand_edits`:
    - single → 4 cylinders with Road chosen gives sized measures;
    - a hand-edited `tail_length_m` stays;
    - 4 → 1 → 4 gives the same measures (Review Focus 5).
  - `a_kept_edit_out_of_range_is_flagged`: the 40 mm collector kept, 4 cylinders → a warning
    names it.
  - `unknown_kept_names_are_ignored` (Review Focus 4).
- [ ] **Step 2–4: FAIL, implement, PASS, sabotage. Step 5: commit.**

### Task 4: The editor

**Files:** `crates/engine-app/src/editor.rs`.

- An "Exhaust system" section above the exhaust's sliders, containing:
  - the header buttons (only the ones this engine can take);
  - the system buttons (Road / Sport / Race);
  - the current choice and the line "re-sized for <cylinders> × <cc>" after a re-size;
  - "Re-size all" (clears `kept`);
  - the warnings in amber under the measures they name.
- A slider edit on an exhaust measure calls `note_hand_edit`.
- [ ] **Step 1: failing kittest** `choosing_road_exhaust_on_a_four_cylinder_single`: the single
  preset, cylinders → 4 (nudge), click "Road": `after_collector` has a catalyst and two
  silencers, the collector diameter is the sized one; cylinders → 3 re-sizes.
- [ ] **Steps 2–5** as above. Run `egui-review` on the change.

### Task 5: The measured check and the user's ear

- [ ] Test `the_four_cylinder_single_with_a_road_exhaust_sounds_like_the_inline4` (engine-app,
  release-ignored like the other render checks):
  - render 3 s at 3000 rpm, 50 % throttle, as the reproduction did;
  - centroid within 50 Hz of the inline-4's, level within 1.5 dB.
  - **If it misses: stop, record the numbers and bring them to the user. Do not tune.**
- [ ] Clips for the user, loudness-matched: single at 4 cylinders as it was, with Road, with
  Sport, with Race, and the inline-4. Wait for the verdict (ear over spectrum).
- [ ] PR:
  - full gate and verify-pr;
  - the mutation run in the background;
  - ROADMAP, CHANGELOG and tag after merge.

Not in this PR: intake, turbo (their own PRs per the spec).
