# Duct Flow Noise Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every engine's exhaust gets broadband flow noise at its abrupt area expansions, at
W = K·ρ·A_j·U⁴/c (U⁶/c³ above the duct's cut-on), with one universal K calibrated on the real
recordings, so the treble is a near-steady hiss instead of a burst per firing.

**Architecture:** A per-site source `DuctJet` in `jet_noise.rs` (the `ValveJet` pattern: seeded
white noise through Tam's `FineScale`, normalized to a level law). The sites come from the
`ExhaustDef` by one pure function, so the waveguide net and the FV model agree on them. The
noise is injected as plane waves into the waveguide `ExhaustNet` at each site: the engine's own
net in waveguide mode, the existing `jet_net` (linear perturbation net the valve jets already
ride) in FV mode. Each site's instantaneous mass flow comes from its own net (waveguide) or from
the FV cells (FV).

**Tech Stack:** Rust (engine-core), Python 3 + numpy for the calibration scripts.

**Spec:** `docs/superpowers/specs/2026-11-05-duct-flow-noise-design.md`

## Global Constraints

- One constant `DUCT_FLOW_K` for every engine; no per-preset or per-engine knob.
- Level law: W = K·ρ·A_j·U⁴/c below f_co = 1.84·c/(π·D_down); above f_co the spectral density
  is multiplied by (M/0.3)², M = U/c, crossfaded linearly in log f over one octave from f_co
  (continuous at f_co). Source: Davies & Holland 1999, JSV 223(3), §4.
- Spectrum: Tam's fine-scale shape (`jet_noise::FineScale`) peaking at f_F = 0.2·U/D_j
  (`PEAK_STROUHAL`), clamped to 0.05/dt like `ValveJet`.
- Injection: plane-wave pressure p_k = √(½·W·ρc/A_k) into each side k of the site (half the
  power each way).
- U ≤ 0 (no flow, or flow towards the smaller side) → no source at that site.
- `sound.duct_flow: f64`, default 1.0 (old files hear it), 0 = off and **bit-identical** to the
  code before this plan.
- Physics in `engine-core`: pure, deterministic, seeded RNG, f64, no allocation per step, cite
  sources in doc comments.
- Heavy jobs: `taskset -c 0-5 nice -n19 scripts/heavy.sh … -j4`.
- `real_engine_sounds/` and `videos/` are local only: scripts read them, nothing from them is
  committed.

## Review Focus

1. Idle and overrun (tiny or reversed flow): the sites must go quiet, not hiss — test in Task 3.
2. Turbo engines (K20C1: `TurbineJunction` in the branch): sites downstream of the turbine still
   fire; none at the turbine itself — test in Task 3.
3. Low/Medium quality (larger dt): f_F near Nyquist must be clamped, no blow-up — test in Task 1.
4. Merged banks (`ExhaustDef::merge`, LFA) and multiple branches: each branch's sites, plus the
   merge — test in Task 2.
5. Exhaust edits rebuild the net (`adopt_exhaust_from`): sources are rebuilt with it, no stale
   site count — test in Task 3.

---

### Task 1: The source — `DuctJet` and its laws

**Files:** Modify `crates/engine-core/src/jet_noise.rs` (module doc: add the duct law and cite
Davies & Holland 1999 §4).

**Interfaces — Produces:**
- `pub const DUCT_FLOW_K: f64 = 1.0e-4;` (provisional; Task 5 replaces it)
- `pub const DUCT_U4_MACH_LIMIT: f64 = 0.3;`
- `pub fn duct_flow_power_w(k: f64, rho: f64, a_jet: f64, u: f64, c: f64) -> f64`
- `pub fn duct_cut_on_hz(c: f64, d: f64) -> f64`
- `pub fn plane_wave_rms_pa(w: f64, rho_c: f64, a: f64) -> f64`
- `pub struct DuctJet` with `pub fn new(seed: u64) -> Self` and
  `pub fn step(&mut self, u: f64, rho: f64, c: f64, d_jet: f64, d_down: f64, dt: f64) -> f64`
  returning the unit-area-free **power** signal W(t)-shaped noise as a pressure for an area of
  1 m² on each side; callers scale by `1/√A_k` (so one source serves both sides).

- [ ] **Step 1: Failing tests** (hand calculations in comments):
  - `the_duct_law_by_hand`: `duct_flow_power_w(1e-4, 0.6, 2e-3, 50.0, 500.0)` = 1.5e-3 W
    (0.6·2e-3·50⁴/500·1e-4) within 1e-12; doubling u gives ×16.
  - `cut_on_of_a_50mm_pipe_in_hot_gas`: `duct_cut_on_hz(500.0, 0.05)` = 5857.2 Hz ± 0.1.
  - `plane_wave_level_by_hand`: `plane_wave_rms_pa(1.5e-3, 400.0, 2e-3)` = √150 = 12.247 Pa.
  - `no_flow_no_duct_noise`: `DuctJet::step` with u = 0 and with u = −20 returns exactly 0.0.
  - `a_duct_jet_is_deterministic`: two `DuctJet::new(7)` give identical 10 000 samples.
  - `the_duct_jet_rms_follows_u_squared` (pressure ∝ √W ∝ U²): rms over 0.5 s at u = 40 vs 20,
    d_jet 0.05, d_down 0.02 (cut-on 14.6 kHz, so the U⁶ shelf stays out of the band), c 500,
    dt 1/48000: ratio 4.0 ± 0.4.
  - `a_coarse_step_stays_finite`: dt = 1/12000, u = 300, d_jet = 0.01: 1 s of samples all finite,
    rms < 10× the rms at dt 1/48000.
- [ ] **Step 2:** `cargo test -p engine-core jet_noise` → the new tests FAIL (not defined).
- [ ] **Step 3:** Implement. `step` normalizes like `ValveJet` (unit white through `FineScale`
  keeps 2·f_F·∫G·dt of its variance); the level is √(½·W·ρc) for 1 m², W from
  `duct_flow_power_w(DUCT_FLOW_K, …)`; above `duct_cut_on_hz(c, d_down)` apply the (M/0.3)²
  crossfade as a first-order shelf on the shaped noise (corner at f_co, gain √((M/0.3)²) above,
  reached over one octave).
- [ ] **Step 4:** `cargo test -p engine-core jet_noise` → PASS.
- [ ] **Step 5:** Commit `feat(core): duct flow noise source, U⁴/c in plane waves (Davies & Holland 1999)`.

### Task 2: The sites — from the exhaust definition

**Files:** Modify `crates/engine-core/src/exhaust.rs`.

**Interfaces — Produces:**
- `#[derive(Debug, Clone, PartialEq)] pub struct ExpansionSite { pub branch: Option<usize>, pub d_jet: f64, pub d_down: f64, pub at: SiteAt }`
- `#[derive(Debug, Clone, Copy, PartialEq)] pub enum SiteAt { Collector { primary: usize }, Merge, Segment { index: usize }, TailExit }`
- `pub fn expansion_sites(def: &ExhaustDef, cylinders: usize) -> Vec<ExpansionSite>` — order:
  for each branch in build order, its primaries' collector sites (primary order), then its
  `after_collector` expansions (segment order), then its tail exit; then the merge (if any) and
  the merged duct's own expansions and exit. A segment boundary is a site when the downstream
  passage area ≥ 1.5× the upstream one (`EXPANSION_RATIO_MIN = 1.5`); a muffler with a
  straight-through tube (`tube_diameter_m > 0`) is not a site (the flow stays in the tube); a
  catalyst's inlet is (into its shell); `Resonator` and `Measured` are not.

- [ ] **Step 1: Failing tests:**
  - `the_inline4_expands_where_by_hand`: `EngineDef::inline4().exhaust`, 4 cylinders → 4
    collector sites (d_jet = primary diameter, d_down = collector diameter), 1 catalyst inlet
    (d_jet 0.05 → d_down 0.1399), 1 tail exit (d_jet = tail diameter); the muffler volume adds a
    site only if it is an empty chamber in the preset (write the expected list by hand from
    `presets/inline4.yaml`).
  - `the_lfa_has_a_merge_site`: `EngineDef::lfa_v10()` sites contain exactly one `SiteAt::Merge`
    and per-branch collector sites for all 10 primaries.
  - `a_straight_pipe_has_only_its_exit`: a def with no collectors' area change and one
    `Segment::Pipe` of the collector's diameter → collector sites only if the areas grow ≥ 1.5×,
    plus the exit.
- [ ] **Step 2:** `cargo test -p engine-core expansion_sites` → FAIL.
- [ ] **Step 3:** Implement from the same walk `segments_duct` / `collector_losses` use (reuse
  their area helpers; do not duplicate the diameter logic of `exit_diameter`).
- [ ] **Step 4:** PASS. **Step 5:** Commit `feat(core): the exhaust's area expansions, listed from its definition`.

### Task 3: Into the waveguide net (waveguide engines and the FV jet net)

**Files:** Modify `crates/engine-core/src/waveguide.rs` (`Duct::send` gains an optional source
per inner node), `crates/engine-core/src/exhaust.rs` (`ExhaustNet` owns one `DuctJet` per site),
`crates/engine-core/src/engine.rs` (passes `sound.duct_flow` and, for waveguide, nothing else),
`crates/engine-core/src/sound.rs` (the field).

**Interfaces:**
- Consumes: Task 1 `DuctJet`, Task 2 `expansion_sites`.
- Produces: `SoundDef { pub duct_flow: f64 }` (serde default 1.0);
  `ExhaustNet::set_duct_flow(&mut self, gain: f64)`;
  `ExhaustNet::set_site_flows(&mut self, flows: &[f64])` — instantaneous mass flow per site,
  kg/s, + towards the tail, in `expansion_sites` order; when never called the net uses its own:
  mean flow (`Duct::mean_now`, the branch's `mean_flows`) + the acoustic flow from its waves at
  the site, ṁ = A·(p⁺ − p⁻)/c on the upstream side.
- Collector sites inject at the branch junction into the collector side and the primary; segment
  sites at their `Duct` node; the tail exit adds its pressure to the tail's outgoing wave before
  `OpenEnd` (it radiates through the existing end).

- [ ] **Step 1: Failing tests:**
  - `sound.rs`: `the_duct_flow_round_trips_and_old_files_hear_it` (JSON and YAML; a file without
    the key → 1.0).
  - `exhaust.rs`: `duct_flow_off_is_the_old_net` — an inline-4 net stepped 0.2 s with
    `set_duct_flow(0.0)` gives samples bit-equal to a net built without the feature (capture the
    hash on the commit before Task 3, as `1b26c12` did for the reverted attempt).
  - `exhaust.rs`: `an_expansion_with_flow_hisses` — inline-4 net, steady flows 0.03 kg/s per
    primary: 1–5 kHz energy of the radiated output with gain 1 ≥ 3 dB above gain 0.
  - `exhaust.rs`: `no_flow_no_hiss` — flows 0: gain 1 output bit-equal to gain 0.
  - `exhaust.rs`: `reversed_flow_is_silent_at_that_site` — `set_site_flows` with all −0.02:
    bit-equal to gain 0.
  - `engine.rs`: `a_turbo_engine_keeps_its_downstream_sites` — `EngineDef::honda_k20c1()` (or
    the turbo preset's constructor) builds without panic, its net's site count equals
    `expansion_sites(...).len()`, and 0.3 s held at 3000 rpm WOT is finite.
  - `engine.rs`: `an_exhaust_edit_rebuilds_the_sites` — after changing `after_collector` on a
    running engine (the `adopt_exhaust_from` path), the net's site count equals the new
    `expansion_sites` length.
- [ ] **Step 2:** `cargo test -p engine-core duct_flow` (and the named tests) → FAIL.
- [ ] **Step 3:** Implement. No allocation per step (sources and site buffers sized at build).
- [ ] **Step 4:** PASS; then `cargo test -p engine-core` whole crate: any render-hash test that
  now differs is re-captured **on purpose** in its own commit, saying which and why.
- [ ] **Step 5:** Commit `feat(core): duct flow noise at the exhaust's expansions (sound.duct_flow)`.

### Task 4: FV engines — site flows from the cells

**Files:** Modify `crates/engine-core/src/exhaust_fv.rs`, `crates/engine-core/src/engine.rs`.

**Interfaces:** Produces `ExhaustFv::site_flows(&self, out: &mut Vec<f64>)` — mass flow at each
`expansion_sites` site (the face of the expansion's upstream cell), kg/s, + towards the tail,
same order. The engine calls it each step in FV mode and hands it to
`jet_net.set_site_flows(..)` and `jet_net.set_duct_flow(def.sound.duct_flow)`.

- [ ] **Step 1: Failing tests:** `the_fv_site_flows_follow_the_cells` — FV inline-4 held 3000
  WOT 0.2 s: `site_flows` has `expansion_sites().len()` entries, the collector sites' cycle mean
  is within 10 % of the cycle-mean exhaust mass flow per cylinder (kg/s, from the engine's own
  port flows), the tail exit's within 10 % of the engine total; and
  `the_fv_engines_hiss_at_their_expansions_too` — `EngineDef::honda_k20a()` (FV) 0.3 s at 3000
  WOT, 1–5 kHz of the output with `duct_flow` 1 ≥ 3 dB above 0, and 0 bit-equal to the old code.
- [ ] **Step 2:** FAIL. **Step 3:** Implement. **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat(core): FV engines feed their expansions' flow to the duct noise`.

### Task 5: Calibrate K once, for all engines

**Files:** Create `scripts/realism/modulation.py` (the firing-rhythm envelope modulation of
0.3–0.8/0.8–2/2–5/5–12 kHz and the band balance 300–1k/1–3k/3–8k re 50–300 Hz, for WAVs or
the steady segments in `real_engine_sounds/segmentos/*estable.wav`; the method of
`docs/agents/debugging.md` → *Sound realism*), `scripts/realism/calibrate-duct-flow.sh`
(renders every preset `--render held:R:L:3` at (1500, 0.1), (3000, 0.3), (6000, 1.0) with
`sound.duct_flow` = g for g in 0, 0.3, 1, 3, 10, 30, writes a table). Modify
`crates/engine-core/src/jet_noise.rs` (`DUCT_FLOW_K` = 1e-4·g*², the sweep's best g*),
`docs/FACTS.md`, `docs/ROADMAP.md` (this item under *Now*), `CHANGELOG.md` at merge.

- [ ] **Step 1:** Self-test of `modulation.py`: a 4-cylinder-like test signal (noise × (1 + m·cos
  2π·100t)) gives m within ±2 % for m = 0.1 and 0.5 (`python3 scripts/realism/modulation.py --self-test`). Run it and see it fail first with a deliberately wrong band before fixing.
- [ ] **Step 2:** Run the real segments → record median and p25/p75 per band (expect ≈ 12 %
  median in 0.8–5 kHz; FACTS has the survey).
- [ ] **Step 3:** Run the sweep. Pick g* where the presets' median 0.8–5 kHz modulation equals
  the real median. **Stop rule:** if at g* any preset at any point is outside the real p25–p75,
  or the band balance leaves the real p25–p75, stop and report (spec: go to option 2).
- [ ] **Step 4:** Set `DUCT_FLOW_K`; re-run Tasks 1–4 tests (hash tests re-captured on purpose);
  FACTS entry (K, date, medians, the per-preset table).
- [ ] **Step 5:** Commit `feat(core): duct flow constant calibrated once on 209 real recordings`.

### Task 6: Ear, cost, PR

- [ ] **Step 1:** Loudness-matched before/after clips (`--render held` and `--render drive`) of
  inline-4, K20A and LFA; send to the user and **wait** for the verdict (the LFA must not get
  worse).
- [ ] **Step 2:** Real-time cost: Ultra realtime factor before/after on the LFA and inline-4
  (`rtf_probe` example, under `scripts/cpu-lock.sh`); no preset may lose real time at Ultra.
- [ ] **Step 3:** Full local gate (fmt, clippy, workspace tests, coverage ≥ 80 %, smoke), PR
  with *How to test manually*, verify-pr comment, background in-diff mutation run.
