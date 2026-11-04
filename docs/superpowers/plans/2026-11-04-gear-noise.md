# Gear Noise Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the manual gearbox is heard: the engaged pair's and the final drive's whine at their mesh orders, and the loose gears' rattle at idle and low load.

**Architecture:** a new pure module `engine-core::gear_noise`:

- **Whine:** each mesh is a one-degree-of-freedom gear pair whose stiffness follows the
  instantaneous contact length. Its fluctuating mesh force drives the housing wall, which
  radiates, below coincidence, as a point source.
- **Rattle:** each loose gear is an inertia in its backlash, driven by the input shaft's speed
  fluctuation. Its impacts go through the same radiation.
- **Wiring:** the vehicle (in `engine-app::sim`) hands the engine a `DriveState` each step. The
  engine mixes the result at the block's position with a `sound.gear_noise` level.

**Tech Stack:** Rust, `engine-core` (f64, deterministic), no new crates.

**Spec:** `docs/superpowers/specs/2026-11-04-transmissions-hybrid-design.md` (section "Transmission noise").

## Global Constraints

- Physics first: every constant cited or marked ESTIMATE with its reasoning in the doc comment; no level set by ear, no fitted gain.
- `engine-core` stays pure: no IO, no RNG, f64.
- Every new `serde` field has a default (old files load); round-trip test JSON and YAML.
- Every new test is seen failing once (sabotage) before it counts.
- Anti-goal (spec): no sampled or hand-shaped gear sound; every tone at its mesh order, every rattle an impact of the modelled gear.
- Hot loop: measure `--headless` time before/after on the LFA at High; the gearbox adds two 1-DOF integrations plus one per loose gear per step.

## Review Focus

1. Neutral with the clutch pedal up at idle: the classic neutral rattle must appear (input shaft follows the crank); with the pedal down it must stop.
2. Gear 0 or the clutch slipping: no whine, no NaN; whine fades with the transmitted torque.
3. A mesh frequency above Nyquist (high gear tooth count at 9000 rpm on a 48 kHz device) must not alias; it is silent above Nyquist (as `CompressorTone`).
4. A vehicle file without `gearbox` loads with the default box; a preset YAML round-trips unchanged.
5. A gear ratio near 1 or below 0.5 (overdrive) still yields valid tooth counts (each ≥ 12) and a positive contact ratio.

---

### Task 1: Gear geometry — tooth counts, contact ratios, contact length

**Files:**
- Create: `crates/engine-core/src/gear_noise.rs` (+ `pub mod gear_noise;` in `lib.rs`)

**Interfaces:**
- Produces:
  - `pub struct Mesh { pub z1: u32, pub z2: u32, pub module_m: f64, pub helix_rad: f64, pub face_m: f64 }`
  - `impl Mesh { pub fn eps_alpha(&self) -> f64; pub fn eps_beta(&self) -> f64; pub fn base_radius_m(&self, which: usize) -> f64 }`
  - `pub fn contact_length(t: f64, eps_alpha: f64, eps_beta: f64) -> f64` (in face widths, `t` in base pitches, periodic in 1)
  - `pub fn teeth_for(ratio: f64, sum: u32) -> (u32, u32)`

- [ ] **Step 1: Failing tests**
  - `a_spur_pairs_contact_ratio_by_hand`:
    - Mesh 20/20, m = 1 mm, β = 0, standard 20° full-depth: `eps_alpha` ≈ 1.557 (±0.001).
    - Hand calculation: r_a = 11, r_b = 9.3969; 2·√(121 − 88.302) = 11.436; minus a·sin α = 6.840; divided by π·m·cos α = 2.9521. This gives 1.5569.
    - `eps_beta` = 0.
  - `a_helical_pairs_overlap_ratio_by_hand`: b = 18 mm, β = 25°, m_n = 2.5 mm. ε_β = b·sin β / (π·m_n) = 0.9686 (±0.001).
  - `contact_length_counts_pairs_on_a_spur`: ε_α = 1.5, ε_β = 0. Over t ∈ [0, 1) the value is 2 on a 0.5 share and 1 on the rest; the mean over 1000 samples is 1.5 (±0.002).
  - `an_integer_overlap_holds_the_length_constant`: ε_α = 1.5, ε_β = 1.0. `contact_length` = 1.5 (±1e-9) at t = 0.1, 0.37, 0.8.
  - `the_mean_contact_length_is_the_profile_ratio` (proptest): ε_α ∈ [1.1, 2.0], ε_β ∈ [0, 2.5]. The mean over 2000 samples of t equals ε_α (±0.01).
  - `tooth_counts_follow_the_ratio`:
    - `teeth_for(1.3, 60)` = (26, 34).
    - `teeth_for(0.75, 60)` = (34, 26).
    - For a ratio of 3.6, both counts are ≥ 12.
- [ ] **Step 2: Run** `cargo test -p engine-core gear_noise`. Expect a compile failure (the module is missing).
- [ ] **Step 3: Implement.**
  - `eps_alpha`: textbook transverse contact ratio. Cite Niemann & Winter vol. II, or ISO 21771 §5.
    - Helical: transverse pressure angle α_t = atan(tan 20° / cos β); transverse module m/cos β.
  - `contact_length` follows Maatar & Velex, *Mech. Mach. Theory* 31(5), 1996. The contact lines are parallel lines across the plane of action [0, ε_α] × face [0, 1]. Write it as the closed sum:
    ```text
    L(t) = Σ_k |{ y ∈ [0,1] : 0 ≤ t + k − ε_β·y ≤ ε_α }|,
    k from ⌊−ε_α⌋−1 to ⌈ε_α + ε_β⌉+1
    ```
  - `teeth_for`:
    - z1 = round(sum / (1 + ratio)), clamped to ≥ 12, and z2 = sum − z1;
    - ESTIMATE: a car box's pairs share one centre distance, so each pair keeps the same tooth sum.
- [ ] **Step 4: Run the tests.** Expect PASS. Sabotage `eps_beta` (drop the sin): the overlap test must fail. Restore it.
- [ ] **Step 5: Commit** `feat(core): gear mesh geometry: contact ratios and contact length`.

### Task 2: The whine of one mesh

**Files:** Modify `crates/engine-core/src/gear_noise.rs`

**Interfaces:**
- Consumes: Task 1's `Mesh` and `contact_length`.
- Produces:
  - `pub fn housing_pressure_pa(force_n: f64, wall_m: f64, r_m: f64) -> f64`: p = ρ₀·F / (2π·r·m″), with m″ = 2700·wall_m.
    - Cremer, Heckl & Petersson, *Structure-Borne Sound* 3rd ed.: below coincidence, a point-excited plate radiates as a baffled monopole of volume acceleration F/m″.
    - Aluminium housing, ρ₀ = 1.2 kg/m³.
  - `pub struct MeshDynamics` with `new(mesh: &Mesh) -> Self` and `step(&mut self, mesh: &Mesh, omega1: f64, load_n: f64, dt: f64) -> f64`.
    - It returns the fluctuating mesh force, N (the load W taken off).
  - `pub const SINGLE_PAIR_STIFFNESS: f64 = 14e9;` in N/m per m of face (ISO 6336-1, c′ ≈ 14 N/(mm·µm) for solid steel spur gears).
  - `pub const MESH_DAMPING: f64 = 0.07;` (damping ratio. ESTIMATE: Kahraman & Singh measured 0.03–0.17.)

- [ ] **Step 1: Failing tests**
  - `the_housing_radiates_the_force_as_a_point_source`: `housing_pressure_pa(100.0, 0.006, 1.0)` = 1.179 Pa (±0.001). Hand: 1.2·100 / (2π·16.2).
  - `the_static_error_under_load_by_hand`: W = 1000 N, b = 20 mm, spur. The static deflection W / (c′·b·L) is 3.571 µm at L = 1 and 1.786 µm at L = 2. Test a `pub fn static_error_m(load_n, face_m, length) -> f64`.
  - `a_spur_mesh_whines_at_its_mesh_frequency`:
    - Setup: z1 = 26, z2 = 34, m = 2.5 mm, b = 18 mm, β = 0; ω₁ for 50 rev/s; W = 3000 N; dt = 1/96000; 0.5 s of output.
    - The Goertzel power at 1300 Hz (= 26 · 50) is ≥ 20 dB over the power at 1000 Hz and at 1700 Hz.
  - `helical_teeth_whine_far_less_than_spur`: same pair at β = 25°. Its force rms is ≥ 10 dB under the spur's.
  - `no_load_no_whine`: W = 0 gives a force rms < 1e-9 N. Only the loaded error is modelled; the unloaded geometric error is a later realism item.
  - `above_nyquist_it_is_silent`: a mesh frequency > 0.5/dt gives an output of 0. This is Review Focus 3.
- [ ] **Step 2: Run them.** Expect compile/test FAIL.
- [ ] **Step 3: Implement `MeshDynamics`.** It is the relative displacement x along the line of action.
  - Equation: m_e·ẍ + c·ẋ + k(t)·x = W, with k(t) = c′·b·L(t).
  - Mesh phase: t advances by z1·ω₁·dt / 2π per step.
  - Equivalent mass: m_e = J₁J₂ / (J₁·r_b2² + J₂·r_b1²), each J from a solid steel disc on its pitch radius and face (ρ = 7850 kg/m³).
  - Damping: c = 2ζ·√(k̄·m_e).
  - Integrator: semi-implicit Euler at `dt`.
  - Output: k(t)·x + c·ẋ − W.
- [ ] **Step 4: Run the tests.** Expect PASS. Sabotage: hold L constant (use ε_α). The spur test must fail. Restore it.
- [ ] **Step 5: Commit** `feat(core): a gear mesh's whine from its varying stiffness (ISO 6336, Maatar–Velex)`.

### Task 3: Rattle of a loose gear

**Files:** Modify `crates/engine-core/src/gear_noise.rs`

**Interfaces:**
- Produces:
  - `pub struct LooseGear` with `new(inertia_kgm2: f64, base_radius_m: f64) -> Self` and `step(&mut self, input_omega: f64, dt: f64, backlash_m: f64, drag_nm: f64) -> f64`.
    - It returns the impact force on the teeth, N (0 between impacts).
  - Model (Singh, Xie & Comparin, JSV 131(2), 1989):
    - The gear moves freely in ±backlash/2 relative to its driver, decelerated by the oil drag `drag_nm`.
    - On contact, a stiffness c′·b with the mesh damping acts.

- [ ] **Step 1: Failing tests** (J = 2e-4 kg·m², r_b = 0.04 m, backlash 1e-4 m, drag 0.05 N·m)
  - `a_steady_input_does_not_rattle`: ω = 100 rad/s constant for 1 s. After the first 0.2 s, the number of samples with force > 0 is 0.
  - `rattle_starts_at_the_singh_threshold`:
    - The input is ω = 100 + A/(2πf)·sin(2πf·t) at f = 50 Hz, an acceleration amplitude A.
    - Threshold (Singh): the gear leaves its flank when J·A > T_d, so A* = T_d / J = 250 rad/s².
    - At A = 0.5·A* there are no impacts after settling. At A = 3·A* there are ≥ 50 impacts per second.
- [ ] **Step 2: Run.** Expect FAIL.
- [ ] **Step 3: Implement** as specified. Count an impact as a rising edge of the force > 0.
- [ ] **Step 4: PASS, then sabotage** (drop the drag): the steady test must fail. Restore it.
- [ ] **Step 5: Commit** `feat(core): loose gears rattle in their backlash (Singh et al. 1989)`.

### Task 4: The gearbox definition and its noise

**Files:**
- Modify: `crates/engine-core/src/gear_noise.rs`
- Modify: `crates/engine-core/src/drivetrain.rs` (`VehicleDef.gearbox`)

**Interfaces:**
- Produces:
  - `pub struct GearboxDef`: `#[serde(default, deny_unknown_fields)]`.
    - Fields: `helix_deg: f64`, `teeth_sum: u32`, `module_m: f64`, `face_m: f64`, `final_pinion_teeth: u32`, `housing_wall_m: f64`, `backlash_m: f64`, `drag_nm: f64`.
    - Defaults, all ESTIMATE with reasoning in the doc comments:
      - 25°: road helical gears, 20–35°;
      - 60 teeth;
      - 2.5 mm module;
      - 18 mm face;
      - 17-tooth final pinion;
      - 6 mm aluminium wall: die-cast cases 4–8 mm;
      - 0.1 mm backlash: 0.05–0.15 mm;
      - 0.05 N·m drag.
  - `pub struct DriveState { pub gear: usize, pub input_omega: f64, pub input_torque_nm: f64, pub clutch_locked: bool }`.
  - `pub struct GearboxNoise` with `new(v: &VehicleDef) -> Self` and `step(&mut self, v: &VehicleDef, s: &DriveState, dt: f64) -> f64`, returning Pa at 1 m.
    - Whine: the engaged pair plus the final drive, loads from torque / base radius.
    - Rattle: every other forward pair's loose gear, driven at input_omega × its ratio.
    - All of it goes through `housing_pressure_pa`.
  - `VehicleDef.gearbox: GearboxDef` (`#[serde(default)]`).
- [ ] **Step 1: Failing tests**
  - `a_vehicle_file_without_a_gearbox_loads_the_default` (Review Focus 4).
  - `the_gearbox_round_trips_json_and_yaml`.
  - `third_gear_whines_at_its_and_the_final_drives_orders`:
    - Fixture: the `fixture()` vehicle with ratios [3.6, 2.1, 1.3, 1.0, 0.8] and final drive 4.1; gear 3; 50 rev/s; 150 N·m.
    - Expected peaks (Goertzel, each ≥ 20 dB over the 100 Hz-away neighbours):
      - 1300 Hz: 26 teeth × 50 rev/s;
      - 650 Hz: 17 × the output's 38.24 rev/s.
  - `neutral_with_the_clutch_up_rattles_and_with_it_down_stays_quiet` (Review Focus 1):
    - Gear 0, input 85 rad/s with a 4-cylinder firing fluctuation (A = 3·A* at 2 × crank frequency).
    - With `clutch_locked = true`, impacts appear; with `false`, the output is 0.
  - `overdrive_ratios_make_valid_pairs` (Review Focus 5): ratio 0.5 gives each count ≥ 12 and ε_α > 1.
  - `a_slipping_clutch_or_gear_0_does_not_whine` (Review Focus 2): output finite, whine part 0.
- [ ] **Step 2: FAIL. Step 3: implement. Step 4: PASS.** Sabotage the final drive's teeth (use z1 of the pair): the 650 Hz assertion must fail.
- [ ] **Step 5: Commit** `feat(core): a gearbox definition and its whine and rattle`.

### Task 5: Heard with the engine

**Files:**
- Modify: `crates/engine-core/src/sound.rs` (`gear_noise: f64`, default 1.0, doc: "1 = physical, 0 = off")
- Modify: `crates/engine-core/src/engine.rs`:
  - `Controls.drive: Option<DriveState>`, default `None`;
  - a `GearboxNoise` in the engine;
  - mixed through `l.from_engine_at(n_banks, …)` next to `whine`.
- Modify: `crates/engine-core/src/drivetrain.rs`: `Vehicle::drive_state(&self, engine_omega: f64, load: &Load) -> DriveState`.
- Modify: `crates/engine-app/src/sim.rs`: set `c.drive` each substep from the vehicle.
- Modify: the sound panel in `crates/engine-app/src/` (wherever `compressor_whine` has its slider): a "Gearbox" slider 0–2.

- [ ] **Step 1: Failing tests**
  - `the_engine_carries_the_gearbox_whine` (engine-core):
    - An inline-4 in 3rd at 3000 rpm. Compare `sound.gear_noise` = 1 against 0.
    - The difference signal's power at 26 × 50 Hz is ≥ 20 dB over its neighbours.
  - `a_road_box_whines_under_the_engine` (engine-core): the plausibility check against Sellgren & Åkerblom (DESIGN 2004). Their design target is gear whine ≥ 15 dB below the other sources.
    - Setup: K20C1 preset vehicle, 4th gear, 2500 rpm, 30 % throttle, 2 s.
    - Assertion: the whine's rms is ≥ 15 dB under the rest of the sound.
    - **If it fails, do not tune:** stop and report the measured margin to the user. It is a check of the physics, not a target to fit.
  - The sim test (engine-app) `the_drive_reaches_the_engine`: after `Simulation` steps in gear, the engine saw `Some(DriveState)` with the vehicle's gear.
- [ ] **Step 2: FAIL. Step 3: implement. Step 4: PASS.** Sabotage the mixing (drop `gear_noise`): the first test must fail.
- [ ] **Step 5: Measure.** Run `--headless` on the LFA at High, before and after. Put the timing in the PR.
- [ ] **Step 6: Commit** `feat: the gearbox is heard with the engine (sound.gear_noise)`.

### Task 6: Ear check and PR

- [ ] Render `--preset honda-k20c1 --render gears` (and `drive`) with `gear_noise` 1 and 0, loudness-matched.
- [ ] Send both clips to the user (SendUserFile) and **wait for the verdict** before anything is built on it (ear over spectrum).
- [ ] Open the PR: full local gate (fmt, clippy, nextest, coverage) and verify-pr.
  - Manual test: K20C1, drive in 3rd and hear the whine rise with rpm and load; sit in neutral at idle with the clutch up (rattle), then press the clutch (it stops).
  - Start the mutation run on the `engine-core` diff in the background.
- [ ] Update `docs/ROADMAP.md` and `CHANGELOG.md`, and tag on merge.

Shift impacts move to the sequential plan: a synchro box makes no impact unless forced, and the dog box is where the clunk lives (spec "Shift noise").
