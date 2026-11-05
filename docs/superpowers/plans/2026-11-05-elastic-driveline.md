# Elastic Driveline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans. Steps use `- [ ]`.

**Goal:** In gear the crank keeps its cycle-to-cycle speed variation: the car joins it through
a torsional spring and damper (first mode "shuffle", 2–7 Hz) instead of as a rigid inertia.

**Spec:** `docs/superpowers/specs/2026-11-05-driveline-compliance-design.md` (approved 2026-11-05).

## Global Constraints
- `vehicle.shuffle_hz` default 3.5 (first gear, ESTIMATE, middle of 2–7 Hz: Farshidianfar et al.
  2002); `vehicle.shuffle_damping` ζ default 0.15 (ESTIMATE); serde defaults, JSON/YAML round trip.
- k at the crank in gear 1: k₁ = (2πf)²/(1/J_e + 1/J_v1), J_v = m_eff·r²/G²; the wheel-side
  stiffness k_w = k₁·G₁² is the car's; in gear n, k = k_w/G_n². c = 2ζ·√(k·J_e·J_v/(J_e + J_v)).
- J_e = the engine's flywheel inertia, handed to the vehicle (`Vehicle::set_engine_inertia`).
- Slipping clutch: unchanged. Locked: no reflected inertia; the load is the spring torque.
- Hot loop: a few multiplies per step.

## Review Focus
1. Lock / unlock transitions (shift, clutch pedal): no torque step on locking — test in Task 1.
2. Neutral, gear 0: no spring — test in Task 1.
3. Stopped car, engine idling in gear with clutch in: unchanged.
4. Very low J_e (single cylinder) — stability at the physics dt: test in Task 1.

### Task 1: Two-inertia locked driveline (crates/engine-core/src/drivetrain.rs)
- [ ] Tests (hand values): `the_shuffle_frequency_by_hand` (fixture, J_e 0.15: J_v1 =
  1000·0.09/144 = 0.625; k₁ = (2π·3.5)²·/(1/0.15 + 1/0.625) = 483.6/8.267 = 58.5 N·m/rad;
  the twist oscillation after a torque step rings at 3.5 ± 0.1 Hz);
  `the_higher_gear_shuffles_faster` (gear 2, G = 8: k = k_w/64, J_v = 1000·0.09/64 = 1.406;
  f = (1/2π)√(k(1/J_e + 1/J_v)) by hand);
  `a_steady_cruise_twists_by_the_torque_over_k` (steady: twist = T_road/k, car speed = ω_e·r/G);
  `energy_is_kept_without_damping_or_load` (ζ 0, no road load: E within 0.1 % over 10 s);
  `locking_does_not_step_the_torque`; `neutral_has_no_spring`; `fields_round_trip`.
- [ ] RED, implement, GREEN, commit.

### Task 2: Wire J_e in the app (crates/engine-app/src/sim.rs) and the sound check
- [ ] Test: `in_gear_the_crank_varies_as_when_free` — the crank's cycle-mean speed spread in 3rd
  at ~3000 rpm, 30 % throttle, steady, within a factor 2 of the same engine free in neutral at
  the same speed and throttle-for-that-speed; and ≥ 5× the rigid driveline's (shuffle_hz → ∞
  is not available: compare with the pre-change hash of the spread, measured first).
- [ ] RED (today rigid), implement, GREEN; full gates; PR with clips and spectrograms.
