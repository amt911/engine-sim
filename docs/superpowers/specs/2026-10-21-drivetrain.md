# Drivetrain, vehicle and dyno — spec (2026-10-21)

Status: approved by delegation (roadmap item "load: dyno, gears, vehicle", spec
`2026-10-21-engine-sim-design.md`). Implemented on its own branch from `main`.

## Goal

Make the engine *work*: drive a car through a clutch and gearbox, or hold it on a dyno, and
see torque/power curves.

## Contracts

### `engine-core::drivetrain` (pure, deterministic, SI, tested)

```rust
pub struct VehicleDef { mass_kg, wheel_radius_m, drag_cd_a_m2, rolling_coeff,
                        gear_ratios: Vec<f64>, final_drive: f64, driveline_efficiency: f64 }
pub struct Vehicle { /* speed_m_s, gear: usize (0 = neutral), clutch: f64 0..1 */ }
impl Vehicle {
    /// Advance by dt given engine speed (rad/s). Returns the load torque on the crank (N·m),
    /// to be passed as `Controls::load_torque_nm`, and the reflected inertia (kg·m²).
    pub fn step(&mut self, dt, engine_omega, brake: f64) -> Load;
}
```
- Clutch: friction-limited coupling (max transmissible torque ∝ clutch engagement); slipping
  when the engine and wheel-side speeds differ; locked otherwise (Guzzella & Onder; Eriksson & Nielsen ch. driveline).
- Road load: aero ½ρ·CdA·v², rolling Crr·m·g, brake torque.
- Presets: a ~1500 kg road car for the I4; LFA: 1480 kg, 6-speed ratios
  3.231/2.188/1.609/1.233/0.970/0.795, final 4.556, tyre radius ~0.34 m
  (verify against a public source; mark unverified values).

### Dyno

`Controls::hold_rpm` already exists (speed hold). Add a **dyno sweep**: hold rpm stepping from
idle to redline at WOT, record averaged brake torque per step → curve (rpm, N·m, hp).
Headless CLI: `--dyno` prints the curve as CSV; GUI: a "Dyno" panel with a "Run sweep" button
and an `egui_plot`-free polyline plot (reuse `view::polyline`).

### UI

- Driver panel: gear (keys: `E` up, `Q` down, `1-6`, `N`), clutch (hold `C` = disengaged),
  brake (hold `B`/Down). Speedometer (km/h) + gear in the gauges.
- Engine stays the same `Simulation`; the vehicle lives in `Simulation` and feeds
  `controls.load_torque_nm` each sample.

## Cases

- Neutral → load 0; in gear with clutch locked, car speed = engine ω·r_wheel/(gear·final).
- Full throttle in 1st accelerates; aero drag caps top speed in top gear below what the
  engine's max power allows (P = F·v).
- Clutch disengaged → engine free-revs, car coasts down.
- Dyno sweep of the I4 peaks between 2000 and 4500 rpm; LFA curve within ±15 % of the
  published 480 N·m / 552 hp.
