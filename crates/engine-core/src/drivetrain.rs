//! Vehicle, clutch and gearbox: turns the engine's crank speed into car speed and back into
//! a load torque (plus reflected inertia) on the crank. Pure, deterministic, SI.
//!
//! Model (Guzzella & Onder, *Introduction to Modeling and Control of Internal Combustion
//! Engine Systems*, 2nd ed., ch. 2 "Vehicle propulsion / driveline"; Eriksson & Nielsen,
//! *Modeling and Control of Engines and Drivelines*, ch. 5-6):
//!
//! * Road load `F = ½ρ·CdA·v² + Crr·m·g + F_brake` (G&O eq. 2.1-2.3).
//! * Overall ratio `G = i_gear · i_final`; the crank-side speed of the wheels is `ω_t = G·v/r`.
//! * Clutch: Coulomb friction with capacity `T_max = engagement² · capacity` (the square gives
//!   a soft bite point; a linear map stalls the engine on every launch). While the
//!   speeds differ it slips and transmits `T_max·tanh(slip/ε)` (the tanh regularises the
//!   sign at zero slip, as in Eriksson & Nielsen §6). When the slip is below [`LOCK_SLIP`]
//!   and the torque needed to keep the car with the engine fits inside `T_max`, it locks:
//!   engine and car then move as one body, the crank sees the car's mass as the reflected
//!   inertia `(m + J_w/r²)·r²/G²` and the road load reflected through `G` and the driveline
//!   efficiency. If the needed torque exceeds `T_max` it slips again.
//! * Locked, the car is not a rigid inertia on the crank: the half-shafts, tyres and clutch
//!   damper are a torsional spring and damper between them, their first mode ("shuffle")
//!   at `VehicleDef::shuffle_hz` in first gear (2–7 Hz on road cars). Above it the crank turns
//!   free of the car's inertia, and each cycle's combustion moves it as it does in neutral
//!   (docs/superpowers/specs/2026-11-05-driveline-compliance-design.md).

use serde::{Deserialize, Serialize};

/// Air density at 20 °C and sea level, kg/m³ (ISA gives 1.225 at 15 °C).
pub const AIR_DENSITY: f64 = 1.2;
/// Standard gravity, m/s².
pub const GRAVITY: f64 = 9.81;
/// Slip (rad/s) below which a slipping clutch may lock.
pub const LOCK_SLIP: f64 = 0.5;
/// Slip regularisation of the Coulomb torque, rad/s (about 20 rpm).
const SLIP_EPS: f64 = 2.0;
/// Speed (m/s) over which rolling resistance fades in, so a stopped car does not creep back.
const ROLL_FADE_SPEED: f64 = 0.1;
/// Clutch engagement rate when the pedal is released (1 / s): about 0.7 s to full.
const CLUTCH_ENGAGE_RATE: f64 = 1.5;
/// The crank's inertia the vehicle assumes until told (`Vehicle::set_engine_inertia`), kg·m².
const DEFAULT_ENGINE_INERTIA: f64 = 0.15;
/// Clutch engagement rate when the pedal is pressed (1 / s).
const CLUTCH_RELEASE_RATE: f64 = 10.0;

/// Static description of a vehicle (data, `serde`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VehicleDef {
    pub name: String,
    /// Total mass including driver, kg.
    pub mass_kg: f64,
    /// Loaded tyre radius, m.
    pub wheel_radius_m: f64,
    /// Drag coefficient times frontal area, m².
    pub drag_cd_a_m2: f64,
    /// Rolling resistance coefficient (dimensionless).
    pub rolling_coeff: f64,
    /// Forward gear ratios, 1st first.
    pub gear_ratios: Vec<f64>,
    /// Final drive (differential, or primary + chain for a motorcycle).
    pub final_drive: f64,
    /// Mean driveline efficiency, 0..1.
    pub driveline_efficiency: f64,
    /// Rotating inertia of wheels, tyres and brakes (total), kg·m².
    pub wheel_inertia_kgm2: f64,
    /// Maximum torque the clutch can transmit when fully engaged, N·m.
    pub clutch_capacity_nm: f64,
    /// Deceleration at full brake, m/s² (tyre grip limit, ~0.9 g road car).
    pub max_brake_decel_m_s2: f64,
    /// Published gear-change time of an automated gearbox, s (0 = not given: a driver's manual
    /// shift). A shift opens the clutch with the engine's torque cut for half of it and closes
    /// it in the other half, the engine's torque held low until the clutch locks (Glielmo et
    /// al., *Gearshift control for automated manual transmissions*, IEEE/ASME Trans.
    /// Mechatronics 11(1), 2006, §III and Fig. 10).
    pub shift_time_s: f64,
    /// How far ahead of the tail pipes' tips the engine sits along the car, m: its intake
    /// mouths and block are heard from there, farther and later than the tails.
    pub engine_ahead_m: f64,
    /// The bay closed round the engine, if it has one: it takes its part of the valve ticking
    /// the head radiates into it ([`crate::engine_bay`]). Like every field, files without it take
    /// the default car's (a road car's bay).
    pub engine_bay: Option<crate::engine_bay::EngineBay>,
    /// First torsional mode of the driveline in first gear ("shuffle"), Hz: the half-shafts,
    /// tyres and clutch damper are a spring between the crank and the car, 2–7 Hz on road cars
    /// (Farshidianfar et al., *Int. J. Heavy Vehicle Systems*, 2002; 1–10 Hz by gear and
    /// stiffness, Michigan Tech's shuffle and clunk model). Default 3.5: the middle (ESTIMATE).
    pub shuffle_hz: f64,
    /// That mode's damping ratio ζ (ESTIMATE: a lightly damped mode that rings once or twice).
    pub shuffle_damping: f64,
}

impl Default for VehicleDef {
    fn default() -> Self {
        Self::inline4()
    }
}

impl VehicleDef {
    /// Light road motorcycle with rider: all values are ESTIMATES (typical 500 cc naked
    /// bike: ~175 kg + 75 kg rider; 160/60 R17 rear tyre = 0.312 m; top gear ~150 km/h
    /// at 6,000 rpm sets the 5.0 overall reduction; Cd·A = 0.6 × 0.6 m² sitting rider).
    pub fn single() -> Self {
        Self {
            name: "Light motorcycle".into(),
            mass_kg: 250.0,
            wheel_radius_m: 0.312,
            drag_cd_a_m2: 0.36,
            rolling_coeff: 0.015,
            gear_ratios: vec![2.75, 1.94, 1.50, 1.24, 1.05, 0.92],
            final_drive: 5.0,
            driveline_efficiency: 0.90,
            wheel_inertia_kgm2: 1.2,
            clutch_capacity_nm: 90.0,
            max_brake_decel_m_s2: 8.0,
            shift_time_s: 0.0,
            // the engine sits ahead of a rear-side silencer's tip (ESTIMATE)
            engine_ahead_m: 0.9,
            engine_bay: None,
            shuffle_hz: 3.5,
            shuffle_damping: 0.15,
        }
    }

    /// Compact hatchback, 5-speed. ESTIMATES (typical 2.0 L C-segment car: 1,300 kg with
    /// driver, 205/55 R16 tyre = 0.316 m, Cd 0.32 × 2.0 m², ratios of a common 5-speed
    /// manual); not taken from a specific model.
    pub fn inline4() -> Self {
        Self {
            name: "Compact hatchback".into(),
            mass_kg: 1300.0,
            wheel_radius_m: 0.316,
            drag_cd_a_m2: 0.64,
            rolling_coeff: 0.012,
            gear_ratios: vec![3.62, 1.95, 1.36, 0.94, 0.78],
            final_drive: 4.1,
            driveline_efficiency: 0.92,
            wheel_inertia_kgm2: 4.0,
            clutch_capacity_nm: 350.0,
            max_brake_decel_m_s2: 9.0,
            shift_time_s: 0.0,
            // front engine over the front axle: 2.6 m wheelbase + 0.8 m rear overhang (ESTIMATE)
            engine_ahead_m: 3.4,
            engine_bay: Some(crate::engine_bay::EngineBay::default()),
            shuffle_hz: 3.5,
            shuffle_damping: 0.15,
        }
    }

    /// Honda Civic Type R (FK8, 2017): its 6-speed's ratios 3.625/2.115/1.529/1.125/0.911/0.734
    /// and 4.105 final drive (Honda press kit, as listed by the motoring press), kerb mass ~1,380
    /// kg + a driver = 1,455 kg; 245/30 R20 tyres: 0.245 × 0.30 + 0.508/2 = 0.3275 m. ESTIMATES:
    /// Cd·A 0.75 m² (Cd ≈ 0.34 × ~2.2 m²); a clutch holding 1.3× the engine's 400 N·m (the usual
    /// margin of a clutch over its engine, Shaver, *Manual Transmission Clutch Systems*, SAE AE-17).
    pub fn civic_type_r_fk8() -> Self {
        Self {
            name: "Honda Civic Type R (FK8)".into(),
            mass_kg: 1455.0,
            wheel_radius_m: 0.3275,
            drag_cd_a_m2: 0.75,
            rolling_coeff: 0.012,
            gear_ratios: vec![3.625, 2.115, 1.529, 1.125, 0.911, 0.734],
            final_drive: 4.105,
            driveline_efficiency: 0.92,
            wheel_inertia_kgm2: 4.0,
            clutch_capacity_nm: 1.3 * 400.0,
            max_brake_decel_m_s2: 9.0,
            shift_time_s: 0.0,
            // front engine over the front axle: 2.7 m wheelbase + 0.9 m rear overhang (ESTIMATE)
            engine_ahead_m: 3.6,
            engine_bay: Some(crate::engine_bay::EngineBay::default()),
            shuffle_hz: 3.5,
            shuffle_damping: 0.15,
        }
    }

    /// Lexus LFA. Kerb mass 1,480 kg, the six gear ratios 3.231/2.188/1.609/1.233/0.970/0.795
    /// and the 4.556 final drive are the figures given for the LFA's 6-speed ASG in the
    /// task spec (not independently re-verified here). The rear tyre 305/30 ZR20 gives
    /// 0.305 × 0.30 + 0.508/2 = 0.3455 m (computed from the size). ESTIMATES: Cd·A 0.65 m²
    /// (Cd ~0.31), Crr, efficiency, wheel inertia, clutch capacity, 1.0 g braking. The ASG's
    /// "upshift speeds down to 0.2 seconds" (Toyota, *Lexus Debuts LFA*, news release,
    /// 21 October 2009).
    pub fn lfa_v10() -> Self {
        Self {
            name: "Lexus LFA".into(),
            mass_kg: 1480.0,
            wheel_radius_m: 0.3455,
            drag_cd_a_m2: 0.65,
            rolling_coeff: 0.011,
            gear_ratios: vec![3.231, 2.188, 1.609, 1.233, 0.970, 0.795],
            final_drive: 4.556,
            driveline_efficiency: 0.93,
            wheel_inertia_kgm2: 5.0,
            clutch_capacity_nm: 800.0,
            max_brake_decel_m_s2: 10.0,
            shift_time_s: 0.2,
            // front-mid engine ~0.35 m behind the front axle: 0.96 m rear overhang + 2.605 m wheelbase − 0.35 m (Lexus GB, 2009-10-21; the 0.35 m an ESTIMATE)
            engine_ahead_m: 3.2,
            // a flat underbody to its carbon diffuser (Lexus, launch release 2009-10-21, as
            // reported by autotrader.co.nz "Lexus supercar unveiled"): the bay closed below
            engine_bay: Some(crate::engine_bay::EngineBay {
                closed_underside: true,
                ..crate::engine_bay::EngineBay::default()
            }),
            shuffle_hz: 3.5,
            shuffle_damping: 0.15,
        }
    }

    /// A mid-engined coupé for the flat-plane V8, after the Ferrari 458 Italia:
    /// 1,485 kg, 7-speed DCT 3.08/2.19/1.63/1.29/1.03/0.84/0.69 with a 4.44 final drive (as
    /// commonly published; not independently re-verified), rear 295/35 ZR20 → 0.295 · 0.35 +
    /// 0.508/2 = 0.35725 m (computed). ESTIMATES: Cd·A, Crr, efficiency, inertias, clutch.
    pub fn flatplane_v8() -> Self {
        Self {
            name: "Flat-plane V8 coupé".into(),
            mass_kg: 1485.0,
            wheel_radius_m: 0.35725,
            drag_cd_a_m2: 0.66,
            rolling_coeff: 0.011,
            gear_ratios: vec![3.08, 2.19, 1.63, 1.29, 1.03, 0.84, 0.69],
            final_drive: 4.44,
            driveline_efficiency: 0.92,
            wheel_inertia_kgm2: 5.0,
            clutch_capacity_nm: 800.0,
            max_brake_decel_m_s2: 10.0,
            shift_time_s: 0.0,
            // mid engine, its centre ~0.7 m ahead of the rear axle, ~1.0 m rear overhang (ESTIMATE)
            engine_ahead_m: 1.7,
            engine_bay: Some(crate::engine_bay::EngineBay::default()),
            shuffle_hz: 3.5,
            shuffle_damping: 0.15,
        }
    }

    /// A front-engined sports car for the cross-plane V8, after the C6 Corvette (LS3): 1,441 kg,
    /// 6-speed manual 2.66/1.78/1.30/1.00/0.74/0.50 with a 3.42 final drive (as commonly
    /// published; not independently re-verified), rear 285/35 ZR19 → 0.285 · 0.35 + 0.4826/2
    /// = 0.34105 m (computed). ESTIMATES: Cd·A, Crr, efficiency, inertias, clutch.
    pub fn crossplane_v8() -> Self {
        Self {
            name: "Cross-plane V8 sports car".into(),
            mass_kg: 1441.0,
            wheel_radius_m: 0.34105,
            drag_cd_a_m2: 0.58,
            rolling_coeff: 0.011,
            gear_ratios: vec![2.66, 1.78, 1.30, 1.00, 0.74, 0.50],
            final_drive: 3.42,
            driveline_efficiency: 0.90,
            wheel_inertia_kgm2: 5.0,
            clutch_capacity_nm: 850.0,
            max_brake_decel_m_s2: 10.0,
            shift_time_s: 0.0,
            // front-mid engine just behind the front axle: ~2.69 m wheelbase + ~1.0 m rear overhang − 0.2 m (ESTIMATE)
            engine_ahead_m: 3.5,
            engine_bay: Some(crate::engine_bay::EngineBay::default()),
            shuffle_hz: 3.5,
            shuffle_damping: 0.15,
        }
    }

    /// Overall ratio crank → wheel for a gear (1-based); 0 for neutral or an unknown gear.
    pub fn overall_ratio(&self, gear: usize) -> f64 {
        match gear.checked_sub(1).and_then(|i| self.gear_ratios.get(i)) {
            Some(r) => r * self.final_drive,
            None => 0.0,
        }
    }
}

/// What the vehicle asks of the crank for one step.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Load {
    /// Torque opposing the crank, N·m (pass as `Controls::load_torque_nm`).
    pub torque_nm: f64,
    /// Inertia added to the crank, kg·m² (pass as `Controls::extra_inertia_kgm2`).
    pub inertia_kgm2: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Vehicle {
    def: VehicleDef,
    speed_m_s: f64,
    gear: usize,
    clutch: f64,
    clutch_target: f64,
    /// Engagement rate when the clutch closes, 1/s.
    engage_rate: f64,
    locked: bool,
    /// Set after an overload: no relock until the slip has grown past `2·LOCK_SLIP`.
    relock_blocked: bool,
    /// The crank's own inertia (flywheel and crank), kg·m²: the shuffle's other mass.
    engine_inertia: f64,
    /// Locked: the driveline's twist referred to the crank, rad (crank ahead of the car), and
    /// the car's speed referred to the crank, rad/s.
    twist: f64,
    car_omega: f64,
    /// The clutch's torque at the last slipping step, N·m (a lock takes it on as the spring's).
    slip_torque: f64,
}

impl Vehicle {
    /// A stationary vehicle in neutral with the clutch engaged.
    pub fn new(def: VehicleDef) -> Self {
        Self {
            def,
            speed_m_s: 0.0,
            gear: 0,
            clutch: 1.0,
            clutch_target: 1.0,
            engage_rate: CLUTCH_ENGAGE_RATE,
            locked: false,
            relock_blocked: false,
            engine_inertia: DEFAULT_ENGINE_INERTIA,
            twist: 0.0,
            car_omega: 0.0,
            slip_torque: 0.0,
        }
    }

    /// The crank's own inertia, kg·m² (the engine's flywheel and crank).
    pub fn set_engine_inertia(&mut self, j: f64) {
        if j > 0.0 {
            self.engine_inertia = j;
        }
    }

    /// The car's inertia referred to the crank in the current gear, kg·m² (0 in neutral).
    pub fn car_inertia_kgm2(&self) -> f64 {
        let g = self.def.overall_ratio(self.gear);
        if g == 0.0 {
            return 0.0;
        }
        self.effective_mass() * self.def.wheel_radius_m.powi(2) / (g * g)
    }

    /// The driveline's torsional stiffness referred to the crank in the current gear, N·m/rad:
    /// the wheel-side stiffness k_w that puts the first gear's shuffle at `shuffle_hz`,
    /// k₁ = (2πf)²/(1/J_e + 1/J_v1), k_w = k₁·G₁², here k_w/G² (0 in neutral).
    pub fn stiffness_nm_rad(&self) -> f64 {
        let (g1, g) = (self.def.overall_ratio(1), self.def.overall_ratio(self.gear));
        if g == 0.0 || g1 == 0.0 {
            return 0.0;
        }
        let j_v1 = self.effective_mass() * self.def.wheel_radius_m.powi(2) / (g1 * g1);
        let w = 2.0 * std::f64::consts::PI * self.def.shuffle_hz;
        let k1 = w * w / (1.0 / self.engine_inertia + 1.0 / j_v1);
        k1 * g1 * g1 / (g * g)
    }

    /// Its damping, N·m·s/rad: c = 2ζ·√(k·J_e·J_v/(J_e + J_v)).
    fn damping(&self, k: f64, j_v: f64) -> f64 {
        let j = self.engine_inertia * j_v / (self.engine_inertia + j_v);
        2.0 * self.def.shuffle_damping * (k * j).sqrt()
    }

    /// The driveline's twist referred to the crank, rad (0 unless locked).
    pub fn twist_rad(&self) -> f64 {
        if self.locked { self.twist } else { 0.0 }
    }

    /// The car's speed referred to the crank, rad/s.
    pub fn car_omega(&self) -> f64 {
        let g = self.def.overall_ratio(self.gear);
        if self.locked {
            self.car_omega
        } else {
            self.speed_m_s / self.def.wheel_radius_m * g
        }
    }
    pub fn def(&self) -> &VehicleDef {
        &self.def
    }
    pub fn speed_m_s(&self) -> f64 {
        self.speed_m_s
    }
    pub fn set_speed(&mut self, v: f64) {
        self.speed_m_s = v.max(0.0);
    }
    pub fn gear(&self) -> usize {
        self.gear
    }
    /// Select a gear (0 = neutral); unknown gears are ignored. Shifting unlocks the clutch.
    /// Shifting with the clutch engaged at speed is allowed on purpose (a synchromesh box
    /// does not forbid it): the clutch then slips to the new speed ratio, or the engine
    /// stalls at standstill, as in a real car.
    pub fn set_gear(&mut self, gear: usize) {
        if gear <= self.def.gear_ratios.len() && gear != self.gear {
            self.gear = gear;
            self.locked = false;
            self.relock_blocked = false;
        }
    }
    /// Clutch engagement now (1 = fully engaged).
    pub fn clutch(&self) -> f64 {
        self.clutch
    }
    /// Command the clutch: engagement 1 = pedal up. The actual engagement ramps there.
    pub fn set_clutch(&mut self, engagement: f64) {
        self.clutch_target = engagement.clamp(0.0, 1.0);
    }
    /// How long the clutch takes to close from open, s: an automated gearbox's actuator
    /// ([`VehicleDef::shift_time_s`]); a non-positive time is a driver's release (1/1.5 s).
    pub fn set_clutch_engage_s(&mut self, seconds: f64) {
        self.engage_rate = if seconds > 0.0 {
            1.0 / seconds
        } else {
            CLUTCH_ENGAGE_RATE
        };
    }
    pub fn locked(&self) -> bool {
        self.locked
    }

    /// Resistive force on the car at speed `v` and brake pedal `brake` (0..1), N.
    pub fn road_force_n(&self, v: f64, brake: f64) -> f64 {
        let d = &self.def;
        let aero = 0.5 * AIR_DENSITY * d.drag_cd_a_m2 * v * v;
        let rolling = d.rolling_coeff * d.mass_kg * GRAVITY * (v / ROLL_FADE_SPEED).tanh();
        aero + rolling + brake.clamp(0.0, 1.0) * d.max_brake_decel_m_s2 * d.mass_kg
    }

    fn effective_mass(&self) -> f64 {
        self.def.mass_kg + self.def.wheel_inertia_kgm2 / self.def.wheel_radius_m.powi(2)
    }

    fn ramp_clutch(&mut self, dt: f64) {
        let rate = if self.clutch_target > self.clutch {
            self.engage_rate
        } else {
            CLUTCH_RELEASE_RATE
        };
        let d = self.clutch_target - self.clutch;
        self.clutch += d.clamp(-rate * dt, rate * dt);
    }

    fn coast(&mut self, dt: f64, brake: f64) {
        let f = self.road_force_n(self.speed_m_s, brake);
        self.speed_m_s = (self.speed_m_s - f / self.effective_mass() * dt).max(0.0);
    }

    /// Advance by `dt` given the engine speed (rad/s) and brake pedal (0..1). Returns the
    /// load on the crank for the engine's next step.
    pub fn step(&mut self, dt: f64, engine_omega: f64, brake: f64) -> Load {
        self.ramp_clutch(dt);
        let g = self.def.overall_ratio(self.gear);
        let t_max = self.clutch * self.clutch * self.def.clutch_capacity_nm;
        if g == 0.0 || t_max <= 0.0 {
            self.locked = false;
            self.coast(dt, brake);
            return Load::default();
        }
        let (r, eta, m_eff) = (
            self.def.wheel_radius_m,
            self.def.driveline_efficiency,
            self.effective_mass(),
        );
        let k = self.stiffness_nm_rad();
        let j_v = m_eff * r * r / (g * g);
        if self.locked {
            // The car joins the crank through the driveline's spring and damper: its torque
            // T = k·twist + c·(ω_e − ω_v) loads the crank and drives the car (the shuffle mode).
            let c = self.damping(k, j_v);
            let t = k * self.twist + c * (engine_omega - self.car_omega);
            if t.abs() > t_max {
                // the clutch lets go: it slips at its capacity from this step
                self.locked = false;
                self.relock_blocked = true;
                self.slip_torque = t_max * t.signum();
                return Load {
                    torque_nm: self.slip_torque,
                    inertia_kgm2: 0.0,
                };
            }
            // symplectic Euler, in sub-steps short against the mode (ω_n·h ≤ 0.2: stable for
            // any step a caller takes)
            let w_n = (k * (1.0 / self.engine_inertia + 1.0 / j_v)).sqrt();
            let n = ((dt * w_n / 0.2).ceil() as usize).max(1);
            let h = dt / n as f64;
            for _ in 0..n {
                let t = k * self.twist + c * (engine_omega - self.car_omega);
                let to_car = if t >= 0.0 { t * eta } else { t / eta };
                let road = self.road_force_n(self.car_omega * r / g, brake) * r / g;
                self.car_omega = (self.car_omega + (to_car - road) / j_v * h).max(0.0);
                self.twist += (engine_omega - self.car_omega) * h;
            }
            self.speed_m_s = self.car_omega * r / g;
            return Load {
                torque_nm: t,
                inertia_kgm2: 0.0,
            };
        }
        let slip = engine_omega - self.speed_m_s / r * g;
        if self.relock_blocked && slip.abs() > 2.0 * LOCK_SLIP {
            self.relock_blocked = false;
        }
        if !self.locked && !self.relock_blocked && slip.abs() < LOCK_SLIP && k > 0.0 {
            // the spring takes on the clutch's last torque: no step on locking
            self.locked = true;
            self.car_omega = self.speed_m_s / r * g;
            self.twist = self.slip_torque / k;
            return Load {
                torque_nm: self.slip_torque,
                inertia_kgm2: 0.0,
            };
        }
        // Slipping: Coulomb friction, regularised at zero slip.
        let t_c = t_max * (slip / SLIP_EPS).tanh();
        self.slip_torque = t_c;
        // Power flows engine → wheels losing η; wheels → engine (engine braking) likewise.
        let f_wheel = if t_c >= 0.0 {
            t_c * g * eta / r
        } else {
            t_c * g / (eta * r)
        };
        let f_road = self.road_force_n(self.speed_m_s, brake);
        self.speed_m_s = (self.speed_m_s + (f_wheel - f_road) / m_eff * dt).max(0.0);
        Load {
            torque_nm: t_c,
            inertia_kgm2: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    /// Round numbers so every expectation is a short hand calculation.
    fn fixture() -> VehicleDef {
        VehicleDef {
            name: "test".into(),
            mass_kg: 1000.0,
            wheel_radius_m: 0.3,
            drag_cd_a_m2: 0.6,
            rolling_coeff: 0.01,
            gear_ratios: vec![3.0, 2.0],
            final_drive: 4.0,
            driveline_efficiency: 0.9,
            wheel_inertia_kgm2: 0.0,
            clutch_capacity_nm: 400.0,
            max_brake_decel_m_s2: 9.0,
            shift_time_s: 0.0,
            engine_ahead_m: 0.0,
            engine_bay: None,
            shuffle_hz: 3.5,
            shuffle_damping: 0.15,
        }
    }

    fn in_first() -> Vehicle {
        let mut v = Vehicle::new(fixture());
        v.set_gear(1);
        v
    }

    #[test]
    fn clutch_ramps_at_its_engage_and_release_rates() {
        let mut v = Vehicle::new(fixture());
        v.set_clutch(0.0);
        for _ in 0..1000 {
            v.ramp_clutch(1e-3);
        }
        v.set_clutch(1.0);
        v.ramp_clutch(0.1); // engage 1.5 /s → +0.15
        assert!((v.clutch() - 0.15).abs() < 1e-12, "{}", v.clutch());
        for _ in 0..20 {
            v.ramp_clutch(0.1);
        }
        assert_eq!(v.clutch(), 1.0, "no overshoot past the target");
        v.set_clutch(0.0);
        v.ramp_clutch(0.05); // release 10 /s → −0.5
        assert!((v.clutch() - 0.5).abs() < 1e-12, "{}", v.clutch());
    }

    /// Locked in 1st (overall 12) at 2 m/s, clutch 0.5 → capacity 0.25·400 = 100 N·m; the crank's
    /// acceleration rises to `alpha` rad/s² over 1 s (smoothly: a step would ring the shuffle
    /// mode) and stays there to 3 s. Needed clutch torque once the driveline settles (hand calc,
    /// fixture): (m·a + F_roll)·r/(g·η) with a = α·r/g, F_roll ≈ 0.01·1000·9.81 = 98.1 N:
    /// α = 111.3 → 80 N·m (holds); α = 168.9 → 120 N·m (slips).
    fn locked_run(alpha: f64) -> bool {
        let mut v = Vehicle::new(fixture());
        v.set_gear(1);
        v.set_clutch(0.5);
        for _ in 0..2000 {
            v.ramp_clutch(1e-3);
        }
        v.set_speed(2.0);
        let mut omega = 2.0 / 0.3 * 12.0;
        let dt = 1e-4;
        v.step(dt, omega, 0.0);
        assert!(v.locked(), "starts locked");
        for i in 0..30_000 {
            omega += alpha * (i as f64 * dt).min(1.0) * dt;
            v.step(dt, omega, 0.0);
            if !v.locked() {
                return false;
            }
        }
        true
    }

    #[test]
    fn locked_clutch_holds_below_capacity_and_slips_above() {
        assert!(locked_run(111.3), "80 N·m < 100 N·m must hold");
        assert!(!locked_run(168.9), "120 N·m > 100 N·m must slip");
    }

    #[test]
    fn after_an_overload_it_relocks_only_past_the_hysteresis() {
        let mut v = Vehicle::new(fixture());
        v.set_gear(1);
        v.set_clutch(0.5);
        for _ in 0..2000 {
            v.ramp_clutch(1e-3);
        }
        v.set_speed(2.0);
        let wheel = |v: &Vehicle| v.speed_m_s() / 0.3 * 12.0;
        let dt = 1e-5;
        v.step(dt, wheel(&v), 0.0);
        assert!(v.locked());
        // a gradual overload (120 N·m > 100 N·m, see `locked_run`): unlocks
        let mut omega = wheel(&v);
        let mut i = 0usize;
        while v.locked() {
            omega += 168.9 * (i as f64 * dt).min(1.0) * dt;
            v.step(dt, omega, 0.0);
            i += 1;
        }
        assert!(!v.locked(), "overload unlocks");
        // slip back inside LOCK_SLIP right away: blocked, must not relock
        v.step(dt, wheel(&v) + 0.2, 0.0);
        assert!(!v.locked(), "relock blocked until slip exceeds 2·LOCK_SLIP");
        // slip past 2·LOCK_SLIP (= 1 rad/s) unblocks; then back under LOCK_SLIP relocks
        v.step(dt, wheel(&v) + 1.5, 0.0);
        assert!(!v.locked());
        v.step(dt, wheel(&v) + 0.2, 0.0);
        assert!(v.locked(), "relocks once the hysteresis was crossed");
    }

    #[test]
    fn road_force_adds_aero_rolling_and_brake() {
        let v = Vehicle::new(fixture());
        // 20 m/s: aero ½·1.2·0.6·20² = 144 N; rolling 0.01·1000·9.81 = 98.1 N (tanh(200) = 1).
        assert_relative_eq!(v.road_force_n(20.0, 0.0), 242.1, max_relative = 1e-9);
        // Half brake: 0.5·9 m/s²·1000 kg = 4,500 N more.
        assert_relative_eq!(v.road_force_n(20.0, 0.5), 4742.1, max_relative = 1e-9);
        // Standing still: nothing pushes the car backwards.
        assert_eq!(v.road_force_n(0.0, 0.0), 0.0);
    }

    #[test]
    fn overall_ratio_multiplies_gear_and_final_drive() {
        let d = fixture();
        assert_eq!(d.overall_ratio(0), 0.0);
        assert_eq!(d.overall_ratio(1), 12.0);
        assert_eq!(d.overall_ratio(2), 8.0);
        assert_eq!(d.overall_ratio(3), 0.0);
    }

    #[test]
    fn neutral_puts_no_load_and_the_car_coasts_down() {
        let mut v = Vehicle::new(fixture());
        v.set_speed(20.0);
        let load = v.step(0.01, 300.0, 0.0);
        assert_eq!(load, Load::default());
        // a = 242.1 N / 1000 kg = 0.2421 m/s² → 20 − 0.002421.
        assert_relative_eq!(v.speed_m_s(), 19.997579, max_relative = 1e-9);
        // Full brake: (9000 + 242.1) N → 9.2421 m/s² for 0.01 s.
        let mut v = Vehicle::new(fixture());
        v.set_speed(20.0);
        v.step(0.01, 300.0, 1.0);
        assert_relative_eq!(v.speed_m_s(), 19.907579, max_relative = 1e-9);
    }

    #[test]
    fn braking_never_reverses_the_car() {
        let mut v = Vehicle::new(fixture());
        v.set_speed(0.05);
        v.step(1.0, 0.0, 1.0);
        assert_eq!(v.speed_m_s(), 0.0);
    }

    #[test]
    fn locked_clutch_reflects_road_load_and_car_inertia() {
        // ω = 120 rad/s in 1st (G = 12): v = 120·0.3/12 = 3.0 m/s, no slip → locks; held there,
        // the driveline settles to the road load.
        let mut v = in_first();
        v.set_speed(3.0);
        v.step(1e-3, 120.0, 0.0);
        assert!(v.locked());
        let mut load = Load::default();
        for _ in 0..20_000 {
            load = v.step(1e-3, 120.0, 0.0);
        }
        // Road load at 3 m/s: aero 3.24 N + rolling 98.1 N = 101.34 N.
        // Crank torque = F·r/(G·η) = 101.34·0.3/(12·0.9) = 2.815 N·m.
        assert_relative_eq!(load.torque_nm, 2.815, max_relative = 1e-3);
        // The car is not on the crank's inertia: it is the spring's other end, m·r²/G² =
        // 1000·0.09/144 = 0.625 kg·m².
        assert_eq!(load.inertia_kgm2, 0.0);
        assert_relative_eq!(v.car_inertia_kgm2(), 0.625, max_relative = 1e-9);
    }

    #[test]
    fn locked_car_speed_follows_the_crank() {
        let mut v = in_first();
        v.set_speed(3.0);
        v.step(1e-3, 120.0, 0.0);
        for _ in 0..20_000 {
            v.step(1e-3, 120.5, 0.0);
        }
        // once the shuffle has died: v = ω·r/G = 120.5·0.3/12 = 3.0125 m/s.
        assert_relative_eq!(v.speed_m_s(), 3.0125, max_relative = 1e-4);
    }

    #[test]
    fn slipping_clutch_transmits_its_capacity() {
        // Engine 300 rad/s against a stopped car: slip = 300 ≫ ε → T = 400 N·m (full clutch).
        let mut v = in_first();
        let load = v.step(0.01, 300.0, 0.0);
        assert!(!v.locked());
        assert_relative_eq!(load.torque_nm, 400.0, max_relative = 1e-9);
        assert_eq!(load.inertia_kgm2, 0.0);
        // Wheel force = T·G·η/r = 400·12·0.9/0.3 = 14,400 N → 14.4 m/s² → 0.144 m/s in 10 ms.
        assert_relative_eq!(v.speed_m_s(), 0.144, max_relative = 1e-9);
    }

    #[test]
    fn half_engaged_clutch_transmits_a_quarter() {
        let mut v = in_first();
        v.set_clutch(0.5);
        // The clutch drops 1 → 0.5 at 10 /s: reaches it in 0.05 s.
        let load = v.step(0.05, 300.0, 0.0);
        assert_relative_eq!(v.clutch(), 0.5, max_relative = 1e-12);
        // Torque ∝ engagement²: 0.25·400 = 100 N·m (soft bite point).
        assert_relative_eq!(load.torque_nm, 100.0, max_relative = 1e-9);
        // 100·12·0.9/0.3 = 3,600 N → 3.6 m/s² · 0.05 s = 0.18 m/s.
        assert_relative_eq!(v.speed_m_s(), 0.18, max_relative = 1e-9);
    }

    #[test]
    fn clutch_engages_slower_than_it_releases() {
        let mut v = in_first();
        v.set_clutch(0.0);
        v.step(0.05, 100.0, 0.0);
        assert_relative_eq!(v.clutch(), 0.5, max_relative = 1e-12); // 10 /s · 0.05 s
        v.set_clutch(1.0);
        v.step(0.05, 100.0, 0.0);
        assert_relative_eq!(v.clutch(), 0.575, max_relative = 1e-12); // 1.5 /s · 0.05 s
    }

    #[test]
    fn disengaged_clutch_frees_the_engine() {
        let mut v = in_first();
        v.set_speed(3.0);
        v.set_clutch(0.0);
        // Pedal fully down: it takes 0.1 s to release, so step once with a long dt.
        let load = v.step(1.0, 300.0, 0.0);
        assert_eq!(load, Load::default());
        assert!(!v.locked());
        // Coasts: 3 m/s − (101.34 N/1000 kg)·1 s.
        assert_relative_eq!(v.speed_m_s(), 3.0 - 0.10134, max_relative = 1e-9);
    }

    #[test]
    fn engine_braking_drags_the_crank_the_other_way() {
        // Engine 50 rad/s, wheels at 10 m/s → 10/0.3·12 = 400 rad/s on the crank side.
        let mut v = in_first();
        v.set_speed(10.0);
        let load = v.step(0.01, 50.0, 0.0);
        assert_relative_eq!(load.torque_nm, -400.0, max_relative = 1e-9);
        // Wheel force = −T·G/(η·r) = −400·12/(0.9·0.3) = −17,777.8 N; road 134.1 N
        // (aero 36 + rolling 98.1) → a = −17.9119 m/s² → 10 − 0.179119.
        assert_relative_eq!(v.speed_m_s(), 9.820881, max_relative = 1e-6);
    }

    #[test]
    fn an_overloaded_lock_lets_go() {
        let mut v = in_first();
        v.set_speed(3.0);
        v.step(1e-3, 120.0, 0.0);
        assert!(v.locked());
        // Crank driven at α = 800 rad/s² → car a = 800·0.3/12 = 20 m/s² → required torque
        // (1000·20 + 101.34)·0.3/(12·0.9) = 558.5 N·m > 400: the spring's torque builds within
        // a fraction of the shuffle's period (0.29 s) and the clutch slips at 400.
        let mut w = 120.0;
        let mut load = Load::default();
        for _ in 0..500 {
            w += 0.8;
            load = v.step(1e-3, w, 0.0);
            if !v.locked() {
                break;
            }
        }
        assert!(!v.locked());
        assert_relative_eq!(load.torque_nm, 400.0, max_relative = 1e-9);
    }

    #[test]
    fn a_locked_clutch_really_slips_when_the_crank_pushes_past_capacity() {
        // Regression: after an overload the clutch relocked in the same step (slip was ~0).
        // Locked at 3 m/s in 1st (capacity 400 N·m); the crank then accelerates at 800 rad/s²,
        // which needs (1000·20 + 101)·0.3/(12·0.9) = 558 N·m > 400.
        let mut v = in_first();
        v.set_speed(3.0);
        let mut w = 120.0;
        v.step(1e-4, w, 0.0);
        assert!(v.locked());
        let mut slipped = false;
        // 1e-4 s steps: the crank gains 0.08 rad/s per step, below LOCK_SLIP, which is what
        // let the old code relock instantly.
        for _ in 0..2000 {
            w += 0.08;
            let load = v.step(1e-4, w, 0.0);
            assert!(load.torque_nm.abs() <= 400.0 + 1e-9, "{}", load.torque_nm);
            slipped |= v.speed_m_s() < w * 0.3 / 12.0 - 0.01;
        }
        assert!(slipped, "car kept pace with a crank the clutch cannot hold");
        assert!(!v.locked());
    }

    #[test]
    fn a_gear_change_unlocks_and_unknown_gears_are_ignored() {
        let mut v = in_first();
        v.set_speed(3.0);
        v.step(0.01, 120.0, 0.0);
        assert!(v.locked());
        v.set_gear(2);
        assert_eq!(v.gear(), 2);
        assert!(!v.locked());
        v.set_gear(9);
        assert_eq!(v.gear(), 2);
        v.set_gear(0);
        assert_eq!(v.gear(), 0);
    }

    #[test]
    fn wheel_inertia_adds_to_the_reflected_inertia() {
        let mut d = fixture();
        d.wheel_inertia_kgm2 = 9.0; // + 9/0.3² = 100 kg effective
        let mut v = Vehicle::new(d);
        v.set_gear(1);
        v.set_speed(3.0);
        v.step(0.01, 120.0, 0.0);
        // (1000 + 100)·0.09/144 = 0.6875 kg·m².
        assert_relative_eq!(v.car_inertia_kgm2(), 0.6875, max_relative = 1e-9);
    }

    #[test]
    fn lfa_top_speed_matches_the_published_325_kmh() {
        // 9,000 rpm in 6th: ω = 942.478 rad/s; v = ω·r/(0.795·4.556) = 942.478·0.3455/3.62202
        // = 89.90 m/s = 323.6 km/h. Lexus quotes 325 km/h.
        let d = VehicleDef::lfa_v10();
        let v = 942.478 * d.wheel_radius_m / d.overall_ratio(6);
        assert_relative_eq!(v * 3.6, 323.6, max_relative = 2e-3);
    }

    #[test]
    fn a_clutch_told_to_close_in_a_tenth_of_a_second_does_so() {
        // an automated gearbox closes its clutch as fast as its shift allows; a driver's
        // default release takes 1/1.5 s
        let close = |engage_s: Option<f64>| {
            let mut v = Vehicle::new(VehicleDef::lfa_v10());
            v.set_clutch(0.0);
            for _ in 0..100 {
                v.step(1e-3, 600.0, 0.0);
            }
            assert_eq!(v.clutch(), 0.0);
            if let Some(s) = engage_s {
                v.set_clutch_engage_s(s);
            }
            v.set_clutch(1.0);
            let mut t = 0.0f64;
            while v.clutch() < 1.0 && t < 2.0 {
                v.step(1e-3, 600.0, 0.0);
                t += 1e-3;
            }
            t
        };
        assert!(
            (close(Some(0.1)) - 0.1).abs() < 2e-3,
            "{}",
            close(Some(0.1))
        );
        assert!((close(None) - 1.0 / 1.5).abs() < 2e-3, "{}", close(None));
        // no time given (a driver's shift) is the driver's release, not an instant one
        assert!((close(Some(0.0)) - 1.0 / 1.5).abs() < 2e-3);
    }

    #[test]
    fn the_single_rides_a_light_motorcycle() {
        // its own figures, not a car's: 250 kg with the rider on a 0.312 m tyre
        let d = VehicleDef::single();
        assert_eq!((d.mass_kg, d.wheel_radius_m), (250.0, 0.312));
        assert_eq!(d.shift_time_s, 0.0);
    }

    #[test]
    fn the_lfa_shifts_in_its_published_time_and_old_files_have_none() {
        // Lexus LFA ASG: 0.20 s per gear change (0.15 s on the Nürburgring Package)
        assert_eq!(VehicleDef::lfa_v10().shift_time_s, 0.2);
        let old: VehicleDef = serde_yaml_ng::from_str("name: x\nmass_kg: 1000\n").unwrap();
        assert_eq!(old.shift_time_s, 0.0);
        let d = VehicleDef::lfa_v10();
        let json: VehicleDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: VehicleDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d));
    }

    #[test]
    fn lfa_has_the_power_to_reach_its_gear_limited_top_speed() {
        // At 89.9 m/s: aero ½·1.2·0.65·89.9³ = 0.39·726,580 = 283.4 kW; rolling
        // 0.011·1480·9.81 = 159.7 N · 89.9 = 14.4 kW → 297.7 kW at the wheels. The engine's
        // ~388 kW (521 hp, see the dyno) · 0.93 = 361 kW exceeds it, so 6th gear, not drag,
        // sets the top speed — as the spec's P = F·v argument requires.
        let v = Vehicle::new(VehicleDef::lfa_v10());
        let p_road = v.road_force_n(89.9, 0.0) * 89.9;
        assert_relative_eq!(p_road, 297_700.0, max_relative = 5e-3);
        assert!(p_road < 388_000.0 * 0.93);
    }

    #[test]
    fn presets_are_sane_and_round_trip() {
        for d in [
            VehicleDef::single(),
            VehicleDef::inline4(),
            VehicleDef::civic_type_r_fk8(),
            VehicleDef::lfa_v10(),
            VehicleDef::flatplane_v8(),
            VehicleDef::crossplane_v8(),
        ] {
            assert!(d.mass_kg > 100.0 && d.wheel_radius_m > 0.2 && d.wheel_radius_m < 0.5);
            assert!(d.gear_ratios.windows(2).all(|w| w[0] > w[1]), "{}", d.name);
            let json = serde_json::to_string(&d).unwrap();
            assert_eq!(serde_json::from_str::<VehicleDef>(&json).unwrap(), d);
            let yaml = serde_yaml_ng::to_string(&d).unwrap();
            assert_eq!(serde_yaml_ng::from_str::<VehicleDef>(&yaml).unwrap(), d);
        }
        assert_eq!(VehicleDef::lfa_v10().gear_ratios.len(), 6);
        assert_eq!(VehicleDef::lfa_v10().mass_kg, 1480.0);
        // Missing fields fall back to defaults; unknown ones are rejected.
        assert_eq!(
            serde_json::from_str::<VehicleDef>("{}").unwrap(),
            VehicleDef::default()
        );
        assert!(serde_json::from_str::<VehicleDef>(r#"{"mas_kg": 1}"#).is_err());
    }

    proptest! {
        #[test]
        fn speed_stays_finite_and_non_negative(
            gear in 0usize..3,
            omegas in proptest::collection::vec(0.0f64..1000.0, 1..40),
            brake in 0.0f64..1.0,
            clutch in 0.0f64..1.0,
        ) {
            let mut v = Vehicle::new(fixture());
            v.set_gear(gear);
            v.set_clutch(clutch);
            for w in omegas {
                let l = v.step(0.001, w, brake);
                prop_assert!(v.speed_m_s().is_finite() && v.speed_m_s() >= 0.0);
                prop_assert!(l.torque_nm.is_finite() && l.inertia_kgm2 >= 0.0);
                prop_assert!(l.torque_nm.abs() <= 400.0 + 1e-9 || v.locked());
            }
        }
    }

    #[test]
    fn the_v8_cars_carry_their_published_gearing() {
        let f = VehicleDef::flatplane_v8();
        assert_eq!(
            f.gear_ratios,
            vec![3.08, 2.19, 1.63, 1.29, 1.03, 0.84, 0.69]
        );
        assert_eq!((f.final_drive, f.mass_kg), (4.44, 1485.0));
        let c = VehicleDef::crossplane_v8();
        assert_eq!(c.gear_ratios, vec![2.66, 1.78, 1.30, 1.00, 0.74, 0.50]);
        assert_eq!((c.final_drive, c.mass_kg), (3.42, 1441.0));
    }

    // --- the elastic driveline (docs/superpowers/specs/2026-11-05-driveline-compliance-design.md)

    fn frictionless() -> VehicleDef {
        VehicleDef {
            drag_cd_a_m2: 0.0,
            rolling_coeff: 0.0,
            shuffle_damping: 0.0,
            driveline_efficiency: 1.0,
            ..fixture()
        }
    }

    /// A locked vehicle in `gear` cruising with the crank at `omega`.
    fn locked_at(def: VehicleDef, gear: usize, omega: f64) -> Vehicle {
        let mut v = Vehicle::new(def);
        v.set_engine_inertia(0.15);
        v.set_gear(gear);
        let g = v.def().overall_ratio(gear);
        v.set_speed(omega * v.def().wheel_radius_m / g);
        v.step(1e-4, omega, 0.0);
        assert!(v.locked());
        v
    }

    /// Frequency of the twist's ringing after a torque step, Hz (zero crossings of twist − mean).
    fn ringing_hz(v: &mut Vehicle, dt: f64) -> f64 {
        let mut twist = Vec::new();
        let mut w = 300.0;
        for _ in 0..(4.0 / dt) as usize {
            let load = v.step(dt, w, 0.0);
            w += (50.0 - load.torque_nm) / 0.15 * dt;
            twist.push(v.twist_rad());
        }
        let mean = twist.iter().sum::<f64>() / twist.len() as f64;
        let crossings = twist
            .windows(2)
            .filter(|p| (p[0] - mean) * (p[1] - mean) < 0.0)
            .count();
        crossings as f64 / 2.0 / (twist.len() as f64 * dt)
    }

    #[test]
    fn the_shuffle_frequency_by_hand() {
        // fixture: J_v1 = 1000·0.3²/12² = 0.625 kg·m², J_e = 0.15: k₁ = (2π·3.5)²/(1/0.15 + 1/0.625)
        // = 483.61/8.2667 = 58.50 N·m/rad, and the twist rings at 3.5 Hz
        let mut v = locked_at(frictionless(), 1, 300.0);
        assert!(
            (v.stiffness_nm_rad() - 58.50).abs() < 0.05,
            "{}",
            v.stiffness_nm_rad()
        );
        let f = ringing_hz(&mut v, 1e-4);
        assert!((f - 3.5).abs() < 0.1, "{f} Hz");
    }

    #[test]
    fn the_higher_gear_shuffles_faster() {
        // gear 2 (G = 8): k = k₁·12²/8² = 131.6 N·m/rad, J_v = 90/64 = 1.406 kg·m²:
        // f = √(131.6·(1/0.15 + 1/1.406))/2π = √971.0/2π = 4.96 Hz
        let mut v = locked_at(frictionless(), 2, 300.0);
        assert!(
            (v.stiffness_nm_rad() - 131.6).abs() < 0.2,
            "{}",
            v.stiffness_nm_rad()
        );
        let f = ringing_hz(&mut v, 1e-4);
        assert!((f - 4.96).abs() < 0.1, "{f} Hz");
    }

    #[test]
    fn a_steady_cruise_twists_by_the_torque_over_k() {
        // the fixture's own road load at a steady crank speed, damped: the spring carries it,
        // twist = T/k, and the car keeps the crank's speed over the ratio
        let mut v = locked_at(fixture(), 1, 300.0);
        let dt = 1e-4;
        let speed = 300.0 * 0.3 / 12.0;
        let t_road = v.road_force_n(speed, 0.0) * 0.3 / (12.0 * 0.9);
        // hold the crank at 300 rad/s for 20 s of a well-damped driveline
        for _ in 0..(20.0 / dt) as usize {
            v.step(dt, 300.0, 0.0);
        }
        let twist = v.twist_rad();
        assert!(
            (twist - t_road / v.stiffness_nm_rad()).abs() < 0.02 * twist.abs(),
            "{twist}"
        );
        assert!((v.speed_m_s() - speed).abs() < 1e-3, "{}", v.speed_m_s());
    }

    #[test]
    fn energy_is_kept_without_damping_or_load() {
        let mut v = locked_at(frictionless(), 1, 300.0);
        let dt = 1e-5;
        let e = |v: &Vehicle, w: f64| {
            0.5 * 0.15 * w * w
                + 0.5 * v.car_inertia_kgm2() * v.car_omega().powi(2)
                + 0.5 * v.stiffness_nm_rad() * v.twist_rad().powi(2)
        };
        // a kick: the crank 5 rad/s ahead of the car, then free
        let mut w = 305.0;
        let e0 = e(&v, w);
        for _ in 0..(10.0 / dt) as usize {
            let load = v.step(dt, w, 0.0);
            w -= load.torque_nm / 0.15 * dt;
        }
        let e1 = e(&v, w);
        assert!(((e1 - e0) / e0).abs() < 1e-3, "{e0} → {e1}");
    }

    #[test]
    fn locking_does_not_step_the_torque() {
        // slipping just under the lock, then locking: the torque the crank feels is continuous
        let mut v = Vehicle::new(fixture());
        v.set_engine_inertia(0.15);
        v.set_gear(1);
        v.set_speed(300.0 * 0.3 / 12.0);
        let before = v.step(1e-4, 300.0 + 0.6, 0.0);
        assert!(!v.locked());
        let after = v.step(1e-4, 300.0 + 0.4, 0.0);
        assert!(v.locked());
        assert!(
            (after.torque_nm - before.torque_nm).abs() < 0.05 * 400.0,
            "{before:?} → {after:?}"
        );
        assert_eq!(after.inertia_kgm2, 0.0);
    }

    #[test]
    fn neutral_has_no_spring() {
        let mut v = Vehicle::new(fixture());
        v.set_engine_inertia(0.15);
        assert_eq!(v.step(1e-4, 300.0, 0.0), Load::default());
        assert_eq!(v.twist_rad(), 0.0);
    }

    #[test]
    fn the_shuffle_fields_round_trip_and_old_files_take_the_defaults() {
        let old: VehicleDef = serde_yaml_ng::from_str("mass_kg: 1200.0\n").unwrap();
        assert_eq!((old.shuffle_hz, old.shuffle_damping), (3.5, 0.15));
        let d = VehicleDef {
            shuffle_hz: 5.0,
            shuffle_damping: 0.2,
            ..fixture()
        };
        let json: VehicleDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: VehicleDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d));
    }

    #[test]
    fn above_the_shuffle_the_crank_turns_free_of_the_car() {
        // A cycle's torque swinging ±10 N·m at 27 Hz (the cycle at 3300 rev/min) on a crank of
        // 0.15 kg·m², locked in 1st (k = 58.50 N·m/rad, J_v = 0.625, ζ = 0): far above the 3.5 Hz
        // shuffle the car's end barely moves, the spring only takes k/ω² off the crank's
        // inertia: |ω_e| = A/(ω·(J_e − k/ω²)) = 10/(169.65·(0.15 − 58.50/28 781)) = 0.3984 rad/s.
        // A rigid car would hold it to 10/(169.65·0.775) = 0.0761.
        let mut v = locked_at(frictionless(), 1, 300.0);
        let (dt, w) = (1e-5, 2.0 * std::f64::consts::PI * 27.0);
        let mut om = 300.0;
        let (mut re, mut im, mut count) = (0.0, 0.0, 0.0);
        let n = (6.0 / dt) as usize;
        for i in 0..n {
            let t = i as f64 * dt;
            let load = v.step(dt, om, 0.0);
            om += (10.0 * (w * t).sin() - load.torque_nm) / (0.15 + load.inertia_kgm2) * dt;
            // the 27 Hz part alone, over the last 27 whole periods (the slow shuffle the start
            // set ringing projects out)
            if t >= 5.0 {
                re += om * (w * t).sin();
                im += om * (w * t).cos();
                count += 1.0;
            }
        }
        let amp = 2.0 * (re * re + im * im).sqrt() / count;
        assert!((amp - 0.3984).abs() < 0.02, "{amp}");
    }
}
