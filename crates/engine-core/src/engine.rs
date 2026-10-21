//! Engine definition (data) and the running engine (0D crank-angle-resolved model).
//!
//! Per cylinder: plenum → intake valve → cylinder → exhaust valve → exhaust waveguides →
//! atmosphere. Mechanics: analytic crank-slider with a lumped flywheel inertia.
//! Model ladder and references: `docs/research/simulador-motor-rust.md`.

use crate::combustion::{AFR_STOICH, charge_energy, wiebe};
use crate::exhaust::{ExhaustDef, ExhaustModel, ExhaustNet, REVERSION_TEMP_K};
use crate::exhaust_fv::ExhaustFv;
use crate::gas::{
    GasVolume, Reservoir, exchange_with_reservoir, exhaust_valve_flow, plan_exchange,
};
use crate::geometry::CylinderGeometry;
use crate::sound::{BURN_DURATION_JITTER, BURN_START_JITTER_DEG, SoundDef, Texture};
use crate::units::{ATM_PA, deg_to_rad, rad_s_to_rpm, rpm_to_rad_s};
use crate::valve::ValveTiming;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Everything that describes an engine. Pure data, (de)serializable.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct EngineDef {
    pub name: String,
    pub geometry: CylinderGeometry,
    /// Number of cylinders (inline for now).
    pub cylinders: usize,
    /// Firing order, 1-based cylinder numbers (`[1, 3, 4, 2]`). Empty = automatic
    /// ([`crate::config::default_firing_order`]).
    #[serde(default)]
    pub firing_order: Vec<usize>,
    pub intake_valve: ValveTiming,
    pub exhaust_valve: ValveTiming,
    pub spark_advance_deg: f64,
    pub burn_duration_deg: f64,
    pub flywheel_inertia_kgm2: f64,
    pub plenum_volume_m3: f64,
    pub throttle_diameter_m: f64,
    /// Fraction of throttle area open at zero pedal (idle air).
    pub idle_throttle: f64,
    pub exhaust: ExhaustDef,
    pub redline_rpm: f64,
    pub starter_torque_nm: f64,
    /// Friction mean effective pressure fit, bar: FMEP = a + b·(N/1000) + c·(N/1000)².
    pub fmep_bar: [f64; 3],
    /// Sound texture levels (noise, intake, mechanics, combustion variability).
    pub sound: SoundDef,
    /// Deceleration fuel cut-off: with the throttle closed above this speed no fuel is injected
    /// (what production ECUs do on overrun); 0 disables it.
    pub overrun_fuel_cut_rpm: f64,
}

impl Default for EngineDef {
    fn default() -> Self {
        Self::single()
    }
}

impl EngineDef {
    /// 500 cc four-stroke single (86 × 86 mm).
    pub fn single() -> Self {
        Self {
            name: "Single 500".into(),
            geometry: CylinderGeometry {
                bore_m: 0.086,
                stroke_m: 0.086,
                rod_m: 0.143,
                compression_ratio: 10.0,
            },
            cylinders: 1,
            firing_order: vec![1],
            intake_valve: ValveTiming {
                open_deg: 345.0,
                duration_deg: 250.0,
                max_lift_m: 0.009,
                diameter_m: 0.034,
            },
            exhaust_valve: ValveTiming {
                open_deg: 135.0,
                duration_deg: 245.0,
                max_lift_m: 0.008,
                diameter_m: 0.029,
            },
            spark_advance_deg: 25.0,
            burn_duration_deg: 55.0,
            flywheel_inertia_kgm2: 0.12,
            plenum_volume_m3: 1.5e-3,
            throttle_diameter_m: 0.040,
            idle_throttle: 0.0015,
            exhaust: ExhaustDef {
                primary_length_m: 0.8,
                primary_diameter_m: 0.040,
                collectors: vec![],
                collector_length_m: 0.05,
                collector_diameter_m: 0.040,
                tail_length_m: 1.0,
                tail_diameter_m: 0.045,
                muffler_transmission: 0.6,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.003,
            },
            redline_rpm: 7500.0,
            starter_torque_nm: 25.0,
            fmep_bar: [0.97, 0.15, 0.05],
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1500.0,
        }
    }

    /// 2.0 L inline four, firing order 1-3-4-2.
    pub fn inline4() -> Self {
        Self {
            name: "Inline-4 2.0".into(),
            cylinders: 4,
            firing_order: vec![1, 3, 4, 2],
            flywheel_inertia_kgm2: 0.15,
            plenum_volume_m3: 4.0e-3,
            throttle_diameter_m: 0.060,
            idle_throttle: 0.0055,
            exhaust: ExhaustDef::default(),
            redline_rpm: 7000.0,
            starter_torque_nm: 60.0,
            ..Self::single()
        }
    }

    /// Lexus LFA, Toyota 1LR-GUE: 4.8 L 72° V10, 88 × 79 mm, CR 12.0, even firing,
    /// 9,000 rpm redline (Toyota LR engine, Wikipedia; PistonHeads LFA spec sheet).
    /// Not public, estimated: rod length, cam timing, valve sizes (4 valves/cyl lumped into
    /// one equivalent valve each), inertia. Firing order: placeholder (Audi 5.2 V10 order),
    /// pending confirmation. Drawn inline until V layouts land.
    pub fn lfa_v10() -> Self {
        Self {
            name: "Lexus LFA V10".into(),
            geometry: CylinderGeometry {
                bore_m: 0.088,
                stroke_m: 0.079,
                rod_m: 0.140,
                compression_ratio: 12.0,
            },
            cylinders: 10,
            firing_order: vec![1, 6, 5, 10, 2, 7, 3, 8, 4, 9],
            intake_valve: ValveTiming {
                open_deg: 340.0,
                duration_deg: 270.0,
                max_lift_m: 0.011,
                diameter_m: 0.046,
            },
            exhaust_valve: ValveTiming {
                open_deg: 125.0,
                duration_deg: 265.0,
                max_lift_m: 0.010,
                diameter_m: 0.040,
            },
            spark_advance_deg: 28.0,
            burn_duration_deg: 45.0,
            flywheel_inertia_kgm2: 0.09,
            plenum_volume_m3: 8.0e-3,
            throttle_diameter_m: 0.120,
            idle_throttle: 0.003,
            // Equal-length primaries, one 5-into-1 collector per bank (odd/even numbering
            // per bank is an assumption), dual tail pipes. Lengths and diameters estimated.
            exhaust: ExhaustDef {
                primary_length_m: 0.6,
                primary_diameter_m: 0.042,
                collectors: vec![vec![1, 3, 5, 7, 9], vec![2, 4, 6, 8, 10]],
                collector_length_m: 0.3,
                collector_diameter_m: 0.065,
                tail_length_m: 1.2,
                tail_diameter_m: 0.060,
                // Sport muffler with bypass valves: less damping than a road car.
                muffler_transmission: 0.7,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.010,
            },
            redline_rpm: 9000.0,
            starter_torque_nm: 150.0,
            // Low-friction racing build (dry sump, titanium valvetrain): estimated.
            fmep_bar: [0.8, 0.10, 0.015],
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1500.0,
        }
    }

    pub fn displacement_m3(&self) -> f64 {
        self.geometry.displacement() * self.cylinders as f64
    }

    /// Crank angle (deg, within 720) at which each cylinder's cycle starts, in cylinder order.
    pub fn phase_offsets_deg(&self) -> Vec<f64> {
        let n = self.cylinders.max(1);
        let order: Vec<usize> = if self.firing_order.len() == n {
            self.firing_order.clone()
        } else {
            crate::config::default_firing_order(n)
        };
        let mut offsets = vec![0.0; n];
        for (slot, &cyl) in order.iter().enumerate() {
            if (1..=n).contains(&cyl) {
                offsets[cyl - 1] = slot as f64 * 720.0 / n as f64;
            }
        }
        offsets
    }
}

/// Driver inputs, set every block from the UI.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Controls {
    /// 0..=1 pedal.
    pub throttle: f64,
    pub ignition: bool,
    pub starter: bool,
    /// External load torque on the crank, N·m (positive opposes rotation).
    pub load_torque_nm: f64,
    /// Dyno speed hold: when set, crank speed is forced to this rpm.
    pub hold_rpm: Option<f64>,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            ignition: true,
            starter: false,
            load_torque_nm: 0.0,
            hold_rpm: None,
        }
    }
}

/// Public read-only view of one cylinder, for drawing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderView {
    pub cycle_deg: f64,
    pub piston_m: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub intake_lift_m: f64,
    pub exhaust_lift_m: f64,
    pub burning: bool,
}

/// Ignition delay between spark and the start of the Wiebe burn, degrees.
const IGNITION_DELAY_DEG: f64 = 5.0;
/// Combustion efficiency (incomplete burn, dissociation) — a lumped tuning constant.
const COMBUSTION_EFFICIENCY: f64 = 0.85;
/// Cycle-to-cycle variation of released energy (±), drives the "lumpy" idle.
const CYCLE_VARIATION: f64 = 0.04;
/// Lumped in-cylinder wall heat-transfer coefficient, W/(m²·K). A constant stand-in
/// for Woschni (SAE 670931), which is a later phase.
const WALL_HTC: f64 = 350.0;
const WALL_TEMP_K: f64 = 400.0;
const AMBIENT_K: f64 = 300.0;
const DISCHARGE_CD: f64 = 0.8;
const TORQUE_AVG_TAU_S: f64 = 0.15;
/// Effective ring leak area as a fraction of piston area. 1e-4 bleeds a stopped 6 bar
/// charge in ~0.3 s and costs ~1–2 % of the trapped mass per cycle at WOT (typical
/// blow-by is ~1 % of intake flow; Heywood, ch. 8).
const BLOWBY_AREA_FRACTION: f64 = 1e-4;
/// Fraction of the equalising mass a valve may move per substep (see `step`).
const VALVE_RELAX: f64 = 0.5;

#[derive(Debug, Clone)]
struct Burn {
    start_deg: f64,
    duration_deg: f64,
    energy_j: f64,
    released: f64,
}

#[derive(Debug, Clone)]
struct CylinderState {
    offset_deg: f64,
    gas: GasVolume,
    trapped_kg: f64,
    burn: Option<Burn>,
    cycle_deg: f64,
}

#[derive(Debug, Clone)]
pub struct Engine {
    def: EngineDef,
    cyl: Vec<CylinderState>,
    plenum: GasVolume,
    atm: Reservoir,
    /// Crank angle, radians within [0, 4π).
    theta: f64,
    omega: f64,
    torque_avg: f64,
    combustions: u64,
    rng: u64,
    /// Exhaust waveguide network, built for the current step size.
    net: Option<ExhaustNet>,
    /// Non-linear finite-volume exhaust, when `exhaust.model` is `fv`.
    fv: Option<ExhaustFv>,
    flows: Vec<f64>,
    texture: Texture,
}

/// Crank angle `a` (deg) crossed in the step (prev, cur], on the 720° circle.
fn crossed(prev: f64, cur: f64, a: f64) -> bool {
    let span = (cur - prev).rem_euclid(720.0);
    let to_a = (a - prev).rem_euclid(720.0);
    to_a > 0.0 && to_a <= span
}

impl Engine {
    pub fn new(def: EngineDef, initial_rpm: f64, seed: u64) -> Self {
        let g = def.geometry;
        let cyl = def
            .phase_offsets_deg()
            .into_iter()
            .map(|offset_deg| {
                let cycle_deg = (0.0 - offset_deg).rem_euclid(720.0);
                CylinderState {
                    offset_deg,
                    gas: GasVolume::at(ATM_PA, AMBIENT_K, g.volume(deg_to_rad(cycle_deg))),
                    trapped_kg: 0.0,
                    burn: None,
                    cycle_deg,
                }
            })
            .collect();
        Self {
            plenum: GasVolume::at(ATM_PA, AMBIENT_K, def.plenum_volume_m3),
            def,
            cyl,
            atm: Reservoir {
                pressure_pa: ATM_PA,
                temperature_k: AMBIENT_K,
            },
            theta: 0.0,
            omega: rpm_to_rad_s(initial_rpm),
            torque_avg: 0.0,
            combustions: 0,
            rng: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            net: None,
            fv: None,
            flows: Vec::new(),
            texture: Texture::new(seed ^ 0x7E57),
        }
    }

    pub fn def(&self) -> &EngineDef {
        &self.def
    }

    /// Apply a new definition in place when the structure (cylinder count, geometry,
    /// volumes) is unchanged — timing, spark, throttle, limits. Returns false when the
    /// engine must be rebuilt instead.
    pub fn retune(&mut self, def: EngineDef) -> bool {
        let d = &self.def;
        let same = d.cylinders == def.cylinders
            && d.firing_order == def.firing_order
            && d.geometry == def.geometry
            && d.plenum_volume_m3 == def.plenum_volume_m3
            && d.exhaust == def.exhaust;
        if same {
            self.def = def;
        }
        same
    }

    /// xorshift64*: uniform in [-1, 1). Seeded, so the sim stays deterministic.
    fn noise(&mut self) -> f64 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let x = self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D);
        (x >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }

    fn throttle_area(&self, throttle: f64) -> f64 {
        let t = throttle.clamp(0.0, 1.0);
        let open =
            self.def.idle_throttle + (1.0 - self.def.idle_throttle) * (1.0 - (t * PI * 0.5).cos());
        DISCHARGE_CD * PI * 0.25 * self.def.throttle_diameter_m.powi(2) * open
    }

    /// Friction torque from a Chen–Flynn-style FMEP fit (SAE 650733 form):
    /// FMEP [bar] = a + b·(N/1000) + c·(N/1000)² (default 0.97, 0.15, 0.05); T = FMEP·Vd / 4π.
    /// tanh fades it to zero at standstill so the crank does not reverse.
    fn friction_torque(&self) -> f64 {
        let k = rad_s_to_rpm(self.omega) / 1000.0;
        let [a, b, c] = self.def.fmep_bar;
        let fmep = (a + b * k + c * k * k) * 1e5;
        fmep * self.def.displacement_m3() / (4.0 * PI) * (self.omega / 5.0).tanh()
    }

    /// Advance by `dt` seconds; returns the raw acoustic source sample (Pa).
    pub fn step(&mut self, dt: f64, c: &Controls) -> f64 {
        let g = self.def.geometry;
        let crank_deg = self.theta.to_degrees();
        let spark_deg = 720.0 - self.def.spark_advance_deg;
        let ivc =
            (self.def.intake_valve.open_deg + self.def.intake_valve.duration_deg).rem_euclid(720.0);
        let evc = (self.def.exhaust_valve.open_deg + self.def.exhaust_valve.duration_deg)
            .rem_euclid(720.0);
        let mut valve_closings = 0;
        let overrun_cut = self.def.overrun_fuel_cut_rpm > 0.0
            && c.throttle <= 0.01
            && self.rpm() > self.def.overrun_fuel_cut_rpm;
        let fire_ok = c.ignition && self.rpm() < self.def.redline_rpm && !overrun_cut;
        let wall_area = 2.0 * g.piston_area();
        match self.def.exhaust.model {
            ExhaustModel::Waveguide => {
                self.fv = None;
                if self.net.as_ref().is_none_or(|n| n.dt != dt) {
                    self.net = Some(ExhaustNet::new(&self.def.exhaust, self.cyl.len(), dt));
                }
            }
            ExhaustModel::Fv => {
                self.net = None;
                if self.fv.as_ref().is_none_or(|n| n.dt != dt) {
                    self.fv = Some(ExhaustFv::new(&self.def.exhaust, self.cyl.len(), dt));
                }
            }
        }
        if self.flows.len() != self.cyl.len() {
            self.flows = vec![0.0; self.cyl.len()];
        }
        let blowby_area = BLOWBY_AREA_FRACTION * g.piston_area();

        let mut gas_torque = 0.0;
        let z_port = self.net.as_ref().map_or(1.0, |n| n.port_impedance());
        for i in 0..self.cyl.len() {
            let arriving = self.net.as_ref().map_or(0.0, |n| n.arriving(i));
            let prev = self.cyl[i].cycle_deg;
            let cur = (crank_deg - self.cyl[i].offset_deg).rem_euclid(720.0);
            let theta_c = deg_to_rad(cur);

            // Piston motion: adiabatic work p·dV.
            let v_new = g.volume(theta_c);
            let cs = &mut self.cyl[i];
            let p = cs.gas.pressure();
            cs.gas.energy_j -= p * (v_new - cs.gas.volume_m3);
            cs.gas.set_volume(v_new);

            // Valve and outlet flows.
            // Both valve flows are planned from the same state (Jacobi) and each may close
            // at most half the pressure gap: during overlap, sequential full equalisation
            // with plenum then runner ping-pongs every substep (loud numerical chatter).
            let a_in = self.def.intake_valve.flow_area(cur);
            let a_ex = self.def.exhaust_valve.flow_area(cur);
            let f_in =
                (a_in > 0.0).then(|| plan_exchange(&self.plenum, &cs.gas, a_in, dt, VALVE_RELAX));
            // The exhaust port and its primary pipe are solved together: the flow raises the
            // pipe pressure at the port by z·ṁ (see `gas::port_flow`).
            let f_ex = (a_ex > 0.0).then(|| match &self.fv {
                // FV: the first pipe cell is a real volume — a planned two-volume exchange.
                Some(fv) => plan_exchange(&cs.gas, &fv.port_volume(i), a_ex, dt, VALVE_RELAX),
                None => exhaust_valve_flow(
                    &cs.gas,
                    a_ex,
                    arriving,
                    z_port,
                    REVERSION_TEMP_K,
                    dt,
                    VALVE_RELAX,
                ),
            });
            if let Some(f) = f_in {
                f.apply(&mut self.plenum, &mut cs.gas);
            }
            self.flows[i] = f_ex.map_or(0.0, |f| {
                f.apply_to(&mut cs.gas);
                f.dm / dt
            });
            if let (Some(fv), Some(f)) = (&mut self.fv, f_ex) {
                fv.add_valve_flow(i, f.dm, f.h);
            }
            // Blow-by past the rings into the (vented) crankcase. Small while running, but it
            // is what lets a charge trapped at a stop bleed off so the starter can turn it.
            exchange_with_reservoir(&mut cs.gas, &self.atm, blowby_area, dt);

            // Combustion events.
            if crossed(prev, cur, ivc) || crossed(prev, cur, evc) {
                valve_closings += 1;
            }
            if crossed(prev, cur, ivc) {
                cs.trapped_kg = cs.gas.mass_kg;
            }
            let mut new_burn = false;
            if crossed(prev, cur, spark_deg) && fire_ok && cs.trapped_kg > 0.0 {
                new_burn = true;
            }
            if let Some(b) = &mut cs.burn {
                // Signed angle since the burn start, in (−360, 360]: the burn is created at
                // the spark, a few degrees *before* it starts, and must not wrap to "done".
                let mut since = (cur - b.start_deg).rem_euclid(720.0);
                if since > 360.0 {
                    since -= 720.0;
                }
                let x = wiebe(since, b.duration_deg);
                cs.gas.energy_j += b.energy_j * (x - b.released);
                b.released = x;
                if since > 2.0 * b.duration_deg {
                    cs.burn = None;
                }
            }

            // Wall heat loss (simplified Newton cooling).
            let t_gas = cs.gas.temperature();
            let area = wall_area + PI * g.bore_m * (v_new / g.piston_area());
            let q = (WALL_HTC * area * (t_gas - WALL_TEMP_K) * dt).min(0.5 * cs.gas.energy_j);
            cs.gas.energy_j -= q;

            gas_torque += (cs.gas.pressure() - ATM_PA) * g.piston_area() * g.dpos_dtheta(theta_c);

            cs.cycle_deg = cur;

            if new_burn {
                let trapped = cs.trapped_kg;
                let var = 1.0 + CYCLE_VARIATION * self.noise();
                let j = self.def.sound.combustion_jitter;
                let start_jitter = j * BURN_START_JITTER_DEG * self.noise();
                let duration =
                    self.def.burn_duration_deg * (1.0 + j * BURN_DURATION_JITTER * self.noise());
                let cs = &mut self.cyl[i];
                cs.burn = Some(Burn {
                    start_deg: (spark_deg + IGNITION_DELAY_DEG + start_jitter).rem_euclid(720.0),
                    duration_deg: duration,
                    energy_j: charge_energy(trapped, AFR_STOICH, COMBUSTION_EFFICIENCY * var),
                    released: 0.0,
                });
                self.combustions += 1;
            }
        }
        // Acoustic source: what the tail pipes radiate.
        let radiated = match (&mut self.net, &mut self.fv) {
            (_, Some(fv)) => fv.step(),
            (Some(net), None) => net.step(&self.flows),
            (None, None) => 0.0,
        };
        let throttle_area = self.throttle_area(c.throttle);
        let bore_area = self.throttle_area(1.0);
        let throttle_dm = exchange_with_reservoir(&mut self.plenum, &self.atm, throttle_area, dt);

        // Texture: jet noise at the tails, induction sound, valvetrain clicks.
        let snd = &self.def.sound;
        let rpm = rad_s_to_rpm(self.omega);
        for _ in 0..valve_closings {
            self.texture.valve_closed(rpm, snd.mechanical);
        }
        let audio = radiated
            + self.texture.flow_noise(radiated, snd.flow_noise, dt)
            + self
                .texture
                .intake(-throttle_dm / dt, throttle_area, bore_area, snd.intake, dt)
            + self.texture.clicks(dt);

        // Crank dynamics.
        let starter = if c.starter && rad_s_to_rpm(self.omega) < 600.0 {
            self.def.starter_torque_nm
        } else {
            0.0
        };
        let net = gas_torque + starter - self.friction_torque() - c.load_torque_nm;
        let brake = gas_torque - self.friction_torque();
        let k = (dt / TORQUE_AVG_TAU_S).min(1.0);
        self.torque_avg += (brake - self.torque_avg) * k;
        match c.hold_rpm {
            Some(rpm) => self.omega = rpm_to_rad_s(rpm),
            None => self.omega = (self.omega + net / self.def.flywheel_inertia_kgm2 * dt).max(0.0),
        }
        self.theta = (self.theta + self.omega * dt).rem_euclid(4.0 * PI);
        audio
    }

    pub fn rpm(&self) -> f64 {
        rad_s_to_rpm(self.omega)
    }
    /// Crank angle in degrees within the 720° cycle.
    pub fn crank_deg(&self) -> f64 {
        self.theta.to_degrees()
    }
    /// Brake torque (gas minus friction) averaged over ~0.15 s, N·m.
    pub fn torque_nm(&self) -> f64 {
        self.torque_avg
    }
    pub fn power_w(&self) -> f64 {
        self.torque_avg * self.omega
    }
    pub fn plenum_pressure_pa(&self) -> f64 {
        self.plenum.pressure()
    }
    /// The exhaust backend running (None before the first step).
    pub fn exhaust_model(&self) -> Option<ExhaustModel> {
        match (&self.net, &self.fv) {
            (_, Some(_)) => Some(ExhaustModel::Fv),
            (Some(_), None) => Some(ExhaustModel::Waveguide),
            (None, None) => None,
        }
    }

    /// Gauge pressure in cylinder `i`'s exhaust port (waves standing at the valve), Pa.
    pub fn exhaust_port_gauge(&self, i: usize) -> f64 {
        match (&self.net, &self.fv) {
            (_, Some(fv)) => fv.port_state(i).0 - ATM_PA,
            (Some(n), None) => n.valve_pressure(i),
            (None, None) => 0.0,
        }
    }
    pub fn combustions(&self) -> u64 {
        self.combustions
    }
    pub fn cylinder(&self, i: usize) -> CylinderView {
        let cs = &self.cyl[i];
        let g = self.def.geometry;
        CylinderView {
            cycle_deg: cs.cycle_deg,
            piston_m: g.piston_position(deg_to_rad(cs.cycle_deg)),
            pressure_pa: cs.gas.pressure(),
            temperature_k: cs.gas.temperature(),
            intake_lift_m: self.def.intake_valve.lift(cs.cycle_deg),
            exhaust_lift_m: self.def.exhaust_valve.lift(cs.cycle_deg),
            burning: cs
                .burn
                .as_ref()
                .is_some_and(|b| b.released > 0.0 && b.released < 0.99),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const DT: f64 = 1.0 / 48_000.0 / 4.0;

    fn run(e: &mut Engine, seconds: f64, c: &Controls) -> Vec<f64> {
        let n = (seconds / DT) as usize;
        (0..n).map(|_| e.step(DT, c)).collect()
    }

    #[test]
    fn displacement_and_even_firing_offsets() {
        let d = EngineDef::inline4();
        // 4 × 4.99556e-4 = 1.99822e-3 m³
        assert_relative_eq!(d.displacement_m3(), 1.99822e-3, max_relative = 1e-5);
        // order 1-3-4-2 every 180°: cyl1 0°, cyl3 180°, cyl4 360°, cyl2 540°
        assert_eq!(d.phase_offsets_deg(), vec![0.0, 540.0, 180.0, 360.0]);
        assert_eq!(EngineDef::single().phase_offsets_deg(), vec![0.0]);
    }

    #[test]
    fn idles_stably_without_nan() {
        let mut e = Engine::new(EngineDef::single(), 900.0, 1);
        let audio = run(&mut e, 3.0, &Controls::default());
        assert!(audio.iter().all(|x| x.is_finite()));
        let rpm = e.rpm();
        assert!((400.0..2000.0).contains(&rpm), "idle rpm {rpm}");
    }

    #[test]
    fn throttle_revs_it_up() {
        let mut e = Engine::new(EngineDef::single(), 900.0, 1);
        run(&mut e, 1.5, &Controls::default());
        let idle = e.rpm();
        run(
            &mut e,
            1.0,
            &Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
        assert!(e.rpm() > idle + 1000.0, "idle {idle} → {}", e.rpm());
    }

    #[test]
    fn ignition_off_stalls() {
        let mut e = Engine::new(EngineDef::single(), 1500.0, 1);
        run(
            &mut e,
            5.0,
            &Controls {
                ignition: false,
                ..Controls::default()
            },
        );
        assert!(e.rpm() < 100.0, "rpm {}", e.rpm());
    }

    #[test]
    fn starter_cranks_from_rest() {
        let mut e = Engine::new(EngineDef::single(), 0.0, 1);
        run(
            &mut e,
            1.0,
            &Controls {
                starter: true,
                ignition: false,
                ..Controls::default()
            },
        );
        assert!(e.rpm() > 100.0, "cranking rpm {}", e.rpm());
    }

    #[test]
    fn motored_compression_pressure_is_plausible() {
        // Ideal adiabatic from 1 bar: 10^1.4 = 25 bar. Late IVC, heat loss and flow lower it.
        let mut e = Engine::new(EngineDef::single(), 1000.0, 1);
        let c = Controls {
            throttle: 1.0,
            ignition: false,
            hold_rpm: Some(1000.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &c);
        let mut peak: f64 = 0.0;
        for _ in 0..(0.2 / DT) as usize {
            e.step(DT, &c);
            peak = peak.max(e.cylinder(0).pressure_pa);
        }
        assert!((8e5..26e5).contains(&peak), "peak {peak}");
    }

    #[test]
    fn wot_torque_is_in_range_for_a_500cc_single() {
        // Real 500 cc singles make ~35–50 N·m; accept a wide band for a 0D model.
        let mut e = Engine::new(EngineDef::single(), 3000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 1.5, &c);
        let t = e.torque_nm();
        assert!((15.0..80.0).contains(&t), "torque {t}");
    }

    #[test]
    fn fires_once_per_two_revolutions_per_cylinder() {
        let mut e = Engine::new(EngineDef::inline4(), 2000.0, 1);
        let c = Controls {
            throttle: 0.3,
            hold_rpm: Some(2000.0),
            ..Controls::default()
        };
        let before = e.combustions();
        run(&mut e, 1.2, &c);
        // 2000 rpm → 1000 cycles/min → 16.67 cycles/s × 4 cylinders × 1.2 s = 80
        let n = e.combustions() - before;
        assert!((78..=82).contains(&n), "combustions {n}");
    }

    #[test]
    fn deterministic_for_a_seed() {
        let mut a = Engine::new(EngineDef::single(), 900.0, 7);
        let mut b = Engine::new(EngineDef::single(), 900.0, 7);
        let c = Controls {
            throttle: 0.4,
            ..Controls::default()
        };
        assert_eq!(run(&mut a, 0.3, &c), run(&mut b, 0.3, &c));
    }

    #[test]
    fn rev_limiter_holds_redline() {
        let mut d = EngineDef::single();
        d.redline_rpm = 4000.0;
        let mut e = Engine::new(d, 3000.0, 1);
        run(
            &mut e,
            3.0,
            &Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
        assert!(e.rpm() < 4400.0, "rpm {}", e.rpm());
    }

    #[test]
    fn combustion_peaks_after_tdc_and_is_visible() {
        // Regression: the burn used to wrap to "finished" one step after the spark and
        // release all heat at once (≈127 bar at TDC). A spark-ignition peak sits
        // ~5–25° after TDC at 40–90 bar at WOT (Heywood, ch. 9).
        let mut e = Engine::new(EngineDef::single(), 3000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &c);
        let (mut peak, mut peak_deg, mut saw_burning) = (0.0f64, 0.0, false);
        for _ in 0..(0.04 / DT) as usize {
            e.step(DT, &c);
            let v = e.cylinder(0);
            saw_burning |= v.burning;
            if v.pressure_pa > peak {
                peak = v.pressure_pa;
                peak_deg = v.cycle_deg;
            }
        }
        assert!(saw_burning, "burn never visible");
        assert!((3.0..30.0).contains(&peak_deg), "peak at {peak_deg}°");
        assert!((30e5..95e5).contains(&peak), "peak {peak}");
    }

    #[test]
    fn crossed_detects_angles_including_wrap() {
        assert!(crossed(10.0, 20.0, 15.0));
        assert!(crossed(10.0, 20.0, 20.0));
        assert!(!crossed(10.0, 20.0, 10.0));
        assert!(!crossed(10.0, 20.0, 25.0));
        assert!(crossed(715.0, 3.0, 0.0));
        assert!(crossed(715.0, 3.0, 718.0));
        assert!(!crossed(715.0, 3.0, 5.0));
        assert!(!crossed(10.0, 10.0, 10.0));
    }

    #[test]
    fn friction_matches_fmep_fit() {
        // 1000 rpm: FMEP = 0.97 + 0.15 + 0.05 = 1.17 bar; T = 1.17e5 · 4.99556e-4 / 4π = 4.6513 N·m
        let e = Engine::new(EngineDef::single(), 1000.0, 1);
        assert_relative_eq!(e.friction_torque(), 4.6513, max_relative = 1e-4);
        // 3000 rpm: 0.97 + 0.45 + 0.45 = 1.87 bar → 7.4341 N·m
        let e = Engine::new(EngineDef::single(), 3000.0, 1);
        assert_relative_eq!(e.friction_torque(), 7.4341, max_relative = 1e-4);
        assert_eq!(
            Engine::new(EngineDef::single(), 0.0, 1).friction_torque(),
            0.0
        );
    }

    #[test]
    fn throttle_area_spans_idle_to_full_bore() {
        let e = Engine::new(EngineDef::single(), 900.0, 1);
        // full: 0.8 · π/4 · 0.04² = 1.00531e-3 m²; idle = 0.0015 of that
        assert_relative_eq!(e.throttle_area(1.0), 1.00531e-3, max_relative = 1e-5);
        assert_relative_eq!(
            e.throttle_area(0.0),
            0.0015 * 1.00531e-3,
            max_relative = 1e-5
        );
        // half pedal: 0.0015 + 0.9985 · (1 − cos 45°) = 0.29395
        assert_relative_eq!(
            e.throttle_area(0.5),
            0.29395 * 1.00531e-3,
            max_relative = 1e-4
        );
    }

    #[test]
    fn noise_is_uniform_in_unit_range_and_seeded() {
        let mut e = Engine::new(EngineDef::single(), 900.0, 3);
        let xs: Vec<f64> = (0..20_000).map(|_| e.noise()).collect();
        assert!(xs.iter().all(|x| (-1.0..1.0).contains(x)));
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        assert!(mean.abs() < 0.02, "mean {mean}");
        let var = xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64;
        assert!((var - 1.0 / 3.0).abs() < 0.01, "var {var}"); // uniform(-1,1): 1/3
        assert!(xs.iter().any(|x| *x < -0.9) && xs.iter().any(|x| *x > 0.9));
        let mut f = Engine::new(EngineDef::single(), 900.0, 3);
        assert_eq!(f.noise(), xs[0]);
    }

    #[test]
    fn retune_keeps_the_engine_running() {
        let mut e = Engine::new(EngineDef::single(), 2000.0, 1);
        let c = Controls {
            throttle: 0.5,
            hold_rpm: Some(2000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        let (n, t) = (e.combustions(), e.torque_nm());
        let mut d = EngineDef::single();
        d.spark_advance_deg = 30.0;
        assert!(e.retune(d.clone()), "same structure retunes in place");
        assert_eq!(e.def().spark_advance_deg, 30.0);
        assert_eq!(e.combustions(), n);
        assert_eq!(e.torque_nm(), t);
        assert!(
            !e.retune(EngineDef::inline4()),
            "different structure needs a rebuild"
        );
    }

    #[test]
    fn accessors_report_running_state() {
        let mut e = Engine::new(EngineDef::single(), 1200.0, 1);
        let c = Controls {
            hold_rpm: Some(1200.0),
            ..Controls::default()
        };
        let before = e.crank_deg();
        e.step(DT, &c);
        // 1200 rpm = 7200 °/s → 7200 · DT degrees per step
        assert_relative_eq!(e.crank_deg() - before, 7200.0 * DT, max_relative = 1e-9);
        run(&mut e, 0.5, &c);
        assert_relative_eq!(
            e.power_w(),
            e.torque_nm() * rpm_to_rad_s(1200.0),
            max_relative = 1e-12
        );
        assert!(e.power_w() != 0.0);
        let p = e.plenum_pressure_pa();
        assert!(
            (0.1e5..0.9e5).contains(&p),
            "closed throttle → manifold vacuum, got {p}"
        );
    }

    #[test]
    fn retune_refuses_any_structural_change() {
        let base = EngineDef::single();
        let mut changes = Vec::new();
        let mut d = base.clone();
        d.geometry.bore_m = 0.09;
        changes.push(d);
        let mut d = base.clone();
        d.plenum_volume_m3 = 2e-3;
        changes.push(d);
        let mut d = base.clone();
        d.exhaust.primary_length_m = 1.2;
        changes.push(d);
        let mut d = base.clone();
        d.exhaust.collectors = vec![vec![1]];
        changes.push(d);
        for d in changes {
            let mut e = Engine::new(base.clone(), 900.0, 1);
            assert!(!e.retune(d), "structural change must rebuild");
        }
    }

    #[test]
    fn lfa_v10_matches_published_specs() {
        let d = EngineDef::lfa_v10();
        // 10 · π/4 · 0.088² · 0.079 = 4.8048e-3 m³ (published: 4,805 cc)
        assert_relative_eq!(d.displacement_m3(), 4.8048e-3, max_relative = 1e-4);
        assert_eq!(d.geometry.compression_ratio, 12.0);
        let mut offsets = d.phase_offsets_deg();
        offsets.sort_by(f64::total_cmp);
        let expected: Vec<f64> = (0..10).map(|k| 72.0 * k as f64).collect();
        assert_eq!(offsets, expected, "even firing every 72°");
    }

    #[test]
    fn lfa_v10_idles_and_makes_big_power() {
        let mut e = Engine::new(EngineDef::lfa_v10(), 1000.0, 1);
        run(&mut e, 2.0, &Controls::default());
        assert!((500.0..1800.0).contains(&e.rpm()), "idle {}", e.rpm());
        // Published: 480 N·m @ 6,800 rpm. A 0D model without tuned headers lands lower.
        let mut e = Engine::new(EngineDef::lfa_v10(), 6800.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(6800.0),
            ..Controls::default()
        };
        run(&mut e, 1.0, &c);
        assert!(
            (280.0..600.0).contains(&e.torque_nm()),
            "torque {}",
            e.torque_nm()
        );
    }

    /// Stall with ignition off after `idle_s` of running; returns the stalled engine.
    fn stalled(def: EngineDef, seed: u64, idle_s: f64) -> Engine {
        let mut e = Engine::new(def, 900.0, seed);
        run(&mut e, idle_s, &Controls::default());
        run(
            &mut e,
            4.0,
            &Controls {
                ignition: false,
                ..Controls::default()
            },
        );
        assert!(e.rpm() < 1.0, "did not stall: {}", e.rpm());
        e
    }

    #[test]
    fn compression_bleeds_off_when_stopped() {
        // Regression: a cylinder sealed on its compression stroke kept ~6 bar forever, a gas
        // spring the starter could not push through. Rings leak (blow-by), so it decays.
        let mut e = stalled(EngineDef::inline4(), 0, 0.5);
        let max_p = |e: &Engine| {
            (0..4)
                .map(|i| e.cylinder(i).pressure_pa)
                .fold(0.0, f64::max)
        };
        // The starter pushes cylinder 1 up its compression stroke and stalls against it.
        run(
            &mut e,
            0.4,
            &Controls {
                ignition: false,
                starter: true,
                ..Controls::default()
            },
        );
        assert!(
            max_p(&e) > 4e5,
            "precondition: trapped charge, got {}",
            max_p(&e)
        );
        run(
            &mut e,
            2.0,
            &Controls {
                ignition: false,
                ..Controls::default()
            },
        );
        assert!(max_p(&e) < 1.5e5, "still {} Pa", max_p(&e));
    }

    #[test]
    fn restarts_from_any_stall_position() {
        // Seeds 0, 7, 9 stalled the I4 against compression (found by sweeping 60 positions).
        for (def, seeds) in [
            (EngineDef::inline4(), vec![0u64, 7]),
            (EngineDef::single(), vec![38]),
            (EngineDef::lfa_v10(), vec![0]),
        ] {
            for k in seeds {
                let mut e = stalled(def.clone(), k, 0.5 + 0.013 * k as f64);
                run(
                    &mut e,
                    1.5,
                    &Controls {
                        starter: true,
                        ..Controls::default()
                    },
                );
                run(&mut e, 1.5, &Controls::default());
                assert!(
                    e.rpm() > 400.0,
                    "{} seed {k}: {} rpm after start",
                    def.name,
                    e.rpm()
                );
            }
        }
    }

    #[test]
    fn exhaust_source_has_no_sample_to_sample_oscillation() {
        // Regression: the valve-to-pipe coupling used last step's wave; near equal pressures
        // the orifice slope made a period-2 instability (±20 kPa alternating every sample),
        // heard as hiss and making closed throttle louder than WOT.
        for (def, throttle) in [
            (EngineDef::single(), 0.0),
            (EngineDef::single(), 1.0),
            (EngineDef::lfa_v10(), 0.0),
        ] {
            let mut e = Engine::new(def.clone(), 3000.0, 1);
            let c = Controls {
                throttle,
                hold_rpm: Some(3000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let xs = run(&mut e, 0.1, &c);
            // second difference catches alternation; smooth waves give small values
            let worst = xs
                .windows(3)
                .map(|w| (w[2] - 2.0 * w[1] + w[0]).abs())
                .fold(0.0, f64::max);
            assert!(
                worst < 3000.0,
                "{} throttle {throttle}: second difference {worst} Pa",
                def.name
            );
        }
    }

    #[test]
    fn exhaust_port_never_goes_below_vacuum() {
        // Regression (review of PR #4): linear waves superposed to −154 kPa gauge at the port
        // (−53 kPa absolute) at closed throttle, 7000 rpm — pulling gas out of a 0.06 bar cylinder.
        let mut e = Engine::new(EngineDef::lfa_v10(), 7000.0, 1);
        let c = Controls {
            throttle: 0.0,
            hold_rpm: Some(7000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        let (mut worst, mut swing) = (f64::MAX, 0.0f64);
        for _ in 0..(0.1 / DT) as usize {
            e.step(DT, &c);
            for i in 0..10 {
                worst = worst.min(ATM_PA + e.exhaust_port_gauge(i));
                swing = swing.max(e.exhaust_port_gauge(i).abs());
            }
        }
        assert!(worst > 0.0, "absolute port pressure {worst} Pa");
        assert!(swing > 1e4, "port waves expected, max |gauge| {swing}");
    }

    /// Crank angle (cycle °) of the pressure peak of each of the next `n` cycles, cylinder 1.
    fn peak_angles(e: &mut Engine, c: &Controls, n: usize) -> Vec<f64> {
        let mut out = Vec::new();
        let (mut best, mut best_deg, mut last) = (0.0, 0.0, e.cylinder(0).cycle_deg);
        while out.len() < n + 1 {
            e.step(DT, c);
            let v = e.cylinder(0);
            if v.cycle_deg < last {
                out.push(best_deg);
                best = 0.0;
            }
            last = v.cycle_deg;
            if v.pressure_pa > best && (v.cycle_deg < 90.0 || v.cycle_deg > 630.0) {
                best = v.pressure_pa;
                best_deg = if v.cycle_deg > 360.0 {
                    v.cycle_deg - 720.0
                } else {
                    v.cycle_deg
                };
            }
        }
        out.remove(0); // the first cycle is incomplete
        out
    }

    fn std(x: &[f64]) -> f64 {
        let m = x.iter().sum::<f64>() / x.len() as f64;
        (x.iter().map(|v| (v - m).powi(2)).sum::<f64>() / x.len() as f64).sqrt()
    }

    #[test]
    fn burn_timing_varies_cycle_to_cycle() {
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        let mut e = Engine::new(EngineDef::single(), 3000.0, 1);
        run(&mut e, 0.3, &c);
        let jittered = std(&peak_angles(&mut e, &c, 30));
        let mut d = EngineDef::single();
        d.sound.combustion_jitter = 0.0;
        let mut e = Engine::new(d, 3000.0, 1);
        run(&mut e, 0.3, &c);
        let steady = std(&peak_angles(&mut e, &c, 30));
        assert!((0.3..5.0).contains(&jittered), "peak angle std {jittered}°");
        assert!(
            steady < 0.5 * jittered,
            "without jitter {steady}° vs {jittered}°"
        );
    }

    #[test]
    fn texture_adds_high_frequency_energy_under_load() {
        let hf = |sound: crate::sound::SoundDef, throttle: f64| {
            let mut d = EngineDef::inline4();
            d.sound = sound;
            let mut e = Engine::new(d, 5000.0, 1);
            let c = Controls {
                throttle,
                hold_rpm: Some(5000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let xs = run(&mut e, 0.2, &c);
            (xs.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>() / xs.len() as f64).sqrt()
        };
        let off = crate::sound::SoundDef {
            flow_noise: 0.0,
            intake: 0.0,
            mechanical: 0.0,
            combustion_jitter: 0.0,
            reverb: 0.0,
        };
        let on = crate::sound::SoundDef::default();
        let (dry_wot, wet_wot) = (hf(off.clone(), 1.0), hf(on.clone(), 1.0));
        let (dry_coast, wet_coast) = (hf(off, 0.0), hf(on, 0.0));
        assert!(wet_wot > 1.3 * dry_wot, "WOT: {dry_wot} → {wet_wot}");
        assert!(
            wet_wot - dry_wot > wet_coast - dry_coast,
            "texture should grow with load"
        );
    }

    fn fv(mut d: EngineDef) -> EngineDef {
        d.exhaust.model = ExhaustModel::Fv;
        d
    }

    #[test]
    fn the_exhaust_model_in_use_follows_the_definition() {
        let mut e = Engine::new(fv(EngineDef::single()), 1000.0, 1);
        run(&mut e, 0.01, &Controls::default());
        assert_eq!(e.exhaust_model(), Some(ExhaustModel::Fv));
        let mut e = Engine::new(EngineDef::single(), 1000.0, 1);
        run(&mut e, 0.01, &Controls::default());
        assert_eq!(e.exhaust_model(), Some(ExhaustModel::Waveguide));
    }

    #[test]
    fn fv_exhaust_engine_idles_and_makes_comparable_torque() {
        let mut e = Engine::new(fv(EngineDef::lfa_v10()), 1000.0, 1);
        let xs = run(&mut e, 2.0, &Controls::default());
        assert!(xs.iter().all(|x| x.is_finite()));
        assert!((500.0..2500.0).contains(&e.rpm()), "idle {}", e.rpm());
        let torque = |d: EngineDef| {
            let mut e = Engine::new(d, 6800.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(6800.0),
                ..Controls::default()
            };
            run(&mut e, 0.8, &c);
            e.torque_nm()
        };
        let (wg, fvt) = (
            torque(EngineDef::lfa_v10()),
            torque(fv(EngineDef::lfa_v10())),
        );
        assert!(
            (fvt / wg - 1.0).abs() < 0.25,
            "waveguide {wg} N·m vs fv {fvt} N·m"
        );
    }

    #[test]
    fn fv_exhaust_is_free_of_sample_oscillation_and_vacuum() {
        let mut e = Engine::new(fv(EngineDef::single()), 3000.0, 1);
        let c = Controls {
            throttle: 0.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        let mut worst_p = f64::MAX;
        let mut xs = Vec::new();
        for _ in 0..(0.1 / DT) as usize {
            xs.push(e.step(DT, &c));
            worst_p = worst_p.min(ATM_PA + e.exhaust_port_gauge(0));
        }
        assert!(worst_p > 0.0, "port {worst_p}");
        assert!(xs.iter().all(|x| x.is_finite()));
    }

    #[test]
    fn overrun_cuts_fuel_above_the_threshold_only() {
        // Deceleration fuel cut-off: closed throttle above `overrun_fuel_cut_rpm` → no combustion;
        // at idle (below it) the engine keeps firing.
        let fires = |rpm: f64, throttle: f64| {
            let mut e = Engine::new(EngineDef::inline4(), rpm, 1);
            let c = Controls {
                throttle,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            run(&mut e, 0.2, &c);
            let n = e.combustions();
            run(&mut e, 0.3, &c);
            e.combustions() - n
        };
        assert_eq!(fires(4000.0, 0.0), 0, "overrun at 4000 rpm must not fire");
        assert!(fires(900.0, 0.0) > 5, "idle keeps firing");
        assert!(fires(4000.0, 0.3) > 30, "on throttle it fires");
        let mut d = EngineDef::inline4();
        d.overrun_fuel_cut_rpm = 0.0;
        let mut e = Engine::new(d, 4000.0, 1);
        let c = Controls {
            hold_rpm: Some(4000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        assert!(e.combustions() > 20, "0 disables the cut");
    }
}
