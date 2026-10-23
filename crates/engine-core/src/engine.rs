//! Engine definition (data) and the running engine (0D crank-angle-resolved model).
//!
//! Per cylinder: plenum → intake valve → cylinder → exhaust valve → exhaust waveguides →
//! atmosphere. Mechanics: analytic crank-slider with a lumped flywheel inertia.
//! Model ladder and references: `docs/research/simulador-motor-rust.md`.

use crate::combustion::{AFR_STOICH, charge_energy, wiebe};
use crate::exhaust::{ExhaustDef, ExhaustModel, ExhaustNet};
use crate::exhaust_fv::ExhaustFv;
use crate::gas::{
    GasVolume, Reservoir, exchange_with_reservoir, exhaust_valve_flow, plan_exchange,
};
use crate::geometry::CylinderGeometry;
use crate::intake_fv::{IntakeDef, IntakeFv, IntakeModel};
use crate::jet_noise::JetNoise;
use crate::layout::Layout;
use crate::listener::Listener;
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
    /// How the cylinders stand (inline, V, flat): drawn by the app.
    #[serde(default)]
    pub layout: Layout,
    pub intake_valve: ValveTiming,
    pub exhaust_valve: ValveTiming,
    pub spark_advance_deg: f64,
    pub burn_duration_deg: f64,
    /// Everything that turns with the crank, referred to it: crank, the rods' rotating share,
    /// flywheel and clutch, kg·m². A car engine's is ≈ 0.15–0.25, mostly flywheel and clutch
    /// (Avtomobilʹnyj Transport 42 (2018), 0.178 for a car engine; Łódź University of
    /// Technology, measured flywheel assemblies at 80–90 % of the total).
    pub flywheel_inertia_kgm2: f64,
    pub plenum_volume_m3: f64,
    pub throttle_diameter_m: f64,
    /// Fraction of throttle area open at zero pedal (idle air).
    pub idle_throttle: f64,
    /// Idle speed the ECU's idle-speed control holds with the pedal released, rpm. Files
    /// written before it existed get a generic 900 rpm (not the default engine's).
    #[serde(default = "default_idle_rpm")]
    pub idle_rpm: f64,
    pub exhaust: ExhaustDef,
    /// Intake system (0D plenum, or 1D runners with ITBs / plenum).
    pub intake: IntakeDef,
    pub redline_rpm: f64,
    pub starter_torque_nm: f64,
    /// Friction mean effective pressure fit, bar: FMEP = a + b·(N/1000) + c·(N/1000)².
    pub fmep_bar: [f64; 3],
    /// Sound texture levels (noise, intake, mechanics, combustion variability).
    pub sound: SoundDef,
    /// Discharge-coefficient ratio for reverse flow through the exhaust valve (pipe → cylinder)
    /// relative to forward flow. Estimate (Blair, *Design and Simulation of Four-Stroke Engines*,
    /// separate inflow/outflow Cd maps); 1 = symmetric.
    pub exhaust_reverse_cd_ratio: f64,
    /// Deceleration fuel cut-off: with the throttle closed above this speed no fuel is injected
    /// (what production ECUs do on overrun); 0 disables it.
    pub overrun_fuel_cut_rpm: f64,
    /// The vehicle this engine drives (gearbox, mass, tyres). Files written before engines
    /// carried their vehicle get a generic car, never whatever was loaded before.
    #[serde(default = "crate::drivetrain::VehicleDef::default")]
    pub vehicle: crate::drivetrain::VehicleDef,
}

fn default_idle_rpm() -> f64 {
    900.0
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
            layout: Layout::default(),
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
            // motorcycle singles idle at 1200–1500 rpm (estimate)
            idle_rpm: 1200.0,
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
                // one tail: no spread
                tail_spacing_m: 0.9,
            },
            intake: IntakeDef::default(),
            redline_rpm: 7500.0,
            starter_torque_nm: 25.0,
            fmep_bar: [0.97, 0.15, 0.05],
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1500.0,
            exhaust_reverse_cd_ratio: 0.75,
            vehicle: crate::drivetrain::VehicleDef::single(),
        }
    }

    /// 2.0 L inline four, firing order 1-3-4-2.
    pub fn inline4() -> Self {
        Self {
            name: "Inline-4 2.0".into(),
            cylinders: 4,
            firing_order: vec![1, 3, 4, 2],
            layout: Layout::default(),
            flywheel_inertia_kgm2: 0.15,
            plenum_volume_m3: 4.0e-3,
            throttle_diameter_m: 0.060,
            idle_throttle: 0.0055,
            // a typical car idle (estimate)
            idle_rpm: 800.0,
            exhaust: ExhaustDef::default(),
            intake: IntakeDef::default(),
            redline_rpm: 7000.0,
            starter_torque_nm: 60.0,
            vehicle: crate::drivetrain::VehicleDef::inline4(),
            ..Self::single()
        }
    }

    /// Lexus LFA, Toyota 1LR-GUE: 4.8 L 72° V10, 88 × 79 mm, CR 12.0, even firing,
    /// 9,000 rpm redline (Toyota LR engine, Wikipedia; PistonHeads LFA spec sheet).
    /// Not public, estimated: rod length, cam timing, valve sizes (4 valves/cyl lumped into
    /// one equivalent valve each), inertia. Firing order: placeholder (Audi 5.2 V10 order),
    /// pending confirmation.
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
            // 72° V10 (Toyota LR); banks as the collectors
            layout: Layout {
                bank_angle_deg: 72.0,
                banks: vec![vec![1, 3, 5, 7, 9], vec![2, 4, 6, 8, 10]],
            },
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
            // from the measured unloaded snap, idle to 9000 rpm in 0.6 s (see the test)
            flywheel_inertia_kgm2: 0.28,
            plenum_volume_m3: 8.0e-3,
            // Ten individual throttle bodies of the runner's 45 mm bore, as one total bore:
            // √10 · 45 mm. Idle bypass: the same area as 0.3 % of a 120 mm throttle.
            throttle_diameter_m: 0.045 * std::f64::consts::SQRT_2 * 5f64.sqrt(),
            idle_throttle: 0.003 * 0.120 * 0.120 / (0.045 * 0.045 * 10.0),
            // no published figure; a typical high-revving V10 idle (estimate)
            idle_rpm: 900.0,
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
                // the tips sit together in the centre of the rear diffuser
                tail_spacing_m: 0.25,
            },
            // Runners and trumpets estimated from the LFA's intake: 250 mm to the valve, 45 mm.
            intake: IntakeDef {
                model: IntakeModel::Fv,
                runner_length_m: 0.25,
                runner_diameter_m: 0.045,
                individual_throttles: true,
                ..IntakeDef::default()
            },
            redline_rpm: 9000.0,
            starter_torque_nm: 150.0,
            // Low-friction racing build (dry sump, titanium valvetrain): estimated.
            fmep_bar: [0.8, 0.10, 0.015],
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1500.0,
            exhaust_reverse_cd_ratio: 0.75,
            vehicle: crate::drivetrain::VehicleDef::lfa_v10(),
        }
    }

    /// Flat-plane V8 after the Ferrari F136 F (458 Italia): 4,497 cc from 94 × 81 mm,
    /// 12.5:1, 570 PS at 9,000 rpm, 540 N·m at 6,000 (Ferrari press kit). The flat crank
    /// fires the banks alternately (1-5-3-7-4-8-2-6, banks 1–4 and 5–8): each bank is an
    /// even-firing inline-4, its order-4 pulses half a period from the other bank's, so order
    /// 8 dominates (the high, clean scream). Not public, estimated: rod, cams, valve sizes
    /// (4 valves lumped into one), exhaust lengths, inertia.
    pub fn flatplane_v8() -> Self {
        Self {
            name: "Flat-plane V8 4.5".into(),
            geometry: CylinderGeometry {
                bore_m: 0.094,
                stroke_m: 0.081,
                rod_m: 0.140,
                compression_ratio: 12.5,
            },
            cylinders: 8,
            firing_order: vec![1, 5, 3, 7, 4, 8, 2, 6],
            layout: Layout {
                bank_angle_deg: 90.0,
                banks: vec![vec![1, 2, 3, 4], vec![5, 6, 7, 8]],
            },
            intake_valve: ValveTiming {
                open_deg: 340.0,
                duration_deg: 270.0,
                max_lift_m: 0.0115,
                diameter_m: 0.050,
            },
            exhaust_valve: ValveTiming {
                open_deg: 125.0,
                duration_deg: 265.0,
                max_lift_m: 0.0105,
                diameter_m: 0.043,
            },
            spark_advance_deg: 28.0,
            burn_duration_deg: 45.0,
            // a light, high-revving car engine: the LFA's class
            flywheel_inertia_kgm2: 0.2,
            plenum_volume_m3: 7.0e-3,
            throttle_diameter_m: 0.095,
            idle_throttle: 0.003,
            // a flat-plane supercar idle (estimate)
            idle_rpm: 1000.0,
            // One 4-into-1 per bank, short primaries for the top end, two tails.
            exhaust: ExhaustDef {
                primary_length_m: 0.45,
                primary_diameter_m: 0.042,
                collectors: vec![vec![1, 2, 3, 4], vec![5, 6, 7, 8]],
                collector_length_m: 0.3,
                collector_diameter_m: 0.065,
                tail_length_m: 1.3,
                tail_diameter_m: 0.065,
                muffler_transmission: 0.7,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.010,
                // the tips sit together in the centre (458 Italia)
                tail_spacing_m: 0.25,
            },
            intake: IntakeDef::default(),
            redline_rpm: 9000.0,
            starter_torque_nm: 150.0,
            fmep_bar: [0.8, 0.10, 0.015],
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1500.0,
            exhaust_reverse_cd_ratio: 0.75,
            vehicle: crate::drivetrain::VehicleDef::flatplane_v8(),
        }
    }

    /// Cross-plane V8 after the GM LS3 (C6 Corvette): 6,162 cc from 103.25 × 92 mm, 10.7:1,
    /// 2 valves (55 / 40.4 mm, 14 mm lift), 6.098 in rods, 430 hp at 5,900 rpm, 575 N·m at
    /// 4,600, 6,600 rpm (GM Performance Parts LS3 spec). Fires 1-8-7-2-6-5-4-3 with the odd
    /// cylinders on the left bank: 90° apart overall, 90/180/180/270 within each bank — the
    /// uneven bank pulses are the burble. Estimated: cam timing, exhaust, inertia.
    pub fn crossplane_v8() -> Self {
        Self {
            name: "Cross-plane V8 6.2".into(),
            geometry: CylinderGeometry {
                bore_m: 0.10325,
                stroke_m: 0.092,
                rod_m: 0.1549,
                compression_ratio: 10.7,
            },
            cylinders: 8,
            firing_order: vec![1, 8, 7, 2, 6, 5, 4, 3],
            layout: Layout {
                bank_angle_deg: 90.0,
                banks: vec![vec![1, 3, 5, 7], vec![2, 4, 6, 8]],
            },
            intake_valve: ValveTiming {
                open_deg: 350.0,
                duration_deg: 250.0,
                max_lift_m: 0.014,
                diameter_m: 0.055,
            },
            exhaust_valve: ValveTiming {
                open_deg: 130.0,
                duration_deg: 250.0,
                max_lift_m: 0.0138,
                diameter_m: 0.0404,
            },
            spark_advance_deg: 26.0,
            burn_duration_deg: 50.0,
            // heavy cross-plane counterweights and a street clutch: the top of the range
            flywheel_inertia_kgm2: 0.25,
            plenum_volume_m3: 6.0e-3,
            throttle_diameter_m: 0.090,
            // a 6.2 L on LS friction needs more bypass air (0.3 % stalls; 0.65 % idles ~1050)
            idle_throttle: 0.0065,
            // a pushrod V8 idle (estimate)
            idle_rpm: 700.0,
            // One 4-into-1 per bank, dual exhaust.
            exhaust: ExhaustDef {
                primary_length_m: 0.75,
                primary_diameter_m: 0.044,
                collectors: vec![vec![1, 3, 5, 7], vec![2, 4, 6, 8]],
                collector_length_m: 0.3,
                collector_diameter_m: 0.075,
                tail_length_m: 2.0,
                tail_diameter_m: 0.065,
                muffler_transmission: 0.6,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.012,
                // two tips each side (C6 Corvette)
                tail_spacing_m: 1.1,
            },
            intake: IntakeDef::default(),
            redline_rpm: 6600.0,
            starter_torque_nm: 200.0,
            fmep_bar: [0.97, 0.15, 0.05],
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1300.0,
            exhaust_reverse_cd_ratio: 0.75,
            vehicle: crate::drivetrain::VehicleDef::crossplane_v8(),
        }
    }

    /// Spark advance (° BTDC) at `rpm`: `spark_advance_deg` is the full-speed value, retarded at
    /// low speed as an ECU's map does, × (0.35 + 0.65·rpm/4000) up to 1: the burn takes about a
    /// fixed time, so its MBT advance grows with speed (Heywood, *ICE Fundamentals*, §9.3).
    pub fn spark_advance_at(&self, rpm: f64) -> f64 {
        self.spark_advance_deg * (0.35 + 0.65 * rpm.max(0.0) / 4000.0).min(1.0)
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
    /// Inertia the crank drags along on top of the flywheel, kg·m² (e.g. the car reflected
    /// through a locked clutch, see `drivetrain`). Default 0.
    pub extra_inertia_kgm2: f64,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            ignition: true,
            starter: false,
            load_torque_nm: 0.0,
            hold_rpm: None,
            extra_inertia_kgm2: 0.0,
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
const WALL_TEMP_K: f64 = 400.0;
const AMBIENT_K: f64 = 300.0;
const DISCHARGE_CD: f64 = 0.8;
const TORQUE_AVG_TAU_S: f64 = 0.15;
/// Effective ring leak area as a fraction of piston area. 1e-4 bleeds a stopped 6 bar
/// charge in ~0.3 s and costs ~1–2 % of the trapped mass per cycle at WOT (typical
/// blow-by is ~1 % of intake flow; Heywood, ch. 8).
const BLOWBY_AREA_FRACTION: f64 = 1e-4;
/// Level of the 1D intake's induction source relative to the exhaust (lumped: trumpets sit in the
/// engine bay, further from the listener than the tail pipes). Estimate.
const INTAKE_SOUND_GAIN: f64 = 0.3;
/// Throttle plate actuator speed, full travel per second (0.1 s closed to wide open).
const PLATE_TRAVEL_PER_S: f64 = 10.0;
/// Throttle dashpot: when released, the plate is caught at a few percent of its *travel* and
/// closes from there with a time constant of about a second (Bosch, *Automotive Handbook*,
/// "dashpot function"; estimates). Through the plate's area curve 3 % of travel is
/// 1 − cos(0.03·π/2) = 0.11 % of its area.
const DASHPOT_HOLD_TRAVEL: f64 = 0.03;
const DASHPOT_TAU_S: f64 = 1.0;
/// Overrun fuel cut-off reactivates this far above the idle speed (Bosch, *Gasoline Engine
/// Management*, "overrun fuel cutoff": injection resumes above idle so the engine is caught
/// before it stalls; the cut-off speed itself is the engine's `overrun_fuel_cut_rpm`), and never
/// closer than `FUEL_CUT_HYSTERESIS_RPM` below the cut-off, so the two cannot chatter. Estimates.
/// Speed above which the deceleration air opens in proportion while injection is cut.
const DECEL_AIR_BASE_RPM: f64 = 1500.0;
const FUEL_REACTIVATION_ABOVE_IDLE_RPM: f64 = 300.0;
const FUEL_CUT_HYSTERESIS_RPM: f64 = 100.0;
/// The speed limiter cuts injection at the redline and resumes it this far below (Bosch,
/// *Gasoline Engine Management*, "engine-speed limitation"): a floored engine bounces off the
/// limiter in a rhythm instead of firing at random cylinders. Estimate.
const LIMITER_HYSTERESIS_RPM: f64 = 200.0;
/// Idle-speed control (Bosch, *Automotive Handbook*, "Idle-speed control"; Hrovat & Sun, "Models
/// and control methodologies for IC engine idle speed control design", Control Eng. Practice 5
/// (1997)): two paths on the relative speed error. The air path (bypass) carries the steady
/// state but acts through the manifold's filling delay, so it is slow and integral-led; the
/// spark path acts on the next firing, so it takes the fast proportional and damping action
/// from a retarded idle timing that keeps torque in reserve both ways.
///
/// Gains from sweeps over the five presets' start → idle → full-throttle blip → release
/// journey (smallest worst-case dip, then idle accuracy).
///
/// Air path: trim of the bypass, per unit of the air the engine swallows at idle.
const IAC_KP: f64 = 0.05;
const IAC_KI_PER_S: f64 = 0.5;
/// An electronic throttle closes nearly to its stop (5 % of the base opening is left as the
/// plate's leakage), and an idle actuator passes a few percent of the bore at most.
const IAC_MIN: f64 = -0.95;
const IAC_MAX_AREA: f64 = 0.03;
/// Spark path: idle timing sits this far retarded from the engine's advance, and the loop
/// moves it by up to the reserve each way, degrees.
const IDLE_SPARK_RESERVE_DEG: f64 = 10.0;
const IDLE_SPARK_KP_DEG: f64 = 80.0;
const IDLE_SPARK_KD_DEG_S: f64 = 10.0;
/// Time constant of the speed-rate filter, s: longer than a firing interval at idle, so the
/// loop follows the mean speed, not the ripple between firings.
const IAC_RATE_TAU_S: f64 = 0.15;
/// How far ahead the air path looks before holding its integral on a fall, s: the delay it
/// acts through, the manifold's filling plus the loop's own response, about a second.
const IAC_LOOKAHEAD_S: f64 = 1.0;
/// Fraction of the equalising mass a valve may move per substep (see `step`).
const VALVE_RELAX: f64 = 0.5;

#[derive(Debug, Clone)]
struct Burn {
    start_deg: f64,
    duration_deg: f64,
    energy_j: f64,
    released: f64,
}

/// Woschni's heat-transfer coefficient (SAE 670931; Heywood, *ICE Fundamentals*, eq. 12.19),
/// W/(m²·K): h = 3.26·B^−0.2·p^0.8·T^−0.55·w^0.8 with p in kPa, B in m, w in m/s.
fn woschni_htc(bore_m: f64, p_pa: f64, t_k: f64, w_m_s: f64) -> f64 {
    // in logarithms: three ln and one exp, not four pow (it runs per cylinder per step)
    let pw = p_pa.max(0.0) / 1000.0 * w_m_s.max(0.0);
    3.26 * (0.8 * pw.ln() - 0.55 * t_k.max(1.0).ln() - 0.2 * bore_m.ln()).exp()
}

/// Mean piston speed S̄p = 2·S·N, m/s (Heywood, *ICE Fundamentals*, eq. 2.2), whichever way the
/// crank turns.
fn mean_piston_speed(stroke_m: f64, omega: f64) -> f64 {
    2.0 * stroke_m * omega.abs() / (2.0 * PI)
}

/// The cylinder is exchanging gas while either valve is off its seat.
fn gas_exchange(a_in: f64, a_ex: f64) -> bool {
    a_in > 0.0 || a_ex > 0.0
}

/// Woschni's characteristic gas velocity, m/s: C₁·S̄p (C₁ = 6.18 with a valve open, 2.28 when
/// closed) plus 3.24e-3·`rise_k`, the combustion term V_d·T_r·(p − p_mot)/(p_r·V_r) in K.
fn woschni_velocity(mean_piston_speed: f64, gas_exchange: bool, rise_k: f64) -> f64 {
    let c1 = if gas_exchange { 6.18 } else { 2.28 };
    c1 * mean_piston_speed + 3.24e-3 * rise_k.max(0.0)
}

#[derive(Debug, Clone)]
struct CylinderState {
    offset_deg: f64,
    gas: GasVolume,
    trapped_kg: f64,
    /// The charge trapped at intake closing still awaits its spark: one spark per charge,
    /// whatever the ECU does with the timing meanwhile.
    armed: bool,
    /// Gas temperature in this cylinder's exhaust primary, and the primary's wall temperature,
    /// K: what reversion pushes back, and how much volume it is.
    port_t: f64,
    /// The exhaust port's last mass flow, kg/s: where the next step's port solve starts.
    port_flow: f64,
    wall_t: f64,
    /// State at intake closing (p_r, V_r, T_r): Woschni's reference for the motored pressure.
    ref_p: f64,
    ref_v: f64,
    ref_t: f64,
    burn: Option<Burn>,
    cycle_deg: f64,
}

/// How finely an engine is simulated (not part of its definition: the same engine at
/// another fidelity). The default is the reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Fidelity {
    /// Scheme of every 1D pipe (intake runners, FV exhaust).
    pub pipe_scheme: crate::fv::Scheme,
}

#[derive(Debug, Clone)]
pub struct Engine {
    def: EngineDef,
    fidelity: Fidelity,
    cyl: Vec<CylinderState>,
    plenum: GasVolume,
    atm: Reservoir,
    /// Crank angle, radians within [0, 4π).
    theta: f64,
    omega: f64,
    torque_avg: f64,
    /// Idle air the idle-speed control adds to `idle_throttle` (fraction of the full throttle
    /// area; negative closes the bypass), and its integral term (per idle-air scale).
    iac: f64,
    iac_integral: f64,
    /// Filtered d(rpm)/dt for the derivative action, rpm/s, and the rpm it was last taken at.
    iac_rate: f64,
    iac_last_rpm: f64,
    /// The ECU's injection cut-offs, each with its hysteresis: overrun (pedal released at
    /// speed) and the speed limiter.
    overrun_cut: bool,
    limiter_cut: bool,
    /// The idle-speed control's spark trim, degrees (negative = retarded); 0 off idle.
    idle_spark_deg: f64,
    /// Opening the throttle dashpot holds (fraction of the plate's area), decaying.
    dashpot: f64,
    /// Throttle plate position in pedal units (0..=1), following the pedal at the actuator's
    /// speed.
    plate: f64,
    combustions: u64,
    /// Running totals for energy balances: heat lost to the walls, and fuel energy released, J.
    wall_heat_j: f64,
    fuel_energy_j: f64,
    rng: u64,
    /// Exhaust waveguide network, built for the current step size.
    net: Option<ExhaustNet>,
    /// Non-linear finite-volume exhaust, when `exhaust.model` is `fv`.
    fv: Option<ExhaustFv>,
    /// 1D intake runners, when `intake.model` is `fv`.
    intake_net: Option<IntakeFv>,
    /// Tail pipes (one per collector group): each is its own jet for the flow noise.
    tail_pipes: usize,
    /// Where the listener hears the tails from, built for the step size it was made with.
    listener: Option<(f64, Listener)>,
    /// The texture layers' signal this step, in the tails' units, before it is radiated.
    texture_signal: f64,
    /// The turbulent jet at each tail pipe.
    jets: Vec<JetNoise>,
    flows: Vec<f64>,
    texture: Texture,
    /// Total mass that flowed back from the exhaust into the cylinders, kg (diagnostics).
    reversion_kg: f64,
    /// Total air inducted from the atmosphere through the throttle(s), kg (diagnostics).
    inducted_kg: f64,
}

/// Gas held in one primary, kg: its volume π/4·d²·L at atmospheric pressure and `t_k`.
fn primary_gas_mass(ex: &crate::exhaust::ExhaustDef, t_k: f64) -> f64 {
    let v = PI * 0.25 * ex.primary_diameter_m.powi(2) * ex.primary_length_m;
    ATM_PA / (crate::gas::R_AIR * t_k) * v
}

/// Exhaust port gas: heat-transfer coefficient to the primary's wall, W/(m²·K) (Heywood, ICE
/// Fundamentals, §12.6: 100–500 in exhaust ports), and the wall's own time constant, s (a
/// steel pipe's thermal mass; estimate).
const PORT_HTC: f64 = 200.0;
const PIPE_WALL_TAU_S: f64 = 60.0;
/// Temperature the exhaust pipes start at, K (a cold engine).
const PIPE_COLD_K: f64 = 320.0;

/// Gas in a primary at `t` K (mass `m` kg, wall area `a_wall`) after `dm` kg at `t_in` mixes in
/// (outflow only: reversion takes the pipe's own gas out) and the wall at `t_wall` cools it for
/// `dt`, an exact first-order step with τ = m·c_p / (h·A).
fn port_gas_temperature(
    t: f64,
    m: f64,
    dm: f64,
    t_in: f64,
    t_wall: f64,
    a_wall: f64,
    dt: f64,
) -> f64 {
    let mixed = if dm > 0.0 {
        (m * t + dm * t_in) / (m + dm)
    } else {
        t
    };
    let tau = m.max(1e-9) * crate::gas::CP / (PORT_HTC * a_wall.max(1e-9));
    t_wall + (mixed - t_wall) * (-dt / tau).exp()
}

/// The waveguide exhaust's source for a valve mass flow `dm_dt` (kg/s) carrying specific total
/// enthalpy `h`: the mass of reference gas with the same volume. An open end radiates volume
/// velocity (monopole, p = ρ₀/4πr · dQ/dt; Kinsler et al., *Fundamentals of Acoustics*, §7.4),
/// and the linear pipes model gas at `C_PRIMARY`; at the pipe's pressure the volume of a flow
/// scales with its temperature, so a flow at T counts T/T_ref of its mass.
fn exhaust_source_flow(dm_dt: f64, h: f64) -> f64 {
    let t_ref = crate::exhaust::C_PRIMARY.powi(2) / (crate::gas::GAMMA * crate::gas::R_AIR);
    dm_dt * (h / crate::gas::CP).max(1.0) / t_ref
}

/// Fraction of the plate's area a pedal position opens: 1 − cos(t·π/2), progressive at the
/// bottom as a throttle cam is.
fn pedal_opening(t: f64) -> f64 {
    1.0 - (t * PI * 0.5).cos()
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
        let tail_pipes = def.exhaust.groups(def.cylinders).len();
        let cyl = def
            .phase_offsets_deg()
            .into_iter()
            .map(|offset_deg| {
                let cycle_deg = (0.0 - offset_deg).rem_euclid(720.0);
                CylinderState {
                    offset_deg,
                    gas: GasVolume::at(ATM_PA, AMBIENT_K, g.volume(deg_to_rad(cycle_deg))),
                    trapped_kg: 0.0,
                    armed: false,
                    port_t: PIPE_COLD_K,
                    port_flow: 0.0,
                    wall_t: PIPE_COLD_K,
                    ref_p: ATM_PA,
                    ref_v: 1.0,
                    ref_t: AMBIENT_K,
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
            iac: 0.0,
            iac_integral: 0.0,
            iac_rate: 0.0,
            idle_spark_deg: 0.0,
            dashpot: 0.0,
            plate: 0.0,
            iac_last_rpm: initial_rpm,
            overrun_cut: false,
            limiter_cut: false,
            combustions: 0,
            wall_heat_j: 0.0,
            fuel_energy_j: 0.0,
            rng: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            net: None,
            fv: None,
            intake_net: None,
            tail_pipes,
            listener: None,
            texture_signal: 0.0,
            jets: Vec::new(),
            flows: Vec::new(),
            texture: Texture::new(seed ^ 0x7E57),
            reversion_kg: 0.0,
            inducted_kg: 0.0,
            fidelity: Fidelity::default(),
        }
    }

    /// [`Engine::new`] at another [`Fidelity`].
    pub fn with_fidelity(def: EngineDef, initial_rpm: f64, seed: u64, fidelity: Fidelity) -> Self {
        Self {
            fidelity,
            ..Self::new(def, initial_rpm, seed)
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
            if self.def.sound != def.sound {
                self.listener = None;
            }
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

    /// Trim the idle air towards `idle_rpm` while the pedal is released and the engine runs
    /// on its own; hold the trim otherwise (pedal pressed, cranking, overrun above the dashpot,
    /// ignition off), so the loop does not wind up while it cannot act.
    fn idle_control(&mut self, dt: f64, c: &Controls) {
        let rpm = self.rpm();
        let target = self.def.idle_rpm;
        let raw = (rpm - self.iac_last_rpm) / dt;
        self.iac_last_rpm = rpm;
        self.iac_rate += (raw - self.iac_rate) * (dt / IAC_RATE_TAU_S).min(1.0);
        if c.throttle > 0.01 || !c.ignition || rpm < 0.4 * target || self.overrun_cut {
            self.idle_spark_deg = self.slew_spark(0.0, dt);
            return;
        }
        let err = (target - rpm) / target;
        // the bypass trim, in units of the air this engine swallows at its idle
        let scale = self.idle_air_scale();
        let lo = IAC_MIN * self.def.idle_throttle / scale;
        let hi = (IAC_MAX_AREA - self.def.idle_throttle).max(0.0) / scale;
        // Integrate towards the target, except while the speed is already falling fast enough
        // to reach it within IAC_LOOKAHEAD_S: falling from a blip, the spark path catches the
        // engine and the integral does not wind down into a stall; a slow drift above the
        // target (a heavy crank after its start flare) is still trimmed.
        if err > 0.0 || rpm + self.iac_rate * IAC_LOOKAHEAD_S > target {
            self.iac_integral = (self.iac_integral + IAC_KI_PER_S * err * dt).clamp(lo, hi);
        }
        self.iac = scale * (self.iac_integral + IAC_KP * err).clamp(lo, hi);
        let r = IDLE_SPARK_RESERVE_DEG;
        let trim = IDLE_SPARK_KP_DEG * err - IDLE_SPARK_KD_DEG_S * self.iac_rate / target;
        self.idle_spark_deg = self.slew_spark(trim.clamp(-r, r) - r, dt);
    }

    /// The air an engine swallows at its idle speed, as a fraction of its full throttle area:
    /// ṁ = ρ·V_d·N/120 at a typical idle manifold (30 kPa, 300 K), through a choked orifice
    /// from the atmosphere. The idle loop's gains are per this, so they mean the same on a
    /// 500 cc single and a 6.2 L V8.
    fn idle_air_scale(&self) -> f64 {
        let rho = 30e3 / (crate::gas::R_AIR * 300.0);
        let mdot = rho * self.def.displacement_m3() * self.def.idle_rpm / 120.0;
        let flux = crate::gas::orifice_mass_flow(1.0, ATM_PA, 300.0, 30e3);
        let full = DISCHARGE_CD * PI * 0.25 * self.def.throttle_diameter_m.powi(2);
        (mdot / flux / full).max(1e-6)
    }

    /// Update the injection cut-offs from the speed and the pedal, each switching off only once
    /// the speed has crossed back past its hysteresis.
    fn fuel_cutoffs(&mut self, throttle: f64) {
        let rpm = self.rpm();
        let cut = self.def.overrun_fuel_cut_rpm;
        let resume = (self.def.idle_rpm + FUEL_REACTIVATION_ABOVE_IDLE_RPM)
            .min(cut - FUEL_CUT_HYSTERESIS_RPM);
        self.overrun_cut =
            cut > 0.0 && throttle <= 0.01 && (rpm > cut || (self.overrun_cut && rpm > resume));
        let red = self.def.redline_rpm;
        self.limiter_cut = rpm >= red || (self.limiter_cut && rpm > red - LIMITER_HYSTERESIS_RPM);
    }

    /// Move the spark trim towards `want` no faster than half the crank's own travel this
    /// step: a spark angle that jumped ahead of the crank between two steps would never be
    /// crossed, and that cylinder would misfire.
    fn slew_spark(&self, want: f64, dt: f64) -> f64 {
        let max = 0.5 * self.omega.abs().to_degrees() * dt;
        self.idle_spark_deg + (want - self.idle_spark_deg).clamp(-max, max)
    }

    /// The throttle plate follows the pedal no faster than its actuator travels: an electronic
    /// throttle body sweeps closed to wide open in about 0.1 s (Bosch, *Automotive Handbook*,
    /// "electronic throttle control"), and a foot on a cable is no quicker. An instant opening
    /// steps the throttle's mass flow, which the induction sound turns into a click.
    fn plate_step(&mut self, throttle: f64, dt: f64) {
        let max = PLATE_TRAVEL_PER_S * dt;
        self.plate += (throttle.clamp(0.0, 1.0) - self.plate).clamp(-max, max);
    }

    /// The throttle's dashpot (the damper on a cable throttle, the closing follower of an
    /// electronic one; Bosch, *Automotive Handbook*): released, the plate is caught at a small
    /// opening and closes from it over about a second, so the manifold does not empty and drop
    /// a decelerating engine through its idle.
    fn dashpot_step(&mut self, throttle: f64, dt: f64) {
        let pedal = pedal_opening(throttle.clamp(0.0, 1.0));
        if pedal >= self.dashpot {
            self.dashpot = pedal.min(pedal_opening(DASHPOT_HOLD_TRAVEL));
        } else {
            self.dashpot *= (-dt / DASHPOT_TAU_S).exp();
        }
    }

    fn throttle_area(&self, throttle: f64) -> f64 {
        let t = throttle.clamp(0.0, 1.0);
        // The idle bypass, trimmed by the idle-speed control. While the overrun cut-off holds,
        // the ECU opens it in proportion to speed above DECEL_AIR_BASE_RPM (its deceleration air):
        // the engine's pumping grows with speed, so the manifold holds ≈ 0.2–0.3 bar instead of
        // being pulled to a near vacuum that drives violent exhaust reversion.
        let decel = if self.overrun_cut {
            (self.rpm() / DECEL_AIR_BASE_RPM).max(1.0)
        } else {
            1.0
        };
        let idle = (self.def.idle_throttle + self.iac) * decel;
        let open = idle + ((1.0 - idle) * pedal_opening(t)).max(self.dashpot);
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
        // the ECU first decides whether it injects, then trims the idle accordingly
        self.fuel_cutoffs(c.throttle);
        self.idle_control(dt, c);
        self.plate_step(c.throttle, dt);
        self.dashpot_step(self.plate, dt);
        let spark_deg = 720.0 - (self.def.spark_advance_at(self.rpm()) + self.idle_spark_deg);
        let ivc =
            (self.def.intake_valve.open_deg + self.def.intake_valve.duration_deg).rem_euclid(720.0);
        let evc = (self.def.exhaust_valve.open_deg + self.def.exhaust_valve.duration_deg)
            .rem_euclid(720.0);
        let mut valve_closings = 0;
        let fire_ok = c.ignition && !self.overrun_cut && !self.limiter_cut;
        let wall_area = 2.0 * g.piston_area();
        let mean_piston_speed = mean_piston_speed(g.stroke_m, self.omega);
        match self.def.intake.model {
            IntakeModel::ZeroD => self.intake_net = None,
            IntakeModel::Fv => {
                if self.intake_net.as_ref().is_none_or(|n| n.dt != dt) {
                    self.intake_net = Some(
                        IntakeFv::new(
                            &self.def.intake,
                            self.def.plenum_volume_m3,
                            self.cyl.len(),
                            dt,
                        )
                        .with_scheme(self.fidelity.pipe_scheme),
                    );
                }
            }
        }
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
                    self.fv = Some(
                        ExhaustFv::new(&self.def.exhaust, self.cyl.len(), dt)
                            .with_scheme(self.fidelity.pipe_scheme),
                    );
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
            // The intake valve draws from the 0D plenum, or from its runner's last cell (1D).
            let f_in = (a_in > 0.0).then(|| match &self.intake_net {
                Some(net) => plan_exchange(&net.port_volume(i), &cs.gas, a_in, dt, VALVE_RELAX),
                None => plan_exchange(&self.plenum, &cs.gas, a_in, dt, VALVE_RELAX),
            });
            // The exhaust port and its primary pipe are solved together: the flow raises the
            // pipe pressure at the port by z·ṁ (see `gas::port_flow`).
            let plan_ex = |area: f64| match &self.fv {
                // FV: the first pipe cell is a real volume — a planned two-volume exchange.
                // FV: on the short time scale between pipe steps the primary is its
                // characteristic impedance to the valve, driven by the wave its first cell sends
                // back (the waveguide's implicit port, so the flow does not step at the FV rate).
                Some(fv) => {
                    let (arriving_fv, z_fv, t_fv) = fv.port_wave(i);
                    exhaust_valve_flow(
                        &cs.gas,
                        area,
                        crate::gas::Port {
                            arriving: arriving_fv,
                            z: z_fv,
                            t_back: t_fv,
                        },
                        dt,
                        VALVE_RELAX,
                        cs.port_flow,
                    )
                }
                None => exhaust_valve_flow(
                    &cs.gas,
                    area,
                    crate::gas::Port {
                        arriving,
                        z: z_port,
                        t_back: cs.port_t,
                    },
                    dt,
                    VALVE_RELAX,
                    cs.port_flow,
                ),
            };
            // Reverse flow (pipe → cylinder) passes the valve against the port's curve with a lower
            // discharge coefficient than forward flow (Blair measures separate in/out maps).
            let f_ex = (a_ex > 0.0).then(|| {
                let f = plan_ex(a_ex);
                if f.dm < 0.0 && self.def.exhaust_reverse_cd_ratio < 1.0 {
                    plan_ex(a_ex * self.def.exhaust_reverse_cd_ratio)
                } else {
                    f
                }
            });
            if let Some(f) = f_ex
                && f.dm < 0.0
            {
                self.reversion_kg -= f.dm;
            }
            if let Some(f) = f_in {
                match &mut self.intake_net {
                    Some(net) => {
                        cs.gas.mass_kg += f.dm;
                        cs.gas.energy_j += f.dm * f.h;
                        net.add_valve_flow(i, f.dm, f.h);
                    }
                    None => f.apply(&mut self.plenum, &mut cs.gas),
                }
            }
            cs.port_flow = f_ex.map_or(0.0, |f| f.dm / dt);
            self.flows[i] = f_ex.map_or(0.0, |f| {
                f.apply_to(&mut cs.gas);
                exhaust_source_flow(f.dm / dt, f.h)
            });
            {
                let ex = &self.def.exhaust;
                let m = primary_gas_mass(ex, cs.port_t);
                let (dm, t_in) = f_ex.map_or((0.0, cs.port_t), |f| (f.dm, f.h / crate::gas::CP));
                let a_wall = PI * ex.primary_diameter_m * ex.primary_length_m;
                cs.port_t = port_gas_temperature(cs.port_t, m, dm, t_in, cs.wall_t, a_wall, dt);
                cs.wall_t += (cs.port_t - cs.wall_t) * (dt / PIPE_WALL_TAU_S);
            }
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
                cs.armed = true;
                cs.ref_p = cs.gas.pressure();
                cs.ref_v = cs.gas.volume_m3;
                cs.ref_t = cs.gas.temperature();
            }
            let mut new_burn = false;
            // The spark fires on the first step at or past its angle (within the half cycle
            // after it), so a timing that moved earlier past the crank still fires this charge,
            // and one that moved later cannot fire it again. A cut cylinder's charge passes.
            if cs.armed && (cur - spark_deg).rem_euclid(720.0) < 360.0 {
                cs.armed = false;
                new_burn = fire_ok && cs.trapped_kg > 0.0;
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
                self.fuel_energy_j += b.energy_j * (x - b.released);
                b.released = x;
                if since > 2.0 * b.duration_deg {
                    cs.burn = None;
                }
            }

            // Wall heat loss, Woschni. The combustion term compares the pressure with the
            // motored one, a polytropic (n = 1.32) compression from intake closing.
            let t_gas = cs.gas.temperature();
            let p_gas = cs.gas.pressure();
            let exchange = gas_exchange(a_in, a_ex);
            let rise_k = if exchange {
                0.0
            } else {
                let p_mot = cs.ref_p * (cs.ref_v / v_new).powf(1.32);
                g.displacement() * cs.ref_t * (p_gas - p_mot) / (cs.ref_p * cs.ref_v)
            };
            let w = woschni_velocity(mean_piston_speed, exchange, rise_k);
            let htc = woschni_htc(g.bore_m, p_gas, t_gas, w);
            let area = wall_area + PI * g.bore_m * (v_new / g.piston_area());
            let q = (htc * area * (t_gas - WALL_TEMP_K) * dt).min(0.5 * cs.gas.energy_j);
            cs.gas.energy_j -= q;
            self.wall_heat_j += q;

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
        // advance the exhaust; what each tail radiates is read below (`last_tails`)
        match (&mut self.net, &mut self.fv) {
            (_, Some(fv)) => {
                fv.step();
            }
            (Some(net), None) => {
                net.step(&self.flows);
            }
            (None, None) => {}
        }
        if self.listener.as_ref().is_none_or(|l| l.0 != dt) {
            let (ex, snd) = (&self.def.exhaust, &self.def.sound);
            let l = Listener::new(
                self.tail_pipes,
                ex.tail_spacing_m,
                snd.listener_distance_m,
                snd.listener_angle_deg,
                PI * 0.25 * ex.tail_diameter_m.powi(2),
                dt,
            );
            self.listener = Some((dt, l));
        }
        let tails = match (&self.net, &self.fv) {
            (_, Some(fv)) => fv.last_tails(),
            (Some(net), None) => net.last_tails(),
            (None, None) => &[],
        };
        let heard = match &mut self.listener {
            Some((_, l)) => l.process(tails),
            None => 0.0,
        };
        let throttle_area = self.throttle_area(self.plate);
        let bore_area = self.throttle_area(1.0);
        // 0D: throttle into the plenum. 1D: the network steps itself (ITBs take the same area each).
        let (throttle_dm, induction, throttle_flow) = match &mut self.intake_net {
            Some(net) => {
                // `throttle_diameter_m` is the total equivalent opening; ITBs share it.
                let area = if self.def.intake.individual_throttles {
                    throttle_area / self.cyl.len().max(1) as f64
                } else {
                    throttle_area
                };
                let induction = net.step(area);
                (-net.last_inducted(), induction, net.inducted_rate())
            }
            None => {
                let dm = exchange_with_reservoir(&mut self.plenum, &self.atm, throttle_area, dt);
                (dm, 0.0, dm / dt)
            }
        };

        self.inducted_kg -= throttle_dm;

        // Texture: jet noise at the tails, induction sound, valvetrain clicks.
        let snd = &self.def.sound;
        let rpm = rad_s_to_rpm(self.omega);
        for _ in 0..valve_closings {
            self.texture.valve_closed(rpm, snd.mechanical);
        }
        // The texture layers are signals in the tails' units (ρc·u); until each is replaced by
        // its physical source, they radiate from the tails like the exhaust does.
        let texture = self.texture.intake(throttle_flow, bore_area, snd.intake)
            + snd.intake * INTAKE_SOUND_GAIN * induction
            + self.texture.clicks(dt);
        self.texture_signal = texture;
        // each tail's jet: its exit velocity u = (ρc·u)/ρc, its own distance to the listener
        let mut jets = 0.0;
        if snd.flow_noise > 0.0
            && let Some((_, l)) = &self.listener
        {
            let z = crate::exhaust::tail_impedance();
            let rho = crate::exhaust::tail_reference_density();
            let d = self.def.exhaust.tail_diameter_m;
            if self.jets.len() != tails.len() {
                // seeded from the engine's own generator: deterministic for a seed
                let base = self.rng;
                self.jets = (0..tails.len())
                    .map(|k| JetNoise::new(base.wrapping_add(k as u64 + 1)))
                    .collect();
            }
            for ((jet, signal), r) in self.jets.iter_mut().zip(tails).zip(l.distances()) {
                jets += jet.step(signal / z, d, rho, *r, dt);
            }
            jets *= snd.flow_noise;
        }
        let audio = heard
            + jets
            + match &mut self.listener {
                Some((_, l)) => l.radiate_shared(texture),
                None => 0.0,
            };

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
            None => {
                let j = self.def.flywheel_inertia_kgm2 + c.extra_inertia_kgm2;
                self.omega = (self.omega + net / j * dt).max(0.0);
            }
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
        match &self.intake_net {
            Some(net) => net.manifold_pressure(),
            None => self.plenum.pressure(),
        }
    }
    /// Total air inducted through the throttle(s) so far, kg.
    pub fn inducted_kg(&self) -> f64 {
        self.inducted_kg
    }

    /// Whether the 1D intake is running.
    pub fn intake_is_fv(&self) -> bool {
        self.intake_net.is_some()
    }

    /// Total exhaust reversion (pipe → cylinder) so far, kg.
    pub fn exhaust_reversion_kg(&self) -> f64 {
        self.reversion_kg
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
    fn each_preset_names_itself_and_sizes_its_plenum_to_its_displacement() {
        // the multi-cylinder presets size their plenum at 1–3 times the displacement (our
        // estimate: big enough to damp the intake pulses, small enough for a quick throttle)
        for d in [EngineDef::inline4(), EngineDef::lfa_v10()] {
            let ratio = d.plenum_volume_m3 / d.displacement_m3();
            assert!((1.0..=3.0).contains(&ratio), "{}: {ratio}", d.name);
        }
        assert_eq!(EngineDef::inline4().name, "Inline-4 2.0");
    }

    #[test]
    fn a_moving_spark_fires_each_charge_exactly_once() {
        // The ECU moves the spark between cycles (rpm map, idle trim). A 30° jump earlier must
        // not skip the cylinder whose crank sits between the two angles, nor one later fire a
        // charge twice: one spark per charge. Each run jumps one way and creeps back the other
        // (slower than the crank, which is always safe), so misses and doubles cannot cancel.
        for jump_earlier in [true, false] {
            let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
            let c = Controls {
                throttle: 0.5,
                hold_rpm: Some(3000.0),
                ..Controls::default()
            };
            run(&mut e, 0.2, &c);
            let c0 = e.combustions;
            let (lo, hi) = (5.0, 35.0);
            e.def.spark_advance_deg = if jump_earlier { lo } else { hi };
            for k in 0..(1.0 / DT) as usize {
                let a = &mut e.def.spark_advance_deg;
                if k % (0.02 / DT) as usize == 0 {
                    *a = if jump_earlier { hi } else { lo };
                } else {
                    // creep back over the 20 ms: 1.5 °/ms against the crank's 18 °/ms
                    let creep = (hi - lo) * DT / 0.02;
                    *a = if jump_earlier {
                        (*a - creep).max(lo)
                    } else {
                        (*a + creep).min(hi)
                    };
                }
                e.step(DT, &c);
            }
            // 3000 rpm, 4 cylinders: 25 cycles/s · 4 = 100 firings
            let fired = e.combustions - c0;
            assert!(
                (99..=101).contains(&fired),
                "jump earlier {jump_earlier}: {fired} firings"
            );
        }
    }

    #[test]
    fn a_primary_holds_its_volume_of_gas_at_one_atmosphere() {
        // 40 mm × 0.7 m: V = π/4·0.04²·0.7 = 8.7965e-4 m³; at 900 K,
        // ρ = 101 325/(287·900) = 0.39228 kg/m³ → 3.4506e-4 kg
        let ex = crate::exhaust::ExhaustDef::default();
        assert_relative_eq!(primary_gas_mass(&ex, 900.0), 3.4506e-4, max_relative = 1e-4);
    }

    #[test]
    fn the_primary_wall_warms_with_a_minute_time_constant() {
        // A first-order lag, dT_w/dt = (T_gas − T_w)/60 s: over one second from cold (320 K)
        // the wall rises by the mean excess of the gas over it divided by 60.
        let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        let (mut excess, n) = (0.0, (1.0 / DT) as usize);
        for _ in 0..n {
            excess += e.cyl[0].port_t - e.cyl[0].wall_t;
            e.step(DT, &c);
        }
        let rise = e.cyl[0].wall_t - 320.0;
        let want = excess / n as f64 / 60.0;
        assert!(want > 3.0, "the gas should be hot: {want} K/s");
        assert_relative_eq!(rise, want, max_relative = 0.02);
    }

    #[test]
    fn primary_gas_mixes_and_cools_to_its_wall() {
        // equal masses at 300 and 900 K mix to 600 K (no time for the wall)
        assert_relative_eq!(
            port_gas_temperature(300.0, 1e-3, 1e-3, 900.0, 400.0, 0.1, 0.0),
            600.0
        );
        // no inflow, one τ = m·c_p/(h·A) = 1e-3·1004.5/(200·0.1) = 0.0502 s: 900 → 400 + 500/e
        let t = port_gas_temperature(900.0, 1e-3, 0.0, 0.0, 400.0, 0.1, 0.050_225);
        assert_relative_eq!(t, 400.0 + 500.0 / std::f64::consts::E, max_relative = 1e-4);
        // reversion takes pipe gas out: the rest keeps its temperature
        assert_eq!(
            port_gas_temperature(700.0, 1e-3, -5e-4, 300.0, 700.0, 0.1, 1e-3),
            700.0
        );
    }

    #[test]
    fn reversion_gas_is_as_hot_as_the_primary_holds() {
        // At full power the primaries fill with burnt gas (~800 K and up); after a second of
        // fuel cut they hold pumped air cooled towards the wall, and that is what reversion
        // pushes back into the cylinder (it was a fixed 800 K).
        let mut e = Engine::new(EngineDef::inline4(), 4400.0, 1);
        let hold = |t: f64| Controls {
            throttle: t,
            hold_rpm: Some(4400.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &hold(1.0));
        let hot = e.cyl[0].port_t;
        run(&mut e, 1.0, &hold(0.0));
        let cold = e.cyl[0].port_t;
        assert!(hot > 700.0, "full power: {hot} K");
        assert!(cold < 600.0, "after a second of overrun: {cold} K");
    }

    #[test]
    fn the_exhaust_source_is_the_volume_flow() {
        // An open pipe end radiates its volume velocity (Q = ṁ·R·T/p at the pipe's pressure),
        // not its mass flow: the waveguide is linear about gas at C_PRIMARY = 600 m/s, i.e.
        // T_ref = 600² / (1.4 · 287) = 895.97 K. A kilogram of gas at twice that is twice the
        // volume; cold reversion gas at 300 K is a third of it.
        let cp = crate::gas::CP;
        assert_relative_eq!(
            exhaust_source_flow(0.1, cp * 895.97),
            0.1,
            max_relative = 1e-4
        );
        assert_relative_eq!(
            exhaust_source_flow(0.1, cp * 2.0 * 895.97),
            0.2,
            max_relative = 1e-4
        );
        assert_relative_eq!(
            exhaust_source_flow(-0.1, cp * 300.0),
            -0.03348,
            max_relative = 1e-3
        );
    }

    #[test]
    fn the_fv_exhaust_valve_does_not_chatter_on_overrun() {
        // Between pipe steps the valve must see the primary as its characteristic impedance,
        // not a port cell that only updates at the FV rate: against that stale cell the flow
        // flipped direction on every pipe step (127 reversals in 7 cycles). An honest exchange
        // turns around about twice a cycle (blowdown or reversion, then the exhaust stroke).
        let mut d = EngineDef::inline4();
        d.exhaust.model = ExhaustModel::Fv;
        let mut e = Engine::new(d, 4400.0, 1);
        let c = Controls {
            hold_rpm: Some(4400.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &c);
        let (mut flips, mut prev) = (0, 0.0_f64);
        for _ in 0..(0.2 / DT) as usize {
            e.step(DT, &c);
            let f = e.flows[0];
            if f * prev < 0.0 {
                flips += 1;
            }
            if f != 0.0 {
                prev = f;
            }
        }
        // 4400 rpm: 36.7 cycles/s · 0.2 s = 7.3 cycles
        assert!(flips <= 25, "{flips} reversals in 7 cycles");
    }

    #[test]
    fn overrun_pumps_less_volume_through_the_valve_than_full_power() {
        // Fuel cut on a closed throttle: the cylinder reaches its exhaust opening at ~0.3 bar
        // instead of ~4.7 bar, so the valve's volume flow swings far less: the blowdown step
        // (1 − 0.32) against (4.74 − 1) bar is 5.5×, 15 dB. Counted as mass, cold reversion
        // gas made it ~0 dB; as volume (the waveguide's source) it must be at least 8 dB down.
        let ac_source = |throttle: f64| {
            let mut e = Engine::new(EngineDef::inline4(), 4400.0, 1);
            let c = Controls {
                throttle,
                hold_rpm: Some(4400.0),
                ..Controls::default()
            };
            run(&mut e, 0.5, &c);
            let q: Vec<f64> = (0..(0.2 / DT) as usize)
                .map(|_| {
                    e.step(DT, &c);
                    e.flows[0]
                })
                .collect();
            let m = q.iter().sum::<f64>() / q.len() as f64;
            (q.iter().map(|v| (v - m) * (v - m)).sum::<f64>() / q.len() as f64).sqrt()
        };
        let db = 20.0 * (ac_source(0.0) / ac_source(1.0)).log10();
        assert!(db < -8.0, "overrun source {db:.1} dB against full power");
    }

    /// Diagnostic (prints): where the overrun-vs-full-power level difference is lost, from the
    /// valve's mass flow to what the tail radiates, per preset.
    /// `cargo test -p engine-core --release coast_chain -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn coast_chain() {
        let ac = |v: &[f64]| {
            let m = v.iter().sum::<f64>() / v.len() as f64;
            (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt()
        };
        for (d, layers) in [
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            EngineDef::crossplane_v8(),
        ]
        .into_iter()
        .flat_map(|d| [(d.clone(), true), (d, false)])
        {
            let mut d = d;
            if !layers {
                d.sound.flow_noise = 0.0;
                d.sound.intake = 0.0;
                d.sound.mechanical = 0.0;
            }
            let rpm = (0.7 * d.redline_rpm).round();
            let probe = |throttle: f64| {
                let mut e = Engine::new(d.clone(), rpm, 1);
                let c = Controls {
                    throttle,
                    hold_rpm: Some(rpm),
                    ..Controls::default()
                };
                run(&mut e, 0.6, &c);
                let (mut m, mut q, mut out, mut pevo) = (vec![], vec![], vec![], 0.0f64);
                for _ in 0..(0.2 / DT) as usize {
                    out.push(e.step(DT, &c));
                    let f = e.flows[0];
                    m.push(f);
                    q.push(exhaust_source_flow(f, crate::gas::CP * e.cyl[0].port_t));
                    pevo = pevo.max(e.cylinder(0).pressure_pa);
                }
                (ac(&m), ac(&q), ac(&out), pevo, out, q, m)
            };
            let (w, c) = (probe(1.0), probe(0.0));
            if !layers {
                // per cycle order: tail and source, overrun relative to full power (dB)
                let amp = |x: &[f64], k: usize| {
                    let w0 = 2.0 * PI * rpm / 120.0 * k as f64 * DT;
                    let (mut re, mut im) = (0.0, 0.0);
                    for (n, v) in x.iter().enumerate() {
                        re += v * (w0 * n as f64).cos();
                        im += v * (w0 * n as f64).sin();
                    }
                    (re * re + im * im).sqrt()
                };
                let row: Vec<String> = (1..=12)
                    .map(|k| {
                        format!(
                            "{k}:{:+.0}/{:+.0}",
                            20.0 * (amp(&c.5, k) / amp(&w.5, k)).log10(),
                            20.0 * (amp(&c.4, k) / amp(&w.4, k)).log10()
                        )
                    })
                    .collect();
                eprintln!("   order: source/tail dB  {}", row.join(" "));
                let flips = |m: &[f64]| m.windows(2).filter(|w| w[0] * w[1] < 0.0).count();
                let cycles = 0.2 * rpm / 120.0;
                eprintln!(
                    "   valve flow reversals per cycle: full {:.1}, overrun {:.1}",
                    flips(&w.6) as f64 / cycles,
                    flips(&c.6) as f64 / cycles
                );
            }
            let db = |a: f64, b: f64| 20.0 * (b / a).log10();
            eprintln!(
                "{:22} layers {layers:5} {rpm}: mass {:+.1} dB, volume {:+.1} dB, radiated {:+.1} dB; peak p {:.1}/{:.1} bar",
                d.name,
                db(w.0, c.0),
                db(w.1, c.1),
                db(w.2, c.2),
                w.3 / 1e5,
                c.3 / 1e5
            );
        }
    }

    #[test]
    fn woschni_gives_the_published_correlation() {
        // h = 3.26·B^−0.2·p^0.8·T^−0.55·w^0.8 (Heywood eq. 12.19, p in kPa): B = 0.1 m,
        // p = 5000 kPa, T = 2000 K, w = 10 m/s → 3.26 · 1.58489 · 910.28 · 0.015291 · 6.30957
        // = 453.8 W/(m²·K)
        assert_relative_eq!(
            woschni_htc(0.1, 5.0e6, 2000.0, 10.0),
            453.76,
            max_relative = 1e-4
        );
        // the gas velocity: 2.28·S̄p while compressing; 6.18·S̄p during gas exchange; plus the
        // combustion term 3.24e-3·(V_d·T_r / p_r·V_r)·(p − p_mot)
        assert_relative_eq!(woschni_velocity(10.0, false, 0.0), 22.8);
        assert_relative_eq!(woschni_velocity(10.0, true, 0.0), 61.8);
        assert_relative_eq!(woschni_velocity(10.0, false, 1000.0), 22.8 + 3.24);
        // S̄p = 2·S·N (Heywood eq. 2.2): 86 mm at 3000 rpm (50 rev/s) → 8.6 m/s, either way round
        let w = rpm_to_rad_s(3000.0);
        assert_relative_eq!(mean_piston_speed(0.086, w), 8.6, max_relative = 1e-12);
        assert_relative_eq!(mean_piston_speed(0.086, -w), 8.6, max_relative = 1e-12);
        // gas exchange: either valve off its seat
        assert!(!gas_exchange(0.0, 0.0));
        assert!(gas_exchange(1e-6, 0.0));
        assert!(gas_exchange(0.0, 1e-6));
        assert!(gas_exchange(1e-6, 1e-6));
    }

    #[test]
    fn the_spark_slews_at_half_the_crank_travel() {
        // 3000 rpm = 18 000 °/s; a 0.1 ms step travels 1.8°, so the trim moves at most 0.9°,
        // and the same running backwards
        for rpm in [3000.0, -3000.0] {
            let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
            e.omega = rpm_to_rad_s(rpm);
            e.idle_spark_deg = 2.0;
            assert_relative_eq!(e.slew_spark(12.0, 1e-4), 2.9, max_relative = 1e-12);
            assert_relative_eq!(e.slew_spark(-8.0, 1e-4), 1.1, max_relative = 1e-12);
            assert_relative_eq!(e.slew_spark(2.5, 1e-4), 2.5, max_relative = 1e-12);
        }
    }

    #[test]
    fn each_cylinder_fires_at_its_spark_angle_before_top_centre() {
        // A charge fires on the first step past `spark_advance_at(rpm) + idle trim` degrees
        // before its firing top centre (crank angle 720°), not half a turn late, and a retarded
        // trim (negative) fires it later. At idle the ECU holds a retard, so the trim is real.
        let mut e = Engine::new(EngineDef::inline4(), 800.0, 1);
        let c = Controls::default();
        run(&mut e, 3.0, &c);
        let mut checked = 0;
        let mut trims = (f64::MAX, f64::MIN);
        for _ in 0..(0.5 / DT) as usize {
            let armed: Vec<bool> = e.cyl.iter().map(|cs| cs.armed).collect();
            e.step(DT, &c);
            // the crank's travel this step, degrees
            let travel = e.omega.abs().to_degrees() * DT;
            let want = e.def.spark_advance_at(e.rpm()) + e.idle_spark_deg;
            for (cs, was) in e.cyl.iter().zip(armed) {
                if was && !cs.armed {
                    // signed: an idle retard can put the spark just after top centre
                    let btdc = (720.0 - cs.cycle_deg + 360.0).rem_euclid(720.0) - 360.0;
                    assert!(
                        btdc <= want + 0.01 && btdc > want - travel - 0.01,
                        "fired {btdc:.3}° BTDC, spark at {want:.3}° (step {travel:.3}°)"
                    );
                    checked += 1;
                    trims = (trims.0.min(e.idle_spark_deg), trims.1.max(e.idle_spark_deg));
                }
            }
        }
        // 800 rpm, 4 cylinders: 6.7 cycles/s · 4 · 0.5 s ≈ 13 firings
        assert!(checked >= 12, "{checked} firings checked");
        assert!(trims.1 < -1.0, "idle trim {trims:?}° should be a retard");
    }

    #[test]
    fn the_walls_take_a_textbook_share_of_the_fuel_energy() {
        // Heat to the combustion-chamber walls is ~15–35 % of the fuel energy at part load
        // (Heywood, ICE Fundamentals, §12.1); a Woschni coefficient lands an inline-4 at
        // 3000 rpm, half throttle, inside that band. And it grows with load: h ∝ p^0.8, so a
        // full cylinder loses about (charge ratio)^0.8 more per cycle than a throttled one.
        let heat = |throttle: f64| {
            let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
            let c = Controls {
                throttle,
                hold_rpm: Some(3000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let (q0, f0) = (e.wall_heat_j, e.fuel_energy_j);
            run(&mut e, 0.5, &c);
            (e.wall_heat_j - q0, e.fuel_energy_j - f0)
        };
        let (q, f) = heat(0.5);
        let share = q / f;
        assert!(
            (0.15..0.35).contains(&share),
            "wall heat {share:.3} of the fuel"
        );
        let ((q_full, f_full), (q_light, f_light)) = (heat(1.0), heat(0.15));
        let load = f_full / f_light;
        let ratio = q_full / q_light;
        assert!(
            (ratio / load.powf(0.8) - 1.0).abs() < 0.15,
            "heat ratio {ratio:.2} for a charge ratio {load:.2} (want ≈ {:.2})",
            load.powf(0.8)
        );
    }

    /// An inline-4 at `rpm` with the idle loop's state set by hand, for one controller step.
    fn idling_at(rpm: f64, integral: f64, rate: f64, spark: f64) -> Engine {
        let mut e = Engine::new(EngineDef::inline4(), rpm, 1);
        e.omega = rpm_to_rad_s(rpm);
        e.iac_last_rpm = rpm;
        e.iac_integral = integral;
        e.iac_rate = rate;
        e.idle_spark_deg = spark;
        e
    }

    #[test]
    fn the_idle_air_scale_is_what_the_engine_swallows_at_idle() {
        // inline-4, 86 × 86 mm (1.9982 L), idle 800 rpm, Ø60 mm throttle:
        // ρ(30 kPa, 300 K) = 0.34843 kg/m³; ṁ = 0.34843 · 1.9982e-3 · 800/120 = 4.6416e-3 kg/s;
        // choked flux from the atmosphere 0.040418 · 101325/√300 = 236.45 kg/(s·m²);
        // Cd·A = 0.8 · π/4 · 0.06² = 2.2619e-3 m² → 4.6416e-3 / 236.45 / 2.2619e-3 = 0.0086787
        let e = Engine::new(EngineDef::inline4(), 800.0, 1);
        assert_relative_eq!(e.idle_air_scale(), 0.008_678_7, max_relative = 1e-4);
    }

    #[test]
    fn one_idle_control_step_by_hand() {
        let dt = 1e-3;
        let scale = 0.008_678_7;
        // 780 rpm against 800: err = 0.025; falling at 400 rpm/s (filtered: no new change,
        // so the rate decays by dt/τ = 1/150 → 397.33)
        let mut e = idling_at(780.0, 0.0, -400.0, -3.0);
        e.idle_control(dt, &Controls::default());
        assert_relative_eq!(e.iac_rate, -397.333, max_relative = 1e-5);
        // integral 0.5 · 0.025 · 1e-3 = 1.25e-5; air = scale · (1.25e-5 + 0.05 · 0.025)
        assert_relative_eq!(e.iac_integral, 1.25e-5, max_relative = 1e-9);
        assert_relative_eq!(e.iac, scale * 1.2625e-3, max_relative = 1e-4);
        // spark: 80 · 0.025 + 10 · 397.33/800 = 6.9667 from the −10° reserve → −3.0333°
        assert_relative_eq!(e.idle_spark_deg, -3.0333, max_relative = 1e-4);
        // a new speed reading: 700 → 720 rpm in 1 ms is 20 000 rpm/s, filtered by 1/150
        let mut e = idling_at(720.0, 0.0, 0.0, -10.0);
        e.iac_last_rpm = 700.0;
        e.idle_control(dt, &Controls::default());
        assert_relative_eq!(e.iac_rate, 133.333, max_relative = 1e-5);
    }

    #[test]
    fn the_idle_integral_holds_while_falling_from_above_and_saturates() {
        // 100 rpm above the 800 target, falling at 400 rpm/s: within the 1 s look-ahead it gets
        // there (900 − 400 = 500), so it holds and the spark path catches the engine
        let mut e = idling_at(900.0, 0.1, -400.0, -10.0);
        e.idle_control(1e-3, &Controls::default());
        assert_eq!(e.iac_integral, 0.1);
        // just fast enough: 900 − 100.2 = 799.8 within the second, so it holds (the rate
        // filter takes 100.9 to 100.2 in this 1 ms step)
        let mut e = idling_at(900.0, 0.1, -100.9, -10.0);
        e.idle_control(1e-3, &Controls::default());
        assert_eq!(e.iac_integral, 0.1);
        // drifting down at 50 rpm/s it would still be at 850: it closes
        let mut e = idling_at(900.0, 0.1, -50.0, -10.0);
        e.idle_control(1e-3, &Controls::default());
        assert!(e.iac_integral < 0.1);
        // above target and steady: it closes
        let mut e = idling_at(900.0, 0.1, 0.0, -10.0);
        e.idle_control(1e-3, &Controls::default());
        assert!(e.iac_integral < 0.1);
        // the bypass stops at 3 % of the bore: 0.03 − 0.0055 added
        let mut e = idling_at(330.0, 5.0, 0.0, -10.0);
        e.idle_control(1e-3, &Controls::default());
        assert_relative_eq!(e.iac, 0.03 - 0.0055, max_relative = 1e-9);
        // and closes no further than its floor below the base bypass
        let mut e = idling_at(1400.0, -5.0, 0.0, -10.0);
        e.idle_control(1e-3, &Controls::default());
        assert_relative_eq!(e.iac, IAC_MIN * 0.0055, max_relative = 1e-9);
    }

    #[test]
    fn the_cut_offs_switch_with_their_hysteresis() {
        // Inline-4: idle 800, cut-off 1500 → reactivation at min(800 + 300, 1500 − 100) = 1100;
        // redline 7000 → the limiter resumes at 6800.
        let at = |e: &mut Engine, rpm: f64, throttle: f64| {
            e.omega = rpm_to_rad_s(rpm);
            e.fuel_cutoffs(throttle);
            (e.overrun_cut, e.limiter_cut)
        };
        let mut e = Engine::new(EngineDef::inline4(), 800.0, 1);
        assert_eq!(at(&mut e, 1200.0, 0.0), (false, false), "below the cut-off");
        assert_eq!(at(&mut e, 1600.0, 0.02), (false, false), "pedal pressed");
        assert_eq!(at(&mut e, 1600.0, 0.0), (true, false));
        assert_eq!(at(&mut e, 1200.0, 0.0), (true, false), "held down to 1100");
        assert_eq!(at(&mut e, 1090.0, 0.0), (false, false));
        assert_eq!(at(&mut e, 1200.0, 0.0), (false, false), "stays fuelled");
        assert_eq!(at(&mut e, 7000.0, 1.0), (false, true));
        assert_eq!(at(&mut e, 6900.0, 1.0), (false, true), "held down to 6800");
        assert_eq!(at(&mut e, 6790.0, 1.0), (false, false));
        assert_eq!(at(&mut e, 6900.0, 1.0), (false, false), "stays fuelled");
        // Single: idle 1200, cut-off 1500 → min(1500, 1400) = 1400
        let mut e = Engine::new(EngineDef::single(), 1200.0, 1);
        assert_eq!(at(&mut e, 1600.0, 0.0), (true, false));
        assert_eq!(at(&mut e, 1450.0, 0.0), (true, false));
        assert_eq!(at(&mut e, 1390.0, 0.0), (false, false));
    }

    #[test]
    fn the_idle_loop_acts_only_on_a_released_pedal_and_a_running_engine() {
        let acts_cut = |rpm: f64, c: Controls, cut: bool| {
            let mut e = idling_at(rpm, 0.0, 0.0, -10.0);
            e.overrun_cut = cut;
            e.idle_control(1e-3, &c);
            e.iac_integral != 0.0
        };
        let acts = |rpm: f64, c: Controls| acts_cut(rpm, c, false);
        let idle = Controls::default();
        // 800 rpm target: above 0.4 · 800 = 320 rpm it runs on its own, at any speed while
        // fuelled; it rests while the overrun cut-off holds
        assert!(acts(321.0, idle));
        assert!(!acts(300.0, idle), "cranking");
        assert!(acts(1501.0, idle));
        assert!(!acts_cut(1501.0, idle, true), "overrun");
        assert!(acts(
            700.0,
            Controls {
                throttle: 0.01,
                ..idle
            }
        ));
        assert!(
            !acts(
                700.0,
                Controls {
                    throttle: 0.02,
                    ..idle
                }
            ),
            "pedal pressed"
        );
        assert!(
            !acts(
                700.0,
                Controls {
                    ignition: false,
                    ..idle
                }
            ),
            "ignition off"
        );
        // inactive, the spark trim returns towards 0 (slewed)
        let mut e = idling_at(700.0, 0.0, 0.0, -10.0);
        e.idle_control(
            1e-3,
            &Controls {
                throttle: 0.5,
                ..idle
            },
        );
        assert!(e.idle_spark_deg > -10.0);
    }

    /// The user's journey on one engine: crank on the starter, let it idle, floor it for 0.3 s,
    /// let go. The ECU's idle control must hold `idle_rpm`, and overrun fuel cut, the dashpot
    /// and the idle control must bring it back from the blip. The single used to stall at idle,
    /// the big engines on the way down.
    #[test]
    fn the_cheap_pipe_scheme_runs_the_same_engine() {
        // the LFA's 1D intake stepped with MUSCL-RK2: the same engine, finite and close in
        // level, but not the same numbers (the fidelity reaches the pipes)
        let run = |f: Fidelity| {
            let mut e = Engine::with_fidelity(EngineDef::lfa_v10(), 6000.0, 1, f);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(6000.0),
                ..Controls::default()
            };
            (0..(0.2 / DT) as usize)
                .map(|_| e.step(DT, &c))
                .collect::<Vec<f64>>()
        };
        let weno = run(Fidelity::default());
        let muscl = run(Fidelity {
            pipe_scheme: crate::fv::Scheme::MusclRk2,
        });
        assert!(muscl.iter().all(|x| x.is_finite()));
        let rms = |x: &[f64]| (x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64).sqrt();
        assert!((rms(&muscl) / rms(&weno) - 1.0).abs() < 0.2);
        assert!(weno.iter().zip(&muscl).any(|(a, b)| a != b));
    }

    fn starts_idles_and_survives_a_blip(d: EngineDef) {
        let target = d.idle_rpm;
        let mut e = Engine::new(d.clone(), 0.0, 1);
        let crank = Controls {
            starter: true,
            ..Controls::default()
        };
        run(&mut e, 1.0, &crank);
        // after the start flare the ECU brings the idle down over ~10 s, as a car does
        run(&mut e, 11.0, &Controls::default());
        let idle: Vec<f64> = (0..(2.0 / DT) as usize)
            .map(|_| {
                e.step(DT, &Controls::default());
                e.rpm()
            })
            .collect();
        let mean = idle.iter().sum::<f64>() / idle.len() as f64;
        let low = idle.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            (mean / target - 1.0).abs() < 0.08,
            "{}: idle {mean} vs {target}",
            d.name
        );
        // a lumpy idle swings; a stall is 0
        assert!(low > 0.55 * target, "{}: idle dipped to {low}", d.name);
        let blip = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        run(&mut e, 0.3, &blip);
        assert!(
            e.rpm() > 1.3 * target,
            "{}: blip only to {}",
            d.name,
            e.rpm()
        );
        let mut low = f64::INFINITY;
        for _ in 0..(4.0 / DT) as usize {
            e.step(DT, &Controls::default());
            low = low.min(e.rpm());
        }
        assert!(
            low > 0.3 * target,
            "{}: fell to {low} after the blip",
            d.name
        );
        assert!(
            (e.rpm() / target - 1.0).abs() < 0.25,
            "{}: back at {} vs {target}",
            d.name,
            e.rpm()
        );
    }

    /// Warm idle, then the pedal floored in neutral until `rpm`, then released: the time to
    /// get there, and the speed trace for `after` seconds once released.
    fn rev_and_release(d: EngineDef, rpm: f64, after: f64) -> (f64, Vec<f64>) {
        let mut e = Engine::new(d.clone(), d.idle_rpm, 1);
        run(&mut e, 4.0, &Controls::default());
        let wot = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        let mut t = 0.0;
        while e.rpm() < rpm && t < 3.0 {
            e.step(DT, &wot);
            t += DT;
        }
        let trace = (0..(after / DT) as usize)
            .map(|_| {
                e.step(DT, &Controls::default());
                e.rpm()
            })
            .collect();
        (t, trace)
    }

    #[test]
    fn the_lfa_snaps_from_idle_to_9000_in_six_tenths() {
        // Measured by the LFA's chief engineer (H. Tanahashi, quoted by evo and Road & Track,
        // 2009): unloaded, a stab of the pedal takes the V10 from idle to 9000 rpm in 0.6 s.
        let (t, _) = rev_and_release(EngineDef::lfa_v10(), 8990.0, 0.0);
        assert!((0.5..0.7).contains(&t), "idle to 9000 in {t:.3} s");
    }

    #[test]
    fn released_from_high_revs_every_engine_settles_at_its_idle() {
        // Overrun fuel cut until injection reactivates near idle, then the idle control takes
        // over (Bosch, Gasoline Engine Management): back at idle within a few seconds and
        // staying there, not hung at the cut-off speed.
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            EngineDef::flatplane_v8(),
            EngineDef::crossplane_v8(),
        ] {
            let target = d.idle_rpm;
            let (_, trace) = rev_and_release(d.clone(), 0.8 * d.redline_rpm, 8.0);
            let tail = &trace[(6.0 / DT) as usize..];
            let mean = tail.iter().sum::<f64>() / tail.len() as f64;
            let low = tail.iter().copied().fold(f64::INFINITY, f64::min);
            assert!(
                (mean / target - 1.0).abs() < 0.1,
                "{}: settled at {mean:.0} vs {target}; trace {:?}",
                d.name,
                trace
                    .iter()
                    .step_by((0.25 / DT) as usize)
                    .map(|r| *r as i64)
                    .collect::<Vec<_>>()
            );
            assert!(low > 0.6 * target, "{}: dipped to {low:.0}", d.name);
        }
    }

    #[test]
    fn the_rev_limiter_bounces_off_the_redline() {
        // A fuel cut-off limiter with hysteresis (Bosch, Gasoline Engine Management, "engine
        // speed limitation"): injection stops at the redline and resumes some way below, so a
        // floored engine in neutral bounces in a rhythm a few times a second.
        let d = EngineDef::inline4();
        let red = d.redline_rpm;
        let mut e = Engine::new(d, 3000.0, 1);
        let wot = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        run(&mut e, 1.5, &wot);
        let trace: Vec<f64> = (0..(1.0 / DT) as usize)
            .map(|_| {
                e.step(DT, &wot);
                e.rpm()
            })
            .collect();
        let hi = trace.iter().copied().fold(0.0, f64::max);
        let lo = trace.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(hi < red + 150.0, "overshoot to {hi:.0}");
        assert!(lo < red - 100.0, "no bounce: down to {lo:.0} only");
        // bounces: crossings of the middle of the band, upwards
        let mid = 0.5 * (hi + lo);
        let ups = trace
            .windows(2)
            .filter(|w| w[0] < mid && w[1] >= mid)
            .count();
        assert!((2..=30).contains(&ups), "{ups} bounces in a second");
    }

    #[test]
    fn overrun_pulls_the_manifold_to_a_deep_vacuum() {
        // Closed throttle at speed: the engine pumps the manifold down to ~0.2–0.3 bar
        // (Heywood, ICE Fundamentals, §7.6.1 and Fig. 7-19, part-load intake vacuum); the idle
        // air alone cannot feed 5000 rpm.
        let mut e = Engine::new(EngineDef::inline4(), 5000.0, 1);
        let c = Controls {
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 3.0, &c);
        let p = e.plenum_pressure_pa();
        assert!(p < 0.35e5, "manifold at {:.2} bar on overrun", p / 1e5);
    }

    #[test]
    fn every_preset_comes_in_its_own_vehicle() {
        use crate::drivetrain::VehicleDef;
        assert_eq!(EngineDef::single().vehicle, VehicleDef::single());
        assert_eq!(EngineDef::inline4().vehicle, VehicleDef::inline4());
        assert_eq!(EngineDef::lfa_v10().vehicle, VehicleDef::lfa_v10());
        assert_eq!(
            EngineDef::flatplane_v8().vehicle,
            VehicleDef::flatplane_v8()
        );
        assert_eq!(
            EngineDef::crossplane_v8().vehicle,
            VehicleDef::crossplane_v8()
        );
    }

    #[test]
    fn the_single_starts_idles_and_survives_a_blip() {
        starts_idles_and_survives_a_blip(EngineDef::single());
    }

    #[test]
    fn the_inline4_starts_idles_and_survives_a_blip() {
        starts_idles_and_survives_a_blip(EngineDef::inline4());
    }

    #[test]
    fn the_lfa_starts_idles_and_survives_a_blip() {
        starts_idles_and_survives_a_blip(EngineDef::lfa_v10());
    }

    #[test]
    fn the_flatplane_starts_idles_and_survives_a_blip() {
        starts_idles_and_survives_a_blip(EngineDef::flatplane_v8());
    }

    #[test]
    fn the_crossplane_starts_idles_and_survives_a_blip() {
        starts_idles_and_survives_a_blip(EngineDef::crossplane_v8());
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
    fn extra_inertia_slows_the_crank_response() {
        // From rest with the starter on and no fuel: one step gives ω = (T_gas + T_starter)·dt/J.
        // Both engines see the same torque, so doubling J (extra = flywheel inertia) halves ω.
        let run_one = |extra: f64| {
            let mut e = Engine::new(EngineDef::single(), 0.0, 1);
            let c = Controls {
                ignition: false,
                starter: true,
                extra_inertia_kgm2: extra,
                ..Controls::default()
            };
            e.step(DT, &c);
            e.rpm()
        };
        let (base, doubled) = (
            run_one(0.0),
            run_one(EngineDef::single().flywheel_inertia_kgm2),
        );
        assert!(base > 0.0);
        assert_relative_eq!(doubled, base / 2.0, max_relative = 1e-9);
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
    fn lfa_v10_breathes_through_ten_unrestrictive_itbs() {
        // The LR engine has one throttle body per cylinder at the trumpet (Toyota LFA press kit).
        let d = EngineDef::lfa_v10();
        assert_eq!(d.intake.model, IntakeModel::Fv);
        assert!(d.intake.individual_throttles);
        assert_eq!(d.intake.runner_length_m, 0.25);
        assert_eq!(d.intake.runner_diameter_m, 0.045);
        // throttle_diameter_m is the total bore the ten share: wide open, each throttle body
        // is at least as wide as its runner (√10 · 45 mm = 142.3 mm), so the runner sets the flow
        assert!(d.throttle_diameter_m.powi(2) / 10.0 >= d.intake.runner_diameter_m.powi(2));
    }

    /// Firing intervals (deg) between consecutive pulses into each exhaust bank, sorted.
    fn bank_intervals(d: &EngineDef) -> Vec<Vec<f64>> {
        let off = d.phase_offsets_deg();
        d.exhaust
            .groups(d.cylinders)
            .into_iter()
            .map(|g| {
                let mut a: Vec<f64> = g.iter().map(|&c| off[c]).collect();
                a.sort_by(f64::total_cmp);
                let mut iv: Vec<f64> = a.windows(2).map(|w| w[1] - w[0]).collect();
                iv.push(720.0 - a[a.len() - 1] + a[0]);
                iv.sort_by(f64::total_cmp);
                iv
            })
            .collect()
    }

    #[test]
    fn flatplane_v8_fires_each_bank_evenly() {
        // A flat crank is two inline-4s at 90°: every bank gets a pulse each 180°.
        let d = EngineDef::flatplane_v8();
        // 8 · π/4 · 0.094² · 0.081 = 4.4966e-3 m³ (Ferrari F136 F: 4,497 cc)
        assert_relative_eq!(d.displacement_m3(), 4.4966e-3, max_relative = 1e-4);
        assert_eq!(bank_intervals(&d), vec![vec![180.0; 4]; 2]);
    }

    #[test]
    fn crossplane_v8_fires_each_bank_unevenly() {
        // The cross crank fires 90° apart overall but 90/180/180/270 within a bank: the burble.
        let d = EngineDef::crossplane_v8();
        // 8 · π/4 · 0.10325² · 0.092 = 6.1625e-3 m³ (GM LS3: 6,162 cc)
        assert_relative_eq!(d.displacement_m3(), 6.1625e-3, max_relative = 1e-4);
        let mut off = d.phase_offsets_deg();
        off.sort_by(f64::total_cmp);
        assert_eq!(off, (0..8).map(|k| 90.0 * k as f64).collect::<Vec<_>>());
        assert_eq!(bank_intervals(&d), vec![vec![90.0, 180.0, 180.0, 270.0]; 2]);
    }

    #[test]
    fn both_v8s_idle_and_make_power() {
        // Published: F136 F 540 N·m @ 6000; LS3 575 N·m @ 4600. Model bands, not calibration.
        for (d, rpm) in [
            (EngineDef::flatplane_v8(), 6000.0),
            (EngineDef::crossplane_v8(), 4600.0),
        ] {
            let mut e = Engine::new(d.clone(), 1000.0, 1);
            run(&mut e, 2.0, &Controls::default());
            assert!(
                (450.0..1800.0).contains(&e.rpm()),
                "{}: idle {}",
                d.name,
                e.rpm()
            );
            let mut e = Engine::new(d.clone(), rpm, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            run(&mut e, 1.0, &c);
            assert!(
                (300.0..800.0).contains(&e.torque_nm()),
                "{}: {}",
                d.name,
                e.torque_nm()
            );
        }
    }

    /// Output of `def` at 3000 rpm WOT with the listener at `angle` (texture off: tonal only).
    fn heard_at(mut def: EngineDef, angle: f64) -> Vec<f64> {
        def.sound = crate::sound::SoundDef {
            flow_noise: 0.0,
            intake: 0.0,
            mechanical: 0.0,
            combustion_jitter: 0.0,
            listener_angle_deg: angle,
            ..crate::sound::SoundDef::default()
        };
        let mut e = Engine::new(def, 3000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c)
    }

    #[test]
    fn the_engine_is_heard_through_the_listener() {
        // One tail: where you stand changes nothing (only the common path, which is dropped).
        // (to rounding: √(sin² + cos²)·r is r in the last bit)
        let (a, b) = (
            heard_at(EngineDef::inline4(), 0.0),
            heard_at(EngineDef::inline4(), 60.0),
        );
        for (x, y) in a.iter().zip(&b) {
            assert_relative_eq!(x, y, max_relative = 1e-12, epsilon = 1e-15);
        }
        // Two tails heard from the side: the far one arrives later, so the sum changes.
        let a = heard_at(EngineDef::crossplane_v8(), 0.0);
        let b = heard_at(EngineDef::crossplane_v8(), 60.0);
        let diff: f64 = a.iter().zip(&b).map(|(x, y)| (x - y).powi(2)).sum();
        let total: f64 = a.iter().map(|x| x * x).sum();
        assert!(
            diff > 0.01 * total,
            "listener angle had no effect: {diff} vs {total}"
        );
    }

    #[test]
    fn moving_the_listener_retunes_in_place() {
        // A tuning change, not a rebuild: the engine keeps running and builds a new listener.
        let mut e = Engine::new(EngineDef::crossplane_v8(), 3000.0, 1);
        run(&mut e, 0.01, &Controls::default());
        assert!(e.listener.is_some());
        let mut d = EngineDef::crossplane_v8();
        d.sound.listener_angle_deg = 75.0;
        assert!(e.retune(d));
        assert!(e.listener.is_none());
        run(&mut e, 0.001, &Controls::default());
        assert!(e.listener.is_some());
    }

    #[test]
    fn the_intake_model_in_use_is_reported() {
        let mut e = Engine::new(EngineDef::inline4(), 1000.0, 1);
        run(&mut e, 0.001, &Controls::default());
        assert!(!e.intake_is_fv());
        let mut e = Engine::new(EngineDef::lfa_v10(), 1000.0, 1);
        run(&mut e, 0.001, &Controls::default());
        assert!(e.intake_is_fv());
    }

    #[test]
    fn the_trumpets_are_heard_at_their_mean_flow() {
        // With only the intake texture on, its signal(1) − signal(0) is the trumpets' radiation
        // 0.3·c·Σρu plus the throttle tone 0.15·c·ṁ/A_bore. Over the runners Σρu averages
        // ṁ/A_runner, so its mean is c·ṁ·(0.3/A_runner + 0.15/A_bore), c = 343 m/s.
        let out = |level: f64| {
            let mut d = EngineDef::lfa_v10();
            d.sound = crate::sound::SoundDef {
                flow_noise: 0.0,
                mechanical: 0.0,
                combustion_jitter: 0.0,
                intake: level,
                ..crate::sound::SoundDef::default()
            };
            let mut e = Engine::new(d, 4000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(4000.0),
                ..Controls::default()
            };
            run(&mut e, 0.5, &c);
            let m0 = e.inducted_kg();
            let y: Vec<f64> = (0..(0.3 / DT) as usize)
                .map(|_| {
                    e.step(DT, &c);
                    e.texture_signal
                })
                .collect();
            (y, (e.inducted_kg() - m0) / 0.3)
        };
        let ((quiet, _), (heard, mdot)) = (out(0.0), out(1.0));
        let mean = heard.iter().zip(&quiet).map(|(a, b)| a - b).sum::<f64>() / heard.len() as f64;
        let d = EngineDef::lfa_v10();
        let a_runner = std::f64::consts::PI * 0.25 * d.intake.runner_diameter_m.powi(2);
        let a_bore = DISCHARGE_CD * std::f64::consts::PI * 0.25 * d.throttle_diameter_m.powi(2);
        let want = 343.0 * mdot * (0.3 / a_runner + 0.15 / a_bore);
        assert!(
            (mean / want - 1.0).abs() < 0.1,
            "mean {mean} vs {want} (ṁ {mdot})"
        );
    }

    #[test]
    fn the_vee_presets_stand_in_their_banks() {
        // LFA: 72° V10 (Toyota LR); both V8s: 90°; the flat plane's banks 1–4 / 5–8, the
        // cross plane's odd/even, as their collectors
        let lfa = EngineDef::lfa_v10();
        assert_eq!(lfa.layout.bank_angle_deg, 72.0);
        assert_eq!(lfa.layout.banks, lfa.exhaust.collectors);
        for d in [EngineDef::flatplane_v8(), EngineDef::crossplane_v8()] {
            assert_eq!(d.layout.bank_angle_deg, 90.0, "{}", d.name);
            assert_eq!(d.layout.banks, d.exhaust.collectors, "{}", d.name);
            assert!(crate::config::validate(&d).is_ok());
        }
        assert_eq!(
            EngineDef::inline4().layout,
            crate::layout::Layout::default()
        );
    }

    #[test]
    fn every_preset_revs_up_from_idle_at_full_throttle() {
        // Flooring it from 900 rpm must not stall (the flat-plane did on a fixed 28° advance:
        // see `spark_advance_follows_an_rpm_curve`).
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            EngineDef::flatplane_v8(),
            EngineDef::crossplane_v8(),
        ] {
            let mut e = Engine::new(d.clone(), 900.0, 1);
            let c = Controls {
                throttle: 1.0,
                ..Controls::default()
            };
            run(&mut e, 0.5, &c);
            assert!(e.rpm() > 2000.0, "{}: {} rpm", d.name, e.rpm());
        }
    }

    #[test]
    fn the_listener_hears_the_tails_through_one_delay_line() {
        // Textures off, the output is the listener's mix of the tail signals. Standing to the
        // side of the LFA's two tails delays the far one: the line must persist across steps.
        let mut d = EngineDef::lfa_v10();
        d.sound = crate::sound::SoundDef {
            flow_noise: 0.0,
            mechanical: 0.0,
            combustion_jitter: 0.0,
            intake: 0.0,
            listener_angle_deg: 90.0,
            ..crate::sound::SoundDef::default()
        };
        let mut e = Engine::new(d.clone(), 3000.0, 1);
        let c = Controls {
            throttle: 0.5,
            ..Controls::default()
        };
        run(&mut e, 0.05, &c);
        let mut l = crate::listener::Listener::new(
            2,
            d.exhaust.tail_spacing_m,
            d.sound.listener_distance_m,
            90.0,
            PI * 0.25 * d.exhaust.tail_diameter_m.powi(2),
            DT,
        );
        let mut tails = Vec::new();
        let (mut got, mut want) = (Vec::new(), Vec::new());
        for _ in 0..4_000 {
            got.push(e.step(DT, &c));
            tails.push(match (&e.net, &e.fv) {
                (_, Some(fv)) => fv.last_tails().to_vec(),
                (Some(net), None) => net.last_tails().to_vec(),
                (None, None) => vec![],
            });
        }
        // the engine's line was primed during warm-up; ours starts from the same point after
        // it has been filled once
        let warm = 2_000;
        for t in &tails {
            want.push(l.process(t));
        }
        for k in warm..got.len() {
            assert!(
                (got[k] - want[k]).abs() <= 1e-9 * want[k].abs().max(1e-3),
                "sample {k}: {} vs {}",
                got[k],
                want[k]
            );
        }
    }

    #[test]
    fn spark_advance_follows_an_rpm_curve() {
        // The burn takes about a fixed time, so its MBT advance grows with speed (Heywood §9.3).
        // Configured 28°: × (0.35 + 0.65·900/4000) = 13.895° at 900 rpm; full from 4000 rpm.
        let d = EngineDef::flatplane_v8();
        assert_relative_eq!(d.spark_advance_at(900.0), 13.895, max_relative = 1e-9);
        assert_relative_eq!(d.spark_advance_at(4000.0), 28.0);
        assert_relative_eq!(d.spark_advance_at(9000.0), 28.0);
    }

    #[test]
    fn a_retarded_idle_spark_lets_a_light_flywheel_rev_up() {
        // The flat-plane stalled when floored from idle on a fixed 28° with 0.08 kg·m².
        let mut d = EngineDef::flatplane_v8();
        d.flywheel_inertia_kgm2 = 0.08;
        let mut e = Engine::new(d, 900.0, 1);
        let c = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        run(&mut e, 0.5, &c);
        assert!(e.rpm() > 2000.0, "{} rpm", e.rpm());
    }

    #[test]
    fn each_collector_is_a_tail_pipe_jet() {
        // two 5-into-1 banks, two tails; the inline-4's 4-into-1 has one
        assert_eq!(Engine::new(EngineDef::lfa_v10(), 1000.0, 1).tail_pipes, 2);
        assert_eq!(Engine::new(EngineDef::inline4(), 1000.0, 1).tail_pipes, 1);
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
        // Find a stall where the starter traps a charge (which seed does depends on the dynamics).
        let max_p = |e: &Engine| {
            (0..4)
                .map(|i| e.cylinder(i).pressure_pa)
                .fold(0.0, f64::max)
        };
        let crank = Controls {
            ignition: false,
            starter: true,
            ..Controls::default()
        };
        let trapped = (0..20u64).find_map(|seed| {
            let mut e = stalled(EngineDef::inline4(), seed, 0.5 + 0.013 * seed as f64);
            run(&mut e, 0.4, &crank);
            (max_p(&e) > 4e5).then_some(e)
        });
        let mut e = trapped.expect("no stall position trapped a compression charge");
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
            leveling: 0.0,
            ..crate::sound::SoundDef::default()
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
    fn the_throttle_plate_takes_a_tenth_of_a_second_to_open() {
        let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
        for _ in 0..(0.05 / DT) as usize {
            e.plate_step(1.0, DT);
        }
        assert_relative_eq!(e.plate, 0.5, max_relative = 1e-3);
        for _ in 0..(0.06 / DT) as usize {
            e.plate_step(1.0, DT);
        }
        assert_eq!(e.plate, 1.0);
        // and back down at the same speed (the dashpot then holds the last few percent)
        for _ in 0..(0.05 / DT) as usize {
            e.plate_step(0.0, DT);
        }
        assert_relative_eq!(e.plate, 0.5, max_relative = 1e-3);
    }

    #[test]
    fn the_dashpot_lets_the_throttle_close_slowly() {
        // Lifting off WOT, the plate is caught at 3 % of its travel, 1 − cos(0.03·π/2) =
        // 1.11013e-3 of its area, and closes from there with τ = 1 s: e^−1 = 0.368 of the hold
        // after 1 s. Pressing again opens it at once.
        let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
        let full = e.throttle_area(1.0);
        let idle = e.throttle_area(0.0);
        let wot = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.1, &wot);
        e.dashpot_step(0.0, DT);
        let held = e.throttle_area(0.0);
        assert_relative_eq!(held, idle + 1.11013e-3 * full, max_relative = 1e-3);
        for _ in 0..(1.0 / DT) as usize {
            e.dashpot_step(0.0, DT);
        }
        assert_relative_eq!(
            e.throttle_area(0.0),
            idle + 1.11013e-3 * 0.3679 * full,
            max_relative = 1e-3
        );
        e.dashpot_step(1.0, DT);
        assert_relative_eq!(e.throttle_area(1.0), full, max_relative = 1e-9);
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

    #[test]
    fn fv_exhaust_survives_sustained_overrun_at_high_rpm() {
        // Regression: closed throttle at 7000 rpm drove FV cells into near-vacuum with
        // |u| + c beyond the fixed step's CFL limit; a primary reached 1000 bar after 0.4 s.
        let mut e = Engine::new(fv(EngineDef::lfa_v10()), 7000.0, 1);
        let c = Controls {
            hold_rpm: Some(7000.0),
            ..Controls::default()
        };
        let xs = run(&mut e, 0.6, &c);
        assert!(xs.iter().all(|x| x.is_finite()));
        let worst = (0..10)
            .map(|i| e.cylinder(i).pressure_pa)
            .fold(0.0, f64::max);
        assert!(worst < 50e5, "cylinder at {worst} Pa while motoring");
    }

    #[test]
    fn idle_air_opens_with_speed_on_overrun() {
        // 0.0015 of the bore up to 1500 rpm; at 7000 rpm with injection cut,
        // 7000/1500 × 0.0015 = 0.007 (hand); fuelled, it stays the idle bypass
        let full = 1.00531e-3;
        let e = Engine::new(EngineDef::single(), 900.0, 1);
        assert_relative_eq!(e.throttle_area(0.0), 0.0015 * full, max_relative = 1e-5);
        let mut e = Engine::new(EngineDef::single(), 7000.0, 1);
        assert_relative_eq!(e.throttle_area(0.0), 0.0015 * full, max_relative = 1e-5);
        e.overrun_cut = true;
        assert_relative_eq!(e.throttle_area(0.0), 0.007 * full, max_relative = 1e-5);
        assert_relative_eq!(e.throttle_area(1.0), full, max_relative = 1e-5);
    }

    #[test]
    fn overrun_manifold_pressure_stays_realistic() {
        // Regression: closed throttle at 7000 rpm pulled the manifold to 0.06 bar (fixed idle air,
        // 8× the pumping of idle), driving violent exhaust reversion. ECUs open the idle valve on
        // deceleration (dashpot): real overrun MAP ≈ 0.2–0.3 bar.
        for def in [EngineDef::inline4(), EngineDef::lfa_v10()] {
            let mut e = Engine::new(def.clone(), 7000.0, 1);
            let c = Controls {
                hold_rpm: Some(7000.0),
                ..Controls::default()
            };
            run(&mut e, 0.5, &c);
            let map = e.plenum_pressure_pa() / 1e5;
            assert!(
                (0.12..0.45).contains(&map),
                "{}: overrun MAP {map:.3} bar",
                def.name
            );
        }
    }

    #[test]
    fn the_fv_exhaust_keeps_its_waves_between_steps() {
        // The FV network is built once and carries the pulses: its tailpipe makes sound comparable
        // to the waveguide's (a network rebuilt every step would sit at rest and stay silent).
        let rms = |d: EngineDef| {
            let mut e = Engine::new(d, 4000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(4000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let xs = run(&mut e, 0.1, &c);
            (xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64).sqrt()
        };
        let (wg, fv_rms) = (rms(EngineDef::inline4()), rms(fv(EngineDef::inline4())));
        assert!(fv_rms > 0.2 * wg, "fv {fv_rms} vs waveguide {wg}");
        // and the port gauge reads those waves: blowdown pulses of tens of kPa at WOT
        let mut e = Engine::new(fv(EngineDef::inline4()), 4000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(4000.0),
            ..Controls::default()
        };
        run(&mut e, 0.2, &c);
        let mut peak: f64 = 0.0;
        for _ in 0..(0.03 * 192_000.0) as usize {
            e.step(1.0 / 192_000.0, &c);
            peak = peak.max(e.exhaust_port_gauge(0));
        }
        assert!(
            (10_000.0..300_000.0).contains(&peak),
            "port gauge peak {peak} Pa"
        );
    }

    #[test]
    fn a_resting_fv_port_reads_atmospheric() {
        let mut e = Engine::new(fv(EngineDef::single()), 0.0, 1);
        let c = Controls {
            ignition: false,
            ..Controls::default()
        };
        run(&mut e, 0.001, &c);
        let g = e.exhaust_port_gauge(0);
        assert!(g.abs() < 2000.0, "gauge {g} Pa");
    }

    #[test]
    fn the_reverse_cd_ratio_leaves_forward_exhaust_flow_alone() {
        // At WOT the exhaust flow is forward: throttling only the reverse flow barely moves torque
        // (throttling the forward flow too would raise pumping losses by several percent).
        let torque = |ratio: f64| {
            let mut d = EngineDef::inline4();
            d.exhaust_reverse_cd_ratio = ratio;
            let mut e = Engine::new(d, 6000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(6000.0),
                ..Controls::default()
            };
            run(&mut e, 0.5, &c);
            e.torque_nm()
        };
        let (full, half) = (torque(1.0), torque(0.4));
        assert!((half / full - 1.0).abs() < 0.02, "{half} vs {full}");
    }

    #[test]
    fn reverse_flow_through_the_exhaust_valve_is_throttled() {
        // Same overrun state, reverse-flow Cd ratio 1.0 vs 0.5: reversion mass into the cylinders
        // over the exhaust valve drops (choked inflow ∝ area); forward flow is unaffected.
        let reverted = |ratio: f64, model: ExhaustModel| {
            let mut d = EngineDef::inline4();
            d.exhaust_reverse_cd_ratio = ratio;
            d.exhaust.model = model;
            let mut e = Engine::new(d, 6000.0, 1);
            let c = Controls {
                hold_rpm: Some(6000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let before = e.exhaust_reversion_kg();
            run(&mut e, 0.2, &c);
            e.exhaust_reversion_kg() - before
        };
        for model in [ExhaustModel::Waveguide, ExhaustModel::Fv] {
            let (full, half) = (reverted(1.0, model), reverted(0.5, model));
            assert!(full > 0.0, "{model:?}: no reversion measured");
            assert!(half < 0.8 * full, "{model:?}: {half} vs {full}");
        }
    }

    /// Output of an inline-4 held at `rpm` with only the given texture levels (jitter and reverb
    /// off), 0.2 s after settling.
    fn textured(flow: f64, intake: f64, mech: f64, rpm: f64, throttle: f64) -> Vec<f64> {
        let mut d = EngineDef::inline4();
        d.sound = crate::sound::SoundDef {
            flow_noise: flow,
            intake,
            mechanical: mech,
            combustion_jitter: 0.0,
            reverb: 0.0,
            leveling: 0.0,
            ..crate::sound::SoundDef::default()
        };
        let mut e = Engine::new(d, rpm, 1);
        let c = Controls {
            throttle,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        // the layers' own signal, before radiation turns it into pressure at the listener
        (0..(0.2 / DT) as usize)
            .map(|_| {
                e.step(DT, &c);
                e.texture_signal
            })
            .collect()
    }

    /// The LFA held at 6000 rpm, full throttle, with `snd`: (audio, texture signal) per step.
    fn heard_with(snd: crate::sound::SoundDef) -> (Vec<f64>, Vec<f64>) {
        let mut d = EngineDef::lfa_v10();
        d.sound = snd;
        let mut e = Engine::new(d, 6000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(6000.0),
            ..Controls::default()
        };
        run(&mut e, 0.2, &c);
        (0..(0.1 / DT) as usize)
            .map(|_| (e.step(DT, &c), e.texture_signal))
            .unzip()
    }

    #[test]
    fn the_jets_add_to_the_sound_in_proportion_to_their_level() {
        // flow_noise only scales the jets' pressure at the listener (the gas does not hear it):
        // with the same seed, the sound at level 2 minus the sound at 0 is exactly twice the
        // sound at level 1 minus it, and not nothing
        let at = |k: f64| {
            heard_with(crate::sound::SoundDef {
                flow_noise: k,
                ..crate::sound::SoundDef::default()
            })
            .0
        };
        let (a0, a1, a2) = (at(0.0), at(1.0), at(2.0));
        let (_, one) = mean_rms_of_difference(&a1, &a0);
        let (_, two) = mean_rms_of_difference(&a2, &a0);
        assert!(one > 1e-6, "no jets: {one}");
        assert_relative_eq!(two, 2.0 * one, max_relative = 1e-9);
    }

    #[test]
    fn the_texture_radiates_like_a_monopole_from_the_tails() {
        // The intake layer's share of the sound (level 1 minus level 0) is the shared
        // radiation of its share of the texture signal: a monopole, p ∝ dQ/dt with a positive
        // constant, so it follows the signal's rate of change with the same polarity.
        let quiet = crate::sound::SoundDef {
            flow_noise: 0.0,
            intake: 0.0,
            ..crate::sound::SoundDef::default()
        };
        let (a0, t0) = heard_with(quiet.clone());
        let (a1, t1) = heard_with(crate::sound::SoundDef {
            intake: 1.0,
            ..quiet
        });
        let heard: Vec<f64> = a1.iter().zip(&a0).map(|(x, y)| x - y).collect();
        let rate: Vec<f64> = t1
            .iter()
            .zip(&t0)
            .map(|(x, y)| x - y)
            .collect::<Vec<_>>()
            .windows(2)
            .map(|w| w[1] - w[0])
            .collect();
        // the listener's delay shifts it; take the best lag within 30 ms
        let corr = |lag: usize| {
            let (x, y) = (&heard[lag + 1..], &rate[..rate.len() - lag]);
            let n = x.len().min(y.len());
            let dot: f64 = (0..n).map(|i| x[i] * y[i]).sum();
            let nx: f64 = x[..n].iter().map(|v| v * v).sum::<f64>().sqrt();
            let ny: f64 = y[..n].iter().map(|v| v * v).sum::<f64>().sqrt();
            dot / (nx * ny)
        };
        let best = (0..(0.03 / DT) as usize).map(corr).fold(f64::MIN, f64::max);
        assert!(best > 0.9, "best correlation {best}");
    }

    fn mean_rms_of_difference(a: &[f64], b: &[f64]) -> (f64, f64) {
        let d: Vec<f64> = a.iter().zip(b).map(|(x, y)| x - y).collect();
        let m = d.iter().sum::<f64>() / d.len() as f64;
        (
            m,
            (d.iter().map(|v| (v - m).powi(2)).sum::<f64>() / d.len() as f64).sqrt(),
        )
    }

    #[test]
    fn each_texture_source_adds_to_the_output() {
        let dry = textured(0.0, 0.0, 0.0, 5000.0, 1.0);
        // the tail jets: zero-mean broadband pressure at the listener. By hand, a 2.0 L at
        // 5000 rpm WOT blows ~0.09 kg/s through a 50 mm tail (u ≈ 81 m/s mean): ≈ 61 dB at 3 m
        // (0.02 Pa), more with the pulse peaks weighing as u^7.5; measured ≈ 0.04 Pa
        let heard = |flow: f64| {
            let mut d = EngineDef::inline4();
            d.sound.flow_noise = flow;
            d.sound.intake = 0.0;
            d.sound.mechanical = 0.0;
            d.sound.combustion_jitter = 0.0;
            let mut e = Engine::new(d, 5000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(5000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            run(&mut e, 0.2, &c)
        };
        let (m, r) = mean_rms_of_difference(&heard(1.0), &heard(0.0));
        assert!(
            (0.01..0.2).contains(&r) && m.abs() < 0.05 * r,
            "jets: mean {m} rms {r} Pa"
        );
        // intake: the tone c·ṁ/A_bore is a positive offset under load
        let (m, _) = mean_rms_of_difference(&textured(0.0, 1.0, 0.0, 5000.0, 1.0), &dry);
        assert!(m > 100.0, "intake tone mean {m}");
        // valvetrain: clicks, more of them and louder at higher rpm
        let (_, r5) = mean_rms_of_difference(&textured(0.0, 0.0, 1.0, 5000.0, 1.0), &dry);
        let dry2 = textured(0.0, 0.0, 0.0, 2000.0, 1.0);
        let (_, r2) = mean_rms_of_difference(&textured(0.0, 0.0, 1.0, 2000.0, 1.0), &dry2);
        assert!(
            r5 > 1.5 * r2 && r2 > 0.0,
            "clicks 2000 rpm {r2} vs 5000 rpm {r5}"
        );
    }

    #[test]
    fn intake_tone_follows_the_throttle_flow() {
        // c·ṁ/A_bore: twice the flow, twice the offset. WOT vs a light throttle at the same rpm.
        let off = |throttle: f64| {
            let dry = textured(0.0, 0.0, 0.0, 4000.0, throttle);
            mean_rms_of_difference(&textured(0.0, 1.0, 0.0, 4000.0, throttle), &dry).0
        };
        let (light, wot) = (off(0.1), off(1.0));
        assert!(
            wot > 2.0 * light && light > 0.0,
            "light {light} vs WOT {wot}"
        );
    }

    #[test]
    fn burns_complete_with_jitter_on() {
        // The burn must run to the end (Wiebe ≥ 99 %) every cycle with jitter enabled: peak
        // pressure with and without jitter within 15 % (a burn dropped early would collapse it).
        let peak = |jitter: f64| {
            let mut d = EngineDef::single();
            d.sound.combustion_jitter = jitter;
            let mut e = Engine::new(d, 3000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(3000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let mut p: f64 = 0.0;
            for _ in 0..(0.2 / DT) as usize {
                e.step(DT, &c);
                p = p.max(e.cylinder(0).pressure_pa);
            }
            p
        };
        let (a, b) = (peak(0.0), peak(1.0));
        assert!((b / a - 1.0).abs() < 0.15, "peak without {a} with {b}");
    }

    fn fv_intake(mut d: EngineDef, individual: bool, runner_m: f64) -> EngineDef {
        d.intake.model = crate::intake_fv::IntakeModel::Fv;
        d.intake.individual_throttles = individual;
        d.intake.runner_length_m = runner_m;
        d
    }

    fn wot_torque(d: EngineDef, rpm: f64) -> f64 {
        let mut e = Engine::new(d, rpm, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        run(&mut e, 0.8, &c);
        e.torque_nm()
    }

    #[test]
    fn fv_intake_makes_comparable_torque_and_is_used() {
        let d = fv_intake(EngineDef::inline4(), false, 0.3);
        let mut e = Engine::new(d.clone(), 3000.0, 1);
        run(&mut e, 0.01, &Controls::default());
        assert!(e.intake_is_fv());
        let (zero_d, one_d) = (
            wot_torque(EngineDef::inline4(), 4000.0),
            wot_torque(d, 4000.0),
        );
        assert!(
            (one_d / zero_d - 1.0).abs() < 0.3,
            "0D {zero_d} vs 1D {one_d} N·m"
        );
    }

    #[test]
    fn longer_runners_favour_low_rpm() {
        // Ram tuning: a longer runner's pressure wave returns later, so it helps at lower rpm
        // (Heywood ch. 7; Winterbone & Pearson). Compare the 3000/6500 rpm torque ratio.
        let ratio = |runner: f64| {
            let d = fv_intake(EngineDef::inline4(), true, runner);
            wot_torque(d.clone(), 3000.0) / wot_torque(d, 6500.0)
        };
        let (short, long) = (ratio(0.15), ratio(0.6));
        assert!(long > short * 1.05, "short {short:.3} vs long {long:.3}");
    }

    #[test]
    fn itb_engine_idles_with_a_realistic_manifold() {
        let mut e = Engine::new(fv_intake(EngineDef::lfa_v10(), true, 0.25), 1000.0, 1);
        let xs = run(&mut e, 2.0, &Controls::default());
        assert!(xs.iter().all(|x| x.is_finite()));
        assert!((500.0..2500.0).contains(&e.rpm()), "idle {}", e.rpm());
        let map = e.plenum_pressure_pa() / 1e5;
        assert!((0.1..0.8).contains(&map), "idle MAP {map:.3} bar");
    }

    #[test]
    fn itb_volumetric_efficiency_stays_physical() {
        // Regression: a 60 mm throttle feeding a 42 mm runner through the orifice law pushed twice
        // the runner's choked flux: VE 1.5 at 2000 rpm. An NA engine stays ≲ 1.1.
        for rpm in [2000.0, 4000.0] {
            let d = fv_intake(EngineDef::inline4(), true, 0.3);
            let mut e = Engine::new(d, rpm, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            run(&mut e, 0.6, &c);
            let m0 = e.inducted_kg();
            run(&mut e, 0.2, &c);
            let ideal = 1.18 * e.def().displacement_m3() * rpm / 120.0 * 0.2;
            let ve = (e.inducted_kg() - m0) / ideal;
            assert!((0.6..1.15).contains(&ve), "{rpm} rpm: VE {ve:.2}");
        }
    }
}
