//! Engine definition (data) and the running engine (0D crank-angle-resolved model).
//!
//! Per cylinder: plenum → intake valve → cylinder → exhaust valve → exhaust waveguides →
//! atmosphere. Mechanics: analytic crank-slider with a lumped flywheel inertia.
//! Model ladder and references: `docs/research/simulador-motor-rust.md`.

use crate::combustion::{AFR_STOICH, LHV_GASOLINE, flame_angle_scale, laminar_flame_speed, wiebe};
use crate::exhaust::{
    ArrivalProbe, BlockIn, BlockOut, ExhaustDef, ExhaustLink, ExhaustModel, ExhaustNet,
};
use crate::exhaust_fv::ExhaustFv;
use crate::gas::{Flow, GasVolume, Reservoir, exchange_with_reservoir, plan_exchange};
use crate::geometry::CylinderGeometry;
use crate::intake_fv::{IntakeDef, IntakeFv, IntakeModel};
use crate::jet_noise::JetNoise;
use crate::layout::Layout;
use crate::listener::Listener;
use crate::sound::SoundDef;
use crate::structure::StructureResponse;
use crate::thermo::Composition;
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
    /// Honda VTEC: when the valves' high lobes engage ([`ValveTiming::high`]); none: fixed lobes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vtec: Option<crate::valve::Vtec>,
    /// Cam phasers (Toyota VVT-i, Honda VTC) on the intake and exhaust camshafts; none: fixed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intake_phaser: Option<crate::valve::Phaser>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exhaust_phaser: Option<crate::valve::Phaser>,
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
    /// How the engine is built, for its friction by components (Patton–Nitschke–Heywood as
    /// improved by Sandoval & Heywood): valvetrain, followers, valves, crankpins, rings, oil.
    pub friction: crate::friction::FrictionDef,
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

/// Peak force of a valve striking its seat at `SEAT_FORCE_RPM`, N: an exhaust valve of a
/// 4-cylinder petrol engine at 2500 rpm full load, 292 N in the head-coupled model validated
/// against the running engine (Zheng et al., Appl. Sci. 10(15) 5145, 2020, Fig. 22; 412 N
/// with a rigid head).
const SEAT_FORCE_N: f64 = 292.0;
const SEAT_FORCE_RPM: f64 = 2500.0;
/// Duration of the strike, s: the impulse m·Δv of the valve's equivalent mass (292 N /
/// 4474 m/s², Zheng Fig. 21: 0.065 kg) against a closing-ramp speed of ~0.3 m/s at 2500 rpm
/// (an estimate: ramps of ~0.02 mm/° at 15 000 °/s), as a half-sine of the peak:
/// τ = πmΔv/2F ≈ 0.1 ms; taken as 0.15 ms.
pub(crate) const SEAT_PULSE_S: f64 = 1.5e-4;

/// The force of a seat strike of `peak` N, `t` s after it began: a half-sine over
/// [`SEAT_PULSE_S`].
pub(crate) fn seat_pulse(peak: f64, t: f64) -> f64 {
    if (0.0..=SEAT_PULSE_S).contains(&t) {
        peak * (std::f64::consts::PI * t / SEAT_PULSE_S).sin()
    } else {
        0.0
    }
}

impl EngineDef {
    /// The engine as the friction model sees it. The definition lumps each cylinder's valves
    /// into one per side with the pair's curtain area (lift × √n); one real valve lifts L/√n.
    pub fn friction_engine(&self) -> crate::friction::FrictionEngine {
        let per_side = (f64::from(self.friction.valves_per_cylinder) / 2.0).max(1.0);
        crate::friction::FrictionEngine {
            bore_m: self.geometry.bore_m,
            stroke_m: self.geometry.stroke_m,
            cylinders: self.cylinders,
            vee: self.layout.bank_angle_deg != 0.0,
            compression_ratio: self.geometry.compression_ratio,
            valve_lift_m: self
                .intake_valve
                .max_lift_m
                .max(self.exhaust_valve.max_lift_m)
                / per_side.sqrt(),
        }
    }

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
                high: None,
            },
            exhaust_valve: ValveTiming {
                open_deg: 135.0,
                duration_deg: 245.0,
                max_lift_m: 0.008,
                diameter_m: 0.029,
                high: None,
            },
            vtec: None,
            intake_phaser: None,
            exhaust_phaser: None,
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
                muffler_transmission: 1.0,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.003,
                muffler_packing: crate::lining::Packing::default(),
                // one tail: no spread
                tail_spacing_m: 0.9,
                after_collector: Vec::new(),
            },
            intake: IntakeDef::default(),
            redline_rpm: 7500.0,
            starter_torque_nm: 25.0,
            // the usual DOHC with bucket tappets and four valves (no particular engine)
            friction: crate::friction::FrictionDef::default(),
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
        .with_tail_pipe()
        .with_catalyst()
    }

    /// Honda K20A (Integra Type R, DC5): the published engine, its unpublished geometry sized
    /// by physics like the LFA's.
    pub fn honda_k20a() -> Self {
        Self {
            // Honda K20A (Integra Type R, DC5): 1998 cc, 86 × 86 mm, CR 11.5, 220 PS at 8000 rpm,
            // 206 N·m at 7000 rpm, 8400 rpm redline, firing 1-3-4-2 (Honda K engine,
            // Wikipedia; JDM spec sheets). Rod 139 mm and valves 35/30 mm (two each) as
            // commonly quoted (not a factory document); the Mach-index sizing at 8000 rpm gives
            // 34.3 mm — the method agrees with the engine.
            name: "Honda K20A 2.0 (inline-4)".into(),
            geometry: CylinderGeometry {
                bore_m: 0.086,
                stroke_m: 0.086,
                rod_m: 0.139,
                compression_ratio: 11.5,
            },
            cylinders: 4,
            firing_order: vec![1, 3, 4, 2],
            layout: Layout::default(),
            // two 35 mm intake and two 30 mm exhaust valves as one equivalent each (√2·D);
            // lift a quarter of each valve's diameter, kept as the pair's curtain area. The
            // intake's VTEC lobe opens 36° longer and lifts 12.65/8.84 as far (K20A: low 244°,
            // 8.84 mm; VTEC 280°, 12.65 mm — motorreviewer, a secondary source), opening 18°
            // earlier. The real engine switches its exhaust lobes too; here they lost torque at
            // every speed (with friction by components and real gas properties, ≈ 20 N·m less at
            // 6000 rpm), so until a primary source gives their timing the exhaust keeps one lobe
            // (spec 2026-10-26).
            intake_valve: ValveTiming {
                open_deg: 340.0,
                duration_deg: 265.0,
                max_lift_m: 0.0124,
                diameter_m: 0.0495,
                high: Some(crate::valve::CamLobe {
                    open_deg: 322.0,
                    duration_deg: 301.0,
                    max_lift_m: 0.017_74,
                }),
            },
            exhaust_valve: ValveTiming {
                open_deg: 130.0,
                duration_deg: 260.0,
                max_lift_m: 0.0106,
                diameter_m: 0.0424,
                high: None,
            },
            flywheel_inertia_kgm2: 0.15,
            plenum_volume_m3: 4.0e-3,
            throttle_diameter_m: 0.062,
            idle_throttle: 0.0055,
            // a typical car idle (estimate)
            idle_rpm: 800.0,
            // primaries timed to the blowdown's second return at the 8000 rpm power peak
            // (Blair, ch. 6; c ≈ 600 m/s): 0.734 m; one 4-into-1 collector, its own tail pipe
            exhaust: ExhaustDef {
                primary_length_m: 0.734,
                ..ExhaustDef::default()
            },
            // runners Helmholtz-tuned to the 7000 rpm torque peak (Engelman; 42 mm): 0.255 m,
            // stepped as 1D pipes from a plenum behind the single throttle (as on the K20A)
            intake: IntakeDef {
                model: IntakeModel::Fv,
                runner_length_m: 0.255,
                ..IntakeDef::default()
            },
            redline_rpm: 8400.0,
            starter_torque_nm: 60.0,
            vehicle: crate::drivetrain::VehicleDef::inline4(),
            // engages at 6000 rpm (Integra Type R DC5; Wikipedia, Honda K engine); its
            // hysteresis and the pedal it waits for are estimates (no factory figure found)
            vtec: Some(crate::valve::Vtec {
                engage_rpm: 6000.0,
                hysteresis_rpm: 300.0,
                min_throttle: 0.3,
            }),
            // VTC on the intake camshaft (Honda R&D Technical Review 12-2, 2000, Niizato &
            // Hayashi): VTEC by speed, VTC by load. Its range spans intake openings over ~60°
            // (Fig. 8): 50° here; the 30 °/s rate is a control limit (US 7,467,611). At full load
            // "VTC establishes the valve timing which will optimize the torque in response to
            // engine speed" (§6.1, p. 51): the full-load advances are this model's own torque optimum,
            // swept in 10° steps on the dyno (low lobe below 6000 rpm, VTEC lobe above). At light
            // load the cam advances for internal EGR (Fig. 4); those degrees are estimates.
            intake_phaser: Some(crate::valve::Phaser {
                range_deg: 50.0,
                rate_deg_s: 30.0,
                map: vec![
                    crate::valve::PhaserPoint {
                        rpm: 1000.0,
                        full_load_deg: 10.0,
                        no_load_deg: 0.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 1500.0,
                        full_load_deg: 35.0,
                        no_load_deg: 9.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 2000.0,
                        full_load_deg: 20.0,
                        no_load_deg: 18.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 3000.0,
                        full_load_deg: 35.0,
                        no_load_deg: 35.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 3500.0,
                        full_load_deg: 20.0,
                        no_load_deg: 33.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 4000.0,
                        full_load_deg: 10.0,
                        no_load_deg: 31.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 5000.0,
                        full_load_deg: 5.0,
                        no_load_deg: 28.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 5500.0,
                        full_load_deg: 5.0,
                        no_load_deg: 26.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 6000.0,
                        full_load_deg: 20.0,
                        no_load_deg: 24.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 6500.0,
                        full_load_deg: 10.0,
                        no_load_deg: 23.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 7500.0,
                        full_load_deg: 10.0,
                        no_load_deg: 20.0,
                    },
                    crate::valve::PhaserPoint {
                        rpm: 8000.0,
                        full_load_deg: 0.0,
                        no_load_deg: 20.0,
                    },
                ],
            }),
            // roller rocker arms on both camshafts (Niizato & Hayashi, Honda R&D Technical Review
            // 12-2, 2000, Fig. 13), four valves
            friction: crate::friction::FrictionDef {
                valvetrain: crate::friction::Valvetrain::DohcFingerFollower,
                roller_followers: true,
                ..crate::friction::FrictionDef::default()
            },
            ..Self::single()
        }
        .with_tail_pipe()
        .with_catalyst()
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
            // 72° V10 (Toyota LR); cylinders 1–5 on one bank and 6–10 on the other, the
            // numbering the firing order 1-6-5-10-2-7-3-8-4-9 belongs to (it alternates the banks:
            // each one fires every 144°); banks as the collectors
            layout: Layout {
                bank_angle_deg: 72.0,
                banks: vec![vec![1, 2, 3, 4, 5], vec![6, 7, 8, 9, 10]],
            },
            // Valve sizes are not published: sized as the engine's designers would, from its
            // published rated speed. Intake: Heywood's Mach index Z = (B/D)²·S̄p/(Cᵢ·a) at its
            // knee, Z ≈ 0.6 (ICE Fundamentals §6.3.2; mean flow coefficient Cᵢ ≈ 0.35) at
            // 8700 rpm (S̄p = 22.91 m/s): one equivalent valve of 49.6 mm, two of 35.1 mm.
            // Exhaust ≈ 0.83 of the intake (Heywood §6.3: exhaust valves ~70 % of the intake
            // area): 41.2 mm. Lift: a quarter of each valve's diameter, where the curtain area
            // reaches the port's (8.8 mm per intake valve; the equivalent valve's lift keeps the
            // two valves' curtain area). Timing: Heywood's typical high-output values
            // (IVO 20° BTDC, IVC 70° ABDC, EVO 55° BBDC, EVC 30° ATDC).
            intake_valve: ValveTiming {
                open_deg: 340.0,
                duration_deg: 270.0,
                max_lift_m: 0.0124,
                diameter_m: 0.0496,
                high: None,
            },
            exhaust_valve: ValveTiming {
                open_deg: 125.0,
                duration_deg: 265.0,
                max_lift_m: 0.0103,
                diameter_m: 0.0412,
                high: None,
            },
            vtec: None,
            intake_phaser: None,
            exhaust_phaser: None,
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
            // Equal-length primaries, one 5-into-1 collector per bank and an equal-length dual
            // system (Toyota, LFA press release, 2009-10-21; ja.wikipedia "トヨタ・LRエンジン").
            // Primary length not published: the second return of the blowdown's reflection
            // reaches the port at the overlap at the 8700 rpm power peak — 2·(2L)/c = (360° −
            // EVO)/(6N) with c ≈ 600 m/s in ~900 K gas (Blair, Design and Simulation of
            // Four-Stroke Engines, ch. 6): L = 0.675 m. Bores estimated.
            exhaust: ExhaustDef {
                primary_length_m: 0.675,
                primary_diameter_m: 0.042,
                collectors: vec![vec![1, 2, 3, 4, 5], vec![6, 7, 8, 9, 10]],
                collector_length_m: 0.3,
                collector_diameter_m: 0.065,
                tail_length_m: 1.2,
                tail_diameter_m: 0.060,
                // Sport muffler with bypass valves: less damping than a road car.
                muffler_transmission: 1.0,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.010,
                muffler_packing: crate::lining::Packing::default(),
                // the tips sit together in the centre of the rear diffuser
                tail_spacing_m: 0.25,
                after_collector: Vec::new(),
            },
            // Ten ITBs (Toyota). Runner length not published: Helmholtz-tuned to the torque peak
            // (Engelman: f = (a/2π)·√(A/(L·V)), V = V_d/2·(r + 1)/(r − 1), tuned at twice the
            // revolution frequency at 6800 rpm, 226.7 Hz; 45 mm bore): L = 0.325 m.
            intake: IntakeDef {
                model: IntakeModel::Fv,
                runner_length_m: 0.325,
                runner_diameter_m: 0.045,
                individual_throttles: true,
                ..IntakeDef::default()
            },
            redline_rpm: 9000.0,
            starter_torque_nm: 150.0,
            // DOHC, four titanium valves a cylinder; Toyota has not published the valve drive, so
            // the usual bucket tappets
            friction: crate::friction::FrictionDef::default(),
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1500.0,
            exhaust_reverse_cd_ratio: 0.75,
            vehicle: crate::drivetrain::VehicleDef::lfa_v10(),
        }
        .with_tail_pipe()
        .with_catalyst()
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
                high: None,
            },
            exhaust_valve: ValveTiming {
                open_deg: 125.0,
                duration_deg: 265.0,
                max_lift_m: 0.0105,
                diameter_m: 0.043,
                high: None,
            },
            vtec: None,
            intake_phaser: None,
            exhaust_phaser: None,
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
                muffler_transmission: 1.0,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.010,
                muffler_packing: crate::lining::Packing::default(),
                // the tips sit together in the centre (458 Italia)
                tail_spacing_m: 0.25,
                after_collector: Vec::new(),
            },
            intake: IntakeDef::default(),
            redline_rpm: 9000.0,
            starter_torque_nm: 150.0,
            // DOHC, four valves a cylinder; valve drive not published (bucket tappets assumed)
            friction: crate::friction::FrictionDef::default(),
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1500.0,
            exhaust_reverse_cd_ratio: 0.75,
            vehicle: crate::drivetrain::VehicleDef::flatplane_v8(),
        }
        .with_tail_pipe()
        .with_catalyst()
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
                high: None,
            },
            exhaust_valve: ValveTiming {
                open_deg: 130.0,
                duration_deg: 250.0,
                max_lift_m: 0.0138,
                diameter_m: 0.0404,
                high: None,
            },
            vtec: None,
            intake_phaser: None,
            exhaust_phaser: None,
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
                muffler_transmission: 1.0,
                model: ExhaustModel::Waveguide,
                muffler_volume_m3: 0.012,
                muffler_packing: crate::lining::Packing::default(),
                // two tips each side (C6 Corvette)
                tail_spacing_m: 1.1,
                after_collector: Vec::new(),
            },
            intake: IntakeDef::default(),
            redline_rpm: 6600.0,
            starter_torque_nm: 200.0,
            // pushrods, hydraulic roller lifters, two valves (GM LS3)
            friction: crate::friction::FrictionDef {
                valvetrain: crate::friction::Valvetrain::Ohv,
                roller_followers: true,
                valves_per_cylinder: 2,
                ..crate::friction::FrictionDef::default()
            },
            sound: SoundDef::default(),
            overrun_fuel_cut_rpm: 1300.0,
            exhaust_reverse_cd_ratio: 0.75,
            vehicle: crate::drivetrain::VehicleDef::crossplane_v8(),
        }
        .with_tail_pipe()
        .with_catalyst()
    }

    /// The ECU's spark advance (° BTDC) for a flame `flame_ratio` times as long in crank angle as
    /// at the reference state ([`flame_ratio`]): MBT timing puts the half-burned point a fixed
    /// [`CA50_ATDC_DEG`] after top centre (Heywood, *ICE Fundamentals*, §9.2.2), and spark to
    /// half-burned (delay + Wiebe 50 %) scales with the flame, so the advance is
    /// (`spark_advance_deg` + CA50)·ratio − CA50, `spark_advance_deg` being the reference's;
    /// within ±[`MAX_SPARK_ADVANCE_DEG`], as far as an ignition map reaches.
    pub fn spark_advance_for(&self, flame_ratio: f64) -> f64 {
        ((self.spark_advance_deg + CA50_ATDC_DEG) * flame_ratio - CA50_ATDC_DEG)
            .clamp(-MAX_SPARK_ADVANCE_DEG, MAX_SPARK_ADVANCE_DEG)
    }

    /// This engine with the exhaust its preset has data for after each collector: one pipe, the
    /// tail's own length and bore. The road car's silencers of [`crate::exhaust::road_exhaust`]
    /// are sized by a rule of thumb, not from a drawing: no preset carries them until one does
    /// (they reshaped the whole spectrum — the V10's 500–1000 Hz band 23 dB over 250–500 Hz).
    fn with_tail_pipe(mut self) -> Self {
        self.exhaust.after_collector = vec![crate::exhaust::Segment::Pipe {
            length_m: self.exhaust.tail_length_m,
            diameter_m: self.exhaust.tail_diameter_m,
        }];
        self
    }

    /// A road car's catalyst first after each collector
    /// ([`crate::exhaust::close_coupled_catalyst`]): every one since the 1990s carries them,
    /// and they shape the sound that reaches the tail (Elnemr, KTH 2011).
    fn with_catalyst(mut self) -> Self {
        let branches = self.exhaust.collectors.len().max(1);
        let cat = crate::exhaust::close_coupled_catalyst(
            self.displacement_m3() / branches as f64,
            self.exhaust.tail_diameter_m,
        );
        self.exhaust.after_collector.insert(0, cat);
        self
    }

    /// `n` cylinders, the rest rebuilt so it stays one engine: an even firing order; a V keeps
    /// two banks, the first half on one and the second on the other (the numbering the
    /// default orders alternate between), at the even-firing angle 720°/n (two pistons on each
    /// crankpin), each bank on its own collector if they had one each; an inline engine keeps
    /// its single collector.
    pub fn set_cylinders(&mut self, n: usize) {
        self.cylinders = n;
        self.firing_order = crate::config::default_firing_order(n);
        if self.layout.banks.len() == 2 && n >= 2 {
            let half = n / 2;
            self.layout.banks = vec![(1..=half).collect(), (half + 1..=n).collect()];
            if n.is_multiple_of(2) {
                self.layout.bank_angle_deg = 720.0 / n as f64;
            }
            if self.exhaust.collectors.len() == 2 {
                self.exhaust.collectors = self.layout.banks.clone();
            }
        } else {
            self.exhaust.collectors.clear();
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
/// Half-burned point of an MBT-timed burn, ° after top centre (Heywood §9.2.2: 8–10°).
const CA50_ATDC_DEG: f64 = 8.0;
/// Furthest an ignition map advances or retards the spark, ° (production maps stay within
/// about 50° BTDC; Bosch, *Gasoline Engine Management*, "ignition map").
const MAX_SPARK_ADVANCE_DEG: f64 = 50.0;
/// Combustion efficiency of a spark-ignition engine at stoichiometric: 95–98 % of the fuel's
/// heating value is released (Heywood, *ICE Fundamentals*, §3.5.5, Fig. 3-9). It was a lumped
/// 0.85 while the residual gas counted as fresh charge; burning only the fresh charge, the
/// LFA makes 494 N·m against the published 480.
const COMBUSTION_EFFICIENCY: f64 = 0.96;
/// Cycle-to-cycle variation of released energy (±), drives the "lumpy" idle.
/// σ of the equivalence ratio near the plug each cycle, relative: 2–6 % (Heywood Fig. 9-34a).
const PHI_DISPERSION: f64 = 0.04;
/// σ of the residual fraction near the plug each cycle, relative: the CO₂ scatter of Heywood
/// Fig. 9-34b (≈ ±0.35 in ≈ 4 % CO₂).
const RESIDUAL_DISPERSION: f64 = 0.09;
/// σ of the turbulence intensity at the plug each cycle, relative. During compression the
/// cycle-by-cycle variation of the mean flow at a point is comparable in magnitude to the
/// turbulence intensity itself (Heywood §8.2, Fig. 8-9), and it carries the early flame kernel
/// (Ozdor, Dulger & Sher 1994: the kernel's development dominates cyclic variability). Its size is
/// set where the burn angles' σ comes out at Heywood's measured share of their mean, about 0.12 for
/// both the flame-development and the rapid-burning angle over many chambers and conditions
/// (Fig. 9-35), with the A/F and residual draws above.
const TURBULENCE_DISPERSION: f64 = 0.175;
/// σ of how much each cylinder breathes apart from the others, fixed: "typically a few
/// percent" (Heywood §9.4.2, §7.6).
const CYLINDER_IMBALANCE: f64 = 0.02;
/// σ of the charge a cylinder burns each cycle, relative: the "cyclic cylinder fuel and air
/// charging variations" of Heywood §9.4.1–9.4.2. Soltau (*Proc. IMechE (AD)* 1960–61 no. 2,
/// Table 4: wide-open throttle, 2000 rev/min, A/F 14, ten consecutive cycles) measured the
/// i.m.e.p. varying by a mean deviation of 1.18–1.53 %, σ = 1.2533 × that = 1.48–1.92 % for a
/// normal spread. Near MBT the burn's timing moves the work little (the model: 0.3 %), so the
/// charge carries the rest, √(1.7² − 0.3²) ≈ 1.7 %.
const CHARGE_DISPERSION: f64 = 0.017;
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
/// Density of the air the intake draws, kg/m³ (1 atm, 300 K).
const AIR_DENSITY: f64 = crate::units::ATM_PA / (crate::gas::R_AIR * 300.0);
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
/// Time constant of unburnt mixture burning in a hot primary, s: a deflagration of a few
/// milliseconds. Estimate (no measurement at hand; see PAPERS.md, afterfire).
const AFTERBURN_TAU_S: f64 = 2e-3;
/// Fraction of the equalising mass a valve may move per substep (see `step`).
const VALVE_RELAX: f64 = 0.5;

#[derive(Debug, Clone)]
struct Burn {
    start_deg: f64,
    duration_deg: f64,
    energy_j: f64,
    released: f64,
    /// The charge the flame burns through, fuel and air, kg: what it has not released when it
    /// ends leaves unburnt.
    fuel_kg: f64,
    air_kg: f64,
}

impl Burn {
    /// What the flame leaves unburnt if it ends now.
    fn unburnt(&self) -> crate::fuel::ExhaustMixture {
        let (fuel_kg, air_kg) = crate::fuel::unburnt(self.fuel_kg, self.air_kg, self.released);
        crate::fuel::ExhaustMixture { fuel_kg, air_kg }
    }
}

/// Stagnation temperature of the gas leaving the tail, K: the waveguide's tail sound speed,
/// T = c²/(γR).
const TAIL_T_K: f64 =
    crate::exhaust::C_TAIL * crate::exhaust::C_TAIL / (crate::gas::GAMMA * crate::gas::R_AIR);
/// Speed of a sonic (choked) exit from gas at [`TAIL_T_K`]: c* = c·√(2/(γ+1)), m/s.
const TAIL_SONIC: f64 = crate::exhaust::C_TAIL * 0.912_870_929_175_276_9;

/// The way out after the collector, quasi-steady at the mean exhaust flow `m` (kg/s):
/// (backpressure, Pa; exit velocity, m/s). The open end is an orifice of the outlet's bore into
/// the atmosphere (the compressible orifice of [`crate::gas::orifice_mass_flow`], solved
/// backwards for the pressure that passes `m` — past the critical ratio it chokes, and the
/// pressure behind it climbs), plus the pipes' wall friction (Darcy) and the catalysts'
/// laminar resistance ([`crate::exhaust::ExitLosses`]). The jet leaves at the isentropic speed
/// of that expansion, at most sonic.
fn tail_exit(m: f64, losses: &crate::exhaust::ExitLosses) -> (f64, f64) {
    if m <= 0.0 {
        return (0.0, 0.0);
    }
    use crate::gas::{CHOKED_FLOW_FACTOR, CRITICAL_PRESSURE_RATIO, GAMMA, R_AIR};
    let atm = crate::units::ATM_PA;
    let a = losses.outlet_area;
    // The orifice equation (`orifice_mass_flow`) solved for p₀ in closed form. Choked:
    // ṁ = A·p₀/√(RT)·Γ. Subsonic, with x = (p_a/p₀)^((γ−1)/γ) and n = 2γ/(γ−1):
    // ṁ·√(RT)/(A·p_a) = K = √(n(1 − x))/x, so K²x² + n·x − n = 0 and x = 2n/(n + √(n² + 4nK²))
    // (the root in (0, 1], written without the cancellation at small K).
    let rt = (R_AIR * TAIL_T_K).sqrt();
    let choked = m * rt / (a * CHOKED_FLOW_FACTOR);
    // The jet leaves at √(2c_pT(1 − x)): sonic (c*) when choked.
    let (p0, u) = if atm / choked <= CRITICAL_PRESSURE_RATIO {
        (choked, TAIL_SONIC)
    } else {
        let n = 2.0 * GAMMA / (GAMMA - 1.0);
        let k = m * rt / (a * atm);
        let x = 2.0 * n / (n + (n * n + 4.0 * n * k * k).sqrt());
        (
            atm / x.powf(GAMMA / (GAMMA - 1.0)),
            (2.0 * crate::gas::CP * TAIL_T_K * (1.0 - x)).sqrt(),
        )
    };
    let rho = p0 / (crate::gas::R_AIR * TAIL_T_K);
    let friction = losses.friction * m * m / (2.0 * rho);
    let laminar = losses.laminar * m / rho;
    (p0 - atm + friction + laminar, u)
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
/// This cycle's flame ratio ([`flame_ratio`]) for the charge as the spark finds it (Heywood
/// §9.4.2): its equivalence ratio (σ [`PHI_DISPERSION`]) and residual (σ [`RESIDUAL_DISPERSION`])
/// near the plug and the turbulence there (σ [`TURBULENCE_DISPERSION`]), each set by a draw of
/// unit σ in `(phi, residual, turbulence)`. The turbulence moves the angles as u′^(−2/3) (Hires
/// eqs. 6–7: Δt ∝ (L/u′)^⅓ (λ/S_L)^⅔ with λ ∝ (Lν/u′)^½). None if the flame is not finite.
fn cycle_flame(
    g: &crate::geometry::CylinderGeometry,
    piston_speed: f64,
    (p, t): (f64, f64),
    rho: f64,
    v: f64,
    (phi_mean, dilution): (f64, f64),
    (n_phi, n_res, n_u): (f64, f64, f64),
) -> Option<f64> {
    let phi = phi_mean * (1.0 + PHI_DISPERSION * n_phi);
    let f_local = (dilution * (1.0 + RESIDUAL_DISPERSION * n_res)).max(0.0);
    let u_turb = (1.0 + TURBULENCE_DISPERSION * n_u).max(0.3);
    let local = flame_ratio(g, piston_speed, (p, t), rho, v, (phi, f_local));
    let ratio = local * u_turb.powf(-2.0 / 3.0);
    ratio.is_finite().then_some(ratio)
}

/// The burned share of a trapped charge: the residual less its unburnt fuel and air, over the
/// trapped mass.
fn dilution_of(cs: &CylinderState) -> f64 {
    if cs.trapped_kg > 0.0 {
        (cs.residual_kg - cs.residual_unburnt_kg).max(0.0) / cs.trapped_kg
    } else {
        0.0
    }
}

/// The flame's crank angles relative to the reference state at which a preset's
/// `burn_duration_deg` and [`IGNITION_DELAY_DEG`] hold: Hires et al. (SAE 780232) eqs. 13/32,
/// (S_p·ν)^⅓·(h/S_L)^⅔ ([`flame_angle_scale`]), for the charge at the spark — pressure and
/// temperature `(p, t)`, density `rho`, volume `v` and residual mass fraction `dilution`,
/// compressed isentropically to top centre — over the same at the reference: S_p = 10 m/s, unburned gas
/// at 700 K and 20 atm, 8 % residual (a typical full-load figure, Heywood Fig. 6-21) and the
/// engine's own TDC chamber height. Stoichiometric (φ = 1), as the model runs.
fn flame_ratio(
    g: &crate::geometry::CylinderGeometry,
    piston_speed: f64,
    (p, t): (f64, f64),
    rho: f64,
    v: f64,
    (phi, dilution): (f64, f64),
) -> f64 {
    const T_REF: f64 = 700.0;
    const P_REF: f64 = 20.0 * ATM_PA;
    let nu = |t: f64, rho: f64| crate::exhaust::gas_viscosity(t) / rho;
    let rho_ref = P_REF / (crate::gas::R_AIR * T_REF);
    let reference = flame_angle_scale(
        10.0,
        nu(T_REF, rho_ref),
        g.clearance_volume() / g.piston_area(),
        laminar_flame_speed(1.0, T_REF, P_REF, 0.08),
    );
    // the charge as the flame meets it: taken from the spark to top centre isentropically
    // (Hires evaluates near TDC), where the reference state is
    let r = (v / g.clearance_volume()).max(1.0);
    let gamma = crate::gas::GAMMA;
    let (p, t, rho) = (p * r.powf(gamma), t * r.powf(gamma - 1.0), rho * r);
    let s_l = laminar_flame_speed(phi, t, p, dilution);
    if s_l <= 0.0 {
        // past the fit's zero the flame does not propagate: a misfire
        return f64::INFINITY;
    }
    let here = flame_angle_scale(
        piston_speed.max(0.1),
        nu(t, rho),
        g.clearance_volume() / g.piston_area(),
        s_l,
    );
    here / reference
}

/// The share of the MBT advance an ECU's map gives at `rpm`: × (0.35 + 0.65·rpm/4000) up to 1.
/// Below full speed the spark runs retarded from MBT so the idle control has a torque reserve
/// to take from the spark at once (Bosch, *Gasoline Engine Management*, "idle-speed control");
/// it is the low-speed shape the spark map had before the flame set MBT.
fn idle_reserve(rpm: f64) -> f64 {
    (0.35 + 0.65 * rpm.max(0.0) / 4000.0).min(1.0)
}

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
    /// Gas left in the cylinder when the exhaust valve closed: burnt, no fuel or oxygen to
    /// give (the residual; Heywood, *ICE Fundamentals*, §6.4).
    residual_kg: f64,
    /// The part of the residual that is unburnt fuel and air (a cut or misfired cycle's), not
    /// burnt diluent, kg.
    residual_unburnt_kg: f64,
    /// The trapped charge's flame, crank-angle ratio to the reference ([`flame_ratio`]): sets
    /// its spark (the ECU's MBT timing) and its burn.
    flame: f64,
    /// How much more or less this cylinder breathes than the mean, fixed (see
    /// [`CYLINDER_IMBALANCE`]).
    charge_bias: f64,
    /// The charge trapped at intake closing still awaits its spark: one spark per charge,
    /// whatever the ECU does with the timing meanwhile.
    armed: bool,
    /// Gas temperature in this cylinder's exhaust primary, and the primary's wall temperature,
    /// K: what reversion pushes back, and how much volume it is.
    port_t: f64,
    /// The exhaust port's last mass flow, kg/s: where the next step's port solve starts.
    port_flow: f64,
    /// The exhaust valve boundary's last root, where the next step's solve starts.
    valve_hint: f64,
    wall_t: f64,
    /// State at intake closing (p_r, V_r, T_r): Woschni's reference for the motored pressure.
    ref_p: f64,
    ref_v: f64,
    ref_t: f64,
    burn: Option<Burn>,
    cycle_deg: f64,
    /// The liquid fuel on this cylinder's port walls ([`crate::fuel::PortFilm`]).
    film: crate::fuel::PortFilm,
    /// The charge trapped at intake closing that awaits its spark: fuel and air, kg.
    fuel_kg: f64,
    air_kg: f64,
    /// Unburnt fuel and air in the cylinder's gas once its flame is over (a partial burn, a
    /// misfire, a lean charge's spare air), leaving with the gas through the exhaust valve.
    slip: crate::fuel::ExhaustMixture,
    /// Unburnt mixture in this cylinder's exhaust primary.
    port_mix: crate::fuel::PortMixture,
    /// The gas's sensible internal energy, J ([`crate::thermo`]); `gas` is its view for the
    /// flow code: true pressure, and R_air·T' = R·T so its orifices see the true √(R·T).
    u_j: f64,
    /// What the gas is made of: burned share (residual, reversion, the flame's products) and
    /// how much of the rest carries fuel.
    comp: Composition,
    /// The gas's true temperature, K.
    t_k: f64,
    /// `comp` as one mixture, recomputed by [`settle`] when `comp` has changed (`mix_stale`).
    mix: crate::thermo::Mixture,
    mix_stale: bool,
    /// Share of the trapped charge that was fresh at intake closing: what a flame turns into
    /// burned gas.
    fresh_share: f64,
    /// Burned share of the gas in the exhaust primary (what a reversion brings back): set by
    /// what leaves the cylinder, like `port_t` (air on overrun, products under power).
    port_xb: f64,
}

/// How far the speed may move before the friction's speed terms are recomputed, rpm: at most
/// ≈ 0.02 kPa of fmep per rpm (the K20A at 8000 rpm), well under 0.1 % of it.
const FRICTION_RPM: f64 = 1.0;

/// Bring a cylinder's temperature and its `gas` view up to its true state (mass, internal
/// energy, burned share) through the real gas's properties.
fn settle(cs: &mut CylinderState, th: &crate::thermo::CylinderGas) {
    if cs.mix_stale {
        cs.mix = th.mixture(cs.comp);
        cs.mix_stale = false;
    }
    let m = cs.gas.mass_kg.max(1e-12);
    cs.t_k = cs.mix.temperature(cs.u_j / m, cs.t_k);
    // p = m·R·T/V; the view keeps it as p·V/(γ_air − 1)
    cs.gas.energy_j = m * cs.mix.r * cs.t_k / (crate::gas::GAMMA - 1.0);
}

/// `dm` kg leave the cylinder at its own enthalpy (its temperature does not jump).
fn cylinder_out(cs: &mut CylinderState, dm: f64) {
    cs.u_j -= dm * cs.mix.h(cs.t_k);
    cs.gas.mass_kg -= dm;
}

/// `dm` kg of gas `c_in` come in carrying `h` J/kg; the compositions mix by mass.
fn cylinder_in(cs: &mut CylinderState, dm: f64, h: f64, c_in: Composition) {
    let m = cs.gas.mass_kg.max(0.0);
    let (fresh, fresh_in) = (m * (1.0 - cs.comp.burned), dm * (1.0 - c_in.burned));
    cs.comp = Composition {
        burned: ((cs.comp.burned * m + c_in.burned * dm) / (m + dm)).clamp(0.0, 1.0),
        fuelled: if fresh + fresh_in > 0.0 {
            ((cs.comp.fuelled * fresh + c_in.fuelled * fresh_in) / (fresh + fresh_in))
                .clamp(0.0, 1.0)
        } else {
            cs.comp.fuelled
        },
    };
    cs.mix_stale = true;
    cs.gas.mass_kg += dm;
    cs.u_j += dm * h;
}

/// Apply a planned flow whose `dm` > 0 leaves the cylinder (and < 0 enters it, gas `c_in`). The flow code carries cold-air enthalpy, c_p,air·T; the gas enters at that
/// temperature with its own mixture's enthalpy, so a reversion neither cools nor heats the
/// cylinder by the model's convention alone.
fn cylinder_flow(
    cs: &mut CylinderState,
    th: &crate::thermo::CylinderGas,
    f: Flow,
    c_in: Composition,
) {
    if f.dm > 0.0 {
        cylinder_out(cs, f.dm);
    } else if f.dm < 0.0 {
        let t_in = f.h / crate::gas::CP;
        // the enthalpy of what comes in, mass-weighted over its parts
        let (a, u, b) = c_in.weights();
        let mut h = 0.0;
        for (m, w) in [(&th.air, a), (&th.unburned, u), (&th.burned, b)] {
            if w > 0.0 {
                h += w * m.h(t_in);
            }
        }
        cylinder_in(cs, -f.dm, h, c_in);
    }
}

/// How finely an engine is simulated (not part of its definition: the same engine at
/// another fidelity). The default is the reference.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fidelity {
    /// Scheme of every 1D pipe (intake runners, FV exhaust).
    pub pipe_scheme: crate::fv::Scheme,
    /// Length of the waveguide silencers' lined segments, m (finer is closer to the
    /// transmission line at high frequencies, and dearer).
    pub lining_segment_m: f64,
}

impl Default for Fidelity {
    fn default() -> Self {
        Self {
            pipe_scheme: crate::fv::Scheme::default(),
            lining_segment_m: crate::exhaust::LINING_SEGMENT_M,
        }
    }
}

/// The exhaust net's side of an [`Engine`] whose net runs elsewhere: the link to it, the
/// waves arriving at the valves (read from the copy of the primaries' lines), the block of
/// steps being collected for the net, and the steps whose sound waits for their tails.
///
/// The net runs a block of `block` steps behind the engine and the engine never reads what it
/// computes before two blocks later: a step's sound comes out `2·block` steps late, its waves
/// at the valves come from the lines' contents (see [`ArrivalProbe`]).
struct Remote {
    link: Box<dyn ExhaustLink>,
    probe: ArrivalProbe,
    arriving: Vec<f64>,
    /// The primaries' area, m².
    a_pipe: f64,
    /// Steps per block.
    block: usize,
    /// The inputs of the block being filled.
    stage: BlockIn,
    /// Blocks handed over and not yet collected (at most two).
    in_flight: usize,
    /// Steps taken since the net was detached, and those whose waves the probe has taken.
    taken: u64,
    applied: u64,
    /// Scratch for one block's results.
    out: BlockOut,
    /// The tails of the steps collected and not yet heard (per step, in order).
    tails: std::collections::VecDeque<f64>,
    /// Per tail: the signals of the step being finished.
    tails_now: Vec<f64>,
    /// The steps taken whose sound waits for their tails: (the intake mouth's and the block's
    /// sound, the cycle-mean constant, the tails' mean exit velocities).
    pending: std::collections::VecDeque<(f64, f64, Vec<f64>)>,
    /// Velocity buffers to reuse.
    spare: Vec<Vec<f64>>,
}

impl Remote {
    /// Send the staged block; with two in flight, take the older one back.
    fn hand_over(&mut self) {
        self.link.submit(&mut self.stage);
        self.in_flight += 1;
        if self.in_flight == 2 {
            self.take_oldest();
        }
    }

    /// Wait for the oldest block in flight: its waves go into the probe, its tails wait to be heard.
    fn take_oldest(&mut self) {
        self.link.collect(&mut self.out);
        self.in_flight -= 1;
        self.applied += self.out.steps as u64;
        self.probe.push(&self.out.from_far);
        self.tails.extend(self.out.tails.iter().copied());
    }
}

/// Holds a [`Remote`]. A clone of an engine does not carry it (the link is not cloneable): a
/// clone runs no exhaust until one is attached again.
struct RemoteSlot(Option<Box<Remote>>);

impl Clone for RemoteSlot {
    fn clone(&self) -> Self {
        RemoteSlot(None)
    }
}

impl std::fmt::Debug for RemoteSlot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() {
            "remote exhaust"
        } else {
            "-"
        })
    }
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
    /// Whether VTEC has the high lobes engaged ([`EngineDef::vtec`]).
    vtec_engaged: bool,
    /// The cam phasers' advance now, crank degrees (intake, exhaust).
    cam_advance: (f64, f64),
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
    /// The friction model at the last speed it was evaluated at ([`Self::friction_torque`]).
    friction_at: std::cell::Cell<crate::friction::AtSpeed>,
    /// The cylinder gas's properties (unburned charge and burned gas).
    thermo: crate::thermo::CylinderGas,
    /// Heat released by unburnt mixture burning in the exhaust primaries, J.
    afterburn_j: f64,
    /// Fuel ledger, kg: into the cylinders, burnt there, burnt in the primaries, and out of the
    /// primaries unburnt (the balance a test closes).
    fuel_ledger: FuelLedger,
    rng: u64,
    /// Exhaust waveguide network, built for the current step size.
    net: Option<ExhaustNet>,
    /// With the FV exhaust, the valves' jet noise rides a linear waveguide of the same pipes
    /// (a small perturbation, it superposes on the gas dynamics; the FV cells are too coarse to
    /// carry it), and what its tails radiate adds to the FV tails'. Its sources, per cylinder.
    jet_net: Option<ExhaustNet>,
    jet_flows: Vec<f64>,
    /// Non-linear finite-volume exhaust, when `exhaust.model` is `fv`.
    fv: Option<ExhaustFv>,
    /// 1D intake runners, when `intake.model` is `fv`.
    intake_net: Option<IntakeFv>,
    /// Tail pipes (one per collector group): each is its own jet for the flow noise.
    tail_pipes: usize,
    /// Where the listener hears the tails from, built for the step size it was made with.
    listener: Option<(f64, Listener)>,
    /// The block's structure response at the step it was built for ([`crate::structure`]).
    structure: Option<(f64, StructureResponse)>,
    /// Valve strikes still ringing: (time since the strike, s; its peak force, N).
    seat_strikes: Vec<(f64, f64)>,
    /// The texture layers' signal this step, in the tails' units, before it is radiated.
    texture_signal: f64,
    /// The turbulent jet at each tail pipe.
    jets: Vec<JetNoise>,
    /// Each exhaust valve's jet ([`crate::jet_noise::ValveJet`]).
    valve_jets: Vec<crate::jet_noise::ValveJet>,
    /// Per tail: the cylinders blowing into it, their exhaust mass flow averaged over a cycle
    /// (kg/s), the waveguide signal's own average, and the exit velocity (m/s) the jets see.
    tail_groups: Vec<Vec<usize>>,
    tail_mean_flow: Vec<f64>,
    tail_dc: Vec<f64>,
    tail_u: Vec<f64>,
    /// Each tail's mean exit velocity this step (mass continuity), m/s.
    tail_u_mean: Vec<f64>,
    /// The tails' signals this step, kept between steps (no allocation per step).
    tails_buf: Vec<f64>,
    /// The exhaust net when it runs elsewhere ([`Engine::detach_exhaust`]).
    remote: RemoteSlot,
    /// Mean backpressure each tail puts on its cylinders' exhaust ports, Pa.
    tail_backpressure: Vec<f64>,
    /// The exhaust's mean-flow losses after the collector (from its segments).
    exit_losses: Option<crate::exhaust::ExitLosses>,
    /// Mean exit Mach number of each tail (its open end absorbs sound with it).
    tail_mach: Vec<f64>,
    /// Each cylinder's exhaust flow averaged over a cycle, kg/s.
    cyl_mean_flow: Vec<f64>,
    /// Steps since the pipes' friction last followed the mean flows.
    friction_age: usize,
    flows: Vec<f64>,
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

/// Where an engine's fuel went, kg.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct FuelLedger {
    into_cylinders: f64,
    burnt: f64,
    afterburnt: f64,
    exited: f64,
}

/// What heat `q` (J) released over `dt` in a primary's `m` kg of gas does: it warms that gas by
/// q/(m·c_p), and it expands it at constant pressure, Q = (γ − 1)/γ·q̇/p — in the waveguide's
/// reference-gas units q̇/(c_p·T_ref), the weight an enthalpy flux of q̇ has in
/// [`exhaust_source_flow`]. Returns (temperature rise, K; source).
fn primary_heat(q: f64, m: f64, dt: f64) -> (f64, f64) {
    let t_ref = crate::exhaust::C_PRIMARY.powi(2) / (crate::gas::GAMMA * crate::gas::R_AIR);
    (q / (m * crate::gas::CP), q / dt / (crate::gas::CP * t_ref))
}

/// Heat a flame releases burning all it can of `fuel` and `air` kg, J: LHV × the combustion
/// efficiency × the cylinder's breathing `bias` ([`CYLINDER_IMBALANCE`]).
fn burn_energy(fuel: f64, air: f64, bias: f64) -> f64 {
    crate::fuel::burnable(fuel, air) * LHV_GASOLINE * COMBUSTION_EFFICIENCY * bias
}

/// Fraction of the plate's area a pedal position opens: 1 − cos(t·π/2), progressive at the
/// bottom as a throttle cam is.
fn pedal_opening(t: f64) -> f64 {
    1.0 - (t * PI * 0.5).cos()
}

/// Crank angle `a` (deg) crossed in the step (prev, cur], on the 720° circle.
fn crossed(prev: f64, cur: f64, a: f64) -> bool {
    let span = crate::units::wrap(cur - prev, 720.0);
    let to_a = crate::units::wrap(a - prev, 720.0);
    to_a > 0.0 && to_a <= span
}

impl Engine {
    pub fn new(def: EngineDef, initial_rpm: f64, seed: u64) -> Self {
        let g = def.geometry;
        let thermo = crate::thermo::CylinderGas::default();
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
                    residual_kg: 0.0,
                    residual_unburnt_kg: 0.0,
                    flame: 1.0,
                    charge_bias: 1.0,
                    armed: false,
                    port_t: PIPE_COLD_K,
                    port_flow: 0.0,
                    valve_hint: f64::NAN,
                    wall_t: PIPE_COLD_K,
                    ref_p: ATM_PA,
                    ref_v: 1.0,
                    ref_t: AMBIENT_K,
                    burn: None,
                    cycle_deg,
                    film: crate::fuel::PortFilm::default(),
                    fuel_kg: 0.0,
                    air_kg: 0.0,
                    slip: crate::fuel::ExhaustMixture::default(),
                    port_mix: crate::fuel::PortMixture::default(),
                    u_j: 0.0,
                    comp: Composition {
                        burned: 0.0,
                        fuelled: 0.0,
                    },
                    t_k: AMBIENT_K,
                    mix: thermo.air,
                    mix_stale: true,
                    fresh_share: 1.0,
                    port_xb: 0.0,
                }
            })
            .map(|mut cs: CylinderState| {
                // air at ambient, its internal energy at the model's datum
                cs.u_j = cs.gas.energy_j;
                settle(&mut cs, &thermo);
                cs
            })
            .collect();
        let mut e = Self {
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
            vtec_engaged: false,
            cam_advance: (0.0, 0.0),
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
            friction_at: std::cell::Cell::new(crate::friction::AtSpeed {
                rpm: f64::NEG_INFINITY,
                rest_pa: 0.0,
                gas_per_pa: 0.0,
            }),
            thermo,
            afterburn_j: 0.0,
            fuel_ledger: FuelLedger::default(),
            rng: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            net: None,
            jet_net: None,
            jet_flows: Vec::new(),
            fv: None,
            intake_net: None,
            tail_pipes,
            listener: None,
            structure: None,
            seat_strikes: Vec::new(),
            texture_signal: 0.0,
            jets: Vec::new(),
            valve_jets: Vec::new(),
            tail_groups: Vec::new(),
            tail_mean_flow: Vec::new(),
            tail_dc: Vec::new(),
            tail_u: Vec::new(),
            tail_u_mean: Vec::new(),
            tails_buf: Vec::new(),
            remote: RemoteSlot(None),
            tail_backpressure: Vec::new(),
            exit_losses: None,
            tail_mach: Vec::new(),
            cyl_mean_flow: Vec::new(),
            friction_age: 0,
            flows: Vec::new(),
            reversion_kg: 0.0,
            inducted_kg: 0.0,
            fidelity: Fidelity::default(),
        };
        // each cylinder's share of the air, a fixed few percent apart (seeded: deterministic)
        for k in 0..e.cyl.len() {
            let n = e.noise() + e.noise() + e.noise();
            e.cyl[k].charge_bias = 1.0 + CYLINDER_IMBALANCE * n;
        }
        e
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
    /// The ECU's valve timing: VTEC engages or drops its high lobes ([`crate::valve::Vtec`]) and
    /// each cam phaser turns towards its map's advance at its rate ([`crate::valve::Phaser`]).
    fn valve_control(&mut self, rpm: f64, throttle: f64, dt: f64) {
        if let Some(v) = &self.def.vtec {
            self.vtec_engaged = v.next(self.vtec_engaged, rpm, throttle);
        }
        let turn = |p: &Option<crate::valve::Phaser>, now: f64| match p {
            Some(p) => p.step(now, p.target(rpm, throttle), dt),
            None => 0.0,
        };
        self.cam_advance = (
            turn(&self.def.intake_phaser, self.cam_advance.0),
            turn(&self.def.exhaust_phaser, self.cam_advance.1),
        );
    }

    /// The valves' timing now: their lobes (high while VTEC is engaged), each camshaft advanced
    /// by its phaser.
    pub fn valves(&self) -> (ValveTiming, ValveTiming) {
        (
            self.def
                .intake_valve
                .effective(self.vtec_engaged, self.cam_advance.0),
            self.def
                .exhaust_valve
                .effective(self.vtec_engaged, self.cam_advance.1),
        )
    }

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

    /// The ECU's spark advance for cylinder `i`'s charge, ° BTDC: MBT for its flame, less the
    /// idle torque reserve ([`idle_reserve`]).
    #[cfg(test)]
    fn spark_advance(&self, i: usize) -> f64 {
        self.def.spark_advance_for(self.cyl[i].flame) * idle_reserve(self.rpm())
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

    /// Friction torque from the engine's friction by components ([`crate::friction`]) at its
    /// speed and manifold pressure; T = FMEP·Vd / 4π. Pumping is not in it: the gas exchange is
    /// simulated. tanh fades it to zero at standstill so the crank does not reverse.
    fn friction_torque(&self) -> f64 {
        // the speed terms are recomputed only when the speed moves by more than FRICTION_RPM
        let rpm = rad_s_to_rpm(self.omega);
        let mut at = self.friction_at.get();
        if (rpm - at.rpm).abs() > FRICTION_RPM {
            at = self.def.friction.at_speed(&self.def.friction_engine(), rpm);
            self.friction_at.set(at);
        }
        at.fmep(self.plenum_pressure_pa()) * self.def.displacement_m3() / (4.0 * PI)
            * (self.omega / 5.0).tanh()
    }

    /// Advance by `dt` seconds; returns the raw acoustic source sample (Pa).
    pub fn step(&mut self, dt: f64, c: &Controls) -> f64 {
        let g = self.def.geometry;
        let crank_deg = self.theta.to_degrees();
        let rpm_now = self.rpm();
        // the ECU first decides whether it injects, then trims the idle accordingly
        self.fuel_cutoffs(c.throttle);
        self.idle_control(dt, c);
        self.plate_step(c.throttle, dt);
        self.dashpot_step(self.plate, dt);
        self.valve_control(rpm_now, c.throttle, dt);
        let (intake_valve, exhaust_valve) = self.valves();
        let ivc = crate::units::wrap(intake_valve.open_deg + intake_valve.duration_deg, 720.0);
        let evc = crate::units::wrap(exhaust_valve.open_deg + exhaust_valve.duration_deg, 720.0);
        // the cut-offs stop the injection; the port film still feeds the cylinders
        let inject = !self.overrun_cut && !self.limiter_cut;
        let (film_x, film_tau) = (self.def.intake.film_fraction, self.def.intake.film_tau_s);
        let film_x = if film_tau > 0.0 { film_x } else { 0.0 };
        // one intake event per cycle per port: the film evaporates over the cycle's period
        let film_share =
            crate::fuel::evaporated_share(4.0 * PI / self.omega.abs().max(1e-3), film_tau);
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
                if self.remote.0.is_none() && self.net.as_ref().is_none_or(|n| n.dt != dt) {
                    self.net = Some(ExhaustNet::with_lining(
                        &self.def.exhaust,
                        self.cyl.len(),
                        dt,
                        self.fidelity.lining_segment_m,
                    ));
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
                if self.def.sound.valve_jet <= 0.0 {
                    self.jet_net = None;
                } else if self.jet_net.as_ref().is_none_or(|n| n.dt != dt) {
                    self.jet_net = Some(ExhaustNet::with_lining(
                        &self.def.exhaust,
                        self.cyl.len(),
                        dt,
                        self.fidelity.lining_segment_m,
                    ));
                    self.jet_flows = vec![0.0; self.cyl.len()];
                }
            }
        }
        if self.flows.len() != self.cyl.len() {
            self.flows = vec![0.0; self.cyl.len()];
        }
        let blowby_area = BLOWBY_AREA_FRACTION * g.piston_area();

        let mut gas_torque = 0.0;
        let reserve = idle_reserve(self.rpm());
        let n_cyl = self.cyl.len();
        for i in 0..n_cyl {
            let arriving = match (&self.net, &self.remote.0) {
                (Some(n), _) => n.arriving(i),
                (None, Some(r)) => r.arriving[i],
                (None, None) => 0.0,
            };
            let back_at_port = self
                .tail_groups
                .iter()
                .position(|g| g.contains(&i))
                .and_then(|t| self.tail_backpressure.get(t))
                .copied()
                .unwrap_or(0.0);
            let prev = self.cyl[i].cycle_deg;
            let cur = crate::units::wrap(crank_deg - self.cyl[i].offset_deg, 720.0);
            let theta_c = deg_to_rad(cur);

            // Piston motion: adiabatic work p·dV.
            let v_new = g.volume(theta_c);
            let th = self.thermo;
            let cs = &mut self.cyl[i];
            let p = cs.gas.pressure();
            cs.u_j -= p * (v_new - cs.gas.volume_m3);
            cs.gas.set_volume(v_new);
            settle(cs, &th);

            // Valve and outlet flows.
            // Both valve flows are planned from the same state (Jacobi) and each may close
            // at most half the pressure gap: during overlap, sequential full equalisation
            // with plenum then runner ping-pongs every substep (loud numerical chatter).
            let a_in = intake_valve.flow_area(cur);
            let a_ex = exhaust_valve.flow_area(cur);
            // The intake valve draws from the 0D plenum, or from its runner's last cell (1D).
            let f_in = (a_in > 0.0).then(|| match &self.intake_net {
                Some(net) => plan_exchange(&net.port_volume(i), &cs.gas, a_in, dt, VALVE_RELAX),
                None => plan_exchange(&self.plenum, &cs.gas, a_in, dt, VALVE_RELAX),
            });
            // The exhaust valve is its primary's boundary, solved by characteristics (Benson;
            // `valve_boundary`): the wave the pipe sends to the valve — the FV's first cell, or the
            // waveguide's arriving wave on its quiet gas — meets the cylinder through the valve.
            let pipe_area = match (&self.fv, &self.net, &self.remote.0) {
                (Some(fv), _, _) => fv.primary_area(),
                (None, Some(n), _) => n.primary_area(),
                (None, None, Some(r)) => r.a_pipe,
                _ => PI * 0.25 * self.def.exhaust.primary_diameter_m.powi(2),
            };
            let (lambda, t_back) = match &self.fv {
                Some(fv) => fv.port_lambda(i, cs.gas.pressure()),
                None => (
                    // the arriving wave on the primary's gas at rest at the atmosphere's pressure
                    // plus the tail's mean backpressure
                    crate::valve_boundary::lambda_of_wave(
                        cs.gas.pressure(),
                        arriving,
                        cs.port_t,
                        ATM_PA + back_at_port,
                    ),
                    cs.port_t,
                ),
            };
            // the boundary starts from this valve's last root (`valve_hint`)
            let hint = std::cell::Cell::new(cs.valve_hint);
            let plan_ex = |area: f64| {
                let mut h = hint.get();
                let pipe = crate::valve_boundary::Pipe {
                    area: pipe_area,
                    lambda,
                    t_back,
                };
                let f =
                    crate::valve_boundary::valve_flow(&cs.gas, area, pipe, dt, VALVE_RELAX, &mut h);
                hint.set(h);
                f
            };
            // Reverse flow (pipe → cylinder) passes the valve against the port's curve with a lower
            // discharge coefficient than forward flow (Blair measures separate in/out maps).
            let fb_ex = (a_ex > 0.0).then(|| {
                let fb = plan_ex(a_ex);
                if fb.0.dm < 0.0 && self.def.exhaust_reverse_cd_ratio < 1.0 {
                    plan_ex(a_ex * self.def.exhaust_reverse_cd_ratio)
                } else {
                    fb
                }
            });
            let f_ex = fb_ex.map(|fb| fb.0);
            cs.valve_hint = hint.get();
            // the FV primary's first face takes the boundary's momentum (p + ρu²)
            if let (Some(fv), Some((_, b))) = (&mut self.fv, fb_ex) {
                let rho = crate::gas::GAMMA * b.p / (b.a * b.a);
                fv.add_valve_boundary(i, b.p + rho * b.u * b.u, dt);
            }
            if let Some(f) = f_ex
                && f.dm < 0.0
            {
                self.reversion_kg -= f.dm;
            }
            if let Some(f) = f_in {
                // dm > 0: intake → cylinder (fresh charge); < 0: the cylinder's gas backflows
                let into_cyl = Flow { dm: -f.dm, h: f.h };
                match &mut self.intake_net {
                    Some(net) => net.add_valve_flow(i, f.dm, f.h),
                    None => {
                        self.plenum.mass_kg -= f.dm;
                        self.plenum.energy_j -= f.dm * f.h;
                    }
                }
                // port injection: the charge carries fuel unless the ECU cuts it
                let fresh = Composition {
                    burned: 0.0,
                    fuelled: if inject { 1.0 } else { 0.0 },
                };
                cylinder_flow(cs, &th, into_cyl, fresh);
            }
            cs.port_flow = f_ex.map_or(0.0, |f| f.dm / dt);
            let cyl_state = (cs.gas.pressure(), cs.gas.temperature());
            let port_t = cs.port_t;
            self.flows[i] = f_ex.map_or(0.0, |f| {
                // a reversion brings back burned gas
                let port = Composition {
                    burned: cs.port_xb,
                    fuelled: cs.comp.fuelled,
                };
                cylinder_flow(cs, &th, f, port);
                exhaust_source_flow(f.dm / dt, f.h)
            });
            // The valve's jet: its mixing noise, generated in the port, rides down the primary
            // with the pulse — a pressure added to the wave the valve sends. With the FV exhaust
            // it rides the jet net (`jet_net`): a small perturbation, linear on the gas dynamics.
            let jet_level = self.def.sound.valve_jet;
            let a_pipe = match (&self.net, &self.remote.0, &self.jet_net) {
                (Some(n), _, _) => Some(n.primary_area()),
                (None, Some(r), _) => Some(r.a_pipe),
                (None, None, Some(j)) => Some(j.primary_area()),
                _ => None,
            };
            if let Some(a_pipe) = a_pipe
                && jet_level > 0.0
                && a_ex > 0.0
            {
                if self.valve_jets.len() != n_cyl {
                    let base = self.rng;
                    self.valve_jets = (0..n_cyl)
                        .map(|k| crate::jet_noise::ValveJet::new(base.wrapping_add(101 + k as u64)))
                        .collect();
                }
                let c = crate::exhaust::C_PRIMARY;
                match &self.fv {
                    None => {
                        // the port: the atmosphere, the wave arriving and the one the flow sends
                        // (p⁺ + p⁻)
                        let p_port = ATM_PA + 2.0 * arriving + c * self.flows[i] / a_pipe;
                        let port = (p_port, port_t);
                        let p = self.valve_jets[i].step(cyl_state, port, a_ex, a_pipe, dt);
                        self.flows[i] += jet_level * p * a_pipe / c;
                    }
                    Some(fv) => {
                        // the port is the first FV cell's gas
                        let port = fv.port_state(i);
                        let p = self.valve_jets[i].step(cyl_state, port, a_ex, a_pipe, dt);
                        self.jet_flows[i] = jet_level * p * a_pipe / c;
                    }
                }
            }
            {
                let ex = &self.def.exhaust;
                let m = primary_gas_mass(ex, cs.port_t);
                let (dm, t_in) = f_ex.map_or((0.0, cs.port_t), |f| (f.dm, f.h / crate::gas::CP));
                // The unburnt mixture rides with the gas out of the cylinder into the primary
                // (see `fuel::PortMixture`), and a reversion brings some back.
                let out = if dm > 0.0 {
                    cs.slip.take_share(dm / (cs.gas.mass_kg + dm))
                } else {
                    crate::fuel::ExhaustMixture::default()
                };
                let a_wall = PI * ex.primary_diameter_m * ex.primary_length_m;
                cs.port_t = port_gas_temperature(cs.port_t, m, dm, t_in, cs.wall_t, a_wall, dt);
                if dm > 0.0 {
                    cs.port_xb = (m * cs.port_xb + dm * cs.comp.burned) / (m + dm);
                }
                let r =
                    cs.port_mix
                        .step(out, (m, dm), (t_in, cs.port_t, ATM_PA), dt, AFTERBURN_TAU_S);
                cs.slip.add(r.back);
                self.fuel_ledger.exited += r.exit.fuel_kg;
                if r.heat_j > 0.0 {
                    let (rise, source) = primary_heat(r.heat_j, m, dt);
                    cs.port_t += rise;
                    self.flows[i] += source;
                    self.afterburn_j += r.heat_j;
                    self.fuel_ledger.afterburnt += r.heat_j / LHV_GASOLINE;
                }
                cs.wall_t += (cs.port_t - cs.wall_t) * (dt / PIPE_WALL_TAU_S);
            }
            if let (Some(fv), Some(f)) = (&mut self.fv, f_ex) {
                fv.add_valve_flow(i, f.dm, f.h);
            }
            // Blow-by past the rings into the (vented) crankcase. Small while running, but it
            // is what lets a charge trapped at a stop bleed off so the starter can turn it.
            let f = crate::gas::plan_with_reservoir(&cs.gas, &self.atm, blowby_area, dt, 1.0);
            let air = Composition {
                burned: 0.0,
                fuelled: 0.0,
            };
            cylinder_flow(cs, &th, f, air);
            settle(cs, &th);

            // Combustion events.
            // a valve closing strikes its seat (both kinds: one valve each, conservative)
            let strike = SEAT_FORCE_N * (rpm_now / SEAT_FORCE_RPM);
            if crossed(prev, cur, evc) || crossed(prev, cur, ivc) {
                self.seat_strikes.push((0.0, strike));
            }
            if crossed(prev, cur, evc) {
                cs.residual_kg = cs.gas.mass_kg;
            }
            if crossed(prev, cur, ivc) {
                cs.trapped_kg = cs.gas.mass_kg;
                cs.armed = true;
                // The ECU meters the fuel for a stoichiometric charge of the fresh air it
                // finds, compensating the port film (Aquino); a cut injects nothing and the film
                // still gives back its share. What the last cycle left unburnt stays in.
                let fresh = (cs.trapped_kg - cs.residual_kg).max(0.0);
                let air = fresh * AFR_STOICH / (1.0 + AFR_STOICH);
                let film_x =
                    crate::fuel::deposit_fraction(film_x, cs.gas.pressure(), self.omega.abs());
                let fuel = if inject {
                    cs.film.meter(air / AFR_STOICH, film_x, film_share).1
                } else {
                    cs.film.intake(0.0, film_x, film_share)
                };
                cs.residual_unburnt_kg = cs.slip.fuel_kg + cs.slip.air_kg;
                self.fuel_ledger.into_cylinders += fuel;
                cs.fuel_kg = fuel + cs.slip.fuel_kg;
                cs.air_kg = air + cs.slip.air_kg;
                cs.slip = crate::fuel::ExhaustMixture::default();
                cs.ref_p = cs.gas.pressure();
                cs.ref_v = cs.gas.volume_m3;
                cs.ref_t = cs.t_k;
                cs.fresh_share = 1.0 - cs.comp.burned;
                // the ECU reads the charge (speed, manifold, temperature) and times its spark
                let dilution = dilution_of(cs);
                let ratio = flame_ratio(
                    &g,
                    mean_piston_speed,
                    (cs.ref_p, cs.ref_t),
                    cs.gas.mass_kg / cs.ref_v,
                    cs.ref_v,
                    (1.0, dilution),
                );
                if ratio.is_finite() {
                    cs.flame = ratio;
                }
            }
            let spark_deg =
                720.0 - (self.def.spark_advance_for(cs.flame) * reserve + self.idle_spark_deg);
            let mut new_burn = false;
            // The spark fires on the first step at or past its angle (within the half cycle
            // after it), so a timing that moved earlier past the crank still fires this charge,
            // and one that moved later cannot fire it again. A cut cylinder's charge passes.
            if cs.armed && crate::units::wrap(cur - spark_deg, 720.0) < 360.0 {
                cs.armed = false;
                // an empty or fuelless charge has no flame (`cycle_flame`)
                new_burn = c.ignition;
            }
            // The exhaust valve opens on whatever the flame has not burnt yet: it leaves
            // unburnt with the gas (a partial burn), as does a charge that never lit.
            if crossed(prev, cur, exhaust_valve.open_deg) {
                if let Some(b) = cs.burn.take() {
                    cs.slip.add(b.unburnt());
                }
                if !cs.armed {
                    cs.slip.add(crate::fuel::ExhaustMixture {
                        fuel_kg: cs.fuel_kg,
                        air_kg: cs.air_kg,
                    });
                    (cs.fuel_kg, cs.air_kg) = (0.0, 0.0);
                }
            }
            if let Some(b) = &mut cs.burn {
                // Signed angle since the burn start, in (−360, 360]: the burn is created at
                // the spark, a few degrees *before* it starts, and must not wrap to "done".
                let mut since = crate::units::wrap(cur - b.start_deg, 720.0);
                if since > 360.0 {
                    since -= 720.0;
                }
                let x = wiebe(since, b.duration_deg);
                cs.u_j += b.energy_j * (x - b.released);
                // what burns turns the fresh charge into products
                cs.comp.burned =
                    (cs.comp.burned + cs.fresh_share * (x - b.released)).clamp(0.0, 1.0);
                cs.mix_stale = true;
                self.fuel_energy_j += b.energy_j * (x - b.released);
                self.fuel_ledger.burnt +=
                    crate::fuel::burnable(b.fuel_kg, b.air_kg) * (x - b.released);
                b.released = x;
                if since > 2.0 * b.duration_deg {
                    cs.slip.add(b.unburnt());
                    cs.burn = None;
                }
            }

            // Wall heat loss, Woschni. The combustion term compares the pressure with the
            // motored one, a polytropic (n = 1.32) compression from intake closing.
            // the gas as settled this step before the flame's increment (one substep behind it:
            // the wall loss settles together with the burn below)
            let t_gas = cs.t_k;
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
            let q = (htc * area * (t_gas - WALL_TEMP_K) * dt).min(0.5 * cs.u_j);
            cs.u_j -= q;
            settle(cs, &th);
            self.wall_heat_j += q;

            gas_torque += (cs.gas.pressure() - ATM_PA) * g.piston_area() * g.dpos_dtheta(theta_c);

            cs.cycle_deg = cur;

            if new_burn {
                // the flame this charge lights: its delay, burn and their dispersion scale
                // from the preset's values at the reference state (see `flame_ratio`)
                // This cycle's charge as the spark finds it (Heywood §9.4.2): the mixture near
                // the plug — its A/F (σ 2–6 % of the mean, Fig. 9-34a: 4 %) and its residual
                // (CO₂ scatter ±9 %, Fig. 9-34b) — and the turbulence there
                // ([`TURBULENCE_DISPERSION`]), which moves the flame's angles as u′^(−2/3)
                // (Hires eqs. 6–7). The flame the ECU timed (`cs.flame`) is the mean one. The
                // mean A/F is the charge's own: the injection and what the port film gave back.
                let (dilution, ref_p, ref_t, ref_v, bias) = (
                    dilution_of(cs),
                    cs.ref_p,
                    cs.ref_t,
                    cs.ref_v,
                    cs.charge_bias,
                );
                let (fuel, air) = (cs.fuel_kg, cs.air_kg);
                let phi = fuel * AFR_STOICH / air.max(1e-12);
                let rho_ivc = cs.trapped_kg / cs.ref_v;
                let j = self.def.sound.combustion_jitter;
                // three near-Gaussian draws of unit σ (each a sum of three uniforms of ±1)
                let mut draw = || -> f64 { (0..3).map(|_| self.noise()).sum() };
                let draws = (j * draw(), j * draw(), j * draw());
                let charge = 1.0 + CHARGE_DISPERSION * j * draw();
                let flame = cycle_flame(
                    &g,
                    mean_piston_speed,
                    (ref_p, ref_t),
                    rho_ivc,
                    ref_v,
                    (phi, dilution),
                    draws,
                );
                // the charge itself: each cylinder breathes a fixed few percent apart from the
                // others, and each cycle a little more or less ([`CHARGE_DISPERSION`])
                let var = bias * charge;
                let cs = &mut self.cyl[i];
                // a mixture whose flame cannot propagate does not light: it waits for the
                // exhaust valve, unburnt
                if let Some(ratio) = flame {
                    let duration = self.def.burn_duration_deg * ratio;
                    cs.burn = Some(Burn {
                        start_deg: crate::units::wrap(
                            spark_deg + IGNITION_DELAY_DEG * ratio,
                            720.0,
                        ),
                        duration_deg: duration,
                        energy_j: burn_energy(fuel, air, var),
                        released: 0.0,
                        fuel_kg: fuel,
                        air_kg: air,
                    });
                    (cs.fuel_kg, cs.air_kg) = (0.0, 0.0);
                    self.combustions += 1;
                }
            }
        }
        // Acoustic source: what the tail pipes radiate.
        // advance the exhaust; what each tail radiates is read below (`last_tails`)
        match (&mut self.net, &mut self.fv, &mut self.remote.0) {
            (_, Some(fv), _) => {
                fv.set_outflow_mach(&self.tail_mach);
                fv.step();
                if let Some(j) = &mut self.jet_net {
                    j.prepare(&self.tail_mach, None);
                    j.step(&self.jet_flows);
                    self.jet_flows.iter_mut().for_each(|f| *f = 0.0);
                }
            }
            (Some(net), None, _) => {
                // the mean flows move over cycles: the pipes' friction follows them every 64 steps
                self.friction_age += 1;
                let due = self.friction_age >= 64;
                if due {
                    self.friction_age = 0;
                }
                let mean = due.then_some((&self.cyl_mean_flow[..], &self.tail_mean_flow[..]));
                net.prepare(&self.tail_mach, mean);
                net.step(&self.flows);
            }
            (None, None, Some(r)) => {
                // the net runs elsewhere: stage this step; a full block goes out and the one
                // before it comes back; the next step's waves come from the primaries' lines
                self.friction_age += 1;
                let due = self.friction_age >= 64;
                if due {
                    self.friction_age = 0;
                }
                r.stage.steps += 1;
                r.stage.flows.extend_from_slice(&self.flows);
                r.stage.mach.extend_from_slice(&self.tail_mach);
                r.stage.has_mean.push(due);
                if due {
                    r.stage.mean.extend_from_slice(&self.cyl_mean_flow);
                    r.stage.mean.extend_from_slice(&self.tail_mean_flow);
                    r.probe.set_mean_flow(&self.cyl_mean_flow);
                }
                r.taken += 1;
                if r.stage.steps == r.block {
                    r.hand_over();
                }
                r.probe
                    .arriving((r.taken - r.applied) as usize, &mut r.arriving);
            }
            _ => {}
        }
        if self.listener.as_ref().is_none_or(|l| l.0 != dt) {
            let (ex, snd) = (&self.def.exhaust, &self.def.sound);
            let l = Listener::with_street(
                self.tail_pipes,
                ex.tail_spacing_m,
                snd.listener_distance_m,
                snd.listener_angle_deg,
                PI * 0.25 * ex.outlet_diameter().powi(2),
                dt,
                snd.ground_heights(),
                snd.street(),
            );
            self.listener = Some((dt, l));
        }
        let n_tails = match (&self.net, &self.fv, &self.remote.0) {
            (_, Some(fv), _) => fv.last_tails().len(),
            (Some(net), None, _) => net.last_tails().len(),
            (None, None, Some(r)) => r.tails_now.len(),
            _ => 0,
        };
        let throttle_area = self.throttle_area(self.plate);
        // 0D: throttle into the plenum. 1D: the network steps itself (ITBs take the same area each).
        let (throttle_dm, _induction, drawn) = match &mut self.intake_net {
            Some(net) => {
                // `throttle_diameter_m` is the total equivalent opening; ITBs share it.
                let area = if self.def.intake.individual_throttles {
                    throttle_area / self.cyl.len().max(1) as f64
                } else {
                    throttle_area
                };
                let induction = net.step(area);
                (-net.last_inducted(), induction, net.drawn_rate())
            }
            None => {
                let dm = exchange_with_reservoir(&mut self.plenum, &self.atm, throttle_area, dt);
                // `dm` is the atmosphere's change: negative drawing in
                (dm, 0.0, -dm / dt)
            }
        };

        self.inducted_kg -= throttle_dm;

        // The intake mouth is a sink of the air it draws in: a monopole of volume flow
        // −ṁ/ρ₀ (1D: what the trumpets take in; 0D: the throttle's flow), heard from the
        // tails' mean distance — no gain chosen by ear. `sound.intake` scales it (1 = physical).
        let snd = &self.def.sound;
        // `drawn`: the mouth's mass flow into the engine, kg/s (1D: held between FV steps)
        let intake_flow = -snd.intake * drawn / AIR_DENSITY;
        self.texture_signal = intake_flow;
        // each tail's jet: its exit velocity u = (ρc·u)/ρc, its own distance to the listener
        // Exit velocity at each tail: the mean is mass continuity, ṁ/(ρA), with ṁ the exhaust
        // valves' flow averaged over a cycle; the swing is the waveguide's, without its own
        // average (a linear line's DC is no flow: losses and reflections do not conserve it).
        if self.tail_groups.len() != n_tails {
            let mut groups = self.def.exhaust.groups(self.cyl.len());
            groups.resize(n_tails, Vec::new());
            self.tail_groups = groups;
            self.tail_mean_flow = vec![0.0; n_tails];
            self.tail_dc = vec![0.0; n_tails];
            self.tail_u = vec![0.0; n_tails];
            self.tail_u_mean = vec![0.0; n_tails];
            self.tail_backpressure = vec![0.0; n_tails];
            self.exit_losses = Some(self.def.exhaust.exit_losses());
            self.tail_mach = vec![0.0; n_tails];
        }
        // one engine cycle (two turns), bounded for a stopped or racing crank
        let k = (dt * self.omega.abs() / (4.0 * PI)).clamp(dt, 1.0);
        {
            if self.cyl_mean_flow.len() != self.cyl.len() {
                self.cyl_mean_flow = vec![0.0; self.cyl.len()];
            }
            for (mean, cs) in self.cyl_mean_flow.iter_mut().zip(&self.cyl) {
                *mean += (cs.port_flow - *mean) * k;
            }
            for t in 0..n_tails {
                let m: f64 = self.tail_groups[t]
                    .iter()
                    .map(|&i| self.cyl[i].port_flow)
                    .sum();
                self.tail_mean_flow[t] += (m - self.tail_mean_flow[t]) * k;
                let losses = self
                    .exit_losses
                    .get_or_insert_with(|| self.def.exhaust.exit_losses());
                let (back, u_mean) = tail_exit(self.tail_mean_flow[t], losses);
                self.tail_backpressure[t] = back;
                self.tail_mach[t] = u_mean / crate::exhaust::C_TAIL;
                self.tail_u_mean[t] = u_mean;
            }
        }
        if snd.flow_noise > 0.0 && self.listener.is_some() && self.jets.len() != n_tails {
            // seeded from the engine's own generator: deterministic for a seed
            let base = self.rng;
            self.jets = (0..n_tails)
                .map(|k| JetNoise::new(base.wrapping_add(k as u64 + 1)))
                .collect();
        }
        // The block: every cylinder's pressure shakes the same structure, which radiates it as
        // combustion noise, heard from the tails' mean distance like the intake mouth.
        // The valves striking their seats shake the same structure through the head, a force
        // F standing for a pressure F/A_bore on it (the path the gas force takes): the
        // valvetrain's ticking (`sound.mechanical`, 1 = physical).
        let mut seat_force = 0.0;
        for (t, peak) in self.seat_strikes.iter_mut() {
            seat_force += seat_pulse(*peak, *t);
            *t += dt;
        }
        self.seat_strikes.retain(|(t, _)| *t <= SEAT_PULSE_S);
        // (always run: a few multiplies a step, and an engine without either is silent here)
        let block = {
            if self.structure.as_ref().is_none_or(|s| s.0 != dt) {
                self.structure = Some((dt, StructureResponse::new(dt)));
            }
            let p: f64 = self.cyl.iter().map(|cs| cs.gas.pressure()).sum();
            let seats = seat_force / g.piston_area();
            match &mut self.structure {
                Some((_, s)) => s.process(snd.block * p + snd.mechanical * seats),
                None => 0.0,
            }
        };
        // what does not wait for the tails: the intake mouth and the block. The mouth is a
        // runner's trumpet with individual throttles, else the throttle bore (the air box's
        // own inlet is not in the definition: an estimate).
        let mouth_d = if self.def.intake.individual_throttles {
            self.def.intake.runner_diameter_m
        } else {
            self.def.throttle_diameter_m
        };
        let mouth_area = PI * 0.25 * mouth_d * mouth_d;
        let rest = match &mut self.listener {
            Some((_, l)) => l.radiate_mouth(intake_flow, mouth_area) + l.spread_from_1m(block),
            None => 0.0,
        };
        let audio = if let Some(r) = self.remote.0.as_mut() {
            // the net runs elsewhere: the sound of the step `2·block` before is finished now
            // (the same samples, in the same order)
            let mut u = r.spare.pop().unwrap_or_default();
            u.clone_from(&self.tail_u_mean);
            r.pending.push_back((rest, k, u));
            if r.pending.len() > 2 * r.block {
                self.finish_remote_step(dt).unwrap_or(0.0)
            } else {
                0.0
            }
        } else {
            let mut tails = std::mem::take(&mut self.tails_buf);
            match (&self.net, &self.fv) {
                (_, Some(fv)) => {
                    tails.clear();
                    tails.extend_from_slice(fv.last_tails());
                    if let Some(j) = &self.jet_net {
                        for (t, &n) in tails.iter_mut().zip(j.last_tails()) {
                            *t += n;
                        }
                    }
                }
                (Some(net), None) => {
                    tails.clear();
                    tails.extend_from_slice(net.last_tails());
                }
                _ => tails.clear(),
            }
            let u = std::mem::take(&mut self.tail_u_mean);
            let a = self.tails_sound(&tails, &u, rest, k, dt);
            self.tail_u_mean = u;
            self.tails_buf = tails;
            a
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
        self.theta = crate::units::wrap(self.theta + self.omega * dt, 4.0 * PI);
        audio
    }

    /// The sound of a step whose tails are in: the listener hears them, each tail's jet swings
    /// with its wave around the mean exit velocity `u_mean`, and `rest` (intake mouth, block)
    /// adds. `k`: that step's cycle-mean constant.
    fn tails_sound(&mut self, tails: &[f64], u_mean: &[f64], rest: f64, k: f64, dt: f64) -> f64 {
        let heard = match &mut self.listener {
            Some((_, l)) => l.radiate(tails),
            None => 0.0,
        };
        let z = crate::exhaust::tail_impedance();
        for (t, &signal) in tails.iter().enumerate() {
            self.tail_dc[t] += (signal - self.tail_dc[t]) * k;
            // the swing rides on the mean; no exit blows past sound (it chokes first)
            self.tail_u[t] = (u_mean[t] + (signal - self.tail_dc[t]) / z).min(TAIL_SONIC);
        }
        let mut jets = 0.0;
        let snd = &self.def.sound;
        if snd.flow_noise > 0.0
            && let Some((_, l)) = &self.listener
        {
            let rho = crate::exhaust::tail_reference_density();
            let d = self.def.exhaust.outlet_diameter();
            for ((jet, u), r) in self.jets.iter_mut().zip(&self.tail_u).zip(l.distances()) {
                jets += jet.step(*u, d, rho, *r, dt);
            }
            jets *= snd.flow_noise;
        }
        heard + jets + rest
    }

    /// Finish the oldest step whose sound waits for its tails (they are in).
    fn finish_remote_step(&mut self, dt: f64) -> Option<f64> {
        let r = self.remote.0.as_mut()?;
        let (rest, k, u) = r.pending.pop_front()?;
        for t in r.tails_now.iter_mut() {
            *t = r.tails.pop_front().unwrap_or(0.0);
        }
        let tails = std::mem::take(&mut r.tails_now);
        let a = self.tails_sound(&tails, &u, rest, k, dt);
        if let Some(r) = self.remote.0.as_mut() {
            r.tails_now = tails;
            r.spare.push(u);
        }
        Some(a)
    }

    /// Run the waveguide exhaust elsewhere from now on: `link` is handed the net and the most
    /// steps a block can have (`max`: the block its link chooses is at most that) and receives
    /// the steps' work a block at a time ([`ExhaustLink`]). The sound comes out `2·block` steps
    /// late — each [`Engine::step`] returns the one that far back, 0 at first — and
    /// [`Engine::flush_exhaust`] gives the rest. The same samples, to the bit, as running it
    /// here. False (nothing changed) without a waveguide net, or with primaries too short for a
    /// block to be worth it ([`ExhaustNet::max_block`]).
    pub fn detach_exhaust(
        &mut self,
        link: impl FnOnce(ExhaustNet, usize) -> Box<dyn ExhaustLink>,
    ) -> bool {
        let Some(net) = self.net.as_ref() else {
            return false;
        };
        let n = self.cyl.len();
        let max = net.max_block();
        if self.remote.0.is_some() || max == 0 {
            return false;
        }
        let arriving = (0..n).map(|i| net.arriving(i)).collect();
        let a_pipe = net.primary_area();
        let tails_now = vec![0.0; net.last_tails().len()];
        let probe = ArrivalProbe::new(net);
        let Some(net) = self.net.take() else {
            return false;
        };
        let link = link(net, max);
        let block = link.block().clamp(1, max);
        self.remote = RemoteSlot(Some(Box::new(Remote {
            link,
            probe,
            arriving,
            a_pipe,
            block,
            stage: BlockIn::default(),
            in_flight: 0,
            taken: 0,
            applied: 0,
            out: BlockOut::default(),
            tails: std::collections::VecDeque::new(),
            tails_now,
            pending: std::collections::VecDeque::new(),
            spare: Vec::new(),
        })));
        true
    }

    /// The sounds of the steps taken with the net elsewhere that are still waiting for their
    /// tails, pushed to `out` in order (the partial block goes out and everything comes back):
    /// how many. Then the link has nothing outstanding.
    pub fn flush_exhaust(&mut self, dt: f64, out: &mut Vec<f64>) -> usize {
        let Some(r) = self.remote.0.as_mut() else {
            return 0;
        };
        if r.stage.steps > 0 {
            r.hand_over();
        }
        while r.in_flight > 0 {
            r.take_oldest();
        }
        let mut count = 0;
        while let Some(a) = self.finish_remote_step(dt) {
            out.push(a);
            count += 1;
        }
        count
    }

    /// Stop running the net elsewhere (after [`Engine::flush_exhaust`]): the link, to get the
    /// net back from wherever it runs and [`Engine::attach_exhaust`] it.
    pub fn end_detached(&mut self) -> Option<Box<dyn ExhaustLink>> {
        self.remote.0.take().map(|r| r.link)
    }

    /// Step the 1D intake's runners from `first` on elsewhere (individual throttles only):
    /// `link` is handed them ([`IntakeFv::detach_runners`]). The same samples, no delay.
    pub fn detach_runners(
        &mut self,
        first: usize,
        link: impl FnOnce(crate::intake_fv::RunnerSet) -> Box<dyn crate::intake_fv::RunnerLink>,
    ) -> bool {
        self.intake_net
            .as_mut()
            .is_some_and(|n| n.detach_runners(first, link))
    }

    /// Stop stepping runners elsewhere: the link, to get them back and attach them again.
    pub fn end_detached_runners(&mut self) -> Option<Box<dyn crate::intake_fv::RunnerLink>> {
        self.intake_net.as_mut().and_then(|n| n.end_detached())
    }

    /// The detached runners back.
    pub fn attach_runners(&mut self, set: crate::intake_fv::RunnerSet) {
        if let Some(n) = self.intake_net.as_mut() {
            n.attach_runners(set);
        }
    }

    /// Run the net here again.
    pub fn attach_exhaust(&mut self, net: ExhaustNet) {
        self.remote = RemoteSlot(None);
        self.net = Some(net);
    }

    /// Exit velocity at each tail pipe this step, m/s (mean from mass continuity).
    pub fn tail_exit_velocity(&self) -> &[f64] {
        &self.tail_u
    }

    /// How many cylinders the engine runs.
    pub fn cylinders(&self) -> usize {
        self.cyl.len()
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

    /// Heat released by unburnt mixture burning in the exhaust primaries so far, J.
    pub fn afterburn_j(&self) -> f64 {
        self.afterburn_j
    }
    pub fn cylinder(&self, i: usize) -> CylinderView {
        let cs = &self.cyl[i];
        let g = self.def.geometry;
        CylinderView {
            cycle_deg: cs.cycle_deg,
            piston_m: g.piston_position(deg_to_rad(cs.cycle_deg)),
            pressure_pa: cs.gas.pressure(),
            temperature_k: cs.t_k,
            intake_lift_m: self.valves().0.lift(cs.cycle_deg),
            exhaust_lift_m: self.valves().1.lift(cs.cycle_deg),
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
    use crate::combustion::charge_energy;
    use approx::assert_relative_eq;

    const DT: f64 = 1.0 / 48_000.0 / 4.0;

    fn run(e: &mut Engine, seconds: f64, c: &Controls) -> Vec<f64> {
        let n = (seconds / DT) as usize;
        (0..n).map(|_| e.step(DT, c)).collect()
    }

    /// Each new burn of cylinder 1 over `seconds` at `throttle`, held at `rpm`: (ignition delay
    /// after the spark, duration), degrees.
    fn burns(throttle: f64, rpm: f64, seconds: f64) -> Vec<(f64, f64)> {
        let mut e = Engine::new(EngineDef::inline4(), rpm, 1);
        let c = Controls {
            throttle,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        run(&mut e, 0.5, &c);
        let mut out = Vec::new();
        let mut last = None;
        for _ in 0..(seconds / DT) as usize {
            e.step(DT, &c);
            if let Some(b) = &e.cyl[0].burn {
                let key = (b.start_deg, b.duration_deg);
                if last != Some(key) {
                    last = Some(key);
                    let spark = 720.0 - e.spark_advance(0) - e.idle_spark_deg;
                    out.push(((b.start_deg - spark).rem_euclid(720.0), b.duration_deg));
                }
            }
        }
        out
    }

    #[test]
    fn burn_angles_vary_cycle_to_cycle_as_measured_engines_do() {
        // Heywood §9.4.2, Fig. 9-35 (many chambers and operating conditions): the standard
        // deviations of the flame-development and rapid-burning angles stay a near-constant
        // share of their means, about 0.12 (read: 1.5/15, 3/27, 5.5/40 for the development
        // angle; 3/30, 7/60, 13/100 for the burn), at fast and slow burns alike
        for (throttle, rpm) in [(1.0, 3000.0), (0.15, 2000.0)] {
            let b = burns(throttle, rpm, 2.0);
            let rel = |v: &[f64]| {
                let m = v.iter().sum::<f64>() / v.len() as f64;
                (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt() / m
            };
            let dev: Vec<f64> = b.iter().map(|x| x.0).collect();
            let burn: Vec<f64> = b.iter().map(|x| x.1).collect();
            let (d, r) = (rel(&dev), rel(&burn));
            assert!(
                (0.09..0.15).contains(&d) && (0.09..0.15).contains(&r),
                "throttle {throttle}: σ/mean development {d:.3}, burn {r:.3} over {} cycles",
                b.len()
            );
        }
    }

    #[test]
    fn a_light_load_burns_slower_and_less_repeatably_than_full_throttle() {
        // Hires et al. (SAE 780232) eqs. 13/32 with Metghalchi & Keck's flame speed: at the same
        // speed a part-throttle charge is thinner (lower p) and more diluted by residual gas, so
        // its laminar flame is slower and both the delay and the burn span more crank angle
        // (Heywood §9.3: the flame-development angle grows with dilution); the early flame's
        // dispersion grows with its time scale
        let stats = |v: &[(f64, f64)], k: usize| {
            let xs: Vec<f64> = v.iter().map(|b| if k == 0 { b.0 } else { b.1 }).collect();
            let m = xs.iter().sum::<f64>() / xs.len() as f64;
            let sd = (xs.iter().map(|x| (x - m).powi(2)).sum::<f64>() / xs.len() as f64).sqrt();
            (m, sd)
        };
        let (light, full) = (burns(0.08, 2000.0, 1.5), burns(1.0, 2000.0, 1.5));
        assert!(light.len() > 10 && full.len() > 10);
        let ((d_l, sd_l), (d_f, sd_f)) = (stats(&light, 0), stats(&full, 0));
        let ((b_l, _), (b_f, _)) = (stats(&light, 1), stats(&full, 1));
        assert!(b_l > 1.15 * b_f, "burn {b_l:.1}° light vs {b_f:.1}° full");
        assert!(d_l > 1.15 * d_f, "delay {d_l:.2}° light vs {d_f:.2}° full");
        // the spread is a fixed share of each cycle's flame (Heywood §9.4.2's measured spreads,
        // not load-dependent in the model): in degrees it is at least the full-load one.
        // The measured growth at light load beyond that (partial burns) is a known gap
        assert!(
            sd_l >= sd_f,
            "delay spread {sd_l:.3}° of {d_l:.2}° light vs {sd_f:.3}° of {d_f:.2}° full"
        );
    }

    /// Peak pressure of each cycle of cylinder 1, held at `rpm` with `throttle`, over `secs`.
    fn cycle_peaks(d: EngineDef, rpm: f64, throttle: f64, secs: f64) -> Vec<f64> {
        let mut e = Engine::new(d, rpm, 1);
        let c = Controls {
            throttle,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        run(&mut e, 1.0, &c);
        let (mut peaks, mut pk, mut prev) = (Vec::new(), 0.0f64, e.cyl[0].cycle_deg);
        for _ in 0..(secs / DT) as usize {
            e.step(DT, &c);
            let cur = e.cyl[0].cycle_deg;
            if cur < prev {
                peaks.push(pk);
                pk = 0.0;
            }
            pk = pk.max(e.cyl[0].gas.pressure());
            prev = cur;
        }
        peaks.remove(0);
        peaks
    }

    fn cov(x: &[f64]) -> f64 {
        let m = x.iter().sum::<f64>() / x.len() as f64;
        (x.iter().map(|v| (v - m).powi(2)).sum::<f64>() / x.len() as f64).sqrt() / m
    }

    #[test]
    fn each_cylinder_breathes_a_few_percent_apart() {
        // Heywood §9.4.2: the air and fuel each cylinder gets differ by "typically a few
        // percent", fixed — what puts sub-firing orders between a multi-cylinder engine's
        // harmonics. Seeded: the same engine twice is the same engine
        let e = Engine::new(EngineDef::lfa_v10(), 1000.0, 7);
        let b: Vec<f64> = e.cyl.iter().map(|c| c.charge_bias).collect();
        let m = b.iter().sum::<f64>() / b.len() as f64;
        let sd = (b.iter().map(|x| (x - m).powi(2)).sum::<f64>() / b.len() as f64).sqrt();
        assert!((0.005..0.05).contains(&sd), "spread {sd}: {b:?}");
        let again: Vec<f64> = Engine::new(EngineDef::lfa_v10(), 1000.0, 7)
            .cyl
            .iter()
            .map(|c| c.charge_bias)
            .collect();
        assert_eq!(b, again);
    }

    #[test]
    fn each_cycles_flame_follows_its_mixture_and_turbulence() {
        // the mean charge of an inline-4 at WOT, IVC state taken to TDC inside `flame_ratio`;
        // a richer mixture (φ 1.04, still lean of the 1.13 peak of S_L) burns faster, more
        // diluted burns slower, more turbulence shortens the angles (u′^(−2/3)): one σ each
        let g = EngineDef::inline4().geometry;
        let at = |n: (f64, f64, f64)| {
            cycle_flame(&g, 10.0, (0.9e5, 330.0), 0.95, 5.5e-4, (1.0, 0.07), n).unwrap()
        };
        let mean = at((0.0, 0.0, 0.0));
        assert!(at((1.0, 0.0, 0.0)) < mean, "richer burns faster");
        assert!(at((-1.0, 0.0, 0.0)) > mean, "leaner burns slower");
        assert!(at((0.0, 1.0, 0.0)) > mean, "more residual burns slower");
        // u′ × 1.175: the angles × 1.175^(−2/3) = 0.89807
        assert!((at((0.0, 0.0, 1.0)) / mean - 0.898_07).abs() < 1e-4);
        assert!(at((0.0, 0.0, -1.0)) > mean);
    }

    #[test]
    fn an_empty_charge_is_undiluted() {
        let mut e = Engine::new(EngineDef::single(), 1000.0, 1);
        e.cyl[0].trapped_kg = 0.0;
        e.cyl[0].residual_kg = 1e-5;
        assert_eq!(dilution_of(&e.cyl[0]), 0.0);
        e.cyl[0].trapped_kg = 4e-5;
        assert!((dilution_of(&e.cyl[0]) - 0.25).abs() < 1e-12);
    }

    #[test]
    fn unburnt_residual_does_not_dilute_the_flame() {
        // after a cut cycle the gas left in the cylinder is pumped air, not burnt gas: of a
        // 1e-5 kg residual holding 6e-6 kg of unburnt air, 4e-6 kg dilute a 4e-5 kg charge, 0.1
        let mut e = Engine::new(EngineDef::single(), 1000.0, 1);
        e.cyl[0].trapped_kg = 4e-5;
        e.cyl[0].residual_kg = 1e-5;
        e.cyl[0].residual_unburnt_kg = 6e-6;
        assert!((dilution_of(&e.cyl[0]) - 0.1).abs() < 1e-12);
    }

    /// Indicated work ∮p dV of each cycle of cylinder 1, held at `rpm` with `throttle`, over
    /// `secs`, J.
    fn cycle_works(d: EngineDef, rpm: f64, throttle: f64, secs: f64) -> Vec<f64> {
        let mut e = Engine::new(d, rpm, 1);
        let c = Controls {
            throttle,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        run(&mut e, 1.0, &c);
        let (mut works, mut w) = (Vec::new(), 0.0);
        let (mut prev, mut v_prev) = (e.cyl[0].cycle_deg, e.cyl[0].gas.volume_m3);
        for _ in 0..(secs / DT) as usize {
            e.step(DT, &c);
            let cs = &e.cyl[0];
            w += cs.gas.pressure() * (cs.gas.volume_m3 - v_prev);
            v_prev = cs.gas.volume_m3;
            if cs.cycle_deg < prev {
                works.push(w);
                w = 0.0;
            }
            prev = cs.cycle_deg;
        }
        works.remove(0);
        works
    }

    /// The K20A's cams with VTEC's lobes (motorreviewer, K20A: low 244°/240° and 8.84/8.59 mm,
    /// VTEC 280°/278° and 12.65/12.14 mm, intake/exhaust), each lobe centred where the preset's
    /// single lobe is, VTEC engaging at `engage_rpm`.
    fn k20a_vtec(engage_rpm: f64) -> EngineDef {
        let mut d = EngineDef::honda_k20a();
        let centre = |v: &ValveTiming| v.open_deg + 0.5 * v.duration_deg;
        let (ci, ce) = (centre(&d.intake_valve), centre(&d.exhaust_valve));
        d.intake_valve = ValveTiming {
            open_deg: ci - 122.0,
            duration_deg: 244.0,
            max_lift_m: 0.008_84,
            diameter_m: d.intake_valve.diameter_m,
            high: Some(crate::valve::CamLobe {
                open_deg: ci - 140.0,
                duration_deg: 280.0,
                max_lift_m: 0.012_65,
            }),
        };
        d.exhaust_valve = ValveTiming {
            open_deg: ce - 120.0,
            duration_deg: 240.0,
            max_lift_m: 0.008_59,
            diameter_m: d.exhaust_valve.diameter_m,
            high: Some(crate::valve::CamLobe {
                open_deg: ce - 139.0,
                duration_deg: 278.0,
                max_lift_m: 0.012_14,
            }),
        };
        d.vtec = Some(crate::valve::Vtec {
            engage_rpm,
            hysteresis_rpm: 300.0,
            min_throttle: 0.5,
        });
        d
    }

    #[test]
    fn vtec_trades_low_speed_torque_for_high_speed_breathing() {
        // why VTEC exists: the long, high lobe breathes more at speed and less down low
        // (Heywood §6.3: longer opening and later intake closing move the volumetric
        // efficiency's peak to higher speed). Forced on (engages at 0 rpm) against never
        // (engages beyond any speed)
        let torque = |d: EngineDef, rpm: f64| {
            let mut e = Engine::new(d, rpm, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            run(&mut e, 1.0, &c);
            e.torque_nm()
        };
        let (high, low) = (k20a_vtec(0.0), k20a_vtec(1e9));
        let (h7, l7) = (torque(high.clone(), 7500.0), torque(low.clone(), 7500.0));
        let (h3, l3) = (torque(high, 2500.0), torque(low, 2500.0));
        assert!(h7 > l7 * 1.05, "7500 rpm: high {h7} low {l7}");
        assert!(h3 < l3, "2500 rpm: high {h3} low {l3}");
    }

    #[test]
    fn the_engine_rides_its_high_lobes_once_vtec_engages() {
        let mut e = Engine::new(k20a_vtec(6000.0), 5000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 0.05, &c);
        assert_eq!(e.valves().0.max_lift_m, 0.008_84);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(6500.0),
            ..Controls::default()
        };
        run(&mut e, 0.05, &c);
        assert_eq!(e.valves().0.max_lift_m, 0.012_65);
        // a phaser advances the effective intake timing
        let mut d = k20a_vtec(1e9);
        d.intake_phaser = Some(crate::valve::Phaser {
            range_deg: 50.0,
            rate_deg_s: 1e6,
            map: vec![crate::valve::PhaserPoint {
                rpm: 0.0,
                full_load_deg: 20.0,
                no_load_deg: 20.0,
            }],
        });
        let base = d.intake_valve.open_deg;
        let mut e = Engine::new(d, 3000.0, 1);
        run(&mut e, 0.01, &c);
        assert!((e.valves().0.open_deg - (base - 20.0)).abs() < 1e-9);
    }

    #[test]
    fn cycles_vary_as_measured_engines_do() {
        // Soltau (Proc. IMechE (AD) 1960–61 no. 2, Table 4), wide-open throttle, 2000 rev/min,
        // A/F 14, two compression ratios: mean deviations of the peak pressure 5.54–7.90 % and
        // of the i.m.e.p. 1.18–1.53 % — σ = 1.2533 × those for a normal spread: 6.9–9.9 % and
        // 1.48–1.92 %. Bands around those (ten cycles each in the paper). Near MBT the burn's
        // timing barely moves the work (Heywood §9.4.1), so the work's share is the charge.
        // Every cycle alike (combustion_jitter 0) is 0.
        let peaks = cov(&cycle_peaks(EngineDef::inline4(), 2500.0, 1.0, 4.0));
        assert!(
            (0.05..0.12).contains(&peaks),
            "full load COV of pmax {peaks}"
        );
        let work = cov(&cycle_works(EngineDef::inline4(), 2500.0, 1.0, 4.0));
        assert!(
            (0.012..0.024).contains(&work),
            "full load COV of the work {work}"
        );
        let mut flat = EngineDef::inline4();
        flat.sound.combustion_jitter = 0.0;
        assert!(cov(&cycle_peaks(flat, 2500.0, 1.0, 1.0)) < 1e-4);
    }

    /// The engine's sound with every source off but the valve seats at `mechanical` and the
    /// block at `block`, over 0.2 s at 3000 rpm, on `def`.
    fn seats_only(mut d: EngineDef, mechanical: f64, block: f64) -> Vec<f64> {
        d.sound = crate::sound::SoundDef {
            flow_noise: 0.0,
            intake: 0.0,
            combustion_jitter: 0.0,
            valve_jet: 0.0,
            mechanical,
            block,
            ..crate::sound::SoundDef::default()
        };
        let mut e = Engine::new(d, 3000.0, 1);
        let c = Controls {
            throttle: 0.3,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.2, &c)
    }

    #[test]
    fn the_valve_seats_tick_linearly_through_the_head() {
        // What the seats add (with them on, minus off) is linear in `sound.mechanical`, and a
        // force acting as F/A_bore on the head: the same strikes on a bore of twice the area
        // add half. The first strike pushes the structure positive (a compressive force).
        let d = EngineDef::single();
        let off = seats_only(d.clone(), 0.0, 0.0);
        let one: Vec<f64> = seats_only(d.clone(), 1.0, 0.0)
            .iter()
            .zip(&off)
            .map(|(a, b)| a - b)
            .collect();
        let two: Vec<f64> = seats_only(d.clone(), 2.0, 0.0)
            .iter()
            .zip(&off)
            .map(|(a, b)| a - b)
            .collect();
        let peak = one.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.0, "the seats add nothing");
        for (a, b) in one.iter().zip(&two) {
            assert!((b - 2.0 * a).abs() <= 1e-9 * peak, "not linear: {a} {b}");
        }
        let first = one.iter().find(|x| x.abs() > 1e-6 * peak).copied().unwrap();
        assert!(first > 0.0, "first tick {first}");
        // twice the piston area: the bore × √2 (the strikes come at the same crank angles)
        let mut wide = d.clone();
        wide.geometry.bore_m *= 2f64.sqrt();
        let wide_off = seats_only(wide.clone(), 0.0, 0.0);
        let wide_one: Vec<f64> = seats_only(wide, 1.0, 0.0)
            .iter()
            .zip(&wide_off)
            .map(|(a, b)| a - b)
            .collect();
        let wide_peak = wide_one.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            (wide_peak / peak - 0.5).abs() < 1e-6,
            "{} vs {}",
            wide_peak,
            peak
        );
    }

    #[test]
    fn the_block_sounds_without_the_seats() {
        let d = EngineDef::single();
        let quiet = seats_only(d.clone(), 0.0, 0.0);
        let block = seats_only(d, 0.0, 1.0);
        let diff = block
            .iter()
            .zip(&quiet)
            .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));
        assert!(diff > 0.0, "the block adds nothing on its own");
    }

    #[test]
    fn coasting_down_from_the_redline_stays_a_sound() {
        // Every engine lifted off at its redline and coasting down a quarter of it. A K20A from 8300 rpm (fuel cut, throttle closed): the tail
        // flow keeps swinging through zero and the jets' band-pass follows it every sample. As a
        // direct-form biquad it ran away at ~6860 rpm (2.3 kPa rms, 264 kPa peaks: the "rarísimo"
        // distortion); the output must stay at the engine's own level (its peaks at full throttle)
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::honda_k20a(),
            EngineDef::lfa_v10(),
            EngineDef::flatplane_v8(),
            EngineDef::crossplane_v8(),
        ] {
            let top = d.redline_rpm - 100.0;
            let mut e = Engine::new(d, top, 1);
            let wot = Controls {
                throttle: 1.0,
                hold_rpm: Some(top),
                ..Controls::default()
            };
            let at_wot = run(&mut e, 1.0, &wot)
                .iter()
                .fold(0.0f64, |a, v| a.max(v.abs()));
            let mut peak = 0.0f64;
            for k in 0..40 {
                let c = Controls {
                    throttle: 0.0,
                    hold_rpm: Some(top * (1.0 - 0.25 * k as f64 / 40.0)),
                    ..Controls::default()
                };
                peak = run(&mut e, 0.1, &c)
                    .iter()
                    .fold(peak, |a, v| a.max(v.abs()));
            }
            assert!(
                peak < 3.0 * at_wot,
                "{}: coasting peak {peak:.1} Pa, WOT {at_wot:.1} Pa",
                e.def.name
            );
        }
    }

    #[test]
    fn a_seating_pulse_carries_its_impulse() {
        // a half-sine of peak F over τ carries ∫ = 2Fτ/π: 292 N over 0.15 ms is 27.885 mN·s
        let dt = 1e-6;
        let j: f64 = (0..200)
            .map(|k| seat_pulse(292.0, k as f64 * dt) * dt)
            .sum();
        assert!((j / 0.027885 - 1.0).abs() < 0.01, "{j}");
        assert_eq!(seat_pulse(292.0, -1e-6), 0.0);
        assert_eq!(seat_pulse(292.0, SEAT_PULSE_S + 1e-9), 0.0);
    }

    #[test]
    fn the_valves_tick_harder_as_the_engine_turns_faster() {
        // valve seating heard through the block alone (no combustion noise, no exhaust): each
        // seat strikes with a force ∝ its closing speed ∝ rpm, twice as often at twice the
        // speed, so the ticking's energy grows ~ rpm³: ×8 from 2000 to 4000 rpm
        let ticks = |rpm: f64| {
            let tick = |mech: f64| {
                let mut d = EngineDef::inline4();
                d.sound.block = 0.0;
                d.sound.mechanical = mech;
                let mut e = Engine::new(d, rpm, 1);
                let c = Controls {
                    throttle: 0.5,
                    hold_rpm: Some(rpm),
                    ..Controls::default()
                };
                run(&mut e, 0.3, &c);
                run(&mut e, 0.3, &c)
            };
            let (with, without) = (tick(1.0), tick(0.0));
            with.iter()
                .zip(&without)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
        };
        let (slow, fast) = (ticks(2000.0), ticks(4000.0));
        assert!(slow > 0.0, "no ticking at all");
        let r = fast / slow;
        assert!(
            (4.0..16.0).contains(&r),
            "energy ×{r} from 2000 to 4000 rpm"
        );
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
        // A charge fires on the first step past its `spark_advance(i) + idle trim` degrees
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
            for (i, (cs, was)) in e.cyl.iter().zip(armed).enumerate() {
                let want = e.spark_advance(i) + e.idle_spark_deg;
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

    #[test]
    fn the_lfa_itb_runners_sound_the_same_batched_or_one_by_one() {
        // the ten ITB runners go four at a time through the vector unit: the engine's output
        // must be the scalar one, bit for bit, with either pipe scheme
        let def = EngineDef::lfa_v10();
        assert_eq!(def.intake.model, crate::intake_fv::IntakeModel::Fv);
        assert!(def.intake.individual_throttles);
        for scheme in [crate::fv::Scheme::Weno5Rk3, crate::fv::Scheme::MusclRk2] {
            let run = |scalar: bool| {
                crate::fv::set_scalar_runners(scalar);
                let mut e = Engine::with_fidelity(
                    EngineDef::lfa_v10(),
                    7000.0,
                    1,
                    Fidelity {
                        pipe_scheme: scheme,
                        ..Fidelity::default()
                    },
                );
                let c = Controls {
                    throttle: 1.0,
                    hold_rpm: Some(7000.0),
                    ..Controls::default()
                };
                let out: Vec<u64> = (0..(0.3 / DT) as usize)
                    .map(|_| e.step(DT, &c).to_bits())
                    .collect();
                crate::fv::set_scalar_runners(false);
                out
            };
            let batched = run(false);
            let one_by_one = run(true);
            assert!(batched.iter().any(|&b| f64::from_bits(b) != 0.0));
            assert_eq!(batched, one_by_one, "{scheme:?}");
        }
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
            ..Fidelity::default()
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

    /// Overrun fuel cut until injection reactivates near idle, then the idle control takes
    /// over (Bosch, Gasoline Engine Management): back at idle within a few seconds and staying
    /// there, not hung at the cut-off speed. One test per engine, so they run side by side.
    fn settles_at_its_idle_when_released(d: EngineDef) {
        let target = d.idle_rpm;
        // 10 s: with friction by components (half the old whole-engine fit, which held
        // pumping twice) the single's heavy estimated flywheel (0.12 kg·m²) coasts down slower
        let (_, trace) = rev_and_release(d.clone(), 0.8 * d.redline_rpm, 10.0);
        let tail = &trace[(8.0 / DT) as usize..];
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

    #[test]
    fn released_from_high_revs_the_single_settles_at_its_idle() {
        settles_at_its_idle_when_released(EngineDef::single());
    }

    #[test]
    fn released_from_high_revs_the_inline4_settles_at_its_idle() {
        settles_at_its_idle_when_released(EngineDef::inline4());
    }

    #[test]
    fn released_from_high_revs_the_lfa_settles_at_its_idle() {
        settles_at_its_idle_when_released(EngineDef::lfa_v10());
    }

    #[test]
    fn released_from_high_revs_the_flatplane_settles_at_its_idle() {
        settles_at_its_idle_when_released(EngineDef::flatplane_v8());
    }

    #[test]
    fn released_from_high_revs_the_crossplane_settles_at_its_idle() {
        settles_at_its_idle_when_released(EngineDef::crossplane_v8());
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
    fn friction_is_the_component_model_at_the_manifold_pressure() {
        // K20A at 8000 rpm, manifold at 1 atm: 2.144 bar by hand (friction::tests); its
        // 86² · π/4 · 86 · 4 = 1.99825e-3 m³ gives T = 2.144e5 · 1.99825e-3 / 4π = 34.10 N·m
        let e = Engine::new(EngineDef::honda_k20a(), 8000.0, 1);
        assert_relative_eq!(e.plenum_pressure_pa(), ATM_PA, max_relative = 1e-9);
        assert_relative_eq!(e.friction_torque(), 34.10, max_relative = 2e-3);
        // the old whole-engine fit gave 5.37 bar there: 85.4 N·m
        assert_eq!(
            Engine::new(EngineDef::single(), 0.0, 1).friction_torque(),
            0.0
        );
    }

    #[test]
    fn the_cycle_converts_fuel_like_a_real_engine_at_full_load() {
        // Net indicated fuel conversion efficiency at WOT: the fuel–air cycle's ≈ 47 % at
        // 11.5:1 (thermo::tests) less heat loss, finite burn and gas exchange — real engines
        // reach ≈ 0.8 of it, 35–40 % (Heywood, ICE Fundamentals, 2nd ed., ch. 5). With cold-air
        // properties (γ = 1.4) the model made 45 %. The cycle alone: fixed cams, so no fresh air
        // blows through the overlap (see `vtec_overlap_blows_some_fresh_air_through`).
        let mut d = EngineDef::honda_k20a();
        (d.vtec, d.intake_phaser, d.exhaust_phaser) = (None, None, None);
        let rpm = 6500.0;
        let mut e = Engine::new(d.clone(), rpm, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        run(&mut e, 1.5, &c);
        let (m0, f0, inj0) = (
            e.inducted_kg(),
            e.fuel_energy_j,
            e.fuel_ledger.into_cylinders,
        );
        let n = (0.5 / DT) as usize;
        let mut work = 0.0;
        for _ in 0..n {
            e.step(DT, &c);
            work += (e.torque_nm() + e.friction_torque()) * rpm_to_rad_s(rpm) * DT;
        }
        let fuel = (e.inducted_kg() - m0) / (1.0 + AFR_STOICH) * LHV_GASOLINE;
        let eta = work / fuel;
        assert!(
            (0.33..0.41).contains(&eta),
            "net indicated efficiency {eta:.3}"
        );
        // and what burnt is what was injected (the ledger agrees with the fuel)
        let injected = (e.fuel_ledger.into_cylinders - inj0) * LHV_GASOLINE;
        assert!(((e.fuel_energy_j - f0) / (injected * COMBUSTION_EFFICIENCY) - 1.0).abs() < 0.05);
    }

    #[test]
    fn vtec_overlap_blows_some_fresh_air_through() {
        // On its VTEC lobe with the cam advanced, the K20A's valves overlap widely at speed: part
        // of the air drawn in passes straight to the exhaust (scavenging), so the fuel the ECU
        // meters on the trapped charge is less than the inducted air's stoichiometric share
        // (measured 0.93 at 6500 rpm); with fixed cams it is the share itself
        let share = |vvt: bool| {
            let mut d = EngineDef::honda_k20a();
            if !vvt {
                (d.vtec, d.intake_phaser, d.exhaust_phaser) = (None, None, None);
            }
            let mut e = Engine::new(d, 6500.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(6500.0),
                ..Controls::default()
            };
            run(&mut e, 1.5, &c);
            let (m0, inj0) = (e.inducted_kg(), e.fuel_ledger.into_cylinders);
            run(&mut e, 0.5, &c);
            (e.fuel_ledger.into_cylinders - inj0) * AFR_STOICH / (e.inducted_kg() - m0)
        };
        let (with, without) = (share(true), share(false));
        assert!(with < 0.97 && with > 0.85, "with VTEC/VTC {with}");
        assert!(without > with, "fixed cams {without}");
    }

    #[test]
    fn gas_coming_in_mixes_its_composition_by_mass() {
        // a cylinder holding 1 kg, half of it products and all its fresh part fuelled, takes
        // 0.6 kg of products-free gas that is ⅓ fuelled: burned (0.5·1 + 0)/(1.6) = 0.3125;
        // fresh 0.5 kg (fuelled 1) + 0.6 kg (fuelled ⅓): fuelled (0.5 + 0.2)/1.1 = 0.636 36
        // (hand); the mass and the energy carried add
        let mut e = Engine::new(EngineDef::inline4(), 1000.0, 1);
        let cs = &mut e.cyl[0];
        cs.gas.mass_kg = 1.0;
        cs.u_j = 5.0e5;
        cs.comp = Composition {
            burned: 0.5,
            fuelled: 1.0,
        };
        let c_in = Composition {
            burned: 0.0,
            fuelled: 1.0 / 3.0,
        };
        cylinder_in(cs, 0.6, 3.0e5, c_in);
        assert_relative_eq!(cs.comp.burned, 0.3125, max_relative = 1e-12);
        assert_relative_eq!(cs.comp.fuelled, 0.7 / 1.1, max_relative = 1e-12);
        assert_relative_eq!(cs.gas.mass_kg, 1.6, max_relative = 1e-12);
        assert_relative_eq!(cs.u_j, 5.0e5 + 0.6 * 3.0e5, max_relative = 1e-12);
        assert!(cs.mix_stale);
        // burned gas coming in counts: 1.6 kg at 0.3125 take 0.4 kg at 0.75 burned →
        // (0.5 + 0.3)/2.0 = 0.4
        cylinder_in(
            cs,
            0.4,
            3.0e5,
            Composition {
                burned: 0.75,
                fuelled: 0.0,
            },
        );
        assert_relative_eq!(cs.comp.burned, 0.4, max_relative = 1e-12);
        // a cylinder of products alone (no fresh part) takes fresh charge: its fresh part is
        // that charge's, fuelled 0.8
        cs.comp = Composition {
            burned: 1.0,
            fuelled: 0.0,
        };
        cylinder_in(
            cs,
            0.1,
            3.0e5,
            Composition {
                burned: 0.0,
                fuelled: 0.8,
            },
        );
        assert_relative_eq!(cs.comp.fuelled, 0.8, max_relative = 1e-12);
    }

    #[test]
    fn gas_flowing_in_brings_its_own_mixtures_enthalpy() {
        // 10 g of half-products, fuelled charge at 800 K (c_p,air·T = 803 600 J/kg in the flow
        // code's convention) enter: the cylinder gains 10 g × h of that mixture at 800 K
        let mut e = Engine::new(EngineDef::inline4(), 1000.0, 1);
        let th = e.thermo;
        let cs = &mut e.cyl[0];
        let c_in = Composition {
            burned: 0.5,
            fuelled: 1.0,
        };
        let u0 = cs.u_j;
        let f = Flow {
            dm: -0.01,
            h: crate::gas::CP * 800.0,
        };
        cylinder_flow(cs, &th, f, c_in);
        assert_relative_eq!(
            cs.u_j - u0,
            0.01 * th.mixture(c_in).h(800.0),
            max_relative = 1e-12
        );
    }

    #[test]
    fn friction_fades_out_at_standstill() {
        // T = FMEP·Vd/4π·tanh(ω/5): at 20 rpm, ω = 2.0944 rad/s and tanh(0.41888) = 0.39599
        let d = EngineDef::inline4();
        let e = Engine::new(d.clone(), 20.0, 1);
        let at = d.friction.at_speed(&d.friction_engine(), 20.0);
        let full = at.fmep(e.plenum_pressure_pa()) * d.displacement_m3() / (4.0 * PI);
        assert_relative_eq!(e.friction_torque() / full, 0.39599, max_relative = 1e-4);
    }

    #[test]
    fn the_primary_holds_products_under_power_and_air_after_a_fuel_cut() {
        let mut e = Engine::new(EngineDef::inline4(), 4400.0, 1);
        let wot = Controls {
            throttle: 1.0,
            hold_rpm: Some(4400.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &wot);
        assert!(e.cyl[0].port_xb > 0.9, "under power {}", e.cyl[0].port_xb);
        let cut = Controls {
            hold_rpm: Some(4400.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &cut);
        assert!(e.cyl[0].port_xb < 0.1, "after the cut {}", e.cyl[0].port_xb);
    }

    #[test]
    fn a_flame_turns_the_fresh_charge_into_products_as_it_burns() {
        // with the valves shut, the burned share is the residual plus the fresh share times
        // what the flame has released: x_b = (1 − f) + f·x
        let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        let mut checked = 0;
        for _ in 0..(0.1 / DT) as usize {
            e.step(DT, &c);
            let cs = &e.cyl[0];
            if let Some(b) = &cs.burn
                && b.released > 0.05
                && b.released < 0.95
            {
                let want = (1.0 - cs.fresh_share) + cs.fresh_share * b.released;
                assert_relative_eq!(cs.comp.burned, want, max_relative = 1e-9);
                checked += 1;
            }
        }
        assert!(checked > 10, "{checked} burning steps seen");
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
        // Helmholtz-tuned to the 6800 rpm torque peak (Engelman; see the preset): 0.325 m
        assert_eq!(d.intake.runner_length_m, 0.325);
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
    fn the_k20a_is_hondas() {
        // Honda K20A: 4 · π/4 · 0.086² · 0.086 = 1.9983e-3 m³ (1998 cc), CR 11.5, 8400 rpm,
        // firing 1-3-4-2, valves 2 × 35 mm intake / 2 × 30 mm exhaust as one equivalent each
        let d = EngineDef::honda_k20a();
        assert!(d.name.contains("K20A"));
        assert_relative_eq!(d.displacement_m3(), 1.9983e-3, max_relative = 1e-4);
        assert_eq!(d.geometry.compression_ratio, 11.5);
        assert_eq!(d.geometry.rod_m, 0.139);
        assert_eq!(d.redline_rpm, 8400.0);
        assert_eq!(d.firing_order, [1, 3, 4, 2]);
        assert!((d.intake_valve.diameter_m - 0.035 * 2f64.sqrt()).abs() < 2e-4);
        assert!((d.exhaust_valve.diameter_m - 0.030 * 2f64.sqrt()).abs() < 2e-4);
        assert_eq!(d.intake.model, IntakeModel::Fv);
    }

    #[test]
    fn the_lfa_fires_each_bank_evenly() {
        // A 72° V10 firing evenly (Toyota): 1-6-5-10-2-7-3-8-4-9 alternates the banks when
        // cylinders 1–5 sit on one and 6–10 on the other (the numbering this order belongs to,
        // as on Audi/Lamborghini V10s), so each bank's 5-into-1 collector gets a pulse every
        // 720/5 = 144°.
        let d = EngineDef::lfa_v10();
        assert_eq!(bank_intervals(&d), vec![vec![144.0; 5]; 2]);
    }

    #[test]
    fn a_v10_grown_to_twelve_stays_one_even_firing_v() {
        // two pistons a crankpin: a V fires evenly at 720/n between its banks — 60° for twelve.
        // Cylinders 1–6 and 7–12 on the banks, 1-7-5-11-3-9-6-12-2-8-4-10 alternating them,
        // each bank's collector getting a pulse every 120°, all twelve in a collector
        let mut d = EngineDef::lfa_v10();
        d.set_cylinders(12);
        assert_eq!(d.cylinders, 12);
        assert_eq!(d.layout.bank_angle_deg, 60.0);
        assert_eq!(
            d.layout.banks,
            [vec![1, 2, 3, 4, 5, 6], vec![7, 8, 9, 10, 11, 12]]
        );
        assert_eq!(d.exhaust.collectors, d.layout.banks);
        assert_eq!(bank_intervals(&d), vec![vec![120.0; 6]; 2]);
        // an inline engine stays inline, one collector for all
        let mut i = EngineDef::inline4();
        i.set_cylinders(6);
        assert!(i.layout.banks.is_empty());
        assert_eq!(i.firing_order, [1, 5, 3, 6, 2, 4]);
        assert!(i.exhaust.collectors.is_empty());
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
    fn the_trumpets_draw_what_the_engine_breathes() {
        // 1D intake: the mouth's volume flow is what the trumpets take in, −ṁ/ρ₀ with
        // ρ₀ = 101325/(287·300) = 1.1768 kg/m³; its mean is the inducted mass rate's
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
        let want = -mdot / 1.1768;
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
            block: 0.0,
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
            d.sound.ground_heights(),
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
            want.push(l.radiate(t));
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
    fn the_ecu_times_the_spark_for_the_flame_it_has() {
        // MBT: half-burned 8° after top centre. Configured 28° at the reference flame; a flame
        // 1.5× as long needs (28 + 8)·1.5 − 8 = 46°, half as long (28 + 8)·0.5 − 8 = 10°
        let d = EngineDef::flatplane_v8();
        assert_relative_eq!(d.spark_advance_for(1.0), 28.0);
        assert_relative_eq!(d.spark_advance_for(1.5), 46.0);
        assert_relative_eq!(d.spark_advance_for(0.5), 10.0);
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

    /// Cranked from where it stalled (`seed` picks the position), the engine starts.
    fn restarts_from_stall_position(def: EngineDef, k: u64) {
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

    // Seeds 0, 7, 9 stalled the I4 against compression (found by sweeping 60 positions).
    #[test]
    fn the_inline4_restarts_from_stall_position_0() {
        restarts_from_stall_position(EngineDef::inline4(), 0);
    }

    #[test]
    fn the_inline4_restarts_from_stall_position_7() {
        restarts_from_stall_position(EngineDef::inline4(), 7);
    }

    #[test]
    fn the_single_restarts_from_stall_position_38() {
        restarts_from_stall_position(EngineDef::single(), 38);
    }

    #[test]
    fn the_lfa_restarts_from_stall_position_0() {
        restarts_from_stall_position(EngineDef::lfa_v10(), 0);
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
            valve_jet: 0.0,
            ..crate::sound::SoundDef::default()
        };
        // the flow textures (the valvetrain's clicks do not depend on load)
        let on = crate::sound::SoundDef {
            mechanical: 0.0,
            ..crate::sound::SoundDef::default()
        };
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

    /// The LFA held at 8000 rpm, at full throttle for a second and then lifted off for
    /// `lift_s`: (combustions, afterburn heat J) over the last 0.5 s at WOT, then over each
    /// 0.25 s of the lift-off.
    fn lift_off(lift_s: f64) -> ((u64, f64), Vec<(u64, f64)>) {
        let mut e = Engine::new(EngineDef::lfa_v10(), 8000.0, 1);
        let hold = |throttle| Controls {
            throttle,
            hold_rpm: Some(8000.0),
            ..Controls::default()
        };
        run(&mut e, 1.0, &hold(1.0));
        let window = |e: &mut Engine, s: f64, t: f64| {
            let (n, q) = (e.combustions(), e.afterburn_j());
            run(e, s, &hold(t));
            (e.combustions() - n, e.afterburn_j() - q)
        };
        let wot = window(&mut e, 0.5, 1.0);
        let lift = (0..(lift_s / 0.25) as usize)
            .map(|_| window(&mut e, 0.25, 0.0))
            .collect();
        (wot, lift)
    }

    #[test]
    fn a_lift_off_fires_on_the_port_film_for_a_while() {
        // Aquino's x–τ (Heywood §7.6.3): the fuel cut stops the injection, not the fuel. The film
        // the port walls hold gives back e·film each intake, so the cylinders keep firing for a
        // while, leaner and leaner, and then stop. A cut that stopped the fuel at once fired none.
        let (_, lift) = lift_off(3.0);
        assert!(
            lift[0].0 > 20,
            "the film fires nothing after the lift-off: {lift:?}"
        );
        assert!(
            lift[lift.len() - 1].0 < lift[0].0 / 4,
            "the film never runs dry: {lift:?}"
        );
    }

    #[test]
    fn a_lift_off_afterburns_in_the_exhaust_and_steady_wot_does_not() {
        // A flame too slow to finish before the exhaust valve opens, or a charge too lean to
        // take, sends unburnt mixture into a primary still above 873 K, where it burns (Heywood
        // §11.6.3): the pops of a lift-off. At steady full throttle every flame finishes.
        let (wot, lift) = lift_off(1.0);
        assert!(wot.1 < 1.0, "afterburn at steady WOT: {} J", wot.1);
        let total: f64 = lift.iter().map(|w| w.1).sum();
        assert!(total > 10.0, "no afterburn on the lift-off: {lift:?}");
    }

    #[test]
    fn heat_in_a_primary_warms_and_expands_its_gas() {
        // 100 J in 1e-3 kg over 1e-4 s: 100/(1e-3·1004.5) = 99.552 K; c_p·T_ref = c²/(γ − 1)
        // = 600²/0.4 = 9e5, so the source is 1e6/9e5 = 1.11111 (hand)
        let (rise, source) = primary_heat(100.0, 1e-3, 1e-4);
        assert!((rise - 99.552).abs() < 1e-3, "{rise}");
        assert!((source - 1.111_111).abs() < 1e-6, "{source}");
    }

    #[test]
    fn a_flame_releases_the_fuel_its_air_can_burn() {
        // lean 1e-5 kg × 44e6 × 0.96 × 1.02 = 430.848 J; rich 2e-5 kg in 1.47e-4 kg air burns
        // 1e-5: 422.4 J (hand)
        assert!((burn_energy(1e-5, 3e-4, 1.02) - 430.848).abs() < 1e-9);
        assert!((burn_energy(2e-5, 1.47e-4, 1.0) - 422.4).abs() < 1e-9);
    }

    /// Fuel still held anywhere in `e`, kg: the charges awaiting their spark, the flames'
    /// unreleased share, the gas leaving the cylinders and the primaries.
    fn fuel_held(e: &Engine) -> f64 {
        e.cyl
            .iter()
            .map(|c| {
                let flame = c.burn.as_ref().map_or(0.0, |b| b.unburnt().fuel_kg);
                c.fuel_kg + flame + c.slip.fuel_kg + c.port_mix.fuel_kg()
            })
            .sum()
    }

    #[test]
    fn every_kilogram_of_fuel_is_accounted_for() {
        // Mass balance through a pull, a lift-off and the limiter: what the ports delivered
        // is burnt in the cylinders, burnt in the primaries, out of the primaries, or still held
        for d in [EngineDef::lfa_v10(), EngineDef::inline4()] {
            let top = d.redline_rpm;
            let mut e = Engine::new(d, 3000.0, 1);
            for (t, s) in [(1.0, 1.5), (0.0, 1.5), (1.0, 1.0)] {
                let c = Controls {
                    throttle: t,
                    ..Controls::default()
                };
                run(&mut e, s, &c);
            }
            assert!(
                e.rpm() > 0.9 * top,
                "never reached the limiter: {}",
                e.rpm()
            );
            let l = e.fuel_ledger;
            let out = l.burnt + l.afterburnt + l.exited + fuel_held(&e);
            assert!(
                l.into_cylinders > 0.0 && l.afterburnt > 0.0 && l.exited > 0.0,
                "{l:?}"
            );
            assert!(
                (out / l.into_cylinders - 1.0).abs() < 1e-9,
                "{}: {l:?}, held {}",
                e.def.name,
                fuel_held(&e)
            );
        }
    }

    #[test]
    fn a_film_of_no_time_constant_holds_no_fuel() {
        // τ = 0 disables the film: a cut delivers nothing at all
        let mut d = EngineDef::lfa_v10();
        d.intake.film_tau_s = 0.0;
        let mut e = Engine::new(d, 8000.0, 1);
        let hold = |throttle| Controls {
            throttle,
            hold_rpm: Some(8000.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &hold(1.0));
        run(&mut e, 0.05, &hold(0.0));
        let fed = e.fuel_ledger.into_cylinders;
        run(&mut e, 0.25, &hold(0.0));
        assert_eq!(e.fuel_ledger.into_cylinders, fed);
        assert!(e.cyl.iter().all(|c| c.film.film_kg == 0.0));
    }

    #[test]
    fn road_presets_carry_close_coupled_catalysts() {
        // every road car has catalysts: one per branch right after its collector, of about the
        // branch's swept volume (the sizing rule of `exhaust::road_exhaust`), 0.13 m long. The
        // LFA: 4.805 L over two banks, Ø = √(4·2.4025e-3/(π·0.13)) = 0.1534 m (hand)
        let lfa = EngineDef::lfa_v10();
        match lfa.exhaust.after_collector.first() {
            Some(crate::exhaust::Segment::Catalyst {
                length_m,
                diameter_m,
                ..
            }) => {
                assert_eq!(*length_m, 0.13);
                assert!((diameter_m - 0.1534).abs() < 5e-4, "{diameter_m}");
            }
            other => panic!("the LFA has no catalyst first: {other:?}"),
        }
        for d in [
            EngineDef::inline4(),
            EngineDef::honda_k20a(),
            EngineDef::flatplane_v8(),
            EngineDef::crossplane_v8(),
        ] {
            assert!(
                matches!(
                    d.exhaust.after_collector.first(),
                    Some(crate::exhaust::Segment::Catalyst { .. })
                ),
                "{}",
                d.name
            );
        }
    }

    #[test]
    fn a_mixture_whose_flame_cannot_propagate_misfires() {
        // Metghalchi–Keck's S_L0 = 0.2758 − 0.7834(φ − 1.13)² falls to zero at φ = 0.537 and
        // 1.723 (hand): there the flame has no speed and its angles no finite ratio
        let g = EngineDef::lfa_v10().geometry;
        let at = |phi: f64| flame_ratio(&g, 15.0, (1e5, 350.0), 1.0, 4e-4, (phi, 0.08));
        assert!(at(1.0).is_finite());
        assert!(at(0.6).is_finite());
        assert!(!at(0.5).is_finite(), "{}", at(0.5));
        assert!(!at(1.8).is_finite(), "{}", at(1.8));
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
    fn the_valve_jets_hiss_through_the_fv_exhaust_too() {
        // The valves' mixing noise rides down the primaries in both exhaust models. With every
        // other random source off, an engine held at speed repeats every cycle; the jets' hiss
        // is what does not: the energy of x(t) − x(t + one cycle) must rise with them on the FV
        // exhaust as it does on the waveguide.
        let aperiodic = |model: ExhaustModel, jet: f64| {
            let mut d = EngineDef::inline4();
            d.exhaust.model = model;
            let s = &mut d.sound;
            (s.flow_noise, s.block, s.mechanical, s.combustion_jitter) = (0.0, 0.0, 0.0, 0.0);
            s.valve_jet = jet;
            let rpm = 6000.0;
            let mut e = Engine::new(d, rpm, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let x = run(&mut e, 0.2, &c);
            let cycle = (120.0 / rpm / DT).round() as usize;
            x.windows(cycle + 1)
                .map(|w| (w[cycle] - w[0]).powi(2))
                .sum::<f64>()
        };
        // and at the same level: the same jets through the same pipes, within what the ports'
        // gas states leave apart (FV computes them, the waveguide estimates a 0D primary
        // temperature; measured −3.6 dB)
        let (wg, fv) = (
            aperiodic(ExhaustModel::Waveguide, 1.0),
            aperiodic(ExhaustModel::Fv, 1.0),
        );
        let apart = 10.0 * (fv / wg).log10();
        assert!(
            apart.abs() < 6.0,
            "FV hiss {apart:+.1} dB from the waveguide's"
        );
        for model in [ExhaustModel::Waveguide, ExhaustModel::Fv] {
            let gain = 10.0 * (aperiodic(model, 1.0) / aperiodic(model, 0.0)).log10();
            assert!(
                gain > 6.0,
                "{model:?}: the valve jets add {gain:.1} dB of hiss"
            );
        }
    }

    #[test]
    fn a_motored_fv_exhaust_repeats_every_cycle() {
        // Held speed, fuel cut, no random source: the engine is a periodic system, so what it
        // radiates must repeat from one cycle to the next. Before the open end's inflow was
        // isentropic from the atmosphere, the tail's flow reversals (~600 a second on the LFA's
        // overrun) left it 0.49–0.81 correlated and 10 dB over full power.
        for d in [EngineDef::inline4(), EngineDef::lfa_v10()] {
            let mut d = fv(d);
            let s = &mut d.sound;
            (s.flow_noise, s.block, s.mechanical, s.intake) = (0.0, 0.0, 0.0, 0.0);
            (s.valve_jet, s.combustion_jitter) = (0.0, 0.0);
            let rpm = 0.85 * d.redline_rpm;
            let mut e = Engine::new(d, rpm, 1);
            let c = Controls {
                throttle: 0.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            run(&mut e, 1.0, &c);
            let x = run(&mut e, 0.4, &c);
            let cycle = 120.0 / rpm / DT;
            let at = |k: usize| &x[(k as f64 * cycle) as usize..][..cycle as usize];
            let n = (x.len() as f64 / cycle) as usize - 1;
            let corr = |a: &[f64], b: &[f64]| {
                let m = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
                let (ma, mb) = (m(a), m(b));
                let cov: f64 = a.iter().zip(b).map(|(x, y)| (x - ma) * (y - mb)).sum();
                let va: f64 = a.iter().map(|x| (x - ma).powi(2)).sum();
                let vb: f64 = b.iter().map(|y| (y - mb).powi(2)).sum();
                cov / (va * vb).sqrt()
            };
            let mean = (0..n - 1).map(|k| corr(at(k), at(k + 1))).sum::<f64>() / (n - 1) as f64;
            assert!(
                mean > 0.95,
                "{}: cycle-to-cycle correlation {mean:.3}",
                e.def.name
            );
        }
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
    fn the_jets_blow_at_the_mean_flow_the_engine_pushes_out() {
        // The tail's mean exit velocity is mass continuity, ṁ/(ρA): a 2.0 L at 5000 rpm WOT
        // with η_v ≈ 0.9 breathes 2.0e-3·0.9·1.18·5000/120 = 0.0885 kg/s of air, + fuel/14.7
        // = 0.094 kg/s, out through 50 mm (A = 1.963e-3 m²) at 622 K (ρ = 0.567): 84 m/s. The
        // waveguide's own DC is not a flow (a lossy or reflecting line does not conserve it).
        let mut e = Engine::new(EngineDef::inline4(), 5000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &c);
        let n = (0.2 / DT) as usize;
        let mut u = 0.0;
        for _ in 0..n {
            e.step(DT, &c);
            u += e.tail_exit_velocity()[0] / n as f64;
        }
        assert!((70.0..100.0).contains(&u), "mean exit velocity {u} m/s");
    }

    /// Brake torque and the tail's exit velocities of an inline-4 at 5000 rpm WOT with a tail
    /// of `d` metres.
    fn with_tail(d: f64) -> (f64, Vec<f64>) {
        let mut def = EngineDef::inline4();
        *def.exhaust.tail_mut().1 = d;
        let mut e = Engine::new(def, 5000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 0.6, &c);
        let mut u = Vec::new();
        for _ in 0..(0.2 / DT) as usize {
            e.step(DT, &c);
            u.push(e.tail_exit_velocity()[0]);
        }
        (e.torque_nm(), u)
    }

    #[test]
    fn a_choked_tail_pipe_holds_the_engine_back() {
        // 0.094 kg/s through 15 mm would leave at ~900 m/s: past sonic (c = 500 m/s in the
        // 622 K gas), so the exit chokes — the pressure behind it rises until the sonic flow
        // passes it (p* = 0.528 p₀): ≈ 2.9 bar of backpressure, which costs torque. The exit
        // never blows faster than sound; a 50 mm tail barely notices (≈ 2 kPa).
        let (t50, u50) = with_tail(0.050);
        let (t15, u15) = with_tail(0.015);
        let fastest = u15.iter().cloned().fold(f64::MIN, f64::max);
        assert!(
            fastest <= 1.05 * crate::exhaust::C_TAIL,
            "exit at {fastest} m/s"
        );
        assert!(t15 < 0.8 * t50, "torque {t15} with a 15 mm tail vs {t50}");
        let mean50 = u50.iter().sum::<f64>() / u50.len() as f64;
        assert!(
            (70.0..100.0).contains(&mean50),
            "50 mm mean exit {mean50} m/s"
        );
    }

    #[test]
    fn the_inline4_has_its_own_flywheel() {
        // a car's dual-mass flywheel and clutch, heavier than the single's 0.12 kg·m²
        assert_eq!(EngineDef::inline4().flywheel_inertia_kgm2, 0.15);
    }

    #[test]
    fn the_mean_exhaust_flows_are_the_valves_cycle_averages() {
        // mass conservation over a cycle: what leaves each tail on average is what its cylinders'
        // exhaust valves pass, averaged over exactly whole cycles (5000 rpm: 24 ms per cycle).
        // The running means follow one cycle's window, so they sit within a few percent of it.
        let mut e = Engine::new(EngineDef::inline4(), 5000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 0.4, &c);
        let cycle = (0.024 / DT).round() as usize;
        let (mut tail, mut cyl0, mut track) = (0.0, 0.0, Vec::new());
        for _ in 0..4 * cycle {
            e.step(DT, &c);
            tail += e.cyl.iter().map(|cs| cs.port_flow).sum::<f64>();
            cyl0 += e.cyl[0].port_flow;
            track.push((e.tail_mean_flow[0], e.cyl_mean_flow[0]));
        }
        let (tail, cyl0) = (tail / (4 * cycle) as f64, cyl0 / (4 * cycle) as f64);
        // four cylinders' pulses overlap: the tail's running mean barely ripples
        for &(t, _) in &track {
            assert!(
                (t / tail - 1.0).abs() < 0.05,
                "tail mean {t} vs {tail} kg/s"
            );
        }
        // one cylinder pulses once a cycle: its running mean ripples, but over whole cycles it
        // averages to the valve's flow
        let c0 = track.iter().map(|p| p.1).sum::<f64>() / track.len() as f64;
        assert!(
            (c0 / cyl0 - 1.0).abs() < 0.01,
            "cylinder mean {c0} vs {cyl0} kg/s"
        );
        let hi = track.iter().map(|p| p.1).fold(f64::MIN, f64::max);
        let lo = track.iter().map(|p| p.1).fold(f64::MAX, f64::min);
        // a one-cycle time constant (24 ms): between pulses the mean decays by at most e^(−1)
        // per cycle, so its ripple stays under a factor e (measured 2.05; at 0.57 cycle, 3.57)
        assert!(hi / lo < std::f64::consts::E, "ripple {hi}/{lo}");
    }

    #[test]
    fn the_tail_jet_swings_with_the_tail_wave_around_its_mean() {
        // u = ū + (s − s̄)/ρc: over whole cycles the swing averages out (the mean exit velocity
        // is the continuity one, Mach ū/c), and it follows the outgoing wave s = ρc·u with slope
        // 1/ρc, ρc = 1.4·101325/500 = 283.71 kg/(m²·s) in the tail's reference gas
        let mut e = Engine::new(EngineDef::inline4(), 5000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 0.4, &c);
        let n = 4 * (0.024 / DT).round() as usize;
        let (mut swing, mut wave, mut us, mut mach) = (Vec::new(), Vec::new(), 0.0, 0.0);
        for _ in 0..n {
            e.step(DT, &c);
            let s = e.net.as_ref().unwrap().last_tails()[0];
            // ū this step is the continuity one, c·Mach
            swing.push(e.tail_u[0] - 500.0 * e.tail_mach[0]);
            wave.push(s);
            us += e.tail_u[0];
            mach += e.tail_mach[0];
        }
        let (us, u_bar) = (us / n as f64, 500.0 * mach / n as f64);
        // the wave against its own average over the whole cycles
        let s_bar = wave.iter().sum::<f64>() / n as f64;
        for s in &mut wave {
            *s -= s_bar;
        }
        assert!(
            (us / u_bar - 1.0).abs() < 0.03,
            "mean exit {us} vs ū {u_bar} m/s"
        );
        let cov: f64 = swing.iter().zip(&wave).map(|(u, s)| u * s).sum();
        let var: f64 = wave.iter().map(|s| s * s).sum();
        let slope = cov / var;
        assert!(
            (slope * 283.71 - 1.0).abs() < 0.04,
            "slope {slope} vs 1/283.71"
        );
    }

    #[test]
    fn the_engine_hands_its_mean_flows_to_the_exhaust_pipes() {
        // the pipes' friction follows the cycle-mean flows: the net holds what it was last
        // given, refreshed every 64 steps — the cylinders' and the tails' running means
        let mut e = Engine::new(EngineDef::inline4(), 5000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        while e.friction_age != 0 {
            e.step(DT, &c);
        }
        // (handed over before this step's means moved on: within a step's change)
        let (cyl, tails) = e.net.as_ref().unwrap().mean_flows();
        let close = |a: &[f64], b: &[f64]| {
            a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x / y - 1.0).abs() < 1e-3)
        };
        assert!(
            close(cyl, &e.cyl_mean_flow),
            "{cyl:?} vs {:?}",
            e.cyl_mean_flow
        );
        assert!(
            close(tails, &e.tail_mean_flow),
            "{tails:?} vs {:?}",
            e.tail_mean_flow
        );
        // a 2.0 L at 5000 rpm WOT breathes ~0.09 kg/s through its one tail
        assert!((0.05..0.15).contains(&tails[0]), "{tails:?}");
    }

    #[test]
    fn an_exhaust_run_in_blocks_elsewhere_gives_the_same_samples_late() {
        // the net behind a block link (here run in order, as a thread would) against the net in
        // the engine: identical samples to the bit, each one `2·block` steps later; flushing gives
        // the rest. A blip midway moves the mean flows the pipes' friction follows; the run ends
        // in the middle of a block.
        for def in [EngineDef::lfa_v10(), EngineDef::inline4()] {
            let mut a = Engine::new(def.clone(), 3000.0, 1);
            let mut b = Engine::new(def.clone(), 3000.0, 1);
            let n = (0.15 / DT) as usize + 7;
            let c = |k: usize| Controls {
                throttle: if (n / 3..n / 2).contains(&k) {
                    1.0
                } else {
                    0.2
                },
                ..Controls::default()
            };
            // a few steps in place first (the net is built on the first step)
            for k in 0..10 {
                assert_eq!(a.step(DT, &c(k)), b.step(DT, &c(k)));
            }
            let mut block = 0;
            assert!(b.detach_exhaust(|net, max| {
                block = max.min(16);
                Box::new(crate::exhaust::LocalLink::new(net, block))
            }));
            assert!(block >= 8, "{}: block {block}", def.name);
            let lag = 2 * block;
            let mut want = Vec::new();
            let mut got = Vec::new();
            for k in 10..n {
                want.push(a.step(DT, &c(k)));
                got.push(b.step(DT, &c(k)));
            }
            let flushed = b.flush_exhaust(DT, &mut got);
            assert_eq!(flushed, lag, "{}", def.name);
            assert_eq!(got.len(), want.len() + lag);
            for (k, (w, g)) in want.iter().zip(&got[lag..]).enumerate() {
                assert_eq!(w.to_bits(), g.to_bits(), "{} step {k}", def.name);
            }
            assert!(got[..lag].iter().all(|x| *x == 0.0));
            assert_eq!(b.flush_exhaust(DT, &mut got), 0);
            assert_eq!(a.rpm(), b.rpm());
        }
    }

    #[test]
    fn runners_stepped_elsewhere_give_the_same_samples() {
        // half the LFA's ITB runners behind a link (stepped in order here, as a thread would):
        // identical samples step by step, through a blip; then back in place, still identical
        let def = EngineDef::lfa_v10();
        let mut a = Engine::new(def.clone(), 3000.0, 1);
        let mut b = Engine::new(def, 3000.0, 1);
        let n = (0.12 / DT) as usize;
        let c = |k: usize| Controls {
            throttle: if (n / 3..n / 2).contains(&k) {
                1.0
            } else {
                0.2
            },
            ..Controls::default()
        };
        for k in 0..10 {
            assert_eq!(a.step(DT, &c(k)), b.step(DT, &c(k)));
        }
        assert!(b.detach_runners(5, |set| Box::new(crate::intake_fv::LocalRunners::new(set))));
        for k in 10..n {
            assert_eq!(a.step(DT, &c(k)), b.step(DT, &c(k)), "step {k}");
            assert_eq!(a.plenum_pressure_pa(), b.plenum_pressure_pa(), "step {k}");
        }
        let _ = b.end_detached_runners();
        assert!(
            b.intake_net
                .as_ref()
                .is_some_and(|n| n.manifold_pressure() > 0.0)
        );
        assert_eq!(a.rpm(), b.rpm());
    }

    #[test]
    fn an_engine_detached_and_taken_back_runs_as_one_never_detached() {
        // the net and half the runners handed out and straight back (a render block's start and
        // end with nothing in between): the same samples as an engine that kept them
        let def = EngineDef::lfa_v10();
        let mut a = Engine::new(def.clone(), 3000.0, 1);
        let mut b = Engine::new(def, 3000.0, 1);
        let c = Controls {
            throttle: 0.3,
            ..Controls::default()
        };
        for _ in 0..10 {
            a.step(DT, &c);
            b.step(DT, &c);
        }
        assert_eq!(b.cylinders(), 10);
        let (mut net, mut set) = (None, None);
        assert!(b.detach_exhaust(|n, max| {
            assert_eq!(n.valves(), 10);
            net = Some(n.clone());
            Box::new(crate::exhaust::LocalLink::new(n, max.min(16)))
        }));
        assert!(format!("{b:?}").contains("remote exhaust"));
        assert!(b.detach_runners(5, |s| {
            set = Some(s.clone());
            Box::new(crate::intake_fv::LocalRunners::new(s))
        }));
        assert_eq!(b.flush_exhaust(DT, &mut Vec::new()), 0);
        assert!(b.end_detached().is_some());
        assert!(b.end_detached_runners().is_some());
        b.attach_exhaust(net.unwrap());
        b.attach_runners(set.unwrap());
        for k in 0..2000 {
            assert_eq!(a.step(DT, &c), b.step(DT, &c), "step {k}");
        }
        // an engine with a plenum or a 0D intake keeps its runners
        assert!(
            !Engine::new(EngineDef::inline4(), 3000.0, 1)
                .detach_runners(2, |s| Box::new(crate::intake_fv::LocalRunners::new(s)))
        );
    }

    /// Sign flips of cylinder 1's exhaust-valve flow over 0.2 s of the LFA held at idle, stepped
    /// at `fs`, while the valve is open (steps with flow either side).
    fn idle_exhaust_flips(fs: f64) -> (usize, usize) {
        let dt = 1.0 / fs;
        let mut e = Engine::new(EngineDef::lfa_v10(), 1100.0, 1);
        let c = Controls {
            throttle: 0.0,
            hold_rpm: Some(1100.0),
            ..Controls::default()
        };
        for _ in 0..(1.0 / dt) as usize {
            e.step(dt, &c);
        }
        let (mut flips, mut open, mut prev) = (0, 0, 0.0f64);
        for _ in 0..(0.2 / dt) as usize {
            e.step(dt, &c);
            let f = e.cyl[0].port_flow;
            if f != 0.0 && prev != 0.0 {
                open += 1;
                flips += usize::from(f.signum() != prev.signum());
            }
            prev = f;
        }
        (flips, open)
    }

    #[test]
    fn the_valve_flow_does_not_chatter_at_a_coarse_step() {
        // at idle, near the pressure balance with the piston pushing, the valve's flow is
        // smooth whatever the step: at Low/Medium's 96 kHz it chattered on and off (103 flips in
        // 0.2 s against 5 at 192 kHz, +45 dB of hiss at 5–10 kHz). A real reversal or two per
        // event is physics; the flips per open step must not grow as the step coarsens
        let (fine, open_fine) = idle_exhaust_flips(192_000.0);
        let (coarse, open_coarse) = idle_exhaust_flips(96_000.0);
        let rate = |f: usize, o: usize| f as f64 / o.max(1) as f64;
        assert!(
            rate(coarse, open_coarse) <= 2.0 * rate(fine, open_fine) + 1e-3,
            "96 kHz: {coarse} flips in {open_coarse} steps; 192 kHz: {fine} in {open_fine}"
        );
    }

    #[test]
    fn an_fv_exhaust_is_heard() {
        // the finite-volume exhaust's tails reach the listener as the waveguide's do
        let mut d = EngineDef::inline4();
        d.exhaust.model = ExhaustModel::Fv;
        d.sound.flow_noise = 0.0;
        d.sound.intake = 0.0;
        d.sound.block = 0.0;
        let mut e = Engine::new(d, 3000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.2, &c);
        let x = run(&mut e, 0.1, &c);
        let rms = (x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64).sqrt();
        assert!(rms > 0.05, "{rms} Pa");
    }

    #[test]
    fn the_fv_and_waveguide_exhausts_agree_on_torque_and_loudness() {
        // two models of the same pipes: at 4000 rpm WOT they give the same torque within 2 %
        // (143.6 N·m waveguide, 144.2 FV) and tail levels within a factor of two (4.1 and 2.8 Pa
        // rms; the FV's shocks and losses differ from the linear lines). The valve's momentum
        // flux p + ρu² on the FV's first face, with ρ = γp/a², keeps it there; ρ off by a²
        // makes the FV five times louder
        let at = |model: ExhaustModel| {
            let mut d = EngineDef::inline4();
            d.exhaust.model = model;
            let mut e = Engine::new(d, 4000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(4000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            let n = (0.1 / DT) as usize;
            let (mut tq, mut ss) = (0.0, 0.0);
            for _ in 0..n {
                let x = e.step(DT, &c);
                tq += e.torque_nm();
                ss += x * x;
            }
            (tq / n as f64, (ss / n as f64).sqrt())
        };
        let (tq_w, rms_w) = at(ExhaustModel::Waveguide);
        let (tq_f, rms_f) = at(ExhaustModel::Fv);
        assert_relative_eq!(tq_f, tq_w, max_relative = 0.02);
        assert!(
            rms_f > 0.5 * rms_w && rms_f < 2.0 * rms_w,
            "FV {rms_f} Pa, waveguide {rms_w} Pa"
        );
    }

    #[test]
    fn the_valve_jets_ride_the_blowdowns() {
        // what the valve jets add at the listener: broadband (zero-mean), and it comes with the
        // blowdowns — its envelope, a cycle at a time, follows the valves' outflow
        let heard = |jet: f64| {
            let mut d = EngineDef::inline4();
            d.sound.valve_jet = jet;
            let mut e = Engine::new(d, 4000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(4000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            run(&mut e, 0.2, &c)
        };
        let (m, r) = mean_rms_of_difference(&heard(1.0), &heard(0.0));
        assert!(
            r > 1e-3 && m.abs() < 0.05 * r,
            "valve jets: mean {m} rms {r} Pa"
        );
    }

    #[test]
    fn the_exhaust_pulses_alone_reach_the_listener() {
        // every other source off: the pulses travel the pipes to the tails and out — a waveguide
        // kept from step to step carries them (rebuilt each step it would hold nothing)
        let mut d = EngineDef::inline4();
        d.sound.flow_noise = 0.0;
        d.sound.intake = 0.0;
        let mut e = Engine::new(d, 5000.0, 1);
        let c = Controls {
            throttle: 1.0,
            hold_rpm: Some(5000.0),
            ..Controls::default()
        };
        run(&mut e, 0.3, &c);
        let x = run(&mut e, 0.1, &c);
        let rms = (x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64).sqrt();
        assert!(rms > 0.1, "{rms} Pa");
    }

    #[test]
    fn the_tail_exit_by_hand() {
        // Gas at c = 500 m/s: T = 500²/(1.4·287) = 622.22 K. Through a 50 mm outlet
        // (A = 1.9635e-3 m²) the choked flow from p₀ = 3 atm is
        // A·p₀/√(R·T)·0.68473 = 1.9635e-3·303975/422.58·0.68473 = 0.96711 kg/s: the pressure
        // that passes it is 2 atm over the street's, and the jet leaves at c* = 500·√(2/2.4)
        let losses = |friction: f64, laminar: f64| crate::exhaust::ExitLosses {
            outlet_area: 1.963_495e-3,
            friction,
            laminar,
        };
        let (back, u) = tail_exit(0.967_11, &losses(0.0, 0.0));
        assert!((back / 202_650.0 - 1.0).abs() < 1e-3, "{back} Pa");
        assert!((u - 456.435).abs() < 0.05, "{u} m/s");
        // the pipes' friction and the catalyst's laminar resistance add at ρ = p₀/(R·T) =
        // 1.70222 kg/m³: 1000·m²/(2ρ) = 274.73 Pa and 1000·m/ρ = 568.15 Pa
        let (with, _) = tail_exit(0.967_11, &losses(1000.0, 1000.0));
        assert!((with - back - 842.88).abs() < 1.0, "{} Pa", with - back);
        // subsonic: 1.1 atm behind the outlet passes (pr = 1/1.1, q = pr^(2/7) = 0.973136)
        // A·p₀/√(RT)·(pr/q)·√(7(1 − q)) = 0.209798 kg/s, leaving at √(2·c_p·T·(1 − q)) =
        // 183.249 m/s with 10 132.5 Pa behind it
        let (back, u) = tail_exit(0.209_798, &losses(0.0, 0.0));
        assert!((back / 10_132.5 - 1.0).abs() < 1e-3, "{back} Pa");
        assert!((u - 183.249).abs() < 0.05, "{u} m/s");
        // no flow, no backpressure
        assert_eq!(tail_exit(0.0, &losses(1000.0, 1000.0)), (0.0, 0.0));
    }

    #[test]
    fn the_tail_exit_inverts_the_orifice() {
        // the backpressure found for a flow passes exactly that flow through the outlet, from a
        // trickle to deep choking
        let losses = crate::exhaust::ExitLosses {
            outlet_area: 1.963_495e-3,
            friction: 0.0,
            laminar: 0.0,
        };
        for m in [1e-3, 0.05, 0.2, 0.4, 0.5, 0.6, 1.0, 3.0] {
            let (back, _) = tail_exit(m, &losses);
            let p0 = crate::units::ATM_PA + back;
            let through =
                crate::gas::orifice_mass_flow(1.963_495e-3, p0, TAIL_T_K, crate::units::ATM_PA);
            assert_relative_eq!(through, m, max_relative = 1e-9);
        }
    }

    #[test]
    fn no_tail_blows_past_sound() {
        // a 15 mm tail at full throttle: the pulses would drive it far past sonic, but a choked
        // exit leaves at c* = 500·√(2/2.4) = 456.435 m/s, and no faster
        let (_, u15) = with_tail(0.015);
        let fastest = u15.iter().cloned().fold(f64::MIN, f64::max);
        assert!((fastest - 456.435).abs() < 0.01, "exit at {fastest} m/s");
    }

    #[test]
    fn each_preset_carries_only_the_exhaust_it_has_data_for() {
        // after each collector, the catalyst every road car carries (sized by its rule) and the
        // tail pipe of the preset's own length and bore — no silencer sized by rule
        for d in [
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            EngineDef::flatplane_v8(),
            EngineDef::crossplane_v8(),
        ] {
            let branch = d.displacement_m3() / d.exhaust.collectors.len().max(1) as f64;
            assert_eq!(
                d.exhaust.after_collector,
                [
                    crate::exhaust::close_coupled_catalyst(branch, d.exhaust.tail_diameter_m),
                    crate::exhaust::Segment::Pipe {
                        length_m: d.exhaust.tail_length_m,
                        diameter_m: d.exhaust.tail_diameter_m,
                    }
                ],
                "{}",
                d.name
            );
        }
        assert_eq!(EngineDef::inline4().exhaust.tail_length_m, 1.5);
        assert_eq!(EngineDef::lfa_v10().exhaust.tail_length_m, 1.2);
    }

    #[test]
    fn only_the_fresh_charge_burns() {
        // At part throttle the residual (the mass left when the exhaust valve closed) is a fair
        // share of what is trapped (measured 7.3 % here): each burn's energy is the fresh
        // charge's, (m_IVC − m_EVC)/(1 + AFR)·LHV·η, within the ±4 % cycle variation — the
        // whole trapped mass's would be 7.9 % more.
        let mut e = Engine::new(EngineDef::inline4(), 3000.0, 1);
        let c = Controls {
            throttle: 0.15,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        run(&mut e, 0.5, &c);
        let mut checked = 0;
        let mut was_burning = e.cyl[0].burn.is_some();
        for _ in 0..(0.5 / DT) as usize {
            e.step(DT, &c);
            let cs = &e.cyl[0];
            if !was_burning && let Some(b) = &cs.burn {
                let fresh = cs.trapped_kg - cs.residual_kg;
                assert!(
                    cs.residual_kg > 0.05 * cs.trapped_kg,
                    "residual {} of {}",
                    cs.residual_kg,
                    cs.trapped_kg
                );
                let want = charge_energy(fresh, AFR_STOICH, COMBUSTION_EFFICIENCY);
                assert!(
                    (b.energy_j / want - 1.0).abs() <= 0.041,
                    "{} vs {want}",
                    b.energy_j
                );
                checked += 1;
            }
            was_burning = cs.burn.is_some();
        }
        assert!(checked >= 5, "{checked} burns");
    }

    #[test]
    fn the_block_radiates_the_cylinder_pressure() {
        // what the block adds at the listener: inline-4, 5000 rpm WOT. By hand: each cylinder's
        // pressure as a Gaussian pulse, 7 MPa peak, 60° wide at half height (σ = 0.85 ms), once
        // per 24 ms cycle; its harmonic k has 2Pσ√(2π)/T·exp(−(2πfσ)²/2), four cylinders adding
        // at the firing orders: 3.3 MPa at 167 Hz, 1.0 MPa at 333 Hz, 0.14 MPa at 500 Hz. The
        // figure's structure response there (26, 39.5, 47 dB for 1 kPa) gives 1.3, 1.9, 0.6 Pa
        // at 1 m: 1.7 Pa rms, 0.53 Pa at the 3.27 m to the ear. The real pulse rises faster than
        // a Gaussian (more above 300 Hz), so up to a few times that
        let heard = |block: f64, distance: f64| {
            let mut d = EngineDef::inline4();
            d.sound.block = block;
            d.sound.listener_distance_m = distance;
            let mut e = Engine::new(d, 5000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(5000.0),
                ..Controls::default()
            };
            run(&mut e, 0.3, &c);
            run(&mut e, 0.2, &c)
        };
        let (m, near) = mean_rms_of_difference(&heard(1.0, 3.0), &heard(0.0, 3.0));
        assert!(
            (0.25..1.6).contains(&near) && m.abs() < 0.05 * near,
            "block: mean {m} rms {near} Pa"
        );
        // a spherical spreader, 1/r: the ear 1.3 m above the tails, so 3 m and 6 m out are
        // √(3² + 1.3²) = 3.27 m and √(6² + 1.3²) = 6.14 m away: 0.533 of the pressure
        let (_, far) = mean_rms_of_difference(&heard(1.0, 6.0), &heard(0.0, 6.0));
        assert!(
            (far / near - 0.533).abs() < 0.005,
            "{far} at 6 m vs {near} at 3 m"
        );
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
        // intake: the mouth is a sink of the air drawn in, −ṁ/ρ₀. A 2.0 L at 5000 rpm WOT with
        // η_v ≈ 0.9 breathes 2.0e-3·0.9·1.177·5000/120 = 0.088 kg/s: −0.075 m³/s
        let (m, _) = mean_rms_of_difference(&textured(0.0, 1.0, 0.0, 5000.0, 1.0), &dry);
        assert!(
            (-0.09..-0.06).contains(&m),
            "intake mouth mean flow {m} m³/s"
        );
        // valvetrain: no physical model yet, so the level does nothing
        let (m, r) = mean_rms_of_difference(&textured(0.0, 0.0, 1.0, 5000.0, 1.0), &dry);
        assert_eq!((m, r), (0.0, 0.0));
    }

    #[test]
    fn the_intake_mouth_follows_the_throttle_flow() {
        // −ṁ/ρ₀: more air at full throttle than at a light one, at the same speed
        let draw = |throttle: f64| {
            let dry = textured(0.0, 0.0, 0.0, 4000.0, throttle);
            -mean_rms_of_difference(&textured(0.0, 1.0, 0.0, 4000.0, throttle), &dry).0
        };
        let (light, wot) = (draw(0.1), draw(1.0));
        assert!(
            wot > 2.0 * light && light > 0.0,
            "light {light} vs WOT {wot} m³/s"
        );
    }

    #[test]
    fn burns_complete_with_jitter_on() {
        // The burn must run to the end (Wiebe ≥ 99 %) every cycle with jitter enabled: the
        // mean of the cycles' peak pressures with and without jitter within 15 % (a burn dropped
        // early would collapse it). The mean, not the highest: with a peak pressure varying
        // ~8 % per cycle (`cycles_vary_as_measured_engines_do`) the highest of a dozen cycles
        // sits ~1.6σ, 13 %, above it.
        let peak = |jitter: f64| {
            let mut d = EngineDef::single();
            d.sound.combustion_jitter = jitter;
            let p = cycle_peaks(d, 3000.0, 1.0, 0.5);
            p.iter().sum::<f64>() / p.len() as f64
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
