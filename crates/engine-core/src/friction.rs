//! Engine friction by components: the Patton–Nitschke–Heywood model (SAE 890836) as improved by
//! Sandoval & Heywood (SAE 2003-01-0725; full expression in Sandoval, B.S. thesis, MIT 2002,
//! Appendix A-2). Crankshaft, reciprocating (piston, rings, rods, ring gas loading), valvetrain and
//! accessory friction from each engine's own design: bore, stroke, bearings, valves, valvetrain.
//! The model's pumping terms are left out: the gas exchange is simulated, so pumping work is
//! already in the p–V cycle.

use serde::{Deserialize, Serialize};

/// Valve drive, with Sandoval's constants for each (thesis Table 4.2): flat follower, roller
/// follower, oscillating hydrodynamic and oscillating mixed, in kPa-based units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Valvetrain {
    SohcFingerFollower,
    SohcRockerArm,
    SohcDirectActing,
    DohcFingerFollower,
    /// Bucket tappets straight under the cams: the usual high-revving DOHC.
    #[default]
    DohcDirectActing,
    /// Cam in block, pushrods and rocker arms.
    Ohv,
}

impl Valvetrain {
    /// (C_ff, C_rf, C_oh, C_om): Sandoval 2002, Table 4.2.
    fn constants(self) -> (f64, f64, f64, f64) {
        match self {
            Self::SohcFingerFollower => (600.0, 0.0227, 0.2, 42.8),
            Self::SohcRockerArm => (400.0, 0.0151, 0.5, 21.4),
            Self::SohcDirectActing => (200.0, 0.0076, 0.5, 10.7),
            Self::DohcFingerFollower => (600.0, 0.0227, 0.2, 25.8),
            Self::DohcDirectActing => (133.0, 0.0050, 0.5, 10.7),
            // the table prints 0.5 for the OHV roller constant; Patton (890836) estimated the
            // OHV cam-follower constants as two thirds of the SOHC finger follower's: 0.0151
            Self::Ohv => (400.0, 0.0151, 0.5, 32.1),
        }
    }

    /// Camshafts per bank.
    fn camshafts(self) -> usize {
        match self {
            Self::DohcFingerFollower | Self::DohcDirectActing => 2,
            _ => 1,
        }
    }
}

/// What the engine is built with, as the friction model needs it. Every field has a default.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FrictionDef {
    pub valvetrain: Valvetrain,
    /// Roller cam followers (rolling contact) instead of flat ones (mixed lubrication).
    pub roller_followers: bool,
    pub valves_per_cylinder: u32,
    /// Connecting rods sharing each crankpin on a vee engine (2 on most V8s and V10s); sets the
    /// bearing sizes (Patton, Tables 1–2). Inline engines ignore it.
    pub rods_per_crankpin: u32,
    /// Piston ring tension against the model's reference rings, F_t/F_t0 (Sandoval: set per engine
    /// from data; 1 = the reference).
    pub ring_tension_ratio: f64,
    /// Piston and liner roughness against the reference, C_r (1 = the reference).
    pub liner_roughness: f64,
    /// Oil viscosity against the reference warm oil, μ/μ0 (≈ 2 on a cold engine, Sandoval Fig.
    /// 11; 1 warm).
    pub oil_viscosity_ratio: f64,
}

impl Default for FrictionDef {
    fn default() -> Self {
        Self {
            valvetrain: Valvetrain::default(),
            roller_followers: false,
            valves_per_cylinder: 4,
            rods_per_crankpin: 2,
            ring_tension_ratio: 1.0,
            liner_roughness: 1.0,
            oil_viscosity_ratio: 1.0,
        }
    }
}

/// The engine as the model sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrictionEngine {
    pub bore_m: f64,
    pub stroke_m: f64,
    pub cylinders: usize,
    /// Two banks (vee or flat) instead of one.
    pub vee: bool,
    pub compression_ratio: f64,
    /// Maximum lift of one real valve, m.
    pub valve_lift_m: f64,
}

/// Friction mean effective pressure by component, Pa.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fmep {
    pub crankshaft: f64,
    /// Piston skirt, rings without gas loading, connecting rod bearings.
    pub reciprocating: f64,
    /// The rings' extra friction from cylinder gas pressure.
    pub piston_gas: f64,
    pub valvetrain: f64,
    /// Oil pump, water pump, unloaded alternator.
    pub auxiliary: f64,
}

impl Fmep {
    pub fn total(&self) -> f64 {
        self.crankshaft + self.reciprocating + self.piston_gas + self.valvetrain + self.auxiliary
    }
}

/// The friction at one speed, split into what the intake pressure does not touch and the ring
/// gas loading, which is proportional to it: cheap to evaluate every step while the speed holds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AtSpeed {
    pub rpm: f64,
    /// Every term but the gas loading, Pa.
    pub rest_pa: f64,
    /// Gas loading per Pa of intake pressure.
    pub gas_per_pa: f64,
}

impl AtSpeed {
    /// Total fmep, Pa, with the intake manifold at `intake_pa`.
    pub fn fmep(&self, intake_pa: f64) -> f64 {
        self.rest_pa + self.gas_per_pa * intake_pa
    }
}

/// Bishop's constant in the ring gas-loading exponent, s/m (Patton 890836), doubled by Sandoval
/// for smoother liners.
const K_BISHOP: f64 = 2.38e-2;
/// The boundary and mixed terms grow as 1 + 500/N, fit from 1000 rpm up. Below this speed (only
/// while cranking or stalling) they hold their value here instead of growing without bound.
const MIN_RPM: f64 = 200.0;
const ATM_KPA: f64 = 101.325;

impl FrictionDef {
    /// Each component's fmep at `rpm` with the intake manifold at `intake_pa` (Sandoval 2002,
    /// Appendix A-2, without its pumping terms). Lengths go in as mm and the result comes out of
    /// the fit in kPa, as the constants are written.
    pub fn fmep(&self, e: &FrictionEngine, rpm: f64, intake_pa: f64) -> Fmep {
        let (b, s) = (e.bore_m * 1e3, e.stroke_m * 1e3);
        let nc = e.cylinders.max(1) as f64;
        let n = rpm.abs();
        let mixed = 1.0 + 500.0 / n.max(MIN_RPM);
        let visc = self.oil_viscosity_ratio.max(0.0).sqrt();
        let sp = 2.0 * e.stroke_m * n / 60.0;
        // bearing sizes from bore and block (Patton 890836, Tables 1 and 2); main bearings: one
        // more than the cylinders inline, one more than half of them on a vee
        let (main_d, main_l, rod_d, rod_l, mains) = match (e.vee, self.rods_per_crankpin) {
            (false, _) => (0.60, 0.37, 0.57, 0.41, nc + 1.0),
            (true, 1) => (0.70, 0.35, 0.60, 0.36, 1.0 + (nc / 2.0).ceil()),
            (true, _) => (0.62, 0.40, 0.57, 0.39, 1.0 + (nc / 2.0).ceil()),
        };
        let (dbm, dbr) = (main_d * b, rod_d * b);
        let (lbm, lbr) = (main_l * dbm, rod_l * dbr);
        let b2snc = b * b * s * nc;
        let crankshaft = 1.22e5 * dbm / b2snc
            + 3.03e-4 * visc * n * dbm.powi(3) * lbm * mains / b2snc
            + 1.35e-10 * dbm * dbm * n * n * mains / nc;
        let reciprocating = 2.94e2 * visc * sp / b
            + 4.06e4 * self.ring_tension_ratio * self.liner_roughness * mixed / (b * b)
            + 3.03e-4 * visc * n * dbr.powi(3) * lbr * nc / b2snc;
        let piston_gas = 6.89
            * (intake_pa / 1e3 / ATM_KPA)
            * (0.088 * visc * e.compression_ratio
                + 0.182
                    * self.ring_tension_ratio
                    * e.compression_ratio.powf(1.33 - 2.0 * K_BISHOP * sp));
        let (cff, crf, coh, com) = self.valvetrain.constants();
        let banks = if e.vee { 2.0 } else { 1.0 };
        let cam_bearings = mains * (self.valvetrain.camshafts() as f64) * banks;
        let nv = self.valves_per_cylinder as f64 * nc;
        let lv = e.valve_lift_m * 1e3;
        let follower = if self.roller_followers {
            crf * n * nv / (s * nc)
        } else {
            cff * mixed * nv / (s * nc)
        };
        let valvetrain = 244.0 * visc * n * cam_bearings / b2snc
            + 4.12
            + follower
            + coh * visc * lv.powf(1.5) * n.sqrt() * nv / (b * s * nc)
            + com * mixed * lv * nv / (s * nc);
        let auxiliary = (6.23 + 5.22e-3 * n - 1.79e-7 * n * n).max(0.0);
        let pa = |kpa: f64| kpa * 1e3;
        Fmep {
            crankshaft: pa(crankshaft),
            reciprocating: pa(reciprocating),
            piston_gas: pa(piston_gas),
            valvetrain: pa(valvetrain),
            auxiliary: pa(auxiliary),
        }
    }
}

impl FrictionDef {
    /// [`Self::fmep`] at `rpm`, ready for any intake pressure.
    pub fn at_speed(&self, e: &FrictionEngine, rpm: f64) -> AtSpeed {
        let f = self.fmep(e, rpm, ATM_KPA * 1e3);
        AtSpeed {
            rpm,
            rest_pa: f.total() - f.piston_gas,
            gas_per_pa: f.piston_gas / (ATM_KPA * 1e3),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    const ATM_PA: f64 = ATM_KPA * 1e3;

    /// Ford 5.4 L V8 (Sandoval 2002, Table 4.1: SOHC direct acting, 2 valves, bore/stroke 0.85,
    /// 9.0:1): 90.2 × 105.8 mm. Valve lift is not given; 10 mm.
    fn ford_5_4() -> (FrictionDef, FrictionEngine) {
        let def = FrictionDef {
            valvetrain: Valvetrain::SohcDirectActing,
            roller_followers: true,
            valves_per_cylinder: 2,
            ..FrictionDef::default()
        };
        let e = FrictionEngine {
            bore_m: 0.0902,
            stroke_m: 0.1058,
            cylinders: 8,
            vee: true,
            compression_ratio: 9.0,
            valve_lift_m: 0.010,
        };
        (def, e)
    }

    #[test]
    fn reproduces_sandovals_components_for_the_ford_5_4_at_2500_rpm_wot() {
        // Sandoval 2002, Fig. 3-3 (improved model, 2500 rpm, WOT), bars read off the chart:
        // crankshaft ≈ 5, piston ≈ 38, piston/gas ≈ 14, valvetrain ≈ 9, auxiliary ≈ 19 kPa
        let (def, e) = ford_5_4();
        let f = def.fmep(&e, 2500.0, ATM_PA);
        assert_abs_diff_eq!(f.crankshaft / 1e3, 5.0, epsilon = 1.0);
        assert_abs_diff_eq!(f.reciprocating / 1e3, 38.0, epsilon = 1.5);
        assert_abs_diff_eq!(f.piston_gas / 1e3, 14.0, epsilon = 1.0);
        // the lift is a guess, so a wider band
        assert_abs_diff_eq!(f.valvetrain / 1e3, 9.0, epsilon = 1.5);
        assert_abs_diff_eq!(f.auxiliary / 1e3, 19.0, epsilon = 1.0);
        // and exactly, by hand: mains 55.92 × 22.37 mm, five of them (one more than half the
        // cylinders on a vee): seals 0.80245 + journals 1.43838 + turbulence 2.55107 kPa
        assert_abs_diff_eq!(f.crankshaft / 1e3, 4.791_90, epsilon = 1e-4);
    }

    /// Honda K20A: 86 × 86 mm, 11.5:1, DOHC roller rocker arms (finger followers), 4 valves of
    /// 12.4/√2 = 8.77 mm lift.
    fn k20a() -> (FrictionDef, FrictionEngine) {
        let def = FrictionDef {
            valvetrain: Valvetrain::DohcFingerFollower,
            roller_followers: true,
            ..FrictionDef::default()
        };
        let e = FrictionEngine {
            bore_m: 0.086,
            stroke_m: 0.086,
            cylinders: 4,
            vee: false,
            compression_ratio: 11.5,
            valve_lift_m: 0.0124 / 2f64.sqrt(),
        };
        (def, e)
    }

    #[test]
    fn an_inline_four_at_8000_rpm_by_hand() {
        // by hand from Appendix A-2 with Patton's line-engine bearings: mains 51.6 × 19.09 mm,
        // five of them; rods 49.02 × 20.10 mm; S_p = 22.93 m/s
        let (def, e) = k20a();
        let f = def.fmep(&e, 8000.0, ATM_PA);
        assert_abs_diff_eq!(f.crankshaft / 1e3, 43.72, epsilon = 0.05);
        assert_abs_diff_eq!(f.reciprocating / 1e3, 93.30, epsilon = 0.05);
        assert_abs_diff_eq!(f.piston_gas / 1e3, 9.20, epsilon = 0.05);
        assert_abs_diff_eq!(f.valvetrain / 1e3, 31.69, epsilon = 0.05);
        assert_abs_diff_eq!(f.auxiliary / 1e3, 36.53, epsilon = 0.05);
        // 2.14 bar in all, against 5.37 from the old whole-engine fit (which held pumping too)
        assert_abs_diff_eq!(f.total() / 1e5, 2.144, epsilon = 0.002);
    }

    #[test]
    fn oil_rings_and_liner_scale_their_own_terms() {
        // K20A geometry at 3000 rpm and 1 atm, bucket tappets with flat followers, oil at
        // μ/μ0 = 2.25 (√ = 1.5), rings at 0.8 of the reference tension, liner 1.2 × as rough.
        // Every component by hand from Appendix A-2 (each viscosity, tension and roughness
        // factor where Sandoval puts it):
        let (_, e) = k20a();
        let def = FrictionDef {
            valvetrain: Valvetrain::DohcDirectActing,
            roller_followers: false,
            oil_viscosity_ratio: 2.25,
            ring_tension_ratio: 0.8,
            liner_roughness: 1.2,
            ..FrictionDef::default()
        };
        let f = def.fmep(&e, 3000.0, ATM_PA);
        assert_abs_diff_eq!(f.crankshaft / 1e3, 13.5467, epsilon = 1e-3);
        assert_abs_diff_eq!(f.reciprocating / 1e3, 55.3232, epsilon = 1e-3);
        assert_abs_diff_eq!(f.piston_gas / 1e3, 19.9629, epsilon = 1e-3);
        assert_abs_diff_eq!(f.valvetrain / 1e3, 21.3205, epsilon = 1e-3);
    }

    #[test]
    fn a_vee_with_one_rod_a_crankpin_has_bigger_bearings() {
        // V6 3.0 L, 89 × 80 mm, split crankpins: Patton Table 1–2 (vee, 1 rod/pin) mains 62.3 mm
        // × 21.8 mm, four of them (1 + 6/2); rods 53.4 × 19.2 mm. By hand at 3000 rpm, 10:1,
        // 9 mm lift, bucket tappets: crankshaft 10.1851, reciprocating 36.6060 kPa
        let def = FrictionDef {
            rods_per_crankpin: 1,
            ..FrictionDef::default()
        };
        let e = FrictionEngine {
            bore_m: 0.089,
            stroke_m: 0.080,
            cylinders: 6,
            vee: true,
            compression_ratio: 10.0,
            valve_lift_m: 0.009,
        };
        let f = def.fmep(&e, 3000.0, ATM_PA);
        assert_abs_diff_eq!(f.crankshaft / 1e3, 10.1851, epsilon = 1e-3);
        assert_abs_diff_eq!(f.reciprocating / 1e3, 36.6060, epsilon = 1e-3);
        assert_abs_diff_eq!(f.valvetrain / 1e3, 20.9917, epsilon = 1e-3);
    }

    #[test]
    fn ring_gas_loading_follows_the_intake_pressure() {
        // the gas term is proportional to p_i/p_a; nothing else depends on it
        let (def, e) = k20a();
        let (full, half) = (
            def.fmep(&e, 3000.0, ATM_PA),
            def.fmep(&e, 3000.0, 0.5 * ATM_PA),
        );
        assert_abs_diff_eq!(half.piston_gas, 0.5 * full.piston_gas, epsilon = 1e-9);
        assert_eq!(half.reciprocating, full.reciprocating);
    }

    #[test]
    fn thick_oil_scales_the_hydrodynamic_terms_by_its_square_root() {
        // μ/μ0 = 4 doubles the hydrodynamic terms only. Crankshaft at 2000 rpm by hand: seals
        // 1.22e5·51.6/(86²·86·4) = 2.4743, journals 3.03e-4·2000·51.6³·19.092·5/(86²·86·4)
        // = 3.1238, turbulence 1.35e-10·51.6²·2000²·5/4 = 1.7972 kPa
        let (mut def, e) = k20a();
        def.oil_viscosity_ratio = 4.0;
        let f = def.fmep(&e, 2000.0, ATM_PA);
        assert_abs_diff_eq!(
            f.crankshaft / 1e3,
            2.4743 + 2.0 * 3.1238 + 1.7972,
            epsilon = 2e-3
        );
    }

    #[test]
    fn a_flat_follower_rubs_more_than_a_roller() {
        // Patton: flat followers run in mixed lubrication, 600·(1 + 500/N)·n_v/(S·n_c) for a
        // finger follower; at 2000 rpm 600·1.25·16/(86·4) = 34.88 kPa against the roller's
        // 0.0227·2000·16/(86·4) = 2.112 kPa
        let (mut def, e) = k20a();
        let roller = def.fmep(&e, 2000.0, ATM_PA).valvetrain;
        def.roller_followers = false;
        let flat = def.fmep(&e, 2000.0, ATM_PA).valvetrain;
        assert_abs_diff_eq!((flat - roller) / 1e3, 34.884 - 2.112, epsilon = 2e-3);
    }

    #[test]
    fn at_a_speed_the_split_gives_the_same_friction_at_any_intake_pressure() {
        let (def, e) = k20a();
        let s = def.at_speed(&e, 5300.0);
        for p in [30e3, ATM_PA, 140e3] {
            assert_abs_diff_eq!(s.fmep(p), def.fmep(&e, 5300.0, p).total(), epsilon = 1e-6);
        }
    }

    #[test]
    fn a_stopped_engine_has_finite_friction() {
        let (def, e) = k20a();
        let f = def.fmep(&e, 0.0, ATM_PA);
        assert!(f.total().is_finite() && f.total() > 0.0);
        assert_eq!(f.total(), def.fmep(&e, -0.0, ATM_PA).total());
    }

    #[test]
    fn friction_round_trips_and_old_files_get_the_defaults() {
        let def = k20a().0;
        let json = serde_json::to_string(&def).unwrap();
        assert_eq!(serde_json::from_str::<FrictionDef>(&json).unwrap(), def);
        let yaml = serde_yaml_ng::to_string(&def).unwrap();
        assert_eq!(serde_yaml_ng::from_str::<FrictionDef>(&yaml).unwrap(), def);
        assert_eq!(
            serde_json::from_str::<FrictionDef>("{}").unwrap(),
            FrictionDef::default()
        );
    }
}
