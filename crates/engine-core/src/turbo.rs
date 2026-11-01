//! A turbocharger: a radial compressor feeding the intake, driven on one shaft by a turbine the
//! exhaust passes through. Quasi-steady 0D, as in the filling-and-emptying engine models
//! (Heywood, *Internal Combustion Engine Fundamentals*, 2nd ed., §6.8 and §7.6; Watson &
//! Janota, *Turbocharging the Internal Combustion Engine*, 1982): the compressor gives the ratio
//! its tip speed can (Euler's work with slip), the turbine passes the exhaust as a nozzle (its
//! "swallowing capacity"), and the shaft's inertia between their powers is the lag.
//!
//! Left out (*my call*): the compressor's surge line and choke; the wastegate's actuator
//! dynamics (a proportional gate).

use serde::{Deserialize, Serialize};

use crate::units::ATM_PA;

/// Air: ratio of specific heats, c_p (J/kg·K), and the gas constant (J/kg·K).
const GAMMA_AIR: f64 = 1.4;
const CP_AIR: f64 = 1005.0;
const R_GAS: f64 = 287.0;
/// Hot exhaust's ratio of specific heats (Heywood Fig. 4-11: 1.3–1.35 at 800–1200 K).
const GAMMA_EXH: f64 = 1.33;
/// Ambient temperature at the compressor inlet, K: the intake's.
const T_AMB_K: f64 = crate::intake_fv::AMBIENT_T_K;
/// Boost over the wastegate's set point at which it is fully open, Pa (ESTIMATE: a spring
/// actuator's travel over ~0.1 bar).
const WASTEGATE_SPAN_PA: f64 = 10_000.0;

/// A turbocharger (data, `serde`). Defaults: a small passenger-car turbo (ESTIMATES, the range
/// of Watson & Janota's ch. 3 examples).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TurboDef {
    /// Compressor wheel's tip diameter, m.
    pub compressor_tip_diameter_m: f64,
    /// Compressor blades (full + splitter): the whine's blade-passing order.
    pub blades: u32,
    /// Slip factor σ of the backswept impeller (Wiesner/Stanitz: 0.85–0.9).
    pub slip: f64,
    /// The impeller's exit: its blades' backsweep, degrees, and its width, m.
    pub backsweep_deg: f64,
    pub exit_width_m: f64,
    /// Isentropic efficiencies of the compressor and the turbine, and the shaft's mechanical.
    pub eta_c: f64,
    pub eta_t: f64,
    pub eta_m: f64,
    /// Turbine's effective nozzle area, m².
    pub turbine_area_m2: f64,
    /// Rotor's moment of inertia (both wheels and the shaft), kg·m².
    pub inertia_kgm2: f64,
    /// Wastegate: the boost (gauge) it opens at, Pa, and its area fully open, m².
    pub wastegate_pa: f64,
    pub wastegate_area_m2: f64,
    /// Intercooler: effectiveness ε, and its pressure drop Δp = k·ṁ², Pa per (kg/s)².
    pub intercooler_effectiveness: f64,
    pub intercooler_drop_k: f64,
    /// Blow-off valve's area, m² (0: none).
    pub bov_area_m2: f64,
}

impl Default for TurboDef {
    fn default() -> Self {
        Self {
            compressor_tip_diameter_m: 0.05,
            blades: 12,
            slip: 0.9,
            backsweep_deg: 35.0,
            exit_width_m: 0.004,
            eta_c: 0.72,
            eta_t: 0.70,
            eta_m: 0.95,
            turbine_area_m2: 6e-4,
            inertia_kgm2: 2e-5,
            wastegate_pa: 100_000.0,
            wastegate_area_m2: 5e-4,
            intercooler_effectiveness: 0.75,
            intercooler_drop_k: 2.5e5,
            bov_area_m2: 3e-4,
        }
    }
}

/// The compressor's total pressure ratio at tip speed `u` (m/s): Euler's work σU² at an
/// isentropic efficiency η, π = (1 + η·σ·U²/(c_p·T₀₁))^(γ/(γ−1)) (Dixon & Hall, *Fluid
/// Mechanics and Thermodynamics of Turbomachinery*, ch. 7; Heywood eq. 6.44 inverted).
pub fn compressor_ratio(u: f64, eta_c: f64, slip: f64, t01: f64) -> f64 {
    ratio_from_work(slip * u * u, eta_c, t01)
}

/// The total pressure ratio of a specific work `w` (J/kg) done at efficiency η from `t01` K.
pub fn ratio_from_work(w: f64, eta_c: f64, t01: f64) -> f64 {
    (1.0 + eta_c * w.max(0.0) / (CP_AIR * t01)).powf(GAMMA_AIR / (GAMMA_AIR - 1.0))
}

/// Euler's specific work of a backswept radial impeller, J/kg: Δh = U²(σ − φ₂·tan β₂), the
/// flow coefficient φ₂ = c_m2/U = ṁ/(ρ·A₂·U) (Dixon & Hall ch. 7: the work falls with flow, the
/// map's speed lines leaning over). `rho` the gas density at the exit, `exit_area` π·D₂·b₂.
/// Never negative.
pub fn compressor_work(
    u: f64,
    m_dot: f64,
    slip: f64,
    tan_b2: f64,
    exit_area: f64,
    rho: f64,
) -> f64 {
    (slip * u * u - u * m_dot.max(0.0) * tan_b2 / (rho * exit_area)).max(0.0)
}

/// Power to compress `m_dot` kg/s from `t01` K through `ratio` at efficiency η_c, W (Heywood
/// eq. 6.44).
pub fn compressor_power(m_dot: f64, t01: f64, ratio: f64, eta_c: f64) -> f64 {
    let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
    m_dot * CP_AIR * t01 * (ratio.powf(k) - 1.0) / eta_c
}

/// Mass flow of a gas (γ, R = 287) through a nozzle of `area` m² from stagnation `p0`, `t0` to
/// `p` downstream, choked below the critical ratio (Heywood App. C, eqs. C.8–C.9). 0 when
/// `p0 ≤ p`.
pub fn nozzle_flow(area: f64, p0: f64, t0: f64, p: f64, gamma: f64) -> f64 {
    if p0 <= p || area <= 0.0 {
        return 0.0;
    }
    let g = gamma;
    let crit = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
    let pr = (p / p0).max(crit);
    area * p0 / (R_GAS * t0).sqrt()
        * pr.powf(1.0 / g)
        * (2.0 * g / (g - 1.0) * (1.0 - pr.powf((g - 1.0) / g))).sqrt()
}

/// The exhaust through the turbine's nozzle, kg/s.
pub fn turbine_flow(area: f64, p03: f64, t03: f64, p4: f64) -> f64 {
    nozzle_flow(area, p03, t03, p4, GAMMA_EXH)
}

/// Power the turbine takes from `m_dot` kg/s expanding from `p03`, `t03` to `p4` at efficiency
/// η_t, W (Heywood eq. 6.50).
pub fn turbine_power(m_dot: f64, t03: f64, p03: f64, p4: f64, eta_t: f64) -> f64 {
    if p03 <= p4 {
        return 0.0;
    }
    let cp = GAMMA_EXH * R_GAS / (GAMMA_EXH - 1.0);
    m_dot * cp * t03 * eta_t * (1.0 - (p4 / p03).powf((GAMMA_EXH - 1.0) / GAMMA_EXH))
}

/// What the engine sees of the turbo this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurboState {
    /// The intake's supply after the intercooler: pressure, Pa, and temperature, K.
    pub boost_pa: f64,
    pub boost_t_k: f64,
    /// Turbine inlet pressure, Pa: the exhaust ports' mean back pressure.
    pub turbine_inlet_pa: f64,
    pub shaft_rpm: f64,
    /// Air the blow-off valve vents, kg/s.
    pub bov_m_dot: f64,
}

/// A turbocharger's running state: its shaft's kinetic energy.
#[derive(Debug, Clone)]
pub struct Turbo {
    def: TurboDef,
    energy_j: f64,
}

impl Turbo {
    pub fn new(def: &TurboDef) -> Self {
        Self {
            def: def.clone(),
            energy_j: 0.0,
        }
    }

    /// One step of `dt` s: `exhaust_m_dot` kg/s at `t03` K into the turbine, `intake_m_dot` kg/s
    /// drawn by the engine, `lift_off` when the throttle is shut.
    pub fn step(
        &mut self,
        exhaust_m_dot: f64,
        t03: f64,
        intake_m_dot: f64,
        lift_off: bool,
        dt: f64,
    ) -> TurboState {
        let d = &self.def;
        let omega = (2.0 * self.energy_j / d.inertia_kgm2.max(1e-9)).sqrt();
        let u = omega * 0.5 * d.compressor_tip_diameter_m;
        let rho = ATM_PA / (R_GAS * T_AMB_K);
        let tan_b2 = d.backsweep_deg.to_radians().tan();
        let exit_area = std::f64::consts::PI * d.compressor_tip_diameter_m * d.exit_width_m;
        let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
        // the compressor's exit at a flow `m`: (ratio, pressure, temperature)
        // (the exit's density is the exit state's: one pass at the inlet's, a second at the
        // state the first gives — the static pressure and temperature taken as the total's)
        let exit = |m: f64| {
            let state = |rho2: f64| {
                let w = compressor_work(u, m, d.slip, tan_b2, exit_area, rho2);
                let r = ratio_from_work(w, d.eta_c, T_AMB_K);
                (r, T_AMB_K * (1.0 + (r.powf(k) - 1.0) / d.eta_c))
            };
            let (r1, t1) = state(rho);
            let (r, t) = state(rho * r1 * T_AMB_K / t1);
            (r, ATM_PA * r, t)
        };
        let intake = intake_m_dot.max(0.0);
        // a lift-off opens the blow-off valve (its diaphragm referenced to the manifold, whose
        // vacuum pulls it open): the compressor then delivers the engine's draw and the valve's vent, whose flow the delivered pressure sets (bisection on
        // the flow: more flow, less pressure, less vented)
        let bov = |m: f64| nozzle_flow(d.bov_area_m2, exit(m).1, exit(m).2, ATM_PA, GAMMA_AIR);
        let venting = lift_off && d.bov_area_m2 > 0.0;
        let m_c = if venting {
            let (mut lo, mut hi) = (intake, intake + bov(intake));
            for _ in 0..40 {
                let m = 0.5 * (lo + hi);
                if intake + bov(m) > m {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            0.5 * (lo + hi)
        } else {
            intake
        };
        let (ratio, p02, t02) = exit(m_c);
        // the wastegate opens on the compressor's boost
        let gate = ((p02 - ATM_PA - d.wastegate_pa) / WASTEGATE_SPAN_PA).clamp(0.0, 1.0);
        let gate_area = gate * d.wastegate_area_m2;
        // the turbine inlet's pressure that passes the exhaust through the turbine and the gate
        let p4 = ATM_PA;
        let p03 = if exhaust_m_dot > 0.0 && t03 > 0.0 {
            let pass = |p: f64| {
                turbine_flow(d.turbine_area_m2, p, t03, p4)
                    + nozzle_flow(gate_area, p, t03, p4, GAMMA_EXH)
            };
            let (mut lo, mut hi) = (p4, 50.0 * p4);
            for _ in 0..50 {
                let p = 0.5 * (lo + hi);
                if pass(p) < exhaust_m_dot {
                    lo = p;
                } else {
                    hi = p;
                }
            }
            0.5 * (lo + hi)
        } else {
            p4
        };
        let m_t = turbine_flow(d.turbine_area_m2, p03, t03, p4);
        let p_t = turbine_power(m_t, t03, p03, p4, d.eta_t);
        let p_c = compressor_power(m_c, T_AMB_K, ratio, d.eta_c);
        self.energy_j = (self.energy_j + (d.eta_m * p_t - p_c) * dt).max(0.0);
        TurboState {
            boost_pa: (p02 - d.intercooler_drop_k * m_c * m_c).max(0.1 * ATM_PA),
            boost_t_k: t02 - d.intercooler_effectiveness * (t02 - T_AMB_K),
            turbine_inlet_pa: p03,
            shaft_rpm: omega * 60.0 / (2.0 * std::f64::consts::PI),
            bov_m_dot: if venting { m_c - intake } else { 0.0 },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_compressor_ratio_follows_its_tip_speed() {
        // U = 400 m/s, η 0.75, σ 0.9, 293 K: 1 + 0.75·0.9·160000/(1005·293) = 1.366767,
        // to the 3.5th: 2.984905
        assert!((compressor_ratio(400.0, 0.75, 0.9, 293.0) - 2.984905).abs() < 1e-5);
        assert_eq!(compressor_ratio(0.0, 0.75, 0.9, 293.0), 1.0);
    }

    #[test]
    fn the_work_falls_with_flow_on_backswept_blades() {
        // U 400, 0.1 kg/s, σ 0.9, β₂ 35°, A₂ = π·0.05·0.004 = 6.2832e-4 m², ρ 1.204:
        // 0.9·160000 − 400·0.1·tan 35°/(1.204·6.2832e-4) = 106976.2 J/kg
        let a2 = std::f64::consts::PI * 0.05 * 0.004;
        let w = compressor_work(400.0, 0.1, 0.9, 35f64.to_radians().tan(), a2, 1.204);
        assert!((w - 106976.2).abs() < 0.5, "{w}");
        assert_eq!(compressor_work(0.0, 0.1, 0.9, 0.7, a2, 1.204), 0.0);
    }

    #[test]
    fn compressing_costs_its_isentropic_work_over_the_efficiency() {
        // 0.1 kg/s from 293 K to 2:1 at 0.75: 0.1·1005·293·(2^(2/7) − 1)/0.75 = 8598.91 W
        assert!((compressor_power(0.1, 293.0, 2.0, 0.75) - 8598.91).abs() < 0.05);
    }

    #[test]
    fn the_turbine_swallows_as_a_nozzle() {
        // 1e-3 m², 1000 K, γ 1.33, R 287 (critical ratio 0.5404):
        // choked from 2 bar to 1: A·p0/√(RT0)·√γ·(2/(γ+1))^((γ+1)/2(γ−1)) = 0.251110 kg/s;
        // from 1.5 bar to 1 (ratio 0.667, not choked): 0.181306 kg/s
        assert!((turbine_flow(1e-3, 2e5, 1000.0, 1e5) - 0.251110).abs() < 1e-5);
        assert!((turbine_flow(1e-3, 1.5e5, 1000.0, 1e5) - 0.181306).abs() < 1e-5);
        assert_eq!(turbine_flow(1e-3, 1e5, 1000.0, 1e5), 0.0);
        // 0.1 kg/s at 1000 K, 2:1, η 0.7, c_p = γR/(γ−1) = 1156.70: 12793.66 W
        assert!((turbine_power(0.1, 1000.0, 2e5, 1e5, 0.7) - 12793.66).abs() < 0.05);
    }

    /// Runs `turbo` for `secs` at constant flows; the boost (gauge) and shaft speed each step.
    fn run(
        turbo: &mut Turbo,
        secs: f64,
        ex: f64,
        t03: f64,
        air: f64,
        lift: bool,
    ) -> Vec<TurboState> {
        let dt = 1e-4;
        (0..(secs / dt) as usize)
            .map(|_| turbo.step(ex, t03, air, lift, dt))
            .collect()
    }

    #[test]
    fn the_shaft_spools_up_with_a_lag_and_the_wastegate_holds_the_boost() {
        let def = TurboDef::default();
        let mut t = Turbo::new(&def);
        let s = run(&mut t, 5.0, 0.12, 1100.0, 0.11, false);
        let last = s.last().unwrap();
        let gauge = last.boost_pa + def.intercooler_drop_k * 0.11 * 0.11 - ATM_PA;
        // the gate holds the compressor's boost between its set point and full opening
        assert!(
            (def.wastegate_pa..=def.wastegate_pa + WASTEGATE_SPAN_PA).contains(&gauge),
            "{gauge}"
        );
        // the lag: the shaft alone, under full exhaust from rest, takes a tenth of a second or
        // more to 90 % of its speed (ESTIMATE bounds: the second a turbo engine takes to boost
        // is mostly the engine's own flow building up, Watson & Janota ch. 9)
        let t90 = s
            .iter()
            .position(|x| x.shaft_rpm >= 0.9 * last.shaft_rpm)
            .unwrap();
        let t90 = t90 as f64 * 1e-4;
        assert!((0.1..1.0).contains(&t90), "{t90} s");
        // the turbine is a restriction: its inlet over the outlet's atmosphere
        assert!(
            last.turbine_inlet_pa > ATM_PA + 0.3e5,
            "{}",
            last.turbine_inlet_pa
        );
        // the intercooler: under the compressor's exit temperature, over the air's
        assert!(last.boost_t_k > T_AMB_K && last.boost_t_k < T_AMB_K + 60.0);
        assert!(
            s.iter()
                .all(|x| x.boost_pa.is_finite() && x.shaft_rpm.is_finite())
        );
    }

    #[test]
    fn with_no_exhaust_the_turbo_stays_at_rest() {
        let mut t = Turbo::new(&TurboDef::default());
        let s = run(&mut t, 0.5, 0.0, 293.0, 0.0, false);
        let last = s.last().unwrap();
        assert_eq!(last.shaft_rpm, 0.0);
        assert_eq!(last.turbine_inlet_pa, ATM_PA);
        assert!((last.boost_pa - ATM_PA).abs() < 1e-9);
    }

    #[test]
    fn a_lift_off_under_boost_vents_through_the_blow_off_valve() {
        let def = TurboDef::default();
        let mut t = Turbo::new(&def);
        run(&mut t, 3.0, 0.12, 1100.0, 0.11, false);
        let off = run(&mut t, 1.0, 0.005, 700.0, 0.002, true);
        assert!(off[0].bov_m_dot > 0.0);
        // venting, the boost falls under 0.2 bar within half a second (ESTIMATE bound: boost logs
        // of blow-off events fall to ambient in 0.2–0.5 s)
        let low = off
            .iter()
            .position(|x| x.boost_pa - ATM_PA < 20_000.0)
            .unwrap();
        assert!((low as f64) * 1e-4 < 0.5, "{} s", low as f64 * 1e-4);
        assert!(off.iter().all(|x| x.boost_pa.is_finite()));
        // without a valve nothing vents
        let mut none = Turbo::new(&TurboDef {
            bov_area_m2: 0.0,
            ..def
        });
        run(&mut none, 3.0, 0.12, 1100.0, 0.11, false);
        assert!(
            run(&mut none, 0.1, 0.005, 700.0, 0.002, true)
                .iter()
                .all(|x| x.bov_m_dot == 0.0)
        );
    }

    #[test]
    fn a_turbo_round_trips() {
        let d = TurboDef::default();
        let json: TurboDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: TurboDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d));
    }
}
