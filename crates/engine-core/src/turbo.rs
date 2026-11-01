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
    /// The valve vents back into the compressor's inlet (a factory bypass valve: nothing heard
    /// outside) rather than to the air (an aftermarket blow-off valve's "pssh").
    pub bov_recirculating: bool,
    /// The compressor's inducer (its eye) diameter, m: its sonic flow is the most it passes
    /// ([`choke_flow`]).
    pub inducer_diameter_m: f64,
    /// The charge air's volume between compressor and throttle (intercooler core and pipes),
    /// m³: the boost is its pressure, filled by the compressor and emptied by the engine and
    /// the blow-off valve.
    pub charge_volume_m3: f64,
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
            // carmakers fit recirculating bypass valves (ESTIMATE: the usual factory practice)
            bov_recirculating: true,
            // ESTIMATE: a front-mounted intercooler's core (~3 L) and its pipes
            charge_volume_m3: 0.005,
            // ESTIMATE: 0.7 of the tip, a common inducer/exducer ratio of passenger-car wheels
            inducer_diameter_m: 0.035,
        }
    }
}

/// The inducer's hub, as a fraction of its diameter (ESTIMATE: radial compressors' 0.3–0.4,
/// Dixon & Hall ch. 7).
const INDUCER_HUB_RATIO: f64 = 0.3;

/// The most a compressor whose inducer is `inducer_d` m across passes, kg/s: the inducer choked
/// on axial inflow from ambient air, ṁ = A₁·ρ₀₁·a₀₁·(2/(γ+1))^((γ+1)/(2(γ−1))) (Dixon & Hall,
/// *Fluid Mechanics and Thermodynamics of Turbomachinery*, ch. 7, choking of the inducer), A₁
/// the annulus between the hub and the shroud.
pub fn choke_flow(inducer_d: f64) -> f64 {
    let g = GAMMA_AIR;
    let area = std::f64::consts::PI
        * 0.25
        * inducer_d
        * inducer_d
        * (1.0 - INDUCER_HUB_RATIO * INDUCER_HUB_RATIO);
    let rho = ATM_PA / (R_GAS * T_AMB_K);
    let a = (g * R_GAS * T_AMB_K).sqrt();
    area * rho * a * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)))
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
    /// How the turbine inlet's pressure follows the exhaust flow, dp₀₃/dṁ, Pa per kg/s: what a
    /// pulse meets there ([`crate::waveguide::TurbineJunction`]); 0 with no flow.
    pub turbine_dp_dm: f64,
    /// The charge air's time constant, s: its compliance V/(RT) over the conductances that
    /// drain a swing of its pressure (the compressor's characteristic, the engine's and the
    /// valve's draw) — the low-pass between the throttle's flow and the compressor's.
    pub charge_tau_s: f64,
}

/// A turbocharger's running state: its shaft's kinetic energy and its charge air's pressure.
#[derive(Debug, Clone)]
pub struct Turbo {
    def: TurboDef,
    energy_j: f64,
    boost_pa: f64,
}

impl Turbo {
    pub fn new(def: &TurboDef) -> Self {
        Self {
            def: def.clone(),
            energy_j: 0.0,
            boost_pa: ATM_PA,
        }
    }

    /// One step of `dt` s: `exhaust_m_dot` kg/s at `t03` K into the turbine, `intake_m_dot` kg/s
    /// drawn by the engine, `lift_off` while the blow-off valve is held open (the manifold under
    /// the boost by its spring: the throttle shut on boost).
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
        // The charge air (intercooler and pipes, volume V) holds the boost: the compressor
        // delivers into it the flow its characteristic gives at the pressure it meets, the
        // engine draws from it, a lift-off's blow-off valve vents it (its diaphragm referenced to
        // the manifold, whose vacuum pulls it open; the engine decides when). One implicit step
        // of dp/dt = RT/V·(ṁ_c − ṁ_engine − ṁ_vent), filling and emptying (Heywood §7.6.2;
        // Watson & Janota ch. 9), solved on ṁ_c: more flow, less pressure on backswept blades,
        // less vented, so the residual falls with it. As the compressor's instant delivery, the
        // boost jumped at every step a bang-bang valve opened or shut.
        let venting = lift_off && d.bov_area_m2 > 0.0;
        // the charge at a compressor flow `m`: (pressure after the intercooler, temperature)
        let charge = |m: f64| {
            let (_, p, t) = exit(m);
            (
                p - d.intercooler_drop_k * m * m,
                t - d.intercooler_effectiveness * (t - T_AMB_K),
            )
        };
        let vent = |p: f64, t: f64| {
            if venting {
                nozzle_flow(d.bov_area_m2, p, t, ATM_PA, GAMMA_AIR)
            } else {
                0.0
            }
        };
        let fill = |t: f64| dt * R_GAS * t / d.charge_volume_m3.max(1e-6);
        let residual = |m: f64| {
            let (p, t) = charge(m);
            p - self.boost_pa - fill(t) * (m - intake - vent(p, t))
        };
        // the inducer passes no more than its sonic flow, whatever the pressure behind it
        let m_choke = choke_flow(d.inducer_diameter_m);
        let choked = residual(m_choke) > 0.0;
        let m_c = if residual(0.0) <= 0.0 {
            // the compressor cannot reach the charge's pressure at any flow: it delivers none
            0.0
        } else if choked {
            m_choke
        } else {
            let (mut lo, mut hi) = (0.0, m_choke);
            for _ in 0..50 {
                let m = 0.5 * (lo + hi);
                if residual(m) > 0.0 {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            0.5 * (lo + hi)
        };
        let t_b = charge(m_c).1;
        self.boost_pa = if m_c > 0.0 && !choked {
            charge(m_c).0
        } else {
            // none delivered, or the choked flow whatever the pressure: the engine and the valve
            // empty it against that (implicit on the vent)
            let (mut lo, mut hi) = (0.0, self.boost_pa + fill(t_b) * m_c);
            for _ in 0..50 {
                let p = 0.5 * (lo + hi);
                if p - self.boost_pa - fill(t_b) * (m_c - intake - vent(p, t_b)) > 0.0 {
                    hi = p;
                } else {
                    lo = p;
                }
            }
            0.5 * (lo + hi)
        };
        let vented = vent(self.boost_pa, t_b);
        // A swing δp of the charge's pressure: the compressor delivers δp/R_c less (its speed
        // line's slope; none choked, the line vertical), the engine and the valve draw ṁ/p·δp
        // more (both in proportion to the pressure they draw from)
        let g_compressor = if m_c > 0.0 && !choked {
            let h = 1e-4;
            let slope = (charge(m_c).0 - charge(m_c + h).0) / h;
            if slope > 0.0 { 1.0 / slope } else { 0.0 }
        } else {
            0.0
        };
        let g_draw = (intake + vented) / self.boost_pa.max(1.0);
        let compliance = d.charge_volume_m3 / (R_GAS * t_b);
        let charge_tau_s = (compliance / (g_compressor + g_draw).max(1e-12)).min(1.0);
        // the impeller does its work on what it passes (choked, the pressure it would make past
        // the inducer is lost in the shocks there)
        let ratio = exit(m_c).0;
        // the wastegate opens on the compressor's boost
        let p02 = self.boost_pa + d.intercooler_drop_k * m_c * m_c;
        let gate = ((p02 - ATM_PA - d.wastegate_pa) / WASTEGATE_SPAN_PA).clamp(0.0, 1.0);
        let gate_area = gate * d.wastegate_area_m2;
        // the turbine inlet's pressure that passes the exhaust through the turbine and the gate
        let p4 = ATM_PA;
        let pass = |p: f64| {
            turbine_flow(d.turbine_area_m2, p, t03, p4)
                + nozzle_flow(gate_area, p, t03, p4, GAMMA_EXH)
        };
        let p03 = if exhaust_m_dot > 0.0 && t03 > 0.0 {
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
        // the slope of its characteristic there (a forward difference of a 10⁻⁴ step)
        let turbine_dp_dm = if p03 > p4 {
            let h = 1e-4 * p03;
            h / (pass(p03 + h) - pass(p03)).max(1e-12)
        } else {
            0.0
        };
        let m_t = turbine_flow(d.turbine_area_m2, p03, t03, p4);
        let p_t = turbine_power(m_t, t03, p03, p4, d.eta_t);
        let p_c = compressor_power(m_c, T_AMB_K, ratio, d.eta_c);
        self.energy_j = (self.energy_j + (d.eta_m * p_t - p_c) * dt).max(0.0);
        TurboState {
            boost_pa: self.boost_pa.max(0.1 * ATM_PA),
            boost_t_k: t_b,
            turbine_inlet_pa: p03,
            shaft_rpm: omega * 60.0 / (2.0 * std::f64::consts::PI),
            bov_m_dot: vented,
            turbine_dp_dm,
            charge_tau_s,
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
    fn the_turbine_inlet_rises_with_its_flow_as_a_choked_nozzle() {
        // Choked, the nozzle passes ṁ = A·p₀₃·ψ*/√(R·T₀₃) (Heywood App. C, eq. C.9): its inlet
        // pressure is in proportion to its flow, dp₀₃/dṁ = p₀₃/ṁ — what a pulse meets there
        let mut t = Turbo::new(&TurboDef::default());
        let s = t.step(0.3, 1100.0, 0.0, false, 1e-3);
        assert!(
            ATM_PA / s.turbine_inlet_pa < 0.5,
            "choked: {}",
            s.turbine_inlet_pa
        );
        let want = s.turbine_inlet_pa / 0.3;
        assert!(
            (s.turbine_dp_dm / want - 1.0).abs() < 1e-3,
            "{} vs {want}",
            s.turbine_dp_dm
        );
        // no flow, nothing to resist
        let mut t = Turbo::new(&TurboDef::default());
        assert_eq!(t.step(0.0, 1100.0, 0.0, false, 1e-3).turbine_dp_dm, 0.0);
    }

    #[test]
    fn the_inducer_chokes_at_its_sonic_flow() {
        // Axial inflow, the inducer passes at most ṁ = A₁·ρ₀₁·a₀₁·(2/(γ+1))^((γ+1)/(2(γ−1)))
        // (Dixon & Hall ch. 7): 40 mm, hub at 0.3 of it, air at 1 atm and 300 K →
        // A₁ = 1.14354e-3 m², ρ₀₁ = 1.17683, a₀₁ = 347.189, 0.578704 → 0.270387 kg/s (by hand)
        let m = choke_flow(0.04);
        assert!((m - 0.270_387).abs() < 1e-5, "{m}");
    }

    #[test]
    fn past_its_choke_the_compressor_cannot_hold_the_boost() {
        // Spun up, then an engine drawing 0.5 kg/s against an inducer choking at 0.27: the charge
        // air empties, the boost falls under the atmosphere; an engine drawing 0.2 keeps it
        let run = |draw: f64| {
            let mut t = Turbo::new(&TurboDef {
                inducer_diameter_m: 0.04,
                ..TurboDef::default()
            });
            let mut s = t.step(0.3, 1100.0, 0.2, false, 1e-3);
            for _ in 0..3000 {
                s = t.step(0.3, 1100.0, 0.2, false, 1e-3);
            }
            for _ in 0..500 {
                s = t.step(0.3, 1100.0, draw, false, 1e-3);
            }
            s.boost_pa
        };
        assert!(run(0.2) > ATM_PA + 30e3, "{}", run(0.2));
        assert!(run(0.5) < ATM_PA, "{}", run(0.5));
    }

    #[test]
    fn the_charge_air_drains_its_swings_through_compressor_and_engine() {
        // At rest the compressor's line is the intercooler's alone, p = p_atm − k·ṁ²: at 0.01
        // kg/s a swing meets dp/dṁ = 2k·ṁ = 5000 Pa per kg/s, a conductance of 2e-4; the engine's
        // ṁ/p = 0.01/101325 = 9.87e-8 more. C = V/(RT) = 0.005/(287·300) = 5.807e-8 kg/Pa:
        // τ = C/G = 2.902e-4 s (by hand)
        let mut t = Turbo::new(&TurboDef::default());
        let mut s = t.step(0.0, 0.0, 0.01, false, 1e-3);
        for _ in 0..200 {
            s = t.step(0.0, 0.0, 0.01, false, 1e-3);
        }
        assert!(
            (s.charge_tau_s / 2.902e-4 - 1.0).abs() < 0.02,
            "{}",
            s.charge_tau_s
        );
    }

    #[test]
    fn a_turbo_round_trips() {
        let d = TurboDef {
            charge_volume_m3: 0.0071,
            ..TurboDef::default()
        };
        let json: TurboDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: TurboDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d));
        // a file from before the charge volume takes the default's
        let old: TurboDef = serde_yaml_ng::from_str("blades: 11").unwrap();
        assert_eq!(old.charge_volume_m3, TurboDef::default().charge_volume_m3);
    }
}
