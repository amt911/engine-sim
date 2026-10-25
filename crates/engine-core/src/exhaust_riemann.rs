//! The exhaust with finite-amplitude waves where they are strong: primaries and collectors as
//! Riemann-variable pipes ([`crate::riemann`], Benson 1982), valves and junctions by Benson's
//! boundaries. Spec: `docs/superpowers/specs/2026-10-25-riemann-exhaust.md`.
//!
//! From the collector on, the silencers, catalysts and tail keep the linear waveguide
//! ([`crate::exhaust::after_collector`]): the waves there are an order weaker. A node joins the
//! two at the collector's far end, where the pressure is common and the mass flow passes:
//! the Riemann side's ρ·A^5·U·a_ref·F against the duct's F·(p⁺ − p⁻)/c.

use crate::exhaust::{C_COLLECTOR, C_PRIMARY, C_TAIL, ExhaustDef, LOSS_PER_M};
use crate::riemann::{
    PipeGas, RPipe, ValveFlow, junction, pressure_ratio, valve_inflow, valve_outflow,
};
use crate::units::ATM_PA;
use crate::waveguide::{Duct, OpenEnd};

const KAPPA: f64 = crate::gas::GAMMA;

#[derive(Debug, Clone)]
struct Branch {
    members: Vec<usize>,
    collector: RPipe,
    collector_gas: PipeGas,
    tail: Duct,
    end: OpenEnd,
}

/// The exhaust net on Riemann variables, one step at a time.
#[derive(Debug, Clone)]
pub struct RiemannNet {
    primaries: Vec<RPipe>,
    primary_gas: PipeGas,
    /// What each valve sends into its primary this step (closed: the arriving β back).
    sent: Vec<f64>,
    branches: Vec<Branch>,
    tails: Vec<f64>,
    /// The pressure wave each collector sent into its silencer and tail in the last step.
    into_tails: Vec<f64>,
    /// Scratch for the junctions (no allocation per step).
    arriving: Vec<f64>,
    weights: Vec<f64>,
    out: Vec<f64>,
}

fn gas(a_ref: f64, diameter_m: f64) -> PipeGas {
    PipeGas {
        p_ref: ATM_PA,
        a_ref,
        area_m2: std::f64::consts::PI * 0.25 * diameter_m * diameter_m,
        kappa: KAPPA,
    }
}

fn pipe(length_m: f64, g: &PipeGas, dt: f64) -> RPipe {
    RPipe::new(
        length_m,
        g.a_ref,
        dt,
        KAPPA,
        (1.0 - LOSS_PER_M).powf(length_m),
    )
}

impl RiemannNet {
    pub fn new(def: &ExhaustDef, cylinders: usize, dt: f64) -> Self {
        let primary_gas = gas(C_PRIMARY, def.primary_diameter_m);
        let collector_gas = gas(C_COLLECTOR, def.collector_diameter_m);
        let branches: Vec<Branch> = def
            .groups(cylinders)
            .into_iter()
            .map(|members| Branch {
                members,
                collector: pipe(def.collector_length_m, &collector_gas, dt),
                collector_gas,
                tail: crate::exhaust::after_collector(def, dt, crate::exhaust::LINING_SEGMENT_M),
                end: OpenEnd::new(def.tail_diameter_m, dt),
            })
            .collect();
        Self {
            primaries: (0..cylinders)
                .map(|_| pipe(def.primary_length_m, &primary_gas, dt))
                .collect(),
            primary_gas,
            sent: vec![1.0; cylinders],
            tails: vec![0.0; branches.len()],
            into_tails: vec![0.0; branches.len()],
            branches,
            arriving: Vec::new(),
            weights: Vec::new(),
            out: Vec::new(),
        }
    }

    /// Cylinder `i` at (`p_cyl` Pa, `t_cyl` K) against its primary through a valve of effective
    /// area `cd_area` m² this step: Benson's outflow or backflow boundary, whichever way the
    /// pressures drive it (a shut valve reflects as a closed end). Call before [`Self::step`].
    pub fn valve(&mut self, i: usize, p_cyl: f64, t_cyl: f64, cd_area: f64) -> ValveFlow {
        let g = &self.primary_gas;
        let psi = (cd_area / g.area_m2).min(1.0);
        let arriving = self.primaries[i].at_near();
        let v = if arriving < crate::riemann::sound_speed_ratio(p_cyl / g.p_ref, KAPPA) {
            valve_outflow(p_cyl, t_cyl, psi, arriving, g)
        } else {
            valve_inflow(p_cyl, psi, arriving, g)
        };
        self.sent[i] = v.sent;
        v
    }

    /// Stagnation temperature of the gas at cylinder `i`'s port, K: what a backflow carries in.
    pub fn port_stagnation_t(&self, i: usize) -> f64 {
        let g = &self.primary_gas;
        let t_ref = g.a_ref * g.a_ref / (KAPPA * crate::gas::R_AIR);
        let beta = self.primaries[i].at_near();
        let (a, u) = crate::riemann::state(self.sent[i], beta, KAPPA);
        t_ref * a * a + (u * g.a_ref).powi(2) / (2.0 * crate::gas::CP)
    }

    /// Advance one step; returns the radiated signal summed over the tails (the waveguide's
    /// units: ρc·u at the outlet).
    pub fn step(&mut self) -> f64 {
        let mut radiated = 0.0;
        for ((b, tail_out), into_tail) in self
            .branches
            .iter_mut()
            .zip(self.tails.iter_mut())
            .zip(self.into_tails.iter_mut())
        {
            // the collector's junction: the members' primaries and the collector
            self.arriving.clear();
            self.weights.clear();
            for &m in &b.members {
                self.arriving.push(self.primaries[m].at_far());
                self.weights
                    .push(self.primary_gas.area_m2 / self.primary_gas.a_ref);
            }
            self.arriving.push(b.collector.at_near());
            self.weights
                .push(b.collector_gas.area_m2 / b.collector_gas.a_ref);
            self.out.resize(self.arriving.len(), 0.0);
            junction(&self.arriving, &self.weights, &mut self.out);
            let into_collector = self.out[b.members.len()];
            for (k, &m) in b.members.iter().enumerate() {
                let back = self.out[k];
                self.primaries[m].send(self.sent[m], back);
            }
            // the node between the collector (Riemann) and the duct (linear)
            let (back, into_duct) = interface(
                b.collector.at_far(),
                &b.collector_gas,
                b.tail.at_near(),
                b.tail.end_areas().0,
            );
            let (reflected, out) = b.end.process(b.tail.at_far());
            radiated += out;
            *tail_out = out;
            *into_tail = into_duct;
            b.collector.send(into_collector, back);
            b.tail.send(into_duct, reflected);
        }
        // primaries without a branch (none in practice) still advance
        radiated
    }

    /// Mean outflow Mach number at each tail's open end ([`OpenEnd::set_outflow_mach`]).
    pub fn set_outflow_mach(&mut self, mach: &[f64]) {
        for (b, m) in self.branches.iter_mut().zip(mach) {
            b.end.set_outflow_mach(*m);
        }
    }

    /// Add a small pressure `dp` (Pa) to what valve `i` sends this step — the valve jet's
    /// mixing noise riding out with the pulse. To first order p = p_ref·A^(2κ/(κ−1)), so
    /// δλ = δA = dp·(κ−1)/(2κ·p_ref·A^((κ+1)/(κ−1))), with A the port's.
    pub fn add_to_sent(&mut self, i: usize, dp: f64) {
        let g = &self.primary_gas;
        let a = 0.5 * (self.sent[i] + self.primaries[i].at_near());
        let n = 2.0 * KAPPA / (KAPPA - 1.0);
        self.sent[i] += dp / (n * g.p_ref * a.powf(n - 1.0));
    }

    /// Gauge pressure at cylinder `i`'s port this step, Pa.
    pub fn valve_pressure(&self, i: usize) -> f64 {
        let a = 0.5 * (self.sent[i] + self.primaries[i].at_near());
        self.primary_gas.p_ref * (pressure_ratio(a, KAPPA) - 1.0)
    }

    /// Radiated signal at each tail in the last step.
    pub fn last_tails(&self) -> &[f64] {
        &self.tails
    }

    /// The pressure wave (Pa) each collector sent into its silencer and tail in the last step.
    pub fn last_into_tails(&self) -> &[f64] {
        &self.into_tails
    }

    pub fn primary_area(&self) -> f64 {
        self.primary_gas.area_m2
    }
}

/// The node at the collector's far end: `lambda` arriving from the Riemann collector (gas
/// `g`), `p_minus` (Pa) arriving from the linear duct of area `duct_area` at the tail's sound
/// speed. Common pressure p_ref·(A^(2κ/(κ−1)) − 1) = p⁺ + p⁻ and common mass flow. Returns (β
/// sent back into the collector, p⁺ sent into the duct).
fn interface(lambda: f64, g: &PipeGas, p_minus: f64, duct_area: f64) -> (f64, f64) {
    let k = 0.5 * (KAPPA - 1.0);
    let rho_ref = KAPPA * g.p_ref / (g.a_ref * g.a_ref);
    let gauge = |a: f64| g.p_ref * (pressure_ratio(a, KAPPA) - 1.0);
    // mass toward the duct: Riemann side falls with A, duct side rises with it
    let excess = |a: f64| {
        let m_duct = duct_area * (gauge(a) - 2.0 * p_minus) / C_TAIL;
        let m_pipe = rho_ref * a.powf(2.0 / (KAPPA - 1.0)) * (lambda - a) / k * g.a_ref * g.area_m2;
        m_duct - m_pipe
    };
    let a = crate::exhaust_fv::rising_root(excess, 0.5 * lambda.min(1.0), 1.5 * lambda.max(1.0));
    (2.0 * a - lambda, gauge(a) - p_minus)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exhaust::ExhaustNet;

    const DT: f64 = 1.0 / 192_000.0;

    #[test]
    fn silence_stays_silent() {
        let d = ExhaustDef::default();
        let mut net = RiemannNet::new(&d, 4, DT);
        for _ in 0..5_000 {
            for i in 0..4 {
                net.valve(i, ATM_PA, 900.0, 0.0);
            }
            assert_eq!(net.step(), 0.0);
        }
    }

    #[test]
    fn a_faint_pulse_reaches_the_tail_as_in_the_linear_net() {
        // A valve opening a cylinder barely above an atmosphere: the waves are a few pascals,
        // where Riemann variables and pressure waves agree to first order. The linear net is
        // driven by the very mass flow the Riemann valve passes; the two tails must match.
        let d = ExhaustDef::default();
        let mut rn = RiemannNet::new(&d, 2, DT);
        let mut ln = ExhaustNet::new(&d, 2, DT);
        let (mut a, mut b) = (Vec::new(), Vec::new());
        for k in 0..3_000 {
            let area = if k < 200 { 2e-4 } else { 0.0 };
            let v = rn.valve(0, ATM_PA + 300.0, 900.0, area);
            rn.valve(1, ATM_PA, 900.0, 0.0);
            a.push(rn.step());
            b.push(ln.step(&[v.mass_flow, 0.0]));
        }
        let peak = b.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.0);
        let worst = a
            .iter()
            .zip(&b)
            .map(|(x, y)| (x - y).abs())
            .fold(0.0, f64::max);
        assert!(
            worst < 0.03 * peak,
            "worst difference {worst} of a peak {peak}"
        );
    }

    #[test]
    fn a_blowdown_leaves_the_collector_steeper_than_in_the_linear_net() {
        // A cylinder at 4 bar, 1200 K blowing down through a valve opening and closing as a
        // smooth cam (4 cm² peak, sin² over 2 ms): the pulse leaves the valve smooth and
        // steepens over the primary and collector (x_s ≈ 2γp₀c/((γ+1)·∂p/∂t) ≈ 0.6 m against
        // their 0.95 m). What leaves the collector for the silencer: the Riemann net's front
        // rises faster than the linear net's, driven by the very same valve mass flow.
        let d = ExhaustDef::default();
        let mut rn = RiemannNet::new(&d, 1, DT);
        let mut ln = ExhaustNet::new(&d, 1, DT);
        let (mut a, mut b) = (Vec::new(), Vec::new());
        for k in 0..2_000 {
            let t = k as f64 * DT;
            let area = if t < 2e-3 {
                4e-4 * (std::f64::consts::PI * t / 2e-3).sin().powi(2)
            } else {
                0.0
            };
            let v = rn.valve(0, 4e5, 1200.0, area);
            rn.step();
            a.push(rn.last_into_tails()[0]);
            ln.step(&[v.mass_flow]);
            b.push(ln.last_into_tails()[0]);
        }
        let steepest = |x: &[f64]| x.windows(2).map(|w| w[1] - w[0]).fold(0.0, f64::max);
        let peak = |x: &[f64]| x.iter().cloned().fold(0.0, f64::max);
        let (sr, sl) = (steepest(&a) / peak(&a), steepest(&b) / peak(&b));
        assert!(
            sr > 2.0 * sl,
            "steepest rise per peak: Riemann {sr} vs linear {sl}"
        );
    }

    #[test]
    fn four_cylinders_blowing_down_in_turn_stay_physical() {
        // Regression (2026-10-25): four cylinders at 4 bar, 1100 K, each opening a cam-shaped
        // valve in turn into one 4-into-1 (6000 rpm: 20 ms a cycle, 5 ms apart). A valve barely
        // off its seat sent a vacuum, and within a cycle the ports read 31 MPa. Port pressures
        // must stay those of real exhaust ports: above a near-vacuum, below a few bar.
        let d = ExhaustDef {
            collectors: vec![vec![1, 2, 3, 4]],
            ..ExhaustDef::default()
        };
        let mut rn = RiemannNet::new(&d, 4, DT);
        let (mut highest, mut lowest) = (0.0f64, 0.0f64);
        for k in 0..(0.06 / DT) as usize {
            let t = k as f64 * DT;
            for i in 0..4 {
                let phase = (t - i as f64 * 5e-3).rem_euclid(20e-3);
                let (area, p) = if phase < 6e-3 {
                    (
                        6e-4 * (std::f64::consts::PI * phase / 6e-3).sin().powi(2),
                        4e5 * (1.0 - phase / 6e-3) + 1e5,
                    )
                } else {
                    (0.0, 1e5)
                };
                rn.valve(i, p, 1100.0, area);
                highest = highest.max(rn.valve_pressure(i));
                lowest = lowest.min(rn.valve_pressure(i));
            }
            assert!(rn.step().is_finite());
        }
        assert!(highest < 5e5, "port up to {highest} Pa");
        assert!(lowest > -9.5e4, "port down to {lowest} Pa");
    }
}
