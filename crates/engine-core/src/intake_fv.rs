//! Intake as non-linear 1D finite-volume runners ([`crate::fv`]): one runner per cylinder from a
//! trumpet to the intake valve, fed either through **individual throttle bodies** at each trumpet
//! (ITBs, like the LFA) or from a common **plenum** behind a single throttle. Gives intake (ram)
//! tuning and induction sound. Spec: `docs/superpowers/specs/2026-10-22-intake-and-pipe-editor.md`.

use crate::exhaust_fv::{CELL_M, PIPE_FRICTION, area, guard, junction_ghost, throttled_ghost};
use crate::fv::{Boundary, Pipe1d, Prim, flux_boundary};
use crate::gas::{GasVolume, R_AIR};
use crate::units::ATM_PA;
use serde::{Deserialize, Serialize};

pub const AMBIENT_T_K: f64 = 300.0;
/// Speed of sound of ambient air, for the induction-sound source c·(ρu).
pub const C_AIR: f64 = 343.0;
const MAX_SUBCYCLES: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntakeModel {
    /// One 0D plenum straight onto the valves (cheap; the original model).
    #[default]
    ZeroD,
    /// 1D runners with trumpets (and a plenum unless the throttles are individual).
    Fv,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct IntakeDef {
    pub model: IntakeModel,
    pub runner_length_m: f64,
    pub runner_diameter_m: f64,
    /// One throttle per runner at its trumpet (ITB) instead of a common throttle + plenum.
    pub individual_throttles: bool,
    /// Entry loss coefficient of the trumpet (sharp edge ≈ 0.5, bellmouth ≈ 0.05; Blair).
    pub trumpet_loss: f64,
}

impl Default for IntakeDef {
    fn default() -> Self {
        Self {
            model: IntakeModel::ZeroD,
            runner_length_m: 0.3,
            runner_diameter_m: 0.042,
            individual_throttles: false,
            trumpet_loss: 0.1,
        }
    }
}

#[derive(Debug, Clone)]
pub struct IntakeFv {
    runners: Vec<Pipe1d>,
    a_runner: f64,
    individual: bool,
    /// Common plenum (unused with individual throttles).
    plenum: GasVolume,
    /// Discharge-coefficient factor of the trumpet entry, 1/√(1 + k).
    trumpet_cd: f64,
    pub substeps: usize,
    pub dt_fv: f64,
    pub dt: f64,
    count: usize,
    acc_mass: Vec<f64>,
    acc_energy: Vec<f64>,
    radiated: f64,
    inducted: f64,
    inducted_rate: f64,
}

impl IntakeFv {
    pub fn new(def: &IntakeDef, plenum_volume_m3: f64, cylinders: usize, dt: f64) -> Self {
        let init = Prim {
            rho: ATM_PA / (R_AIR * AMBIENT_T_K),
            u: 0.0,
            p: ATM_PA,
        };
        let a_runner = area(def.runner_diameter_m);
        let runners = (0..cylinders)
            .map(|_| {
                let n = (def.runner_length_m / CELL_M).round().max(2.0);
                let mut p = Pipe1d::segmented(
                    &[(def.runner_length_m, a_runner)],
                    def.runner_length_m / n,
                    init,
                );
                p.floors = (1e-3, 500.0);
                p.friction = PIPE_FRICTION;
                p
            })
            .collect();
        // cold air, |u| + c up to ~700 m/s at a choked trumpet: 0.8 · cell / 700
        let dt_target = 0.8 * CELL_M / 700.0;
        let substeps = ((dt_target / dt).floor() as usize).max(1);
        Self {
            runners,
            a_runner,
            individual: def.individual_throttles,
            plenum: GasVolume::at(ATM_PA, AMBIENT_T_K, plenum_volume_m3.max(1e-5)),
            trumpet_cd: 1.0 / (1.0 + def.trumpet_loss.max(0.0)).sqrt(),
            substeps,
            dt_fv: substeps as f64 * dt,
            dt,
            count: 0,
            acc_mass: vec![0.0; cylinders],
            acc_energy: vec![0.0; cylinders],
            radiated: 0.0,
            inducted: 0.0,
            inducted_rate: 0.0,
        }
    }

    /// The runner's last cell (at the valve) as a 0D volume, for the valve flow.
    pub fn port_volume(&self, i: usize) -> GasVolume {
        let pipe = &self.runners[i];
        let c = pipe.cells[pipe.cells.len() - 1];
        let v = self.a_runner * pipe.dx;
        let prim = c.to_prim();
        GasVolume {
            mass_kg: c.rho * v,
            energy_j: prim.p / (crate::gas::GAMMA - 1.0) * v,
            volume_m3: v,
        }
    }

    /// Register this physics step's valve flow: `dm` kg from the runner into the cylinder
    /// (negative = back-flow into the runner) carrying total specific enthalpy `h`.
    pub fn add_valve_flow(&mut self, i: usize, dm: f64, h: f64) {
        self.acc_mass[i] += dm;
        self.acc_energy[i] += dm * h;
    }

    /// Advance one physics step with throttle effective area `throttle_area` (per runner with
    /// ITBs, total for a common throttle). Returns the induction sound source (c·ρu at the
    /// trumpets, or at the throttle for a plenum), held between FV steps.
    pub fn step(&mut self, throttle_area: f64) -> f64 {
        self.count += 1;
        self.inducted = 0.0;
        if self.count < self.substeps {
            return self.radiated;
        }
        self.count = 0;
        let dt = self.dt_fv;
        let a = self.a_runner;
        for (i, pipe) in self.runners.iter_mut().enumerate() {
            let n = pipe.cells.len();
            let cell = pipe.prim(n - 1);
            // what leaves the runner into the cylinder is a +x flux at its right face
            let available = 0.5 * pipe.cells[n - 1].rho * a * pipe.dx;
            let scale = if self.acc_mass[i] > available {
                available / self.acc_mass[i]
            } else {
                1.0
            };
            let mf = scale * self.acc_mass[i] / (a * dt);
            pipe.right = Boundary::Flux(flux_boundary(
                mf,
                scale * self.acc_energy[i] / (a * dt),
                cell,
            ));
            self.acc_mass[i] = 0.0;
            self.acc_energy[i] = 0.0;
        }
        let limit = self
            .runners
            .iter()
            .map(|p| p.max_dt(0.9))
            .fold(f64::INFINITY, f64::min);
        let k = ((dt / limit).ceil() as usize).clamp(1, MAX_SUBCYCLES);
        let h = dt / k as f64;
        let mut radiated = 0.0;
        for _ in 0..k {
            radiated += self.substep(h, throttle_area) / k as f64;
        }
        self.radiated = radiated;
        self.inducted_rate = self.inducted / dt;
        radiated
    }

    fn substep(&mut self, h: f64, throttle_area: f64) -> f64 {
        let a = self.a_runner;
        // The flow cannot enter through more area than the runner offers: a throttle body wider than
        // its runner (or a common throttle) is limited by the duct it feeds.
        let a_eff = throttle_area.max(0.0) * self.trumpet_cd;
        let a_itb = a_eff.min(self.a_runner * self.trumpet_cd);
        let mut radiated = 0.0;
        if self.individual {
            // Throttle body at the trumpet: a throttled characteristic end on the atmosphere
            // (stable; an explicit orifice flux from the cell pressure chatters at the FV rate).
            let psi = a_itb / a;
            let rho_atm = ATM_PA / (R_AIR * AMBIENT_T_K);
            for pipe in &mut self.runners {
                let ghost = throttled_ghost(pipe.prim(0), ATM_PA, rho_atm, -1.0, psi);
                pipe.left = Boundary::State(ghost);
                pipe.step(h);
                self.inducted += pipe.end_flux.0.rho * a * h;
                radiated += C_AIR * pipe.end_flux.0.rho;
            }
        } else {
            let dm = crate::gas::exchange_with_reservoir(
                &mut self.plenum,
                &crate::gas::Reservoir {
                    pressure_pa: ATM_PA,
                    temperature_k: AMBIENT_T_K,
                },
                a_eff,
                h,
            );
            self.inducted -= dm;
            radiated += C_AIR * (-dm / h) / a;
            let pl = Prim {
                rho: self.plenum.mass_kg / self.plenum.volume_m3,
                u: 0.0,
                p: self.plenum.pressure(),
            };
            for pipe in &mut self.runners {
                let first = pipe.prim(0);
                pipe.left = Boundary::State(junction_ghost(first, pl.p, pl.rho, -1.0));
                pipe.step(h);
                let f = pipe.end_flux.0;
                self.plenum.mass_kg -= f.rho * a * h;
                self.plenum.energy_j -= f.e * a * h;
            }
            guard(&mut self.plenum);
        }
        radiated
    }

    /// Manifold pressure as an engine gauge reads it: the plenum, or the runners' mean with ITBs.
    pub fn manifold_pressure(&self) -> f64 {
        if self.individual {
            let mid = |p: &Pipe1d| p.prim(p.cells.len() / 2).p;
            self.runners.iter().map(mid).sum::<f64>() / self.runners.len().max(1) as f64
        } else {
            self.plenum.pressure()
        }
    }

    /// Air that entered from the atmosphere during the last physics step, kg (the whole FV
    /// step's air arrives on the step that advances the pipes: exact for mass bookkeeping).
    pub fn last_inducted(&self) -> f64 {
        self.inducted
    }

    /// Mean induction rate over the last FV step, kg/s, held between FV steps: what a sound
    /// source should read (`last_inducted / dt` is a spike train at the FV rate).
    pub fn inducted_rate(&self) -> f64 {
        self.inducted_rate
    }

    pub fn total_mass(&self) -> f64 {
        let pipes: f64 = self
            .runners
            .iter()
            .map(|p| {
                p.cells
                    .iter()
                    .zip(&p.area)
                    .map(|(c, a)| c.rho * a)
                    .sum::<f64>()
                    * p.dx
            })
            .sum();
        pipes
            + if self.individual {
                0.0
            } else {
                self.plenum.mass_kg
            }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas::CP;

    const DT: f64 = 1.0 / 192_000.0;

    fn def(individual: bool) -> IntakeDef {
        IntakeDef {
            model: IntakeModel::Fv,
            individual_throttles: individual,
            ..IntakeDef::default()
        }
    }

    #[test]
    fn open_and_quiet_stays_at_atmospheric() {
        for individual in [true, false] {
            let mut net = IntakeFv::new(&def(individual), 2e-3, 2, DT);
            for _ in 0..20_000 {
                net.step(1e-3);
            }
            let p = net.manifold_pressure();
            assert!((p - ATM_PA).abs() < 100.0, "individual {individual}: {p}");
        }
    }

    #[test]
    fn steady_suction_is_fed_through_the_throttle() {
        // 0.02 kg/s drawn by one valve; after settling, the throttle inducts the same on average.
        for individual in [true, false] {
            let mut net = IntakeFv::new(&def(individual), 2e-3, 1, DT);
            let mut inducted = 0.0;
            let n = (0.4 / DT) as usize;
            for k in 0..n {
                net.add_valve_flow(0, 0.02 * DT, CP * 300.0);
                net.step(5e-4);
                if k > n / 2 {
                    inducted += net.last_inducted();
                }
            }
            let rate = inducted / (n / 2) as f64 / DT;
            assert!(
                (rate / 0.02 - 1.0).abs() < 0.03,
                "individual {individual}: {rate} kg/s"
            );
        }
    }

    #[test]
    fn the_induction_rate_is_held_smooth_between_fv_steps() {
        // The mass arrives in one lump per FV step; the rate the sound reads must not: it holds
        // the FV step's mean (lump / dt_fv) on every physics step until the next one.
        for individual in [true, false] {
            let mut net = IntakeFv::new(&def(individual), 2e-3, 1, DT);
            assert!(net.substeps > 1);
            for _ in 0..(0.2 / DT) as usize {
                net.add_valve_flow(0, 0.02 * DT, CP * 300.0);
                net.step(5e-4);
            }
            let (mut rates, mut lumps) = (Vec::new(), Vec::new());
            for _ in 0..net.substeps {
                net.add_valve_flow(0, 0.02 * DT, CP * 300.0);
                net.step(5e-4);
                rates.push(net.inducted_rate());
                lumps.push(net.last_inducted());
            }
            // exactly one FV step fell inside the window: from it on, its mean is held
            let fv: Vec<usize> = (0..lumps.len()).filter(|&i| lumps[i] != 0.0).collect();
            assert_eq!(fv.len(), 1, "individual {individual}: {lumps:?}");
            let want = lumps[fv[0]] / net.dt_fv;
            assert!(
                (want / 0.02 - 1.0).abs() < 0.1,
                "individual {individual}: {want}"
            );
            assert!(
                rates[fv[0]..].iter().all(|&r| r == want),
                "{rates:?} vs {want}"
            );
            assert!(rates[..fv[0]].windows(2).all(|w| w[0] == w[1]), "{rates:?}");
        }
    }

    #[test]
    fn pulsed_runners_radiate_without_step_to_step_chatter() {
        // A 3000 rpm intake event (half-sine draw over a third of each 40 ms cycle) rings the
        // runner at its quarter-wave (~340 Hz for 0.25 m): a smooth signal, so its step-to-step
        // difference is far below its swing. An explicit orifice at the trumpet flips the face
        // flux every FV step instead (Nyquist chatter, difference > swing).
        let dt = 1.0 / 48_000.0;
        for (individual, throttle) in [(true, 1.6e-3), (true, 2e-4), (false, 1.6e-2)] {
            let d = IntakeDef {
                runner_length_m: 0.25,
                runner_diameter_m: 0.045,
                ..def(individual)
            };
            let mut net = IntakeFv::new(&d, 8e-3, 1, dt);
            let mut xs = Vec::new();
            for k in 0..(0.6 / dt) as usize {
                let ph = (k as f64 * dt / 0.04).fract();
                let m = if ph < 0.33 {
                    0.08 * (ph / 0.33 * std::f64::consts::PI).sin()
                } else {
                    0.0
                };
                net.add_valve_flow(0, m * dt, CP * 300.0);
                xs.push(net.step(throttle));
            }
            let x = &xs[xs.len() / 2..];
            let mean = x.iter().sum::<f64>() / x.len() as f64;
            let rms = (x.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / x.len() as f64).sqrt();
            let diff =
                (x.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>() / x.len() as f64).sqrt();
            assert!(
                diff < 0.3 * rms,
                "individual {individual}, throttle {throttle}: diff {diff} vs rms {rms}"
            );
        }
    }

    #[test]
    fn a_nearly_closed_throttle_pulls_a_vacuum() {
        for individual in [true, false] {
            let mut net = IntakeFv::new(&def(individual), 2e-3, 1, DT);
            for _ in 0..(0.3 / DT) as usize {
                net.add_valve_flow(0, 0.01 * DT, CP * 300.0);
                net.step(1e-6);
            }
            let p = net.manifold_pressure();
            assert!(p < 0.5 * ATM_PA && p > 0.0, "individual {individual}: {p}");
        }
    }

    #[test]
    fn mass_balances_through_the_runners() {
        let mut net = IntakeFv::new(&def(false), 2e-3, 2, DT);
        let m0 = net.total_mass();
        let (mut drawn, mut inducted) = (0.0, 0.0);
        for k in 0..20_000 {
            let dm = if k % 2_000 < 600 { 4e-7 } else { 0.0 };
            net.add_valve_flow(0, dm, CP * 300.0);
            drawn += dm;
            net.step(5e-4);
            inducted += net.last_inducted();
        }
        let m1 = net.total_mass();
        assert!(
            ((m1 - m0) - (inducted - drawn)).abs() < 1e-6 * m0,
            "Δm {} vs {}",
            m1 - m0,
            inducted - drawn
        );
    }

    #[test]
    fn a_cylinder_drawing_every_step_cannot_create_mass() {
        // Regression: between FV steps the valve drew again and again from the same stale cell;
        // the cylinder gained more than the runner gave up (VE ≫ 1). Network + cylinder mass must
        // change only by what the throttle inducted.
        for individual in [true, false] {
            let mut net = IntakeFv::new(&def(individual), 2e-3, 1, DT);
            let mut cyl = GasVolume::at(0.2e5, 300.0, 5e-4);
            let total0 = net.total_mass() + cyl.mass_kg;
            let mut inducted = 0.0;
            for _ in 0..20_000 {
                let f = crate::gas::plan_exchange(&net.port_volume(0), &cyl, 5e-4, DT, 0.5);
                cyl.mass_kg += f.dm;
                cyl.energy_j += f.dm * f.h;
                net.add_valve_flow(0, f.dm, f.h);
                net.step(5e-4);
                inducted += net.last_inducted();
            }
            let total1 = net.total_mass() + cyl.mass_kg;
            assert!(
                ((total1 - total0) - inducted).abs() < 1e-6 * total0,
                "individual {individual}: {} vs {inducted}",
                total1 - total0
            );
        }
    }

    #[test]
    fn violent_suction_stays_finite() {
        let mut net = IntakeFv::new(&def(true), 2e-3, 4, DT);
        for k in 0..30_000 {
            for c in 0..4 {
                let ph = (k + c * 1_200) % 4_800;
                net.add_valve_flow(c, if ph < 1_200 { 8e-7 } else { -2e-7 }, CP * 400.0);
            }
            assert!(net.step(2e-4).is_finite(), "step {k}");
        }
        assert!(net.manifold_pressure() > 0.0);
    }

    fn runner_energy(net: &IntakeFv) -> f64 {
        let p = &net.runners[0];
        p.cells
            .iter()
            .zip(&p.area)
            .map(|(c, a)| c.e * a * p.dx)
            .sum()
    }

    /// One FV step through a closed ITB (a wall: nothing enters), the valve asking `dm` with
    /// enthalpy `h`: what the runner gave up, (Δm, ΔE).
    fn one_closed_draw(dm: f64, h: f64) -> (f64, f64, f64) {
        let mut net = IntakeFv::new(&def(true), 2e-3, 1, DT);
        let last = net.runners[0].cells.len() - 1;
        let cell_mass = net.runners[0].cells[last].rho * net.a_runner * net.runners[0].dx;
        let (m0, e0) = (net.total_mass(), runner_energy(&net));
        net.add_valve_flow(0, dm * cell_mass, h);
        for _ in 0..net.substeps {
            net.step(0.0);
        }
        assert_eq!(net.last_inducted(), 0.0);
        (net.total_mass() - m0, runner_energy(&net) - e0, cell_mass)
    }

    #[test]
    fn a_valve_takes_at_most_half_the_last_cell_per_fv_step() {
        let h = CP * 300.0;
        // asked for 10 cells: gets half of one, and the energy it carries scales the same
        let (dm, de, cell) = one_closed_draw(10.0, h);
        assert!((dm / cell + 0.5).abs() < 1e-9, "{}", dm / cell);
        assert!((de / (cell * h) + 0.5).abs() < 1e-9, "{}", de / (cell * h));
        // asked for a tenth: gets all of it
        let (dm, de, cell) = one_closed_draw(0.1, h);
        assert!((dm / cell + 0.1).abs() < 1e-9, "{}", dm / cell);
        assert!((de / (cell * h) + 0.1).abs() < 1e-9, "{}", de / (cell * h));
    }

    #[test]
    fn the_induction_source_is_c_times_the_mass_flux_at_the_mouth() {
        // c·ρu with ρu = ṁ / A_runner, averaged over the FV step's sub-cycles (a coarse physics
        // step forces several), whatever the throttle arrangement.
        for individual in [true, false] {
            let dt = 2e-4;
            let mut net = IntakeFv::new(&def(individual), 2e-3, 1, dt);
            let mut seen = 0.0_f64;
            for _ in 0..2_000 {
                net.add_valve_flow(0, 0.02 * dt, CP * 300.0);
                let src = net.step(5e-4);
                let want = C_AIR * net.inducted_rate() / net.a_runner;
                assert!(
                    (src - want).abs() <= 1e-9 * want.abs().max(1e-6),
                    "{src} vs {want}"
                );
                seen = seen.max(src);
            }
            assert!(seen > 1.0, "individual {individual}: no flow ({seen})");
        }
    }

    fn settled_vacuum(trumpet_loss: f64, throttle: f64) -> f64 {
        let d = IntakeDef {
            trumpet_loss,
            ..def(true)
        };
        let mut net = IntakeFv::new(&d, 2e-3, 1, DT);
        for _ in 0..(0.2 / DT) as usize {
            net.add_valve_flow(0, 0.01 * DT, CP * 300.0);
            net.step(throttle);
        }
        ATM_PA - net.manifold_pressure()
    }

    #[test]
    fn the_throttle_flows_its_area_times_the_trumpet_cd() {
        // loss 3 → Cd = 1/√(1 + 3) = 0.5: 4 cm² with Cd 0.5 is 2 cm² with Cd 1
        let half = settled_vacuum(3.0, 4e-4);
        let full = settled_vacuum(0.0, 2e-4);
        assert!(half > 1_000.0, "{half} Pa");
        assert!((half / full - 1.0).abs() < 1e-6, "{half} vs {full}");
    }

    #[test]
    fn a_throttle_wider_than_its_runner_flows_as_the_runner() {
        // Ø42 mm runner = 13.85 cm²; ten times that opens no further than the runner itself
        let a = std::f64::consts::PI * 0.021 * 0.021;
        let at_runner = settled_vacuum(3.0, a);
        let wider = settled_vacuum(3.0, 10.0 * a);
        assert!(at_runner > 1.0, "{at_runner} Pa");
        assert!(
            (wider / at_runner - 1.0).abs() < 1e-9,
            "{wider} vs {at_runner}"
        );
    }

    #[test]
    fn the_port_is_the_last_runner_cell() {
        // 0.3 m runner in 8 cells of 37.5 mm, Ø42 mm: 1.38544e-3 m² · 0.0375 m = 5.1954e-5 m³
        let net = IntakeFv::new(&def(true), 2e-3, 1, DT);
        let v = net.port_volume(0);
        assert!(
            (v.volume_m3 / 5.1954e-5 - 1.0).abs() < 1e-4,
            "{}",
            v.volume_m3
        );
        assert!(
            (v.pressure() / ATM_PA - 1.0).abs() < 1e-9,
            "{}",
            v.pressure()
        );
    }

    #[test]
    fn the_itb_gauge_reads_the_runner_middle() {
        let mut net = IntakeFv::new(&def(true), 2e-3, 1, DT);
        let n = net.runners[0].cells.len();
        for (i, c) in net.runners[0].cells.iter_mut().enumerate() {
            *c = Prim {
                rho: 1.2,
                u: 0.0,
                p: 90_000.0 + 1_000.0 * i as f64,
            }
            .to_cons();
        }
        assert_eq!(n, 8);
        assert!((net.manifold_pressure() - 94_000.0).abs() < 1e-6);
    }
}
