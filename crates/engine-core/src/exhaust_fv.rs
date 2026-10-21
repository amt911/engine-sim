//! Exhaust system as non-linear 1D finite-volume pipes ([`crate::fv`]): primaries → 0D
//! junction per collector → collector pipe → muffler expansion chamber (a wide 1D pipe between
//! two small 0D junctions: reactive muffler with frequency-dependent loss, Munjal) → tail pipe →
//! atmosphere. Pulses steepen with amplitude, which the linear waveguides cannot do.

use crate::exhaust::{C_TAIL, ExhaustDef};
use crate::fv::{Boundary, Cons, Pipe1d, Prim};
use crate::gas::{GasVolume, R_AIR};
use crate::units::ATM_PA;

/// Target cell size, m (≈ 7.5 cells per wavelength at 2 kHz in hot gas; cost ∝ 1/dx²).
pub const CELL_M: f64 = 0.04;
const AMBIENT_RHO: f64 = 1.18;
const INIT_T_K: f64 = 600.0;

#[derive(Debug, Clone)]
struct Branch {
    members: Vec<usize>,
    junction: GasVolume,
    /// Collector, muffler can and tail pipe as one variable-area quasi-1D pipe.
    out: Pipe1d,
}

/// Muffler chamber length, m (typical silencer can); its diameter follows from the volume.
const CHAMBER_LENGTH_M: f64 = 0.4;

#[derive(Debug, Clone)]
pub struct ExhaustFv {
    primaries: Vec<Pipe1d>,
    a_primary: f64,
    branches: Vec<Branch>,
    /// Physics steps per FV step, and the FV step.
    pub substeps: usize,
    pub dt_fv: f64,
    /// Physics step this network was built for.
    pub dt: f64,
    count: usize,
    /// Valve mass and energy accumulated since the last FV step, per cylinder (kg, J).
    acc_mass: Vec<f64>,
    acc_energy: Vec<f64>,
    radiated: f64,
    /// Mass that left through the tail outlets in the last physics step, kg.
    vented: f64,
}

/// Keep a 0D volume physical under extreme forcing (explicit coupling can overdraw it).
fn guard(v: &mut GasVolume) {
    let floor = 1e-6 * v.volume_m3;
    if !(v.mass_kg > floor) {
        v.mass_kg = floor;
    }
    let e_min = v.mass_kg * crate::gas::CV * 50.0;
    if !(v.energy_j > e_min) {
        v.energy_j = e_min;
    }
}

fn area(d: f64) -> f64 {
    std::f64::consts::PI * 0.25 * d * d
}

fn pipe(length: f64, init: Prim) -> Pipe1d {
    let mut p = Pipe1d::new(length, (length / CELL_M).round().max(2.0) as usize, init);
    p.floors = (1e-3, 500.0);
    p
}

impl ExhaustFv {
    pub fn new(def: &ExhaustDef, cylinders: usize, dt: f64) -> Self {
        let init = Prim {
            rho: ATM_PA / (R_AIR * INIT_T_K),
            u: 0.0,
            p: ATM_PA,
        };
        let vol = |v: f64| GasVolume::at(ATM_PA, INIT_T_K, v);
        let a_collector = area(def.collector_diameter_m);
        let branches = def
            .groups(cylinders)
            .into_iter()
            .map(|members| {
                let mut out = Pipe1d::segmented(
                    &[
                        (def.collector_length_m, a_collector),
                        (
                            CHAMBER_LENGTH_M,
                            def.muffler_volume_m3.max(1e-5) / CHAMBER_LENGTH_M,
                        ),
                        (def.tail_length_m, area(def.tail_diameter_m)),
                    ],
                    CELL_M,
                    init,
                );
                out.floors = (1e-3, 500.0);
                out.right = Boundary::Open {
                    p: ATM_PA,
                    rho: AMBIENT_RHO,
                };
                Branch {
                    members,
                    // junction: the merge region, about three collector diameters long
                    junction: vol(a_collector * 3.0 * def.collector_diameter_m.max(CELL_M)),
                    out,
                }
            })
            .collect();
        // 0.8 · cell / (|u| + c)_max with 800 m/s for hot, fast exhaust
        let dt_target = 0.8 * CELL_M / 800.0;
        let substeps = ((dt_target / dt).floor() as usize).max(1);
        Self {
            primaries: (0..cylinders)
                .map(|_| pipe(def.primary_length_m, init))
                .collect(),
            a_primary: area(def.primary_diameter_m),
            branches,
            substeps,
            dt_fv: substeps as f64 * dt,
            dt,
            count: 0,
            acc_mass: vec![0.0; cylinders],
            acc_energy: vec![0.0; cylinders],
            radiated: 0.0,
            vented: 0.0,
        }
    }

    /// Pressure and temperature of the gas at cylinder `i`'s port (first primary cell).
    pub fn port_state(&self, i: usize) -> (f64, f64) {
        let p = self.primaries[i].prim(0);
        (p.p, p.p / (p.rho * R_AIR))
    }

    /// The first primary cell as a 0D volume, for the valve flow limiter.
    pub fn port_volume(&self, i: usize) -> GasVolume {
        let pipe = &self.primaries[i];
        let c = pipe.cells[0];
        let v = self.a_primary * pipe.dx;
        let prim = c.to_prim();
        GasVolume {
            mass_kg: c.rho * v,
            energy_j: prim.p / (crate::gas::GAMMA - 1.0) * v,
            volume_m3: v,
        }
    }

    /// Register this physics step's valve flow: `dm` kg into the pipe (negative = back into
    /// the cylinder) carrying specific enthalpy `h` (J/kg).
    pub fn add_valve_flow(&mut self, i: usize, dm: f64, h: f64) {
        self.acc_mass[i] += dm;
        self.acc_energy[i] += dm * h;
    }

    /// Advance one physics step; every `substeps` steps the pipes advance by `dt_fv`.
    /// Returns the radiated source (c·ρu at the tail outlets, summed), held between FV steps.
    pub fn step(&mut self) -> f64 {
        self.count += 1;
        self.vented = 0.0;
        if self.count < self.substeps {
            return self.radiated;
        }
        self.count = 0;
        let dt = self.dt_fv;
        // Valves: the accumulated flow becomes the prescribed flux into each primary's first face.
        for (i, pipe) in self.primaries.iter_mut().enumerate() {
            let a = self.a_primary;
            let cell = pipe.prim(0);
            // Reversion cannot take more than half of what the first cell holds.
            let available = 0.5 * pipe.cells[0].rho * a * pipe.dx;
            let scale = if self.acc_mass[i] < -available {
                available / -self.acc_mass[i]
            } else {
                1.0
            };
            let mass_flux = scale * self.acc_mass[i] / (a * dt);
            let energy_flux = scale * self.acc_energy[i] / (a * dt);
            let rho_up = if mass_flux >= 0.0 {
                cell.rho.max(1e-3)
            } else {
                cell.rho
            };
            pipe.left = Boundary::Flux(Cons {
                rho: mass_flux,
                mom: mass_flux * mass_flux / rho_up + cell.p,
                e: energy_flux,
            });
            self.acc_mass[i] = 0.0;
            self.acc_energy[i] = 0.0;
        }
        let state = |v: &GasVolume| Prim {
            rho: v.mass_kg / v.volume_m3,
            u: 0.0,
            p: v.pressure(),
        };
        for b in &mut self.branches {
            let j = state(&b.junction);
            for &m in &b.members {
                self.primaries[m].right = Boundary::State(j);
            }
            b.out.left = Boundary::State(j);
        }
        for p in &mut self.primaries {
            p.step(dt);
        }
        let mut radiated = 0.0;
        for b in &mut self.branches {
            b.out.step(dt);
            // the junction takes what the faces carried (total enthalpy flux → internal energy)
            for &m in &b.members {
                let f = self.primaries[m].end_flux.1;
                b.junction.mass_kg += f.rho * self.a_primary * dt;
                b.junction.energy_j += f.e * self.a_primary * dt;
            }
            let (a_in, a_out) = b.out.end_areas();
            let fc = b.out.end_flux.0;
            b.junction.mass_kg -= fc.rho * a_in * dt;
            b.junction.energy_j -= fc.e * a_in * dt;
            guard(&mut b.junction);
            let o = b.out.end_flux.1;
            self.vented += o.rho * a_out * dt;
            // ρc·u at the outlet: c·(ρu), the same source quantity as the waveguide model
            radiated += C_TAIL * o.rho;
        }
        self.radiated = radiated;
        radiated
    }

    /// Mass vented to the atmosphere during the last physics step, kg.
    pub fn last_vented(&self) -> f64 {
        self.vented
    }

    /// Total gas mass in the network (pipes, junctions, chambers), kg.
    pub fn total_mass(&self) -> f64 {
        let pipe_mass = |p: &Pipe1d, scale: f64| {
            p.cells
                .iter()
                .zip(&p.area)
                .map(|(c, a)| c.rho * a)
                .sum::<f64>()
                * p.dx
                * scale
        };
        let mut m: f64 = self
            .primaries
            .iter()
            .map(|p| pipe_mass(p, self.a_primary))
            .sum();
        for b in &self.branches {
            m += b.junction.mass_kg + pipe_mass(&b.out, 1.0);
        }
        m
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exhaust::ExhaustModel;

    const DT: f64 = 1.0 / 192_000.0;

    fn def() -> ExhaustDef {
        ExhaustDef {
            model: ExhaustModel::Fv,
            ..ExhaustDef::default()
        }
    }

    #[test]
    fn substeps_respect_the_cfl_limit() {
        let net = ExhaustFv::new(&def(), 4, DT);
        // c + |u| up to ~800 m/s in hot exhaust: dt_fv ≤ 0.8·0.04/800 = 40 µs
        assert!(net.dt_fv <= 40e-6 + 1e-12, "{}", net.dt_fv);
        assert_eq!(net.dt_fv, net.substeps as f64 * DT);
    }

    #[test]
    fn quiet_system_stays_quiet() {
        let mut net = ExhaustFv::new(&def(), 4, DT);
        let mut worst: f64 = 0.0;
        for _ in 0..20_000 {
            worst = worst.max(net.step().abs());
        }
        assert!(worst < 1.0, "spurious radiation {worst}");
        let (p, _) = net.port_state(0);
        assert!((p - ATM_PA).abs() < 50.0, "port pressure drifted to {p}");
    }

    #[test]
    fn mass_is_conserved_through_the_network() {
        let mut net = ExhaustFv::new(&def(), 2, DT);
        let m0 = net.total_mass();
        let mut injected = 0.0;
        // 20 ms of pulses into cylinder 1's primary, then read the books before the gas
        // reaches the open end (≈ 0.7 + 0.25 + 1.5 m at ≥ 340 m/s ≈ 7 ms!) — so close the
        // check by tracking outflow instead: total = initial + injected − vented.
        let mut vented = 0.0;
        for k in 0..4_000 {
            let dm = if k % 400 < 40 { 2e-6 } else { 0.0 };
            net.add_valve_flow(0, dm, 1004.5 * 900.0);
            injected += dm;
            net.step();
            vented += net.last_vented();
        }
        let m1 = net.total_mass();
        assert!(
            ((m1 - m0) - (injected - vented)).abs() < 1e-6 * m0,
            "Δm {} vs {}",
            m1 - m0,
            injected - vented
        );
    }

    #[test]
    fn a_pulse_reaches_the_tail_after_the_travel_time() {
        let mut net = ExhaustFv::new(&def(), 1, DT);
        let mut arrived = None;
        for k in 0..20_000 {
            if k < 40 {
                net.add_valve_flow(0, 5e-7, 1004.5 * 600.0);
            }
            let r = net.step();
            if arrived.is_none() && r.abs() > 50.0 {
                arrived = Some(k);
            }
        }
        // through 0.7 + 0.25 + 1.5 m of 600 K gas (c ≈ 491 m/s) plus the chamber: ≥ 5 ms
        let k = arrived.expect("nothing radiated");
        let t = k as f64 * DT;
        assert!((0.004..0.012).contains(&t), "arrived after {t} s");
    }

    /// Amplitude of the radiated signal at `f` Hz for a sinusoidal mean-plus-ripple valve flow.
    fn tone_through(d: &ExhaustDef, f: f64) -> f64 {
        let mut net = ExhaustFv::new(d, 1, DT);
        let n = (0.25 / DT) as usize;
        let (mut re, mut im, mut count) = (0.0, 0.0, 0.0);
        for k in 0..n {
            let t = k as f64 * DT;
            let dm = (0.05 + 0.02 * (2.0 * std::f64::consts::PI * f * t).sin()) * DT;
            net.add_valve_flow(0, dm, 1004.5 * 900.0);
            let r = net.step();
            if t > 0.1 {
                re += r * (2.0 * std::f64::consts::PI * f * t).cos();
                im += r * (2.0 * std::f64::consts::PI * f * t).sin();
                count += 1.0;
            }
        }
        2.0 * (re * re + im * im).sqrt() / count
    }

    #[test]
    fn the_muffler_passes_firing_frequencies() {
        // A "muffler" as wide as the tail (= straight pipe) vs the LFA's 10 L one, at 600 Hz
        // (V10 firing frequency at 7200 rpm).
        let d = def();
        let straight = ExhaustDef {
            muffler_volume_m3: area(d.tail_diameter_m) * CHAMBER_LENGTH_M,
            ..d
        };
        let muffled = ExhaustDef {
            muffler_volume_m3: 0.010,
            ..def()
        };
        let (a, b) = (
            tone_through(&straight, 600.0),
            tone_through(&muffled, 600.0),
        );
        assert!(a > 0.0);
        assert!(b > 0.1 * a && b < a, "muffled {b} vs straight {a}");
    }

    #[test]
    fn strong_flows_stay_finite() {
        let mut net = ExhaustFv::new(&def(), 4, DT);
        for k in 0..40_000 {
            for c in 0..4 {
                let phase = (k + c * 2_400) % 9_600;
                let dm = if phase < 1_500 {
                    6e-6
                } else if phase < 3_000 {
                    -1e-6
                } else {
                    0.0
                };
                net.add_valve_flow(c, dm, 1004.5 * 1100.0);
            }
            assert!(net.step().is_finite(), "step {k}");
        }
        let (p, t) = net.port_state(0);
        assert!(p.is_finite() && p > 0.0 && t > 0.0, "p {p} T {t}");
    }
}
