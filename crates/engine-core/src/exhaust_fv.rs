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
/// Upper bound on CFL sub-cycles per FV step (beyond it the state is beyond saving anyway).
const MAX_SUBCYCLES: usize = 16;

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

/// Boundary state of a pipe end at a junction of pressure `p_j` (Benson's constant-pressure
/// junction; Winterbone & Pearson): the characteristic leaving the pipe is kept, so the end
/// sees the junction pressure without being forced to rest (a u = 0 plenum ghost low-passes
/// and reflects the pulses). `sigma` = +1 for a pipe's right end, −1 for its left end. Gas
/// flowing out of the junction carries the junction's density (entropy).
fn junction_ghost(cell: Prim, p_j: f64, rho_j: f64, sigma: f64) -> Prim {
    let g = crate::gas::GAMMA;
    // Bound the expansion/compression a single boundary state may represent.
    let ratio = (p_j / cell.p).clamp(0.05, 20.0);
    let p = cell.p * ratio;
    let c_c = cell.sound_speed();
    let c_b = c_c * ratio.powf((g - 1.0) / (2.0 * g));
    let u = cell.u + sigma * 2.0 / (g - 1.0) * (c_c - c_b);
    let rho = if sigma * u > 0.0 {
        cell.rho * ratio.powf(1.0 / g)
    } else {
        rho_j
    };
    // A junction boundary is at most sonic (choked): |u| ≤ c of the boundary gas.
    let c = (g * p / rho).sqrt();
    Prim {
        rho,
        u: u.clamp(-c, c),
        p,
    }
}

/// Keep a 0D volume physical under extreme forcing (explicit coupling can overdraw it).
fn guard(v: &mut GasVolume) {
    let floor = 1e-6 * v.volume_m3;
    if v.mass_kg.is_nan() || v.mass_kg <= floor {
        v.mass_kg = floor;
    }
    let e_min = v.mass_kg * crate::gas::CV * 50.0;
    if v.energy_j.is_nan() || v.energy_j <= e_min {
        v.energy_j = e_min;
    }
}

fn area(d: f64) -> f64 {
    std::f64::consts::PI * 0.25 * d * d
}

/// Darcy friction factor of exhaust pipes (turbulent flow in drawn steel tube, Re ~ 10⁵;
/// Winterbone & Pearson use 0.02–0.03).
pub const PIPE_FRICTION: f64 = 0.025;

fn pipe(length: f64, diameter: f64, init: Prim) -> Pipe1d {
    let n = (length / CELL_M).round().max(2.0);
    let mut p = Pipe1d::segmented(&[(length, area(diameter))], length / n, init);
    p.floors = (1e-3, 500.0);
    p.friction = PIPE_FRICTION;
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
                out.friction = PIPE_FRICTION;
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
                .map(|_| pipe(def.primary_length_m, def.primary_diameter_m, init))
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
            let rho_up = cell.rho.max(1e-3);
            pipe.left = Boundary::Flux(Cons {
                rho: mass_flux,
                mom: mass_flux * mass_flux / rho_up + cell.p,
                e: energy_flux,
            });
            self.acc_mass[i] = 0.0;
            self.acc_energy[i] = 0.0;
        }
        // Adaptive sub-cycling: deep rarefactions (overrun) can push |u| + c beyond the speed
        // the fixed step was sized for; split the step so every sub-step respects CFL ≤ 0.9.
        let limit = self
            .primaries
            .iter()
            .chain(self.branches.iter().map(|b| &b.out))
            .map(|p| p.max_dt(0.9))
            .fold(f64::INFINITY, f64::min);
        let n = ((dt / limit).ceil() as usize).clamp(1, MAX_SUBCYCLES);
        let h = dt / n as f64;
        let mut radiated = 0.0;
        for _ in 0..n {
            radiated += self.substep(h) / n as f64;
        }
        self.radiated = radiated;
        radiated
    }

    /// One FV sub-step of length `h` with the valve fluxes already set.
    fn substep(&mut self, h: f64) -> f64 {
        let state = |v: &GasVolume| Prim {
            rho: v.mass_kg / v.volume_m3,
            u: 0.0,
            p: v.pressure(),
        };
        for b in &mut self.branches {
            let j = state(&b.junction);
            for &m in &b.members {
                let pipe = &mut self.primaries[m];
                let last = pipe.prim(pipe.cells.len() - 1);
                pipe.right = Boundary::State(junction_ghost(last, j.p, j.rho, 1.0));
            }
            let first = b.out.prim(0);
            b.out.left = Boundary::State(junction_ghost(first, j.p, j.rho, -1.0));
        }
        for p in &mut self.primaries {
            p.step(h);
        }
        let mut radiated = 0.0;
        for b in &mut self.branches {
            b.out.step(h);
            // the junction takes what the faces carried (total enthalpy flux → internal energy)
            for &m in &b.members {
                let f = self.primaries[m].end_flux.1;
                b.junction.mass_kg += f.rho * self.a_primary * h;
                b.junction.energy_j += f.e * self.a_primary * h;
            }
            let (a_in, a_out) = b.out.end_areas();
            let fc = b.out.end_flux.0;
            b.junction.mass_kg -= fc.rho * a_in * h;
            b.junction.energy_j -= fc.e * a_in * h;
            guard(&mut b.junction);
            let o = b.out.end_flux.1;
            self.vented += o.rho * a_out * h;
            // ρc·u at the outlet: c·(ρu), the same source quantity as the waveguide model
            radiated += C_TAIL * o.rho;
        }
        radiated
    }

    /// Mass vented to the atmosphere during the last physics step, kg.
    pub fn last_vented(&self) -> f64 {
        self.vented
    }

    /// Total gas mass in the network (pipes, junctions, chambers), kg.
    pub fn total_mass(&self) -> f64 {
        let pipe_mass = |p: &Pipe1d| {
            p.cells
                .iter()
                .zip(&p.area)
                .map(|(c, a)| c.rho * a)
                .sum::<f64>()
                * p.dx
        };
        let mut m: f64 = self.primaries.iter().map(pipe_mass).sum();
        for b in &self.branches {
            m += b.junction.mass_kg + pipe_mass(&b.out);
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
    fn a_one_to_one_junction_is_transparent() {
        // One primary into a collector of the same diameter, no muffler, same tail: the junction
        // must pass a 600 Hz tone like the equivalent straight pipe (stagnation-plenum junctions
        // low-passed it). Reference: one segmented pipe with the same lengths and flux inlet.
        let d = ExhaustDef {
            primary_diameter_m: 0.05,
            collector_diameter_m: 0.05,
            tail_diameter_m: 0.05,
            muffler_volume_m3: area(0.05) * CHAMBER_LENGTH_M,
            ..def()
        };
        let through_net = tone_through(&d, 600.0);
        let a = area(0.05);
        let init = Prim {
            rho: ATM_PA / (R_AIR * INIT_T_K),
            u: 0.0,
            p: ATM_PA,
        };
        let len = d.primary_length_m + d.collector_length_m + CHAMBER_LENGTH_M + d.tail_length_m;
        let mut straight = Pipe1d::segmented(&[(len, a)], CELL_M, init);
        straight.floors = (1e-3, 500.0);
        straight.right = Boundary::Open {
            p: ATM_PA,
            rho: AMBIENT_RHO,
        };
        let dt = 5.0 * DT;
        let (mut re, mut im, mut count) = (0.0, 0.0, 0.0);
        let mut t = 0.0;
        while t < 0.25 {
            let m = (0.05 + 0.02 * (2.0 * std::f64::consts::PI * 600.0 * t).sin()) / a;
            let cell = straight.prim(0);
            straight.left = Boundary::Flux(Cons {
                rho: m,
                mom: m * m / cell.rho + cell.p,
                e: m * 1004.5 * 900.0,
            });
            straight.step(dt);
            t += dt;
            if t > 0.1 {
                let r = C_TAIL * straight.end_flux.1.rho;
                re += r * (2.0 * std::f64::consts::PI * 600.0 * t).cos();
                im += r * (2.0 * std::f64::consts::PI * 600.0 * t).sin();
                count += 1.0;
            }
        }
        let through_straight = 2.0 * (re * re + im * im).sqrt() / count;
        let ratio = through_net / through_straight;
        assert!(
            (0.6..1.6).contains(&ratio),
            "junction transmits {ratio:.2} of a straight pipe ({through_net} vs {through_straight})"
        );
    }

    /// Riemann invariant J = u + σ·2c/(γ−1) carried by the characteristic leaving the pipe.
    fn invariant(s: Prim, sigma: f64) -> f64 {
        s.u + sigma * 2.0 / (crate::gas::GAMMA - 1.0) * s.sound_speed()
    }

    #[test]
    fn junction_ghost_keeps_the_outgoing_characteristic() {
        // Outflow through either end into a lower junction pressure: the ghost sits at p_j, keeps
        // the invariant leaving the pipe and expands isentropically from the cell (p/ρ^γ kept).
        let g = crate::gas::GAMMA;
        for (sigma, u) in [(1.0, 30.0), (-1.0, -30.0)] {
            let cell = Prim {
                rho: 1.0,
                u,
                p: 1.5e5,
            };
            let b = junction_ghost(cell, 1.2e5, 0.7, sigma);
            assert!((b.p - 1.2e5).abs() < 1e-6, "σ {sigma}: {b:?}");
            assert!((invariant(b, sigma) - invariant(cell, sigma)).abs() < 1e-9);
            let entropy = |s: Prim| s.p / s.rho.powf(g);
            assert!(
                (entropy(b) / entropy(cell) - 1.0).abs() < 1e-12,
                "σ {sigma}: {b:?}"
            );
            assert!(
                sigma * b.u > sigma * cell.u,
                "expansion accelerates the outflow"
            );
        }
    }

    #[test]
    fn gas_entering_a_pipe_from_a_junction_has_the_junction_density() {
        // Right end, flow into the pipe (u < 0), junction above the cell pressure.
        let cell = Prim {
            rho: 1.0,
            u: -30.0,
            p: 1.0e5,
        };
        let b = junction_ghost(cell, 1.3e5, 0.7, 1.0);
        assert_eq!(b.rho, 0.7);
        assert!(b.u < -30.0);
        // At rest nothing leaves the pipe, so the boundary gas is the junction's.
        let rest = Prim { u: 0.0, ..cell };
        assert_eq!(junction_ghost(rest, 1.0e5, 0.7, 1.0).rho, 0.7);
        assert_eq!(junction_ghost(rest, 1.0e5, 0.7, -1.0).rho, 0.7);
    }

    #[test]
    fn junction_ghost_clamps_the_pressure_ratio_and_chokes() {
        let cell = Prim {
            rho: 1.0,
            u: 0.0,
            p: 1.0e5,
        };
        assert!((junction_ghost(cell, 1.0e7, 0.7, 1.0).p - 20.0e5).abs() < 1e-6);
        let b = junction_ghost(cell, 1.0, 0.7, 1.0);
        assert!((b.p - 0.05e5).abs() < 1e-6);
        // The expansion to 5% would be supersonic: the ghost is choked at exactly sonic.
        let c = (crate::gas::GAMMA * b.p / b.rho).sqrt();
        assert!((b.u - c).abs() < 1e-9 * c, "{b:?} c {c}");
    }

    #[test]
    fn guard_floors_mass_and_energy() {
        let mut v = GasVolume {
            mass_kg: f64::NAN,
            energy_j: f64::NAN,
            volume_m3: 2.0,
        };
        guard(&mut v);
        assert_eq!(v.mass_kg, 2.0e-6);
        // 50 K worth of internal energy: m·cv·T
        assert!((v.energy_j - 2.0e-6 * 717.5 * 50.0).abs() < 1e-12);
        let mut neg = GasVolume {
            mass_kg: -1.0,
            energy_j: -1.0,
            volume_m3: 2.0,
        };
        guard(&mut neg);
        assert_eq!(neg.mass_kg, 2.0e-6);
        assert!(neg.energy_j > 0.0);
        // A healthy volume is left alone.
        let ok = GasVolume::at(1.0e5, 600.0, 1.0e-3);
        let mut same = ok;
        guard(&mut same);
        assert_eq!((same.mass_kg, same.energy_j), (ok.mass_kg, ok.energy_j));
        // Finite mass with NaN energy still gets the energy floor.
        let mut half = GasVolume {
            energy_j: f64::NAN,
            ..ok
        };
        guard(&mut half);
        assert_eq!(half.mass_kg, ok.mass_kg);
        assert!(half.energy_j.is_finite());
    }

    #[test]
    fn area_is_the_circle_of_the_diameter() {
        // 50 mm bore: π/4 · 0.05² = 1.9635e-3 m²
        assert!((area(0.05) - 1.963_495e-3).abs() < 1e-9);
    }

    #[test]
    fn a_fresh_port_holds_the_initial_gas() {
        let n = ExhaustFv::new(&def(), 4, DT);
        let (p, t) = n.port_state(2);
        assert!(
            (p - 101_325.0).abs() < 1e-6 && (t - 600.0).abs() < 1e-9,
            "{p} {t}"
        );
        let v = n.port_volume(2);
        let pipe = &n.primaries[2];
        let vol = area(def().primary_diameter_m) * pipe.dx;
        assert!((v.volume_m3 - vol).abs() < 1e-15);
        assert!((v.pressure() - 101_325.0).abs() < 1e-6);
        assert!((v.temperature() - 600.0).abs() < 1e-9);
    }

    #[test]
    fn a_fresh_network_holds_its_geometric_volume_of_gas() {
        // 2 × π/4·0.04²·0.7 primaries + junction + collector + 6 L muffler + 1.5 m tail, at
        // 1 atm / 600 K (ρ = 0.5884 kg/m³), defaults of `ExhaustDef`, all diameters by hand.
        let d = def();
        let disc = |dia: f64| std::f64::consts::PI / 4.0 * dia * dia;
        let v = 2.0 * disc(d.primary_diameter_m) * d.primary_length_m
            + disc(d.collector_diameter_m) * 3.0 * d.collector_diameter_m
            + disc(d.collector_diameter_m) * d.collector_length_m
            + d.muffler_volume_m3
            + disc(d.tail_diameter_m) * d.tail_length_m;
        let m = ExhaustFv::new(&d, 2, DT).total_mass();
        assert!(
            (m / (0.5884 * v) - 1.0).abs() < 0.03,
            "{m} kg vs {}",
            0.5884 * v
        );
    }

    #[test]
    fn the_pipes_advance_once_per_fv_step() {
        // Physics time and pipe time stay in step: dt_fv = substeps · dt, and the radiated value
        // changes exactly once per FV step under a steady flow.
        let mut net = ExhaustFv::new(&def(), 1, DT);
        assert!((net.dt_fv - net.substeps as f64 * DT).abs() < 1e-18);
        assert!(net.substeps > 1);
        // let the flow reach the tail first (≈ 5 ms), ending on an FV step boundary
        for _ in 0..net.substeps * 400 {
            net.add_valve_flow(0, 0.05 * DT, 1004.5 * 900.0);
            net.step();
        }
        let mut updates = 0;
        let mut last = net.step();
        let steps = 70 * net.substeps;
        for _ in 1..steps {
            net.add_valve_flow(0, 0.05 * DT, 1004.5 * 900.0);
            let r = net.step();
            if r != last {
                updates += 1;
            }
            last = r;
        }
        assert!(
            (68..=70).contains(&updates),
            "{updates} updates in {steps} steps"
        );
    }

    #[test]
    fn reversion_takes_at_most_half_of_the_first_cell() {
        let energy = |p: &Pipe1d| {
            p.cells
                .iter()
                .zip(&p.area)
                .map(|(c, a)| c.e * a)
                .sum::<f64>()
                * p.dx
        };
        for (demand, expected) in [(2.0, None), (1e-9, Some(1e-9))] {
            let mut net = ExhaustFv::new(&def(), 1, DT);
            let half = 0.5 * net.port_volume(0).mass_kg;
            let m0 = net.total_mass();
            let e0 = energy(&net.primaries[0]);
            net.add_valve_flow(0, -demand, 1004.5 * 600.0);
            let mut vented = 0.0;
            for _ in 0..net.substeps {
                net.step();
                vented += net.last_vented();
            }
            let taken = m0 - net.total_mass() - vented;
            let want = expected.unwrap_or(half);
            assert!(
                (taken - want).abs() < 1e-6 * want,
                "{demand}: took {taken}, want {want}"
            );
            // the energy is limited with the mass: every kilogram leaves with its h = cp·T
            // (the far end is still at rest after one FV step, so only the valve face moved it)
            let e_taken = e0 - energy(&net.primaries[0]);
            let e_want = want * 1004.5 * 600.0;
            assert!(
                (e_taken - e_want).abs() < 1e-6 * e_want,
                "{demand}: took {e_taken} J, want {e_want} J"
            );
        }
    }

    #[test]
    fn outflow_radiates_a_positive_source() {
        // Gas leaving the tail is ρu > 0: a steady exhaust flow radiates a positive mean.
        let mut net = ExhaustFv::new(&def(), 1, DT);
        let mut sum = 0.0;
        let n = (0.05 / DT) as usize;
        for k in 0..n {
            net.add_valve_flow(0, 0.05 * DT, 1004.5 * 900.0);
            let r = net.step();
            if k > n / 2 {
                sum += r;
            }
        }
        assert!(sum > 0.0, "mean source {sum}");
    }

    #[test]
    fn a_valve_jet_enters_the_primary_at_its_flow_velocity() {
        // 50 g/s into a 40 mm primary of 0.59 kg/m³ gas: G = ṁ/A ≈ 39.8 kg/(m²·s), so the gas at
        // the port moves at about G/ρ — tens of m/s, set by the momentum the jet brings in.
        let mut net = ExhaustFv::new(&def(), 1, DT);
        for _ in 0..(0.002 / DT) as usize {
            net.add_valve_flow(0, 0.05 * DT, 1004.5 * 600.0);
            net.step();
        }
        let c = net.primaries[0].prim(0);
        let g = 0.05 / area(0.04);
        assert!((c.u / (g / c.rho) - 1.0).abs() < 0.1, "{c:?}");
        // and it brings its enthalpy: 600 K stagnation gas keeps the port near 600 K, not cold
        let (_, t) = net.port_state(0);
        assert!((520.0..620.0).contains(&t), "port at {t} K");
    }

    #[test]
    fn junction_ghost_is_subsonic_and_bounded() {
        let cell = Prim {
            rho: 0.5,
            u: 50.0,
            p: 1.0e5,
        };
        // equal pressure: the pipe end passes through unchanged
        let same = junction_ghost(cell, 1.0e5, 0.6, 1.0);
        assert!((same.u - 50.0).abs() < 1e-9 && same.rho == 0.5);
        // a near-vacuum junction must not produce supersonic boundary velocities
        for p_j in [1.0, 100.0, 5e3, 2e6] {
            for sigma in [1.0, -1.0] {
                let g = junction_ghost(cell, p_j, 0.6, sigma);
                assert!(
                    g.u.abs() <= g.sound_speed() * (1.0 + 1e-9),
                    "p_j {p_j} σ {sigma}: {g:?}"
                );
                assert!(g.rho > 0.0 && g.p > 0.0);
            }
        }
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
