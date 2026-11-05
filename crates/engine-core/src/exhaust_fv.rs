//! Exhaust system with non-linear 1D finite-volume pipes ([`crate::fv`]) where the waves are
//! strong: primaries → 0D junction per collector → collector pipe. From each collector's far end
//! the designed path ([`ExhaustDef::path`]: catalysts, silencers, resonators, tail) is the
//! waveguide's linear duct with its open end, where the waves are an order weaker. Pulses steepen
//! with amplitude, which the linear waveguides cannot do. Spec:
//! `docs/superpowers/specs/2026-10-26-fv-designed-path.md`.

use crate::exhaust::{C_TAIL, ExhaustDef};
use crate::fv::{Boundary, Pipe1d, Prim, flux_boundary};
use crate::gas::{GasVolume, ISENTROPIC_P_POW, ISENTROPIC_RHO_POW, R_AIR};
use crate::units::ATM_PA;
use crate::waveguide::{Duct, OpenEnd};

/// Target cell size, m (≈ 7.5 cells per wavelength at 2 kHz in hot gas; cost ∝ 1/dx²).
pub const CELL_M: f64 = 0.04;
const INIT_T_K: f64 = 600.0;
/// Upper bound on CFL sub-cycles per FV step (beyond it the state is beyond saving anyway).
const MAX_SUBCYCLES: usize = 16;

#[derive(Debug, Clone)]
struct Branch {
    members: Vec<usize>,
    junction: GasVolume,
    /// The collector pipe (FV).
    out: Pipe1d,
    /// The designed path after the collector, linear ([`crate::exhaust::after_collector`]), and
    /// its open end.
    duct: Duct,
    end: OpenEnd,
    /// The wave the collector sends into the duct at the last two pipe steps: the physics steps
    /// between get the straight line from the older to the newer, one pipe step late.
    sent: [f64; 2],
    /// The wave arriving from the duct, read at each pipe step for the collector's end.
    arriving: f64,
}

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
    /// The valve boundary's momentum flux per area, p + ρu², integrated over the physics steps
    /// since the last FV step, and the time it covers ([`Self::add_valve_boundary`]).
    acc_mom: Vec<f64>,
    acc_t: Vec<f64>,
    /// Radiated source at each tail outlet this physics step.
    tails: Vec<f64>,
    /// Mass that left the FV pipes for the designed path in the last physics step, kg.
    vented: f64,
    /// The duct the branches' tails join, its open end or valved silencer
    /// ([`ExhaustDef::merge`]): the same linear path as the waveguide's after the collectors.
    shared: Option<crate::exhaust::Shared>,
    /// The merge junction's scratch (areas, incoming, outgoing): no allocation per step.
    merge_scratch: [Vec<f64>; 3],
}

/// The node at a collector's far end between the FV collector (its last cell `inner`, area
/// `a_fv`) and the linear duct of the designed path (area `a_duct`, gas of impedance `rho_c_duct`
/// and density `rho_duct` at p_atm), with the wave `p_minus` (Pa) arriving from the duct. The
/// pressure is common and the volume flow passes: along the characteristic reaching the end from
/// inside, p + Z·v = p_i + Z·v_i (Z = ρc of the cell, as `fv::open_end`); on the duct,
/// p = p_atm + 2p⁻ + ρc_d·(A_fv/A_d)·v. Returns the ghost state (the interior gas expanded to p on
/// outflow, the duct's gas on inflow) and the wave p⁺ (Pa) sent into the duct.
pub(crate) fn duct_node(
    inner: Prim,
    a_fv: f64,
    p_minus: f64,
    a_duct: f64,
    (rho_c_duct, rho_duct): (f64, f64),
) -> (Prim, f64) {
    let z = inner.rho * inner.sound_speed();
    let z_duct = rho_c_duct * a_fv / a_duct;
    let big_p = inner.p + z * inner.u;
    let u = (big_p - ATM_PA - 2.0 * p_minus) / (z + z_duct);
    let p = (ATM_PA + 2.0 * p_minus + z_duct * u).max(0.05 * ATM_PA);
    let k = 1.0 / crate::gas::GAMMA;
    let rho = if u >= 0.0 {
        inner.rho * (p / inner.p).powf(k)
    } else {
        rho_duct * (p / ATM_PA).powf(k)
    };
    (Prim { rho, u, p }, p - ATM_PA - p_minus)
}

/// The wave a collector sends a share `f` of the way from its last pipe step to the next: on
/// the line between the two, `[previous, last]`, so the duct gets no staircase.
fn between(sent: [f64; 2], f: f64) -> f64 {
    sent[0] + (sent[1] - sent[0]) * f
}

/// Root of `f`, rising through 0 on [`lo`, `hi`] (f(lo) < 0 < f(hi)), by the Illinois variant
/// of regula falsi (Dowell & Jarratt 1971): bracketing like bisection, superlinear like the
/// secant; a dozen evaluations reach 1e-12 where bisection takes forty. Returns the bracket's
/// end if `f` does not change sign.
pub(crate) fn rising_root(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    let (mut f_lo, mut f_hi) = (f(lo), f(hi));
    if f_lo >= 0.0 {
        return lo;
    }
    if f_hi <= 0.0 {
        return hi;
    }
    let mut side = 0;
    // Safeguard: Illinois may crawl when one end's value is many orders of magnitude smaller
    // (a root beside an infinite slope), its secant even landing on that end; then bisect,
    // both when the secant is not strictly inside the bracket and whenever three iterations
    // failed to halve it, so the bracket shrinks at least as fast as one bisection in three
    // (cf. Brent 1973, ch. 4).
    let mut width = hi - lo;
    for it in 0..200 {
        let secant = (lo * f_hi - hi * f_lo) / (f_hi - f_lo);
        let bisect = !(lo < secant && secant < hi) || (it % 3 == 2 && hi - lo > 0.5 * width);
        let x = if bisect { 0.5 * (lo + hi) } else { secant };
        let fx = f(x);
        if fx == 0.0 || (hi - lo) <= 1e-13 * hi.abs().max(1.0) {
            return x;
        }
        if fx < 0.0 {
            lo = x;
            f_lo = fx;
            if side == -1 && !bisect {
                f_hi *= 0.5;
            }
            side = -1;
        } else {
            hi = x;
            f_hi = fx;
            if side == 1 && !bisect {
                f_lo *= 0.5;
            }
            side = 1;
        }
        if it % 3 == 2 {
            width = hi - lo;
        }
    }
    if -f_lo < f_hi { lo } else { hi }
}

/// Boundary state of a pipe end at a junction of pressure `p_j` (Benson's constant-pressure
/// junction; Winterbone & Pearson): the characteristic leaving the pipe is kept, so the end
/// sees the junction pressure without being forced to rest (a u = 0 plenum ghost low-passes
/// and reflects the pulses). `sigma` = +1 for a pipe's right end, −1 for its left end.
/// Outflow expands isentropically from the cell to `p_j`. Inflow starts from the junction at
/// rest: (`p_j`, `rho_j`) is the stagnation state, so the entering gas keeps the junction's
/// entropy and total enthalpy and its static pressure drops by the dynamic head (Benson, The
/// Thermodynamics and Gas Dynamics of IC Engines, vol. 1, §6 inflow boundary).
pub(crate) fn junction_ghost(cell: Prim, p_j: f64, rho_j: f64, sigma: f64) -> Prim {
    throttled_ghost(cell, p_j, rho_j, sigma, f64::INFINITY)
}

/// [`junction_ghost`] through a throttle of `psi` = effective area / pipe area (an ITB at its
/// trumpet; Benson's partially open end): the flow either way is at most what a compressible
/// orifice of that area passes (no pressure recovery past the throat), solved together with
/// the outgoing characteristic, so the boundary stays stable where an explicit orifice flux
/// chatters. `psi` ≥ 1 is the open end; 0 is a wall.
pub(crate) fn throttled_ghost(cell: Prim, p_j: f64, rho_j: f64, sigma: f64, psi: f64) -> Prim {
    let g = crate::gas::GAMMA;
    let k = 0.5 * (g - 1.0);
    let psi = psi.max(0.0);
    // Bound the expansion/compression a single boundary state may represent.
    let ratio = (p_j / cell.p).clamp(0.05, 20.0);
    let p0 = cell.p * ratio;
    let c_c = cell.sound_speed();
    // Pipe side, along the outgoing characteristic on the cell's isentrope, in
    // y = (p/p_c)^((γ−1)/2γ): outflow speed σu = σu_c + (c_c/k)(1 − y).
    let out_speed = |y: f64| sigma * cell.u + c_c / k * (1.0 - y);
    let y_j = ratio.powf((g - 1.0) / (2.0 * g));
    if out_speed(y_j) > 0.0 {
        // Outflow. Open end: expand to p_j, at most sonic (choked).
        let cell_state = |y: f64| {
            let rho = cell.rho * y.powi(ISENTROPIC_RHO_POW);
            let p = cell.p * y.powi(ISENTROPIC_P_POW);
            let c = (g * p / rho).sqrt();
            Prim {
                rho,
                u: sigma * out_speed(y).min(c),
                p,
            }
        };
        let open = cell_state(y_j);
        if psi >= 1.0 {
            return open;
        }
        // Throttled: the face holds back to the pressure where its mass flux is what the
        // orifice passes from the face's stagnation state to p_j; the pipe side's speed falls
        // to 0 at y_hi, where the orifice still passes some: a root in between.
        let excess = |s: Prim| {
            let t = s.p / (s.rho * crate::gas::R_AIR);
            let c2 = g * s.p / s.rho;
            let t0 = t * (1.0 + k * s.u * s.u / c2);
            // (t0/t)^(γ/(γ−1)) = r^3.5
            let r = t0 / t;
            let p0f = s.p * r * r * r * r.sqrt();
            let a = crate::gas::orifice_mass_flow(psi, p0f, t0, p_j);
            s.rho * s.u.abs() - a
        };
        if excess(open) <= 0.0 {
            return open;
        }
        let y_hi = 1.0 + k * sigma * cell.u / c_c;
        // excess falls with y: its negative rises through 0
        let y = rising_root(|y| -excess(cell_state(y)), y_j, y_hi.max(y_j));
        return cell_state(y);
    }
    // Inflow: a contact between the cell gas and the junction gas, with the same p and u on both
    // sides. In x = (p/p₀)^((γ−1)/2γ), with (p₀, ρ₀) the junction as the stagnation state at the
    // junction's entropy, the inflow speeds are
    //   pipe side, outgoing characteristic on the cell's isentrope:
    //     w_c(x) = −σ·u_c − (c_c/k)·(1 − x·(p₀/p_c)^((γ−1)/2γ))   (rises with x),
    //   junction side, adiabatic from rest:  w_j(x) = (c₀/√k)·√(1 − x²)   (0 at x = 1),
    // with k = (γ−1)/2; sonic at x* = 1/√(1+k) (p* = 0.5283 p₀). A throttle passes ψ of the
    // isentropic mass flux at its throat, x_t = max(x, x*): w_a(x) = ψ·x_t^(1/k)·w_j(x_t)/x^(1/k).
    let rho0 = rho_j * (p0 / p_j).powf(1.0 / g);
    let c0 = (g * p0 / rho0).sqrt();
    let w_c = |x: f64| -sigma * cell.u - c_c / k * (1.0 - x * y_j);
    let w_j = |x: f64| c0 / k.sqrt() * (1.0 - x * x).max(0.0).sqrt();
    let x_star = 1.0 / (1.0 + k).sqrt();
    let w_a = |x: f64| {
        if psi >= 1.0 {
            return w_j(x);
        }
        let xt = x.max(x_star);
        psi * (xt / x).powi(ISENTROPIC_RHO_POW) * w_j(xt)
    };
    // A throttled face may sit below p* (choked throat); an open one stops at sonic.
    let x_lo = if psi >= 1.0 { x_star } else { 0.3 };
    let (x, w) = if w_c(x_lo) >= w_a(x_lo) {
        (x_lo, w_a(x_lo))
    } else {
        // w_c − w_a rises monotonically on [x_lo, 1] and changes sign (bracketed: Newton would
        // stall on w_j's infinite slope at x = 1)
        let x = rising_root(|x| w_c(x) - w_a(x), x_lo, 1.0);
        // the pipe side's speed: smooth at x = 1, where w_j is not
        (x, w_c(x).max(0.0))
    };
    // p = p₀·x^(2γ/(γ−1)), ρ = ρ₀·x^(2/(γ−1)) (isentropic from the junction), at most sonic
    let rho = rho0 * x.powi(ISENTROPIC_RHO_POW);
    let p = p0 * x.powi(ISENTROPIC_P_POW);
    let c = (g * p / rho).sqrt();
    Prim {
        rho,
        u: -sigma * w.min(c),
        p,
    }
}

/// Keep a 0D volume physical under extreme forcing (explicit coupling can overdraw it).
pub(crate) fn guard(v: &mut GasVolume) {
    let floor = 1e-6 * v.volume_m3;
    if v.mass_kg.is_nan() || v.mass_kg <= floor {
        v.mass_kg = floor;
    }
    let e_min = v.mass_kg * crate::gas::CV * 50.0;
    if v.energy_j.is_nan() || v.energy_j <= e_min {
        v.energy_j = e_min;
    }
}

pub(crate) fn area(d: f64) -> f64 {
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
    p.acoustic_loss = acoustic_loss();
    p
}

/// The waves' wall loss in the FV pipes, Np/m: the waveguide's [`crate::exhaust::LOSS_PER_M`]
/// (Kirchhoff's boundary layers at the firing orders), so the two models wear a wave alike.
fn acoustic_loss() -> f64 {
    -(1.0 - crate::exhaust::LOSS_PER_M).ln()
}

impl ExhaustFv {
    /// Step every pipe of the network with `scheme`.
    pub fn with_scheme(mut self, scheme: crate::fv::Scheme) -> Self {
        for p in self
            .primaries
            .iter_mut()
            .chain(self.branches.iter_mut().map(|b| &mut b.out))
        {
            p.scheme = scheme;
        }
        self
    }

    pub fn new(def: &ExhaustDef, cylinders: usize, dt: f64) -> Self {
        Self::with_memo(
            def,
            cylinders,
            dt,
            &mut crate::silencer2d::EqMemo::default(),
        )
    }

    /// [`ExhaustFv::new`], its silencers' equalizers taken from `memo` when designed before.
    pub fn with_memo(
        def: &ExhaustDef,
        cylinders: usize,
        dt: f64,
        memo: &mut crate::silencer2d::EqMemo,
    ) -> Self {
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
            .enumerate()
            .map(|(bi, members)| {
                let mut out =
                    Pipe1d::segmented(&[(def.collector_length(bi), a_collector)], CELL_M, init);
                out.floors = (1e-3, 500.0);
                out.friction = PIPE_FRICTION;
                out.acoustic_loss = acoustic_loss();
                Branch {
                    members,
                    // junction: the merge region, about three collector diameters long
                    junction: vol(a_collector * 3.0 * def.collector_diameter_m.max(CELL_M)),
                    out,
                    duct: crate::exhaust::after_collector(
                        def,
                        dt,
                        crate::exhaust::LINING_SEGMENT_M,
                        memo,
                    ),
                    end: OpenEnd::new(def.outlet_diameter(), dt),
                    sent: [0.0; 2],
                    arriving: 0.0,
                }
            })
            .collect::<Vec<Branch>>();
        let shared = crate::exhaust::Shared::new(
            def,
            branches.len(),
            dt,
            crate::exhaust::LINING_SEGMENT_M,
            memo,
        );
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
            acc_mom: vec![0.0; cylinders],
            acc_t: vec![0.0; cylinders],
            tails: vec![0.0; def.groups(cylinders).len()],
            vented: 0.0,
            shared,
            merge_scratch: Default::default(),
        }
    }

    /// Pressure and temperature of the gas at cylinder `i`'s port (first primary cell).
    pub fn port_state(&self, i: usize) -> (f64, f64) {
        let p = self.primaries[i].prim(0);
        (p.p, p.p / (p.rho * R_AIR))
    }

    /// The primaries' cross-section, m².
    pub fn primary_area(&self) -> f64 {
        self.a_primary
    }

    /// The incoming Riemann variable at cylinder `i`'s valve from the first primary cell, referred
    /// to the cylinder's pressure `p_cyl` ([`crate::valve_boundary::lambda_in`]), and that cell's
    /// temperature, K.
    pub fn port_lambda(&self, i: usize, p_cyl: f64) -> (f64, f64) {
        let c = self.primaries[i].prim(0);
        let lambda = crate::valve_boundary::lambda_in(p_cyl, c.p, c.u, c.sound_speed());
        (lambda, c.p / (c.rho * R_AIR))
    }

    /// The first primary cell as a 0D volume, for the valve flow limiter.
    pub fn port_volume(&self, i: usize) -> GasVolume {
        let pipe = &self.primaries[i];
        let c = pipe.cells[0];
        let v = self.a_primary * pipe.dx;
        let prim = c.to_prim();
        // Plus what the valve has pushed since the last FV step (its enthalpy as internal
        // energy, as the junctions take it): against the stale cell alone the valve overshoots
        // and the flow flips direction every physics step.
        GasVolume {
            mass_kg: (c.rho * v + self.acc_mass[i]).max(1e-9),
            energy_j: (prim.p / (crate::gas::GAMMA - 1.0) * v + self.acc_energy[i]).max(1e-9),
            volume_m3: v,
        }
    }

    /// Register this physics step's valve flow: `dm` kg into the pipe (negative = back into
    /// the cylinder) carrying specific enthalpy `h` (J/kg).
    pub fn add_valve_flow(&mut self, i: usize, dm: f64, h: f64) {
        self.acc_mass[i] += dm;
        self.acc_energy[i] += dm * h;
    }

    /// Register this physics step's valve boundary (`p` + ρu², per area, for `dt` s): the
    /// first face then carries the boundary's own pressure instead of rebuilding it from a
    /// first cell several physics steps old — against that stale cell a strong temperature
    /// contrast across the valve made the flow flip every few steps (FINDINGS, 2026-10-27).
    pub fn add_valve_boundary(&mut self, i: usize, momentum_flux: f64, dt: f64) {
        self.acc_mom[i] += momentum_flux * dt;
        self.acc_t[i] += dt;
    }

    /// Advance one physics step: every `substeps` steps the FV pipes advance by `dt_fv`; the
    /// designed paths every step. Returns the radiated source summed over the tails.
    pub fn step(&mut self) -> f64 {
        self.count += 1;
        self.vented = 0.0;
        if self.count >= self.substeps {
            self.count = 0;
            self.pipe_step();
        }
        // the designed paths: what each collector sends, on its line between pipe steps
        let f = self.count as f64 / self.substeps as f64;
        let mut radiated = 0.0;
        match &mut self.shared {
            None => {
                for (b, tail) in self.branches.iter_mut().zip(self.tails.iter_mut()) {
                    let sent = between(b.sent, f);
                    let (reflected, out) = b.end.process(b.duct.at_far());
                    b.duct.send(sent, reflected);
                    *tail = out;
                    radiated += out;
                }
            }
            Some(sh) => {
                // the tails' far ends join the shared duct as on the waveguide
                let [areas, incoming, outgoing] = &mut self.merge_scratch;
                sh.solve_merge(
                    self.branches
                        .iter()
                        .map(|b| (b.duct.end_areas().1, b.duct.at_far())),
                    areas,
                    incoming,
                    outgoing,
                );
                let n = self.branches.len();
                radiated = sh.radiate(outgoing[n], 0.0, self.dt);
                let share = radiated / n.max(1) as f64;
                for ((b, tail), back) in self
                    .branches
                    .iter_mut()
                    .zip(self.tails.iter_mut())
                    .zip(outgoing.iter())
                {
                    let sent = between(b.sent, f);
                    b.duct.send(sent, crate::exhaust::clamp_wave(*back));
                    *tail = share;
                }
            }
        }
        radiated
    }

    /// The engine speed, rpm, that a bypass valve follows (it opens above its `open_rpm`).
    pub fn set_engine_speed(&mut self, rpm: f64) {
        if let Some(sh) = &mut self.shared {
            sh.set_engine_speed(rpm);
        }
    }

    /// A bypass valve's opening fraction (0 shut … 1 open), if the exhaust has one.
    pub fn bypass_opening(&self) -> Option<f64> {
        self.shared.as_ref()?.bypass_opening()
    }

    /// Mean outflow Mach number at each tail's open end ([`OpenEnd::set_outflow_mach`]).
    pub fn set_outflow_mach(&mut self, mach: &[f64]) {
        if let Some(sh) = &mut self.shared {
            sh.set_outflow_mach(mach);
        }
        for (b, m) in self.branches.iter_mut().zip(mach) {
            b.end.set_outflow_mach(*m);
        }
    }

    /// One FV step of `dt_fv`, with the valves' accumulated flows.
    fn pipe_step(&mut self) {
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
            let mut flux = flux_boundary(mass_flux, energy_flux, cell);
            if self.acc_t[i] > 0.0 {
                // the momentum the valve's boundary carried, on average over the open steps
                flux.mom = self.acc_mom[i] / self.acc_t[i];
            }
            pipe.left = Boundary::Flux(flux);
            self.acc_mass[i] = 0.0;
            self.acc_energy[i] = 0.0;
            self.acc_mom[i] = 0.0;
            self.acc_t[i] = 0.0;
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
        for b in &mut self.branches {
            b.arriving = b.duct.at_near();
            b.sent[0] = b.sent[1];
        }
        for _ in 0..n {
            self.substep(h);
        }
    }

    /// Radiated source at each tail outlet this physics step; they sum to `step`'s.
    pub fn last_tails(&self) -> &[f64] {
        &self.tails
    }

    /// One FV sub-step of length `h` with the valve fluxes already set.
    fn substep(&mut self, h: f64) {
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
            // the collector's far end against the designed path
            let last = b.out.prim(b.out.cells.len() - 1);
            let (_, a_fv) = b.out.end_areas();
            let duct_gas = (
                crate::gas::GAMMA * ATM_PA / C_TAIL,
                crate::exhaust::tail_reference_density(),
            );
            let (ghost, sent) = duct_node(last, a_fv, b.arriving, b.duct.end_areas().0, duct_gas);
            b.out.right = Boundary::State(ghost);
            b.sent[1] = sent;
        }
        for p in &mut self.primaries {
            p.step(h);
        }
        for b in self.branches.iter_mut() {
            b.out.step(h);
            // the junction takes what the faces carried (total enthalpy flux → internal energy)
            for &m in &b.members {
                let f = self.primaries[m].end_flux.1;
                b.junction.mass_kg += f.rho * self.a_primary * h;
                b.junction.energy_j += f.e * self.a_primary * h;
            }
            let (a_in, _) = b.out.end_areas();
            let fc = b.out.end_flux.0;
            b.junction.mass_kg -= fc.rho * a_in * h;
            b.junction.energy_j -= fc.e * a_in * h;
            guard(&mut b.junction);
        }
        for b in &self.branches {
            let (_, a_out) = b.out.end_areas();
            self.vented += b.out.end_flux.1.rho * a_out * h;
        }
    }

    /// Mass vented to the atmosphere during the last physics step, kg.
    pub fn last_vented(&self) -> f64 {
        self.vented
    }

    /// Total gas mass in the network (pipes, junctions, chambers), kg.
    /// The pipes' gas for the 3D view, as [`crate::exhaust::ExhaustNet::probe`]: the primaries,
    /// then each collector. Here each pipe's cells are read directly: the temperature of its
    /// gas as a whole, Σp/(R·Σρ), and the speed its mass moves at, Σρu/Σρ (this instant, not a
    /// cycle's mean). No wall model runs on the FV pipes.
    pub fn probe(&self, out: &mut Vec<crate::exhaust::PipeState>) {
        out.clear();
        let gas = |pipe: &Pipe1d| {
            let (mut p, mut rho, mut mom) = (0.0, 0.0, 0.0);
            for c in &pipe.cells {
                let w = c.to_prim();
                p += w.p;
                rho += w.rho;
                mom += w.rho * w.u;
            }
            let rho = rho.max(f64::MIN_POSITIVE);
            crate::exhaust::PipeState {
                temperature_k: p / (crate::gas::R_AIR * rho),
                velocity_m_s: mom / rho,
                wall_k: None,
            }
        };
        out.extend(self.primaries.iter().map(gas));
        out.extend(self.branches.iter().map(|b| gas(&b.out)));
    }

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
    use crate::exhaust::chamber_passage;
    use crate::gas::orifice_mass_flow;
    use crate::lining::Packing;
    use approx::assert_relative_eq;

    const DT: f64 = 1.0 / 192_000.0;

    fn def() -> ExhaustDef {
        ExhaustDef {
            model: ExhaustModel::Fv,
            ..ExhaustDef::default()
        }
    }

    #[test]
    fn the_collector_end_passes_pressure_and_volume_flow_to_the_duct() {
        // Hot gas at rest at p_atm (ρ 0.4: c = 595.5 m/s, Z = 238.2) meets a 1000 Pa wave from a
        // duct of ρc 300 and half its area: v = −2000/(238.2 + 600) = −2.386 m/s, p = p_atm +
        // 568.4 Pa, p⁺ = −431.6 Pa (hand); the duct's gas comes in
        let inner = Prim {
            rho: 0.4,
            u: 0.0,
            p: ATM_PA,
        };
        let (g, p_plus) = duct_node(inner, 2e-3, 1000.0, 1e-3, (300.0, 0.6));
        assert!((g.u + 2.386).abs() < 0.005, "{g:?}");
        assert!((g.p - (ATM_PA + 568.4)).abs() < 0.5, "{g:?}");
        assert!((p_plus + 431.6).abs() < 0.5, "{p_plus}");
        assert!(
            (g.rho - 0.6 * (g.p / ATM_PA).powf(1.0 / 1.4)).abs() < 1e-9,
            "{g:?}"
        );
        // outflow carries the cell's own gas, expanded to the node's pressure
        // a rarefaction deeper than the gas can follow: the node holds 5 % of p_atm (u =
        // 2e6/838.2 = 2386 m/s gives p = p_atm − 2e6 + 600·2386 < 0)
        let (g, _) = duct_node(inner, 2e-3, -1e6, 1e-3, (300.0, 0.6));
        assert!((g.p - 0.05 * ATM_PA).abs() < 1e-6, "{g:?}");
        let out = Prim { u: 50.0, ..inner };
        let (g, _) = duct_node(out, 2e-3, 0.0, 1e-3, (300.0, 0.6));
        assert!(g.u > 0.0 && (g.rho - 0.4 * (g.p / ATM_PA).powf(1.0 / 1.4)).abs() < 1e-9);
    }

    #[test]
    fn the_collector_sends_on_a_line_between_pipe_steps() {
        assert_eq!(between([1.0, 3.0], 0.0), 1.0);
        assert_eq!(between([1.0, 3.0], 0.25), 1.5);
        assert_eq!(between([1.0, 3.0], 1.0), 3.0);
    }

    #[test]
    fn a_tails_outflow_mach_reaches_its_open_end() {
        // Mach 0.5 out of the tail: the end reflects (1 − 0.5)/(1 + 0.5) = 1/3 of what it did
        // at rest (Munt 1977)
        let d = ExhaustDef::default();
        let mut still = ExhaustFv::new(&d, 1, DT);
        let mut moving = ExhaustFv::new(&d, 1, DT);
        moving.set_outflow_mach(&[0.5]);
        let (a, _) = still.branches[0].end.process(1.0);
        let (b, _) = moving.branches[0].end.process(1.0);
        assert!((b / a - 1.0 / 3.0).abs() < 1e-12, "{a} {b}");
    }

    /// Amplitude radiated at `f` Hz from a small sinusoidal flow at the first primary (mean
    /// `mean`, ripple 0.5 g/s: the linear regime), by the FV net (`fv`) or the waveguide's.
    fn tone_from(d: &ExhaustDef, cylinders: usize, f: f64, fv: bool, mean: f64) -> f64 {
        let mut net = ExhaustFv::new(d, cylinders, DT);
        let mut wg = crate::exhaust::ExhaustNet::with_lining(
            d,
            cylinders,
            DT,
            crate::exhaust::LINING_SEGMENT_M,
        );
        let mut flows = vec![0.0; cylinders];
        let w = 2.0 * std::f64::consts::PI * f;
        let n = (0.3 / DT) as usize;
        let (mut re, mut im, mut count) = (0.0, 0.0, 0.0);
        for k in 0..n {
            let t = k as f64 * DT;
            let mdot = mean + 0.0005 * (w * t).sin();
            // the waveguide's source is reference-gas mass: at 896 K, the gas's own
            let r = if fv {
                net.add_valve_flow(0, mdot * DT, 1004.5 * 896.0);
                net.step()
            } else {
                flows[0] = mdot;
                wg.step(&flows)
            };
            if t > 0.15 {
                re += r * (w * t).cos();
                im += r * (w * t).sin();
                count += 1.0;
            }
        }
        2.0 * (re * re + im * im).sqrt() / count
    }

    #[test]
    fn in_the_linear_regime_the_fv_net_sounds_like_the_waveguide() {
        // Small waves: FV's gas dynamics reduce to linear acoustics, the waveguide's. With the
        // same designed path after the collectors (the LFA's: catalyst and tail) and the same wall
        // loss, the two must agree by bands within the gap their gas temperatures leave: FV
        // computes them (the nine shut primaries stay at 600 K), the waveguide fixes 600 and
        // 560 m/s. Measured +4.9, +5.9, −3.4 dB (FV's 4 cm cells wear the highs); its own can and
        // no wall loss left it +6.7 dB at the bottom.
        // the same pipes in both models: the FV net keeps the banks apart, so no merge; the
        // lengths and the 60 mm tail the figures above were measured with (the model gap, not
        // the preset, is under test)
        let mut d = crate::exhaust::ExhaustDef {
            merge: None,
            primary_length_m: 0.675,
            collector_length_m: 0.3,
            collector_extra_length_m: vec![0.0, 0.10],
            primary_length_deviation_m: Vec::new(),
            ..crate::engine::EngineDef::lfa_v10().exhaust
        };
        for seg in &mut d.after_collector {
            if let crate::exhaust::Segment::Pipe {
                length_m,
                diameter_m,
            } = seg
            {
                (*length_m, *diameter_m) = (1.98, 0.060);
            }
        }
        let bands: [&[f64]; 3] = [
            &[60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0],
            &[300.0, 400.0, 500.0, 600.0, 700.0, 800.0],
            &[900.0, 1000.0, 1100.0, 1200.0, 1300.0, 1400.0, 1500.0],
        ];
        for band in bands {
            let e = |fv: bool| {
                band.iter()
                    .map(|&f| tone_from(&d, 10, f, fv, 0.002).powi(2))
                    .sum::<f64>()
            };
            let apart = 10.0 * (e(true) / e(false)).log10();
            assert!(
                apart.abs() < 6.0,
                "{band:?}: FV {apart:+.1} dB from the waveguide"
            );
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
    fn a_junction_at_rest_gives_its_own_pressure() {
        // Captured from a run: the junction 3e-11 Pa below the cell, the gas at rest to
        // round-off. Rounding sent it down the inflow branch, whose root sits within 1e-16 of
        // x = 1, where the junction-side speed has an infinite slope: the root finder ran out of
        // iterations and handed back the middle of its bracket, 74 kPa instead of 101 kPa.
        let cell = Prim {
            rho: 0.588_414_634_146_372_5,
            u: 1.221_604_656_179_534_5e-26,
            p: 101_325.000_000_000_2,
        };
        let g = junction_ghost(cell, 101_325.000_000_000_17, 0.588_414_634_146_346_9, -1.0);
        assert!((g.p - 101_325.0).abs() < 1e-3, "{g:?}");
        assert!(g.u.abs() < 1e-6, "{g:?}");
    }

    #[test]
    fn rising_root_converges_beside_an_infinite_slope() {
        // the junction inflow of the case above: the pipe's characteristic line minus the
        // junction's square-root speed, f(x) = 1.2e-26 + 2431.6·(x − 1) − 1087·√(1 − x²) on
        // [0.9129, 1]: f(lo) ≈ −655 against f(hi) = 1.2e-26, and the root within 1e-16 of 1
        let f = |x: f64| 1.2e-26 + 2431.6 * (x - 1.0) - 1087.0 * (1.0 - x * x).max(0.0).sqrt();
        let calls = std::cell::Cell::new(0);
        let x = rising_root(
            |x| {
                calls.set(calls.get() + 1);
                f(x)
            },
            0.9129,
            1.0,
        );
        assert!((x - 1.0).abs() < 1e-12, "{x}");
        // the secant lands on hi itself (f(hi) is 1e-28 of f(lo)), so only bisection moves the
        // bracket: 0.087 down to 1e-13 is 40 halvings, within 50 evaluations, not the 200 cap
        assert!(calls.get() <= 50, "{} evaluations", calls.get());
        // mirrored, the secant lands on lo instead: −f(2 − x) on [1, 1.0871]
        calls.set(0);
        let x = rising_root(
            |x| {
                calls.set(calls.get() + 1);
                -f(2.0 - x)
            },
            1.0,
            1.0871,
        );
        assert!((x - 1.0).abs() < 1e-12, "{x}");
        assert!(calls.get() <= 50, "{} evaluations", calls.get());
        // and an ordinary root to full precision: x² − 2 on [1, 2]
        assert_relative_eq!(
            rising_root(|x| x * x - 2.0, 1.0, 2.0),
            std::f64::consts::SQRT_2,
            max_relative = 1e-12
        );
    }

    #[test]
    fn still_gas_stays_still_under_round_off() {
        // Round-off (1e-14 of the energy, like the last bit of a real run) in still gas must not
        // grow. With the 0D junction a step behind the pipes it did: a 1.25 kHz junction mode
        // went from 1e-9 Pa to 100 Pa in 0.06 s, the junction's lag acting as negative damping.
        let mut net = ExhaustFv::new(&def(), 4, DT);
        for b in net.branches.iter_mut() {
            for (i, c) in b.out.cells.iter_mut().enumerate() {
                c.e *= 1.0 + 1e-14 * ((i * 7919 % 13) as f64 - 6.0);
            }
        }
        let mut worst: f64 = 0.0;
        for _ in 0..40_000 {
            worst = worst.max(net.step().abs());
        }
        assert!(worst < 1e-3, "round-off grew to {worst}");
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

    #[test]
    fn the_packing_leaves_at_least_the_tail_bore_open() {
        // 10 L over 0.4 m: can area 0.025 m², radius 89.2 mm; 25 mm of packing leaves a
        // 64.2 mm passage (0.012951 m², perimeter 0.40345 m)
        let tail = area(0.05);
        let (a, p, k) = chamber_passage(0.010, Packing::default(), tail);
        assert_relative_eq!(a, 0.012_951, max_relative = 1e-3);
        assert_relative_eq!(p, 0.403_45, max_relative = 1e-3);
        assert_relative_eq!(k.thickness_m, 0.025, max_relative = 1e-9);
        // a 1 L can is only 28.2 mm in radius: the packing shrinks to keep the 25 mm tail bore
        let (a, _, k) = chamber_passage(0.001, Packing::default(), tail);
        assert_relative_eq!(a, tail, max_relative = 1e-9);
        assert_relative_eq!(k.thickness_m, 0.028_209 - 0.025, max_relative = 1e-3);
        // no packing: the whole can
        let bare = Packing {
            thickness_m: 0.0,
            ..Packing::default()
        };
        assert_relative_eq!(
            chamber_passage(0.010, bare, tail).0,
            0.025,
            max_relative = 1e-9
        );
    }

    #[test]
    fn a_one_to_one_junction_is_transparent() {
        // One primary into a collector of the same diameter: FV's 0D junction must pass a tone
        // as the same exhaust without a junction does — one FV pipe from the valve to the
        // collector's end, the same node, designed path and open end (stagnation-plenum
        // junctions low-passed it)
        let d = ExhaustDef {
            primary_diameter_m: 0.05,
            collector_diameter_m: 0.05,
            tail_diameter_m: 0.05,
            tail_outlets: 1,
            ..def()
        };
        let w = 2.0 * std::f64::consts::PI * 600.0;
        let tone = |radiate: &mut dyn FnMut(f64) -> f64| {
            let n = (0.3 / DT) as usize;
            let (mut re, mut im, mut count) = (0.0, 0.0, 0.0);
            for k in 0..n {
                let t = k as f64 * DT;
                let r = radiate(0.05 + 0.02 * (w * t).sin());
                if t > 0.15 {
                    re += r * (w * t).cos();
                    im += r * (w * t).sin();
                    count += 1.0;
                }
            }
            2.0 * (re * re + im * im).sqrt() / count
        };
        let mut net = ExhaustFv::new(&d, 1, DT);
        let through_net = tone(&mut |mdot| {
            net.add_valve_flow(0, mdot * DT, 1004.5 * 900.0);
            net.step()
        });
        // the same without the junction
        let a = area(0.05);
        let init = Prim {
            rho: ATM_PA / (R_AIR * INIT_T_K),
            u: 0.0,
            p: ATM_PA,
        };
        let mut pipe = super::pipe(d.primary_length_m + d.collector_length_m, 0.05, init);
        let mut duct = crate::exhaust::after_collector(
            &d,
            DT,
            crate::exhaust::LINING_SEGMENT_M,
            &mut crate::silencer2d::EqMemo::default(),
        );
        let mut end = OpenEnd::new(d.outlet_diameter(), DT);
        let gas = (
            crate::gas::GAMMA * ATM_PA / C_TAIL,
            crate::exhaust::tail_reference_density(),
        );
        let substeps = net.substeps;
        let (mut count, mut acc, mut sent) = (0, 0.0, [0.0; 2]);
        let through_straight = tone(&mut |mdot| {
            acc += mdot * DT;
            count += 1;
            if count >= substeps {
                count = 0;
                let h = substeps as f64 * DT;
                let cell = pipe.prim(0);
                pipe.left = Boundary::Flux(flux_boundary(
                    acc / (a * h),
                    acc * 1004.5 * 900.0 / (a * h),
                    cell,
                ));
                acc = 0.0;
                let arriving = duct.at_near();
                sent[0] = sent[1];
                let n = ((h / pipe.max_dt(0.9)).ceil() as usize).max(1);
                for _ in 0..n {
                    let last = pipe.prim(pipe.cells.len() - 1);
                    let (ghost, s) = duct_node(last, a, arriving, duct.end_areas().0, gas);
                    pipe.right = Boundary::State(ghost);
                    sent[1] = s;
                    pipe.step(h / n as f64);
                }
            }
            let f = count as f64 / substeps as f64;
            let (reflected, out) = end.process(duct.at_far());
            duct.send(sent[0] + (sent[1] - sent[0]) * f, reflected);
            out
        });
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
    fn gas_entering_a_pipe_from_a_junction_starts_from_the_junction_stagnation_state() {
        // Inflow from a volume (Benson's constant-pressure junction): the junction state is the
        // stagnation state, so the entering gas keeps the junction's entropy and total enthalpy
        // (c² /(γ−1) + u²/2 = c₀²/(γ−1)), the invariant leaving the pipe, and pays the dynamic
        // pressure: its static pressure is below the junction's.
        let g = crate::gas::GAMMA;
        let (p0, rho0) = (1.3e5, 1.3);
        let c0 = (g * p0 / rho0).sqrt();
        for (sigma, u) in [(1.0, -30.0), (-1.0, 30.0)] {
            let cell = Prim {
                rho: 1.0,
                u,
                p: 1.0e5,
            };
            let b = junction_ghost(cell, p0, rho0, sigma);
            assert!(sigma * b.u < sigma * u, "σ {sigma}: flows in faster: {b:?}");
            let c = b.sound_speed();
            let h0 = c * c / (g - 1.0) + 0.5 * b.u * b.u;
            assert!(
                (h0 / (c0 * c0 / (g - 1.0)) - 1.0).abs() < 1e-9,
                "σ {sigma}: {b:?}"
            );
            let entropy = |p: f64, r: f64| p / r.powf(g);
            assert!((entropy(b.p, b.rho) / entropy(p0, rho0) - 1.0).abs() < 1e-9);
            // pipe side: the outgoing characteristic on the cell's own isentrope
            let c_c = cell.sound_speed();
            let drop = 1.0 - (b.p / cell.p).powf((g - 1.0) / (2.0 * g));
            let along = u + sigma * 2.0 * c_c / (g - 1.0) * drop;
            assert!((b.u - along).abs() < 1e-6, "σ {sigma}: {b:?} vs {along}");
            assert!(b.p < p0 && b.p > cell.p, "σ {sigma}: {b:?}");
        }
    }

    #[test]
    fn inflow_from_a_junction_chokes_at_the_critical_pressure_ratio() {
        // A pipe end already drawing fast from a junction above it: sonic inflow at
        // p* = p₀·(2/(γ+1))^(γ/(γ−1)) = 0.5283 p₀ (isentropic nozzle, Anderson, Modern
        // Compressible Flow, eq. 5.21).
        let cell = Prim {
            rho: 0.6,
            u: -350.0,
            p: 5.0e4,
        };
        let b = junction_ghost(cell, 1.0e5, 1.2, 1.0);
        assert!((b.p / 1.0e5 - 0.5283).abs() < 1e-3, "{b:?}");
        assert!(
            (b.u + b.sound_speed()).abs() < 1e-9 * b.sound_speed(),
            "{b:?}"
        );
    }

    #[test]
    fn each_fv_tail_reports_its_own_signal_and_they_sum_to_the_radiated() {
        let d = ExhaustDef {
            collectors: vec![vec![1, 3], vec![2, 4]],
            ..def()
        };
        let mut net = ExhaustFv::new(&d, 4, DT);
        let mut seen = 0.0;
        for k in 0..40_000 {
            if k % 8000 < 800 {
                net.add_valve_flow(0, 0.05 * DT, 1004.5 * 900.0);
            }
            let r = net.step();
            let t = net.last_tails();
            assert_eq!(t.len(), 2);
            assert!(
                (t[0] + t[1] - r).abs() <= 1e-9 * r.abs().max(1.0),
                "{t:?} vs {r}"
            );
            seen += t[0].abs();
        }
        // the blowing bank radiates more than the resting one
        assert!(seen > 0.0);
    }

    #[test]
    fn the_radiated_signal_moves_between_pipe_steps() {
        // A pipe step spans several physics steps; held between them, the radiated source is a
        // staircase whose edges the listener's radiation filter (∝ s) turns into spikes at the
        // pipe rate. Interpolated, it moves every physics step, landing on each pipe step's
        // value one pipe step late.
        let dt = 1.0 / 192_000.0;
        let mut net = ExhaustFv::new(&def(), 1, dt);
        assert!(net.substeps >= 4, "{}", net.substeps);
        let mut x = Vec::new();
        for k in 0..20_000 {
            if k % 4000 < 400 {
                net.add_valve_flow(0, 0.05 * dt, 1004.5 * 900.0);
            }
            x.push(net.step());
        }
        let busy = &x[1000..4000];
        let held = busy.windows(2).filter(|w| w[0] == w[1]).count();
        assert!(
            held < busy.len() / 20,
            "{held} of {} steps held",
            busy.len()
        );
        // the tails follow the same line and sum to it
        assert!((net.last_tails().iter().sum::<f64>() - x[x.len() - 1]).abs() < 1e-9);
    }

    #[test]
    fn rising_root_finds_the_root_in_few_evaluations() {
        // x³ − 2 on [0, 2]: the cube root of 2 (1.259921049894873); bisection would need 40+.
        let points = std::cell::RefCell::new(Vec::new());
        let r = rising_root(
            |x| {
                points.borrow_mut().push(x);
                x * x * x - 2.0
            },
            0.0,
            2.0,
        );
        let points = points.into_inner();
        assert!((r - 1.259_921_049_894_873).abs() < 1e-12, "{r}");
        // the ends first, then the secant through them: (0·6 − 2·(−2)) / (6 + 2) = 0.5
        assert_eq!(&points[..3], &[0.0, 2.0, 0.5]);
        // every step stays inside the bracket
        assert!(points.iter().all(|x| (0.0..=2.0).contains(x)), "{points:?}");
        // Illinois converges superlinearly: 13 evaluations (bisection: 2 + 44)
        assert_eq!(points.len(), 13, "{points:?}");
        // A step, where Illinois alone creeps (it ran out of tries 0.06 from the root): the
        // bisection safeguard closes the bracket to 1e-13 anyway, in 2 + 51 evaluations (plain
        // bisection: 2 + 44).
        let calls = std::cell::Cell::new(0);
        let r = rising_root(
            |x| {
                calls.set(calls.get() + 1);
                if x < 0.3 { -1.0 } else { 1.0 }
            },
            0.0,
            1.0,
        );
        assert_eq!(calls.get(), 53);
        assert!((r - 0.3).abs() < 1e-12, "{r}");
        // a lopsided step (−1 against 1e-9): Illinois creeps by halving f(lo) for thirty steps
        // at a time, and only the one-bisection-in-three guarantee closes the bracket, 0.3 to
        // 1e-13 being 42 halvings, so within 2 + 3·42 evaluations
        calls.set(0);
        let r = rising_root(
            |x| {
                calls.set(calls.get() + 1);
                if x < 0.3 { -1.0 } else { 1e-9 }
            },
            0.0,
            1.0,
        );
        assert!((r - 0.3).abs() < 1e-12, "{r}");
        assert!(calls.get() <= 128, "{} evaluations", calls.get());
        // the same step at 0.9 on [0.5, 1]: the first check measures against the initial
        // bracket's 0.5 (from it, Illinois' first three steps fail to halve it): 112 evaluations
        calls.set(0);
        let r = rising_root(
            |x| {
                calls.set(calls.get() + 1);
                if x < 0.9 { -1.0 } else { 1e-9 }
            },
            0.5,
            1.0,
        );
        assert!((r - 0.9).abs() < 1e-12, "{r}");
        assert_eq!(calls.get(), 112);
        // a steep, one-sided function (√ near its end, like w_j at x = 1) still converges
        let r = rising_root(|x| x - (1.0 - x * x).max(0.0).sqrt(), 0.0, 1.0);
        assert!((r - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12, "{r}");
        // no sign change: the bracket's end where the root would be
        assert_eq!(rising_root(|x| x + 1.0, 0.0, 1.0), 0.0);
        assert_eq!(rising_root(|x| x - 5.0, 0.0, 1.0), 1.0);
    }

    #[test]
    fn rising_root_stops_when_the_bracket_is_relatively_tight() {
        // Scaled up 10⁶ (x³ − 2·10¹⁸ on [0, 2·10⁶]) the root is never hit exactly: the search
        // ends when the bracket is within 1e-13 of hi, long before the 40-step cap.
        let calls = std::cell::Cell::new(0);
        let r = rising_root(
            |x| {
                calls.set(calls.get() + 1);
                x * x * x - 2e18
            },
            0.0,
            2e6,
        );
        assert!((r / 1_259_921.049_894_873 - 1.0).abs() < 1e-12, "{r}");
        assert!(calls.get() <= 20, "{} evaluations", calls.get());
    }

    #[test]
    fn a_throttled_end_passes_what_its_orifice_passes() {
        // ψ = 0.3 of the pipe area: the face mass flux equals a compressible orifice of that area
        // (per pipe area), fed from the junction at rest inward and from the face's own
        // stagnation state outward; ψ = 1 is the open end, ψ = 0 a wall.
        let g = crate::gas::GAMMA;
        let t = |s: Prim| s.p / (s.rho * R_AIR);
        let (p_j, rho_j) = (1.0e5, 1.16);
        for (sigma, u, p) in [
            (1.0, -60.0, 0.9e5),
            (-1.0, 60.0, 0.9e5),
            (1.0, -100.0, 0.3e5),
        ] {
            let cell = Prim { rho: 1.0, u, p };
            let b = throttled_ghost(cell, p_j, rho_j, sigma, 0.3);
            assert!(-sigma * b.u > 0.0, "inflow: {b:?}");
            let want = orifice_mass_flow(
                0.3,
                p_j,
                t(Prim {
                    rho: rho_j,
                    u: 0.0,
                    p: p_j,
                }),
                b.p,
            );
            let got = b.rho * b.u.abs();
            assert!(
                (got / want - 1.0).abs() < 1e-6,
                "σ {sigma}: {got} vs {want}"
            );
            let open = throttled_ghost(cell, p_j, rho_j, sigma, 1.0);
            assert!(
                b.u.abs() < open.u.abs() && b.p < open.p,
                "{b:?} vs {open:?}"
            );
            assert_eq!(open, junction_ghost(cell, p_j, rho_j, sigma));
        }
        for (sigma, u, p) in [(1.0, 60.0, 1.1e5), (-1.0, -60.0, 1.1e5)] {
            let cell = Prim { rho: 1.3, u, p };
            let b = throttled_ghost(cell, p_j, rho_j, sigma, 0.3);
            assert!(sigma * b.u > 0.0, "outflow: {b:?}");
            let c2 = g * b.p / b.rho;
            let t0 = t(b) * (1.0 + 0.5 * (g - 1.0) * b.u * b.u / c2);
            let p0 = b.p * (t0 / t(b)).powf(g / (g - 1.0));
            let want = orifice_mass_flow(0.3, p0, t0, p_j);
            let got = b.rho * b.u.abs();
            assert!(
                (got / want - 1.0).abs() < 1e-6,
                "σ {sigma}: {got} vs {want}"
            );
            let open = throttled_ghost(cell, p_j, rho_j, sigma, 1.0);
            assert!(
                b.u.abs() < open.u.abs() && b.p > open.p,
                "{b:?} vs {open:?}"
            );
            assert_eq!(open, junction_ghost(cell, p_j, rho_j, sigma));
        }
        for (u, p) in [(-60.0, 0.9e5), (60.0, 1.1e5)] {
            let wall = throttled_ghost(Prim { rho: 1.2, u, p }, p_j, rho_j, 1.0, 0.0);
            assert!(wall.u.abs() < 1e-6, "{wall:?}");
        }
    }

    #[test]
    fn junction_ghost_clamps_the_pressure_ratio_and_chokes() {
        let cell = Prim {
            rho: 1.0,
            u: 0.0,
            p: 1.0e5,
        };
        // inflow from at most 20× the cell pressure: a 100 bar junction acts as the same gas
        // (same entropy) at 20 bar, which compresses the resting pipe end without choking
        let hi = junction_ghost(cell, 1.0e7, 0.7, 1.0);
        let rho20 = 0.7 * 0.2_f64.powf(1.0 / crate::gas::GAMMA);
        let at20 = junction_ghost(cell, 20.0e5, rho20, 1.0);
        assert!((hi.p - at20.p).abs() < 1e-6 * at20.p && (hi.u - at20.u).abs() < 1e-9);
        assert!(hi.p > 0.5283 * 20.0e5 && hi.p < 20.0e5, "{hi:?}");
        assert!(hi.u < 0.0 && -hi.u < hi.sound_speed(), "{hi:?}");
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
        // 2 × π/4·0.04²·0.7 primaries + junction + collector at 1 atm / 600 K (ρ = 0.5884
        // kg/m³), defaults of `ExhaustDef`, diameters by hand: the FV pipes end at the collector,
        // the designed path after it is the linear duct's (sound, no gas)
        let d = def();
        let disc = |dia: f64| std::f64::consts::PI / 4.0 * dia * dia;
        let v = 2.0 * disc(d.primary_diameter_m) * d.primary_length_m
            + disc(d.collector_diameter_m) * 3.0 * d.collector_diameter_m
            + disc(d.collector_diameter_m) * d.collector_length_m;
        let m = ExhaustFv::new(&d, 2, DT).total_mass();
        assert!(
            (m / (0.5884 * v) - 1.0).abs() < 0.03,
            "{m} kg vs {}",
            0.5884 * v
        );
    }

    #[test]
    fn the_pipes_advance_once_per_fv_step() {
        // Physics time and pipe time stay in step: dt_fv = substeps · dt, and the line the
        // radiated value follows gets a new end once per FV step under a steady flow (the value
        // at each pipe step's boundary changes every pipe step).
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
        for k in 1..steps {
            net.add_valve_flow(0, 0.05 * DT, 1004.5 * 900.0);
            let r = net.step();
            if k % net.substeps == 0 {
                if r != last {
                    updates += 1;
                }
                last = r;
            }
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

    #[test]
    fn a_coarse_step_subcycles_without_changing_the_source() {
        // At 12.5 kHz one FV step needs several CFL sub-cycles (and the designed path's duct is
        // still a pipe); the steady source (and each tail's share of it) matches the fine-step
        // network's.
        let mean = |dt: f64| {
            let d = ExhaustDef {
                collectors: vec![vec![1], vec![2]],
                ..def()
            };
            let mut net = ExhaustFv::new(&d, 2, dt);
            let (mut sum, mut tails) = (0.0, 0.0);
            let n = (0.1 / dt) as usize;
            for k in 0..n {
                net.add_valve_flow(0, 0.05 * dt, 1004.5 * 900.0);
                net.add_valve_flow(1, 0.05 * dt, 1004.5 * 900.0);
                let r = net.step();
                if k > n / 2 {
                    sum += r;
                    tails += net.last_tails().iter().sum::<f64>();
                }
            }
            let m = (n - n / 2 - 1) as f64;
            (sum / m, tails / m)
        };
        let (fine, _) = mean(DT);
        let (coarse, coarse_tails) = mean(8e-5);
        assert!((coarse / fine - 1.0).abs() < 0.05, "{coarse} vs {fine}");
        assert!(
            (coarse_tails / coarse - 1.0).abs() < 1e-9,
            "{coarse_tails} vs {coarse}"
        );
    }

    #[test]
    fn the_tail_source_is_referenced_to_gas_at_its_sound_speed() {
        // c = 500 m/s ⇒ T = c²/(γR) = 250 000/(1.4·287) = 622.2 K; at 101 325 Pa,
        // ρ = p/(R·T) = 101 325/(287·622.2) = 0.56742 kg/m³
        approx::assert_relative_eq!(
            crate::exhaust::tail_reference_density(),
            0.56742,
            max_relative = 1e-4
        );
    }

    #[test]
    fn the_valve_sees_the_characteristic_arriving_from_its_first_cell() {
        // ρ = 0.5 kg/m³, u = 30 m/s away from the valve, p = 1.5 bar: c = √(1.4·1.5e5/0.5) =
        // 648.074 m/s. Against a 2-bar cylinder λ_in = (0.75)^(1/7)·(1 − 0.2·30/648.074) =
        // 0.950 850; T = p/(ρR) = 1 045.30 K
        let mut net = ExhaustFv::new(&def(), 1, DT);
        net.primaries[0].cells[0] = Prim {
            rho: 0.5,
            u: 30.0,
            p: 1.5e5,
        }
        .to_cons();
        let (lambda, t) = net.port_lambda(0, 2.0e5);
        approx::assert_relative_eq!(lambda, 0.950_850_2, max_relative = 1e-6);
        approx::assert_relative_eq!(t, 1_045.30, max_relative = 1e-5);
    }

    #[test]
    fn the_first_face_carries_the_valve_boundarys_momentum() {
        // two physics steps of a boundary at p + ρu² = 1.6e5 and 1.8e5 Pa over equal times:
        // the next FV step's first face carries their mean, 1.7e5 Pa (not the first cell's
        // pressure); with no boundary registered it falls back to the cell's
        let mut net = ExhaustFv::new(&def(), 1, DT);
        net.add_valve_flow(0, 1e-6, 1004.5 * 900.0);
        net.add_valve_boundary(0, 1.6e5, DT);
        net.add_valve_flow(0, 1e-6, 1004.5 * 900.0);
        net.add_valve_boundary(0, 1.8e5, DT);
        net.pipe_step();
        match net.primaries[0].left {
            Boundary::Flux(f) => approx::assert_relative_eq!(f.mom, 1.7e5, max_relative = 1e-12),
            other => panic!("{other:?}"),
        }
        // (the fallback rebuilds it from the first cell as it was before the step)
        let cell = net.primaries[0].prim(0);
        net.pipe_step();
        match net.primaries[0].left {
            Boundary::Flux(f) => assert!((f.mom - cell.p).abs() < 1e-6 * cell.p.max(1.0) + 1.0),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn the_port_holds_what_the_valve_already_pushed() {
        // Between FV steps the valve sees the port cell as a 0D volume: with this step's flow
        // already in it (mass, and its enthalpy as internal energy), or it overshoots and chatters.
        let mut net = ExhaustFv::new(&def(), 1, DT);
        let before = net.port_volume(0);
        net.add_valve_flow(0, 1e-6, 1004.5 * 900.0);
        let after = net.port_volume(0);
        approx::assert_relative_eq!(after.mass_kg - before.mass_kg, 1e-6, max_relative = 1e-9);
        approx::assert_relative_eq!(
            after.energy_j - before.energy_j,
            1e-6 * 1004.5 * 900.0,
            max_relative = 1e-9
        );
    }

    #[test]
    fn the_probe_reads_each_fv_pipes_gas_by_hand() {
        // a fresh network rests at 600 K: two primaries and one collector, still
        let d = def();
        let mut x = ExhaustFv::new(&d, 2, DT);
        let mut out = Vec::new();
        x.probe(&mut out);
        assert_eq!(out.len(), 3);
        for p in &out {
            assert!(
                (p.temperature_k - 600.0).abs() < 1e-6,
                "{}",
                p.temperature_k
            );
            assert_eq!((p.velocity_m_s, p.wall_k), (0.0, None));
        }
        // primary 2 holds 0.5 kg/m³ at 1 bar moving out at 20 m/s in its first half and still
        // gas in the rest: T = 1e5/(0.5·287) = 696.86 K, the mass moves at 10 m/s on average
        let n = x.primaries[1].cells.len();
        for (k, c) in x.primaries[1].cells.iter_mut().enumerate() {
            let u = if 2 * k < n { 20.0 } else { 0.0 };
            *c = Prim {
                rho: 0.5,
                u,
                p: 1e5,
            }
            .to_cons();
        }
        x.probe(&mut out);
        assert!(
            (out[1].temperature_k - 696.864).abs() < 1e-3,
            "{}",
            out[1].temperature_k
        );
        let half = (n / 2 + n % 2) as f64 / n as f64;
        assert!(
            (out[1].velocity_m_s - 20.0 * half).abs() < 1e-9,
            "{}",
            out[1].velocity_m_s
        );
        assert!(
            (out[0].temperature_k - 600.0).abs() < 1e-6,
            "the others untouched"
        );
    }
}
