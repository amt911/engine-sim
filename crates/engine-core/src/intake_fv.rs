//! Intake as non-linear 1D finite-volume runners ([`crate::fv`]): one runner per cylinder from a
//! trumpet to the intake valve, fed either through **individual throttle bodies** at each trumpet
//! (ITBs, like the LFA) or from a common **plenum** behind a single throttle. Gives intake (ram)
//! tuning and induction sound. Spec: `docs/superpowers/specs/2026-10-22-intake-and-pipe-editor.md`.

use crate::exhaust_fv::{CELL_M, PIPE_FRICTION, area, guard, junction_ghost, throttled_ghost};
use crate::fv::{Boundary, Cons, Pipe1d, Prim, flux_boundary};
use crate::gas::{GasVolume, R_AIR};
use crate::units::ATM_PA;
use serde::{Deserialize, Serialize};

pub const AMBIENT_T_K: f64 = 300.0;
/// Speed of sound of ambient air, for the induction-sound source c·(ρu).
pub const C_AIR: f64 = 343.0;
/// Most substeps of an FV step (the CFL limit is never reached at the worst flow).
pub const MAX_SUBCYCLES: usize = 16;

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
    /// Share X of each injection that lands on the port walls as liquid film (Aquino's x–τ
    /// model, SAE 810494) at 100 kPa and 30 rev/s; it follows load and speed as
    /// [`crate::fuel::deposit_fraction`]. Almkvist & Eriksson (SAE 941931), warm port injection:
    /// ≈ 0.18 at full load (Fig. 9).
    pub film_fraction: f64,
    /// Time constant τ of the film's evaporation, s. Almkvist & Eriksson (SAE 941931) Figs.
    /// 11–13, warm port injection: ≈ 1 s from 20 to 40 rev/s, a little less at full load
    /// (Heywood §7.6.3: of order 1 s for a manifold film). 0 disables the film.
    pub film_tau_s: f64,
    /// The surge tank (air box) the trumpets draw from, breathing through its inlet ducts;
    /// none: the trumpets (or the throttle) open straight to the air.
    pub airbox: Option<AirboxDef>,
}

/// A surge tank: a 0D volume fed by `ducts` equal inlet ducts from the atmosphere. With
/// individual throttles the trumpets draw from it; with a common throttle, the throttle does.
/// The induction sound then radiates from the duct mouths, filtered by the box: a Helmholtz
/// resonator of f = (c/2π)·√(n·A/(L'·V)) that passes the pulsation below f and holds it back
/// above (Kinsler, Frey, Coppens & Sanders, Fundamentals of Acoustics 4th ed., §10.8–10.9;
/// Munjal, Acoustics of Ducts and Mufflers, ch. 2). Each duct is a 1D pipe of the effective
/// length L' = L + 0.85a + 0.6a: the end corrections of a flanged end at the box (Rayleigh) and
/// an unflanged mouth (Levine & Schwinger), Kinsler §10.8.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct AirboxDef {
    pub volume_m3: f64,
    pub ducts: usize,
    pub duct_length_m: f64,
    pub duct_diameter_m: f64,
}

impl Default for AirboxDef {
    fn default() -> Self {
        Self {
            volume_m3: 0.010,
            ducts: 1,
            duct_length_m: 0.3,
            duct_diameter_m: 0.07,
        }
    }
}

impl AirboxDef {
    /// Length of each duct's 1D pipe: the geometric length plus both end corrections.
    pub fn effective_duct_length(&self) -> f64 {
        self.duct_length_m + (0.85 + 0.6) * 0.5 * self.duct_diameter_m
    }

    /// The Lexus LFA's surge tank: the ten trumpets draw from one air-cleaner case on top of
    /// the V, fed by two inlet ducts (Toyota EPC 176966A: case 17190, ducts 17861F/17862D).
    /// Toyota has not published its size; every number is an ESTIMATE from that drawing's
    /// proportions against the engine: the case spans the banks (≈ 0.5 × 0.35 m) by ≈ 0.12 m
    /// deep → ≈ 20 L (4 × the 4.8 L swept); each duct's mouth is as wide as the drawing's
    /// bank spacing over five cylinders → Ø 90 mm, ≈ 0.40 m along its curve. Helmholtz with
    /// both end corrections (L' = 0.465 m): (c/2π)√(2A/(L'V)) = 65 Hz lumped, 62 Hz as a
    /// line. Yamaha tuned the case's walls to radiate the firing tone (250–500 Hz) — a path
    /// this model does not have yet, which is why the preset does not use it (2026-10-29).
    pub fn lfa_v10_estimate() -> Self {
        Self {
            volume_m3: 0.020,
            ducts: 2,
            duct_length_m: 0.40,
            duct_diameter_m: 0.09,
        }
    }
}

/// The state a runner's trumpet (or the throttle) draws from: the atmosphere, or the air box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Upstream {
    pub p: f64,
    pub rho: f64,
}

impl Upstream {
    pub const ATMOSPHERE: Upstream = Upstream {
        p: ATM_PA,
        rho: ATM_PA / (R_AIR * AMBIENT_T_K),
    };
}

impl Default for IntakeDef {
    fn default() -> Self {
        Self {
            model: IntakeModel::ZeroD,
            runner_length_m: 0.3,
            runner_diameter_m: 0.042,
            individual_throttles: false,
            trumpet_loss: 0.1,
            film_fraction: 0.18,
            film_tau_s: 1.0,
            airbox: None,
        }
    }
}

/// Physics steps per FV step when a port passes more than its runner's section
/// ([`IntakeFv::coupled_to_port`]): measured on a bridge-ported 13B at 384 kHz, every 8 steps
/// still whistled at the FV rate's half, every 4 did not (FINDINGS 2026-10-31).
pub const STIFF_PORT_SUBSTEPS: usize = 4;

#[derive(Debug, Clone)]
pub struct IntakeFv {
    runners: Vec<Pipe1d>,
    a_runner: f64,
    individual: bool,
    /// Common plenum (unused with individual throttles).
    plenum: GasVolume,
    /// The surge tank and its ducts, if the engine has one.
    airbox: Option<Airbox>,
    /// What the intake's mouths draw from: the atmosphere, or a compressor's delivery
    /// ([`IntakeFv::set_inlet`]).
    inlet: Upstream,
    /// Discharge-coefficient factor of the trumpet entry, 1/√(1 + k).
    trumpet_cd: f64,
    pub substeps: usize,
    pub dt_fv: f64,
    pub dt: f64,
    count: usize,
    acc_mass: Vec<f64>,
    acc_energy: Vec<f64>,
    /// The part of each step's valve flow pushed back into the runner, kg and J (≤ 0): the
    /// gas sitting in the port that the valve draws first ([`IntakeFv::port_volume`]).
    back_mass: Vec<f64>,
    back_energy: Vec<f64>,
    /// What each valve took over the last FV step beyond what its port cell could give (the
    /// runner's flux limit), kg: never in the runner, to be handed back by the cylinder
    /// ([`IntakeFv::take_unserved`]).
    unserved: Vec<f64>,
    radiated: f64,
    inducted: f64,
    inducted_rate: f64,
    /// The FV step's rate before the last one (the interpolation's start).
    previous_rate: f64,
    /// Runners stepped elsewhere ([`IntakeFv::detach_runners`]); the ones from `first` on.
    remote: RemoteRunners,
    /// This step's left-end mass flux of every runner, per substep, and the local runners'
    /// valve flows (scratch, kept: no allocation per FV step).
    flux: Vec<f64>,
    /// The energy fluxes that go with `flux` (what the box loses with the mass).
    eflux: Vec<f64>,
    acc: Vec<(f64, f64)>,
}

/// A surge tank as a 0D volume with its inlet ducts as 1D pipes ([`AirboxDef`]).
#[derive(Debug, Clone)]
struct Airbox {
    gas: GasVolume,
    ducts: Vec<Pipe1d>,
    a_duct: f64,
}

impl Airbox {
    fn new(def: &AirboxDef, init: Prim) -> Self {
        let a_duct = area(def.duct_diameter_m.max(1e-3));
        let length = def.effective_duct_length().max(2.0 * CELL_M);
        let n = (length / CELL_M).round().max(2.0);
        let ducts = (0..def.ducts.max(1))
            .map(|_| {
                let mut p = Pipe1d::segmented(&[(length, a_duct)], length / n, init);
                p.floors = (1e-3, 500.0);
                p.friction = PIPE_FRICTION;
                p
            })
            .collect();
        Self {
            gas: GasVolume::at(ATM_PA, AMBIENT_T_K, def.volume_m3.max(1e-4)),
            ducts,
            a_duct,
        }
    }

    fn upstream(&self) -> Upstream {
        Upstream {
            p: self.gas.pressure(),
            rho: self.gas.mass_kg / self.gas.volume_m3,
        }
    }

    /// One substep of the ducts: each breathes between the atmosphere at its mouth and the box
    /// at its other end (both Benson junction ends), and the box takes what they bring.
    /// Returns the air that came in at the mouths, kg (negative: blown out), and the mouths'
    /// sound source c·Σρu.
    fn breathe(&mut self, h: f64, inlet: Upstream) -> (f64, f64) {
        let up = self.upstream();
        let (mut inducted, mut at_mouths) = (0.0, 0.0);
        for duct in &mut self.ducts {
            let n = duct.cells.len();
            duct.left = Boundary::State(junction_ghost(duct.prim(0), inlet.p, inlet.rho, -1.0));
            duct.right = Boundary::State(junction_ghost(duct.prim(n - 1), up.p, up.rho, 1.0));
            duct.step(h);
            let (mouth, into) = duct.end_flux;
            inducted += mouth.rho * self.a_duct * h;
            at_mouths += C_AIR * mouth.rho;
            self.gas.mass_kg += into.rho * self.a_duct * h;
            self.gas.energy_j += into.e * self.a_duct * h;
        }
        guard(&mut self.gas);
        (inducted, at_mouths)
    }

    fn mass(&self) -> f64 {
        self.gas.mass_kg
            + self
                .ducts
                .iter()
                .map(|p| p.cells.iter().map(|c| c.rho).sum::<f64>() * self.a_duct * p.dx)
                .sum::<f64>()
    }
}

/// A group of ITB runners and what stepping them needs: the part of an [`IntakeFv`] that can run
/// on another thread.
#[derive(Debug, Clone)]
pub struct RunnerSet {
    pub pipes: Vec<Pipe1d>,
    a_runner: f64,
    trumpet_cd: f64,
}

/// What a [`RunnerSet`] hands back after an FV step: each runner's left-end mass flux per
/// substep (k per runner, runner after runner), its last cell (at the valve) and its middle
/// cell's pressure, and the group's largest stable step for the next one.
#[derive(Debug, Clone, Default)]
pub struct RunnerResult {
    pub flux: Vec<f64>,
    /// The energy flux at the same faces, per substep, like `flux`.
    pub eflux: Vec<f64>,
    pub last: Vec<Cons>,
    pub mid_p: Vec<f64>,
    pub min_dt: f64,
}

impl RunnerSet {
    /// One FV step of `dt`, k substeps, the throttle's area `throttle_area`, the state the
    /// trumpets draw from `up`, each runner's valve flow over the step `acc` (mass, energy).
    pub fn fv_step(
        &mut self,
        k: usize,
        dt: f64,
        throttle_area: f64,
        up: Upstream,
        acc: &[(f64, f64)],
        out: &mut RunnerResult,
    ) {
        out.flux.resize(self.pipes.len() * k, 0.0);
        out.eflux.resize(self.pipes.len() * k, 0.0);
        itb_runners_step(
            &mut self.pipes,
            acc,
            self.a_runner,
            self.trumpet_cd,
            throttle_area,
            up,
            dt,
            k,
            &mut out.flux,
            &mut out.eflux,
        );
        out.last.clear();
        out.last
            .extend(self.pipes.iter().map(|p| p.cells[p.cells.len() - 1]));
        out.mid_p.clear();
        out.mid_p
            .extend(self.pipes.iter().map(|p| p.prim(p.cells.len() / 2).p));
        out.min_dt = stable_dt(&self.pipes);
    }
}

/// Where an [`IntakeFv`] sends the FV steps of its detached runners (another thread, in the
/// app): `submit` an FV step (k substeps over `dt`, the throttle area, what the trumpets draw
/// from, each runner's valve flow), then `collect` its [`RunnerResult`].
pub trait RunnerLink: Send {
    fn submit(&mut self, k: usize, dt: f64, throttle_area: f64, up: Upstream, acc: &[(f64, f64)]);
    fn collect(&mut self, out: &mut RunnerResult);
    /// No more steps: whatever runs the runners may hand them back.
    fn stop(&mut self) {}
}

/// [`RunnerLink`] that steps the set right here (the reference for the threaded one).
pub struct LocalRunners {
    pub set: RunnerSet,
    pending: Option<Submitted>,
}

/// An FV step submitted to [`LocalRunners`], waiting to be collected.
struct Submitted {
    k: usize,
    dt: f64,
    throttle_area: f64,
    up: Upstream,
    acc: Vec<(f64, f64)>,
}

impl LocalRunners {
    pub fn new(set: RunnerSet) -> Self {
        Self { set, pending: None }
    }
}

impl RunnerLink for LocalRunners {
    fn submit(&mut self, k: usize, dt: f64, throttle_area: f64, up: Upstream, acc: &[(f64, f64)]) {
        self.pending = Some(Submitted {
            k,
            dt,
            throttle_area,
            up,
            acc: acc.to_vec(),
        });
    }
    fn collect(&mut self, out: &mut RunnerResult) {
        if let Some(j) = self.pending.take() {
            self.set
                .fv_step(j.k, j.dt, j.throttle_area, j.up, &j.acc, out);
        }
    }
}

/// The detached runners' side of an [`IntakeFv`]: the link, where they start, their last and
/// middle cells as of the last FV step, and their largest stable step.
struct Remote {
    link: Box<dyn RunnerLink>,
    first: usize,
    dx: f64,
    result: RunnerResult,
    acc: Vec<(f64, f64)>,
}

/// Holds a [`Remote`]; a clone does not carry it (see `engine::RemoteSlot`).
struct RemoteRunners(Option<Box<Remote>>);

impl Clone for RemoteRunners {
    fn clone(&self) -> Self {
        RemoteRunners(None)
    }
}

impl std::fmt::Debug for RemoteRunners {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() {
            "remote runners"
        } else {
            "-"
        })
    }
}

/// The valve's flow over an FV step as the runner's right-end flux: what the cylinder took
/// (`m` kg, `e` J; never more than half the last cell holds) spread over `dt`.
fn set_valve_boundary(pipe: &mut Pipe1d, m: f64, e: f64, a: f64, dt: f64) {
    let n = pipe.cells.len();
    let cell = pipe.prim(n - 1);
    // what leaves the runner into the cylinder is a +x flux at its right face
    let available = 0.5 * pipe.cells[n - 1].rho * a * pipe.dx;
    let scale = if m > available { available / m } else { 1.0 };

    let mf = scale * m / (a * dt);
    pipe.right = Boundary::Flux(flux_boundary(mf, scale * e / (a * dt), cell));
}

/// The largest stable FV step of `pipes` (CFL 0.9 on the fastest), s.
fn stable_dt(pipes: &[Pipe1d]) -> f64 {
    pipes
        .iter()
        .map(|p| p.max_dt(0.9))
        .fold(f64::INFINITY, f64::min)
}

/// ITB runners through one FV step: each runner's valve flow `acc` becomes its right-end flux,
/// then k substeps of dt/k with the throttled trumpet (stable characteristic end on `up`: the
/// atmosphere or the air box, held over the step) at its left; each runner's left-end mass and
/// energy flux per substep go to `flux` and `eflux`. Each runner is independent of the others,
/// so they step one after another.
#[allow(clippy::too_many_arguments)]
fn itb_runners_step(
    pipes: &mut [Pipe1d],
    acc: &[(f64, f64)],
    a: f64,
    trumpet_cd: f64,
    throttle_area: f64,
    up: Upstream,
    dt: f64,
    k: usize,
    flux: &mut [f64],
    eflux: &mut [f64],
) {
    let a_eff = throttle_area.max(0.0) * trumpet_cd;
    let a_itb = a_eff.min(a * trumpet_cd);
    let psi = a_itb / a;
    let h = dt / k as f64;
    for (((pipe, &(m, e)), f), ef) in pipes
        .iter_mut()
        .zip(acc)
        .zip(flux.chunks_mut(k))
        .zip(eflux.chunks_mut(k))
    {
        set_valve_boundary(pipe, m, e, a, dt);
        for (fs, es) in f.iter_mut().zip(ef) {
            let ghost = throttled_ghost(pipe.prim(0), up.p, up.rho, -1.0, psi);
            pipe.left = Boundary::State(ghost);
            pipe.step(h);
            *fs = pipe.end_flux.0.rho;
            *es = pipe.end_flux.0.e;
        }
    }
}

impl IntakeFv {
    /// The mouths draw from `p_pa`, `t_k` (a compressor's delivery after its intercooler)
    /// instead of the atmosphere.
    pub fn set_inlet(&mut self, p_pa: f64, t_k: f64) {
        self.inlet = Upstream {
            p: p_pa,
            rho: p_pa / (R_AIR * t_k),
        };
    }

    /// Step every runner with `scheme`.
    pub fn with_scheme(mut self, scheme: crate::fv::Scheme) -> Self {
        for p in &mut self.runners {
            p.scheme = scheme;
        }
        if let Some(b) = &mut self.airbox {
            for p in &mut b.ducts {
                p.scheme = scheme;
            }
        }
        self
    }

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
            airbox: def.airbox.as_ref().map(|b| Airbox::new(b, init)),
            inlet: Upstream::ATMOSPHERE,
            trumpet_cd: 1.0 / (1.0 + def.trumpet_loss.max(0.0)).sqrt(),
            substeps,
            dt_fv: substeps as f64 * dt,
            dt,
            count: 0,
            acc_mass: vec![0.0; cylinders],
            acc_energy: vec![0.0; cylinders],
            back_mass: vec![0.0; cylinders],
            back_energy: vec![0.0; cylinders],
            unserved: vec![0.0; cylinders],
            radiated: 0.0,
            inducted: 0.0,
            inducted_rate: 0.0,
            previous_rate: 0.0,
            remote: RemoteRunners(None),
            flux: Vec::new(),
            eflux: Vec::new(),
            acc: Vec::new(),
        }
    }

    /// The air box's pressure, Pa; none without an air box.
    pub fn airbox_pressure(&self) -> Option<f64> {
        self.airbox.as_ref().map(|b| b.gas.pressure())
    }

    /// The runner's last cell (at the valve) as a 0D volume, for the valve flow, with what the
    /// valve has moved through it since the FV step began (`add_valve_flow`): gas pushed back
    /// is drawn again before the cell's own. Without it, a backflow and the draw after it within
    /// one FV step took the cold cell's gas and left the backflow's heat with no mass behind —
    /// a runner heated without bound (FINDINGS 2026-10-31).
    pub fn port_volume(&self, i: usize) -> GasVolume {
        let (c, dx) = match &self.remote.0 {
            Some(r) if i >= r.first => (r.result.last[i - r.first], r.dx),
            _ => {
                let pipe = &self.runners[i];
                (pipe.cells[pipe.cells.len() - 1], pipe.dx)
            }
        };
        let v = self.a_runner * dx;
        let prim = c.to_prim();
        let (mass, energy) = (c.rho * v, prim.p / (crate::gas::GAMMA - 1.0) * v);
        // gas pushed back since the step began sits in the port: what the valve draws is at
        // the mixture's temperature, at the cell's pressure (its neighbours hold the pressure
        // until the FV step; a draw since then leaves the cell as it was for the same reason)
        let (back_m, back_e) = (self.back_mass[i], self.back_energy[i]);
        let (mass_kg, energy_j) = if back_m < 0.0 && back_e < 0.0 {
            let t_mix = (energy - back_e) / ((mass - back_m) * crate::gas::CV);
            (energy / (crate::gas::CV * t_mix), energy)
        } else {
            (mass, energy)
        };
        GasVolume {
            mass_kg,
            energy_j,
            volume_m3: v,
        }
    }

    /// This intake on valves (or ports) of the given largest effective flow area, m². The valve
    /// trades with the port cell as an FV step began and the runner then takes the lump: a port
    /// that passes more than the runner's own section empties that cell faster than the duct
    /// refills it, and the lump overshoots — the flow flipped every FV step (a whistle at half
    /// the FV rate). Such a port couples on a step of [`STIFF_PORT_SUBSTEPS`]; a valve's port
    /// (every poppet preset: 0.77–0.96 of its runner) keeps the CFL step and its cost.
    pub fn coupled_to_port(mut self, port_flow_area_m2: f64) -> Self {
        if port_flow_area_m2 > self.a_runner && self.substeps > STIFF_PORT_SUBSTEPS {
            self.substeps = STIFF_PORT_SUBSTEPS;
            self.dt_fv = STIFF_PORT_SUBSTEPS as f64 * self.dt;
        }
        self
    }

    /// The mass in runner `i`'s last cell, kg.
    fn port_cell_mass(&self, i: usize) -> f64 {
        let (c, dx) = match &self.remote.0 {
            Some(r) if i >= r.first => (r.result.last[i - r.first], r.dx),
            _ => {
                let pipe = &self.runners[i];
                (pipe.cells[pipe.cells.len() - 1], pipe.dx)
            }
        };
        c.rho * self.a_runner * dx
    }

    /// What valve `i` took beyond its runner's reach since this was last called, kg (see
    /// `unserved`); the cylinder gives it back.
    pub fn take_unserved(&mut self, i: usize) -> f64 {
        std::mem::take(&mut self.unserved[i])
    }

    /// Register this physics step's valve flow: `dm` kg from the runner into the cylinder
    /// (negative = back-flow into the runner) carrying total specific enthalpy `h`.
    pub fn add_valve_flow(&mut self, i: usize, dm: f64, h: f64) {
        self.acc_mass[i] += dm;
        self.acc_energy[i] += dm * h;
        if dm < 0.0 {
            self.back_mass[i] += dm;
            self.back_energy[i] += dm * h;
        }
    }

    /// Advance one physics step with throttle effective area `throttle_area` (per runner with
    /// ITBs, total for a common throttle). Returns the induction sound source (c·ρu at the
    /// trumpets, at the throttle for a plenum, or at the duct mouths with an air box), held
    /// between FV steps.
    pub fn step(&mut self, throttle_area: f64) -> f64 {
        let inlet = self.inlet;
        self.count += 1;
        self.inducted = 0.0;
        if self.count < self.substeps {
            return self.radiated;
        }
        self.count = 0;
        let dt = self.dt_fv;
        let a = self.a_runner;
        // what the valves took beyond their port cells' limit (`set_valve_boundary`'s): the
        // runners give only that limit, so the excess goes back from the cylinders
        for i in 0..self.acc_mass.len() {
            let limit = 0.5 * self.port_cell_mass(i);
            self.unserved[i] += (self.acc_mass[i] - limit).max(0.0);
        }
        if self.individual {
            let total =
                self.runners.len() + self.remote.0.as_ref().map_or(0, |r| r.result.last.len());
            let mut limit = match &self.remote.0 {
                Some(r) => stable_dt(&self.runners).min(r.result.min_dt),
                None => stable_dt(&self.runners),
            };
            if let Some(b) = &self.airbox {
                limit = limit.min(stable_dt(&b.ducts));
            }
            let k = ((dt / limit).ceil() as usize).clamp(1, MAX_SUBCYCLES);
            let h = dt / k as f64;
            let local = self.runners.len();
            // the box's state is held over the FV step for the runners (the step is a few
            // tens of µs; the box holds litres); the ducts and the box then follow substep by
            // substep
            let up = self.airbox.as_ref().map_or(self.inlet, Airbox::upstream);
            if let Some(r) = self.remote.0.as_mut() {
                // the detached runners go first: they step while these do
                r.acc.clear();
                r.acc
                    .extend((r.first..total).map(|i| (self.acc_mass[i], self.acc_energy[i])));
                r.link.submit(k, dt, throttle_area, up, &r.acc);
            }
            self.acc.clear();
            self.acc
                .extend((0..local).map(|i| (self.acc_mass[i], self.acc_energy[i])));
            self.flux.resize(total * k, 0.0);
            self.eflux.resize(total * k, 0.0);
            itb_runners_step(
                &mut self.runners,
                &self.acc,
                a,
                self.trumpet_cd,
                throttle_area,
                up,
                dt,
                k,
                &mut self.flux[..local * k],
                &mut self.eflux[..local * k],
            );
            if let Some(r) = self.remote.0.as_mut() {
                r.link.collect(&mut r.result);
                self.flux[local * k..].copy_from_slice(&r.result.flux);
                self.eflux[local * k..].copy_from_slice(&r.result.eflux);
            }
            for i in 0..total {
                self.acc_mass[i] = 0.0;
                self.acc_energy[i] = 0.0;
                self.back_mass[i] = 0.0;
                self.back_energy[i] = 0.0;
            }
            // the sums as the runners would add up side by side, substep by substep
            let mut radiated = 0.0;
            for s in 0..k {
                let (mut at_trumpets, mut drawn, mut energy) = (0.0, 0.0, 0.0);
                for i in 0..total {
                    let f = self.flux[i * k + s];
                    drawn += f * a * h;
                    energy += self.eflux[i * k + s] * a * h;
                    at_trumpets += C_AIR * f;
                }
                match &mut self.airbox {
                    Some(b) => {
                        // the trumpets took it from the box; the ducts breathe behind them
                        b.gas.mass_kg -= drawn;
                        b.gas.energy_j -= energy;
                        let (in_at_mouths, at_mouths) = b.breathe(h, inlet);
                        self.inducted += in_at_mouths;
                        radiated += at_mouths / k as f64;
                    }
                    None => {
                        self.inducted += drawn;
                        radiated += at_trumpets / k as f64;
                    }
                }
            }
            self.radiated = radiated;
            self.previous_rate = self.inducted_rate;
            self.inducted_rate = self.inducted / dt;
            return radiated;
        }
        for (i, pipe) in self.runners.iter_mut().enumerate() {
            set_valve_boundary(pipe, self.acc_mass[i], self.acc_energy[i], a, dt);
            self.acc_mass[i] = 0.0;
            self.acc_energy[i] = 0.0;
            self.back_mass[i] = 0.0;
            self.back_energy[i] = 0.0;
        }
        let mut limit = stable_dt(&self.runners);
        if let Some(b) = &self.airbox {
            limit = limit.min(stable_dt(&b.ducts));
        }
        let k = ((dt / limit).ceil() as usize).clamp(1, MAX_SUBCYCLES);
        let h = dt / k as f64;
        let mut radiated = 0.0;
        for _ in 0..k {
            radiated += self.substep(h, throttle_area) / k as f64;
        }
        self.radiated = radiated;
        self.previous_rate = self.inducted_rate;
        self.inducted_rate = self.inducted / dt;
        radiated
    }

    /// One substep of a plenum intake (ITBs: [`itb_runners_step`]).
    fn substep(&mut self, h: f64, throttle_area: f64) -> f64 {
        let inlet = self.inlet;
        let a = self.a_runner;
        // The flow cannot enter through more area than the duct offers.
        let a_eff = throttle_area.max(0.0) * self.trumpet_cd;
        let mut radiated = 0.0;
        {
            // the throttle draws from the air box if there is one, else from the atmosphere
            let reservoir = match &self.airbox {
                Some(b) => crate::gas::Reservoir {
                    pressure_pa: b.gas.pressure(),
                    temperature_k: b.gas.temperature(),
                },
                None => crate::gas::Reservoir {
                    pressure_pa: self.inlet.p,
                    temperature_k: self.inlet.p / (self.inlet.rho * R_AIR),
                },
            };
            let flow = crate::gas::plan_with_reservoir(&self.plenum, &reservoir, a_eff, h, 1.0);
            flow.apply_to(&mut self.plenum);
            let dm = flow.dm;
            match &mut self.airbox {
                Some(b) => {
                    b.gas.mass_kg += dm;
                    b.gas.energy_j += dm * flow.h;
                    let (in_at_mouths, at_mouths) = b.breathe(h, inlet);
                    self.inducted += in_at_mouths;
                    radiated += at_mouths;
                }
                None => {
                    self.inducted -= dm;
                    radiated += C_AIR * (-dm / h) / a;
                }
            }
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
            let away = self
                .remote
                .0
                .as_ref()
                .map_or(&[][..], |r| &r.result.mid_p[..]);
            let sum: f64 = self
                .runners
                .iter()
                .map(mid)
                .chain(away.iter().copied())
                .sum();
            sum / (self.runners.len() + away.len()).max(1) as f64
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

    /// The rate the trumpets draw, kg/s, at this physics step: the FV steps' rates joined by
    /// straight lines (one FV step late) instead of held — a held staircase, differentiated by
    /// a radiator, would spike at every FV step.
    pub fn drawn_rate(&self) -> f64 {
        let f = self.count as f64 / self.substeps as f64;
        self.previous_rate + (self.inducted_rate - self.previous_rate) * f
    }

    /// Step the runners from `first` on elsewhere (ITBs only: they are independent of each
    /// other within an FV step): `link` is handed them. The same results as stepping them here.
    /// False (nothing changed) with a plenum, nothing to hand over, or already detached.
    pub fn detach_runners(
        &mut self,
        first: usize,
        link: impl FnOnce(RunnerSet) -> Box<dyn RunnerLink>,
    ) -> bool {
        if !self.individual || first == 0 || first >= self.runners.len() || self.remote.0.is_some()
        {
            return false;
        }
        let pipes = self.runners.split_off(first);
        let result = RunnerResult {
            flux: Vec::new(),
            eflux: Vec::new(),
            last: pipes.iter().map(|p| p.cells[p.cells.len() - 1]).collect(),
            mid_p: pipes.iter().map(|p| p.prim(p.cells.len() / 2).p).collect(),
            min_dt: stable_dt(&pipes),
        };
        let dx = pipes[0].dx;
        let set = RunnerSet {
            pipes,
            a_runner: self.a_runner,
            trumpet_cd: self.trumpet_cd,
        };
        self.remote = RemoteRunners(Some(Box::new(Remote {
            link: link(set),
            first,
            dx,
            result,
            acc: Vec::new(),
        })));
        true
    }

    /// Stop stepping runners elsewhere: the link, to get them back and [`IntakeFv::attach_runners`].
    pub fn end_detached(&mut self) -> Option<Box<dyn RunnerLink>> {
        self.remote.0.take().map(|r| r.link)
    }

    /// The detached runners back, after the others.
    pub fn attach_runners(&mut self, set: RunnerSet) {
        self.remote = RemoteRunners(None);
        self.runners.extend(set.pipes);
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
            + self.airbox.as_ref().map_or(0.0, Airbox::mass)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runners_split_with_substeps_add_up_the_same() {
        // a physics step longer than the FV's stable one (0.1 ms against ~26 µs): every FV step
        // takes several substeps (k > 1), and half the ITB runners stepped behind a link give
        // the same pipes, the same sound source and the same air drawn
        let def = IntakeDef {
            model: IntakeModel::Fv,
            individual_throttles: true,
            ..IntakeDef::default()
        };
        let mut here = IntakeFv::new(&def, 1e-3, 4, 1e-4);
        let mut split = IntakeFv::new(&def, 1e-3, 4, 1e-4);
        assert!(split.detach_runners(2, |set| Box::new(LocalRunners::new(set))));
        // detached and taken straight back: the same intake as never detached
        let mut round = IntakeFv::new(&def, 1e-3, 4, 1e-4);
        let mut kept = None;
        assert!(round.detach_runners(2, |set| {
            kept = Some(set.clone());
            Box::new(LocalRunners::new(set))
        }));
        assert!(round.end_detached().is_some());
        round.attach_runners(kept.unwrap());
        for k in 0..400 {
            for i in 0..4 {
                let dm = if (k + 25 * i) % 100 < 30 { 2e-6 } else { 0.0 };
                here.add_valve_flow(i, dm, 3.0e5);
                split.add_valve_flow(i, dm, 3.0e5);
                round.add_valve_flow(i, dm, 3.0e5);
            }
            let area = 5e-4;
            let r = round.step(area);
            assert_eq!(here.step(area), split.step(area), "step {k}");
            assert_eq!(r, split.radiated, "round trip, step {k}");
            assert_eq!(here.last_inducted(), split.last_inducted(), "step {k}");
            for i in 0..4 {
                assert_eq!(here.port_volume(i).mass_kg, split.port_volume(i).mass_kg);
            }
            assert_eq!(here.manifold_pressure(), split.manifold_pressure());
        }
        assert!(
            here.flux.len() > 4,
            "k > 1: {} fluxes for 4 runners",
            here.flux.len()
        );
        // a plenum couples its runners every substep: they stay together
        let mut plenum = IntakeFv::new(
            &IntakeDef {
                individual_throttles: false,
                ..def
            },
            1e-3,
            4,
            1e-4,
        );
        assert!(!plenum.detach_runners(2, |set| Box::new(LocalRunners::new(set))));
        assert!(format!("{:?}", plenum).contains("remote: -"));
        // and back in place, the runners carry on as if never away
        let link = split.end_detached();
        assert!(link.is_some());
        assert!(format!("{:?}", split.remote).contains('-'));
    }

    #[test]
    fn the_air_drawn_is_what_the_runners_and_valves_took() {
        // mass conservation over FV steps of several substeps (k > 1): what came in at the
        // trumpets (last_inducted) is what the runners gained plus what the valves took out
        let def = IntakeDef {
            model: IntakeModel::Fv,
            individual_throttles: true,
            ..IntakeDef::default()
        };
        let mut net = IntakeFv::new(&def, 1e-3, 4, 1e-4);
        for k in 0..300 {
            let before = net.total_mass();
            let mut taken = 0.0;
            for i in 0..4 {
                let dm = if (k + 25 * i) % 100 < 30 { 2e-6 } else { 0.0 };
                net.add_valve_flow(i, dm, 3.0e5);
                taken += dm;
            }
            net.step(5e-4);
            let gained = net.total_mass() - before;
            assert!(
                (gained - (net.last_inducted() - taken)).abs() < 1e-9 * before,
                "step {k}: gained {gained}, drawn {} − taken {taken}",
                net.last_inducted()
            );
        }
        assert!(net.flux.len() > 4);
    }

    #[test]
    fn the_drawn_rate_runs_straight_between_fv_steps() {
        // between two FV steps the rate climbs linearly from the earlier step's to the later's
        let mut net = IntakeFv::new(&crate::intake_fv::IntakeDef::default(), 1e-3, 1, 1e-6);
        assert!(net.substeps > 2);
        (net.previous_rate, net.inducted_rate, net.count) = (0.0, 0.08, 0);
        assert_eq!(net.drawn_rate(), 0.0);
        net.count = net.substeps / 2;
        let half = net.count as f64 / net.substeps as f64;
        assert!((net.drawn_rate() - 0.08 * half).abs() < 1e-15);
    }
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

    fn boxed(individual: bool) -> IntakeDef {
        IntakeDef {
            model: IntakeModel::Fv,
            individual_throttles: individual,
            runner_length_m: 0.325,
            runner_diameter_m: 0.045,
            airbox: Some(AirboxDef::lfa_v10_estimate()),
            ..IntakeDef::default()
        }
    }

    /// Amplitude of `x` (sampled at `dt`) at `f`, by a straight DFT (rectangular window).
    fn amplitude_at(x: &[f64], dt: f64, f: f64) -> f64 {
        let (mut re, mut im) = (0.0, 0.0);
        for (i, v) in x.iter().enumerate() {
            let ph = 2.0 * std::f64::consts::PI * f * i as f64 * dt;
            re += v * ph.cos();
            im += v * ph.sin();
        }
        2.0 * (re * re + im * im).sqrt() / x.len() as f64
    }

    #[test]
    fn the_lfa_surge_tank_estimate_resonates_below_its_idle_firing() {
        // Two ducts (EPC 176966A); Helmholtz by hand, f = (c/2π)·√(n·A/(L'·V)) with
        // c = 347.2 m/s (300 K), A = π·0.045² = 6.362e-3 m², L' = 0.40 + 1.45 · 0.045 =
        // 0.46525 m, V = 0.020 m³: 55.26 · √(1.2723e-2 / 9.305e-3) = 64.6 Hz — just above
        // the idle's firing (650 rpm × 5 / 60 = 54 Hz), so the idle pulse passes the ducts
        let b = AirboxDef::lfa_v10_estimate();
        assert_eq!(b.ducts, 2);
        let a = std::f64::consts::PI * 0.25 * b.duct_diameter_m.powi(2);
        let f = 347.2 / (2.0 * std::f64::consts::PI)
            * (2.0 * a / (b.effective_duct_length() * b.volume_m3)).sqrt();
        assert!((f - 64.6).abs() < 0.5, "{f} Hz");
        assert!((b.effective_duct_length() - 0.46525).abs() < 1e-9);
        assert!(f > 54.0 && f < 1.5 * 54.0);
    }

    #[test]
    fn airbox_round_trips_and_an_old_file_has_none() {
        let def = boxed(true);
        let json = serde_json::to_string(&def).unwrap();
        assert_eq!(serde_json::from_str::<IntakeDef>(&json).unwrap(), def);
        let yaml = serde_yaml_ng::to_string(&def).unwrap();
        assert_eq!(serde_yaml_ng::from_str::<IntakeDef>(&yaml).unwrap(), def);
        assert!(yaml.contains("duct_length_m: 0.4"));
        let old: IntakeDef =
            serde_yaml_ng::from_str("model: fv\nindividual_throttles: true\n").unwrap();
        assert_eq!(old.airbox, None);
        assert_eq!(IntakeDef::default().airbox, None);
    }

    #[test]
    fn the_airbox_and_its_ducts_ring_at_the_helmholtz_frequency() {
        // Box V = 20 L breathing through two Ø90 mm ducts of 0.40 m. Effective duct length with
        // end corrections L' = L + 0.85a (flanged at the box, Rayleigh) + 0.6a (unflanged mouth,
        // Levine–Schwinger) = 0.46525 m (Kinsler, Frey, Coppens & Sanders, Fundamentals of
        // Acoustics 4th ed., §10.8–10.9). Lumped Helmholtz f = (c/2π)√(A_t/(L'V)) = 64.6 Hz with
        // c = √(1.4·287·300) = 347.2 m/s, A_t = 2·6.3617e-3 m². The ducts are 0.47 m long,
        // λ/4 ≈ 1.4 m, so the distributed line's resonance (Kinsler §10.9: x·tan x = A_t·L'/V
        // = 0.29598, x = kL' = 0.51861) sits lower: f = c·x/(2π·L') = 61.59 Hz. A short draw
        // through the runners sets it ringing; the box pressure then swings at that frequency.
        let dt = 1e-5;
        let mut net = IntakeFv::new(&boxed(true), 1e-3, 1, dt);
        let a = net.a_runner;
        let mut p = Vec::new();
        for i in 0..(0.6 / dt) as usize {
            // 1 g drawn in the first 10 ms: a 2 % dip in the 24 g the box holds
            let dm = if i < 1000 { 1e-6 } else { 0.0 };
            net.add_valve_flow(0, dm, CP * 300.0);
            net.step(a);
            if i >= 10_000 {
                p.push(net.airbox_pressure().unwrap() - ATM_PA);
            }
        }
        let swing = p.iter().cloned().fold(0.0_f64, |m, v| m.max(v.abs()));
        assert!(swing > 100.0, "the box rings by {swing} Pa");
        let mean = p.iter().sum::<f64>() / p.len() as f64;
        let x: Vec<f64> = p.iter().map(|v| v - mean).collect();
        let (mut best, mut at) = (0.0, 0.0);
        let mut f = 30.0;
        while f < 120.0 {
            let amp = amplitude_at(&x, dt, f);
            if amp > best {
                (best, at) = (amp, f);
            }
            f += 0.25;
        }
        assert!((at / 61.59 - 1.0).abs() < 0.04, "rings at {at} Hz");
    }

    #[test]
    fn the_airbox_low_passes_the_trumpets_pulsation() {
        // The same box; one cylinder's runner (0.325 m, Ø45 mm) draws 20 g/s · (1 + sin 2πft)
        // at the valve with f = 2 × 61.59 = 123.19 Hz. Mouth volume velocity over the valve's,
        // by transfer matrices (Munjal, Acoustics of Ducts and Mufflers, ch. 2: a line of
        // length L is [cos kL, jZ sin kL; (j/Z) sin kL, cos kL], Z = ρc/A; a volume shunts
        // jωC = j·kV/(ρc)): the ducts as one line of length L' and area A_t open at the mouth
        // (p = 0), the box on its end, the runner line between the valve and the box,
        //   Q_valve/Q_mouth = cos kL_r·[cos kL' − (kV/A_t)·sin kL'] − (A_r/A_t)·sin kL_r·sin kL'.
        // k = 2πf/c = 2.2292 m⁻¹: kL' = 1.0371 (cos 0.5087, sin 0.8610), kL_r = 0.7245
        // (cos 0.7488, sin 0.6628), kV/A_t = 3.5041, A_r/A_t = 0.125:
        //   Q_valve/Q_mouth = 0.7488·(0.5087 − 3.0169) − 0.125·0.6628·0.8610 = −1.9496,
        // |H| = 0.5129 (−5.8 dB; the ducts and box alone would give 0.3986, the lumped
        // 1/((f/f_H)² − 1) 0.38). The mouths' source is c·Σρu: the mouth mass flux amplitude
        // summed over the two ducts is |H|·ṁ₁/A_duct.
        let dt = 1e-5;
        let f = 2.0 * 61.59;
        let mut net = IntakeFv::new(&boxed(true), 1e-3, 1, dt);
        let a = net.a_runner;
        let m1 = 0.02;
        let mut src = Vec::new();
        let n = (1.2 / dt) as usize;
        for i in 0..n {
            let t = i as f64 * dt;
            // the draw fades in over 0.3 s so the box's own mode is not set ringing (a step
            // of the mean flow alone rings it for seconds: the ducts' only loss is friction)
            let ramp = (t / 0.3).min(1.0);
            let dm = ramp * m1 * (1.0 + (2.0 * std::f64::consts::PI * f * t).sin()) * dt;
            net.add_valve_flow(0, dm, CP * 300.0);
            let r = net.step(a);
            if i >= n / 2 {
                src.push(r);
            }
        }
        let a_duct = net.airbox.as_ref().unwrap().a_duct;
        let h = amplitude_at(&src, dt, f) / (C_AIR * m1 / a_duct);
        assert!((h / 0.5129 - 1.0).abs() < 0.03, "|H| = {h} at 2 f_H");
        // and the box's own mode was not set ringing: nothing to speak of at f_H
        let at_fh = amplitude_at(&src, dt, 61.59) / (C_AIR * m1 / a_duct);
        assert!(at_fh < 0.05, "{at_fh} at f_H");
    }

    #[test]
    fn the_air_drawn_through_the_ducts_is_what_box_ducts_runners_and_valves_took() {
        // mass conservation with an airbox, ITBs and a common throttle alike: what came in at
        // the duct mouths (last_inducted) is what the box, ducts and runners gained plus what
        // the valves took out
        for individual in [true, false] {
            let mut net = IntakeFv::new(&boxed(individual), 1e-3, 4, 1e-4);
            let mut lowest = ATM_PA;
            for k in 0..300 {
                let before = net.total_mass();
                let mut taken = 0.0;
                for i in 0..4 {
                    let dm = if (k + 25 * i) % 100 < 30 { 2e-6 } else { 0.0 };
                    net.add_valve_flow(i, dm, 3.0e5);
                    taken += dm;
                }
                net.step(5e-4);
                lowest = lowest.min(net.airbox_pressure().unwrap());
                let gained = net.total_mass() - before;
                assert!(
                    (gained - (net.last_inducted() - taken)).abs() < 1e-9 * before,
                    "individual {individual}, step {k}: gained {gained}, drawn {} − taken {taken}",
                    net.last_inducted()
                );
            }
            // the draw pulled the box below the atmosphere at some point, and the air that
            // left took its energy along: the box stays at the ambient 300 K (the reversion
            // carries 3.0e5 J/kg = c_p · 298 K)
            assert!(lowest < ATM_PA - 1.0, "{lowest} Pa");
            let t = net.airbox.as_ref().unwrap().gas.temperature();
            assert!((t - 300.0).abs() < 2.0, "{t} K");
            assert!(net.airbox_pressure().unwrap().is_finite());
            // the runners detached and stepped elsewhere draw from the same box
            let mut here = IntakeFv::new(&boxed(individual), 1e-3, 4, 1e-4);
            let mut split = IntakeFv::new(&boxed(individual), 1e-3, 4, 1e-4);
            assert_eq!(
                split.detach_runners(2, |set| Box::new(LocalRunners::new(set))),
                individual
            );
            for k in 0..200 {
                for i in 0..4 {
                    let dm = if (k + 25 * i) % 100 < 30 { 2e-6 } else { 0.0 };
                    here.add_valve_flow(i, dm, 3.0e5);
                    split.add_valve_flow(i, dm, 3.0e5);
                }
                assert_eq!(here.step(5e-4), split.step(5e-4), "step {k}");
                assert_eq!(here.airbox_pressure(), split.airbox_pressure(), "step {k}");
            }
        }
    }

    #[test]
    fn the_valve_draws_the_gas_its_backflow_left_in_the_port() {
        // Within one FV step the cylinder pushes hot gas back into the runner, then draws again:
        // it must draw the mixture now in the port cell, not the cell as the FV step began.
        // Drawing the stale cold gas left the backflow's heat behind with no mass: a runner of a
        // bridge-ported 13B reached 4600 K after a closed-throttle shift (FINDINGS 2026-10-31).
        // The cell's m kg at 300 K takes m/2 of 1200 K gas (enthalpy c_p·1200): its internal
        // energy m·c_v·300 + m/2·c_p·1200 over 1.5·m·c_v: (300 + 0.7·1200)/1.5 = 760 K
        let mut net = IntakeFv::new(&def(false), 1e-3, 1, DT);
        let cold = net.port_volume(0);
        let cv = crate::gas::CV;
        let t0 = cold.energy_j / (cold.mass_kg * cv);
        assert!((t0 - 300.0).abs() < 5.0, "{t0}");
        let m = cold.mass_kg;
        // the cell at exactly 300 K for the hand value
        let scale = 300.0 / t0;
        net.add_valve_flow(0, -0.5 * m, CP * 1200.0 * scale);
        let port = net.port_volume(0);
        let t = port.energy_j / (port.mass_kg * cv);
        assert!((t / t0 - 760.0 / 300.0).abs() < 0.01, "port at {t} K");
        // at the cell's pressure: the runner holds it until the FV step
        assert!((port.energy_j / cold.energy_j - 1.0).abs() < 1e-12);
        // a backflow then a larger draw within the step: the gas pushed back is still drawn
        // first (the net flow being out of the runner does not hide it)
        net.add_valve_flow(0, 0.8 * m, CP * 760.0 * scale);
        let after = net.port_volume(0);
        assert!((after.energy_j / (after.mass_kg * cv) / t0 - 760.0 / 300.0).abs() < 0.01);
        // a draw since the step began leaves the cell as it was (its neighbours refill it)
        let mut net = IntakeFv::new(&def(false), 1e-3, 1, DT);
        net.add_valve_flow(0, 0.3 * m, CP * 300.0);
        assert_eq!(net.port_volume(0), cold);
    }

    #[test]
    fn a_port_wider_than_its_runner_couples_on_a_finer_fv_step() {
        // a port that passes more than the runner's own section (a bridge-ported 13B's,
        // 1.34×) empties the port cell faster than the duct refills it: held over a whole FV
        // step the valve and the cell flipped the flow every step, a whistle at half the FV
        // rate (FINDINGS 2026-10-31); every 4 physics steps it is gone, every 8 it was not
        let net = IntakeFv::new(&def(false), 1e-3, 1, DT);
        let a = net.a_runner;
        let (wide, narrow) = (
            net.clone().coupled_to_port(1.3 * a),
            net.clone().coupled_to_port(0.96 * a),
        );
        assert_eq!(narrow.substeps, net.substeps, "a valve's port leaves it");
        assert!(net.substeps > STIFF_PORT_SUBSTEPS);
        assert_eq!(wide.substeps, STIFF_PORT_SUBSTEPS);
        assert_eq!(wide.dt_fv, STIFF_PORT_SUBSTEPS as f64 * DT);
    }

    #[test]
    fn a_draw_the_port_cell_cannot_serve_is_handed_back() {
        // the valve took more over one FV step than the runner's flux limit lets the port cell
        // give (half of it): the excess was never in the runner, the cylinder must give it back
        // (it was created there: a bridge-ported 13B's port starved into a 4600 K plug)
        let mut net = IntakeFv::new(&def(false), 1e-3, 1, DT);
        let cell = net.port_volume(0).mass_kg;
        net.add_valve_flow(0, 0.8 * cell, CP * 300.0);
        while net.count + 1 < net.substeps {
            net.step(1e-3);
        }
        assert_eq!(net.take_unserved(0), 0.0, "not before the FV step");
        net.step(1e-3);
        let back = net.take_unserved(0);
        assert!((back - 0.3 * cell).abs() < 1e-9 * cell, "{back} of {cell}");
        assert_eq!(net.take_unserved(0), 0.0, "taken once");
        // within the limit nothing comes back
        let mut net = IntakeFv::new(&def(false), 1e-3, 1, DT);
        net.add_valve_flow(0, 0.3 * cell, CP * 300.0);
        for _ in 0..net.substeps {
            net.step(1e-3);
        }
        assert_eq!(net.take_unserved(0), 0.0);
    }
}
