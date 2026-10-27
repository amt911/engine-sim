//! Quasi-1D compressible Euler solver for pipes: finite volumes, HLLC Riemann solver (Toro,
//! *Riemann Solvers and Numerical Methods for Fluid Dynamics*, 3rd ed., ch. 10), WENO5-Z
//! reconstruction (Borges, Carmona, Costa & Don, J. Comput. Phys. 227 (2008)) and SSP-RK3 time
//! stepping (Shu & Osher, J. Comput. Phys. 77 (1988)). Non-linear: pulses steepen into shocks,
//! which is what makes a loaded exhaust sound harsh (research addendum).
//!
//! Why fifth order: the pipes carry sound up to a few kHz at ~2–4 cells per σ of a pulse. A
//! second-order TVD scheme (MUSCL, van Leer) kept 57 % of a smooth pulse's peak at 2.5 cells per
//! σ after 0.34 m; WENO5-Z with RK3 keeps 89 %, and the Sod error drops by a third
//! (`examples/fv_accuracy.rs`). The scheme's own damping no longer eats the exhaust's rasp.
//!
//! Conserved per cell (per unit area): ρ, ρu, E = p/(γ−1) + ½ρu². γ = 1.4 (as `gas`).

use crate::gas::GAMMA;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prim {
    pub rho: f64,
    pub u: f64,
    pub p: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Cons {
    pub rho: f64,
    pub mom: f64,
    pub e: f64,
}

impl Prim {
    pub fn to_cons(self) -> Cons {
        Cons {
            rho: self.rho,
            mom: self.rho * self.u,
            e: self.p / (GAMMA - 1.0) + 0.5 * self.rho * self.u * self.u,
        }
    }
    pub fn sound_speed(self) -> f64 {
        (GAMMA * self.p / self.rho).sqrt()
    }
}

impl Cons {
    pub fn to_prim(self) -> Prim {
        let u = self.mom / self.rho;
        Prim {
            rho: self.rho,
            u,
            p: (GAMMA - 1.0) * (self.e - 0.5 * self.rho * u * u),
        }
    }
}

/// HLLC flux across an interface with left/right states (Toro §10.4, Davis wave speeds).
pub fn hllc(l: Prim, r: Prim) -> Cons {
    let flux = |s: Prim| {
        let e = s.p / (GAMMA - 1.0) + 0.5 * s.rho * s.u * s.u;
        (
            Cons {
                rho: s.rho * s.u,
                mom: s.rho * s.u * s.u + s.p,
                e: s.u * (e + s.p),
            },
            e,
        )
    };
    let (cl, cr) = (l.sound_speed(), r.sound_speed());
    let sl = (l.u - cl).min(r.u - cr);
    let sr = (l.u + cl).max(r.u + cr);
    let (fl, el) = flux(l);
    let (fr, er) = flux(r);
    if sl >= 0.0 {
        return fl;
    }
    if sr <= 0.0 {
        return fr;
    }
    let ss = (r.p - l.p + l.rho * l.u * (sl - l.u) - r.rho * r.u * (sr - r.u))
        / (l.rho * (sl - l.u) - r.rho * (sr - r.u));
    let star = |s: Prim, e: f64, sk: f64| {
        let k = s.rho * (sk - s.u) / (sk - ss);
        Cons {
            rho: k,
            mom: k * ss,
            e: k * (e / s.rho + (ss - s.u) * (ss + s.p / (s.rho * (sk - s.u)))),
        }
    };
    let (u, f, sk, ustar) = if ss >= 0.0 {
        (l.to_cons(), fl, sl, star(l, el, sl))
    } else {
        (r.to_cons(), fr, sr, star(r, er, sr))
    };
    Cons {
        rho: f.rho + sk * (ustar.rho - u.rho),
        mom: f.mom + sk * (ustar.mom - u.mom),
        e: f.e + sk * (ustar.e - u.e),
    }
}

/// What sits beyond an end of the pipe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Boundary {
    /// Closed end (reflecting wall).
    Wall,
    /// Open to a reservoir at pressure `p` and temperature-equivalent density `rho`.
    Open { p: f64, rho: f64 },
    /// A prescribed state (e.g. set by a coupled valve or junction each step).
    State(Prim),
    /// A prescribed face flux (per unit area, positive = along +x), e.g. a valve's mass flow.
    Flux(Cons),
}

/// How a pipe reconstructs its cells and steps in time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Scheme {
    /// WENO5-Z + SSP-RK3: fifth order in space, a sound pulse keeps 89 % of its peak where
    /// MUSCL keeps 57 % (v0.11.0). The reference.
    #[default]
    Weno5Rk3,
    /// MUSCL with the van Leer limiter + SSP-RK2 (Toro, *Riemann Solvers*, §13–14): second
    /// order, two stages of a cheap reconstruction, about a third of the cost.
    MusclRk2,
}

/// A uniform pipe of `n` cells.
#[derive(Debug, Clone)]
pub struct Pipe1d {
    pub cells: Vec<Cons>,
    pub scheme: Scheme,
    pub dx: f64,
    pub left: Boundary,
    pub right: Boundary,
    /// Scratch buffers (no allocation per step): primitive states with three ghosts a side,
    /// each cell's reconstructed left and right edge states, face fluxes, the step's starting
    /// state, and the right-hand side.
    w: Vec<Prim>,
    /// Volume flow u·A of every entry of `w` (scratch), and −f/(2D) per cell, from the areas.
    qv: Vec<f64>,
    /// Density and pressure of every entry of `w` (scratch, laid out for the vector unit), and
    /// the reconstructed edges of ρ, u·A and p ([left, right] per cell, WENO only).
    rho_w: Vec<f64>,
    p_w: Vec<f64>,
    recon: [Vec<f64>; 6],
    wall_coef: Vec<f64>,
    edge_l: Vec<Prim>,
    edge_r: Vec<Prim>,
    fluxes: Vec<Cons>,
    u0: Vec<Cons>,
    rhs: Vec<Cons>,
    /// Flux through the left and right end faces in the last step (RK-averaged), per area.
    pub end_flux: (Cons, Cons),
    /// Density and pressure floors (see [`positive`]); tiny by default.
    pub floors: (f64, f64),
    /// Cross-section per cell and per face (n + 1), m²; all 1 for a uniform pipe, so the
    /// per-area formulation reduces to plain 1D.
    pub area: Vec<f64>,
    face_area: Vec<f64>,
    /// Darcy friction factor of the wall (0 = inviscid). Momentum source −f/(2D)·ρu|u| with
    /// D the hydraulic diameter √(4A/π); the dissipated work stays in the gas as heat (Winterbone &
    /// Pearson; the standard 1D engine-code wall term). Without it pipe resonances never decay.
    pub friction: f64,
    /// Amplitude loss of the waves per metre (Np/m) at the wall, from its viscous and thermal
    /// boundary layers: a momentum sink −2αc·ρ(u − ū) on the motion about the cell's slow mean
    /// ū (Darcy's term above takes the mean flow). 0 = none.
    pub acoustic_loss: f64,
    /// Each cell's slow mean velocity ū (time constant [`MEAN_FLOW_TAU_S`]), for the acoustic
    /// loss.
    u_mean: Vec<f64>,
    /// Heat exchange with the wall by Reynolds' analogy (needs `friction`): Benson vol. 1, eqs.
    /// 7.91–7.92, h = (f/2)·ρ|u|c_p with f Fanning's (Darcy's/4), so the gas gains
    /// ρq = (f_D/2D)·ρ|u|c_p·(T_w − T) per volume. Real tubes lose energy to their walls as well
    /// as to friction; without it pipe resonances ring too long (Ange, *Improving My Engine Sound
    /// Simulator With Real Data*).
    pub wall_heat: bool,
    /// The wall's temperature, K: it takes the gas's heat into its thermal mass and gives it to
    /// the ambient by free convection and radiation (see [`Pipe1d::step`]).
    pub wall_t: f64,
}

/// The pipe wall: a 16-gauge (1.5 mm) carbon-steel tube, as tubular headers are built
/// (Incropera et al., *Fundamentals of Heat and Mass Transfer*, Table A.1, AISI 1010 at 600 K:
/// ρ = 7832 kg/m³, c_p = 559 J/(kg·K)).
const WALL_THICKNESS_M: f64 = 1.5e-3;
const WALL_RHO_CP: f64 = 7832.0 * 559.0;
/// Outside the pipe: free convection to still air, h ≈ 10 W/(m²·K) (Incropera, Table 1.1:
/// 2–25 for gases), and radiation from oxidized steel, ε = 0.79 (Table A.11), to 300 K.
const WALL_H_FREE: f64 = 10.0;
const WALL_EMISSIVITY: f64 = 0.79;
const AMBIENT_T_K: f64 = 300.0;
const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;

/// Time constant of the cells' mean velocity ū under the acoustic wall loss, s: slower than any
/// engine order (an idling four-cylinder fires at ~27 Hz), so the loss acts on the waves only.
const MEAN_FLOW_TAU_S: f64 = 0.2;

impl Pipe1d {
    pub fn new(length: f64, n: usize, init: Prim) -> Self {
        let n = n.max(1);
        Self {
            cells: vec![init.to_cons(); n],
            scheme: Scheme::default(),
            dx: length / n as f64,
            left: Boundary::Wall,
            right: Boundary::Wall,
            w: Vec::with_capacity(n + 6),
            qv: vec![0.0; n + 6],
            rho_w: vec![0.0; n + 6],
            p_w: vec![0.0; n + 6],
            recon: std::array::from_fn(|_| vec![0.0; n + 6]),
            wall_coef: Vec::new(),
            edge_l: vec![init; n + 6],
            edge_r: vec![init; n + 6],
            fluxes: vec![Cons::default(); n + 1],
            u0: vec![Cons::default(); n],
            rhs: vec![Cons::default(); n],
            end_flux: (Cons::default(), Cons::default()),
            floors: (1e-9, 1e-9),
            area: vec![1.0; n],
            face_area: vec![1.0; n + 1],
            friction: 0.0,
            acoustic_loss: 0.0,
            u_mean: Vec::new(),
            wall_heat: false,
            wall_t: init.p / (init.rho * crate::gas::R_AIR),
        }
    }
    /// Largest stable step for Courant number `cfl`.
    pub fn max_dt(&self, cfl: f64) -> f64 {
        let vmax = self
            .cells
            .iter()
            .map(|c| {
                let p = c.to_prim();
                p.u.abs() + p.sound_speed()
            })
            .fold(1e-9, f64::max);
        cfl * self.dx / vmax
    }
    /// A pipe made of consecutive segments `(length, cross-section area)`: quasi-1D Euler with
    /// the geometric source p·dA/dx (Toro §1.6.3; Winterbone & Pearson), so area changes
    /// (collector → muffler can → tail) transmit and reflect waves without 0D junctions.
    pub fn segmented(segments: &[(f64, f64)], cell: f64, init: Prim) -> Self {
        let counts: Vec<usize> = segments
            .iter()
            .map(|(l, _)| ((l / cell).round() as usize).max(1))
            .collect();
        let n: usize = counts.iter().sum();
        let mut p = Pipe1d::new(n as f64 * cell, n, init);
        p.area = segments
            .iter()
            .zip(&counts)
            .flat_map(|((_, a), k)| std::iter::repeat_n(*a, *k))
            .collect();
        p.face_area[0] = p.area[0];
        p.face_area[n] = p.area[n - 1];
        for k in 1..n {
            p.face_area[k] = 0.5 * (p.area[k - 1] + p.area[k]);
        }
        p
    }

    /// Cross-section of the end faces (left, right), m² (1 for a uniform pipe).
    pub fn end_areas(&self) -> (f64, f64) {
        (self.face_area[0], self.face_area[self.cells.len()])
    }

    /// Advance by `dt`: SSP-RK3 (Shu & Osher), U¹ = Uⁿ + Δt·L(Uⁿ), U² = ¾Uⁿ + ¼(U¹ + Δt·L(U¹)),
    /// Uⁿ⁺¹ = ⅓Uⁿ + ⅔(U² + Δt·L(U²)); the end fluxes are weighted (1, 1, 4)/6 like the stages.
    pub fn step(&mut self, dt: f64) {
        let n = self.cells.len();
        if self.acoustic_loss > 0.0 && self.u_mean.len() != n {
            self.u_mean = self.cells.iter().map(|c| c.mom / c.rho).collect();
        }
        self.u0.copy_from_slice(&self.cells);
        let mut ends = (Cons::default(), Cons::default());
        // (share of Uⁿ kept, weight of the stage's end flux)
        let stages: &[(f64, f64)] = match self.scheme {
            Scheme::Weno5Rk3 => &[(0.0, 1.0 / 6.0), (0.75, 1.0 / 6.0), (1.0 / 3.0, 4.0 / 6.0)],
            // U¹ = Uⁿ + Δt·L(Uⁿ), Uⁿ⁺¹ = ½Uⁿ + ½(U¹ + Δt·L(U¹)) (Shu & Osher)
            Scheme::MusclRk2 => &[(0.0, 0.5), (0.5, 0.5)],
        };
        for (stage, &(keep, w)) in stages.iter().enumerate() {
            self.eval_rhs();
            ends.0 = add(ends.0, self.fluxes[0], w);
            ends.1 = add(ends.1, self.fluxes[n], w);
            for ((c, d), c0) in self.cells.iter_mut().zip(&self.rhs).zip(&self.u0) {
                let u = add(*c, *d, dt);
                let next = if stage == 0 { u } else { mix(*c0, u, keep) };
                *c = positive(next, self.floors.0, self.floors.1);
            }
        }
        self.end_flux = ends;
        if self.wall_heat {
            self.wall_step(dt);
        }
        if self.acoustic_loss > 0.0 {
            let k = (dt / MEAN_FLOW_TAU_S).min(1.0);
            for (m, c) in self.u_mean.iter_mut().zip(&self.cells) {
                *m += k * (c.mom / c.rho - *m);
            }
        }
    }

    /// The wall over `dt`: the heat the gas gives it (Reynolds' analogy, as in the energy
    /// equation) into its thermal mass ρc_p·πD·t per length, less what it loses outside,
    /// (h_free + εσ(T_w² + T_a²)(T_w + T_a))·π(D + 2t)·(T_w − T_a).
    fn wall_step(&mut self, dt: f64) {
        let pi = std::f64::consts::PI;
        let (mut q, mut c, mut outside) = (0.0, 0.0, 0.0);
        let tw = self.wall_t;
        let ta = AMBIENT_T_K;
        let h_out =
            WALL_H_FREE + WALL_EMISSIVITY * STEFAN_BOLTZMANN * (tw * tw + ta * ta) * (tw + ta);
        for (j, cell) in self.cells.iter().enumerate() {
            let p = cell.to_prim();
            let t = p.p / (p.rho * crate::gas::R_AIR);
            let d = (4.0 * self.area[j] / pi).sqrt();
            q += self.friction / (2.0 * d)
                * p.rho
                * p.u.abs()
                * crate::gas::CP
                * (t - tw)
                * self.area[j]
                * self.dx;
            c += WALL_RHO_CP * pi * d * WALL_THICKNESS_M * self.dx;
            outside += h_out * pi * (d + 2.0 * WALL_THICKNESS_M) * self.dx * (tw - ta);
        }
        if c > 0.0 {
            self.wall_t += dt * (q - outside) / c;
        }
    }

    /// Mass flow per unit area through the right end during the last step (ρu at the face).
    pub fn prim(&self, i: usize) -> Prim {
        self.cells[i].to_prim()
    }
}

/// Face flux for a boundary that imposes a mass flux `mass_flux` (kg/(s·m²), +x) and energy
/// flux `energy_flux`, next to `cell`. The crossing gas moves at u = ṁ″/ρ, at most sonic (a
/// valve or throttle cannot push gas faster than sound; ρ near the floor would otherwise blow
/// the momentum flux up).
pub fn flux_boundary(mass_flux: f64, energy_flux: f64, cell: Prim) -> Cons {
    let c = cell.sound_speed();
    let u = (mass_flux / cell.rho).clamp(-c, c);
    Cons {
        rho: mass_flux,
        mom: mass_flux * u + cell.p,
        e: energy_flux,
    }
}

/// `keep`·a + (1 − `keep`)·b
fn mix(a: Cons, b: Cons, keep: f64) -> Cons {
    let k = 1.0 - keep;
    Cons {
        rho: keep * a.rho + k * b.rho,
        mom: keep * a.mom + k * b.mom,
        e: keep * a.e + k * b.e,
    }
}

fn add(c: Cons, d: Cons, dt: f64) -> Cons {
    Cons {
        rho: c.rho + dt * d.rho,
        mom: c.mom + dt * d.mom,
        e: c.e + dt * d.e,
    }
}

/// Keep a cell physical: density and pressure at least the floors, NaN reset to rest.
/// Extreme forcing (or a strong rarefaction against a prescribed flux) can push a cell
/// negative, which a Riemann solver cannot recover from.
/// Velocity bound of the positivity guard, m/s.
const U_MAX: f64 = 2000.0;

fn positive(c: Cons, rho_floor: f64, p_floor: f64) -> Cons {
    if !(c.rho.is_finite() && c.mom.is_finite() && c.e.is_finite()) || c.rho < rho_floor {
        return Prim {
            rho: rho_floor,
            u: 0.0,
            p: p_floor,
        }
        .to_cons();
    }
    // No gas in these ducts moves near 2 km/s; a runaway velocity would overflow ½ρu² (inf − inf).
    let u = (c.mom / c.rho).clamp(-U_MAX, U_MAX);
    let c = Cons {
        mom: c.rho * u,
        ..c
    };
    let e_min = 0.5 * c.rho * u * u + p_floor / (GAMMA - 1.0);
    if c.e < e_min {
        Cons { e: e_min, ..c }
    } else {
        c
    }
}

/// A reconstructed edge state if it is physical (ρ > 0 and p > 0), else the cell's own state.
fn physical_or(edge: Prim, cell: Prim) -> Prim {
    if edge.rho > 0.0 && edge.p > 0.0 {
        edge
    } else {
        cell
    }
}

/// MUSCL edge values of cell `s` between `a` and `c`: (left edge, right edge), the slope
/// limited by van Leer's harmonic mean (smooth and TVD; Toro, *Riemann Solvers*, §13.8).
fn muscl(a: f64, s: f64, c: f64) -> (f64, f64) {
    let (l, r) = (s - a, c - s);
    let slope = if l * r <= 0.0 {
        0.0
    } else {
        2.0 * l * r / (l + r)
    };
    (s - 0.5 * slope, s + 0.5 * slope)
}

/// A number the reconstruction can run on: one `f64`, or four at once on the vector unit. Both
/// do the same IEEE operations in the same order, so a lane gives what the scalar gives.
trait Lane:
    Copy
    + std::ops::Add<Output = Self>
    + std::ops::Sub<Output = Self>
    + std::ops::Mul<Output = Self>
    + std::ops::Div<Output = Self>
    + std::ops::Neg<Output = Self>
{
    fn k(x: f64) -> Self;
    fn abs(self) -> Self;
}

impl Lane for f64 {
    fn k(x: f64) -> Self {
        x
    }
    fn abs(self) -> Self {
        f64::abs(self)
    }
}

impl Lane for wide::f64x4 {
    fn k(x: f64) -> Self {
        wide::f64x4::splat(x)
    }
    fn abs(self) -> Self {
        wide::f64x4::abs(self)
    }
}

/// WENO5-Z edge values of the middle cell of `v` = [v₋₂, v₋₁, v₀, v₁, v₂]: (left edge, right
/// edge). Three third-order candidates weighted by their smoothness (Jiang & Shu, J. Comput.
/// Phys. 126 (1996)) with the Z weights of Borges et al. (2008), which keep fifth order at
/// smooth extrema, the crest of a sound wave. The left edge mirrors the stencil, so both share
/// the smoothness indicators.
fn weno5z<T: Lane>(v: [T; 5]) -> (T, T) {
    let [a, b, c, d, e] = v;
    let k = T::k;
    let sq = |x: T| x * x;
    let b0 = k(13.0 / 12.0) * sq(a - k(2.0) * b + c) + k(0.25) * sq(a - k(4.0) * b + k(3.0) * c);
    let b1 = k(13.0 / 12.0) * sq(b - k(2.0) * c + d) + k(0.25) * sq(b - d);
    let b2 = k(13.0 / 12.0) * sq(c - k(2.0) * d + e) + k(0.25) * sq(k(3.0) * c - k(4.0) * d + e);
    let tau = (b0 - b2).abs();
    // ε guards a flat stencil (β = 0). 1e-40 (as in Borges et al.) keeps (τ/ε)² finite for
    // any τ these pipes produce (it would overflow only past τ ≈ 1e114); a denormal-sized ε
    // makes it infinite and the weights ∞/∞.
    let z = |beta: T| k(1.0) + sq(tau / (beta + k(1e-40)));
    let (z0, z1, z2) = (z(b0), z(b1), z(b2));
    // q_k = (stencil polynomial)/6: the 1/6 folds into the normalisation, one division each
    let right = {
        let (a0, a1, a2) = (k(0.1) * z0, k(0.6) * z1, k(0.3) * z2);
        (a0 * (k(2.0) * a - k(7.0) * b + k(11.0) * c)
            + a1 * (-b + k(5.0) * c + k(2.0) * d)
            + a2 * (k(2.0) * c + k(5.0) * d - e))
            / (k(6.0) * (a0 + a1 + a2))
    };
    let left = {
        let (a0, a1, a2) = (k(0.1) * z2, k(0.6) * z1, k(0.3) * z0);
        (a0 * (k(2.0) * e - k(7.0) * d + k(11.0) * c)
            + a1 * (-d + k(5.0) * c + k(2.0) * b)
            + a2 * (k(2.0) * c + k(5.0) * b - a))
            / (k(6.0) * (a0 + a1 + a2))
    };
    (left, right)
}

/// [`weno5z`] over every entry of `v` with two neighbours a side, into `left` and `right`
/// (entries 2..len − 2): four cells at a time on the vector unit, the rest one by one.
fn weno5z_run(v: &[f64], left: &mut [f64], right: &mut [f64]) {
    use wide::f64x4;
    let end = v.len() - 2;
    let at = |i: usize| f64x4::from([v[i], v[i + 1], v[i + 2], v[i + 3]]);
    let mut i = 2;
    while i + 4 <= end {
        let (l, r) = weno5z([at(i - 2), at(i - 1), at(i), at(i + 1), at(i + 2)]);
        left[i..i + 4].copy_from_slice(&l.to_array());
        right[i..i + 4].copy_from_slice(&r.to_array());
        i += 4;
    }
    for i in i..end {
        let (l, r) = weno5z([v[i - 2], v[i - 1], v[i], v[i + 1], v[i + 2]]);
        left[i] = l;
        right[i] = r;
    }
}

/// [`muscl`] over every entry of `v` with a neighbour a side, into `left` and `right`.
fn muscl_run(v: &[f64], left: &mut [f64], right: &mut [f64]) {
    for i in 1..v.len() - 1 {
        let (l, r) = muscl(v[i - 1], v[i], v[i + 1]);
        left[i] = l;
        right[i] = r;
    }
}

/// An open end to a reservoir at rest (`p0`, `rho0`), continuous through a reversal of the
/// flow. Along the characteristic that reaches the end from inside, p + Z·v is the interior's
/// (Z = ρc of the interior gas, v the velocity along `outward`): P = p_i + Z·v_i (the linear
/// form of Benson's method of characteristics, vol. 1, §2.8). Outflow (P ≥ p₀): the outside's
/// pressure, v = (P − p₀)/Z. Inflow: the atmosphere accelerates into the pipe isentropically,
/// p = p₀ − ½ρ₀v² (Benson §2.8.2, low Mach), and ½ρ₀v² − Z·v + (P − p₀) = 0 gives v < 0; the gas
/// entering is the reservoir's (ρ₀ expanded to p). At v = 0 both read p₀: only the density jumps
/// (a contact surface: outside air meets exhaust gas).
fn open_end(p0: f64, rho0: f64, inner: Prim, outward: f64) -> Prim {
    let z = inner.rho * inner.sound_speed();
    let v_i = inner.u * outward;
    let big_p = inner.p + z * v_i;
    if big_p >= p0 {
        return Prim {
            rho: inner.rho * (p0 / inner.p).powf(1.0 / crate::gas::GAMMA),
            u: (big_p - p0) / z * outward,
            p: p0,
        };
    }
    let disc = (z * z - 2.0 * rho0 * (big_p - p0)).max(0.0);
    let v = (z - disc.sqrt()) / rho0;
    let p = (p0 - 0.5 * rho0 * v * v).max(0.05 * p0);
    Prim {
        rho: rho0 * (p / p0).powf(1.0 / crate::gas::GAMMA),
        u: v * outward,
        p,
    }
}

/// Ghost state beyond an end, given the adjacent interior state. `outward` = +1 for the right
/// end (flow leaving when u > 0), −1 for the left end.
fn ghost(b: Boundary, inner: Prim, outward: f64) -> Prim {
    match b {
        Boundary::Wall => Prim {
            u: -inner.u,
            ..inner
        },
        Boundary::Open { p, rho } => open_end(p, rho, inner, outward),
        Boundary::State(s) => s,
        // the face flux is overridden; reconstruction sees a wall
        Boundary::Flux(_) => Prim {
            u: -inner.u,
            ..inner
        },
    }
}

impl Pipe1d {
    /// dU/dt for every cell into `self.rhs`; face fluxes into `self.fluxes`.
    fn eval_rhs(&mut self) {
        let n = self.cells.len();
        let first = self.cells[0].to_prim();
        let last = self.cells[n - 1].to_prim();
        let gl = ghost(self.left, first, -1.0);
        let gr = ghost(self.right, last, 1.0);
        let w = &mut self.w;
        w.clear();
        w.extend([gl; 3]);
        w.extend(self.cells.iter().map(|c| c.to_prim()));
        w.extend([gr; 3]);
        for (k, (q, s)) in self.qv.iter_mut().zip(w.iter()).enumerate() {
            *q = s.u * self.area[(k.max(3) - 3).min(n - 1)];
        }
        if (self.friction > 0.0 || self.wall_heat) && self.wall_coef.len() != n {
            // the areas are set once (`new`, `segmented`); D = √(4A/π)
            self.wall_coef = self
                .area
                .iter()
                .map(|a| -1.0 / (2.0 * (4.0 * a / std::f64::consts::PI).sqrt()))
                .collect();
        }
        // edges of every cell next to a face: w[2..n + 4]; a reconstruction that is not
        // physical (ρ or p ≤ 0 at a strong shock) falls back to the cell's own state.
        // Each quantity is reconstructed in its own straight loop over a contiguous array (the
        // compiler runs it on the vector unit): the same operations per cell, so the same
        // numbers as one cell at a time.
        for (k, s) in w.iter().enumerate() {
            self.rho_w[k] = s.rho;
            self.p_w[k] = s.p;
        }
        let [rl, rr, ql, qr, pl, pr] = &mut self.recon;
        let run = match self.scheme {
            Scheme::Weno5Rk3 => weno5z_run,
            Scheme::MusclRk2 => muscl_run,
        };
        run(&self.rho_w, rl, rr);
        run(&self.qv, ql, qr);
        run(&self.p_w, pl, pr);
        for i in 2..n + 4 {
            let s = w[i];
            // velocity as volume flow u·A, continuous across a compact area step (where u
            // itself jumps by the area ratio), back to u with this cell's area;
            // cell i (ghosts included) has faces i − 3 (left) and i − 2 (right)
            let face = |k: usize| self.face_area[k.clamp(3, n + 3) - 3];
            let (ul, ur) = (ql[i] / face(i), qr[i] / face(i + 1));
            let ok = |r: Prim| physical_or(r, s);
            self.edge_l[i] = ok(Prim {
                rho: rl[i],
                u: ul,
                p: pl[i],
            });
            self.edge_r[i] = ok(Prim {
                rho: rr[i],
                u: ur,
                p: pr[i],
            });
        }
        // face k sits between w[k + 2] and w[k + 3], k = 0..=n; cell j between faces j and j + 1
        for (k, f) in self.fluxes.iter_mut().enumerate() {
            *f = hllc(self.edge_r[k + 2], self.edge_l[k + 3]);
        }
        // A wall passes no mass or energy; MUSCL at a mirrored ghost is not exactly symmetric,
        // so impose it (keeps closed pipes conservative to round-off).
        if self.left == Boundary::Wall {
            self.fluxes[0].rho = 0.0;
            self.fluxes[0].e = 0.0;
        }
        if self.right == Boundary::Wall {
            self.fluxes[n].rho = 0.0;
            self.fluxes[n].e = 0.0;
        }
        if let Boundary::Flux(f) = self.left {
            self.fluxes[0] = f;
        }
        if let Boundary::Flux(f) = self.right {
            self.fluxes[n] = f;
        }
        for (j, r) in self.rhs.iter_mut().enumerate() {
            let (a, b) = (self.fluxes[j], self.fluxes[j + 1]);
            let (fa, fb) = (self.face_area[j], self.face_area[j + 1]);
            let inv = 1.0 / (self.area[j] * self.dx);
            // Geometric source ∫p dA over the cell, split at its centre: each half-cell's area
            // change times the pressure reconstructed at that edge, consistent with the edge
            // states the fluxes see (with the cell pressure instead, a 4:1 step transmitted 1.38
            // where linear theory gives 1.6). Still gas: p·(A_r − A_l), so it stays still.
            let area = self.area[j];
            let ps = self.edge_r[j + 3].p * (fb - area) + self.edge_l[j + 3].p * (area - fa);
            let cell = self.w[j + 3];
            let mut wall = if self.friction > 0.0 {
                self.friction * self.wall_coef[j] * cell.rho * cell.u * cell.u.abs()
            } else {
                0.0
            };
            if self.acoustic_loss > 0.0 {
                let c = (crate::gas::GAMMA * cell.p / cell.rho).sqrt();
                wall -= 2.0 * self.acoustic_loss * c * cell.rho * (cell.u - self.u_mean[j]);
            }
            // Reynolds' analogy: −f_D·wall_coef = f_D/(2D)
            let heat = if self.wall_heat {
                let t = cell.p / (cell.rho * crate::gas::R_AIR);
                -self.friction
                    * self.wall_coef[j]
                    * cell.rho
                    * cell.u.abs()
                    * crate::gas::CP
                    * (self.wall_t - t)
            } else {
                0.0
            };
            *r = Cons {
                rho: -(fb * b.rho - fa * a.rho) * inv,
                mom: -(fb * b.mom - fa * a.mom - ps) * inv + wall,
                e: -(fb * b.e - fa * a.e) * inv + heat,
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_open_end_draws_the_atmosphere_in_by_bernoulli() {
        // Hot gas (ρ 0.4) at rest at p₀ − 2400 Pa inside the right end: c = √(1.4·98 925/0.4) =
        // 588.42 m/s, Z = ρc = 235.37; ½·1.2·v² − 235.37·v − 2400 = 0 → v = (235.37 −
        // √61 159)/1.2 = −9.945 m/s, and the air comes in at p₀ − ½ρ₀v² = p₀ − 59.34 Pa (hand)
        let p0 = 101_325.0;
        let inner = Prim {
            rho: 0.4,
            u: 0.0,
            p: p0 - 2400.0,
        };
        let g = open_end(p0, 1.2, inner, 1.0);
        assert!((g.u + 9.945).abs() < 0.01, "{}", g.u);
        assert!((g.p - (p0 - 59.34)).abs() < 0.1, "{}", g.p);
        // the gas that comes in is the outside air's, expanded to p
        assert!(
            (g.rho - 1.2 * (g.p / p0).powf(1.0 / 1.4)).abs() < 1e-9,
            "{}",
            g.rho
        );
    }

    #[test]
    fn an_open_end_passes_outflow_at_the_outside_pressure() {
        // the same gas at p₀ moving out at 10 m/s: P = p₀ + 2400 → v = 10 m/s at p₀ (hand); at
        // P = p₀ it stands still either way: no jump through a reversal
        let p0 = 101_325.0;
        let c2 = 600.0 * 600.0;
        let gas = |p: f64, u: f64| Prim {
            rho: crate::gas::GAMMA * p / c2,
            u,
            p,
        };
        let out = open_end(p0, 1.2, gas(p0, 10.0), 1.0);
        assert!((out.u - 10.0).abs() < 1e-9 && out.p == p0, "{out:?}");
        let left = open_end(p0, 1.2, gas(p0, -10.0), -1.0);
        assert!((left.u + 10.0).abs() < 1e-9, "{left:?}");
        let still = open_end(p0, 1.2, gas(p0, 0.0), 1.0);
        assert!(
            still.u.abs() < 1e-9 && (still.p - p0).abs() < 1e-6,
            "{still:?}"
        );
    }
    use approx::assert_relative_eq;

    proptest::proptest! {
        #[test]
        fn the_vector_reconstruction_is_the_scalar_one_bit_for_bit(
            v in proptest::collection::vec(-1e6f64..1e6, 5..40)
        ) {
            // four cells at a time on the vector unit must give exactly what one cell at a
            // time gives (same IEEE operations in the same order), or the output would drift
            let (mut l, mut r) = (vec![0.0; v.len()], vec![0.0; v.len()]);
            weno5z_run(&v, &mut l, &mut r);
            for i in 2..v.len() - 2 {
                let (sl, sr) = weno5z([v[i - 2], v[i - 1], v[i], v[i + 1], v[i + 2]]);
                proptest::prop_assert_eq!(l[i].to_bits(), sl.to_bits(), "left {}", i);
                proptest::prop_assert_eq!(r[i].to_bits(), sr.to_bits(), "right {}", i);
            }
        }
    }

    /// Sod shock tube at t = 0.2 (Toro, Test 1): left (1, 0, 1), right (0.125, 0, 0.1).
    /// Exact star region: p* = 0.30313, u* = 0.92745, ρ*L = 0.42632, ρ*R = 0.26557;
    /// shock at x = 0.5 + 1.75216·0.2 = 0.85043, contact at 0.5 + 0.92745·0.2 = 0.68549.
    fn sod(n: usize) -> Pipe1d {
        let mut p = Pipe1d::new(
            1.0,
            n,
            Prim {
                rho: 1.0,
                u: 0.0,
                p: 1.0,
            },
        );
        for i in n / 2..n {
            p.cells[i] = Prim {
                rho: 0.125,
                u: 0.0,
                p: 0.1,
            }
            .to_cons();
        }
        p.left = Boundary::Wall;
        p.right = Boundary::Wall;
        let mut t = 0.0;
        while t < 0.2 {
            let dt = p.max_dt(0.8).min(0.2 - t);
            p.step(dt);
            t += dt;
        }
        p
    }

    fn at(p: &Pipe1d, x: f64) -> Prim {
        p.prim(((x / p.dx) as usize).min(p.cells.len() - 1))
    }

    #[test]
    fn primitive_conserved_round_trip() {
        let s = Prim {
            rho: 1.2,
            u: 30.0,
            p: 101_325.0,
        };
        let b = s.to_cons().to_prim();
        assert_relative_eq!(b.rho, 1.2, max_relative = 1e-12);
        assert_relative_eq!(b.u, 30.0, max_relative = 1e-12);
        assert_relative_eq!(b.p, 101_325.0, max_relative = 1e-12);
        // c = √(γp/ρ) = √(1.4·101325/1.2) = 343.82
        assert_relative_eq!(s.sound_speed(), 343.82, max_relative = 1e-4);
    }

    #[test]
    fn hllc_is_consistent_for_equal_states() {
        // F(U, U) = physical flux: (ρu, ρu² + p, u(E + p))
        let s = Prim {
            rho: 1.0,
            u: 2.0,
            p: 3.0,
        };
        let f = hllc(s, s);
        let e = 3.0 / 0.4 + 0.5 * 4.0;
        assert_relative_eq!(f.rho, 2.0, max_relative = 1e-12);
        assert_relative_eq!(f.mom, 4.0 + 3.0, max_relative = 1e-12);
        assert_relative_eq!(f.e, 2.0 * (e + 3.0), max_relative = 1e-12);
    }

    #[test]
    fn sod_shock_tube_matches_the_exact_solution() {
        let p = sod(400);
        let star_l = at(&p, 0.6); // between rarefaction tail and contact
        let star_r = at(&p, 0.77); // between contact and shock
        assert_relative_eq!(star_l.p, 0.30313, max_relative = 0.02);
        assert_relative_eq!(star_l.u, 0.92745, max_relative = 0.02);
        assert_relative_eq!(star_l.rho, 0.42632, max_relative = 0.02);
        assert_relative_eq!(star_r.rho, 0.26557, max_relative = 0.03);
        // ahead of the shock: undisturbed; behind it: star state
        assert_relative_eq!(at(&p, 0.87).rho, 0.125, max_relative = 0.02);
        assert_relative_eq!(at(&p, 0.83).rho, 0.26557, max_relative = 0.05);
        // left state untouched beyond the rarefaction head (x = 0.5 − 1.18322·0.2 = 0.2634)
        assert_relative_eq!(at(&p, 0.2).rho, 1.0, max_relative = 1e-3);
    }

    #[test]
    fn prescribed_flux_adds_exactly_that_mass_and_energy() {
        let air = Prim {
            rho: 1.2,
            u: 0.0,
            p: 101_325.0,
        };
        let mut p = Pipe1d::new(0.5, 50, air);
        let f = Cons {
            rho: 2.0,
            mom: 101_325.0 + 2.0 * 2.0 / 1.2,
            e: 2.0 * 3.0e5,
        };
        p.left = Boundary::Flux(f);
        let m0: f64 = p.cells.iter().map(|c| c.rho).sum::<f64>() * p.dx;
        let e0: f64 = p.cells.iter().map(|c| c.e).sum::<f64>() * p.dx;
        let (dt, steps) = (1e-6, 100);
        for _ in 0..steps {
            p.step(dt);
        }
        let m1: f64 = p.cells.iter().map(|c| c.rho).sum::<f64>() * p.dx;
        let e1: f64 = p.cells.iter().map(|c| c.e).sum::<f64>() * p.dx;
        // 2 kg/(s·m²) for 100 µs → 2e-4 kg/m²; energy 6e5·1e-4 = 60 J/m² (right end is a wall)
        assert_relative_eq!(m1 - m0, 2.0e-4, max_relative = 1e-9);
        assert_relative_eq!(e1 - e0, 60.0, max_relative = 1e-9);
        assert_eq!(p.end_flux.0, f);
    }

    #[test]
    fn violent_extraction_keeps_cells_physical() {
        let air = Prim {
            rho: 1.2,
            u: 0.0,
            p: 101_325.0,
        };
        let mut p = Pipe1d::new(0.3, 10, air);
        p.left = Boundary::Flux(Cons {
            rho: -500.0,
            mom: 101_325.0,
            e: -500.0 * 3.0e5,
        });
        for _ in 0..200 {
            p.step(2e-6);
        }
        for c in &p.cells {
            let q = c.to_prim();
            assert!(q.rho > 0.0 && q.p > 0.0 && q.u.is_finite(), "{q:?}");
        }
    }

    fn stepped(a1: f64, a2: f64) -> Pipe1d {
        let air = Prim {
            rho: 1.2,
            u: 0.0,
            p: 101_325.0,
        };
        Pipe1d::segmented(&[(1.0, a1), (1.0, a2)], 0.01, air)
    }

    #[test]
    fn still_gas_stays_still_across_an_area_step() {
        let mut p = stepped(1e-3, 4e-3);
        for _ in 0..2_000 {
            p.step(1e-5);
        }
        let worst = (0..p.cells.len())
            .map(|i| p.prim(i).u.abs())
            .fold(0.0, f64::max);
        assert!(worst < 1e-9, "spurious velocity {worst}");
        assert_eq!(p.end_areas(), (1e-3, 4e-3));
    }

    #[test]
    fn variable_area_pipe_conserves_mass() {
        let mut p = stepped(1e-3, 4e-3);
        for i in 10..20 {
            p.cells[i] = Prim {
                rho: 1.5,
                u: 0.0,
                p: 140_000.0,
            }
            .to_cons();
        }
        let mass = |p: &Pipe1d| {
            p.cells
                .iter()
                .zip(&p.area)
                .map(|(c, a)| c.rho * a)
                .sum::<f64>()
                * p.dx
        };
        let m0 = mass(&p);
        for _ in 0..3_000 {
            p.step(1e-5);
        }
        assert_relative_eq!(mass(&p), m0, max_relative = 1e-12);
    }

    /// Peak of a small right-going pulse after it has crossed into the second segment.
    fn transmitted_peak(a1: f64, a2: f64) -> f64 {
        let mut p = stepped(a1, a2);
        let base = 101_325.0;
        for i in 20..30 {
            let s = (std::f64::consts::PI * (i - 20) as f64 / 10.0).sin();
            p.cells[i] = Prim {
                rho: 1.2 * (1.0 + 0.001 * s / 1.4),
                u: 0.001 * s * 343.8 / 1.4,
                p: base * (1.0 + 0.001 * s),
            }
            .to_cons();
        }
        // pulse centre: 0.25 m + 343.8 m/s · 3.2 ms ≈ 1.35 m, fully past the step at 1 m
        let mut t = 0.0;
        while t < 3.2e-3 {
            p.step(1e-5);
            t += 1e-5;
        }
        (100..200).map(|i| p.prim(i).p - base).fold(0.0, f64::max)
    }

    #[test]
    fn area_step_transmits_two_a1_over_a1_plus_a2() {
        // Linear theory for a compact area step: pressure transmission 2A₁/(A₁+A₂) = 0.4 for
        // A₂ = 4A₁. Normalised by the same pipe without a step (removes numerical damping).
        // Reconstructing velocity as volume flow at the face area converges to it (0.400 and
        // 1.597 at 2.5 mm cells); reconstructing u itself converged to 0.37 and 1.47.
        let ratio = transmitted_peak(1e-3, 4e-3) / transmitted_peak(1e-3, 1e-3);
        assert!((ratio - 0.4).abs() < 0.03, "transmitted {ratio:.3}");
        // and into a narrower pipe (A₂ = A₁/4): 2·4/(4+1) = 1.6
        let up = transmitted_peak(4e-3, 1e-3) / transmitted_peak(4e-3, 4e-3);
        assert!((up - 1.6).abs() < 0.08, "transmitted {up:.3}");
    }

    #[test]
    fn wall_friction_gives_the_darcy_pressure_drop() {
        // Steady 30 m/s air through 2 m of 40 mm pipe with Darcy f = 0.02:
        // Δp = f·(L/D)·½ρu² = 0.02·50·0.5·1.2·900 = 540 Pa (hand calc).
        let air = Prim {
            rho: 1.2,
            u: 30.0,
            p: 101_325.0,
        };
        let a = std::f64::consts::PI * 0.25 * 0.04 * 0.04;
        let mut p = Pipe1d::segmented(&[(2.0, a)], 0.02, air);
        p.friction = 0.02;
        let m = 1.2 * 30.0;
        p.right = Boundary::Open {
            p: 101_325.0,
            rho: 1.2,
        };
        for _ in 0..40_000 {
            let c = p.prim(0);
            p.left = Boundary::Flux(Cons {
                rho: m,
                mom: m * m / c.rho + c.p,
                e: m * (1004.5 * 293.0 + 450.0),
            });
            p.step(1e-5);
        }
        let n = p.cells.len();
        let dp = p.prim(2).p - p.prim(n - 3).p;
        let span = (n - 5) as f64 / n as f64;
        assert!((dp / (540.0 * span) - 1.0).abs() < 0.15, "Δp {dp:.0} Pa");
    }

    /// 0.2 m of 40 mm pipe full of gas at 882.6 K (ρ 0.4, 1 atm) flowing at 50 m/s, the same
    /// state at both ends so the fluxes cancel: only the wall acts.
    fn hot_flow(wall_t: f64) -> Pipe1d {
        let gas = Prim {
            rho: 0.4,
            u: 50.0,
            p: 101_325.0,
        };
        let a = std::f64::consts::PI * 0.25 * 0.04 * 0.04;
        let mut p = Pipe1d::segmented(&[(0.2, a)], 0.02, gas);
        p.friction = 0.025;
        p.wall_heat = true;
        p.wall_t = wall_t;
        p.left = Boundary::State(gas);
        p.right = Boundary::State(gas);
        p
    }

    #[test]
    fn the_wall_takes_heat_by_reynolds_analogy() {
        // Benson eq. 7.92 per volume: (f_D/2D)·ρ|u|c_p·(T_w − T) = (0.025/0.08)·0.4·50·1004.5·
        // (500 − 882.62) = −2.401 9e6 W/m³: over 1 µs each cell loses 2.4019 J/m³ (hand)
        let mut p = hot_flow(500.0);
        let before = p.cells[5].e;
        p.step(1e-6);
        let t_gas = 101_325.0 / (0.4 * crate::gas::R_AIR);
        let want = 1e-6 * 0.025 / 0.08 * 0.4 * 50.0 * crate::gas::CP * (500.0 - t_gas);
        approx::assert_relative_eq!(p.cells[5].e - before, want, max_relative = 1e-3);
        approx::assert_relative_eq!(want, -2.4019, max_relative = 1e-3);
        // still gas exchanges nothing by the analogy (h ∝ |u|)
        let mut still = hot_flow(500.0);
        for c in still.cells.iter_mut() {
            *c = Prim {
                rho: 0.4,
                u: 0.0,
                p: 101_325.0,
            }
            .to_cons();
        }
        still.left = Boundary::Wall;
        still.right = Boundary::Wall;
        let e0 = still.cells[5].e;
        still.step(1e-6);
        assert_eq!(still.cells[5].e, e0);
    }

    #[test]
    fn the_wall_warms_by_what_it_takes_less_what_it_sheds() {
        // the same pipe, wall at 500 K: in 603.7 W (2.4019e6 W/m³ over 2.5133e-4 m³); out
        // (10 + 0.79·σ·(500² + 300²)·(500 + 300))·π·0.043·0.2·(500 − 300) = 22.18·0.027018·200
        // = 119.86 W; thermal mass 7832·559·π·0.04·0.0015·0.2 = 164.98 J/K (hand). Over 1 ms:
        // ΔT_w = 1e-3·(603.7 − 119.86)/164.98 = 2.933e-3 K, in 100 stable steps of 10 µs (the
        // gas cools ≈ 2 % meanwhile, so it gives a little less)
        let mut p = hot_flow(500.0);
        for _ in 0..100 {
            p.step(1e-5);
        }
        approx::assert_relative_eq!(p.wall_t - 500.0, 2.933e-3, max_relative = 0.03);
    }

    #[test]
    fn the_acoustic_wall_loss_wears_a_wave_by_its_rate_per_metre() {
        // A small Gaussian pressure pulse in still air splits into two halves that each travel
        // c·t; with an amplitude loss α (Np/m) each keeps e^(−α·d) of the inviscid peak. α =
        // −ln 0.97 = 0.030459 over d = 343·0.006 = 2.06 m: 0.93922 (hand)
        let alpha = -(0.97f64).ln();
        let peak = |loss: f64| {
            let air = Prim {
                rho: 1.2,
                u: 0.0,
                p: 101_325.0,
            };
            let mut p = Pipe1d::new(6.0, 600, air);
            p.acoustic_loss = loss;
            for (i, c) in p.cells.iter_mut().enumerate() {
                let x = (i as f64 + 0.5) * 0.01 - 3.0;
                let dp = 100.0 * (-(x / 0.05).powi(2)).exp();
                *c = Prim {
                    rho: 1.2 * (1.0 + dp / (1.4 * 101_325.0)),
                    u: 0.0,
                    p: 101_325.0 + dp,
                }
                .to_cons();
            }
            let mut t = 0.0;
            while t < 0.006 {
                let dt = p.max_dt(0.5).min(0.006 - t);
                p.step(dt);
                t += dt;
            }
            // the right-running half
            (300..600)
                .map(|i| p.prim(i).p - 101_325.0)
                .fold(f64::MIN, f64::max)
        };
        let ratio = peak(alpha) / peak(0.0);
        assert!((ratio - 0.939_22).abs() < 0.006, "{ratio}");
    }

    #[test]
    fn the_acoustic_loss_follows_the_mean_flow_and_leaves_it_alone() {
        // The loss acts on u − ū, ū an exponential mean of time constant τ = 0.2 s. Still air
        // (ū = 0) suddenly moving at 20 m/s, at the pipe's middle where the ends' waves have
        // not arrived (3 m at c + u): du/dt = −a(u − ū), dū/dt = (u − ū)/τ with a = 2αc =
        // 2·0.03·343.8 = 20.63 /s, so u + aτ·ū stays 20 and u − ū = 20·e^(−(a + 1/τ)t).
        // After 6 ms: u − ū = 17.15, ū = (20 − 17.15)/(1 + 4.126) = 0.556 m/s (hand).
        let air = Prim {
            rho: 1.2,
            u: 0.0,
            p: 101_325.0,
        };
        let moving = Prim { u: 20.0, ..air }.to_cons();
        let run = |mean: Option<f64>| {
            let mut p = Pipe1d::new(6.0, 600, air);
            p.acoustic_loss = 0.03;
            p.step(1e-7);
            if let Some(m) = mean {
                p.u_mean.iter_mut().for_each(|u| *u = m);
            }
            p.cells.iter_mut().for_each(|c| *c = moving);
            let mut t = 0.0;
            while t < 0.006 {
                let dt = p.max_dt(0.5).min(0.006 - t);
                p.step(dt);
                t += dt;
            }
            (p.u_mean[300], p.prim(300).u)
        };
        let (u_mean, _) = run(None);
        assert!((u_mean - 0.556).abs() < 0.005, "{u_mean}");
        // a flow already at its mean is not braked: no loss on a steady stream
        let (_, u) = run(Some(20.0));
        assert!((u - 20.0).abs() < 0.01, "{u}");
        // and a pipe that starts out flowing takes that flow as its mean
        let mut p = Pipe1d::new(6.0, 600, Prim { u: 20.0, ..air });
        p.acoustic_loss = 0.03;
        let mut t = 0.0;
        while t < 0.006 {
            let dt = p.max_dt(0.5).min(0.006 - t);
            p.step(dt);
            t += dt;
        }
        assert!((p.prim(300).u - 20.0).abs() < 0.01, "{}", p.prim(300).u);
    }

    #[test]
    fn wall_friction_damps_a_ringing_pipe() {
        let ring = |f: f64| {
            let air = Prim {
                rho: 1.2,
                u: 0.0,
                p: 101_325.0,
            };
            let mut p = Pipe1d::segmented(&[(1.0, 1.26e-3)], 0.02, air);
            p.friction = f;
            for i in 0..10 {
                p.cells[i] = Prim {
                    p: 111_325.0,
                    rho: 1.29,
                    ..air
                }
                .to_cons();
            }
            for _ in 0..30_000 {
                p.step(1e-5);
            }
            // acoustic energy ∝ Σp'² about the mean (the pulse's mass raises the static pressure
            // of the closed pipe; a steepened pulse makes the maximum meaningless)
            let n = p.cells.len();
            let mean = (0..n).map(|i| p.prim(i).p).sum::<f64>() / n as f64;
            (0..n).map(|i| (p.prim(i).p - mean).powi(2)).sum::<f64>()
        };
        // Clear damping only (measured 0.69 on 2026-10-22); the magnitude is checked against a hand
        // calculation in `wall_friction_gives_the_darcy_pressure_drop`.
        assert!(
            ring(0.03) < 0.85 * ring(0.0),
            "friction {} vs inviscid {}",
            ring(0.03),
            ring(0.0)
        );
    }

    #[test]
    fn flux_boundary_caps_the_crossing_speed_at_sonic() {
        let cell = Prim {
            rho: 1.2,
            u: 0.0,
            p: 101_325.0,
        };
        // ṁ″ = 36 → u = 30 m/s: momentum 36·30 + p
        let f = flux_boundary(36.0, 1.0, cell);
        assert_eq!(f.rho, 36.0);
        assert!((f.mom - (36.0 * 30.0 + 101_325.0)).abs() < 1e-9);
        // near-vacuum cell: u would be 36/0.001 = 36 km/s; capped at c
        let thin = Prim {
            rho: 1e-3,
            u: 0.0,
            p: 500.0,
        };
        let f = flux_boundary(36.0, 1.0, thin);
        assert!((f.mom - (36.0 * thin.sound_speed() + 500.0)).abs() < 1e-6);
    }

    #[test]
    fn closed_tube_conserves_mass_and_energy() {
        let p = sod(200);
        let mass: f64 = p.cells.iter().map(|c| c.rho).sum::<f64>() * p.dx;
        let energy: f64 = p.cells.iter().map(|c| c.e).sum::<f64>() * p.dx;
        // initial: 0.5·1 + 0.5·0.125 = 0.5625; E: 0.5·2.5 + 0.5·0.25 = 1.375
        assert_relative_eq!(mass, 0.5625, max_relative = 1e-12);
        assert_relative_eq!(energy, 1.375, max_relative = 1e-12);
    }

    #[test]
    fn acoustic_pulse_travels_at_the_speed_of_sound() {
        // 1 m pipe of air, small pressure bump at x = 0.2; open far end. After 1 ms the
        // right-going half sits at 0.2 + 343.8·0.001 = 0.544 m.
        let air = Prim {
            rho: 1.2,
            u: 0.0,
            p: 101_325.0,
        };
        let mut p = Pipe1d::new(1.0, 500, air);
        for i in 95..105 {
            p.cells[i] = Prim {
                p: 101_325.0 + 500.0,
                ..air
            }
            .to_cons();
        }
        p.right = Boundary::Open {
            p: 101_325.0,
            rho: 1.2,
        };
        let mut t = 0.0;
        while t < 1e-3 {
            let dt = p.max_dt(0.8).min(1e-3 - t);
            p.step(dt);
            t += dt;
        }
        let peak = (260..400)
            .max_by(|a, b| p.prim(*a).p.total_cmp(&p.prim(*b).p))
            .unwrap();
        let x = (peak as f64 + 0.5) * p.dx;
        assert!((x - 0.544).abs() < 0.01, "peak at {x}");
    }

    #[test]
    fn a_smooth_pulse_keeps_its_peak() {
        // Gaussian bump (σ = 10 mm, 500 Pa) at rest splits into two 250 Pa halves (d'Alembert).
        // WENO5-Z + RK3 carries the right-going half 0.34 m keeping 98 % of its peak at 5 cells
        // per σ (MUSCL–van Leer kept 78 %, first-order upwinding 36 %; measured).
        let air = Prim {
            rho: 1.2,
            u: 0.0,
            p: 101_325.0,
        };
        let mut p = Pipe1d::new(1.0, 500, air);
        for i in 0..500 {
            let x = (i as f64 + 0.5) * p.dx - 0.2;
            p.cells[i] = Prim {
                p: 101_325.0 + 500.0 * (-0.5 * (x / 0.01).powi(2)).exp(),
                ..air
            }
            .to_cons();
        }
        let mut t = 0.0;
        while t < 1e-3 {
            let dt = p.max_dt(0.8).min(1e-3 - t);
            p.step(dt);
            t += dt;
        }
        let peak = (230..330)
            .map(|i| p.prim(i).p - 101_325.0)
            .fold(0.0, f64::max);
        assert!(
            (0.95 * 250.0..1.02 * 250.0).contains(&peak),
            "peak {peak} Pa"
        );
    }

    #[test]
    fn weno5z_mirrors_its_stencil_and_is_exact_for_quadratics() {
        // cell averages of x on unit cells centred at −2..2: edges ∓0.5, whatever the weights;
        // of x², k² + 1/12: both edges 0.25 (every candidate stencil is exact for a quadratic)
        let (l, r) = weno5z([-2.0, -1.0, 0.0, 1.0, 2.0]);
        assert_relative_eq!(l, -0.5, epsilon = 1e-12);
        assert_relative_eq!(r, 0.5, epsilon = 1e-12);
        let q = |k: f64| k * k + 1.0 / 12.0;
        let (l, r) = weno5z([q(-2.0), q(-1.0), q(0.0), q(1.0), q(2.0)]);
        assert_relative_eq!(l, 0.25, epsilon = 1e-12);
        assert_relative_eq!(r, 0.25, epsilon = 1e-12);
        // the left edge is the right edge of the mirrored stencil, for any data
        for v in [
            [0.3, -1.2, 2.5, 0.7, 4.1],
            [1.0, 1.0, 1.0, 0.0, 0.0],
            [5.0, 2.0, -3.0, 8.0, 1.5],
        ] {
            let (l, r) = weno5z(v);
            let (ml, mr) = weno5z([v[4], v[3], v[2], v[1], v[0]]);
            assert_relative_eq!(l, mr, max_relative = 1e-14);
            assert_relative_eq!(r, ml, max_relative = 1e-14);
        }
    }

    #[test]
    fn weno5z_keeps_fifth_order_through_the_crest_of_a_wave() {
        // edges of the exact cell averages of sin(2πx) on [0, 1], n cells, against sin at the
        // faces. WENO5-Z keeps its design order at the extrema (Borges et al. 2008, Table 2:
        // L∞ order ≈ 5); the classic weights drop to ≈ 3 there.
        let err = |n: usize| {
            let h = 1.0 / n as f64;
            let w = 2.0 * std::f64::consts::PI;
            let avg = |k: i64| {
                let (a, b) = (k as f64 * h, (k + 1) as f64 * h);
                ((w * a).cos() - (w * b).cos()) / (w * h)
            };
            (0..n as i64)
                .map(|k| {
                    let (l, r) = weno5z([avg(k - 2), avg(k - 1), avg(k), avg(k + 1), avg(k + 2)]);
                    let el = (l - (w * k as f64 * h).sin()).abs();
                    let er = (r - (w * (k + 1) as f64 * h).sin()).abs();
                    el.max(er)
                })
                .fold(0.0, f64::max)
        };
        let order = (err(20) / err(40)).log2();
        assert!(order > 4.5, "L∞ order {order}");
    }

    #[test]
    fn weno5z_leans_on_the_smooth_side_of_a_jump() {
        // a ramp 0, 1, 2 then a jump to 10, 11. By hand: β₀ = 1, β₁ = 73.333, β₂ = 185.333,
        // τ = |β₀ − β₂| = 184.333, z = 1 + (τ/β)² = 33979.8, 7.3184, 1.9892; the right edge
        // weighs the candidates 2.5, 4.8333, 7.1667 by 0.1·z₀ : 0.6·z₁ : 0.3·z₂ → 2.50383, the
        // left edge the mirrored 1.5, … by 0.1·z₂ : 0.6·z₁ : 0.3·z₀ → 1.49938: both close to the
        // ramp's own extrapolation (2.5 and 1.5), nowhere near the jump.
        let (l, r) = weno5z([0.0, 1.0, 2.0, 10.0, 11.0]);
        assert_relative_eq!(l, 1.499_383_906, max_relative = 1e-8);
        assert_relative_eq!(r, 2.503_829_204, max_relative = 1e-8);
    }

    #[test]
    fn an_acoustic_pulse_converges_at_high_order() {
        // a right-running 100 Pa Gaussian (σ = 10 mm; u = p′/ρc, ρ′ = p′/c²) on 125, 250 and
        // 500 cells, 1 ms: velocity and pressure ride the reconstruction here, where the entropy
        // wave keeps them uniform. Self-convergence: each solution against the next finer one,
        // averaged onto its cells. At CFL 0.2 the time error of SSP-RK3 is 4³ smaller than at
        // 0.8 (where it sets the rate, 2.8), so the spatial order shows: measured 3.55, and 2.8
        // with one wrong cell in the pressure stencil.
        let (rho0, p0): (f64, f64) = (1.2, 101_325.0);
        let c = (1.4 * p0 / rho0).sqrt();
        let run = |n: usize| {
            let mut p = Pipe1d::new(
                0.5,
                n,
                Prim {
                    rho: rho0,
                    u: 0.0,
                    p: p0,
                },
            );
            for i in 0..n {
                let x = (i as f64 + 0.5) * p.dx - 0.1;
                let dp = 100.0 * (-0.5 * (x / 0.01).powi(2)).exp();
                p.cells[i] = Prim {
                    rho: rho0 + dp / (c * c),
                    u: dp / (rho0 * c),
                    p: p0 + dp,
                }
                .to_cons();
            }
            let open = Boundary::Open { p: p0, rho: rho0 };
            (p.left, p.right) = (open, open);
            let mut t = 0.0;
            while t < 1e-3 {
                let dt = p.max_dt(0.2).min(1e-3 - t);
                p.step(dt);
                t += dt;
            }
            (0..n)
                .map(|i| (p.prim(i).u, p.prim(i).p))
                .collect::<Vec<_>>()
        };
        let diff = |coarse: &[(f64, f64)], fine: &[(f64, f64)]| {
            coarse
                .iter()
                .enumerate()
                .fold((0.0, 0.0), |(du, dp), (i, &(u, p))| {
                    let (u2, p2) = (
                        0.5 * (fine[2 * i].0 + fine[2 * i + 1].0),
                        0.5 * (fine[2 * i].1 + fine[2 * i + 1].1),
                    );
                    (du + (u - u2).abs(), dp + (p - p2).abs())
                })
        };
        let (s1, s2, s3) = (run(125), run(250), run(500));
        let (d12, d23) = (diff(&s1, &s2), diff(&s2, &s3));
        // each sum is over twice the cells of the coarser grid: per-length, that factor of 2
        let (ou, op) = ((2.0 * d12.0 / d23.0).log2(), (2.0 * d12.1 / d23.1).log2());
        assert!(ou > 3.2 && op > 3.2, "orders u {ou} p {op}");
    }

    #[test]
    fn an_unphysical_edge_falls_back_to_its_cell() {
        let cell = Prim {
            rho: 1.0,
            u: 3.0,
            p: 1e5,
        };
        let edge = |rho: f64, p: f64| Prim { rho, u: 9.0, p };
        assert_eq!(physical_or(edge(1.1, 2e5), cell).u, 9.0);
        // a positive density does not excuse a negative pressure, nor the other way round
        assert_eq!(physical_or(edge(1.1, -5.0), cell).u, 3.0);
        assert_eq!(physical_or(edge(-0.1, 2e5), cell).u, 3.0);
        // vacuum is not a state the Riemann solver can take either
        assert_eq!(physical_or(edge(0.0, 2e5), cell).u, 3.0);
        assert_eq!(physical_or(edge(1.1, 0.0), cell).u, 3.0);
    }

    #[test]
    fn a_near_vacuum_beside_a_strong_jump_stays_physical() {
        // 1 bar against 10 Pa at the same temperature: the reconstructed edge pressure of the
        // low side overshoots below zero; such an edge falls back to the cell's own state
        let mut p = Pipe1d::new(
            0.2,
            40,
            Prim {
                rho: 1.2,
                u: 0.0,
                p: 1e5,
            },
        );
        for i in 20..40 {
            p.cells[i] = Prim {
                rho: 1.2e-4,
                u: 0.0,
                p: 10.0,
            }
            .to_cons();
        }
        for _ in 0..20 {
            let dt = p.max_dt(0.5);
            p.step(dt);
        }
        for i in 0..40 {
            let s = p.prim(i);
            assert!(
                s.rho > 0.0 && s.p > 0.0 && s.u.is_finite(),
                "cell {i}: {s:?}"
            );
        }
    }

    /// L1 error of ρ for an entropy wave (a Gaussian density bump drifting 50 mm at 50 m/s)
    /// on `n` cells with `scheme`, against the exact drifted bump.
    fn bump_l1(scheme: Scheme, n: usize) -> f64 {
        let mut p = Pipe1d::new(
            0.5,
            n,
            Prim {
                rho: 1.2,
                u: 50.0,
                p: 101_325.0,
            },
        );
        p.scheme = scheme;
        let bump = |x: f64| 1.2 + 0.3 * (-0.5 * (x / 0.01).powi(2)).exp();
        for i in 0..n {
            let x = (i as f64 + 0.5) * p.dx - 0.1;
            p.cells[i] = Prim {
                rho: bump(x),
                u: 50.0,
                p: 101_325.0,
            }
            .to_cons();
        }
        // the uniform flow comes in as it is; the open end lets it out (at P = p₀ + Z·50 it
        // leaves at 50 m/s and p₀, the open end's outflow)
        p.left = Boundary::State(Prim {
            rho: 1.2,
            u: 50.0,
            p: 101_325.0,
        });
        p.right = Boundary::Open {
            p: 101_325.0,
            rho: 1.2,
        };
        let mut t = 0.0;
        while t < 1e-3 {
            let dt = p.max_dt(0.8).min(1e-3 - t);
            p.step(dt);
            t += dt;
        }
        (0..n)
            .map(|i| (p.prim(i).rho - bump((i as f64 + 0.5) * p.dx - 0.15)).abs() * p.dx)
            .sum::<f64>()
    }

    #[test]
    fn an_advected_density_bump_converges_at_high_order() {
        // on 125, 250 and 500 cells. WENO5 in space with SSP-RK3 at a fixed CFL: the spatial
        // error dominates at these resolutions, so halving the cell must cut it by close to 2⁵
        // (measured 4.9 and 5.0)
        let l1 = |n| bump_l1(Scheme::Weno5Rk3, n);
        let (e1, e2, e3) = (l1(125), l1(250), l1(500));
        assert!((e1 / e2).log2() > 4.5, "{e1:e} {e2:e} {e3:e}");
        assert!((e2 / e3).log2() > 4.5, "{e1:e} {e2:e} {e3:e}");
    }

    #[test]
    fn the_cheap_scheme_converges_at_second_order() {
        // MUSCL (van Leer) with SSP-RK2 is second order away from extrema; its limiter clips
        // the bump's crest, so the observed order sits a little under 2 (Toro, *Riemann
        // Solvers*, §13.8): measured 1.68 and 1.75, and well behind WENO5 on every grid
        let l1 = |n| bump_l1(Scheme::MusclRk2, n);
        let (e1, e2, e3) = (l1(125), l1(250), l1(500));
        for (a, b) in [(e1, e2), (e2, e3)] {
            let order = (a / b).log2();
            assert!((1.6..2.2).contains(&order), "{e1:e} {e2:e} {e3:e}");
        }
        assert!(e3 > 10.0 * bump_l1(Scheme::Weno5Rk3, 500));
    }

    #[test]
    fn an_advected_density_bump_stays_bounded_and_keeps_its_peak() {
        // Entropy wave: uniform p and u = 50 m/s, Gaussian density bump (σ = 10 mm, +0.3 kg/m³).
        // It just drifts 50 mm in 1 ms; TVD reconstruction keeps it within its initial bounds
        // (no new extrema) and keeps most of its peak.
        let mut p = Pipe1d::new(
            0.5,
            250,
            Prim {
                rho: 1.2,
                u: 50.0,
                p: 101_325.0,
            },
        );
        for i in 0..250 {
            let x = (i as f64 + 0.5) * p.dx - 0.1;
            p.cells[i] = Prim {
                rho: 1.2 + 0.3 * (-0.5 * (x / 0.01).powi(2)).exp(),
                u: 50.0,
                p: 101_325.0,
            }
            .to_cons();
        }
        p.left = Boundary::State(Prim {
            rho: 1.2,
            u: 50.0,
            p: 101_325.0,
        });
        p.right = Boundary::Open {
            p: 101_325.0,
            rho: 1.2,
        };
        let mut t = 0.0;
        while t < 1e-3 {
            let dt = p.max_dt(0.8).min(1e-3 - t);
            p.step(dt);
            t += dt;
        }
        let rho: Vec<f64> = (20..120).map(|i| p.prim(i).rho).collect();
        let (lo, hi) = rho
            .iter()
            .fold((f64::MAX, f64::MIN), |(a, b), &r| (a.min(r), b.max(r)));
        assert!(lo > 1.2 - 0.01 && hi < 1.5 + 1e-6, "range {lo}..{hi}");
        assert!(hi - 1.2 > 0.7 * 0.3, "peak {hi}");
    }

    #[test]
    fn hllc_follows_the_sonic_point_of_a_transonic_rarefaction() {
        // A rarefaction straddling the interface: the exact Godunov flux is that of the sonic
        // state inside the fan (Toro, Riemann Solvers, eqs. 4.56 and 4.63), not the upwind state.
        // Left fan, l = (1, 100, 1e5), r = (1, 900, 1e5): c_l = 374.17 m/s,
        //   u* = 2/(γ+1)·(c_l + (γ−1)/2·u_l) = 328.47, ρ* = (2/(γ+1) + (γ−1)/((γ+1)c_l)·u_l)^5
        //   = 0.5216 → ρ*u* ≈ 171.3 kg/(m²·s). HLLC with Davis speeds overshoots sonic points
        //   (≈ 242 here, measured); the upwind flux (100) would mean the fan was never seen.
        let state = |u: f64| Prim {
            rho: 1.0,
            u,
            p: 1.0e5,
        };
        let f = hllc(state(100.0), state(900.0));
        assert!(
            (f.rho / 171.3 - 1.0).abs() < 0.5 && f.rho > 130.0,
            "left fan: {}",
            f.rho
        );
        // Mirror image (right fan, flow to the left): ρ*u* ≈ −171.3.
        let f = hllc(state(-900.0), state(-100.0));
        assert!(
            (f.rho / -171.3 - 1.0).abs() < 0.5 && f.rho < -130.0,
            "right fan: {}",
            f.rho
        );
    }

    #[test]
    fn hllc_is_symmetric_for_a_head_on_collision() {
        // Mirror-image states colliding: by symmetry nothing crosses the interface (no mass, no
        // energy), only the star pressure pushes.
        let l = Prim {
            rho: 1.0,
            u: 100.0,
            p: 1.0e5,
        };
        let f = hllc(l, Prim { u: -100.0, ..l });
        assert!(f.rho.abs() < 1e-9, "mass {}", f.rho);
        assert!(f.e.abs() < 1e-6, "energy {}", f.e);
        assert!(f.mom > 1.0e5, "star pressure {}", f.mom);
    }

    #[test]
    fn hllc_resolves_a_moving_contact_exactly() {
        // Same p and u, different density: the exact flux is the upwind (left) state's flux,
        // and HLLC reproduces isolated contacts exactly (Toro §10.4).
        let l = Prim {
            rho: 1.0,
            u: 10.0,
            p: 1.0e5,
        };
        let r = Prim { rho: 0.5, ..l };
        let f = hllc(l, r);
        let e = 1.0e5 / 0.4 + 0.5 * 1.0 * 100.0;
        assert_relative_eq!(f.rho, 10.0, max_relative = 1e-9);
        assert_relative_eq!(f.mom, 100.0 + 1.0e5, max_relative = 1e-9);
        assert_relative_eq!(f.e, 10.0 * (e + 1.0e5), max_relative = 1e-9);
    }

    #[test]
    fn positive_resets_broken_cells_and_floors_the_energy() {
        let (rf, pf) = (1e-3, 500.0);
        let rest = Prim {
            rho: rf,
            u: 0.0,
            p: pf,
        }
        .to_cons();
        let good = Prim {
            rho: 2.0,
            u: 100.0,
            p: 1.0e5,
        }
        .to_cons();
        for bad in [
            Cons {
                rho: f64::NAN,
                ..good
            },
            Cons {
                mom: f64::NAN,
                ..good
            },
            Cons {
                e: f64::INFINITY,
                ..good
            },
            Cons {
                rho: 0.5e-3,
                ..good
            },
        ] {
            assert_eq!(positive(bad, rf, pf), rest, "{bad:?}");
        }
        assert_eq!(positive(good, rf, pf), good);
        // exactly at the density floor the cell keeps its momentum
        let thin = Cons {
            rho: rf,
            mom: 0.05,
            e: 10.0,
        };
        assert_eq!(positive(thin, rf, pf).mom, 0.05);
        // too little energy: raised to kinetic (½ρu² = 10 000 J/m³) + p_floor/(γ−1) = 1250 J/m³
        let cold = Cons { e: 1.0, ..good };
        assert_relative_eq!(
            positive(cold, rf, pf).e,
            10_000.0 + 1250.0,
            max_relative = 1e-12
        );
        assert_eq!(positive(cold, rf, pf).mom, good.mom);
    }

    #[test]
    fn ghosts_mirror_walls_and_open_ends_follow_the_characteristic() {
        let inner = Prim {
            rho: 0.8,
            u: 30.0,
            p: 1.1e5,
        };
        assert_eq!(ghost(Boundary::Wall, inner, 1.0).u, -30.0);
        let f = Boundary::Flux(Cons {
            rho: 1.0,
            mom: 1.0,
            e: 1.0,
        });
        assert_eq!(ghost(f, inner, 1.0).u, -30.0);
        let open = Boundary::Open { p: 1.0e5, rho: 1.2 };
        // right end, u = 30 out: Z = 0.8·√(1.4·1.1e5/0.8) = 351.0, P = 1.1e5 + 351·30 =
        // 120 530 ≥ p₀ → outflow at p₀, v = 20 530/351 = 58.49 m/s, the gas expanded to p₀:
        // ρ = 0.8·(1/1.1)^(1/1.4) = 0.74735 (hand)
        let out = ghost(open, inner, 1.0);
        assert!((out.u - 58.49).abs() < 0.01 && out.p == 1.0e5, "{out:?}");
        assert!((out.rho - 0.747_35).abs() < 1e-4, "{out:?}");
        // left end, the same u is inflow: P = 1.1e5 − 10 530 = 99 470 < p₀ → the reservoir's
        // air comes in: 0.6v² − 351v − 530 = 0, v = −1.508 m/s (hand), u = +1.508
        let inflow = ghost(open, inner, -1.0);
        assert!((inflow.u - 1.508).abs() < 0.01, "{inflow:?}");
        assert!(
            (inflow.rho - 1.2).abs() < 1e-3 && inflow.p < 1.0e5,
            "{inflow:?}"
        );
        // at rest at the outside pressure nothing moves
        let still = ghost(
            open,
            Prim {
                u: 0.0,
                p: 1.0e5,
                ..inner
            },
            1.0,
        );
        assert!(
            still.u.abs() < 1e-9 && (still.p - 1.0e5).abs() < 1e-9,
            "{still:?}"
        );
    }

    #[test]
    fn a_reservoir_feeds_the_left_end_with_its_own_gas() {
        // Dense cold reservoir (2 kg/m³, 1.2 bar) on the left of hot thin gas: the inflow brings
        // the reservoir's density into the first cells, not the pipe's own.
        let hot = Prim {
            rho: 0.6,
            u: 0.0,
            p: 1.0e5,
        };
        let mut p = Pipe1d::new(0.5, 50, hot);
        p.left = Boundary::Open { p: 1.2e5, rho: 2.0 };
        p.right = Boundary::Open { p: 1.0e5, rho: 0.6 };
        let mut t = 0.0;
        while t < 2e-3 {
            let dt = p.max_dt(0.8).min(2e-3 - t);
            p.step(dt);
            t += dt;
        }
        let c = p.prim(1);
        assert!(c.u > 0.0 && c.rho > 1.2, "{c:?}");
    }

    #[test]
    fn large_pulses_steepen_small_ones_do_not() {
        // Non-linear propagation: a strong compression front sharpens as it travels (the
        // crest outruns the base, u + c grows with amplitude). Compare front widths.
        let width = |amp: f64| {
            let air = Prim {
                rho: 1.2,
                u: 0.0,
                p: 101_325.0,
            };
            let mut p = Pipe1d::new(2.0, 1000, air);
            for i in 0..100 {
                let s = (std::f64::consts::PI * i as f64 / 100.0).sin();
                p.cells[i + 50] = Prim {
                    p: 101_325.0 * (1.0 + amp * s),
                    rho: 1.2 * (1.0 + amp * s / 1.4),
                    ..air
                }
                .to_cons();
            }
            let mut t = 0.0;
            while t < 2.5e-3 {
                let dt = p.max_dt(0.8).min(2.5e-3 - t);
                p.step(dt);
                t += dt;
            }
            let ps: Vec<f64> = (0..1000).map(|i| p.prim(i).p - 101_325.0).collect();
            let max = ps.iter().cloned().fold(0.0, f64::max);
            // cells on the leading edge between 10 % and 90 % of the crest
            let crest = ps.iter().position(|v| *v == max).unwrap();
            (crest..1000)
                .filter(|&i| ps[i] > 0.1 * max && ps[i] < 0.9 * max)
                .count()
        };
        assert!(
            width(0.5) < width(0.01) / 2,
            "strong {} vs weak {}",
            width(0.5),
            width(0.01)
        );
    }
}
