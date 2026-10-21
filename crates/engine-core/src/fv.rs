//! Quasi-1D compressible Euler solver for pipes: finite volumes, HLLC Riemann solver,
//! MUSCL reconstruction (minmod) and SSP-RK2 time stepping (Toro, *Riemann Solvers and
//! Numerical Methods for Fluid Dynamics*, 3rd ed., ch. 10, 13, 14). Non-linear: pulses steepen
//! into shocks, which is what makes a loaded exhaust sound harsh (research addendum).
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

/// A uniform pipe of `n` cells.
#[derive(Debug, Clone)]
pub struct Pipe1d {
    pub cells: Vec<Cons>,
    pub dx: f64,
    pub left: Boundary,
    pub right: Boundary,
    /// Scratch buffers (no allocation per step): primitive states with ghosts, face fluxes,
    /// stage-1 copy, and the right-hand sides of both RK stages.
    w: Vec<Prim>,
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
}

impl Pipe1d {
    pub fn new(length: f64, n: usize, init: Prim) -> Self {
        let n = n.max(1);
        Self {
            cells: vec![init.to_cons(); n],
            dx: length / n as f64,
            left: Boundary::Wall,
            right: Boundary::Wall,
            w: Vec::with_capacity(n + 4),
            fluxes: vec![Cons::default(); n + 1],
            u0: vec![Cons::default(); n],
            rhs: vec![Cons::default(); n],
            end_flux: (Cons::default(), Cons::default()),
            floors: (1e-9, 1e-9),
            area: vec![1.0; n],
            face_area: vec![1.0; n + 1],
            friction: 0.0,
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

    /// Advance by `dt` (SSP-RK2 with MUSCL-minmod reconstruction).
    pub fn step(&mut self, dt: f64) {
        self.u0.copy_from_slice(&self.cells);
        self.eval_rhs();
        let (l1, r1) = (self.fluxes[0], self.fluxes[self.cells.len()]);
        for (c, d) in self.cells.iter_mut().zip(&self.rhs) {
            *c = positive(add(*c, *d, dt), self.floors.0, self.floors.1);
        }
        self.eval_rhs();
        let (l2, r2) = (self.fluxes[0], self.fluxes[self.cells.len()]);
        for ((c, d), c0) in self.cells.iter_mut().zip(&self.rhs).zip(&self.u0) {
            let u1 = add(*c, *d, dt);
            *c = Cons {
                rho: 0.5 * (c0.rho + u1.rho),
                mom: 0.5 * (c0.mom + u1.mom),
                e: 0.5 * (c0.e + u1.e),
            };
        }
        for c in &mut self.cells {
            *c = positive(*c, self.floors.0, self.floors.1);
        }
        let avg = |a: Cons, b: Cons| Cons {
            rho: 0.5 * (a.rho + b.rho),
            mom: 0.5 * (a.mom + b.mom),
            e: 0.5 * (a.e + b.e),
        };
        self.end_flux = (avg(l1, l2), avg(r1, r2));
    }

    /// Mass flow per unit area through the right end during the last step (ρu at the face).
    pub fn prim(&self, i: usize) -> Prim {
        self.cells[i].to_prim()
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
fn positive(c: Cons, rho_floor: f64, p_floor: f64) -> Cons {
    if !(c.rho.is_finite() && c.mom.is_finite() && c.e.is_finite()) || c.rho < rho_floor {
        return Prim {
            rho: rho_floor,
            u: 0.0,
            p: p_floor,
        }
        .to_cons();
    }
    let u = c.mom / c.rho;
    let e_min = 0.5 * c.rho * u * u + p_floor / (GAMMA - 1.0);
    if c.e < e_min {
        Cons { e: e_min, ..c }
    } else {
        c
    }
}

fn minmod(a: f64, b: f64) -> f64 {
    // van Leer limiter (smooth, TVD; less dissipative than minmod at a few cells per wavelength)
    if a * b <= 0.0 {
        0.0
    } else {
        2.0 * a * b / (a + b)
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
        Boundary::Open { p, rho } => {
            if inner.u * outward > 0.0 {
                // subsonic outflow: the outside imposes its pressure
                Prim { p, ..inner }
            } else {
                // inflow: reservoir density and pressure
                Prim { rho, u: inner.u, p }
            }
        }
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
        w.push(gl);
        w.push(gl);
        w.extend(self.cells.iter().map(|c| c.to_prim()));
        w.push(gr);
        w.push(gr);
        let w = &self.w;
        let face = |i: usize, side: f64| -> Prim {
            let (a, s, c) = (w[i - 1], w[i], w[i + 1]);
            let r = Prim {
                rho: s.rho + side * 0.5 * minmod(s.rho - a.rho, c.rho - s.rho),
                u: s.u + side * 0.5 * minmod(s.u - a.u, c.u - s.u),
                p: s.p + side * 0.5 * minmod(s.p - a.p, c.p - s.p),
            };
            if r.rho > 0.0 && r.p > 0.0 { r } else { s }
        };
        // face k sits between w[k] and w[k+1], k = 1..=n+1; cell j lies between faces j and j+1
        for (k, f) in self.fluxes.iter_mut().enumerate() {
            *f = hllc(face(k + 1, 1.0), face(k + 2, -1.0));
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
            // geometric source p·dA/dx keeps still gas still across an area change
            let p = self.w[j + 2].p;
            let cell = self.w[j + 2];
            let wall = if self.friction > 0.0 {
                let d = (4.0 * self.area[j] / std::f64::consts::PI).sqrt();
                -self.friction / (2.0 * d) * cell.rho * cell.u * cell.u.abs()
            } else {
                0.0
            };
            *r = Cons {
                rho: -(fb * b.rho - fa * a.rho) * inv,
                mom: -(fb * b.mom - fa * a.mom - p * (fb - fa)) * inv + wall,
                e: -(fb * b.e - fa * a.e) * inv,
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

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
        let ratio = transmitted_peak(1e-3, 4e-3) / transmitted_peak(1e-3, 1e-3);
        assert!((ratio - 0.4).abs() < 0.05, "transmitted {ratio:.3}");
        // and into a narrower pipe (A₂ = A₁/4): 2·4/(4+1) = 1.6
        let up = transmitted_peak(4e-3, 1e-3) / transmitted_peak(4e-3, 4e-3);
        assert!((up - 1.6).abs() < 0.15, "transmitted {up:.3}");
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
            let front = (crest..1000)
                .filter(|&i| ps[i] > 0.1 * max && ps[i] < 0.9 * max)
                .count();
            front
        };
        assert!(
            width(0.5) < width(0.01) / 2,
            "strong {} vs weak {}",
            width(0.5),
            width(0.01)
        );
    }
}
