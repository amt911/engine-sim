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
}

/// A uniform pipe of `n` cells.
#[derive(Debug, Clone)]
pub struct Pipe1d {
    pub cells: Vec<Cons>,
    pub dx: f64,
    pub left: Boundary,
    pub right: Boundary,
}

impl Pipe1d {
    pub fn new(length: f64, n: usize, init: Prim) -> Self {
        Self {
            cells: vec![init.to_cons(); n.max(1)],
            dx: length / n.max(1) as f64,
            left: Boundary::Wall,
            right: Boundary::Wall,
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
    /// Advance by `dt` (SSP-RK2 with MUSCL-minmod reconstruction).
    pub fn step(&mut self, dt: f64) {
        let r1 = self.rhs();
        let u0 = self.cells.clone();
        for (c, d) in self.cells.iter_mut().zip(&r1) {
            *c = add(*c, *d, dt);
        }
        let r2 = self.rhs();
        for ((c, d), c0) in self.cells.iter_mut().zip(&r2).zip(&u0) {
            let u1 = add(*c, *d, dt);
            *c = Cons {
                rho: 0.5 * (c0.rho + u1.rho),
                mom: 0.5 * (c0.mom + u1.mom),
                e: 0.5 * (c0.e + u1.e),
            };
        }
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

fn minmod(a: f64, b: f64) -> f64 {
    if a * b <= 0.0 {
        0.0
    } else if a.abs() < b.abs() {
        a
    } else {
        b
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
    }
}

impl Pipe1d {
    /// dU/dt for every cell.
    fn rhs(&self) -> Vec<Cons> {
        let n = self.cells.len();
        // primitive states with two ghost cells on each side
        let mut w = Vec::with_capacity(n + 4);
        let first = self.cells[0].to_prim();
        let last = self.cells[n - 1].to_prim();
        let gl = ghost(self.left, first, -1.0);
        let gr = ghost(self.right, last, 1.0);
        w.push(gl);
        w.push(gl);
        w.extend(self.cells.iter().map(|c| c.to_prim()));
        w.push(gr);
        w.push(gr);
        // limited slopes for cells 1..=n+2 of w
        let slope = |i: usize, f: fn(&Prim) -> f64| {
            minmod(f(&w[i]) - f(&w[i - 1]), f(&w[i + 1]) - f(&w[i]))
        };
        let face = |i: usize, side: f64| -> Prim {
            let s = w[i];
            let r = Prim {
                rho: s.rho + side * 0.5 * slope(i, |p| p.rho),
                u: s.u + side * 0.5 * slope(i, |p| p.u),
                p: s.p + side * 0.5 * slope(i, |p| p.p),
            };
            if r.rho > 0.0 && r.p > 0.0 { r } else { s }
        };
        // faces between w[k] and w[k+1] for k = 1..=n+1 (cell j = k−2 sits between faces k−1, k)
        let fluxes: Vec<Cons> = (1..=n + 1)
            .map(|k| hllc(face(k, 1.0), face(k + 1, -1.0)))
            .collect();
        (0..n)
            .map(|j| {
                let (a, b) = (fluxes[j], fluxes[j + 1]);
                Cons {
                    rho: -(b.rho - a.rho) / self.dx,
                    mom: -(b.mom - a.mom) / self.dx,
                    e: -(b.e - a.e) / self.dx,
                }
            })
            .collect()
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
