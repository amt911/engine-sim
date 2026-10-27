//! Four pipes at a time: the same finite-volume step as [`Pipe1d::step`], with the pipes of a
//! group (ITB runners: same geometry, different states and valve fluxes) in the four lanes of a
//! vector. Every lane does exactly the IEEE operations, in the same order, that the scalar step
//! does for that pipe (no fused multiply-add, no reassociation, no approximate sqrt/divide), and
//! every branch of the scalar code is a per-lane select of the value that branch would compute,
//! so the result is bit-for-bit the scalar one.

use super::*;
use std::cell::RefCell;
use std::sync::atomic::{AtomicBool, Ordering};
use wide::f64x4;

type V = f64x4;

static FORCE_SCALAR: AtomicBool = AtomicBool::new(false);

/// Force [`step_runners`] to step the pipes one at a time (the reference path). For tests that
/// compare the two paths; process-wide.
#[doc(hidden)]
pub fn set_scalar_runners(on: bool) {
    FORCE_SCALAR.store(on, Ordering::Relaxed);
}

#[derive(Clone, Copy)]
struct P4 {
    rho: V,
    u: V,
    p: V,
}

#[derive(Clone, Copy)]
struct C4 {
    rho: V,
    mom: V,
    e: V,
}

const ZERO_C4: C4 = C4 {
    rho: V::ZERO,
    mom: V::ZERO,
    e: V::ZERO,
};

fn k(x: f64) -> V {
    V::splat(x)
}

/// `m` is a comparison mask: `a` where set, `b` elsewhere.
fn sel(m: V, a: V, b: V) -> V {
    m.bitselect(a, b)
}

/// `f64::min` per lane (a NaN loses to a number).
fn fmin(a: V, b: V) -> V {
    sel(a.simd_lt(b) | b.is_nan(), a, b)
}

/// `f64::max` per lane (a NaN loses to a number).
fn fmax(a: V, b: V) -> V {
    sel(a.simd_gt(b) | b.is_nan(), a, b)
}

fn to_prim(c: C4) -> P4 {
    let u = c.mom / c.rho;
    P4 {
        rho: c.rho,
        u,
        p: k(GAMMA - 1.0) * (c.e - k(0.5) * c.rho * u * u),
    }
}

/// [`hllc`] per lane.
fn hllc4(l: P4, r: P4) -> C4 {
    let gm1 = k(GAMMA - 1.0);
    let half = k(0.5);
    let flux = |s: P4| {
        let e = s.p / gm1 + half * s.rho * s.u * s.u;
        (
            C4 {
                rho: s.rho * s.u,
                mom: s.rho * s.u * s.u + s.p,
                e: s.u * (e + s.p),
            },
            e,
        )
    };
    let cl = (k(GAMMA) * l.p / l.rho).sqrt();
    let cr = (k(GAMMA) * r.p / r.rho).sqrt();
    let sl = fmin(l.u - cl, r.u - cr);
    let sr = fmax(l.u + cl, r.u + cr);
    let (fl, el) = flux(l);
    let (fr, er) = flux(r);
    let ss = (r.p - l.p + l.rho * l.u * (sl - l.u) - r.rho * r.u * (sr - r.u))
        / (l.rho * (sl - l.u) - r.rho * (sr - r.u));
    // the star state of the side the contact sits on
    let left_side = ss.simd_ge(k(0.0));
    let s = P4 {
        rho: sel(left_side, l.rho, r.rho),
        u: sel(left_side, l.u, r.u),
        p: sel(left_side, l.p, r.p),
    };
    let f = C4 {
        rho: sel(left_side, fl.rho, fr.rho),
        mom: sel(left_side, fl.mom, fr.mom),
        e: sel(left_side, fl.e, fr.e),
    };
    let e = sel(left_side, el, er);
    let sk = sel(left_side, sl, sr);
    let kk = s.rho * (sk - s.u) / (sk - ss);
    let ustar = C4 {
        rho: kk,
        mom: kk * ss,
        e: kk * (e / s.rho + (ss - s.u) * (ss + s.p / (s.rho * (sk - s.u)))),
    };
    let u = C4 {
        rho: s.rho,
        mom: s.rho * s.u,
        e: s.p / gm1 + half * s.rho * s.u * s.u,
    };
    let star = C4 {
        rho: f.rho + sk * (ustar.rho - u.rho),
        mom: f.mom + sk * (ustar.mom - u.mom),
        e: f.e + sk * (ustar.e - u.e),
    };
    let go_left = sl.simd_ge(k(0.0));
    let go_right = sr.simd_le(k(0.0));
    let pick = |a: V, b: V, c: V| sel(go_left, a, sel(go_right, b, c));
    C4 {
        rho: pick(fl.rho, fr.rho, star.rho),
        mom: pick(fl.mom, fr.mom, star.mom),
        e: pick(fl.e, fr.e, star.e),
    }
}

/// [`muscl`] per lane.
fn muscl4(a: V, s: V, c: V) -> (V, V) {
    let (l, r) = (s - a, c - s);
    let lr = l * r;
    let slope = sel(lr.simd_le(k(0.0)), k(0.0), k(2.0) * l * r / (l + r));
    (s - k(0.5) * slope, s + k(0.5) * slope)
}

fn mix4(a: C4, b: C4, keep: f64) -> C4 {
    let (keep, kk) = (k(keep), k(1.0 - keep));
    C4 {
        rho: keep * a.rho + kk * b.rho,
        mom: keep * a.mom + kk * b.mom,
        e: keep * a.e + kk * b.e,
    }
}

fn add4(c: C4, d: C4, dt: V) -> C4 {
    C4 {
        rho: c.rho + dt * d.rho,
        mom: c.mom + dt * d.mom,
        e: c.e + dt * d.e,
    }
}

/// [`positive`] per lane.
fn positive4(c: C4, rho_floor: f64, p_floor: f64) -> C4 {
    let (gm1, half) = (k(GAMMA - 1.0), k(0.5));
    let (rf, pf) = (k(rho_floor), k(p_floor));
    let finite = c.rho.is_finite() & c.mom.is_finite() & c.e.is_finite();
    let broken = !finite | c.rho.simd_lt(rf);
    // Prim { rho: floor, u: 0, p: floor }.to_cons()
    let reset = C4 {
        rho: rf,
        mom: rf * k(0.0),
        e: pf / gm1 + half * rf * k(0.0) * k(0.0),
    };
    let x = c.mom / c.rho;
    let (lo, hi) = (k(-U_MAX), k(U_MAX));
    let u = sel(x.simd_lt(lo), lo, sel(x.simd_gt(hi), hi, x));
    let e_min = half * c.rho * u * u + pf / gm1;
    C4 {
        rho: sel(broken, reset.rho, c.rho),
        mom: sel(broken, reset.mom, c.rho * u),
        e: sel(broken, reset.e, sel(c.e.simd_lt(e_min), e_min, c.e)),
    }
}

/// Everything the lanes share: the group's geometry.
struct Geo<'a> {
    n: usize,
    scheme: Scheme,
    dx: f64,
    floors: (f64, f64),
    friction: f64,
    area: &'a [f64],
    face_area: &'a [f64],
    wall_coef: Vec<f64>,
}

/// Scratch of the vector step (kept per thread: no allocation per step).
#[derive(Default)]
struct Work {
    cells: Vec<C4>,
    u0: Vec<C4>,
    rhs: Vec<C4>,
    w: Vec<P4>,
    qv: Vec<V>,
    rho_w: Vec<V>,
    p_w: Vec<V>,
    recon: [Vec<V>; 6],
    edge_l: Vec<P4>,
    edge_r: Vec<P4>,
    fluxes: Vec<C4>,
}

impl Work {
    fn fit(&mut self, n: usize) {
        let z = P4 {
            rho: V::ZERO,
            u: V::ZERO,
            p: V::ZERO,
        };
        self.cells.resize(n, ZERO_C4);
        self.u0.resize(n, ZERO_C4);
        self.rhs.resize(n, ZERO_C4);
        self.w.resize(n + 6, z);
        self.qv.resize(n + 6, V::ZERO);
        self.rho_w.resize(n + 6, V::ZERO);
        self.p_w.resize(n + 6, V::ZERO);
        for r in &mut self.recon {
            r.resize(n + 6, V::ZERO);
        }
        self.edge_l.resize(n + 6, z);
        self.edge_r.resize(n + 6, z);
        self.fluxes.resize(n + 1, ZERO_C4);
    }
}

/// [`Pipe1d::eval_rhs`] per lane, for pipes with a state ghost `gl` on the left and a prescribed
/// flux `right` on the right (and no acoustic loss).
fn eval_rhs4(wk: &mut Work, g: &Geo, gl: P4, right: C4) {
    let n = g.n;
    let last = to_prim(wk.cells[n - 1]);
    // the flux end reconstructs against a wall
    let gr = P4 { u: -last.u, ..last };
    let w = &mut wk.w;
    w[..3].fill(gl);
    for (s, c) in w[3..n + 3].iter_mut().zip(&wk.cells) {
        *s = to_prim(*c);
    }
    w[n + 3..].fill(gr);
    for (kk, (q, s)) in wk.qv.iter_mut().zip(w.iter()).enumerate() {
        *q = s.u * k(g.area[(kk.max(3) - 3).min(n - 1)]);
    }
    for (kk, s) in w.iter().enumerate() {
        wk.rho_w[kk] = s.rho;
        wk.p_w[kk] = s.p;
    }
    let [rl, rr, ql, qr, pl, pr] = &mut wk.recon;
    let run = |v: &[V], left: &mut [V], right: &mut [V]| match g.scheme {
        Scheme::Weno5Rk3 => {
            for i in 2..v.len() - 2 {
                let (l, r) = weno5z([v[i - 2], v[i - 1], v[i], v[i + 1], v[i + 2]]);
                left[i] = l;
                right[i] = r;
            }
        }
        Scheme::MusclRk2 => {
            for i in 1..v.len() - 1 {
                let (l, r) = muscl4(v[i - 1], v[i], v[i + 1]);
                left[i] = l;
                right[i] = r;
            }
        }
    };
    run(&wk.rho_w, rl, rr);
    run(&wk.qv, ql, qr);
    run(&wk.p_w, pl, pr);
    for i in 2..n + 4 {
        let s = w[i];
        let face = |kk: usize| k(g.face_area[kk.clamp(3, n + 3) - 3]);
        let (ul, ur) = (ql[i] / face(i), qr[i] / face(i + 1));
        let ok = |e: P4| {
            let m = e.rho.simd_gt(k(0.0)) & e.p.simd_gt(k(0.0));
            P4 {
                rho: sel(m, e.rho, s.rho),
                u: sel(m, e.u, s.u),
                p: sel(m, e.p, s.p),
            }
        };
        wk.edge_l[i] = ok(P4 {
            rho: rl[i],
            u: ul,
            p: pl[i],
        });
        wk.edge_r[i] = ok(P4 {
            rho: rr[i],
            u: ur,
            p: pr[i],
        });
    }
    for (kk, f) in wk.fluxes.iter_mut().enumerate() {
        *f = hllc4(wk.edge_r[kk + 2], wk.edge_l[kk + 3]);
    }
    wk.fluxes[n] = right;
    for (j, r) in wk.rhs.iter_mut().enumerate() {
        let (a, b) = (wk.fluxes[j], wk.fluxes[j + 1]);
        let (fa, fb) = (k(g.face_area[j]), k(g.face_area[j + 1]));
        let inv = k(1.0 / (g.area[j] * g.dx));
        let area = k(g.area[j]);
        let ps = wk.edge_r[j + 3].p * (fb - area) + wk.edge_l[j + 3].p * (area - fa);
        let cell = w[j + 3];
        let wall = if g.friction > 0.0 {
            k(g.friction * g.wall_coef[j]) * cell.rho * cell.u * cell.u.abs()
        } else {
            k(0.0)
        };
        *r = C4 {
            rho: -(fb * b.rho - fa * a.rho) * inv,
            mom: -(fb * b.mom - fa * a.mom - ps) * inv + wall,
            e: -(fb * b.e - fa * a.e) * inv,
        };
    }
}

/// [`Pipe1d::step`] per lane: returns the end fluxes.
fn step4(wk: &mut Work, g: &Geo, gl: P4, right: C4, dt: f64) -> (C4, C4) {
    let n = g.n;
    let dt = k(dt);
    wk.u0.copy_from_slice(&wk.cells);
    let mut ends = (ZERO_C4, ZERO_C4);
    let stages: &[(f64, f64)] = match g.scheme {
        Scheme::Weno5Rk3 => &[(0.0, 1.0 / 6.0), (0.75, 1.0 / 6.0), (1.0 / 3.0, 4.0 / 6.0)],
        Scheme::MusclRk2 => &[(0.0, 0.5), (0.5, 0.5)],
    };
    for (stage, &(keep, w)) in stages.iter().enumerate() {
        eval_rhs4(wk, g, gl, right);
        ends.0 = add4(ends.0, wk.fluxes[0], k(w));
        ends.1 = add4(ends.1, wk.fluxes[n], k(w));
        for ((c, d), c0) in wk.cells.iter_mut().zip(&wk.rhs).zip(&wk.u0) {
            let u = add4(*c, *d, dt);
            let next = if stage == 0 { u } else { mix4(*c0, u, keep) };
            *c = positive4(next, g.floors.0, g.floors.1);
        }
    }
    ends
}

fn lanes(f: impl Fn(usize) -> f64) -> V {
    V::from([f(0), f(1), f(2), f(3)])
}

fn cons_lane(c: C4, l: usize) -> Cons {
    Cons {
        rho: c.rho.to_array()[l],
        mom: c.mom.to_array()[l],
        e: c.e.to_array()[l],
    }
}

/// Whether `pipes` can share a vector step: the same geometry and scheme, a flux on the right,
/// no acoustic loss.
fn same_group(pipes: &[Pipe1d]) -> bool {
    let a = &pipes[0];
    let same = |x: &[f64], y: &[f64]| x.iter().zip(y).all(|(x, y)| x.to_bits() == y.to_bits());
    a.cells.len() >= 2
        && pipes.iter().all(|p| {
            p.cells.len() == a.cells.len()
                && p.scheme == a.scheme
                && p.dx.to_bits() == a.dx.to_bits()
                && p.floors.0.to_bits() == a.floors.0.to_bits()
                && p.floors.1.to_bits() == a.floors.1.to_bits()
                && p.friction.to_bits() == a.friction.to_bits()
                && p.acoustic_loss <= 0.0
                && matches!(p.right, Boundary::Flux(_))
                && same(&p.area, &a.area)
                && same(&p.face_area, &a.face_area)
        })
}

thread_local! {
    static WORK: RefCell<Work> = RefCell::new(Work::default());
}

/// `substeps` steps of `h` for every pipe of `pipes`, each with its right end on its prescribed
/// flux and its left end on the state `ghost(cell 0)` chosen afresh before every substep; the
/// left-end mass flux of pipe `r`'s substep `s` goes to `out[r * substeps + s]`. The same as
/// calling [`Pipe1d::step`] pipe by pipe, bit for bit, but the pipes go four at a time through
/// the vector unit (a group short of four pads the lanes it lacks).
pub fn step_runners(
    pipes: &mut [Pipe1d],
    h: f64,
    substeps: usize,
    ghost: impl Fn(Prim) -> Prim,
    out: &mut [f64],
) {
    if substeps == 0 {
        return;
    }
    for (group, out) in pipes.chunks_mut(4).zip(out.chunks_mut(4 * substeps)) {
        if group.len() < 2 || FORCE_SCALAR.load(Ordering::Relaxed) || !same_group(group) {
            for (pipe, o) in group.iter_mut().zip(out.chunks_mut(substeps)) {
                for fs in o.iter_mut() {
                    pipe.left = Boundary::State(ghost(pipe.prim(0)));
                    pipe.step(h);
                    *fs = pipe.end_flux.0.rho;
                }
            }
        } else {
            WORK.with_borrow_mut(|wk| step_group(wk, group, h, substeps, &ghost, out));
        }
    }
}

fn step_group(
    wk: &mut Work,
    group: &mut [Pipe1d],
    h: f64,
    substeps: usize,
    ghost: &impl Fn(Prim) -> Prim,
    out: &mut [f64],
) {
    let m = group.len();
    // lanes past the group repeat the first pipe: whatever they do, the real ones don't see it
    let src = |l: usize| &group[if l < m { l } else { 0 }];
    let n = group[0].cells.len();
    wk.fit(n);
    for i in 0..n {
        wk.cells[i] = C4 {
            rho: lanes(|l| src(l).cells[i].rho),
            mom: lanes(|l| src(l).cells[i].mom),
            e: lanes(|l| src(l).cells[i].e),
        };
    }
    let flux_of = |l: usize| match src(l).right {
        Boundary::Flux(f) => f,
        _ => Cons::default(),
    };
    let right = C4 {
        rho: lanes(|l| flux_of(l).rho),
        mom: lanes(|l| flux_of(l).mom),
        e: lanes(|l| flux_of(l).e),
    };
    let a = &group[0];
    let geo = Geo {
        n,
        scheme: a.scheme,
        dx: a.dx,
        floors: a.floors,
        friction: a.friction,
        area: &a.area,
        face_area: &a.face_area,
        wall_coef: a
            .area
            .iter()
            .map(|a| -1.0 / (2.0 * (4.0 * a / std::f64::consts::PI).sqrt()))
            .collect(),
    };
    let mut gl_last = [Prim {
        rho: 1.0,
        u: 0.0,
        p: 1.0,
    }; 4];
    let mut ends = (ZERO_C4, ZERO_C4);
    for s in 0..substeps {
        let gs: [Prim; 4] = std::array::from_fn(|l| ghost(cons_lane(wk.cells[0], l).to_prim()));
        gl_last = gs;
        let gl = P4 {
            rho: lanes(|l| gs[l].rho),
            u: lanes(|l| gs[l].u),
            p: lanes(|l| gs[l].p),
        };
        ends = step4(wk, &geo, gl, right, h);
        let f0 = ends.0.rho.to_array();
        for (r, o) in out.chunks_mut(substeps).enumerate().take(m) {
            o[s] = f0[r];
        }
    }
    for (r, pipe) in group.iter_mut().enumerate() {
        for (i, c) in pipe.cells.iter_mut().enumerate() {
            *c = cons_lane(wk.cells[i], r);
        }
        pipe.end_flux = (cons_lane(ends.0, r), cons_lane(ends.1, r));
        pipe.left = Boundary::State(gl_last[r]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exhaust_fv::throttled_ghost;

    /// A small deterministic generator in [0, 1).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    /// `count` ITB-like runners (same geometry) in different states: a quiet one, ones ringing
    /// at different amplitudes, a strong step (HLLC across a shock, `physical_or`), a near
    /// vacuum (the floors).
    fn runners(count: usize, scheme: Scheme) -> Vec<Pipe1d> {
        let mut rng = Lcg(7);
        (0..count)
            .map(|i| {
                let init = Prim {
                    rho: 1.2,
                    u: 0.0,
                    p: 101_325.0,
                };
                let mut p = Pipe1d::segmented(&[(0.325, 1.6e-3)], 0.325 / 20.0, init);
                p.floors = (1e-3, 500.0);
                p.friction = 0.02;
                p.scheme = scheme;
                let n = p.cells.len();
                for (j, c) in p.cells.iter_mut().enumerate() {
                    let x = j as f64 / n as f64;
                    let amp = 0.05 * i as f64 + 0.01;
                    let s = Prim {
                        rho: 1.2 * (1.0 + amp * (6.0 * x + i as f64).sin()),
                        u: 80.0 * amp * (9.0 * x).cos() * (rng.next() + 0.5),
                        p: 101_325.0 * (1.0 + 2.0 * amp * (6.0 * x + i as f64).sin()),
                    };
                    *c = s.to_cons();
                }
                match i {
                    3 => {
                        for c in &mut p.cells[n / 2..] {
                            *c = Prim {
                                rho: 6.0,
                                u: -40.0,
                                p: 8.0e5,
                            }
                            .to_cons();
                        }
                    }
                    4 => {
                        for c in &mut p.cells[..n / 3] {
                            *c = Prim {
                                rho: 2e-3,
                                u: 300.0,
                                p: 900.0,
                            }
                            .to_cons();
                        }
                    }
                    _ => {}
                }
                p
            })
            .collect()
    }

    /// The valve end of pipe `i` at step `t`: a flux that changes sign and size.
    fn valve(p: &Pipe1d, i: usize, t: usize) -> Boundary {
        let n = p.cells.len();
        let phase = t as f64 * 0.013 * (1.0 + 0.2 * i as f64);
        let mass = 40.0
            * phase.sin()
            * (1.0 + i as f64 * 0.3)
            * (1.0 + 4.0 * t.is_multiple_of(97) as u8 as f64);
        Boundary::Flux(flux_boundary(mass, mass * 3.0e5, p.prim(n - 1)))
    }

    /// Both paths for `steps` steps, the batched one against pipe-by-pipe `Pipe1d::step`: every
    /// cell and end flux is the same to the bit, every step.
    /// `mode`: 0 a throttled trumpet, 1 a hard push, 2 the hard push with a NaN now and then.
    fn same_both_ways(count: usize, scheme: Scheme, steps: usize, mode: u8) {
        let mut a = runners(count, scheme);
        let mut b = a.clone();
        let rho_atm = 1.17;
        let ghost = |c: Prim| {
            if mode >= 1 {
                // a hard push: compression one moment, near vacuum the next
                let p = c.p * (0.2 + 6.0 * (c.u * 0.02).sin().abs());
                Prim {
                    rho: c.rho * (1.0 + 2.0 * (c.p * 1e-4).sin().abs()),
                    u: -60.0 * (c.u * 0.01).cos(),
                    p: if mode == 2 && (c.p * 1e-3).sin() > 0.995 {
                        f64::NAN
                    } else {
                        p
                    },
                }
            } else {
                throttled_ghost(c, 101_325.0, rho_atm, -1.0, 0.4)
            }
        };
        let k = 2;
        let mut fa = vec![0.0; count * k];
        let mut fb = vec![0.0; count * k];
        for t in 0..steps {
            for (i, p) in a.iter_mut().enumerate() {
                p.right = valve(p, i, t);
            }
            for (i, p) in b.iter_mut().enumerate() {
                p.right = valve(p, i, t);
            }
            let h = a
                .iter()
                .map(|p| p.max_dt(0.7))
                .fold(f64::INFINITY, f64::min);
            // scalar reference
            for (i, p) in a.iter_mut().enumerate() {
                for s in 0..k {
                    p.left = Boundary::State(ghost(p.prim(0)));
                    p.step(h);
                    fa[i * k + s] = p.end_flux.0.rho;
                }
            }
            step_runners(&mut b, h, k, ghost, &mut fb);
            for (i, (x, y)) in a.iter().zip(&b).enumerate() {
                for (j, (c, d)) in x.cells.iter().zip(&y.cells).enumerate() {
                    assert_eq!(
                        c.rho.to_bits(),
                        d.rho.to_bits(),
                        "step {t} pipe {i} cell {j} rho"
                    );
                    assert_eq!(
                        c.mom.to_bits(),
                        d.mom.to_bits(),
                        "step {t} pipe {i} cell {j} mom"
                    );
                    assert_eq!(c.e.to_bits(), d.e.to_bits(), "step {t} pipe {i} cell {j} e");
                }
                for (u, v) in [(x.end_flux.0, y.end_flux.0), (x.end_flux.1, y.end_flux.1)] {
                    assert_eq!(
                        u.rho.to_bits(),
                        v.rho.to_bits(),
                        "step {t} pipe {i} end rho"
                    );
                    assert_eq!(
                        u.mom.to_bits(),
                        v.mom.to_bits(),
                        "step {t} pipe {i} end mom"
                    );
                    assert_eq!(u.e.to_bits(), v.e.to_bits(), "step {t} pipe {i} end e");
                }
            }
            for (u, v) in fa.iter().zip(&fb) {
                assert_eq!(u.to_bits(), v.to_bits(), "step {t} left-end flux");
            }
        }
        // the states really went somewhere
        assert!(b.iter().all(|p| p.cells.iter().all(|c| c.rho.is_finite())));
    }

    #[test]
    fn batched_weno_runners_equal_the_scalar_step_to_the_bit() {
        same_both_ways(6, Scheme::Weno5Rk3, 2500, 0);
    }

    #[test]
    fn batched_muscl_runners_equal_the_scalar_step_to_the_bit() {
        same_both_ways(6, Scheme::MusclRk2, 2500, 0);
    }

    #[test]
    fn batched_runners_equal_the_scalar_step_under_a_hard_push() {
        same_both_ways(6, Scheme::Weno5Rk3, 2500, 1);
        same_both_ways(6, Scheme::MusclRk2, 2500, 1);
    }

    #[test]
    fn batched_runners_equal_the_scalar_step_through_a_nan_ghost() {
        same_both_ways(6, Scheme::Weno5Rk3, 2500, 2);
        same_both_ways(6, Scheme::MusclRk2, 2500, 2);
    }

    #[test]
    fn a_lone_runner_and_a_full_batch_of_four_also_agree() {
        same_both_ways(1, Scheme::Weno5Rk3, 300, 0);
        same_both_ways(4, Scheme::Weno5Rk3, 300, 0);
        same_both_ways(10, Scheme::MusclRk2, 300, 0);
    }
}
