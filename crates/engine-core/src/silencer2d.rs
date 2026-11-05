//! A straight-through perforated dissipative silencer by 2D axisymmetric mode matching (Selamet,
//! Xu, Lee & Huff, *JASA* 117 (2005) 2078; eigen-equation re-derived in e^{jωt}; fibre and
//! perforate of Selamet, Lee & Huff, *JSV* 262 (2003) eqs. (20)–(23)), and its transmission loss
//! realised as a cascade of peaking biquads (Välimäki & Liski, "Accurate cascade graphic
//! equalizer", *IEEE SPL* 24(2), 2017) for the waveguide.
//!
//! The 1D lumped lining ([`crate::waveguide::Shunt::lining`]) misses the measured silencer of
//! JSV 262 Fig. 7 by up to 45 dB above 2 kHz (no beaming) and 17 dB below 1 kHz; this model is
//! within a few dB of it (docs/FACTS.md, "The packed silencer against a measured one").

use num_complex::Complex64 as C;

/// The perforated tube between the airway and the packing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Perforate {
    /// open-area ratio
    pub porosity: f64,
    pub wall_m: f64,
    pub hole_m: f64,
}

impl Default for Perforate {
    /// JSV 262's tube: 8 %, 0.9 mm wall, 2.49 mm holes.
    fn default() -> Self {
        Self {
            porosity: 0.08,
            wall_m: 0.9e-3,
            hole_m: 2.49e-3,
        }
    }
}

/// The gas the silencer holds: sound speed (m/s) and density (kg/m³).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gas {
    pub c: f64,
    pub rho: f64,
}

/// One silencer: airway radius `r1`, shell radius `r2`, length `len` (m); fibre of flow
/// resistivity `resistivity` (Pa·s/m²), or air behind the perforate if `None`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Silencer {
    pub r1: f64,
    pub r2: f64,
    pub len: f64,
    pub resistivity: Option<f64>,
    pub perforate: Perforate,
}

/// Airway modes and chamber modes kept in the matching (the prototype's 6 and 14).
const N_AIRWAY: usize = 6;
const N_CHAMBER: usize = 14;
/// Points of the radial quadrature, each region.
const N_RADIAL: usize = 100;
/// Zeros of J₁ after the plane mode's 0: the rigid airway's radial wavenumbers × r1.
const J1_ZEROS: [f64; N_AIRWAY - 1] = [
    3.831_705_97,
    7.015_586_67,
    10.173_468_14,
    13.323_691_94,
    16.470_630_05,
];

/// J₀(z), J₁(z), Y₀(z), Y₁(z); NaN where they overflow.
fn bessel01(z: C) -> [C; 4] {
    let nan = C::new(f64::NAN, f64::NAN);
    let seq = |r: Result<complex_bessel::BesselResult<f64>, _>| match r {
        Ok(v) if v.values.len() == 2 => [v.values[0], v.values[1]],
        _ => [nan, nan],
    };
    let s = complex_bessel::Scaling::Unscaled;
    let j = seq(complex_bessel::besselj_seq(0.0, z, 2, s));
    // Y has a log singularity at 0: the airway's plane mode never asks for it
    let y = if z.norm() > 0.0 {
        seq(complex_bessel::bessely_seq(0.0, z, 2, s))
    } else {
        [nan, nan]
    };
    [j[0], j[1], y[0], y[1]]
}

fn j0(z: C) -> C {
    complex_bessel::besselj(0.0, z).unwrap_or(C::new(f64::NAN, f64::NAN))
}

/// Characteristic impedance and wavenumber of the fibre: Nice's fit (JSV 262 eqs. 22–23,
/// conjugated to e^{jωt}), made at ρ = 1.1555 kg/m³ in ρf/R — so a gas of other density
/// reads it at f·ρ/1.1555.
fn fibre(f: f64, r: f64, z0: f64, k0: f64, rho0: f64) -> (C, C) {
    let x = f * rho0 / 1.1555 / r;
    let za = z0 * C::new(1.0 + 0.0855 * x.powf(-0.754), -0.0765 * x.powf(-0.732));
    let ka = k0 * C::new(1.0 + 0.1472 * x.powf(-0.577), -0.1734 * x.powf(-0.595));
    (za, ka)
}

/// Solve `m·x = b` in place by Gaussian elimination with partial pivoting; None if singular.
fn solve(mut m: Vec<Vec<C>>, mut b: Vec<C>) -> Option<Vec<C>> {
    let n = b.len();
    for col in 0..n {
        let p = (col..n).max_by(|&a, &c| m[a][col].norm().total_cmp(&m[c][col].norm()))?;
        let pn = m[p][col].norm();
        if pn.is_nan() || pn <= 0.0 {
            return None;
        }
        m.swap(col, p);
        b.swap(col, p);
        let pivot = m[col].clone();
        for row in col + 1..n {
            let k = m[row][col] / pivot[col];
            if k == C::new(0.0, 0.0) {
                continue;
            }
            for (a, v) in m[row][col..].iter_mut().zip(&pivot[col..]) {
                *a -= k * v;
            }
            let v = b[col];
            b[row] -= k * v;
        }
    }
    let mut x = vec![C::new(0.0, 0.0); n];
    for row in (0..n).rev() {
        let s: C = (row + 1..n).map(|c| m[row][c] * x[c]).sum();
        x[row] = (b[row] - s) / m[row][row];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

fn trapezoid(y: &[C], r: &[f64]) -> C {
    (1..r.len())
        .map(|i| 0.5 * (y[i] * r[i] + y[i - 1] * r[i - 1]) * (r[i] - r[i - 1]))
        .sum()
}

impl Silencer {
    /// Transmission loss (dB) at `f` Hz in `gas`, between anechoic pipes of the airway's bore;
    /// None if the matching fails to solve.
    pub fn transmission_loss_db(&self, f: f64, gas: Gas) -> Option<f64> {
        let (r1, r2, len) = (self.r1, self.r2, self.len);
        let w = 2.0 * std::f64::consts::PI * f;
        let k0 = w / gas.c;
        let rho0 = gas.rho;
        let z0 = rho0 * gas.c;
        let pf = self.perforate;
        let (zt, kt, zp) = match self.resistivity {
            // air behind the perforate; a vanishing loss keeps the plane mode off kr = 0
            None => {
                let zt = z0 * C::new(1.0, -1e-5);
                let kt = k0 * C::new(1.0, -1e-5);
                let zp = C::new(0.006, k0 * (pf.wall_m + 0.75 * pf.hole_m)) / pf.porosity;
                (zt, kt, zp)
            }
            Some(res) => {
                let (zt, kt) = fibre(f, res, z0, k0, rho0);
                let corr = 1.0 + zt * kt / (z0 * k0);
                let zp = (C::new(0.006, 0.0)
                    + C::i() * k0 * (pf.wall_m + 0.375 * pf.hole_m * corr))
                    / pf.porosity;
                (zt, kt, zp)
            }
        };
        let rhot = zt * kt / w;
        let ktr = |kr: C| (kt * kt - k0 * k0 + kr * kr).sqrt();
        // the chamber's radial factor: q, and F = ratio at r1 of the packing's (Z₀/Z₁) pair
        let chamber = |kr: C| {
            let kq = ktr(kr);
            let b2 = bessel01(kq * r2);
            let q = b2[1] / b2[3];
            let b1 = bessel01(kq * r1);
            (kq, q, b1)
        };
        let g = |s: C| {
            let kr = s.sqrt();
            let (kq, q, b1) = chamber(kr);
            let f_ = (b1[0] - q * b1[2]) / (b1[1] - q * b1[3]);
            let a = bessel01(kr * r1);
            a[0] + C::i() * zp * (kr / k0) * a[1] - (rhot * kr / (rho0 * kq)) * a[1] * f_
        };
        // roots in s = kr²: Newton from the local minima of |G| over a grid of axial
        // wavenumbers kx (s = k0² − kx²), seeded first by the bulk-reacting effective medium
        // kx² = (k0²A1/ρ0 + k̃²A2/ρ̃)/(A1/ρ0 + A2/ρ̃)
        let kmax = (4.0 * kt.norm()).max(2.5 * N_CHAMBER as f64 / r2 + 2.0 * k0);
        let (nr, ni) = (61, 46);
        let mut grid = vec![vec![(C::new(0.0, 0.0), f64::INFINITY); nr]; ni];
        for (i, row) in grid.iter_mut().enumerate() {
            let xi = -kmax + 1.05 * kmax * i as f64 / (ni - 1) as f64;
            for (j, cell) in row.iter_mut().enumerate() {
                let xr = -kmax + 2.0 * kmax * j as f64 / (nr - 1) as f64;
                let kx = C::new(xr, xi);
                let s = k0 * k0 - kx * kx;
                let v = g(s).norm();
                *cell = (s, if v.is_finite() { v } else { f64::INFINITY });
            }
        }
        let a1 = std::f64::consts::PI * r1 * r1;
        let a2 = std::f64::consts::PI * (r2 * r2 - r1 * r1);
        let kx_eff2 = (k0 * k0 * a1 / rho0 + kt * kt * a2 / rhot) / (a1 / rho0 + a2 / rhot);
        let mut cand: Vec<C> = [1.0, 0.9, 1.1]
            .iter()
            .map(|m| k0 * k0 - *m * kx_eff2)
            .collect();
        for i in 1..ni - 1 {
            for j in 1..nr - 1 {
                let v = grid[i][j].1;
                if v.is_finite()
                    && (i - 1..=i + 1).all(|a| (j - 1..=j + 1).all(|b| v <= grid[a][b].1))
                {
                    cand.push(grid[i][j].0);
                }
            }
        }
        let mut roots: Vec<C> = Vec::new();
        for s0 in cand {
            let mut s = s0;
            let mut ok = false;
            for _ in 0..80 {
                let gv = g(s);
                let h = 1e-7 * s.norm().max(1.0);
                let d = (g(s + h) - gv) / h;
                if d.norm() <= 0.0 || !d.is_finite() || !gv.is_finite() {
                    break;
                }
                let step = gv / d;
                s -= step;
                if step.norm() < 1e-11 * s.norm().max(1.0) {
                    ok = g(s).norm() < 1e-6;
                    break;
                }
            }
            if ok
                && roots
                    .iter()
                    .all(|r| (s - r).norm() > 1e-6 * r.norm().max(1.0))
            {
                roots.push(s);
            }
        }
        // (kr, kx), the least attenuated first
        let mut modes: Vec<(C, C)> = roots
            .iter()
            .map(|s| {
                let kr = s.sqrt();
                let mut kx = (k0 * k0 - kr * kr).sqrt();
                if kx.im > 0.0 || (kx.im == 0.0 && kx.re < 0.0) {
                    kx = -kx;
                }
                (kr, kx)
            })
            .collect();
        modes.sort_by(|a, b| b.1.im.total_cmp(&a.1.im));
        modes.truncate(N_CHAMBER);
        let nc = modes.len();
        if nc == 0 {
            return None;
        }

        // radial quadrature
        let ra: Vec<f64> = (0..N_RADIAL)
            .map(|i| r1 * i as f64 / (N_RADIAL - 1) as f64)
            .collect();
        let rb: Vec<f64> = (0..N_RADIAL)
            .map(|i| r1 + (r2 - r1) * i as f64 / (N_RADIAL - 1) as f64)
            .collect();
        let psi = |kr: C| {
            let (kq, q, b1) = chamber(kr);
            let c3 = (rhot * kr / (rho0 * kq)) * bessel01(kr * r1)[1] / (b1[1] - q * b1[3]);
            let pa: Vec<C> = ra.iter().map(|&r| j0(kr * r)).collect();
            let pb: Vec<C> = rb
                .iter()
                .map(|&r| {
                    let b = bessel01(kq * r);
                    c3 * (b[0] - q * b[2])
                })
                .collect();
            (pa, pb)
        };
        let alpha: Vec<f64> = std::iter::once(0.0)
            .chain(J1_ZEROS.iter().map(|z| z / r1))
            .collect();
        let phi_a: Vec<Vec<C>> = alpha
            .iter()
            .map(|&a| ra.iter().map(|&r| j0(C::new(a * r, 0.0))).collect())
            .collect();
        let kx_a: Vec<C> = alpha
            .iter()
            .map(|&a| {
                let k = C::new(k0 * k0 - a * a, 0.0).sqrt();
                if k.im > 0.0 { -k } else { k }
            })
            .collect();
        let pc: Vec<(Vec<C>, Vec<C>)> = modes.iter().map(|m| psi(m.0)).collect();
        let kx_c: Vec<C> = modes.iter().map(|m| m.1).collect();
        let prod = |a: &[C], b: &[C]| -> Vec<C> { a.iter().zip(b).map(|(x, y)| x * y).collect() };
        let na = N_AIRWAY;
        let m_pa: Vec<Vec<C>> = (0..na)
            .map(|m| {
                (0..nc)
                    .map(|n| trapezoid(&prod(&pc[n].0, &phi_a[m]), &ra))
                    .collect()
            })
            .collect();
        let n_a: Vec<C> = (0..na)
            .map(|m| trapezoid(&prod(&phi_a[m], &phi_a[m]), &ra))
            .collect();
        let u_c: Vec<Vec<C>> = (0..nc)
            .map(|m| {
                (0..nc)
                    .map(|n| {
                        trapezoid(&prod(&pc[n].0, &pc[m].0), &ra) / rho0
                            + trapezoid(&prod(&pc[n].1, &pc[m].1), &rb) / rhot
                    })
                    .collect()
            })
            .collect();
        let u_a: Vec<Vec<C>> = (0..nc)
            .map(|m| {
                (0..na)
                    .map(|q| trapezoid(&prod(&phi_a[q], &pc[m].0), &ra) / rho0)
                    .collect()
            })
            .collect();
        // unknowns: A− (na), C+ (nc), D (nc), E+ (na); the plane wave A+ = 1 comes in. The
        // backward chamber waves are referred to x = L (C−·e^{jkx(x−L)}, D = C−·e^{jkxL}) so no
        // evanescent mode's factor exceeds 1 (|e^{−jkxL}| ≤ 1 for Im kx ≤ 0)
        let size = 2 * na + 2 * nc;
        let zero = C::new(0.0, 0.0);
        let mut mx = vec![vec![zero; size]; size];
        let mut b = vec![zero; size];
        let el: Vec<C> = kx_c.iter().map(|k| (-C::i() * k * len).exp()).collect();
        let (cp, cm, ep) = (na, na + nc, na + 2 * nc);
        let mut row = 0;
        // x = 0, pressure on φA,m: Σ(C+ + D·e^{−jkL})<ψC,φA> = (A+ + A−)N_A
        for m in 0..na {
            mx[row][m] = -n_a[m];
            for n in 0..nc {
                mx[row][cp + n] = m_pa[m][n];
                mx[row][cm + n] = m_pa[m][n] * el[n];
            }
            if m == 0 {
                b[row] = n_a[0];
            }
            row += 1;
        }
        // x = 0, velocity on ψC,m: Σ kxC(C+ − D·e^{−jkL})U_C = Σ kxA(A+ − A−)U_A
        for m in 0..nc {
            for n in 0..nc {
                mx[row][cp + n] = kx_c[n] * u_c[m][n];
                mx[row][cm + n] = -kx_c[n] * u_c[m][n] * el[n];
            }
            for q in 0..na {
                mx[row][q] = kx_a[q] * u_a[m][q];
            }
            b[row] = kx_a[0] * u_a[m][0];
            row += 1;
        }
        // x = L, pressure: Σ(C+e^{−jkL} + D)<ψC,φA> = E+ N_A
        for m in 0..na {
            for n in 0..nc {
                mx[row][cp + n] = m_pa[m][n] * el[n];
                mx[row][cm + n] = m_pa[m][n];
            }
            mx[row][ep + m] = -n_a[m];
            row += 1;
        }
        // x = L, velocity: Σ kxC(C+e^{−jkL} − D)U_C = Σ kxA E+ U_A
        for m in 0..nc {
            for n in 0..nc {
                mx[row][cp + n] = kx_c[n] * u_c[m][n] * el[n];
                mx[row][cm + n] = -kx_c[n] * u_c[m][n];
            }
            for q in 0..na {
                mx[row][ep + q] = -kx_a[q] * u_a[m][q];
            }
            row += 1;
        }
        let sol = solve(mx, b)?;
        let tl = -20.0 * sol[ep].norm().log10();
        tl.is_finite().then_some(tl)
    }
}

/// A biquad section, transposed direct form II.
#[derive(Debug, Clone, PartialEq)]
pub struct Biquad {
    b: [f64; 3],
    a: [f64; 2],
    z: [f64; 2],
}

impl Biquad {
    /// A peaking filter of `gain_db` at `fc` Hz, quality `q`, at `fs` (Bristow-Johnson,
    /// "Cookbook formulae for audio EQ biquad filter coefficients").
    pub fn peaking(fc: f64, gain_db: f64, q: f64, fs: f64) -> Self {
        let a = 10f64.powf(gain_db / 40.0);
        let w0 = 2.0 * std::f64::consts::PI * fc / fs;
        let alpha = w0.sin() / (2.0 * q);
        let a0 = 1.0 + alpha / a;
        Self {
            b: [
                (1.0 + alpha * a) / a0,
                -2.0 * w0.cos() / a0,
                (1.0 - alpha * a) / a0,
            ],
            a: [-2.0 * w0.cos() / a0, (1.0 - alpha / a) / a0],
            z: [0.0; 2],
        }
    }

    pub fn process(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.z[0];
        self.z[0] = self.b[1] * x - self.a[0] * y + self.z[1];
        self.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }

    /// Magnitude (dB) at `f` Hz.
    pub fn response_db(&self, f: f64, fs: f64) -> f64 {
        let z = (-C::i() * 2.0 * std::f64::consts::PI * f / fs).exp();
        let num = self.b[0] + self.b[1] * z + self.b[2] * z * z;
        let den = 1.0 + self.a[0] * z + self.a[1] * z * z;
        20.0 * (num / den).norm().log10()
    }
}

/// Biquads in series.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Cascade(pub Vec<Biquad>);

impl Cascade {
    pub fn process(&mut self, x: f64) -> f64 {
        self.0.iter_mut().fold(x, |v, b| b.process(v))
    }
    pub fn response_db(&self, f: f64, fs: f64) -> f64 {
        self.0.iter().map(|b| b.response_db(f, fs)).sum()
    }
}

/// Half-octave band centres of the graphic equalizer, 62.5 Hz–16 kHz, below 0.45·`fs`: a
/// packed silencer's loss climbs ~13 dB per half octave, more than octave bands follow.
pub fn eq_centres(fs: f64) -> Vec<f64> {
    (0..17)
        .map(|k| 62.5 * 2f64.powf(0.5 * k as f64))
        .filter(|&f| f < 0.45 * fs)
        .collect()
}

/// Where the equalizer is fitted: the centres and the quarter-octave points between them.
pub fn eq_points(fs: f64) -> Vec<f64> {
    let c = eq_centres(fs);
    (0..2 * c.len() - 1)
        .map(|i| c[0] * 2f64.powf(0.25 * i as f64))
        .collect()
}

/// Bandwidth of each band: Välimäki & Liski's octave design takes 1.5 × the centre; halved
/// for half-octave bands, Q = 1/0.75.
const EQ_Q: f64 = 1.0 / 0.75;
/// Their prototype gain, dB.
const EQ_PROTO_DB: f64 = 17.0;
/// Deepest transmission loss the equalizer follows, dB.
const MAX_EQ_CUT_DB: f64 = 50.0;

/// Least squares of `b·g ≈ t` (rows = points, cols = bands) by the normal equations.
fn least_squares(b: &[Vec<f64>], t: &[f64]) -> Option<Vec<f64>> {
    let n = b[0].len();
    let m: Vec<Vec<C>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| C::new(b.iter().map(|r| r[i] * r[j]).sum(), 0.0))
                .collect()
        })
        .collect();
    let v: Vec<C> = (0..n)
        .map(|i| C::new(b.iter().zip(t).map(|(r, y)| r[i] * y).sum(), 0.0))
        .collect();
    solve(m, v).map(|x| x.iter().map(|c| c.re).collect())
}

/// A cascade of octave peaking filters whose response follows `target_db` at [`eq_points`]
/// (Välimäki & Liski, "Accurate cascade graphic equalizer", IEEE SPL 24(2), 2017: the bands'
/// interaction matrix at a prototype gain, least squares, then once more with each band's
/// interaction at the gain found).
pub fn graphic_eq(target_db: &[f64], fs: f64) -> Cascade {
    let centres = eq_centres(fs);
    let points = eq_points(fs);
    let interaction = |gains: &[f64]| -> Vec<Vec<f64>> {
        points
            .iter()
            .map(|&f| {
                centres
                    .iter()
                    .zip(gains)
                    .map(|(&fc, &g)| Biquad::peaking(fc, g, EQ_Q, fs).response_db(f, fs) / g)
                    .collect()
            })
            .collect()
    };
    let proto = vec![EQ_PROTO_DB; centres.len()];
    let mut gains =
        least_squares(&interaction(&proto), target_db).unwrap_or(vec![0.0; centres.len()]);
    // a band left near 0 dB keeps the prototype's shape in the second pass
    let refined: Vec<f64> = gains
        .iter()
        .map(|&g| if g.abs() < 0.1 { EQ_PROTO_DB } else { g })
        .collect();
    if let Some(g2) = least_squares(&interaction(&refined), target_db) {
        gains = g2;
    }
    Cascade(
        centres
            .iter()
            .zip(&gains)
            .map(|(&fc, &g)| Biquad::peaking(fc, g, EQ_Q, fs))
            .collect(),
    )
}

/// Silencer equalizers already designed, by their silencer, gas and rate (each design solves the
/// mode matching at every band: most of a second on a big silencer). The design is a pure
/// function of those, so a remembered one is the same, bit for bit; an engine rebuilt after an
/// edit that leaves a silencer alone takes it from here instead of solving it again.
#[derive(Debug, Clone, Default)]
pub struct EqMemo(Vec<([u64; 10], Cascade)>);

/// How many designs a memo keeps (the oldest goes first): a few engines' silencers.
const MEMO_KEEP: usize = 32;

impl EqMemo {
    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl Silencer {
    /// [`Silencer::transmission_eq`] through `memo`.
    pub fn transmission_eq_in(&self, gas: Gas, fs: f64, memo: &mut EqMemo) -> Cascade {
        let key = [
            self.r1,
            self.r2,
            self.len,
            self.resistivity.unwrap_or(-1.0),
            self.perforate.porosity,
            self.perforate.wall_m,
            self.perforate.hole_m,
            gas.c,
            gas.rho,
            fs,
        ]
        .map(f64::to_bits);
        if let Some((_, eq)) = memo.0.iter().find(|(k, _)| *k == key) {
            return eq.clone();
        }
        let eq = self.transmission_eq(gas, fs);
        if memo.0.len() >= MEMO_KEEP {
            memo.0.remove(0);
        }
        memo.0.push((key, eq.clone()));
        eq
    }

    /// The silencer's transmission as an equalizer at `fs`: −TL at [`eq_points`] in `gas`
    /// (a point the matching fails on takes its neighbour's).
    pub fn transmission_eq(&self, gas: Gas, fs: f64) -> Cascade {
        let mut t: Vec<Option<f64>> = eq_points(fs)
            .iter()
            .map(|&f| self.transmission_loss_db(f, gas).map(|tl| -tl))
            .collect();
        for i in 1..t.len() {
            if t[i].is_none() {
                t[i] = t[i - 1];
            }
        }
        for i in (0..t.len().saturating_sub(1)).rev() {
            if t[i].is_none() {
                t[i] = t[i + 1];
            }
        }
        loss_eq(t.iter().map(|v| v.unwrap_or(0.0)).collect(), fs)
    }
}

/// The equalizer of a transmission `target_db` (≤ 0) at [`eq_points`]: octave bands cannot
/// follow a notch deeper than ~50 dB without spoiling its neighbours, and a wave 50 dB down is
/// gone against the rest of the sound. A silencer gives no energy: where the fit overshoots
/// 0 dB (between bands, or below the lowest) the target is lowered by the excess and fitted
/// again.
fn loss_eq(target_db: Vec<f64>, fs: f64) -> Cascade {
    let mut t: Vec<f64> = target_db
        .iter()
        .map(|v| v.clamp(-MAX_EQ_CUT_DB, 0.0))
        .collect();
    let check: Vec<f64> = (0..=200)
        .map(|i| 5.0 * 9000f64.powf(i as f64 / 200.0))
        .filter(|&f| f < 0.45 * fs)
        .collect();
    let mut eq = graphic_eq(&t, fs);
    for _ in 0..6 {
        let over = check
            .iter()
            .map(|&f| eq.response_db(f, fs))
            .fold(f64::MIN, f64::max);
        if over <= 0.0 {
            break;
        }
        let pts = eq_points(fs);
        for (ti, f) in t.iter_mut().zip(&pts) {
            *ti -= eq.response_db(*f, fs).max(0.0) + 0.5 * over;
        }
        eq = graphic_eq(&t, fs);
    }
    eq
}

/// A published transmission loss, `points` of (frequency Hz, dB) in rising frequency, as an
/// equalizer at `fs` ([`loss_eq`]): linear in log-frequency between points; below the first
/// the loss falls in proportion to frequency to 0 at 0 Hz (a chamber is a compliance there:
/// its loss grows with ω·V/(2·c·S), Munjal, *Acoustics of Ducts and Mufflers*, ch. 2); above
/// the last it holds. No points: no loss.
pub fn measured_eq(points: &[[f64; 2]], fs: f64) -> Cascade {
    let target: Vec<f64> = eq_points(fs)
        .iter()
        .map(|&f| -measured_loss_db(points, f))
        .collect();
    loss_eq(target, fs)
}

/// The loss of `points` ([`measured_eq`]) at `f` Hz.
pub fn measured_loss_db(points: &[[f64; 2]], f: f64) -> f64 {
    let (Some(first), Some(last)) = (points.first(), points.last()) else {
        return 0.0;
    };
    if f <= first[0] {
        return if first[0] > 0.0 {
            first[1] * f / first[0]
        } else {
            first[1]
        };
    }
    if f >= last[0] {
        return last[1];
    }
    for w in points.windows(2) {
        let ([f0, l0], [f1, l1]) = (w[0], w[1]);
        if f >= f0 && f <= f1 {
            if f1 <= f0 || f0 <= 0.0 {
                return l1;
            }
            let x = (f / f0).ln() / (f1 / f0).ln();
            return l0 + (l1 - l0) * x;
        }
    }
    last[1]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// JSV 262's silencer: d1 = 49 mm, d2 = 164.4 mm, L = 257.2 mm, in air.
    fn jsv262(resistivity: Option<f64>) -> Silencer {
        Silencer {
            r1: 0.049 / 2.0,
            r2: 0.1644 / 2.0,
            len: 0.2572,
            resistivity,
            perforate: Perforate::default(),
        }
    }
    const AIR: Gas = Gas { c: 343.0, rho: 1.2 };

    #[test]
    fn a_designed_silencer_is_remembered_and_the_same() {
        let (s, fs) = (jsv262(Some(16_000.0)), 48_000.0);
        let fresh = s.transmission_eq(AIR, fs);
        let mut memo = EqMemo::default();
        assert_eq!(s.transmission_eq_in(AIR, fs, &mut memo), fresh);
        assert_eq!(memo.len(), 1);
        assert_eq!(
            s.transmission_eq_in(AIR, fs, &mut memo),
            fresh,
            "remembered"
        );
        assert_eq!(memo.len(), 1, "not designed again");
        // another rate, gas or silencer is another design
        let other = s.transmission_eq_in(AIR, 96_000.0, &mut memo);
        assert_ne!(other, fresh);
        let warm = Gas { c: 500.0, rho: 0.6 };
        assert_ne!(s.transmission_eq_in(warm, fs, &mut memo), fresh);
        let longer = Silencer { len: 0.3, ..s };
        assert_ne!(longer.transmission_eq_in(AIR, fs, &mut memo), fresh);
        assert_eq!(memo.len(), 4);
    }

    #[test]
    fn an_empty_chamber_passes_where_it_is_whole_half_waves_long() {
        // An expansion chamber of area ratio m = (164.4/49)² = 11.257: TL = 10·log10(1 +
        // ¼(m − 1/m)²·sin²kL) (Munjal ch. 3) is 0 at kL = π, f = c/2L = 666.8 Hz, and 15.0 dB
        // at kL = π/2 (333.4 Hz). The perforate's mass shifts the zero a little up (JSV 262
        // Fig. 6 measures it near 680 Hz).
        let s = jsv262(None);
        let zero = (640..=720)
            .step_by(10)
            .map(|f| s.transmission_loss_db(f as f64, AIR).unwrap())
            .fold(f64::INFINITY, f64::min);
        assert!(zero < 2.0, "dip {zero:.1} dB");
        let dome = s.transmission_loss_db(333.4, AIR).unwrap();
        assert!((dome - 15.0).abs() < 1.5, "dome {dome:.1} dB");
    }

    #[test]
    fn a_packed_chamber_matches_the_measured_one() {
        // JSV 262 Fig. 7 (ρ_fibre 100 g/l, R = 4896 rayl/m), read off: 21, 29, 36, 30, 22, 18 dB
        // at 0.5–3 kHz. The 1D lining gave 3.7 … 62.7 here.
        let s = jsv262(Some(4896.0));
        for (f, want) in [
            (500.0, 21.0),
            (1000.0, 29.0),
            (1500.0, 36.0),
            (2000.0, 30.0),
            (2500.0, 22.0),
            (3000.0, 18.0),
        ] {
            let got = s.transmission_loss_db(f, AIR).unwrap();
            assert!(
                (got - want).abs() < 7.5,
                "{f} Hz: {got:.1} dB, measured {want}"
            );
        }
    }
    #[test]
    fn the_equalizer_follows_a_silencers_dome() {
        // a dome of transmission loss like a packed silencer's: 0 dB low, 35 dB at 2 kHz,
        // back to 5 dB at 16 kHz — the cascade within 1.5 dB at every fitted point
        let fs = 96_000.0;
        let pts = eq_points(fs);
        let t: Vec<f64> = pts
            .iter()
            .map(|&f| {
                let x = (f / 2000.0f64).log2();
                -35.0 * (-x * x / 4.0).exp()
            })
            .collect();
        let eq = graphic_eq(&t, fs);
        for (f, want) in pts.iter().zip(&t) {
            let got = eq.response_db(*f, fs);
            assert!(
                (got - want).abs() < 1.5,
                "{f:.0} Hz: {got:.1} dB, want {want:.1}"
            );
        }
    }

    #[test]
    fn a_silencers_equalizer_never_amplifies() {
        // passive: |H| ≤ 1 at every frequency, here for the single's packed can (0.4 m, Ø97.7
        // over 47.7 mm, 16 kPa·s/m², tail gas), whose plain fit rose 1.6 dB at 62 Hz
        let gas = Gas {
            c: 500.0,
            rho: 0.567_42,
        };
        let s = Silencer {
            r1: 0.5 * 0.047_720_5,
            r2: 0.5 * 0.097_720_5,
            len: 0.4,
            resistivity: Some(16_000.0),
            perforate: Perforate::default(),
        };
        let fs = 96_000.0;
        let eq = s.transmission_eq(gas, fs);
        for i in 0..=400 {
            let f = 5.0 * 9000f64.powf(i as f64 / 400.0);
            let r = eq.response_db(f, fs);
            assert!(r <= 1e-6, "{f:.0} Hz: {r:.2} dB");
        }
    }

    #[test]
    fn a_peaking_biquad_has_its_gain_at_its_centre() {
        // RBJ: |H(f0)| = A² = 10^(G/20) exactly; and at DC 1
        let b = Biquad::peaking(1000.0, -12.0, 0.7, 48_000.0);
        assert!((b.response_db(1000.0, 48_000.0) + 12.0).abs() < 1e-9);
        assert!(b.response_db(0.0, 48_000.0).abs() < 1e-9);
    }
}
