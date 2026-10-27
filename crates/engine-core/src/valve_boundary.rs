//! The exhaust valve as a boundary of its pipe, solved by characteristics (Benson, *The
//! Thermodynamics and Gas Dynamics of Internal Combustion Engines*, vol. 1, §4.1 and §6.6).
//!
//! A linear port (p = p₀ + 2p⁻ + (c/A)·ṁ) overstates the pressure a Mach-0.7 blowdown raises in
//! its pipe. Here the pipe sends in one characteristic, λ_in, and the valve answers:
//!
//! - **outflow** by Jenny's constant-pressure model: isentropic from the cylinder to the valve
//!   throat, then an irreversible expansion at constant pressure (subsonic throat) or a pressure
//!   drop (choked throat) to fill the pipe (eqs. 6.43–6.62);
//! - **inflow** as a partially open end, scaled to the cylinder's pressure (eqs. 6.33–6.42).
//!
//! The reference state is the cylinder's (p_ref = p_c), so π_α = λ_in.
//! λ_in = (p/p_c)^((κ−1)/2κ)·(1 − (κ−1)/2·u/a), from the pipe's first cell with u positive away
//! from the valve (the pressure form of the incoming Riemann variable).
//!
//! κ is the pipes' (1.4: they hold cold-air-γ gas); the cylinder's real gas enters through its
//! stagnation pressure and speed of sound.

const KAPPA: f64 = crate::gas::GAMMA;
/// (κ − 1)/2.
const KM: f64 = (KAPPA - 1.0) / 2.0;
/// 2κ/(κ − 1): p/p_c = π^P_EXP (7 for κ = 1.4).
const P_EXP: i32 = 7;
/// S = √(2/(κ+1)): π (and the area-ratio variable) at the sonic boundary (6.48), (6.53).
const S: f64 = 0.912_870_929_175_276_9;
/// Below this valve-to-pipe area ratio the valve is closed: a lift that small passes under
/// 1e-10 kg/s, and the sonic root's distance from π_α falls below f64's resolution.
const PSI_MIN: f64 = 1e-9;
/// (2/(κ+1))^((κ+1)/(2(κ−1))) for κ = 1.4: 5/6 cubed, 0.578 703…
const SONIC_FLOW: f64 = 0.578_703_703_703_703_7;

/// The boundary state at the valve end of the pipe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Boundary {
    /// Pressure, Pa.
    pub p: f64,
    /// Velocity, m/s, positive into the pipe (outflow from the cylinder).
    pub u: f64,
    /// Speed of sound, m/s (its entropy level: the blowdown gas's on outflow, the pipe's on
    /// inflow).
    pub a: f64,
}

impl Boundary {
    /// Mass flow into a pipe of cross-section `area`, kg/s: ρ·u·F with ρ = κp/a².
    pub fn mass_flow(&self, area: f64) -> f64 {
        KAPPA * self.p / (self.a * self.a) * self.u * area
    }
}

/// The incoming Riemann variable at the valve, in the pressure form referred to the cylinder.
/// `p`, `u` (positive away from the valve) and `a` are the pipe's first cell.
pub fn lambda_in(p_cyl: f64, p: f64, u: f64, a: f64) -> f64 {
    (p / p_cyl).powf(1.0 / f64::from(P_EXP)) * (1.0 - KM * u / a)
}

/// Solve the valve boundary: cylinder stagnation pressure `p_cyl` and speed of sound `a_cyl`,
/// valve-to-pipe effective area ratio `psi` (capped at 1), incoming `lambda` ([`lambda_in`])
/// and the speed of sound of the pipe's own gas brought to the cylinder's pressure,
/// `a_pipe_at_pc` (its entropy level, what flows back in).
pub fn solve(p_cyl: f64, a_cyl: f64, psi: f64, lambda: f64, a_pipe_at_pc: f64) -> Boundary {
    let mut none = f64::NAN;
    solve_with(p_cyl, a_cyl, psi, lambda, a_pipe_at_pc, &mut none)
}

/// [`solve`] started from `hint`, the last step's root (π on outflow, A on inflow; NaN for
/// none), which it updates: between two physics steps the boundary barely moves, so a secant
/// from there converges in two or three evaluations instead of a dozen.
pub fn solve_with(
    p_cyl: f64,
    a_cyl: f64,
    psi: f64,
    lambda: f64,
    a_pipe_at_pc: f64,
    hint: &mut f64,
) -> Boundary {
    let psi = psi.clamp(0.0, 1.0);
    if psi < PSI_MIN {
        // a closed valve reflects: u = 0, A = λ_in
        return Boundary {
            p: p_cyl * lambda.powi(P_EXP),
            u: 0.0,
            a: a_pipe_at_pc * lambda,
        };
    }
    if lambda < 1.0 {
        let (pi, minus_ub, c) = outflow(psi, lambda, hint);
        Boundary {
            p: p_cyl * pi.powi(P_EXP),
            u: minus_ub * a_cyl,
            a: c * a_cyl * pi,
        }
    } else if lambda > 1.0 {
        let (a, u) = inflow(psi, lambda, hint);
        Boundary {
            p: p_cyl * a.powi(P_EXP),
            u: -u * a_pipe_at_pc,
            a: a * a_pipe_at_pc,
        }
    } else {
        Boundary {
            p: p_cyl,
            u: 0.0,
            a: a_pipe_at_pc,
        }
    }
}

/// The incoming Riemann variable of a linear wave `arriving` (gauge Pa, travelling towards the
/// valve) on a pipe of gas at rest at `t_pipe` K and pressure `p0` (the atmosphere plus the
/// tail's mean backpressure), as a simple wave: p = p₀ + p⁻, u = −p⁻/(ρc) (towards the valve).
pub fn lambda_of_wave(p_cyl: f64, arriving: f64, t_pipe: f64, p0: f64) -> f64 {
    let a = (KAPPA * crate::gas::R_AIR * t_pipe).sqrt();
    let rho = p0 / (crate::gas::R_AIR * t_pipe);
    let p = (p0 + arriving).max(0.05 * p0);
    lambda_in(p_cyl, p, -arriving / (rho * a), a)
}

/// The pipe at the valve, as the boundary needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pipe {
    /// Cross-section, m².
    pub area: f64,
    /// Incoming Riemann variable ([`lambda_in`] or [`lambda_of_wave`]).
    pub lambda: f64,
    /// Temperature of its gas at the valve, K (what flows back).
    pub t_back: f64,
}

/// One step of flow through an exhaust valve of effective area `cd_area` into a pipe of
/// cross-section `pipe_area`, by the characteristic boundary: the pipe's incoming `lambda`
/// ([`lambda_in`] or [`lambda_of_wave`]) and its gas at `t_back` K (what flows back). The step
/// is limited, as [`crate::gas::exhaust_valve_flow`]'s, to `relax` of the mass that would
/// equalise the cylinder with the boundary and to half the cylinder's mass. + = out.
pub fn valve_flow(
    gas: &crate::gas::GasVolume,
    cd_area: f64,
    pipe: Pipe,
    dt: f64,
    relax: f64,
    hint: &mut f64,
) -> (crate::gas::Flow, Boundary) {
    use crate::gas::{CP, GAMMA, R_AIR};
    let Pipe {
        area: pipe_area,
        lambda,
        t_back,
    } = pipe;
    let (p_c, t_c) = (gas.pressure(), gas.temperature());
    if cd_area <= 0.0 || pipe_area <= 0.0 {
        let closed = Boundary {
            p: p_c,
            u: 0.0,
            a: (KAPPA * R_AIR * t_c).sqrt(),
        };
        return (
            crate::gas::Flow {
                dm: 0.0,
                h: CP * t_c,
            },
            closed,
        );
    }
    let a_c = (KAPPA * R_AIR * t_c).sqrt();
    // the pipe gas's speed of sound brought isentropically to the cylinder's pressure, from
    // its state on the arriving characteristic (λ is (p/p_c)^(1/7) at rest)
    let a_back = (KAPPA * R_AIR * t_back).sqrt() / lambda.max(1e-3);
    let b = solve_with(p_c, a_c, cd_area / pipe_area, lambda, a_back, hint);
    let mdot = b.mass_flow(pipe_area);
    let h = if mdot >= 0.0 { CP * t_c } else { CP * t_back };
    let limit = relax * (p_c - b.p).abs() * gas.volume_m3 / ((GAMMA - 1.0) * h);
    let mut dm = (mdot * dt).clamp(-limit, limit);
    if dm > 0.0 {
        dm = dm.min(0.5 * gas.mass_kg);
    }
    (crate::gas::Flow { dm, h }, b)
}

/// Bracket width the roots stop at, relative: π^7 is the pressure, so 1e-10 in π is 7e-10 in p.
const ROOT_TOL: f64 = 1e-10;

/// Root of `f` in [lo, hi] by secant from `guess` (the last step's root); [`root`] if the guess
/// is outside the bracket or the secant leaves it.
fn root_from(f: impl Fn(f64) -> f64, lo: f64, hi: f64, guess: f64) -> f64 {
    if !(guess > lo && guess < hi) {
        return root(f, lo, hi);
    }
    let mut x0 = guess;
    let mut f0 = f(x0);
    // the second point a hair inside the bracket from the guess
    let mut x1 = (guess + 1e-7 * (hi - lo)).min(hi);
    let mut f1 = f(x1);
    for _ in 0..8 {
        if f1 == f0 {
            break;
        }
        let x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
        if !(x2 >= lo && x2 <= hi) {
            break;
        }
        if (x2 - x1).abs() < ROOT_TOL * x2.abs().max(1.0) {
            return x2;
        }
        (x0, f0) = (x1, f1);
        x1 = x2;
        f1 = f(x1);
    }
    root(f, lo, hi)
}

/// Root of `f` in [lo, hi] where f(lo) and f(hi) have opposite signs: Illinois regula falsi.
fn root(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    let (mut flo, mut fhi) = (f(lo), f(hi));
    if flo == 0.0 {
        return lo;
    }
    if fhi == 0.0 {
        return hi;
    }
    if (flo > 0.0) == (fhi > 0.0) {
        // no sign change (a root at an end, met in round-off): the nearer end
        return if flo.abs() < fhi.abs() { lo } else { hi };
    }
    let mut side = 0;
    let mut last = f64::NAN;
    for _ in 0..100 {
        let x = (lo * fhi - hi * flo) / (fhi - flo);
        // converged when the estimate stops moving (Illinois keeps one end fixed for a while,
        // so the bracket itself need not shrink)
        if (x - last).abs() < ROOT_TOL * x.abs().max(1.0) {
            return x;
        }
        last = x;
        let fx = f(x);
        if fx == 0.0 {
            return x;
        }
        if (fx > 0.0) == (flo > 0.0) {
            lo = x;
            flo = fx;
            if side == -1 {
                fhi *= 0.5;
            }
            side = -1;
        } else {
            hi = x;
            fhi = fx;
            if side == 1 {
                flo *= 0.5;
            }
            side = 1;
        }
    }
    0.5 * (lo + hi)
}

/// Outflow through the valve (λ_in = π_α < 1): returns (π, −U_B, C), with π = (p/p_c)^(1/7),
/// −U_B = u/a_c into the pipe and C = a_A2/a_A1 the entropy rise across the valve.
fn outflow(psi: f64, pi_a: f64, hint: &mut f64) -> (f64, f64, f64) {
    // Benson finds C by successive substitution (6.61), solving (6.47) or (6.51) each time. It
    // has a closed form in π: with d = π − π_α, B = C·d and −U_B = B/KM, (6.56) reads
    // C²π² = 1 − C²d²/KM, so C = 1/√(π² + d²/KM), and 1 − B²/KM = C²π² (never negative: no
    // pole). One equation in π remains:
    let c_of = |p: f64| {
        let d = p - pi_a;
        1.0 / (p * p + d * d / KM).sqrt()
    };
    // (6.47), subsonic throat: ψ/π·{(1/π² − 1)/KM}^½ − (B/KM)/(1 − B²/KM) = 0
    let f1 = |p: f64| {
        let c = c_of(p);
        psi / p * ((1.0 / (p * p) - 1.0) / KM).max(0.0).sqrt() - (p - pi_a) / (KM * c * p * p)
    };
    // f₁ falls with π; the throat is subsonic when its root lies at or above the sonic π = S
    let pi = if pi_a >= S || f1(S) > 0.0 {
        root_from(f1, pi_a.max(S), 1.0, *hint)
    } else {
        // (6.51), choked throat: π^(2κ/(κ−1)) − E·(1 − B²/KM)/(B/KM) = 0, i.e.
        // π⁷ − E·KM·C·π²/d = 0, which falls to −∞ as π → π_α
        let e = psi * SONIC_FLOW;
        let f2 = |p: f64| {
            let d = p - pi_a;
            p.powi(P_EXP) - e * KM * c_of(p) * p * p / d
        };
        let mut lo = pi_a + e * KM;
        while lo > pi_a && lo < S && f2(lo) >= 0.0 {
            lo = pi_a + 0.5 * (lo - pi_a);
        }
        if lo <= pi_a {
            // ψ so small the root is within round-off of π_α: no flow worth the name
            return (pi_a, 0.0, 1.0 / pi_a);
        }
        root_from(f2, lo.min(S), S, *hint)
    };
    *hint = pi;
    let c = c_of(pi);
    (pi, c * (pi - pi_a) / KM, c)
}

/// Inflow from the pipe (λ_in > 1): the partially open end with the cylinder as its receiver.
/// Returns (A, U): A = (p/p_c)^(1/7) and U the velocity toward the cylinder over the pipe gas's
/// speed of sound at p_c.
fn inflow(phi: f64, lambda: f64, hint: &mut f64) -> (f64, f64) {
    // subsonic (6.35): (A^(4/(κ−1)) − φ²)(λ − A)² − (κ−1)/2·(A² − 1)φ² = 0, A in [1, λ]
    let f = |a: f64| {
        let a10 = a.powi(2 * P_EXP - 4);
        (a10 - phi * phi) * (lambda - a) * (lambda - a) - KM * (a * a - 1.0) * phi * phi
    };
    let a = root_from(f, 1.0, lambda, *hint);
    *hint = a;
    let u = (lambda - a) / KM;
    // sonic throat test (6.37): U_t = A^(2/(κ−1))·U/φ
    if a.powi(P_EXP - 2) * u / phi <= 1.0 {
        return (a, u);
    }
    // sonic (6.38): φ² − {(κ+1)/(κ−1) − 2/(κ−1)·y²}·y^(4/(κ−1)) = 0 for y = (A/A_t)_cr,
    // solved, as Benson does, for its inverse x = (A_t/A)_cr between √(2/(κ+1)) (φ = 0) and 1
    let g = |x: f64| {
        let y = 1.0 / x;
        phi * phi - ((KAPPA + 1.0) / (KAPPA - 1.0) - y * y / KM) * y.powi(2 * P_EXP - 4)
    };
    let x = root(g, S, 1.0);
    // (6.39)–(6.41): λ_out = K_s·λ_in with C' = x^((κ+1)/(κ−1))
    let c = x.powi(P_EXP - 1);
    let ks = (1.0 - KM * phi * c) / (1.0 + KM * phi * c);
    let a = 0.5 * lambda * (1.0 + ks);
    (a, (lambda - a) / KM)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const R: f64 = 287.0;

    /// Isentropic nozzle mass flow, kg/s (Heywood, *ICE Fundamentals*, App. C, C.8–C.9).
    fn nozzle(area: f64, p0: f64, t0: f64, p_down: f64) -> f64 {
        let k = KAPPA;
        let pr = p_down / p0;
        let base = area * p0 / (R * t0).sqrt();
        if pr <= (2.0 / (k + 1.0)).powf(k / (k - 1.0)) {
            base * k.sqrt() * (2.0 / (k + 1.0)).powf((k + 1.0) / (2.0 * (k - 1.0)))
        } else {
            base * pr.powf(1.0 / k) * (2.0 * k / (k - 1.0) * (1.0 - pr.powf((k - 1.0) / k))).sqrt()
        }
    }

    fn sound(t: f64) -> f64 {
        (KAPPA * R * t).sqrt()
    }

    /// A pipe at rest at `p` and `t`, seen from a cylinder at `p_c`.
    fn quiet(p_c: f64, p: f64, t: f64) -> (f64, f64) {
        let a = sound(t);
        (lambda_in(p_c, p, 0.0, a), a * (p_c / p).powf(1.0 / 7.0))
    }

    #[test]
    fn a_small_valve_passes_the_nozzle_flow() {
        // ψ → 0: the pipe barely moves, the valve is a nozzle from the cylinder to the pipe's
        // pressure. Hand values (Heywood App. C, κ = 1.4, R = 287, 1000 K, 1 mm² of throat in a
        // 1000 mm² pipe): 1.5 → 1 bar 1.834e-3 kg/s; 4 → 1 bar (choked) 5.113e-3 kg/s
        let (f_pipe, psi, t) = (1.0e-3, 0.01, 1000.0);
        for (p_c, want) in [(1.5e5, 1.834_03e-3), (4.0e5, 5.112_57e-3)] {
            let (lam, a_back) = quiet(p_c, 1.0e5, t);
            let b = solve(p_c, sound(t), psi, lam, a_back);
            assert_relative_eq!(b.mass_flow(f_pipe), want, max_relative = 0.02);
            // and it is the nozzle formula's own value at the boundary's pressure
            let own = nozzle(psi * f_pipe, p_c, t, b.p);
            assert_relative_eq!(b.mass_flow(f_pipe), own, max_relative = 1e-6);
        }
    }

    /// The outflow model's defining laws at any ψ (Benson eqs. 4.1–4.2): the stagnation
    /// enthalpy holds from cylinder to pipe, and the throat's isentropic flow fills the pipe.
    #[test]
    fn outflow_keeps_energy_continuity_and_the_incoming_characteristic() {
        let t_c = 1300.0;
        let a_c = sound(t_c);
        for (p_c, psi) in [(1.6e5, 0.6), (3.0e5, 0.6), (3.0e5, 1.0), (6.0e5, 0.3)] {
            let (lam, a_back) = quiet(p_c, 1.0e5, 700.0);
            let b = solve(p_c, a_c, psi, lam, a_back);
            assert!(b.u > 0.0);
            assert_relative_eq!(b.a * b.a + KM * b.u * b.u, a_c * a_c, max_relative = 1e-9);
            let f_pipe = 1.0e-3;
            let throat = nozzle(psi * f_pipe, p_c, t_c, b.p);
            assert_relative_eq!(b.mass_flow(f_pipe), throat, max_relative = 1e-6);
            assert_relative_eq!(lambda_in(p_c, b.p, b.u, b.a), lam, max_relative = 1e-9);
        }
    }

    #[test]
    fn inflow_is_the_pipe_gas_passing_a_nozzle_into_the_cylinder() {
        // a wave arriving at a cylinder below the pipe's pressure: the pipe's gas flows in
        // through the valve, isentropic to the throat at the cylinder's pressure (or choked)
        let (p_c, t_pipe) = (0.8e5, 800.0);
        for (p_pipe, psi) in [(1.0e5, 0.5), (1.0e5, 1.0), (2.5e5, 0.2)] {
            let (lam, a_back) = quiet(p_c, p_pipe, t_pipe);
            let b = solve(p_c, sound(1200.0), psi, lam, a_back);
            assert!(b.u < 0.0);
            // the pipe gas keeps its entropy: a/a(p_c) = (p/p_c)^(1/7)
            assert_relative_eq!(
                b.a / a_back,
                (b.p / p_c).powf(1.0 / 7.0),
                max_relative = 1e-9
            );
            // stagnation state of the boundary gas, then a nozzle of ψF into the cylinder
            let t_b = b.a * b.a / (KAPPA * R);
            let t0 = t_b + KM * b.u * b.u / (KAPPA * R);
            let p0 = b.p * (t0 / t_b).powf(KAPPA / (KAPPA - 1.0));
            let f_pipe = 1.0e-3;
            assert_relative_eq!(
                -b.mass_flow(f_pipe),
                nozzle(psi * f_pipe, p0, t0, p_c),
                max_relative = 1e-6
            );
            assert_relative_eq!(lambda_in(p_c, b.p, b.u, b.a), lam, max_relative = 1e-9);
        }
    }

    #[test]
    fn the_flow_is_continuous_through_the_sonic_boundary() {
        // sweep the cylinder pressure across the throat's choking (≈ 1.9 × the pipe's) in 10 Pa
        // steps: the flow rises smoothly, ≈ 1e-4 a step, with no jump where the regime changes
        let (t_c, psi, f_pipe) = (1200.0, 0.5, 1.0e-3);
        let flow = |p_c: f64| {
            let (lam, a_back) = quiet(p_c, 1.0e5, 700.0);
            solve(p_c, sound(t_c), psi, lam, a_back).mass_flow(f_pipe)
        };
        let mut p = 1.5e5;
        let mut prev = flow(p);
        while p < 3.0e5 {
            p += 10.0;
            let m = flow(p);
            assert!(m > prev && (m - prev) / prev < 5e-4, "{p}: {prev} → {m}");
            prev = m;
        }
    }

    #[test]
    fn a_valve_just_cracking_open_is_finite() {
        // the sin² lift starts from nothing: ψ of 1e-26 at the first step of the opening (a
        // 5.4-bar blowdown, seen in the engine) must give a finite, near-closed boundary
        for psi in [4.4e-26, 1e-12, 1e-8, 1e-6] {
            let b = solve(5.4e5, 790.0, psi, 0.786, 456.0);
            assert!(
                b.p.is_finite() && b.u.is_finite() && b.a.is_finite(),
                "{psi}: {b:?}"
            );
            assert!(b.u >= 0.0 && b.u < 1.0, "{psi}: {b:?}");
        }
    }

    #[test]
    fn real_blowdowns_from_the_engine_solve() {
        // states met in the engine (p_c, a_c from t_c, ψ, λ_in, a_back): (6.47)'s second term has
        // a pole at B = √((κ−1)/2), the flow's limit, which a bracket up to π = 1 can straddle
        for (p_c, t_c, psi, lam, a_back) in [
            (192_370.0, 1027.6, 0.426_057, 0.825_318, 662.27),
            (234_722.0, 1108.1, 0.514_417, 0.824_566, 642.28),
            (173_541.0, 968.3, 0.544_547, 0.820_900, 741.15),
        ] {
            let b = solve(p_c, sound(t_c), psi, lam, a_back);
            assert!(b.p.is_finite() && b.u > 0.0, "{b:?}");
            // and it keeps the outflow's laws
            assert_relative_eq!(
                b.a * b.a + KM * b.u * b.u,
                sound(t_c).powi(2),
                max_relative = 1e-9
            );
            assert_relative_eq!(lambda_in(p_c, b.p, b.u, b.a), lam, max_relative = 1e-9);
        }
    }

    #[test]
    fn a_warm_start_lands_on_the_same_boundary() {
        // from a good hint, a stale one or none, the root is the same within the tolerance
        for (lam, psi) in [
            (0.83, 0.5),
            (0.95, 0.2),
            (0.7, 0.9),
            (1.04, 0.6),
            (1.2, 0.1),
        ] {
            let cold = solve(2.0e5, 700.0, psi, lam, 600.0);
            for start in [f64::NAN, 0.5, 0.9, 0.999, 1.02, 3.0] {
                let mut hint = start;
                let warm = solve_with(2.0e5, 700.0, psi, lam, 600.0, &mut hint);
                assert_relative_eq!(warm.p, cold.p, max_relative = 1e-8);
                assert_relative_eq!(warm.u, cold.u, max_relative = 1e-6, epsilon = 1e-9);
            }
        }
    }

    #[test]
    fn illinois_finds_newtons_cubic_root_in_few_evaluations() {
        // x³ − 2x − 5 = 0 (Newton's own example): x = 2.094 551 481 542 33. Illinois regula falsi
        // from [2, 3] converges superlinearly: well under 20 evaluations
        let n = std::cell::Cell::new(0);
        let f = |x: f64| {
            n.set(n.get() + 1);
            x * x * x - 2.0 * x - 5.0
        };
        let x = root(f, 2.0, 3.0);
        assert_relative_eq!(x, 2.094_551_481_542_33, max_relative = 1e-9);
        assert!(n.get() <= 20, "{} evaluations", n.get());
        // the bracket's own ends are roots when they are
        assert_eq!(root(|x: f64| x - 2.0, 2.0, 3.0), 2.0);
        assert_eq!(root(|x: f64| x - 3.0, 2.0, 3.0), 3.0);
        // no sign change: the end nearer zero
        assert_eq!(root(|x: f64| x * x + 1.0, 0.5, 3.0), 0.5);
        assert_eq!(root(|x: f64| 10.0 - x, -5.0, 3.0), 3.0);
    }

    #[test]
    fn a_secant_from_a_good_guess_needs_three_or_four_evaluations() {
        // from a guess 1e-4 off the cubic's root the secant lands in a few steps; a guess outside
        // the bracket falls back to the bracketed solve and still finds it
        let n = std::cell::Cell::new(0);
        let f = |x: f64| {
            n.set(n.get() + 1);
            x * x * x - 2.0 * x - 5.0
        };
        let x = root_from(f, 2.0, 3.0, 2.094_65);
        assert_relative_eq!(x, 2.094_551_481_542_33, max_relative = 1e-9);
        assert!(n.get() <= 5, "{} evaluations", n.get());
        n.set(0);
        let x = root_from(f, 2.0, 3.0, 7.0);
        assert_relative_eq!(x, 2.094_551_481_542_33, max_relative = 1e-9);
        assert!(
            n.get() > 5,
            "fell back to the bracket: {} evaluations",
            n.get()
        );
    }

    #[test]
    fn a_tiny_valve_still_passes_the_choked_nozzle_flow() {
        // ψ = 1e-6, 4 bar and 1000 K into a 1-bar pipe of 1000 mm²: choked through 1e-9 m²,
        // 0.6847·1e-9·4e5/√(287·1000) = 5.1126e-7 kg/s (Heywood App. C; the sonic bracket's
        // lower bound is exercised at its smallest)
        let (lam, a_back) = quiet(4.0e5, 1.0e5, 1000.0);
        let b = solve(4.0e5, sound(1000.0), 1e-6, lam, a_back);
        assert_relative_eq!(b.mass_flow(1e-3), 5.1126e-7, max_relative = 0.01);
    }

    #[test]
    fn the_valve_step_lets_the_pipe_gas_back_in_with_its_enthalpy() {
        use crate::gas::GasVolume;
        // a 0.8-bar, 1000 K cylinder against a 1.2-bar, 600 K wave-free pipe: gas flows back in
        // carrying the pipe gas's c_p·T, at the boundary's flow
        let gas = GasVolume::at(0.8e5, 1000.0, 5.0e-4);
        let pipe = std::f64::consts::PI * 0.02 * 0.02;
        let lam = lambda_of_wave(gas.pressure(), 0.0, 600.0, 1.2e5);
        let at = Pipe {
            area: pipe,
            lambda: lam,
            t_back: 600.0,
        };
        let dt = 1.0 / 192_000.0;
        let (f, b) = valve_flow(&gas, 3.0e-4, at, dt, 1.0, &mut { f64::NAN });
        assert!(f.dm < 0.0);
        assert_relative_eq!(f.h, crate::gas::CP * 600.0, max_relative = 1e-12);
        assert_relative_eq!(f.dm, b.mass_flow(pipe) * dt, max_relative = 1e-12);
        // the pipe gas's speed of sound brought to the cylinder's pressure: 600 K at 1.2 bar
        // → (0.8/1.2)^(1/7) of √(1.4·287·600) = 491.00·0.943 7 = 463.36 m/s, so at the boundary
        // a/A = 463.36
        let a_pc = (KAPPA * 287.0 * 600.0).sqrt() * (0.8f64 / 1.2).powf(1.0 / 7.0);
        let big_a = (b.p / 0.8e5).powf(1.0 / 7.0);
        assert_relative_eq!(b.a / big_a, a_pc, max_relative = 1e-9);
    }

    #[test]
    fn a_long_step_stops_where_the_pressures_meet_or_at_half_the_mass() {
        use crate::gas::GasVolume;
        // equalising mass (gas.rs): moving dm with enthalpy h changes p by (γ−1)·h·dm/V, so a
        // one-second step stops at relax·|p_c − p_b|·V/((γ−1)·h), and never past half the mass
        let pipe = std::f64::consts::PI * 0.02 * 0.02;
        for v in [5.0e-4, 5.0e-7] {
            let gas = GasVolume::at(3.0e5, 1300.0, v);
            let lam = lambda_of_wave(gas.pressure(), 0.0, 700.0, crate::units::ATM_PA);
            let at = Pipe {
                area: pipe,
                lambda: lam,
                t_back: 700.0,
            };
            let (f, b) = valve_flow(&gas, 3.0e-4, at, 1.0, 0.5, &mut { f64::NAN });
            let h = crate::gas::CP * 1300.0;
            let limit = 0.5 * (3.0e5 - b.p) * v / (0.4 * h);
            let want = limit.min(0.5 * gas.mass_kg);
            assert_relative_eq!(f.dm, want, max_relative = 1e-9);
        }
        // with relax 1 the equalising mass is 0.714·(1 − p_b/p_c) of the charge (R/((γ−1)c_p)),
        // so a 40-bar blowdown into a 1-bar pipe stops at half the charge
        let gas = GasVolume::at(40.0e5, 1800.0, 5.0e-4);
        let lam = lambda_of_wave(gas.pressure(), 0.0, 700.0, crate::units::ATM_PA);
        let at = Pipe {
            area: pipe,
            lambda: lam,
            t_back: 700.0,
        };
        let (f, _) = valve_flow(&gas, 3.0e-4, at, 1.0, 1.0, &mut { f64::NAN });
        assert_relative_eq!(f.dm, 0.5 * gas.mass_kg, max_relative = 1e-12);
        // closed: nothing moves, at the cylinder's own enthalpy and state
        let gas = GasVolume::at(3.0e5, 1300.0, 5.0e-4);
        let at = Pipe {
            area: pipe,
            lambda: 0.9,
            t_back: 700.0,
        };
        let (f, b) = valve_flow(&gas, 0.0, at, 1e-5, 1.0, &mut { f64::NAN });
        assert_eq!(f.dm, 0.0);
        assert_relative_eq!(f.h, crate::gas::CP * 1300.0, max_relative = 1e-12);
        assert_relative_eq!(b.p, 3.0e5, max_relative = 1e-12);
        assert_relative_eq!(b.a, (KAPPA * 287.0 * 1300.0).sqrt(), max_relative = 1e-12);
    }

    #[test]
    fn equal_states_do_not_flow_and_a_closed_valve_reflects() {
        let a = sound(900.0);
        let b = solve(1.0e5, a, 0.5, 1.0, a);
        assert_eq!(b.u, 0.0);
        assert_eq!(b.p, 1.0e5);
        // closed: the arriving wave doubles at the wall, u = 0, A = λ_in
        let b = solve(1.0e5, a, 0.0, 1.05, a);
        assert_eq!(b.u, 0.0);
        assert_relative_eq!(b.p, 1.0e5 * 1.05f64.powi(7), max_relative = 1e-12);
        assert_relative_eq!(b.a, a * 1.05, max_relative = 1e-12);
    }

    #[test]
    fn a_wave_arriving_on_a_quiet_pipe_is_its_simple_wave() {
        // no wave: the pipe at atmospheric pressure and rest, λ = (p₀/p_c)^(1/7)
        let p0 = crate::units::ATM_PA;
        assert_relative_eq!(
            lambda_of_wave(2.0e5, 0.0, 600.0, p0),
            (p0 / 2.0e5).powf(1.0 / 7.0),
            max_relative = 1e-12
        );
        // a compression arriving raises it: at rest a reflecting wall would see p₀ + 2p⁻,
        // λ ≈ ((p₀ + 2p⁻)/p_c)^(1/7) to first order in p⁻
        let small = 200.0;
        assert_relative_eq!(
            lambda_of_wave(p0, small, 600.0, p0),
            ((p0 + 2.0 * small) / p0).powf(1.0 / 7.0),
            max_relative = 1e-6
        );
    }

    #[test]
    fn the_valve_step_passes_the_boundarys_flow_within_its_limits() {
        use crate::gas::GasVolume;
        // a 3-bar, 1300 K cylinder of 0.5 L blowing into a quiet 40 mm primary through 3 cm²
        let gas = GasVolume::at(3.0e5, 1300.0, 5.0e-4);
        let pipe = std::f64::consts::PI * 0.02 * 0.02;
        let lam = lambda_of_wave(gas.pressure(), 0.0, 700.0, crate::units::ATM_PA);
        let at = |area, lambda| Pipe {
            area,
            lambda,
            t_back: 700.0,
        };
        let dt = 1.0 / 192_000.0;
        let (f, _) = valve_flow(&gas, 3.0e-4, at(pipe, lam), dt, 1.0, &mut { f64::NAN });
        let a_back = (KAPPA * 287.0 * 700.0).sqrt() / lam;
        let b = solve(
            3.0e5,
            (KAPPA * 287.0 * 1300.0).sqrt(),
            3.0e-4 / pipe,
            lam,
            a_back,
        );
        assert_relative_eq!(f.dm, b.mass_flow(pipe) * dt, max_relative = 1e-12);
        assert_relative_eq!(f.h, crate::gas::CP * 1300.0, max_relative = 1e-12);
        // a step long enough to empty it stops at half the mass or at equal pressures
        let (f, _) = valve_flow(&gas, 3.0e-4, at(pipe, lam), 1.0, 1.0, &mut { f64::NAN });
        assert!(f.dm <= 0.5 * gas.mass_kg + 1e-15);
        // a closed valve passes nothing
        assert_eq!(
            valve_flow(&gas, 0.0, at(pipe, lam), dt, 1.0, &mut { f64::NAN })
                .0
                .dm,
            0.0
        );
    }

    #[test]
    fn a_weak_outflow_is_the_acoustic_impedance_of_the_gas_it_sends() {
        // small amplitude: λ_in held, δ(p^(1/7)) = (κ−1)/2·u/a·p^(1/7), so δp = κ·p·u/a = ρ·a·u =
        // a·ṁ/F — the pipe's characteristic impedance, with the outflowing gas's a (Kinsler et
        // al., *Fundamentals of Acoustics*, §9.2 for the linear relation)
        let (p_pipe, f_pipe) = (1.0e5, 1.0e-3);
        let p_c = 1.004e5;
        let (lam, a_back) = quiet(p_c, p_pipe, 900.0);
        let b = solve(p_c, sound(900.0), 1.0, lam, a_back);
        assert_relative_eq!(
            b.p - p_pipe,
            b.a * b.mass_flow(f_pipe) / f_pipe,
            max_relative = 0.01
        );
    }

    proptest::proptest! {
        /// Any state the engine can meet solves to a finite boundary that keeps the incoming
        /// characteristic; outflow keeps the stagnation enthalpy.
        #[test]
        fn every_state_solves_and_keeps_its_laws(
            lam in 0.4f64..1.6,
            psi in 0.0f64..1.2,
            t_c in 300.0f64..2500.0,
            t_back in 280.0f64..1300.0,
        ) {
            let (p_c, a_c, a_back) = (2.0e5, sound(t_c), sound(t_back));
            let b = solve(p_c, a_c, psi, lam, a_back);
            proptest::prop_assert!(b.p.is_finite() && b.u.is_finite() && b.a.is_finite());
            proptest::prop_assert!(b.p > 0.0 && b.a > 0.0);
            if psi >= PSI_MIN && lam < 1.0 {
                proptest::prop_assert!(
                    ((b.a * b.a + KM * b.u * b.u) / (a_c * a_c) - 1.0).abs() < 1e-8
                );
            }
            if psi >= PSI_MIN {
                proptest::prop_assert!((lambda_in(p_c, b.p, b.u, b.a) / lam - 1.0).abs() < 1e-8);
            }
        }
    }
}
