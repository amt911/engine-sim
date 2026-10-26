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
    let psi = psi.clamp(0.0, 1.0);
    if psi <= 0.0 {
        // a closed valve reflects: u = 0, A = λ_in
        return Boundary {
            p: p_cyl * lambda.powi(P_EXP),
            u: 0.0,
            a: a_pipe_at_pc * lambda,
        };
    }
    if lambda < 1.0 {
        let (pi, minus_ub, c) = outflow(psi, lambda);
        Boundary {
            p: p_cyl * pi.powi(P_EXP),
            u: minus_ub * a_cyl,
            a: c * a_cyl * pi,
        }
    } else if lambda > 1.0 {
        let (a, u) = inflow(psi, lambda);
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

/// Root of `f` in [lo, hi] where f(lo) and f(hi) have opposite signs: Illinois regula falsi.
fn root(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    let (mut flo, mut fhi) = (f(lo), f(hi));
    if flo == 0.0 {
        return lo;
    }
    if fhi == 0.0 {
        return hi;
    }
    let mut side = 0;
    for _ in 0..100 {
        let x = (lo * fhi - hi * flo) / (fhi - flo);
        let fx = f(x);
        if fx == 0.0 || (hi - lo).abs() < 1e-14 {
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
        if (hi - lo).abs() < 1e-13 * hi.abs().max(1.0) {
            break;
        }
    }
    0.5 * (lo + hi)
}

/// Outflow through the valve (λ_in = π_α < 1): returns (π, −U_B, C), with π = (p/p_c)^(1/7),
/// −U_B = u/a_c into the pipe and C = a_A2/a_A1 the entropy rise across the valve.
fn outflow(psi: f64, pi_a: f64) -> (f64, f64, f64) {
    let e = psi * SONIC_FLOW;
    // C by successive substitution from 1/π_α (6.61), π from (6.47) or (6.51) each time
    let mut c = 1.0 / pi_a;
    let mut pi = pi_a;
    for _ in 0..50 {
        // sonic boundary π_cr (6.55); above it the throat is subsonic
        let pi_cr = S / (2.0 * psi)
            * (2.0 * psi + (1.0 - (1.0 + (KAPPA * KAPPA - 1.0) * psi * psi).sqrt()) / c);
        let b = |p: f64| c * (p - pi_a);
        pi = if pi_a >= pi_cr {
            // (6.47): ψ/π·{2/(κ−1)(1/π² − 1)}^½ − (2B/(κ−1))/(1 − 2B²/(κ−1)) = 0
            let f1 = |p: f64| {
                let bb = b(p);
                psi / p * ((1.0 / (p * p) - 1.0) / KM).max(0.0).sqrt()
                    - (bb / KM) / (1.0 - bb * bb / KM)
            };
            root(f1, pi_a.max(S), 1.0)
        } else {
            // (6.51): π^(2κ/(κ−1)) − E·(1 − 2B²/(κ−1))/(2B/(κ−1)) = 0
            let f2 = |p: f64| {
                let bb = b(p);
                p.powi(P_EXP) - e * (1.0 - bb * bb / KM) / (bb / KM)
            };
            root(f2, pi_a + 1e-15, S)
        };
        let bb = c * (pi - pi_a);
        // (6.56): C = (1 − 2B²/(κ−1))^½ / π
        let c_new = (1.0 - bb * bb / KM).max(0.0).sqrt() / pi;
        if (c_new - c).abs() < 1e-13 {
            c = c_new;
            break;
        }
        c = c_new;
    }
    (pi, c * (pi - pi_a) / KM, c)
}

/// Inflow from the pipe (λ_in > 1): the partially open end with the cylinder as its receiver.
/// Returns (A, U): A = (p/p_c)^(1/7) and U the velocity toward the cylinder over the pipe gas's
/// speed of sound at p_c.
fn inflow(phi: f64, lambda: f64) -> (f64, f64) {
    // subsonic (6.35): (A^(4/(κ−1)) − φ²)(λ − A)² − (κ−1)/2·(A² − 1)φ² = 0, A in [1, λ]
    let f = |a: f64| {
        let a10 = a.powi(2 * P_EXP - 4);
        (a10 - phi * phi) * (lambda - a) * (lambda - a) - KM * (a * a - 1.0) * phi * phi
    };
    let a = root(f, 1.0, lambda);
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
    fn equal_states_do_not_flow_and_a_closed_valve_reflects() {
        let a = sound(900.0);
        let b = solve(1.0e5, a, 0.5, 1.0, a);
        assert_eq!(b.u, 0.0);
        assert_eq!(b.p, 1.0e5);
        // closed: the arriving wave doubles at the wall, u = 0, A = λ_in
        let b = solve(1.0e5, a, 0.0, 1.05, a);
        assert_eq!(b.u, 0.0);
        assert_relative_eq!(b.p, 1.0e5 * 1.05f64.powi(7), max_relative = 1e-12);
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
}
