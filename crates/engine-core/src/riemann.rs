//! Finite-amplitude plane waves in a pipe as Riemann variables (Benson, *The Thermodynamics
//! and Gas Dynamics of Internal-Combustion Engines*, Vol. 1, 1982, §2.9 and §6.1).
//!
//! Homentropic flow about a reference state (p_ref, a_ref): A = a/a_ref = (p/p_ref)^((κ−1)/2κ)
//! (eq. 2.34) and U = u/a_ref. The Riemann variables λ = A + (κ−1)/2·U and β = A − (κ−1)/2·U
//! (eqs. 6.1–6.2) are constant along their characteristics, which run at dX/dZ = bλ − aβ and
//! aλ − bβ, with a = (3−κ)/(2(κ−1)) and b = (κ+1)/(2(κ−1)) (eqs. 6.5–6.8). At rest both are 1.

/// The Riemann variables (λ, β) of a state of non-dimensional sound speed `a` and velocity `u`.
pub fn riemann(a: f64, u: f64, kappa: f64) -> (f64, f64) {
    let h = 0.5 * (kappa - 1.0) * u;
    (a + h, a - h)
}

/// The state (A, U) of Riemann variables `lambda`, `beta` (Benson §2.9).
pub fn state(lambda: f64, beta: f64, kappa: f64) -> (f64, f64) {
    (0.5 * (lambda + beta), (lambda - beta) / (kappa - 1.0))
}

/// A = (p/p_ref)^((κ−1)/2κ) (eq. 2.34).
pub fn sound_speed_ratio(p_ratio: f64, kappa: f64) -> f64 {
    p_ratio.powf((kappa - 1.0) / (2.0 * kappa))
}

/// p/p_ref of a non-dimensional sound speed `a` (the inverse of [`sound_speed_ratio`]).
pub fn pressure_ratio(a: f64, kappa: f64) -> f64 {
    a.powf(2.0 * kappa / (kappa - 1.0))
}

/// The speeds (in a_ref) of the λ and β characteristics (eqs. 6.7–6.8): λ's runs forward at
/// bλ − aβ; β's runs backward, at the magnitude bβ − aλ.
pub fn slopes(lambda: f64, beta: f64, kappa: f64) -> (f64, f64) {
    let a = (3.0 - kappa) / (2.0 * (kappa - 1.0));
    let b = (kappa + 1.0) / (2.0 * (kappa - 1.0));
    (b * lambda - a * beta, b * beta - a * lambda)
}

/// Open end with outflow (Benson §2.8.2): the pressure there is the outside one, A = 1, so the
/// variable reflected back is 2 − (the one arriving).
pub fn open_end(arriving: f64) -> f64 {
    // A = (λ + β)/2 = 1
    2.0 - arriving
}

/// Closed end (Benson §2.8.1): U = 0, the arriving variable returns unchanged.
pub fn closed_end(arriving: f64) -> f64 {
    // U = (λ − β)/(κ − 1) = 0
    arriving
}

/// A pipe of finite-amplitude waves: λ runs forward from the near end, β back from the far one,
/// each sample at its characteristic's speed ([`slopes`]) with the other variable taken as its
/// mean along the pipe (exact for a simple wave). The lines carry the deviation from rest
/// (λ − 1, β − 1), on which the pipe's per-trip loss acts. Shocks form and dissipate as in
/// [`crate::waveguide::WarpLine`].
#[derive(Debug, Clone)]
pub struct RPipe {
    fwd: crate::waveguide::WarpLine,
    bwd: crate::waveguide::WarpLine,
    kappa: f64,
    gain: f64,
}

impl RPipe {
    /// `length_m` at reference sound speed `a_ref` (m/s), steps of `dt`; `gain`: amplitude kept
    /// per trip.
    pub fn new(length_m: f64, a_ref: f64, dt: f64, kappa: f64, gain: f64) -> Self {
        let delay = length_m / a_ref / dt;
        Self {
            fwd: crate::waveguide::WarpLine::new(delay, 0.0),
            bwd: crate::waveguide::WarpLine::new(delay, 0.0),
            kappa,
            gain,
        }
    }
    /// λ arriving at the far end this step.
    pub fn at_far(&self) -> f64 {
        1.0 + self.gain * self.fwd.read()
    }
    /// β arriving at the near end this step.
    pub fn at_near(&self) -> f64 {
        1.0 + self.gain * self.bwd.read()
    }
    /// This step's outgoing variables: λ into the near end, β into the far end.
    pub fn send(&mut self, lambda: f64, beta: f64) {
        // each against the other's mean along the pipe
        let (f, _) = slopes(lambda, 1.0 + self.bwd.mean(), self.kappa);
        let (_, r) = slopes(1.0 + self.fwd.mean(), beta, self.kappa);
        self.fwd.push_at(lambda - 1.0, f);
        self.bwd.push_at(beta - 1.0, r);
    }
}

/// The gas in a pipe the Riemann variables refer to: its reference pressure and sound speed,
/// its cross-section and its ratio of heats.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeGas {
    pub p_ref: f64,
    pub a_ref: f64,
    pub area_m2: f64,
    pub kappa: f64,
}

/// What a valve boundary hands back: the variable it sends into the pipe, the mass flow
/// (kg/s, out of the cylinder > 0) and the pressure at the pipe end.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValveFlow {
    pub sent: f64,
    pub mass_flow: f64,
    pub p_pipe: f64,
}

/// Outflow from a cylinder at stagnation (`p0`, `t0`) through a valve of effective area
/// `psi`·(pipe area) into a pipe whose arriving variable is `arriving` (Benson §4.1, the
/// constant-pressure model).
///
/// In cylinder units, U = u₂/a₀ and π = (p₂/p₀)^((κ−1)/2κ). The throat sets π(U):
/// - subsonic (eq. 4.12), which with s = U/(1 − kU²), k = (κ−1)/2 and y = 1/π² is the
///   quadratic y² − y − k·s²/ψ² = 0;
/// - sonic below π* = (2/(κ+1))^½ (eqs. 4.15, 4.20);
/// - and U is at most (2/(κ+1))^½, where the pipe itself chokes (eq. 4.21).
///
/// The pipe sets the other relation: the arriving β = A₂ − k·U·a₀/a_ref with A₂ = π·A₀,
/// A₀ = (p₀/p_ref)^((κ−1)/2κ). The pipe's variable going out is λ = 2A₂ − β, and the mass flow
/// is ρ₂u₂F₂ with a₂ from the energy equation, (a₂/a₀)² = 1 − kU² (eq. 4.10).
/// A pipe at or above the cylinder's pressure gets no outflow and reflects as a closed end.
pub fn valve_outflow(p0: f64, t0: f64, psi: f64, arriving: f64, gas: &PipeGas) -> ValveFlow {
    let kappa = gas.kappa;
    let k = 0.5 * (kappa - 1.0);
    let a_cyl = sound_speed_ratio(p0 / gas.p_ref, kappa);
    if psi <= 0.0 || arriving >= a_cyl {
        return ValveFlow {
            sent: closed_end(arriving),
            mass_flow: 0.0,
            p_pipe: gas.p_ref * pressure_ratio(arriving, kappa),
        };
    }
    let a0 = (kappa * crate::gas::R_AIR * t0).sqrt();
    let r = a0 / gas.a_ref;
    let pi_star = (2.0 / (kappa + 1.0)).sqrt();
    let u_choke = pi_star;
    // the throat's π for a pipe velocity U (eqs. 4.12, 4.20)
    let sonic_k = (2.0 / (kappa + 1.0)).powf((kappa + 1.0) / (2.0 * (kappa - 1.0)));
    let pi_of = |u: f64| {
        if u <= 0.0 {
            return 1.0;
        }
        let sv = u / (1.0 - k * u * u);
        let y = 0.5 * (1.0 + (1.0 + 4.0 * k * sv * sv / (psi * psi)).sqrt());
        let pi = 1.0 / y.sqrt();
        if pi >= pi_star {
            pi
        } else {
            (psi * sonic_k * (1.0 - k * u * u) / u).powf((kappa - 1.0) / (2.0 * kappa))
        }
    };
    // the pipe: g(U) = π(U)·A₀ − k·U·r − β falls with U from A₀ − β > 0
    let g = |u: f64| pi_of(u) * a_cyl - k * u * r - arriving;
    let u = if g(u_choke) >= 0.0 {
        u_choke
    } else {
        crate::exhaust_fv::rising_root(|u| -g(u), 0.0, u_choke)
    };
    let pi = pi_of(u);
    let p_pipe = p0 * pressure_ratio(pi, kappa);
    let a2 = a0 * (1.0 - k * u * u).sqrt();
    let rho2 = kappa * p_pipe / (a2 * a2);
    ValveFlow {
        sent: 2.0 * pi * a_cyl - arriving,
        mass_flow: rho2 * u * a0 * gas.area_m2,
        p_pipe,
    }
}

/// Inflow from the pipe back into a cylinder at pressure `p_cyl` through a valve of effective
/// area `psi`·(pipe area) (Benson §4.4: the partially open end of §3.4.1 with the cylinder
/// as the receiver). Mass flow negative (into the cylinder). A pipe below the cylinder's
/// pressure gets none and reflects as a closed end.
///
/// Referred to the cylinder pressure, A* = A·c and U* = |U|·c with c = (p_ref/p_cyl)^((κ−1)/2κ)
/// (§4.4). The nozzle then sets:
/// - subsonic, U*² = 2/(κ−1)·(A*² − 1) / (A*^(4/(κ−1))/ψ² − 1) (eq. 3.15), while the throat
///   (at the cylinder pressure, A_t = 1) runs below its sound speed;
/// - choked, U* = m·A* with m = ψ·r^(−(κ+1)/(κ−1)), where r = A/A_t is the subsonic root of
///   2/(κ−1)·r² + ψ²·r^(−4/(κ−1)) = (κ+1)/(κ−1) (eqs. 3.18–3.20).
///
/// The pipe sets the other relation: the arriving β = A + k·|U|.
pub fn valve_inflow(p_cyl: f64, psi: f64, arriving: f64, gas: &PipeGas) -> ValveFlow {
    let kappa = gas.kappa;
    let k = 0.5 * (kappa - 1.0);
    let a_cyl = sound_speed_ratio(p_cyl / gas.p_ref, kappa);
    if psi <= 0.0 || arriving <= a_cyl {
        return ValveFlow {
            sent: closed_end(arriving),
            mass_flow: 0.0,
            p_pipe: gas.p_ref * pressure_ratio(arriving, kappa),
        };
    }
    let e = 2.0 / (kappa - 1.0);
    let rho_ref = kappa * gas.p_ref / (gas.a_ref * gas.a_ref);
    let flow = |a_pipe: f64, u: f64| ValveFlow {
        sent: 2.0 * a_pipe - arriving,
        mass_flow: -rho_ref * a_pipe.powf(e) * u * gas.a_ref * gas.area_m2,
        p_pipe: gas.p_ref * pressure_ratio(a_pipe, kappa),
    };
    if psi >= 1.0 {
        // no contraction: the pipe end is at the cylinder's pressure (an open end, §2.8.2)
        // and β sets the velocity, at most sonic (then the pipe chokes: A = U = β/(1 + k))
        let u = (arriving - a_cyl) / k;
        return if u <= a_cyl {
            flow(a_cyl, u)
        } else {
            let a = arriving / (1.0 + k);
            flow(a, a)
        };
    }
    let c = 1.0 / a_cyl;
    // the choked line's slope: r from 2/(κ−1)·r² + ψ²·r^(−2e) = (κ+1)/(κ−1), r ≥ 1
    let rhs = (kappa + 1.0) / (kappa - 1.0);
    let r_max = ((kappa + 1.0) / 2.0).sqrt();
    let r = crate::exhaust_fv::rising_root(
        |r| e * r * r + psi * psi * r.powf(-2.0 * e) - rhs,
        1.0,
        r_max,
    );
    let m = psi * r.powf(-(kappa + 1.0) / (kappa - 1.0));
    // U* of a pipe state A* (subsonic throat, else choked)
    let u_star = |a: f64| {
        if a <= 1.0 {
            return 0.0;
        }
        let den = a.powf(2.0 * e) / (psi * psi) - 1.0;
        let u2 = if den > 0.0 {
            e * (a * a - 1.0) / den
        } else {
            f64::INFINITY
        };
        // the throat's velocity at the cylinder pressure, from the energy equation
        let ut2 = (a * a + k * u2 - 1.0) / k;
        if ut2 <= 1.0 { u2.sqrt() } else { m * a }
    };
    // g(A) = A + k·U*(A·c)/c − β rises from 1/c − β < 0 to k·U*(β·c)/c ≥ 0
    let g = |a: f64| a + k * u_star(a * c) / c - arriving;
    let a_pipe = crate::exhaust_fv::rising_root(g, a_cyl, arriving);
    flow(a_pipe, u_star(a_pipe * c) / c)
}

/// A constant-pressure junction of pipe ends (Benson §5.5.1, eqs. 5.23–5.34, for any number of
/// pipes): every end has the same pressure, so the same A (all pipes referred to the same
/// p_ref), and the mass flows into the junction sum to zero. `arriving[n]` is the variable
/// coming into the junction along pipe n, A + k·Uₙ with Uₙ toward the junction;
/// `weights[n]` = Fₙ/a_ref,n (ρₙuₙFₙ ∝ Uₙ·Fₙ/a_ref,n at a common pressure). Writes the variable
/// each pipe receives, 2A − arriving, and returns A.
pub fn junction(arriving: &[f64], weights: &[f64], sent: &mut [f64]) -> f64 {
    // Σ wₙ·Uₙ = 0 with Uₙ = (arriving − A)/k: A is the weighted mean of what arrives
    let total: f64 = weights.iter().sum();
    let a = weights
        .iter()
        .zip(arriving)
        .map(|(w, l)| w * l)
        .sum::<f64>()
        / total;
    for (s, l) in sent.iter_mut().zip(arriving) {
        *s = 2.0 * a - l;
    }
    a
}

/// The open end of a pipe at the outside pressure, with `arriving` the variable reaching it.
/// Outflow (it arrives above the outside state, 1): the end is at the outside pressure, A = 1
/// (Benson §2.8.2). Inflow: air from the still atmosphere (stagnation A₀ = 1) accelerates into
/// the pipe isentropically, A² + k·U² = 1 (Benson §3.3), with U into the pipe. Either way at
/// most sonic. Returns the variable sent back into the pipe.
pub fn open_end_flow(arriving: f64, kappa: f64) -> f64 {
    let k = 0.5 * (kappa - 1.0);
    if arriving >= 1.0 {
        // outflow at A = 1, U = (λ − 1)/k; past U = A it chokes: A = U = λ/(1 + k)
        if (arriving - 1.0) / k <= 1.0 {
            return open_end(arriving);
        }
        return arriving * (1.0 - k) / (1.0 + k);
    }
    // inflow: (k + 1)·A² − 2λ·A + λ² − k = 0, the root above λ; sonic (U = −A) below
    // λ = (1 − k)/√(1 + k), where A = 1/√(1 + k) and β = A·(1 + k)
    let a_sonic = 1.0 / (1.0 + k).sqrt();
    if arriving <= a_sonic * (1.0 - k) {
        return a_sonic * (1.0 + k);
    }
    let a = (arriving + (k * (1.0 + k) - k * arriving * arriving).sqrt()) / (1.0 + k);
    2.0 * a - arriving
}

#[cfg(test)]
mod tests {
    use super::*;

    const K: f64 = 1.4;

    #[test]
    fn riemann_variables_by_hand() {
        // A = 1.1, U = 0.2, (κ−1)/2 = 0.2: λ = 1.14, β = 1.06, and back
        let (l, b) = riemann(1.1, 0.2, K);
        assert!(
            (l - 1.14).abs() < 1e-12 && (b - 1.06).abs() < 1e-12,
            "{l} {b}"
        );
        let (a, u) = state(l, b, K);
        assert!(
            (a - 1.1).abs() < 1e-12 && (u - 0.2).abs() < 1e-12,
            "{a} {u}"
        );
    }

    #[test]
    fn the_characteristics_run_at_u_plus_and_minus_a() {
        // κ = 1.4: a = (3−κ)/(2(κ−1)) = 1.6/0.8 = 2, b = (κ+1)/(2(κ−1)) = 2.4/0.8 = 3.
        // λ = 1.14, β = 1.06: forward 3·1.14 − 2·1.06 = 1.30 = U + A; backward
        // 3·1.06 − 2·1.14 = 0.90 = A − U
        let (f, r) = slopes(1.14, 1.06, K);
        assert!(
            (f - 1.30).abs() < 1e-12 && (r - 0.90).abs() < 1e-12,
            "{f} {r}"
        );
        // at rest both run at the sound speed
        assert_eq!(slopes(1.0, 1.0, K), (1.0, 1.0));
    }

    #[test]
    fn sound_speed_and_pressure_by_hand() {
        // doubling the pressure: A = 2^(0.4/2.8) = 2^(1/7) = 1.104090
        let a = sound_speed_ratio(2.0, K);
        assert!((a - 1.104_090).abs() < 1e-6, "{a}");
        assert!((pressure_ratio(a, K) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn ends_reflect_as_benson_draws_them() {
        // open end: a compression λ = 1.1 returns as β = 0.9 (a rarefaction)
        assert!((open_end(1.1) - 0.9).abs() < 1e-12);
        // closed end: unchanged
        assert_eq!(closed_end(1.1), 1.1);
    }

    /// Steps until the far end sees λ rise past half of `step` above rest.
    fn arrival(p: &mut RPipe, lambda: f64, beta: f64, steps: usize) -> Option<usize> {
        (0..steps).find(|_| {
            let seen = p.at_far() - 1.0 > 0.5 * (lambda - 1.0);
            p.send(lambda, beta);
            seen
        })
    }

    #[test]
    fn a_compression_step_is_a_shock_at_the_mean_speed() {
        // 1 m at a_ref = 500 m/s, dt = 1/192 000 s: D = 384 steps at rest. A step to λ = 1.01
        // into still gas is a weak shock: it runs at the mean of the characteristic speeds
        // ahead (1) and behind (3·1.01 − 2 = 1.03), 1.015 (Rankine–Hugoniot for weak shocks;
        // Salomons 2025, §2): 384/1.015 = 378.3 steps
        let dt = 1.0 / 192_000.0;
        let mut p = RPipe::new(1.0, 500.0, dt, K, 1.0);
        let k = arrival(&mut p, 1.01, 1.0, 1000).unwrap();
        assert!(k.abs_diff(378) <= 2, "arrived after {k}");
    }

    #[test]
    fn a_rarefaction_ramp_spreads_into_a_fan() {
        // λ ramping down from 1 to 0.95 over 20 steps is an expansion: each value runs at its
        // own speed 3λ − 2. Its head (λ = 1) arrives at the rest delay, 384 steps; the middle
        // (0.975, pushed at step 10) at 10 + 384/0.925 = 425.1; the tail (0.95, step 20) at
        // 20 + 384/0.85 = 471.8
        let dt = 1.0 / 192_000.0;
        let mut p = RPipe::new(1.0, 500.0, dt, K, 1.0);
        let mut far = Vec::new();
        for k in 0..700 {
            far.push(p.at_far());
            p.send(1.0 - 0.05 * (k as f64 / 20.0).min(1.0), 1.0);
        }
        let first = |level: f64| far.iter().position(|&l| l < level).unwrap();
        assert!(
            first(0.9999).abs_diff(385) <= 2,
            "head at {}",
            first(0.9999)
        );
        assert!(
            first(0.975).abs_diff(425) <= 2,
            "middle at {}",
            first(0.975)
        );
        assert!(
            first(0.9501).abs_diff(472) <= 2,
            "tail at {}",
            first(0.9501)
        );
    }

    #[test]
    fn a_wave_against_a_rarefaction_slows_down() {
        // the pipe full of β = 0.95 (a rarefaction running back): a faint λ ≈ 1 runs at
        // 3·1 − 2·0.95 = 1.1, so 384/1.1 = 349.1 steps
        let dt = 1.0 / 192_000.0;
        let mut p = RPipe::new(1.0, 500.0, dt, K, 1.0);
        for _ in 0..2000 {
            p.send(1.0, 0.95);
        }
        let k = arrival(&mut p, 1.0 + 1e-6, 0.95, 1000).unwrap();
        assert!(k.abs_diff(349) <= 2, "arrived after {k}");
        // and the near end receives the β it was sent
        assert!((p.at_near() - 0.95).abs() < 1e-12);
    }

    #[test]
    fn the_pipe_loss_wears_the_deviation_not_the_rest_state() {
        let dt = 1.0 / 192_000.0;
        let mut p = RPipe::new(0.1, 500.0, dt, K, 0.5);
        for _ in 0..500 {
            p.send(1.2, 1.0);
        }
        // λ − 1 = 0.2 kept at half: 1.1; the rest state keeps 1
        assert!((p.at_far() - 1.1).abs() < 1e-9, "{}", p.at_far());
        assert!((p.at_near() - 1.0).abs() < 1e-12);
    }

    fn exhaust_gas() -> PipeGas {
        // a 42 mm primary of air at 1 atm, 600 K (a = √(1.4·287·600) = 491.0 m/s)
        PipeGas {
            p_ref: crate::units::ATM_PA,
            a_ref: (1.4 * crate::gas::R_AIR * 600.0f64).sqrt(),
            area_m2: std::f64::consts::PI * 0.25 * 0.042 * 0.042,
            kappa: 1.4,
        }
    }

    #[test]
    fn a_valve_passes_what_an_orifice_passes_to_the_same_pipe_pressure() {
        // Whatever pipe state the boundary settles on, the flow through the throat is an
        // isentropic orifice from the cylinder's stagnation state to that pressure
        // (crate::gas::orifice_mass_flow: Heywood, App. C), subsonic or choked.
        let gas = exhaust_gas();
        for (p0, t0, psi) in [
            (1.3e5, 900.0, 0.3),
            (2.0e5, 1100.0, 0.5),
            (4.0e5, 1200.0, 0.2),
            (6.0e5, 1300.0, 0.05),
        ] {
            let v = valve_outflow(p0, t0, psi, 1.0, &gas);
            let orifice = crate::gas::orifice_mass_flow(psi * gas.area_m2, p0, t0, v.p_pipe);
            assert!(v.mass_flow > 0.0, "{p0} Pa: no flow");
            assert!(
                (v.mass_flow / orifice - 1.0).abs() < 1e-6,
                "{p0} Pa, ψ {psi}: {} vs orifice {orifice}",
                v.mass_flow
            );
            // and what it sends is the pipe-end state: (λ + β)/2 = (p/p_ref)^((κ−1)/2κ)
            let a = 0.5 * (v.sent + 1.0);
            assert!((a - sound_speed_ratio(v.p_pipe / gas.p_ref, 1.4)).abs() < 1e-9);
        }
    }

    #[test]
    fn no_outflow_against_a_higher_pipe_pressure() {
        let gas = exhaust_gas();
        // a cylinder at 1 atm against an arriving compression (β = 1.05: the pipe above 1 atm)
        let v = valve_outflow(crate::units::ATM_PA, 900.0, 0.3, 1.05, &gas);
        assert_eq!(v.mass_flow, 0.0);
        assert_eq!(v.sent, 1.05, "a shut boundary reflects as a closed end");
    }

    #[test]
    fn a_wide_valve_at_high_pressure_chokes_in_the_pipe() {
        // ψ = 1 from 20 bar: the pipe itself runs sonic, U = (2/(κ+1))^½ = 0.912871 of a₀
        // (Benson eq. 4.21), u₂ = a₂
        let gas = exhaust_gas();
        let (p0, t0) = (20e5, 1200.0);
        let v = valve_outflow(p0, t0, 1.0, 1.0, &gas);
        let a0 = (1.4 * crate::gas::R_AIR * t0).sqrt();
        let u2 = 0.912_871 * a0;
        let a2 = a0 * (1.0 - 0.2 * 0.912_871f64.powi(2)).sqrt();
        // ρ₂u₂F₂ with u₂ = a₂ = 0.912871·a₀ and ρ₂ = κp₂/a₂²
        let rho2 = 1.4 * v.p_pipe / (a2 * a2);
        assert!((u2 / a2 - 1.0).abs() < 1e-5);
        assert!((v.mass_flow / (rho2 * u2 * gas.area_m2) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn backflow_into_the_cylinder_is_an_isentropic_nozzle_from_the_pipe() {
        // Overrun: the cylinder at 0.3 bar opens onto a pipe at 1 atm and at rest (β = 1).
        // Whatever pipe state the boundary settles on, the flow back through the throat is an
        // isentropic orifice from that state's stagnation conditions to the cylinder pressure
        // (crate::gas::orifice_mass_flow, Heywood App. C), subsonic or choked.
        let gas = exhaust_gas();
        let cp = 1.4 / 0.4 * crate::gas::R_AIR;
        for (p_cyl, psi) in [(0.9e5, 0.3), (0.3e5, 0.3), (0.15e5, 0.05), (0.3e5, 1.0)] {
            let v = valve_inflow(p_cyl, psi, 1.0, &gas);
            assert!(v.mass_flow < 0.0, "{p_cyl} Pa: no backflow");
            // the pipe-end state from the two variables
            let (a, u) = state(v.sent, 1.0, 1.4);
            assert!(u < 0.0, "flowing toward the valve");
            let t = (a * gas.a_ref).powi(2) / (1.4 * crate::gas::R_AIR);
            let vel = u * gas.a_ref;
            let t0 = t + vel * vel / (2.0 * cp);
            let p0 = v.p_pipe * (t0 / t).powf(1.4 / 0.4);
            let orifice = crate::gas::orifice_mass_flow(psi * gas.area_m2, p0, t0, p_cyl);
            assert!(
                (-v.mass_flow / orifice - 1.0).abs() < 1e-6,
                "{p_cyl} Pa, ψ {psi}: {} vs orifice {orifice}",
                -v.mass_flow
            );
            assert!((a - sound_speed_ratio(v.p_pipe / gas.p_ref, 1.4)).abs() < 1e-9);
        }
        // a cylinder above the pipe: no backflow, a closed end
        let v = valve_inflow(1.5e5, 0.3, 1.0, &gas);
        assert_eq!((v.mass_flow, v.sent), (0.0, 1.0));
    }

    #[test]
    fn a_faint_wave_splits_at_a_junction_as_kelly_lochbaum() {
        // Small amplitude: the pressure waves through the junction are the linear N-port's
        // (crate::waveguide::junction), here three pipes of areas 1, 2 and 3 with a wave in the
        // first. To first order p/p_ref − 1 = 2κ/(κ−1)·(A − 1), so a variable's deviation is
        // proportional to the pressure wave it carries, and the junction must scatter the
        // deviations as Kelly–Lochbaum scatters pressure waves.
        let areas = [1.0, 2.0, 3.0];
        let eps = 1e-6;
        let mut sent = [0.0; 3];
        junction(&[1.0 + eps, 1.0, 1.0], &areas, &mut sent);
        // linear: incoming pressure wave p⁺ in pipe 0 only
        let mut lin = [0.0; 3];
        crate::waveguide::junction(&areas, &[1.0, 0.0, 0.0], &mut lin);
        for (n, (s, l)) in sent.iter().zip(lin).enumerate() {
            let got = (s - 1.0) / eps;
            assert!((got - l).abs() < 1e-5, "pipe {n}: {got} vs {l}");
        }
    }

    #[test]
    fn a_junction_conserves_mass_at_finite_amplitude() {
        // a strong compression into pipe 0 (λ = 1.15) and a rarefaction in pipe 2 (0.95):
        // Σ ρₙ·uₙ·Fₙ into the junction is zero, with ρ ∝ A^(2/(κ−1))/a_ref² and u = U·a_ref
        let (a_ref, areas) = ([600.0, 560.0, 500.0], [1.0, 1.0, 2.0]);
        let weights: Vec<f64> = areas.iter().zip(a_ref).map(|(f, a)| f / a).collect();
        let arriving = [1.15, 1.0, 0.95];
        let mut sent = [0.0; 3];
        let a = junction(&arriving, &weights, &mut sent);
        let into: f64 = (0..3)
            .map(|n| {
                // velocity toward the junction from the variables arriving and leaving
                let u = (arriving[n] - sent[n]) / (2.0 * 0.2) * a_ref[n];
                let rho = a.powf(5.0) / (a_ref[n] * a_ref[n]);
                rho * u * areas[n]
            })
            .sum();
        assert!(into.abs() < 1e-15, "net mass flow {into}");
        // and every end leaves at the common A
        for n in 0..3 {
            assert!((0.5 * (arriving[n] + sent[n]) - a).abs() < 1e-12);
        }
    }

    #[test]
    fn an_open_end_draws_air_in_isentropically() {
        // λ = 0.95 arriving (a rarefaction): A² + 0.2·U² = 1 with λ = A + 0.2·U gives
        // 1.2A² − 1.9A + 0.7025 = 0, the root above λ: A = (1.9 + 0.4878524)/2.4 = 0.9949385,
        // U = (0.95 − A)/0.2 = −0.224693 (into the pipe); β = 2A − λ = 1.0398770
        let b = open_end_flow(0.95, 1.4);
        assert!((b - 1.039_877_0).abs() < 1e-6, "{b}");
        // outflow is the plain open end: A = 1
        assert!((open_end_flow(1.05, 1.4) - 0.95).abs() < 1e-12);
        // and the two meet at rest
        assert!((open_end_flow(1.0, 1.4) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_strong_pulse_travels_as_the_euler_solver_says() {
        // The same 1 ms half-sine of mass flow (0.16 kg/s peak, about half a bar) into a 42 mm
        // pipe of still air at 300 K, through 2 m: the finite-volume Euler solver (WENO5-Z +
        // SSP-RK3, independent of this module) and the Riemann pipe must agree on the pulse
        // they deliver. It steepens into a shock well before 2 m (x_s ≈ 0.4 m).
        use crate::fv::{Boundary, Pipe1d, Prim, flux_boundary};
        let (p0, t_ref, kappa) = (crate::units::ATM_PA, 300.0, 1.4);
        let r = crate::gas::R_AIR;
        let a_ref = (kappa * r * t_ref).sqrt();
        let rho0 = p0 / (r * t_ref);
        let area = std::f64::consts::PI * 0.25 * 0.042 * 0.042;
        let dt = 5e-6;
        let mdot = |t: f64| {
            if t < 1e-3 {
                0.16 * (std::f64::consts::PI * t / 1e-3).sin().powi(2)
            } else {
                0.0
            }
        };
        let mut fv = Pipe1d::segmented(
            &[(4.0, area)],
            0.005,
            Prim {
                rho: rho0,
                u: 0.0,
                p: p0,
            },
        );
        fv.right = Boundary::Wall;
        let station = (2.0 / fv.dx).round() as usize;
        let mut pipe = RPipe::new(2.0, a_ref, dt, kappa, 1.0);
        let k = 0.5 * (kappa - 1.0);
        let rho_ref = kappa * p0 / (a_ref * a_ref);
        let cp = kappa / (kappa - 1.0) * r;
        let (mut p_fv, mut p_r) = (Vec::new(), Vec::new());
        for n in 0..1800 {
            let t = n as f64 * dt;
            // the Riemann pipe's near end: the state whose mass flux is ṁ against the arriving β
            let beta = pipe.at_near();
            let target = mdot(t) / (rho_ref * a_ref * area);
            let a = crate::exhaust_fv::rising_root(
                |a| a.powf(5.0) * (a - beta) / k - target,
                beta,
                beta + 0.5,
            );
            let u = (a - beta) / k * a_ref;
            p_r.push(p0 * pressure_ratio(0.5 * (pipe.at_far() + 1.0), kappa));
            pipe.send(2.0 * a - beta, 1.0);
            // the Euler pipe gets the same mass and total-enthalpy flux
            let m = mdot(t) / area;
            let h0 = cp * t_ref * a * a + 0.5 * u * u;
            fv.left = Boundary::Flux(flux_boundary(m, m * h0, fv.prim(0)));
            fv.step(dt);
            p_fv.push(fv.prim(station).p);
        }
        let peak = |x: &[f64]| x.iter().cloned().fold(0.0, f64::max) - p0;
        let half = |x: &[f64]| {
            let h = p0 + 0.5 * peak(x);
            x.iter().position(|&p| p > h).unwrap() as f64 * dt
        };
        let (pf, pr) = (peak(&p_fv), peak(&p_r));
        assert!(pf > 2e4, "a strong pulse: {pf} Pa");
        assert!(
            (pr / pf - 1.0).abs() < 0.05,
            "peak: Riemann {pr} vs Euler {pf}"
        );
        let (tf, tr) = (half(&p_fv), half(&p_r));
        assert!(
            (tr / tf - 1.0).abs() < 0.02,
            "arrival: Riemann {tr} vs Euler {tf}"
        );
    }
}
