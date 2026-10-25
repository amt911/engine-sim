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
}
