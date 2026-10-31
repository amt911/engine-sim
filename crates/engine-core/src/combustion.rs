//! Combustion heat release: Wiebe burn fraction (Ghojel, IJER 2010;
//! Ferguson & Kirkpatrick use a = 5, m = 2) and fuel energy.

/// Lower heating value of gasoline, J/kg (Heywood, App. D).
pub const LHV_GASOLINE: f64 = 44.0e6;
/// Stoichiometric air/fuel ratio of gasoline.
pub const AFR_STOICH: f64 = 14.7;

pub const WIEBE_A: f64 = 5.0;
pub const WIEBE_M: f64 = 2.0;

/// Fraction of the charge burned `since` degrees after combustion start, for a burn
/// lasting `duration` degrees: x_b = 1 − exp(−a·(Δθ/θ_d)^(m+1)). 0 before start.
pub fn wiebe(since_deg: f64, duration_deg: f64) -> f64 {
    if since_deg <= 0.0 {
        return 0.0;
    }
    1.0 - (-WIEBE_A * (since_deg / duration_deg).powf(WIEBE_M + 1.0)).exp()
}

/// Chemical energy released by burning the fuel in a trapped premixed charge.
pub fn charge_energy(trapped_mass_kg: f64, afr: f64, efficiency: f64) -> f64 {
    trapped_mass_kg / (1.0 + afr) * LHV_GASOLINE * efficiency
}

/// Laminar burning velocity of a gasoline–air mixture, m/s: Metghalchi & Keck, Combust. Flame
/// 48 (1982), eq. 24 with the RMFD-303 (indolene) fit — S_L = S_L0(φ)·(T_u/298)^α·(p/1 atm)^β·
/// (1 − 2.1 f), S_L0 = B_m + B₂(φ − φ_m)² (Table 3A: 27.58, −78.34 cm/s, φ_m = 1.13),
/// α = 2.18 − 0.8(φ − 1), β = −0.16 + 0.22(φ − 1) (eqs. 16–17). `f` is the diluent (burned
/// residual) mass fraction; the fit covers 0–0.2 and is held at its f = 0.4 value beyond that.
pub fn laminar_flame_speed(phi: f64, t_u: f64, p_pa: f64, f: f64) -> f64 {
    let s0 = 0.2758 - 0.7834 * (phi - 1.13).powi(2);
    let alpha = 2.18 - 0.8 * (phi - 1.0);
    let beta = -0.16 + 0.22 * (phi - 1.0);
    let dilution = 1.0 - 2.1 * f.clamp(0.0, 0.4);
    s0 * (t_u / 298.0).powf(alpha) * (p_pa / 101_325.0).powf(beta) * dilution
}

/// The flame's crank-angle scale, Hires, Tabaczynski & Novak, SAE 780232 (1978), eqs. 13 and
/// 32: both the ignition delay and the combustion interval go as (S_p·ν)^⅓·(h/S_L)^⅔, with
/// `piston_speed` S_p (m/s), `nu` the unburned gas's kinematic viscosity (m²/s), `h` the
/// chamber height (m) and `s_l` the laminar flame speed (m/s).
pub fn flame_angle_scale(piston_speed: f64, nu: f64, h: f64, s_l: f64) -> f64 {
    (piston_speed * nu * (h / s_l).powi(2)).cbrt()
}

/// Lower heating value of diesel fuel, J/kg (Heywood, App. D).
pub const LHV_DIESEL: f64 = 42.5e6;
/// Stoichiometric air/fuel ratio of diesel fuel (Heywood, App. D).
pub const AFR_STOICH_DIESEL: f64 = 14.5;
/// Universal gas constant, J/(mol·K).
const R_UNIVERSAL: f64 = 8.3143;

/// Ignition delay of a direct-injection diesel, crank-angle degrees, in air at `p_bar` and
/// `t_k` with mean piston speed `piston_speed` m/s and fuel of `cetane` number: Hardenberg &
/// Hase (SAE 790493; Heywood, *ICE Fundamentals* 1988, eq. 10.37),
/// τ = (0.36 + 0.22·S̄p)·exp[E_A·(1/(R̃T) − 1/17 190) + (21.2/(p − 12.4))^0.63],
/// E_A = 618 840/(CN + 25) J/mol. Below 12.4 bar the correlation has no ignition: infinite.
pub fn ignition_delay_deg(p_bar: f64, t_k: f64, piston_speed: f64, cetane: f64) -> f64 {
    if p_bar <= 12.4 || t_k <= 0.0 {
        return f64::INFINITY;
    }
    let ea = 618_840.0 / (cetane + 25.0);
    let x = ea * (1.0 / (R_UNIVERSAL * t_k) - 1.0 / 17_190.0) + (21.2 / (p_bar - 12.4)).powf(0.63);
    (0.36 + 0.22 * piston_speed) * x.exp()
}

/// Watson's premixed share of the fuel (Watson, Pilley & Marzouk, SAE 800029; Heywood 1988
/// eq. 14.49): β = 1 − a·φ^b/τ^c with Watson's a = 0.926, b = 0.37, c = 0.26 (within the
/// book's ranges), φ the overall equivalence ratio and τ the delay in ms; kept in [0, 0.95].
pub fn watson_premixed_fraction(phi: f64, tau_ms: f64) -> f64 {
    (1.0 - 0.926 * phi.max(1e-3).powf(0.37) / tau_ms.max(1e-3).powf(0.26)).clamp(0.0, 0.95)
}

/// Watson's normalising combustion period, degrees (Heywood 1988, §14.4.3 footnote: 125°).
pub const WATSON_DURATION_DEG: f64 = 125.0;

/// The Watson double-Wiebe heat release of one injection (Heywood 1988 eqs. 14.47–14.48):
/// premixed f₁ = 1 − (1 − t^K₁)^K₂ and mixing-controlled f₂ = 1 − exp(−K₃·t^K₄), weighted
/// β and 1 − β; t is the time since ignition over [`WATSON_DURATION_DEG`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Watson {
    pub beta: f64,
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub k4: f64,
}

impl Watson {
    /// For an overall equivalence ratio `phi`, a delay of `tau_ms` and engine speed `rpm`:
    /// K₁ = 2 + 1.25·10⁻⁸·(τN)^2.4, K₂ = 5000, K₃ = 14.2/φ^0.644, K₄ = 0.79·K₃^0.25.
    pub fn new(phi: f64, tau_ms: f64, rpm: f64) -> Self {
        let phi = phi.max(1e-3);
        let k3 = 14.2 / phi.powf(0.644);
        Self {
            beta: watson_premixed_fraction(phi, tau_ms),
            k1: 2.0 + 1.25e-8 * (tau_ms * rpm).max(0.0).powf(2.4),
            k2: 5000.0,
            k3,
            k4: 0.79 * k3.powf(0.25),
        }
    }

    /// Fraction of the injected fuel burned at normalised time `t` since ignition.
    pub fn burned(&self, t: f64) -> f64 {
        if t <= 0.0 {
            return 0.0;
        }
        let t = t.min(1.0);
        let f1 = 1.0 - (1.0 - t.powf(self.k1)).powf(self.k2);
        let f2 = 1.0 - (-self.k3 * t.powf(self.k4)).exp();
        self.beta * f1 + (1.0 - self.beta) * f2
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    const ATM: f64 = 101_325.0;

    #[test]
    fn wiebe_reference_points() {
        assert_eq!(wiebe(-5.0, 50.0), 0.0);
        assert_eq!(wiebe(0.0, 50.0), 0.0);
        // half duration: 1 − e^(−5/8) = 0.464739 (hand)
        assert_relative_eq!(wiebe(25.0, 50.0), 0.464739, max_relative = 1e-5);
        // end of duration: 1 − e^(−5) = 0.993262
        assert_relative_eq!(wiebe(50.0, 50.0), 0.993262, max_relative = 1e-5);
    }

    #[test]
    fn charge_energy_reference() {
        // 2e-4 kg / 15.7 · 44 MJ/kg = 560.51 J
        assert_relative_eq!(charge_energy(2e-4, 14.7, 1.0), 560.51, max_relative = 1e-4);
        assert_relative_eq!(charge_energy(2e-4, 14.7, 0.5), 280.25, max_relative = 1e-4);
    }

    #[test]
    fn the_laminar_flame_is_metghalchi_and_kecks() {
        // RMFD-303 at φ = 1, 298 K, 1 atm, undiluted: S_L0 = 27.58 − 78.34·(1 − 1.13)²
        // = 27.58 − 1.3239 = 26.256 cm/s (Metghalchi & Keck 1982, Table 3A)
        assert_relative_eq!(
            laminar_flame_speed(1.0, 298.0, ATM, 0.0),
            0.262_561,
            max_relative = 1e-5
        );
        // 596 K, 4 atm, 10 % diluent: × 2^2.18 = 4.53154, × 4^−0.16 = 0.801070, × (1 − 0.21)
        // → 0.262561·4.53154·0.801070·0.79 = 0.752962 m/s (eqs. 16, 17, 24)
        assert_relative_eq!(
            laminar_flame_speed(1.0, 596.0, 4.0 * ATM, 0.1),
            0.752_962,
            max_relative = 1e-4
        );
        // past the fit, the dilution floors at f = 0.4: 1 − 0.84 = 0.16
        assert_relative_eq!(
            laminar_flame_speed(1.0, 298.0, ATM, 0.6),
            0.16 * 0.262_561,
            max_relative = 1e-5
        );
    }

    #[test]
    fn a_rich_flame_is_metghalchi_and_kecks_too() {
        // φ = 1.2, 298 K, 4 atm, undiluted: S_L0 = 27.58 − 78.34·(0.07)² = 27.196 cm/s;
        // α = 2.18 − 0.16 (no effect at 298 K); β = −0.16 + 0.22·0.2 = −0.116 (eq. 17):
        // 4^−0.116 = 0.851454 → 0.231562 m/s
        assert_relative_eq!(
            laminar_flame_speed(1.2, 298.0, 4.0 * ATM, 0.0),
            0.231_562,
            max_relative = 1e-4
        );
        // and its temperature exponent: 596 K, 1 atm: × 2^2.02 = 4.05583 → 1.103 03 m/s
        assert_relative_eq!(
            laminar_flame_speed(1.2, 596.0, ATM, 0.0),
            0.271_96 * 4.055_83,
            max_relative = 1e-4
        );
    }

    #[test]
    fn the_burn_angles_scale_as_hires() {
        // (S_p·ν)^⅓·(h/S_L)^⅔ at 10 m/s, 1e-6 m²/s, 1 cm, 1 m/s: (1e-5)^⅓·(1e-2)^⅔ = (1e-9)^⅓
        assert_relative_eq!(
            flame_angle_scale(10.0, 1e-6, 0.01, 1.0),
            1e-3,
            max_relative = 1e-9
        );
        let base = flame_angle_scale(10.0, 1e-6, 0.01, 1.0);
        // ×8 flame speed → 8^(−2/3) = 1/4; ×8 piston speed or ν → 8^(1/3) = 2; ×8 h → 4
        assert_relative_eq!(
            flame_angle_scale(10.0, 1e-6, 0.01, 8.0) / base,
            0.25,
            max_relative = 1e-9
        );
        assert_relative_eq!(
            flame_angle_scale(80.0, 1e-6, 0.01, 1.0) / base,
            2.0,
            max_relative = 1e-9
        );
        assert_relative_eq!(
            flame_angle_scale(10.0, 8e-6, 0.01, 1.0) / base,
            2.0,
            max_relative = 1e-9
        );
        assert_relative_eq!(
            flame_angle_scale(10.0, 1e-6, 0.08, 1.0) / base,
            4.0,
            max_relative = 1e-9
        );
    }

    proptest! {
        #[test]
        fn wiebe_monotonic_and_bounded(a in 0.0f64..200.0, b in 0.0f64..200.0, d in 10.0f64..90.0) {
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            let (xl, xh) = (wiebe(lo, d), wiebe(hi, d));
            prop_assert!(xl <= xh);
            prop_assert!((0.0..=1.0).contains(&xl) && (0.0..=1.0).contains(&xh));
        }
    }

    #[test]
    fn hardenberg_hase_delay_matches_the_hand_value() {
        // CN 45, 60 bar, 900 K, 8 m/s (Heywood 1988 eq. 10.37): E_A = 618 840/70 = 8840.57
        // J/mol; E_A·(1/(8.3143·900) − 1/17 190) = 0.66714; (21.2/47.6)^0.63 = 0.60077;
        // (0.36 + 0.22·8)·e^1.26791 = 2.12·3.55325 = 7.533 °CA
        let d = ignition_delay_deg(60.0, 900.0, 8.0, 45.0);
        assert!((d - 7.533).abs() < 0.005, "delay {d}");
        // hotter and denser air lights sooner; a higher cetane fuel too
        assert!(ignition_delay_deg(60.0, 1000.0, 8.0, 45.0) < d);
        assert!(ignition_delay_deg(80.0, 900.0, 8.0, 45.0) < d);
        assert!(ignition_delay_deg(60.0, 900.0, 8.0, 55.0) < d);
    }

    #[test]
    fn watson_premixed_share_matches_the_hand_value() {
        // φ 0.6, τ 1 ms: β = 1 − 0.926·0.6^0.37/1^0.26 = 0.23347; a longer delay mixes more
        let b = watson_premixed_fraction(0.6, 1.0);
        assert!((b - 0.23347).abs() < 1e-4, "beta {b}");
        assert!(watson_premixed_fraction(0.6, 3.0) > b);
        // never negative nor all of it
        assert!((0.0..1.0).contains(&watson_premixed_fraction(1.2, 0.05)));
    }

    #[test]
    fn watson_burn_matches_its_closed_forms() {
        // φ 0.6, τ 1 ms at 2000 rpm: K1 = 2 + 1.25e-8·2000^2.4 = 3.04564, premixed
        // f1(0.05) = 1 − (1 − 0.05^K1)^5000 = 0.42025; K3 = 14.2/0.6^0.644 = 19.7314,
        // K4 = 0.79·K3^0.25 = 1.66501, diffusion f2(0.3) = 1 − e^(−K3·0.3^K4) = 0.92991
        let (beta, k1, k3, k4) = (0.5, 3.04564, 19.7314, 1.66501);
        let w = Watson::new(0.6, 1.0, 2000.0);
        assert!((w.k1 - k1).abs() < 1e-4 && (w.k3 - k3).abs() < 1e-3 && (w.k4 - k4).abs() < 1e-4);
        let w = Watson { beta, ..w };
        let x = w.burned(0.05);
        let f2 = 1.0 - (-k3 * 0.05f64.powf(k4)).exp();
        assert!((x - (beta * 0.42025 + (1.0 - beta) * f2)).abs() < 1e-4, "x {x}");
        assert!((w.burned(0.3) - (beta * 1.0 + (1.0 - beta) * 0.92991)).abs() < 1e-4);
        assert_eq!(w.burned(0.0), 0.0);
        assert!((w.burned(1.0) - 1.0).abs() < 1e-6);
    }
}
