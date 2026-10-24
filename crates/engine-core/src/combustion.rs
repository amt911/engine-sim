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
}
