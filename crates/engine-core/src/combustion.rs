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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

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
