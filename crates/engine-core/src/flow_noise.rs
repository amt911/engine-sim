//! Flow noise of an obstruction in a duct: the random sound the turbulent wake of a junction,
//! an area step or a plate generates in the pipe ("regenerated" or "self" noise).
//!
//! Nelson & Morfey (*Aerodynamic sound production in low speed flow ducts*, JSV 79(2), 1981):
//! the wake's fluctuating drag is a dipole whose rms value in each proportional band is a
//! Strouhal-number function K(St) times the steady drag, so the sound power follows from the
//! obstruction's pressure loss. Oldham & Ukpoho (*A pressure-based technique for predicting
//! regenerated noise levels in ventilation systems*, JSV 140(2), 1990) write it from the
//! pressure-loss coefficient alone, for circular ducts and orifice-like elements, and give the
//! trend line K²(St) this module uses.

use std::f64::consts::PI;

/// Highest constriction Mach number the dipole model holds to: above it Gordon (JASA 45, 1969)
/// found a quadrupole (Nelson & Morfey §4.5). The noise's velocity stops here.
const MAX_MACH: f64 = 0.3;

/// Oldham & Ukpoho's Fig. 9 trend line (dampers and orifice plates): (St, K²(St) in dB re
/// 10⁻¹²), digitized. It agrees with Nelson & Morfey's Fig. 9 after their 2009 erratum (−6 dB).
const K2_DB: [(f64, f64); 7] = [
    (0.1, 62.0),
    (1.0, 61.0),
    (3.0, 59.0),
    (5.0, 53.0),
    (10.0, 44.0),
    (20.0, 33.0),
    (50.0, 22.0),
];

/// An obstruction as the flow sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spoiler {
    /// Open-area ratio σ (Oldham eq. 11).
    pub sigma: f64,
    /// Velocity through the constriction, U/σ (eq. 12), m/s; capped at Mach 0.3.
    pub u_c: f64,
    /// Drag coefficient C_L/(1 − σ) (eq. 9).
    pub c_d: f64,
    /// Characteristic dimension D(1 − σ)/2 of an orifice-like element in a circular duct (§3.2), m.
    pub d: f64,
}

/// The obstruction of pressure-loss coefficient `c_l` = Δp/(½ρU²) met by a flow of speed `u` (m/s)
/// in a circular duct of diameter `duct_d` (m), in gas whose speed of sound is `c`. None where
/// nothing obstructs or nothing flows.
pub fn spoiler(c_l: f64, u: f64, duct_d: f64, c: f64) -> Option<Spoiler> {
    if c_l <= 0.0 || u == 0.0 || !c_l.is_finite() || !u.is_finite() {
        return None;
    }
    let sigma = 1.0 / (c_l.sqrt() + 1.0);
    Some(Spoiler {
        sigma,
        u_c: (u.abs() / sigma).min(MAX_MACH * c),
        c_d: c_l / (1.0 - sigma),
        d: duct_d * (1.0 - sigma) / 2.0,
    })
}

/// K²(St), the ratio (not dB), log-interpolated in St on the trend line, held flat beyond it.
pub fn k2(st: f64) -> f64 {
    let db = match K2_DB.iter().position(|&(s, _)| s >= st) {
        Some(0) => K2_DB[0].1,
        None => K2_DB[K2_DB.len() - 1].1,
        Some(i) => {
            let ((s0, l0), (s1, l1)) = (K2_DB[i - 1], K2_DB[i]);
            l0 + (l1 - l0) * (st / s0).log10() / (s1 / s0).log10()
        }
    };
    10f64.powf(db / 10.0) * 1e-12
}

/// Sound power (W) generated in the third octave centred at `f_c` (Hz), both directions of the
/// duct together, in gas of density `rho` and speed of sound `c`. Below the duct's first cut-on
/// f₀ = 1.84c/(πD) plane waves: W = ρAσ⁴C_D²U_c⁴K²/(16c) (Oldham eq. 16); above it
/// W = ρπA²St²σ⁴C_D²U_c⁶K²/(24c³d²)·[1 + 3c/(8πf_cD)] (eq. 17, with its edge-mode term).
pub fn band_power(sp: &Spoiler, f_c: f64, duct_d: f64, rho: f64, c: f64) -> f64 {
    let area = PI * 0.25 * duct_d * duct_d;
    let st = f_c * sp.d / sp.u_c;
    let drag = sp.sigma.powi(4) * sp.c_d * sp.c_d;
    let f0 = 1.84 * c / (PI * duct_d);
    if f_c < f0 {
        rho * area * drag * sp.u_c.powi(4) * k2(st) / (16.0 * c)
    } else {
        rho * PI * area * area * st * st * drag * sp.u_c.powi(6) * k2(st)
            / (24.0 * c.powi(3) * sp.d * sp.d)
            * (1.0 + 3.0 * c / (8.0 * PI * f_c * duct_d))
    }
}

/// Pressure-loss coefficient of a sudden expansion from `a_small` to `a_large` (m²), on the small
/// pipe's velocity: Borda–Carnot, (1 − A₁/A₂)² (White, *Fluid Mechanics*, §6.9).
pub fn expansion_cl(a_small: f64, a_large: f64) -> f64 {
    (1.0 - a_small / a_large).powi(2)
}

/// Pressure-loss coefficient of a sudden contraction into `a_small`, on its velocity: the vena
/// contracta's C_c = 0.63 + 0.37·(A₁/A₂)³ (Weisbach), C_L = (1/C_c − 1)².
pub fn contraction_cl(a_small: f64, a_large: f64) -> f64 {
    let cc = 0.63 + 0.37 * (a_small / a_large).powi(3);
    (1.0 / cc - 1.0).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn spoiler_from_the_pressure_loss() {
        // C_L = 4 (Oldham eq. 11): σ = 1/(√4 + 1) = 1/3; U_c = U/σ = 30 m/s (eq. 12);
        // C_D = C_L/(1 − σ) = 6 (eq. 9); d = D(1 − σ)/2 = 0.2·(2/3)/2 = 0.0667 m (§3.2)
        let sp = spoiler(4.0, 10.0, 0.2, 343.0).unwrap();
        assert_relative_eq!(sp.sigma, 1.0 / 3.0, max_relative = 1e-12);
        assert_relative_eq!(sp.u_c, 30.0, max_relative = 1e-12);
        assert_relative_eq!(sp.c_d, 6.0, max_relative = 1e-12);
        assert_relative_eq!(sp.d, 0.2 / 3.0, max_relative = 1e-12);
    }

    #[test]
    fn k2_follows_oldhams_trend_line() {
        // Fig. 9's trend line, dB re 1e-12: 61 at St 1, 44 at St 10, held flat beyond its ends;
        // at St √10, log-interpolated between (3, 59) and (5, 53): 58.381 dB
        assert_relative_eq!(k2(1.0), 10f64.powf(6.1) * 1e-12, max_relative = 1e-12);
        assert_relative_eq!(k2(10.0), 10f64.powf(4.4) * 1e-12, max_relative = 1e-12);
        assert_eq!(k2(0.01), k2(0.1));
        assert_eq!(k2(500.0), k2(50.0));
        assert_relative_eq!(
            10.0 * (k2(10f64.sqrt()) / 1e-12).log10(),
            58.381_234,
            max_relative = 1e-6
        );
    }

    /// The test spoiler: σ = 1/3, U_c = 30, C_D = 6 in a 200 mm duct.
    fn test_spoiler() -> Spoiler {
        spoiler(4.0, 10.0, 0.2, 343.0).unwrap()
    }

    #[test]
    fn band_power_below_cut_on_by_hand() {
        // f₀ = 1.84·343/(π·0.2) = 1004.5 Hz. At 250 Hz, St = 250·0.0667/30 = 0.5556, K² =
        // 61.2553 dB; eq. 16: W = ρAσ⁴C_D²U_c⁴K²/(16c) = 1.2·0.031416·(1/81)·36·810000·
        // 1.3354e-6/5488 = 3.30177e-6 W
        let w = band_power(&test_spoiler(), 250.0, 0.2, 1.2, 343.0);
        assert_relative_eq!(w, 3.301_769_6e-6, max_relative = 1e-6);
    }

    #[test]
    fn band_power_above_cut_on_by_hand() {
        // At 2000 Hz, St = 4.444, K² = 54.3834 dB; eq. 17: W = ρπA²St²σ⁴C_D²U_c⁶K²/(24c³d²)
        // ·[1 + 3c/(8πfD)] = 1.67327e-6 W
        let w = band_power(&test_spoiler(), 2000.0, 0.2, 1.2, 343.0);
        assert_relative_eq!(w, 1.673_270_5e-6, max_relative = 1e-6);
    }

    #[test]
    fn velocity_scaling() {
        // at a fixed Strouhal number, doubling the flow raises the power by 2⁴ (12.04 dB) in plane
        // waves and by 2⁶ (18.06 dB) above cut-on, less the edge-mode term's change
        let (d, rho, c) = (0.2, 1.2, 343.0);
        let a = spoiler(4.0, 10.0, d, c).unwrap();
        let b = spoiler(4.0, 20.0, d, c).unwrap();
        let below = band_power(&b, 200.0, d, rho, c) / band_power(&a, 100.0, d, rho, c);
        assert_relative_eq!(10.0 * below.log10(), 12.041, max_relative = 1e-3);
        let edge = |f: f64| 1.0 + 3.0 * c / (8.0 * std::f64::consts::PI * f * d);
        let above = band_power(&b, 4000.0, d, rho, c) / band_power(&a, 2000.0, d, rho, c);
        assert_relative_eq!(
            above,
            64.0 * edge(4000.0) / edge(2000.0),
            max_relative = 1e-9
        );
    }

    #[test]
    fn no_obstruction_no_noise() {
        assert!(spoiler(0.0, 10.0, 0.05, 600.0).is_none());
        assert!(spoiler(-1.0, 10.0, 0.05, 600.0).is_none());
        assert!(spoiler(2.0, 0.0, 0.05, 600.0).is_none());
    }

    #[test]
    fn mach_cap() {
        // the dipole model holds to a constriction Mach number of 0.3 (Nelson & Morfey §4.5,
        // after Gordon 1969): the noise's U_c stops there
        assert_eq!(spoiler(4.0, 500.0, 0.05, 600.0).unwrap().u_c, 0.3 * 600.0);
        assert!(
            band_power(
                &spoiler(4.0, 1e9, 0.05, 600.0).unwrap(),
                500.0,
                0.05,
                0.4,
                600.0
            )
            .is_finite()
        );
    }

    #[test]
    fn area_change_coefficients() {
        // Borda–Carnot: (1 − 1/4)² = 0.5625; Weisbach: C_c = 0.63 + 0.37/64 = 0.635781, so
        // C_L = (1/C_c − 1)² = 0.328178
        assert_eq!(expansion_cl(1.0, 4.0), 0.5625);
        assert_relative_eq!(contraction_cl(1.0, 4.0), 0.328_177_8, max_relative = 1e-6);
        assert_eq!(expansion_cl(1.0, 1.0), 0.0);
    }
}
