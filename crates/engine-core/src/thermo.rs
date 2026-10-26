//! Thermodynamic properties of the cylinder's gas: the unburned charge (air and gasoline vapour)
//! and the burned gas (frozen products), each with specific heats that rise with temperature.
//! A constant γ = 1.4 (cold air) makes the cycle an air-standard Otto cycle, 62 % efficient at
//! 11.5:1; real charges and products (γ ≈ 1.35 falling to ≈ 1.25) give the fuel–air cycle's
//! ≈ 47 % (Heywood, *ICE Fundamentals*, 2nd ed., §4.7 and ch. 5).
//!
//! Species specific heats are the NASA polynomials of Heywood Table 4.10 (JANAF data),
//! c_p/R = a₁ + a₂T + a₃T² + a₄T³ + a₅T⁴ in two ranges split at 1000 K; gasoline vapour
//! (C₈.₂₆H₁₅.₅) is Heywood Table 4.11, c_p = A₁ + A₂t + A₃t² + A₄t³ + A₅/t² cal/(gmol·K) with
//! t = T/1000. The unburned mixture is stoichiometric air and fuel (Table 4.4: 3.773 N₂ per O₂,
//! 138.2 kg of air per kmol O₂; A/F 14.64); the burned one is Table 4.7's frozen φ = 1 products,
//! per kg of air 4.64 mol CO₂, 5.2 mol H₂O, 27.3 mol N₂. Dissociation above ~1700 K is not
//! modelled (Heywood §4.7.2): the burned gas is a little too hot at its peak.
//!
//! Internal energy is sensible, with the datum the rest of the model uses: u(300 K) = c_v,air ·
//! 300 for both mixtures, so a combustion adds the fuel's heating value on top.

/// Universal gas constant, J/(mol·K).
const R_UNIVERSAL: f64 = 8.314_462_618;
/// cal → J.
const CAL: f64 = 4.184;
/// The model's reference temperature, where every mixture's internal energy is c_v,air·T.
const T_DATUM: f64 = 300.0;
/// Where the NASA polynomials switch range.
const T_SPLIT: f64 = 1000.0;

/// A species' c_p/R coefficients (a₁..a₅): (1000–5000 K, 300–1000 K). Heywood Table 4.10.
type Nasa = ([f64; 5], [f64; 5]);
const CO2: Nasa = (
    [0.44608e1, 0.30982e-2, -0.12393e-5, 0.22741e-9, -0.15526e-13],
    [0.24008e1, 0.87351e-2, -0.66071e-5, 0.20022e-8, 0.63274e-15],
);
const H2O: Nasa = (
    [0.27168e1, 0.29451e-2, -0.80224e-6, 0.10227e-9, -0.48472e-14],
    [0.40701e1, -0.11084e-2, 0.41521e-5, -0.29637e-8, 0.80702e-12],
);
const O2: Nasa = (
    [
        0.36220e1,
        0.73618e-3,
        -0.19652e-6,
        0.36202e-10,
        -0.28946e-14,
    ],
    [0.36256e1, -0.18782e-2, 0.70555e-5, -0.67635e-8, 0.21556e-11],
);
const N2: Nasa = (
    [
        0.28963e1,
        0.15155e-2,
        -0.57235e-6,
        0.99807e-10,
        -0.65224e-14,
    ],
    [
        0.36748e1,
        -0.12082e-2,
        0.23240e-5,
        -0.63218e-9,
        -0.22577e-12,
    ],
);
/// Gasoline C₈.₂₆H₁₅.₅ vapour, A_f1..A_f5 (Heywood Table 4.11), molar mass 114.8 g/mol, A/F 14.64.
const GASOLINE: [f64; 5] = [-24.078, 256.63, -201.68, 64.750, 0.5808];
const GASOLINE_KG_MOL: f64 = 0.1148;
const AFR: f64 = 14.64;
/// kg of air per mol of O₂ (Heywood Table 4.5: 138.2 kg/kmol).
const AIR_KG_PER_MOL_O2: f64 = 0.1382;

/// A gas mixture's specific heat per kg, c_p(T) = Σ cₖTᵏ (k = 0..4) + c₋₂/T², in two ranges, and
/// its gas constant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mixture {
    /// J/(kg·K) coefficients for T ≥ 1000 K and below: [c₀, c₁, c₂, c₃, c₄, c₋₂].
    hi: [f64; 6],
    lo: [f64; 6],
    /// Gas constant, J/(kg·K).
    pub r: f64,
    /// Enthalpy offsets (per range) that put u(300 K) at the datum and keep h continuous.
    h_lo: f64,
    h_hi: f64,
}

/// c_p(T) and u(T) of one range from its coefficients, sharing T's powers and one division:
/// h = T(c₀ + T(c₁/2 + T(c₂/3 + T(c₃/4 + T·c₄/5)))) − c₋₂/T + offset.
#[inline]
fn cp_u(c: &[f64; 6], h_off: f64, r: f64, t: f64) -> (f64, f64) {
    let inv = 1.0 / t;
    let cp = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * c[4]))) + c[5] * inv * inv;
    let h = t
        * (c[0] + t * (c[1] * 0.5 + t * (c[2] * (1.0 / 3.0) + t * (c[3] * 0.25 + t * c[4] * 0.2))))
        - c[5] * inv
        + h_off;
    (cp - r, h - r * t)
}

impl Mixture {
    /// From moles of each species per `mass_kg` of mixture, plus moles of gasoline vapour.
    fn new(species: &[(Nasa, f64)], fuel_mol: f64, mass_kg: f64) -> Self {
        let mut hi = [0.0; 6];
        let mut lo = [0.0; 6];
        let mut mol = fuel_mol;
        for &((a_hi, a_lo), n) in species {
            mol += n;
            for k in 0..5 {
                hi[k] += n * R_UNIVERSAL * a_hi[k] / mass_kg;
                lo[k] += n * R_UNIVERSAL * a_lo[k] / mass_kg;
            }
        }
        // gasoline: c_p = A₁ + A₂(T/1000) + A₃(T/1000)² + A₄(T/1000)³ + A₅(1000/T)², cal/(mol·K)
        let f = fuel_mol * CAL / mass_kg;
        let fuel = [
            GASOLINE[0] * f,
            GASOLINE[1] * f * 1e-3,
            GASOLINE[2] * f * 1e-6,
            GASOLINE[3] * f * 1e-9,
            0.0,
            GASOLINE[4] * f * 1e6,
        ];
        for k in 0..6 {
            hi[k] += fuel[k];
            lo[k] += fuel[k];
        }
        let r = mol * R_UNIVERSAL / mass_kg;
        let mut m = Self {
            hi,
            lo,
            r,
            h_lo: 0.0,
            h_hi: 0.0,
        };
        // u(300) = c_v,air·300 → h(300) = c_v,air·300 + R·300; then h continuous at 1000 K
        m.h_lo = crate::gas::CV * T_DATUM + r * T_DATUM - Self::poly_h(&lo, T_DATUM);
        m.h_hi = m.h_lo + Self::poly_h(&lo, T_SPLIT) - Self::poly_h(&hi, T_SPLIT);
        m
    }

    /// The mixture of `parts` (mixture, mass share): every property here is linear in the
    /// mass shares, so the mixture's coefficients are the weighted sums.
    pub fn mix(parts: &[(Mixture, f64)]) -> Self {
        let mut m = Self {
            hi: [0.0; 6],
            lo: [0.0; 6],
            r: 0.0,
            h_lo: 0.0,
            h_hi: 0.0,
        };
        for (p, w) in parts {
            for k in 0..6 {
                m.hi[k] += w * p.hi[k];
                m.lo[k] += w * p.lo[k];
            }
            m.r += w * p.r;
            m.h_lo += w * p.h_lo;
            m.h_hi += w * p.h_hi;
        }
        m
    }

    /// (c_v, u) at `t`: one evaluation for Newton.
    #[inline]
    pub fn cv_u(&self, t: f64) -> (f64, f64) {
        if t >= T_SPLIT {
            cp_u(&self.hi, self.h_hi, self.r, t)
        } else {
            cp_u(&self.lo, self.h_lo, self.r, t)
        }
    }

    /// The temperature whose internal energy is `u` (J/kg), by Newton from `guess`. Newton
    /// converges quadratically: after a step of δ the error is ≈ (c_v′/2c_v)·δ² ≈ 2e-4 K/K · δ²,
    /// so a step under 0.1 % of T (≲ 3 K) leaves well under 0.01 K — and from the last step's
    /// temperature that is the first step.
    pub fn temperature(&self, u: f64, guess: f64) -> f64 {
        let mut t = guess.clamp(T_MIN, T_MAX);
        for _ in 0..12 {
            let (cv, ut) = self.cv_u(t);
            let step = (ut - u) / cv;
            t = (t - step).clamp(T_MIN, T_MAX);
            if step.abs() < 1e-3 * t {
                break;
            }
        }
        t
    }

    fn poly_cp(c: &[f64; 6], t: f64) -> f64 {
        c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * c[4]))) + c[5] / (t * t)
    }

    /// ∫c_p dT without its constant.
    fn poly_h(c: &[f64; 6], t: f64) -> f64 {
        t * (c[0] + t * (c[1] / 2.0 + t * (c[2] / 3.0 + t * (c[3] / 4.0 + t * c[4] / 5.0))))
            - c[5] / t
    }

    /// Specific heat at constant pressure, J/(kg·K).
    pub fn cp(&self, t: f64) -> f64 {
        if t >= T_SPLIT {
            Self::poly_cp(&self.hi, t)
        } else {
            Self::poly_cp(&self.lo, t)
        }
    }

    pub fn cv(&self, t: f64) -> f64 {
        self.cp(t) - self.r
    }

    /// Sensible enthalpy, J/kg (datum: u(300 K) = c_v,air · 300).
    pub fn h(&self, t: f64) -> f64 {
        if t >= T_SPLIT {
            Self::poly_h(&self.hi, t) + self.h_hi
        } else {
            Self::poly_h(&self.lo, t) + self.h_lo
        }
    }

    /// Sensible internal energy, J/kg.
    pub fn u(&self, t: f64) -> f64 {
        self.h(t) - self.r * t
    }
}

/// Stoichiometric air and gasoline vapour, per kg of mixture (Heywood Tables 4.4, 4.5, 4.11).
pub fn unburned() -> Mixture {
    let o2 = 1.0 / AIR_KG_PER_MOL_O2;
    let mass = 1.0 + 1.0 / AFR;
    Mixture::new(
        &[(O2, o2), (N2, 3.773 * o2)],
        1.0 / AFR / GASOLINE_KG_MOL,
        mass,
    )
}

/// Frozen stoichiometric products, per kg (Heywood Table 4.7, φ = 1).
pub fn burned() -> Mixture {
    Mixture::new(&[(CO2, 4.64), (H2O, 5.2), (N2, 27.3)], 0.0, 1.0 + 1.0 / AFR)
}

/// Air alone (Table 4.4, φ = 0): for checks against the cold-air model.
pub fn air() -> Mixture {
    let o2 = 1.0 / AIR_KG_PER_MOL_O2;
    Mixture::new(&[(O2, o2), (N2, 3.773 * o2)], 0.0, 1.0)
}

/// What the cylinder's gas is made of, by mass: `burned` products, and of the rest, the share
/// `fuelled` that is stoichiometric fuel and air (the remainder is air: a cut injection).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Composition {
    pub burned: f64,
    pub fuelled: f64,
}

impl Composition {
    /// Mass weights of (air, stoichiometric charge, products).
    pub fn weights(self) -> (f64, f64, f64) {
        let fresh = 1.0 - self.burned;
        (
            fresh * (1.0 - self.fuelled),
            fresh * self.fuelled,
            self.burned,
        )
    }
}

/// The cylinder's gas: air, stoichiometric charge and burned gas mixed by mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderGas {
    pub air: Mixture,
    pub unburned: Mixture,
    pub burned: Mixture,
}

impl Default for CylinderGas {
    fn default() -> Self {
        Self {
            air: air(),
            unburned: unburned(),
            burned: burned(),
        }
    }
}

const T_MIN: f64 = 150.0;
const T_MAX: f64 = 5000.0;

impl CylinderGas {
    /// The gas of composition `c` as one mixture.
    pub fn mixture(&self, c: Composition) -> Mixture {
        let (a, u, b) = c.weights();
        let (x, y, z) = (&self.air, &self.unburned, &self.burned);
        let mut m = *x;
        for k in 0..6 {
            m.hi[k] = a * x.hi[k] + u * y.hi[k] + b * z.hi[k];
            m.lo[k] = a * x.lo[k] + u * y.lo[k] + b * z.lo[k];
        }
        m.r = a * x.r + u * y.r + b * z.r;
        m.h_lo = a * x.h_lo + u * y.h_lo + b * z.h_lo;
        m.h_hi = a * x.h_hi + u * y.h_hi + b * z.h_hi;
        m
    }
    pub fn r(&self, c: Composition) -> f64 {
        self.mixture(c).r
    }
    pub fn u(&self, t: f64, c: Composition) -> f64 {
        self.mixture(c).u(t)
    }
    pub fn h(&self, t: f64, c: Composition) -> f64 {
        self.mixture(c).h(t)
    }
    pub fn cv(&self, t: f64, c: Composition) -> f64 {
        self.mixture(c).cv(t)
    }
    pub fn gamma(&self, t: f64, c: Composition) -> f64 {
        1.0 + self.r(c) / self.cv(t, c)
    }
    pub fn temperature(&self, u: f64, c: Composition, guess: f64) -> f64 {
        self.mixture(c).temperature(u, guess)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn species_specific_heats_are_janafs() {
        // CO₂ at 1000 K from Table 4.10's low range by hand: 2.4008 + 8.7351 − 6.6071 + 2.0022
        // + 0.0006 = 6.5316 (Heywood Fig. 4.11 reads ≈ 6.6); N₂ at 2000 K, high range:
        // 2.8963 + 3.0310 − 2.2894 + 0.7985 − 0.1044 = 4.3320 (Fig. 4.11 ≈ 4.3)
        let co2 = Mixture::new(&[(CO2, 1.0)], 0.0, 1.0);
        assert_abs_diff_eq!(co2.cp(999.999_999) / R_UNIVERSAL, 6.5316, epsilon = 1e-3);
        let n2 = Mixture::new(&[(N2, 1.0)], 0.0, 1.0);
        assert_abs_diff_eq!(n2.cp(2000.0) / R_UNIVERSAL, 4.3320, epsilon = 1e-3);
    }

    #[test]
    fn gammas_follow_heywoods_figure_4_13() {
        // Heywood Fig. 4.13, x_b = 0: air (φ = 0) 1.40 at 300 K and 1.335 at 1000 K; gasoline
        // and air at φ = 1, 1.355 at 300 K and 1.267 at 1000 K (read off the chart, ±0.005)
        let g = |m: Mixture, t: f64| m.cp(t) / m.cv(t);
        assert_abs_diff_eq!(g(air(), 300.0), 1.400, epsilon = 0.005);
        assert_abs_diff_eq!(g(air(), 1000.0), 1.335, epsilon = 0.005);
        assert_abs_diff_eq!(g(unburned(), 300.0), 1.355, epsilon = 0.005);
        assert_abs_diff_eq!(g(unburned(), 1000.0), 1.267, epsilon = 0.005);
    }

    #[test]
    fn gas_constants_come_from_the_moles() {
        // burned: 37.1 mol per 1.0683 kg (Table 4.7) → 8.3145 · 37.14 / 1.06831 = 289.06;
        // unburned: (7.2359 + 27.301 + 0.59500) mol per 1.06831 kg → 273.43; air 287.0
        assert_abs_diff_eq!(burned().r, 289.06, epsilon = 0.05);
        assert_abs_diff_eq!(unburned().r, 273.43, epsilon = 0.05);
        assert_abs_diff_eq!(air().r, 287.0, epsilon = 0.3);
    }

    #[test]
    fn every_mixture_shares_the_models_datum_and_enthalpy_is_continuous() {
        for m in [unburned(), burned(), air()] {
            assert_abs_diff_eq!(m.u(300.0), crate::gas::CV * 300.0, epsilon = 1e-6);
            assert_abs_diff_eq!(m.h(1000.0 - 1e-9), m.h(1000.0), epsilon = 1e-3);
            // and dh/dT is c_p
            let d = (m.h(1500.0 + 1e-3) - m.h(1500.0 - 1e-3)) / 2e-3;
            assert_abs_diff_eq!(d, m.cp(1500.0), epsilon = 1e-3);
        }
    }

    #[test]
    fn a_cut_charge_is_air_and_a_fuelled_one_carries_fuel_vapour() {
        let g = CylinderGas::default();
        let cut = Composition {
            burned: 0.0,
            fuelled: 0.0,
        };
        assert_eq!(g.r(cut), air().r);
        assert_abs_diff_eq!(g.gamma(300.0, cut), 1.400, epsilon = 0.005);
        // half and half by mass: the mean of their gas constants (287.0 and 273.43)
        let half = Composition {
            burned: 0.0,
            fuelled: 0.5,
        };
        assert_abs_diff_eq!(g.r(half), 0.5 * (air().r + unburned().r), epsilon = 1e-9);
        // products are products whatever the fresh part was
        let burnt = |fuelled| Composition {
            burned: 1.0,
            fuelled,
        };
        assert_eq!(g.r(burnt(0.0)), g.r(burnt(1.0)));
    }

    #[test]
    fn the_fused_evaluation_matches_the_separate_ones() {
        for m in [air(), unburned(), burned()] {
            for t in [250.0, 800.0, 999.0, 1000.0, 2400.0] {
                let (cv, u) = m.cv_u(t);
                assert_abs_diff_eq!(cv, m.cv(t), epsilon = 1e-9);
                assert_abs_diff_eq!(u, m.u(t), epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn a_mixture_is_the_mass_weighted_sum_of_its_parts() {
        // Mixing is linear in mass: c_p, u, h and R of a 30/70 blend are the weighted means
        let (a, b) = (air(), burned());
        let m = Mixture::mix(&[(a, 0.3), (b, 0.7)]);
        for t in [400.0, 1500.0] {
            assert_abs_diff_eq!(m.cp(t), 0.3 * a.cp(t) + 0.7 * b.cp(t), epsilon = 1e-9);
            assert_abs_diff_eq!(m.u(t), 0.3 * a.u(t) + 0.7 * b.u(t), epsilon = 1e-6);
        }
        assert_abs_diff_eq!(m.r, 0.3 * a.r + 0.7 * b.r, epsilon = 1e-12);
    }

    #[test]
    fn temperature_inverts_internal_energy() {
        let g = CylinderGas::default();
        for (t, burned, fuelled) in [
            (300.0, 0.0, 1.0),
            (750.0, 0.08, 1.0),
            (2600.0, 1.0, 1.0),
            (1200.0, 0.5, 0.0),
            (600.0, 0.0, 0.3),
        ] {
            let c = Composition { burned, fuelled };
            let u = g.u(t, c);
            for guess in [t, 400.0, 3000.0] {
                assert_abs_diff_eq!(g.temperature(u, c, guess), t, epsilon = 1e-2);
            }
        }
    }

    #[test]
    fn the_fuel_air_otto_cycle_is_far_below_the_air_standard_one() {
        // Isentropic compression 11.5:1 from 330 K, constant-volume burn of the stoichiometric
        // heating value (44 MJ/kg fuel per 15.64 kg of mixture, 96 % burnt), expansion back:
        // the fuel-air cycle. Heywood ch. 5 (Fig. 5.9) puts it near 47 % at this ratio, against
        // 1 − 11.5^−0.4 = 62.4 % for cold air
        let g = CylinderGas::default();
        let mix = |burned: f64| Composition {
            burned,
            fuelled: 1.0,
        };
        let isentropic = |t0: f64, ratio: f64, xb: f64| {
            // dT/T = −(R/c_v) dV/V in small steps
            let n = 2000;
            let mut t = t0;
            for _ in 0..n {
                t *= ratio.powf(g.r(mix(xb)) / g.cv(t, mix(xb)) / n as f64);
            }
            t
        };
        let rc = 11.5;
        let t2 = isentropic(330.0, rc, 0.0);
        let q = 44.0e6 / (1.0 + AFR) * 0.96;
        let t3 = g.temperature(g.u(t2, mix(0.0)) + q, mix(1.0), 2500.0);
        let t4 = isentropic(t3, 1.0 / rc, 1.0);
        let w =
            (g.u(t3, mix(1.0)) - g.u(t4, mix(1.0))) - (g.u(t2, mix(0.0)) - g.u(330.0, mix(0.0)));
        let eta = w / q * 0.96;
        assert!((0.45..0.49).contains(&eta), "fuel-air cycle {eta}");
    }
}
