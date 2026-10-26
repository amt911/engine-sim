//! Sound in narrow channels — a catalyst's monolith: viscosity and heat conduction at the walls
//! (Zwikker & Kosten, *Sound Absorbing Materials*, 1949; Stinson, JASA 89, 1991; for catalysts
//! Dokumaci, JSV 210, 1998). A circular tube of radius a carries plane waves with an effective
//! density ρ/(1 − F(k_v·a)) and compressibility (1 + (γ − 1)·F(k_t·a))/(γp), F(z) =
//! 2J₁(z)/(z·J₀(z)), k_v = √(−iω/ν), k_t = √(−iω·Pr/ν). Square channels are taken as circular of
//! their hydraulic radius (Dokumaci finds the difference small).
//!
//! For the time domain, the excess over the lossless tube is a passive Foster network in the
//! dimensionless frequency x = ωa²/ν, fitted once by `scripts/fit_narrow_tube.py` (non-negative
//! least squares, 0.3 ≤ a·√(ω/ν) ≤ 60): the series impedance r₀ + Σ rₖ·ixτₖ/(1 + ixτₖ) in units
//! of μ/a² per open area, the shunt admittance Σ gₖ·ixτₖ/(1 + ixτₖ) in units of
//! (γ − 1)ν/(γpa²) per open area. Within 1 % of the exact functions over that band.

/// Prandtl number the thermal fit assumes: air and exhaust gas, nearly constant with
/// temperature (0.70–0.72 from 300 to 1000 K).
pub const PRANDTL: f64 = 0.71;

/// `scripts/fit_narrow_tube.py`, 10 time constants (the zero branches dropped): (τ, r) and (τ, g).
const VISCOUS_R0: f64 = 7.985734756354215;
const VISCOUS: [(f64, f64); 6] = [
    (2.777777777777778e-05, 386.9456467690825),
    (0.00015040411567170968, 82.2847282040066),
    (0.000814370328395604, 37.785662315955264),
    (0.004409447366578338, 15.699653072606182),
    (0.02387516514376188, 7.257678690133957),
    (0.12927323159868706, 0.40885318971572815),
];
const THERMAL: [(f64, f64); 6] = [
    (2.777777777777778e-05, 439.4697126827103),
    (0.00015040411567170968, 103.71150902636398),
    (0.000814370328395604, 43.48128539057197),
    (0.004409447366578338, 19.829970861452374),
    (0.02387516514376188, 7.5789905692790995),
    (0.12927323159868706, 5.193678941766592),
];

/// A first-order branch, k·sT/(1 + sT): a parallel RL (series) or series RC (shunt).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Branch {
    pub k: f64,
    pub t_s: f64,
}

/// The excess series impedance per metre of channels of radius `a` (m) and total open area
/// `area` (m²) in gas of density `rho` and viscosity `mu`: (R₀, branches), Pa·s/m³ per m.
pub fn viscous(a: f64, area: f64, rho: f64, mu: f64) -> (f64, Vec<Branch>) {
    let unit = mu / (a * a * area);
    let t_unit = a * a * rho / mu; // a²/ν
    let branches = VISCOUS
        .iter()
        .map(|&(tau, r)| Branch {
            k: r * unit,
            t_s: tau * t_unit,
        })
        .collect();
    (VISCOUS_R0 * unit, branches)
}

/// The excess shunt admittance per metre of those channels at pressure `p` (Pa): branches,
/// m³/(Pa·s) per m.
pub fn thermal(a: f64, area: f64, rho: f64, mu: f64, p: f64) -> Vec<Branch> {
    let gamma = crate::gas::GAMMA;
    let nu = mu / rho;
    let unit = area * (gamma - 1.0) * nu / (gamma * p * a * a);
    THERMAL
        .iter()
        .map(|&(tau, g)| Branch {
            k: g * unit,
            t_s: tau * a * a / nu,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// (re, im) arithmetic, enough for the exact functions.
    type C = (f64, f64);
    fn mul(a: C, b: C) -> C {
        (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
    }
    fn div(a: C, b: C) -> C {
        let d = b.0 * b.0 + b.1 * b.1;
        ((a.0 * b.0 + a.1 * b.1) / d, (a.1 * b.0 - a.0 * b.1) / d)
    }
    /// F(z) = 2J₁(z)/(zJ₀(z)), J₁/J₀ by the backward recurrence J_n/J_{n−1} = 1/(2n/z − J_{n+1}/J_n).
    fn f_zk(z: C) -> C {
        let mut r = (0.0, 0.0);
        let n_top = (z.0.hypot(z.1) as usize) + 60;
        for n in (1..=n_top).rev() {
            let two_n_over_z = div((2.0 * n as f64, 0.0), z);
            r = div((1.0, 0.0), (two_n_over_z.0 - r.0, two_n_over_z.1 - r.1));
        }
        div(mul((2.0, 0.0), r), z)
    }
    /// √(−i)·s
    fn rotated(s: f64) -> C {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        (s * h, -s * h)
    }
    fn branches_at(bs: &[Branch], w: f64) -> C {
        bs.iter().fold((0.0, 0.0), |acc, b| {
            let st = (0.0, w * b.t_s);
            let v = div(mul((b.k, 0.0), st), (1.0, w * b.t_s));
            (acc.0 + v.0, acc.1 + v.1)
        })
    }

    // 400 cpsi channels (radius 0.55 mm) in exhaust gas at 700 K: ρ = 0.504, μ = 3.3e-5
    const A: f64 = 0.55e-3;
    const AREA: f64 = 0.01;
    const RHO: f64 = 0.504;
    const MU: f64 = 3.3e-5;
    const P: f64 = 101_325.0;

    #[test]
    fn the_channels_resist_like_poiseuille_at_low_frequency() {
        // 8μ/(a²·A) = 8·3.3e-5/(3.025e-7·0.01) = 87 272.7 Pa·s/m³ per m (hand); the branches
        // carry no resistance at zero frequency
        let (r0, _) = viscous(A, AREA, RHO, MU);
        assert!((r0 / 87_272.7 - 1.0).abs() < 0.005, "{r0}");
    }

    #[test]
    fn the_networks_follow_zwikker_and_kosten() {
        let nu = MU / RHO;
        let gamma = crate::gas::GAMMA;
        let (r0, vb) = viscous(A, AREA, RHO, MU);
        let tb = thermal(A, AREA, RHO, MU, P);
        for s in [0.5, 3.0, 10.0, 40.0] {
            let w = s * s * nu / (A * A);
            // exact: Z_e = iω(ρ_eff − ρ)/A, Y_e = iω·A(γ − 1)F_t/(γp)
            let fv = f_zk(rotated(s));
            let excess = div(fv, (1.0 - fv.0, -fv.1)); // 1/(1 − F) − 1 = F/(1 − F)
            let z_exact = mul((0.0, w * RHO / AREA), excess);
            let ft = f_zk(rotated(s * PRANDTL.sqrt()));
            let y_exact = mul((0.0, w * AREA * (gamma - 1.0) / (gamma * P)), ft);
            let z = branches_at(&vb, w);
            let z = (z.0 + r0, z.1);
            let y = branches_at(&tb, w);
            let ez = (z.0 - z_exact.0).hypot(z.1 - z_exact.1) / z_exact.0.hypot(z_exact.1);
            let ey = (y.0 - y_exact.0).hypot(y.1 - y_exact.1) / y_exact.0.hypot(y_exact.1);
            assert!(
                ez < 0.015 && ey < 0.015,
                "s = {s}: series {ez:.4}, shunt {ey:.4}"
            );
        }
    }
}
