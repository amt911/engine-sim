//! Crank-slider kinematics (analytic, 1 DOF) and cylinder volume.
//!
//! θ is crank angle from top dead centre. Piston displacement from TDC:
//! s(θ) = r + l − (r·cosθ + √(l² − r²·sin²θ)), r = stroke/2, l = rod length
//! (Heywood, *ICE Fundamentals*, eq. 2.4).
//!
//! A Wankel chamber ([`Rotor`]) is the same with an infinitely long rod: its volume is a pure
//! sinusoid of the chamber angle ψ, two thirds of the eccentric shaft's (spec
//! `docs/superpowers/specs/2026-10-31-rotary-design.md`).

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CylinderGeometry {
    pub bore_m: f64,
    pub stroke_m: f64,
    pub rod_m: f64,
    pub compression_ratio: f64,
    /// A Wankel rotor's chamber instead of a piston: `bore_m`/`stroke_m` are then its
    /// equivalent piston ([`CylinderGeometry::rotary`]) and `rod_m` is unused.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rotor: Option<Rotor>,
}

/// A Wankel rotor and its housing: eccentricity, generating radius (rotor centre to apex) and
/// width, m (Heywood, *ICE Fundamentals* 2nd ed., §1.8, Fig. 1.23).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rotor {
    pub eccentricity_m: f64,
    pub generating_radius_m: f64,
    pub width_m: f64,
}

impl Rotor {
    /// How far the rotor's flank moves past the housing per degree of chamber angle, m: the
    /// rotor turns ψ/2 (a third of the shaft, ψ = ⅔ of it), its flank at about the generating
    /// radius (Heywood 2nd ed., §1.8) — the speed its edge sweeps across the side ports.
    pub fn port_sweep_m_per_deg(&self) -> f64 {
        0.5 * self.generating_radius_m * std::f64::consts::PI / 180.0
    }
}

impl CylinderGeometry {
    /// A Wankel chamber of a rotor with eccentricity `e`, generating radius `r` and width `b`,
    /// m: displacement 3√3·e·R·B (Heywood 2nd ed., Fig. 1.23), as an equivalent piston of the
    /// rotor flank's area √3·R·B and stroke 3e on an infinitely long rod.
    pub fn rotary(e: f64, r: f64, b: f64, compression_ratio: f64) -> Self {
        let area = 3f64.sqrt() * r * b;
        Self {
            bore_m: (4.0 * area / PI).sqrt(),
            stroke_m: 3.0 * e,
            rod_m: 0.0,
            compression_ratio,
            rotor: Some(Rotor {
                eccentricity_m: e,
                generating_radius_m: r,
                width_m: b,
            }),
        }
    }
    pub fn crank_radius(&self) -> f64 {
        self.stroke_m * 0.5
    }
    pub fn piston_area(&self) -> f64 {
        PI * 0.25 * self.bore_m * self.bore_m
    }
    /// Swept (displaced) volume of one cylinder.
    pub fn displacement(&self) -> f64 {
        self.piston_area() * self.stroke_m
    }
    /// Clearance volume Vc = Vd / (CR − 1).
    pub fn clearance_volume(&self) -> f64 {
        self.displacement() / (self.compression_ratio - 1.0)
    }
    /// Piston distance below TDC.
    pub fn piston_position(&self, theta: f64) -> f64 {
        let r = self.crank_radius();
        if self.rotor.is_some() {
            return r * (1.0 - theta.cos());
        }
        let l = self.rod_m;
        let sin = theta.sin();
        r + l - (r * theta.cos() + (l * l - r * r * sin * sin).sqrt())
    }
    /// ds/dθ, the lever arm that turns gas force into crank torque.
    pub fn dpos_dtheta(&self, theta: f64) -> f64 {
        let r = self.crank_radius();
        if self.rotor.is_some() {
            return r * theta.sin();
        }
        let l = self.rod_m;
        let (sin, cos) = theta.sin_cos();
        r * sin + r * r * sin * cos / (l * l - r * r * sin * sin).sqrt()
    }
    pub fn volume(&self, theta: f64) -> f64 {
        self.clearance_volume() + self.piston_area() * self.piston_position(theta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    /// 86 × 86 mm, 143 mm rod, CR 10 — a typical 500 cc single.
    fn single() -> CylinderGeometry {
        CylinderGeometry {
            bore_m: 0.086,
            stroke_m: 0.086,
            rod_m: 0.143,
            compression_ratio: 10.0,
            rotor: None,
        }
    }

    #[test]
    fn a_rotor_flank_sweeps_its_ports_at_half_the_generating_radius_per_chamber_radian() {
        // the rotor turns ψ/2 (a third of the shaft, ψ = ⅔ shaft): R/2 per radian of ψ;
        // RENESIS R = 105 mm: 0.0525 · π/180 = 9.1630e-4 m per degree
        let r = CylinderGeometry::rotary(0.015, 0.105, 0.080, 10.0)
            .rotor
            .unwrap();
        assert!((r.port_sweep_m_per_deg() - 9.1630e-4).abs() < 1e-8);
    }

    #[test]
    fn displacement_is_bore_area_times_stroke() {
        // π/4 · 0.086² · 0.086 = 4.99556e-4 m³ (hand calculation)
        assert_relative_eq!(single().displacement(), 4.99556e-4, max_relative = 1e-5);
    }

    #[test]
    fn clearance_matches_compression_ratio() {
        let g = single();
        let vmax = g.volume(PI);
        let vmin = g.volume(0.0);
        assert_relative_eq!(vmax / vmin, 10.0, max_relative = 1e-9);
    }

    #[test]
    fn piston_at_tdc_and_bdc() {
        let g = single();
        assert_relative_eq!(g.piston_position(0.0), 0.0, epsilon = 1e-12);
        assert_relative_eq!(g.piston_position(PI), 0.086, max_relative = 1e-12);
    }

    #[test]
    fn piston_at_90_degrees_is_past_half_stroke() {
        // r + l − √(l² − r²) with r = 0.043, l = 0.143: 0.043 + 0.143 − 0.136377 = 0.049623
        assert_relative_eq!(
            single().piston_position(PI / 2.0),
            0.049623,
            max_relative = 1e-4
        );
    }

    #[test]
    fn lever_arm_zero_at_dead_centres_and_r_at_90() {
        let g = single();
        assert_relative_eq!(g.dpos_dtheta(0.0), 0.0, epsilon = 1e-12);
        assert_relative_eq!(g.dpos_dtheta(PI), 0.0, epsilon = 1e-12);
        // At 90° the second term vanishes: ds/dθ = r
        assert_relative_eq!(g.dpos_dtheta(PI / 2.0), 0.043, max_relative = 1e-12);
    }

    proptest! {
        #[test]
        fn position_stays_within_stroke(theta in -20.0f64..20.0) {
            let s = single().piston_position(theta);
            prop_assert!((-1e-12..=0.086 + 1e-12).contains(&s));
        }

        #[test]
        fn lever_arm_is_numerical_derivative(theta in -6.0f64..6.0) {
            let g = single();
            let h = 1e-6;
            let num = (g.piston_position(theta + h) - g.piston_position(theta - h)) / (2.0 * h);
            prop_assert!((num - g.dpos_dtheta(theta)).abs() < 1e-7);
        }
    }

    #[test]
    fn a_renesis_chamber_displaces_what_heywood_gives() {
        // Heywood 2nd ed. Fig. 1.23: e 15 mm, R 105 mm, B 80 mm → 3√3·e·R·B = 5.19615·0.015·
        // 0.105·0.080 = 654.7 cm³ per chamber; CR 10 → V_min = V_d/9 = 72.7 cm³
        let g = CylinderGeometry::rotary(0.015, 0.105, 0.080, 10.0);
        assert!(
            (g.displacement() * 1e6 - 654.7).abs() < 0.1,
            "{}",
            g.displacement()
        );
        assert!((g.clearance_volume() * 1e6 - 72.75).abs() < 0.05);
        assert!(g.rotor.is_some());
    }

    #[test]
    fn a_rotary_chamber_breathes_as_a_pure_sinusoid() {
        // V(ψ) = V_min + V_d/2·(1 − cos ψ), ψ the chamber angle (⅔ of the shaft's)
        let g = CylinderGeometry::rotary(0.015, 0.105, 0.080, 10.0);
        let (vmin, vd) = (g.clearance_volume(), g.displacement());
        for deg in [0.0f64, 37.0, 90.0, 180.0, 251.0, 360.0, 540.0] {
            let th = deg.to_radians();
            let want = vmin + 0.5 * vd * (1.0 - th.cos());
            assert!((g.volume(th) - want).abs() < 1e-12, "{deg}°");
            // the lever arm is dV/dψ over the equivalent piston's area
            let h = 1e-6;
            let dv = (g.volume(th + h) - g.volume(th - h)) / (2.0 * h);
            assert!((dv - g.piston_area() * g.dpos_dtheta(th)).abs() < 1e-9 * vd.max(1.0));
        }
    }

    #[test]
    fn a_rotor_round_trips_and_old_files_have_none() {
        let g = CylinderGeometry::rotary(0.015, 0.105, 0.080, 10.0);
        let j: CylinderGeometry =
            serde_json::from_str(&serde_json::to_string(&g).unwrap()).unwrap();
        assert_eq!(j, g);
        let y: CylinderGeometry =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&g).unwrap()).unwrap();
        assert_eq!(y, g);
        let old: CylinderGeometry = serde_yaml_ng::from_str(
            "bore_m: 0.086\nstroke_m: 0.086\nrod_m: 0.143\ncompression_ratio: 10.0\n",
        )
        .unwrap();
        assert_eq!(old.rotor, None);
    }
}
