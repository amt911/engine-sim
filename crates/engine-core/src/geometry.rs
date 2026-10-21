//! Crank-slider kinematics (analytic, 1 DOF) and cylinder volume.
//!
//! θ is crank angle from top dead centre. Piston displacement from TDC:
//! s(θ) = r + l − (r·cosθ + √(l² − r²·sin²θ)), r = stroke/2, l = rod length
//! (Heywood, *ICE Fundamentals*, eq. 2.4).

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CylinderGeometry {
    pub bore_m: f64,
    pub stroke_m: f64,
    pub rod_m: f64,
    pub compression_ratio: f64,
}

impl CylinderGeometry {
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
        let l = self.rod_m;
        let sin = theta.sin();
        r + l - (r * theta.cos() + (l * l - r * r * sin * sin).sqrt())
    }
    /// ds/dθ, the lever arm that turns gas force into crank torque.
    pub fn dpos_dtheta(&self, theta: f64) -> f64 {
        let r = self.crank_radius();
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
        }
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
}
