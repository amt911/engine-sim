//! Poppet valve lift and effective flow area.
//!
//! Angles are in the 720° four-stroke cycle, 0° = TDC at the start of the power stroke.
//! Lift profile: sin² bump over the opening window (smooth, zero velocity at the ends).

use serde::{Deserialize, Serialize};

/// Discharge coefficient of a poppet valve at moderate L/D (Soriano et al., ENCIT 2012).
pub const VALVE_CD: f64 = 0.7;
/// Exponent of the smooth turn from curtain to port area (see [`ValveTiming::flow_area`]):
/// ln 2 / ln(1/0.9).
const AREA_BLEND: f64 = 6.578_813_478_960_585;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ValveTiming {
    /// Crank angle where the valve starts to open, degrees in [0, 720).
    pub open_deg: f64,
    /// Crank degrees the valve stays open.
    pub duration_deg: f64,
    pub max_lift_m: f64,
    pub diameter_m: f64,
}

impl ValveTiming {
    /// Lift at a cycle angle in degrees (any value; wrapped to 720).
    pub fn lift(&self, cycle_deg: f64) -> f64 {
        let x = (cycle_deg - self.open_deg).rem_euclid(720.0) / self.duration_deg;
        if x >= 1.0 {
            return 0.0;
        }
        let s = (std::f64::consts::PI * x).sin();
        self.max_lift_m * s * s
    }
    /// Effective area at a cycle angle: Cd times the smaller of the curtain (πDL) and the port
    /// (πD²/4), turned smoothly from one to the other, (A_c⁻ⁿ + A_p⁻ⁿ)^(−1/n). Measured
    /// exhaust valves pass ~95 % of the geometric minimum below L/D = 0.2 and ~90 % at 0.25,
    /// where the two are equal (Heywood, *ICE Fundamentals*, p. 229, Fig. 6-18): n =
    /// ln 2 / ln(1/0.9) ≈ 6.6 gives both (0.969 at L/D = 0.2). A sharp min() turned the slope
    /// abruptly there: a kink in every valve's flow, heard as a click per valve event.
    pub fn flow_area(&self, cycle_deg: f64) -> f64 {
        let pi = std::f64::consts::PI;
        let curtain = pi * self.diameter_m * self.lift(cycle_deg);
        if curtain <= 0.0 {
            return 0.0;
        }
        let port = 0.25 * pi * self.diameter_m * self.diameter_m;
        let n = AREA_BLEND;
        VALVE_CD * (curtain.powf(-n) + port.powf(-n)).powf(-1.0 / n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    fn exhaust() -> ValveTiming {
        // opens 40° before BDC of power stroke, closes 15° after exhaust TDC
        ValveTiming {
            open_deg: 140.0,
            duration_deg: 235.0,
            max_lift_m: 0.009,
            diameter_m: 0.030,
        }
    }

    #[test]
    fn the_flow_area_turns_from_curtain_to_port_smoothly() {
        // Heywood, ICE Fundamentals, p. 229 / Fig. 6-18: an exhaust valve's effective area is
        // ~95 % of the geometric minimum below L/D = 0.2 and ~90 % at L/D = 0.25, where curtain
        // (πDL) and port (πD²/4) are equal. A sharp min() made the area's slope jump there:
        // a kink in every valve's flow, heard as a click per event
        let v = exhaust();
        let d = v.diameter_m;
        let port = VALVE_CD * 0.25 * PI * d * d;
        let at_lift = |l: f64| {
            let t = ValveTiming { max_lift_m: l, ..v };
            // full lift is reached mid-window
            t.flow_area(t.open_deg + 0.5 * t.duration_deg)
        };
        assert_relative_eq!(at_lift(0.25 * d) / port, 0.9, max_relative = 0.01);
        let curtain_02 = VALVE_CD * PI * d * 0.2 * d;
        assert!((at_lift(0.2 * d) / curtain_02 - 0.97).abs() < 0.02);
        // no corner: the slope just below and just above the crossing agree
        let h = 1e-6 * d;
        let slope = |l: f64| (at_lift(l + h) - at_lift(l - h)) / (2.0 * h);
        let (below, above) = (slope(0.25 * d - 20.0 * h), slope(0.25 * d + 20.0 * h));
        assert!(
            (below - above).abs() < 0.01 * below.abs(),
            "{below} vs {above}"
        );
    }

    #[test]
    fn closed_outside_window_open_inside() {
        let v = exhaust();
        assert_eq!(v.lift(100.0), 0.0);
        assert_eq!(v.lift(400.0), 0.0);
        assert!(v.lift(200.0) > 0.0);
    }

    #[test]
    fn peak_lift_at_middle_of_window() {
        let v = exhaust();
        assert_relative_eq!(v.lift(140.0 + 117.5), 0.009, max_relative = 1e-12);
    }

    #[test]
    fn window_wraps_around_720() {
        let v = ValveTiming {
            open_deg: 700.0,
            duration_deg: 40.0,
            max_lift_m: 0.01,
            diameter_m: 0.03,
        };
        assert_relative_eq!(v.lift(0.0), 0.01, max_relative = 1e-12); // 20° into the window = middle
        assert_relative_eq!(v.lift(720.0), v.lift(0.0));
        assert_relative_eq!(v.lift(-720.0), v.lift(0.0));
        assert_eq!(v.lift(30.0), 0.0);
    }

    #[test]
    fn flow_area_is_curtain_then_port_limited() {
        let v = exhaust();
        // tiny lift → curtain area Cd·π·d·L
        let small = 140.0 + 5.0;
        let l = v.lift(small);
        assert_relative_eq!(
            v.flow_area(small),
            VALVE_CD * PI * 0.030 * l,
            max_relative = 1e-12
        );
        // full lift: curtain π·0.03·0.009 = 8.48e-4 = 1.2 × port π/4·0.03² = 7.07e-4 → port
        // limited, 96 % of it on the smooth turn: (1 + 1.2^−6.5788)^(−1/6.5788) = 0.960752
        assert_relative_eq!(
            v.flow_area(257.5),
            VALVE_CD * 7.0686e-4 * 0.960_752,
            max_relative = 1e-4
        );
        assert_eq!(v.flow_area(100.0), 0.0);
    }
}
