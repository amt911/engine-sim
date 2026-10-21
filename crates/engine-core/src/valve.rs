//! Poppet valve lift and effective flow area.
//!
//! Angles are in the 720° four-stroke cycle, 0° = TDC at the start of the power stroke.
//! Lift profile: sin² bump over the opening window (smooth, zero velocity at the ends).

use serde::{Deserialize, Serialize};

/// Discharge coefficient of a poppet valve at moderate L/D (Soriano et al., ENCIT 2012).
pub const VALVE_CD: f64 = 0.7;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
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
    /// Effective area Cd·min(curtain, port) at a cycle angle.
    pub fn flow_area(&self, cycle_deg: f64) -> f64 {
        let pi = std::f64::consts::PI;
        let curtain = pi * self.diameter_m * self.lift(cycle_deg);
        let port = 0.25 * pi * self.diameter_m * self.diameter_m;
        VALVE_CD * curtain.min(port)
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
        // full lift: curtain π·0.03·0.009 = 8.48e-4 > port π/4·0.03² = 7.07e-4 → port limited
        assert_relative_eq!(
            v.flow_area(257.5),
            VALVE_CD * 7.0686e-4,
            max_relative = 1e-4
        );
        assert_eq!(v.flow_area(100.0), 0.0);
    }
}
