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
    /// A second lobe the valve rides while VTEC is engaged ([`Vtec`]); none: one lobe.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub high: Option<CamLobe>,
}

/// A cam lobe: where it starts opening its valve, for how long and how far.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CamLobe {
    pub open_deg: f64,
    pub duration_deg: f64,
    pub max_lift_m: f64,
}

/// Honda's VTEC: a pin locks the valve's rocker onto its high lobe above a speed, with the pedal
/// down (the real solenoid also waits for oil pressure and coolant temperature, not modelled),
/// and lets it go below the speed less a hysteresis (Honda, Integra Type R DC5: 6000 rpm).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Vtec {
    pub engage_rpm: f64,
    pub hysteresis_rpm: f64,
    /// Pedal it needs, 0–1.
    pub min_throttle: f64,
}

impl Vtec {
    /// Whether the high lobes are engaged after this step, from whether they were.
    pub fn next(&self, engaged: bool, rpm: f64, throttle: f64) -> bool {
        if throttle < self.min_throttle {
            return false;
        }
        if engaged {
            rpm >= self.engage_rpm - self.hysteresis_rpm
        } else {
            rpm >= self.engage_rpm
        }
    }
}

/// One point of a cam phaser's map: the advance the ECU asks for at `rpm` with the pedal down
/// and with it released, crank degrees.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhaserPoint {
    pub rpm: f64,
    pub full_load_deg: f64,
    pub no_load_deg: f64,
}

/// A hydraulic cam phaser (Toyota VVT-i, Honda VTC): it turns its camshaft up to `range_deg`
/// crank degrees earlier, at most `rate_deg_s` (VVT-i: up to 60° CA, Toyota SAE 960579; control
/// rate limits of 25–40 °/s, US 7,467,611), towards its ECU map's advance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Phaser {
    pub range_deg: f64,
    pub rate_deg_s: f64,
    pub map: Vec<PhaserPoint>,
}

impl Phaser {
    /// The advance the map asks for: interpolated in speed between its points (held beyond them)
    /// and linearly in pedal between no load and full load, clamped to the range.
    pub fn target(&self, rpm: f64, throttle: f64) -> f64 {
        let Some(first) = self.map.first() else {
            return 0.0;
        };
        let t = throttle.clamp(0.0, 1.0);
        let at = |p: &PhaserPoint| p.no_load_deg + (p.full_load_deg - p.no_load_deg) * t;
        let mut deg = at(first);
        for w in self.map.windows(2) {
            let (a, b) = (&w[0], &w[1]);
            if rpm >= b.rpm {
                deg = at(b);
            } else {
                if rpm > a.rpm {
                    deg = at(a) + (at(b) - at(a)) * (rpm - a.rpm) / (b.rpm - a.rpm);
                }
                break;
            }
        }
        deg.clamp(0.0, self.range_deg)
    }
    /// The cam's angle after `dt` moving from `current` towards `target` at its rate.
    pub fn step(&self, current: f64, target: f64, dt: f64) -> f64 {
        let d = self.rate_deg_s * dt;
        current + (target - current).clamp(-d, d)
    }
}

impl ValveTiming {
    /// The timing the valve has now: its high lobe if `high` (and it has one), its whole lift
    /// curve moved `advance_deg` crank degrees earlier.
    pub fn effective(&self, high: bool, advance_deg: f64) -> ValveTiming {
        let lobe = match (high, self.high) {
            (true, Some(l)) => l,
            _ => CamLobe {
                open_deg: self.open_deg,
                duration_deg: self.duration_deg,
                max_lift_m: self.max_lift_m,
            },
        };
        ValveTiming {
            open_deg: crate::units::wrap(lobe.open_deg - advance_deg, 720.0),
            duration_deg: lobe.duration_deg,
            max_lift_m: lobe.max_lift_m,
            diameter_m: self.diameter_m,
            high: self.high,
        }
    }
}

impl ValveTiming {
    /// Lift at a cycle angle in degrees (any value; wrapped to 720).
    pub fn lift(&self, cycle_deg: f64) -> f64 {
        let x = crate::units::wrap(cycle_deg - self.open_deg, 720.0) / self.duration_deg;
        if x >= 1.0 {
            return 0.0;
        }
        let s = (std::f64::consts::PI * x).sin();
        self.max_lift_m * s * s
    }
    /// The most this valve passes: its effective area at its peak lift, on its high lobe if it
    /// has one and that lifts it further, m².
    pub fn max_flow_area(&self) -> f64 {
        let lift = self
            .high
            .map_or(self.max_lift_m, |h| h.max_lift_m.max(self.max_lift_m));
        let peak = ValveTiming {
            max_lift_m: lift,
            ..*self
        };
        peak.flow_area(peak.open_deg + 0.5 * peak.duration_deg)
    }
    /// Effective area of a rotary's port at a cycle angle: the rotor's edge sweeps across the
    /// round port (diameter `diameter_m`) at `travel_m_per_deg`, so the area uncovered is the
    /// circular segment of that depth, r²·acos((r − s)/r) − (r − s)·√(2rs − s²), full once
    /// the edge has crossed it, and mirrored as the next edge covers it over the window's
    /// end (Yamamoto, *Rotary Engine*, 1981, ch. 3: side ports opened by the rotor's flank). A
    /// cam's sin² lift opened a port over its whole window; an edge opens it in tens of
    /// degrees, and the blowdown is that much sharper.
    pub fn swept_port_area(&self, cycle_deg: f64, travel_m_per_deg: f64) -> f64 {
        let x = crate::units::wrap(cycle_deg - self.open_deg, 720.0);
        if x >= self.duration_deg {
            return 0.0;
        }
        let r = 0.5 * self.diameter_m;
        let s = (x.min(self.duration_deg - x) * travel_m_per_deg).min(2.0 * r);
        let h = r - s;
        VALVE_CD
            * (r * r * (h / r).clamp(-1.0, 1.0).acos() - h * (2.0 * r * s - s * s).max(0.0).sqrt())
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

    fn vtec_intake() -> ValveTiming {
        ValveTiming {
            open_deg: 350.0,
            duration_deg: 244.0,
            max_lift_m: 0.0088,
            diameter_m: 0.035,
            high: Some(CamLobe {
                open_deg: 340.0,
                duration_deg: 280.0,
                max_lift_m: 0.01265,
            }),
        }
    }

    #[test]
    fn a_valve_passes_most_at_its_peak_lift_on_either_lobe() {
        // 35 mm at 10 mm: curtain π·0.035·0.01 = 1.0996e-3, port π/4·0.035² = 9.6211e-4, the
        // blend (n = 6.5788) 9.1262e-4, ×0.7 = 6.3884e-4 m²; a 12 mm high lobe: curtain
        // 1.3195e-3, blend 9.4502e-4, ×0.7 = 6.6151e-4 m²
        let mut v = ValveTiming {
            open_deg: 350.0,
            duration_deg: 240.0,
            max_lift_m: 0.010,
            diameter_m: 0.035,
            high: None,
        };
        assert_relative_eq!(v.max_flow_area(), 6.3884e-4, max_relative = 1e-4);
        v.high = Some(CamLobe {
            open_deg: 340.0,
            duration_deg: 260.0,
            max_lift_m: 0.012,
        });
        assert_relative_eq!(v.max_flow_area(), 6.6151e-4, max_relative = 1e-4);
    }

    #[test]
    fn a_rotor_edge_uncovers_a_round_port_as_a_circular_segment() {
        // d = 40 mm port, the edge 1 mm per degree, open at 100° for 200°. Hand values: half
        // the depth (20 mm, 120°) half the circle, Cd·π·0.02²/2 = 4.3982e-4 m²; a quarter
        // (10 mm, 110°) the segment r²·acos(½) − ½r·√(2r·½r − ¼r²)·… = 4e-4·1.0472 −
        // 0.01·√3e-4 = 2.4567e-4, ×0.7 = 1.7197e-4 m²; full from 140° to 260°; the closing
        // edge mirrors it.
        let v = ValveTiming {
            open_deg: 100.0,
            duration_deg: 200.0,
            max_lift_m: 0.01,
            diameter_m: 0.04,
            high: None,
        };
        let full = VALVE_CD * PI * 0.02 * 0.02;
        assert_eq!(v.swept_port_area(99.0, 1e-3), 0.0);
        assert_relative_eq!(
            v.swept_port_area(110.0, 1e-3),
            1.7197e-4,
            max_relative = 1e-4
        );
        assert_relative_eq!(
            v.swept_port_area(120.0, 1e-3),
            0.5 * full,
            max_relative = 1e-12
        );
        assert_relative_eq!(v.swept_port_area(140.0, 1e-3), full, max_relative = 1e-12);
        assert_relative_eq!(v.swept_port_area(200.0, 1e-3), full, max_relative = 1e-12);
        assert_relative_eq!(
            v.swept_port_area(280.0, 1e-3),
            0.5 * full,
            max_relative = 1e-12
        );
        assert_eq!(v.swept_port_area(300.0, 1e-3), 0.0);
    }

    #[test]
    fn vtec_opens_the_valve_on_its_high_lobe() {
        // mid-opening of each lobe (sin² peak): low 350 + 122 = 472°, 8.8 mm; high 340 + 140 =
        // 480°, 12.65 mm
        let v = vtec_intake();
        assert_relative_eq!(
            v.effective(false, 0.0).lift(472.0),
            0.0088,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            v.effective(true, 0.0).lift(480.0),
            0.012_65,
            max_relative = 1e-12
        );
        // a valve with one lobe keeps it either way
        let one = ValveTiming { high: None, ..v };
        assert_eq!(one.effective(true, 0.0), one.effective(false, 0.0));
    }

    #[test]
    fn a_phaser_moves_the_whole_lift_curve_earlier() {
        // advanced by 30°: what the cam lifted at θ + 30 it now lifts at θ
        let v = vtec_intake();
        let a = v.effective(false, 30.0);
        for theta in [300.0, 350.0, 400.0, 470.0, 560.0] {
            assert_relative_eq!(
                a.lift(theta),
                v.effective(false, 0.0).lift(theta + 30.0),
                epsilon = 1e-15
            );
        }
    }

    #[test]
    fn vtec_engages_at_its_speed_and_holds_through_its_hysteresis() {
        let s = Vtec {
            engage_rpm: 6000.0,
            hysteresis_rpm: 300.0,
            min_throttle: 0.5,
        };
        assert!(!s.next(false, 5999.0, 1.0));
        assert!(s.next(false, 6000.0, 1.0));
        assert!(s.next(true, 5701.0, 1.0), "held above 5700");
        assert!(!s.next(true, 5699.0, 1.0), "released below 5700");
        assert!(s.next(false, 7000.0, 0.5), "at its pedal it engages");
        assert!(!s.next(false, 7000.0, 0.4), "not without the pedal");
        assert!(
            !s.next(true, 7000.0, 0.4),
            "and dropped when the pedal lifts"
        );
    }

    #[test]
    fn a_phaser_follows_its_map_at_its_rate() {
        // map: 1000 rpm 0°/0°, 5000 rpm 40° at full load and 10° at none (hand: at 3000 rpm and
        // half pedal, 20° full and 5° none → 12.5°)
        let p = Phaser {
            range_deg: 50.0,
            rate_deg_s: 30.0,
            map: vec![
                PhaserPoint {
                    rpm: 1000.0,
                    full_load_deg: 0.0,
                    no_load_deg: 0.0,
                },
                PhaserPoint {
                    rpm: 5000.0,
                    full_load_deg: 40.0,
                    no_load_deg: 10.0,
                },
            ],
        };
        assert_relative_eq!(p.target(3000.0, 0.5), 12.5, max_relative = 1e-12);
        assert_relative_eq!(p.target(500.0, 1.0), 0.0);
        assert_relative_eq!(p.target(9000.0, 1.0), 40.0);
        // clamped to its range
        let wide = Phaser {
            range_deg: 30.0,
            ..p.clone()
        };
        assert_relative_eq!(wide.target(9000.0, 1.0), 30.0);
        // between two advanced points: 2000 rpm 10°, 4000 rpm 30° (full load) → 20° at 3000
        let raised = Phaser {
            map: vec![
                PhaserPoint {
                    rpm: 2000.0,
                    full_load_deg: 10.0,
                    no_load_deg: 10.0,
                },
                PhaserPoint {
                    rpm: 4000.0,
                    full_load_deg: 30.0,
                    no_load_deg: 10.0,
                },
            ],
            ..p.clone()
        };
        assert_relative_eq!(raised.target(3000.0, 1.0), 20.0, max_relative = 1e-12);
        // 30 °/s: half a second covers 15°, a second the whole 30°
        let mut a = 0.0;
        for _ in 0..500 {
            a = p.step(a, 30.0, 0.001);
        }
        assert_relative_eq!(a, 15.0, max_relative = 1e-9);
        for _ in 0..1000 {
            a = p.step(a, 30.0, 0.001);
        }
        assert_relative_eq!(a, 30.0, max_relative = 1e-12);
    }

    #[test]
    fn variable_timing_round_trips_and_old_valves_load_without_it() {
        let v = vtec_intake();
        let json = serde_json::to_string(&v).unwrap();
        assert_eq!(serde_json::from_str::<ValveTiming>(&json).unwrap(), v);
        let yaml = serde_yaml_ng::to_string(&v).unwrap();
        assert_eq!(serde_yaml_ng::from_str::<ValveTiming>(&yaml).unwrap(), v);
        let old: ValveTiming = serde_yaml_ng::from_str(
            "open_deg: 340.0\nduration_deg: 265.0\nmax_lift_m: 0.0124\ndiameter_m: 0.0495\n",
        )
        .unwrap();
        assert_eq!(old.high, None);
        let p = Phaser {
            range_deg: 50.0,
            rate_deg_s: 30.0,
            map: vec![PhaserPoint {
                rpm: 1000.0,
                full_load_deg: 5.0,
                no_load_deg: 0.0,
            }],
        };
        let y = serde_yaml_ng::to_string(&p).unwrap();
        assert_eq!(serde_yaml_ng::from_str::<Phaser>(&y).unwrap(), p);
        let s = Vtec {
            engage_rpm: 6000.0,
            hysteresis_rpm: 300.0,
            min_throttle: 0.5,
        };
        assert_eq!(
            serde_json::from_str::<Vtec>(&serde_json::to_string(&s).unwrap()).unwrap(),
            s
        );
    }

    fn exhaust() -> ValveTiming {
        // opens 40° before BDC of power stroke, closes 15° after exhaust TDC
        ValveTiming {
            open_deg: 140.0,
            duration_deg: 235.0,
            max_lift_m: 0.009,
            diameter_m: 0.030,
            high: None,
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
            high: None,
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
