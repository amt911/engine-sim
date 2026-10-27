//! Unit conversions at the UI/config boundary. Physics uses SI only.

use std::f64::consts::PI;

/// Standard atmosphere, Pa.
pub const ATM_PA: f64 = 101_325.0;

pub fn rpm_to_rad_s(rpm: f64) -> f64 {
    rpm * 2.0 * PI / 60.0
}
pub fn rad_s_to_rpm(w: f64) -> f64 {
    w * 60.0 / (2.0 * PI)
}
pub fn deg_to_rad(d: f64) -> f64 {
    d.to_radians()
}
pub fn rad_to_deg(r: f64) -> f64 {
    r.to_degrees()
}
pub fn mm_to_m(mm: f64) -> f64 {
    mm * 1e-3
}
/// Mechanical power in W to metric... no: to imperial (mechanical) horsepower, 745.7 W.
pub fn w_to_hp(w: f64) -> f64 {
    w / 745.7
}
pub fn pa_to_bar(pa: f64) -> f64 {
    pa * 1e-5
}
pub fn m3_to_cc(m3: f64) -> f64 {
    m3 * 1e6
}

/// `x.rem_euclid(period)` (period > 0), with one add or subtract where `x` is within one period
/// of [0, period), which is how angles move between steps. The result is the same bits:
/// x − p is exact for p ≤ x ≤ 2p (Sterbenz), as `fmod` is, and x + p is the very addition
/// `rem_euclid` makes for −p < x < 0. Elsewhere it is `rem_euclid` itself, which goes through a
/// software `fmod` costing ~5 % of a step.
#[inline]
pub fn wrap(x: f64, period: f64) -> f64 {
    if (0.0..period).contains(&x) || x == 0.0 {
        x
    } else if (period..2.0 * period).contains(&x) {
        x - period
    } else if x > -period && x < 0.0 {
        // (−p itself is left to `rem_euclid`: fmod gives −0.0 there, not p − p = +0.0)
        x + period
    } else {
        x.rem_euclid(period)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_is_rem_euclid_bit_for_bit() {
        // the one-step wrap gives exactly what `rem_euclid` gives, inside its fast range
        // [−p, 2p), at its edges, and outside it (where it falls back)
        let four_pi = 4.0 * std::f64::consts::PI;
        for p in [720.0, four_pi] {
            let mut xs = vec![
                0.0,
                -0.0,
                p,
                -p,
                2.0 * p,
                -1e-300,
                -1e-12,
                p - 1e-12,
                2.0 * p - 1e-9,
                3.5 * p,
                -2.5 * p,
                f64::MIN_POSITIVE,
            ];
            let mut x = -1.3 * p;
            while x < 2.6 * p {
                xs.push(x);
                x += p / 977.0;
            }
            for x in xs {
                assert_eq!(
                    wrap(x, p).to_bits(),
                    x.rem_euclid(p).to_bits(),
                    "{x} mod {p}"
                );
            }
        }
    }

    proptest::proptest! {
        #[test]
        fn wrap_matches_rem_euclid(x in -5000.0f64..5000.0) {
            proptest::prop_assert_eq!(wrap(x, 720.0).to_bits(), x.rem_euclid(720.0).to_bits());
        }
    }
    use approx::assert_relative_eq;

    #[test]
    fn rpm_round_trip_and_known_value() {
        // 60 rpm = 1 rev/s = 2π rad/s
        assert_relative_eq!(rpm_to_rad_s(60.0), 2.0 * PI);
        assert_relative_eq!(rad_s_to_rpm(2.0 * PI), 60.0);
    }

    #[test]
    fn angles_lengths_power_pressure_volume() {
        assert_relative_eq!(deg_to_rad(180.0), PI);
        assert_relative_eq!(rad_to_deg(PI / 2.0), 90.0);
        assert_relative_eq!(mm_to_m(86.0), 0.086);
        assert_relative_eq!(w_to_hp(1491.4), 2.0);
        assert_relative_eq!(pa_to_bar(250_000.0), 2.5);
        assert_relative_eq!(m3_to_cc(0.0005), 500.0);
    }
}
