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

#[cfg(test)]
mod tests {
    use super::*;
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
