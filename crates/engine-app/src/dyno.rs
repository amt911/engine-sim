//! Engine dyno: hold the crank at a series of speeds at wide-open throttle and record the
//! brake torque at each (a "stepped-speed" dyno test, as in SAE J1349 steady-state runs).

use engine_core::engine::{Controls, Engine, EngineDef};
use engine_core::units::{rpm_to_rad_s, w_to_hp};

/// First speed of a sweep, rpm.
pub const START_RPM: f64 = 1000.0;
/// Speed increment, rpm.
pub const STEP_RPM: f64 = 500.0;
/// Time to let each new speed settle before measuring, s.
const SETTLE_S: f64 = 0.4;
/// Averaging window, s (several engine cycles even at 1,000 rpm).
const MEASURE_S: f64 = 0.4;
/// Physics step, s (same as the app at 48 kHz × 4).
const DT: f64 = 1.0 / 192_000.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynoPoint {
    pub rpm: f64,
    pub torque_nm: f64,
    pub power_w: f64,
}

impl DynoPoint {
    pub fn new(rpm: f64, torque_nm: f64) -> Self {
        Self {
            rpm,
            torque_nm,
            power_w: torque_nm * rpm_to_rad_s(rpm),
        }
    }
    pub fn power_hp(&self) -> f64 {
        w_to_hp(self.power_w)
    }
}

/// Speeds of a sweep: `START_RPM` up to (not including) the redline, `STEP_RPM` apart.
pub fn sweep_speeds(redline_rpm: f64) -> Vec<f64> {
    std::iter::successors(Some(START_RPM), |r| Some(r + STEP_RPM))
        .take_while(|r| *r < redline_rpm)
        .collect()
}

/// Run the sweep. Deterministic (fixed seed); takes a few seconds in release builds.
pub fn sweep(def: &EngineDef) -> Vec<DynoPoint> {
    let speeds = sweep_speeds(def.redline_rpm);
    let mut engine = Engine::new(def.clone(), START_RPM, 1);
    let settle = (SETTLE_S / DT) as usize;
    let measure = (MEASURE_S / DT) as usize;
    speeds
        .into_iter()
        .map(|rpm| {
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            for _ in 0..settle {
                engine.step(DT, &c);
            }
            let mut sum = 0.0;
            for _ in 0..measure {
                engine.step(DT, &c);
                sum += engine.torque_nm();
            }
            DynoPoint::new(rpm, sum / measure as f64)
        })
        .collect()
}

/// `rpm,torque_nm,power_hp` with a header line.
pub fn to_csv(points: &[DynoPoint]) -> String {
    let mut out = String::from("rpm,torque_nm,power_hp\n");
    for p in points {
        out.push_str(&format!(
            "{:.0},{:.1},{:.1}\n",
            p.rpm,
            p.torque_nm,
            p.power_hp()
        ));
    }
    out
}

/// The point with the highest torque / power.
pub fn peaks(points: &[DynoPoint]) -> Option<(DynoPoint, DynoPoint)> {
    let by = |f: fn(&DynoPoint) -> f64| points.iter().copied().max_by(|a, b| f(a).total_cmp(&f(b)));
    Some((by(|p| p.torque_nm)?, by(|p| p.power_w)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn power_is_torque_times_angular_speed() {
        // 100 N·m at 3,000 rpm: ω = 3000·2π/60 = 314.159 rad/s → 31,415.9 W = 42.13 hp.
        let p = DynoPoint::new(3000.0, 100.0);
        assert_relative_eq!(p.power_w, 31_415.9, max_relative = 1e-6);
        assert_relative_eq!(p.power_hp(), 42.130, max_relative = 1e-4);
    }

    #[test]
    fn speeds_run_from_1000_in_500_steps_below_redline() {
        assert_eq!(
            sweep_speeds(3200.0),
            vec![1000.0, 1500.0, 2000.0, 2500.0, 3000.0]
        );
        assert_eq!(sweep_speeds(3000.0).last(), Some(&2500.0));
        assert!(sweep_speeds(900.0).is_empty());
    }

    #[test]
    fn csv_has_a_header_and_one_row_per_point() {
        let csv = to_csv(&[DynoPoint::new(3000.0, 100.0), DynoPoint::new(4000.0, 120.0)]);
        // 120 N·m at 4,000 rpm: 120·418.879 = 50,265 W = 67.41 hp.
        assert_eq!(
            csv,
            "rpm,torque_nm,power_hp\n3000,100.0,42.1\n4000,120.0,67.4\n"
        );
    }

    #[test]
    fn peaks_pick_the_maximum_of_each_curve() {
        let pts = [
            DynoPoint::new(2000.0, 100.0),
            DynoPoint::new(3000.0, 90.0),
            DynoPoint::new(6000.0, 60.0),
        ];
        let (t, p) = peaks(&pts).unwrap();
        assert_eq!(t.rpm, 2000.0);
        assert_eq!(p.rpm, 6000.0); // 60·628 = 37.7 kW > 90·314 = 28.3 kW > 100·209 = 20.9 kW
        assert!(peaks(&[]).is_none());
    }

    #[test]
    fn inline4_sweep_has_a_plausible_curve() {
        let pts = sweep(&EngineDef::inline4());
        assert_eq!(pts.len(), sweep_speeds(7000.0).len());
        assert!(pts.iter().all(|p| p.torque_nm.is_finite()));
        let (t, _) = peaks(&pts).unwrap();
        assert!(
            (2000.0..=4500.0).contains(&t.rpm),
            "peak torque at {}",
            t.rpm
        );
        assert!((100.0..300.0).contains(&t.torque_nm), "{} N·m", t.torque_nm);
    }
}
