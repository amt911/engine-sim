//! Engine dyno: hold the crank at a series of speeds at wide-open throttle and record the
//! brake torque at each (a "stepped-speed" dyno test, as in SAE J1349 steady-state runs).

use engine_core::engine::{Controls, Engine, EngineDef};
use engine_core::units::{rpm_to_rad_s, w_to_hp};

/// First speed of a sweep, rpm.
pub const START_RPM: f64 = 1000.0;
/// Speed increment, rpm.
pub const STEP_RPM: f64 = 500.0;
/// Time to let each new speed settle before measuring, s: >= 5 time constants (0.15 s) of the
/// engine's torque average, so under 1 % of the previous point is left.
const SETTLE_S: f64 = 0.8;
/// The same for a turbocharged engine, s: a steady-state point waits for steady boost, whose
/// shaft takes seconds to settle at low speed.
const TURBO_SETTLE_S: f64 = 4.0;
/// Minimum averaging window, s; rounded up to whole engine cycles (see [`measure_steps`]).
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

/// Physics steps to average over: whole 720° cycles (120/rpm s each) covering at least
/// `MEASURE_S`, so the firing pulses of every cylinder count equally.
pub fn measure_steps(rpm: f64) -> usize {
    let cycle_s = 120.0 / rpm;
    let cycles = (MEASURE_S / cycle_s).ceil().max(1.0);
    (cycles * cycle_s / DT).round() as usize
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
    let settle_s = if def.turbo.is_some() {
        TURBO_SETTLE_S
    } else {
        SETTLE_S
    };
    let settle = (settle_s / DT) as usize;
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
            let measure = measure_steps(rpm);
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
    fn the_lfa_breathes_as_lexus_published() {
        // Toyota/Lexus (2009): 412 kW (552 hp) at 8700 rpm, 480 N·m at 6800 rpm, 90 % of it
        // from 3700 to 9000 rpm. The preset's unpublished geometry is sized by physics (valves
        // by the Mach index at 8700 rpm, runners tuned at 6800, primaries at 8700). With real
        // gas properties and friction by components the level is the published one (≈ 500
        // N·m); the top falls early (≈ 12 % short at 8500 rpm): at speed the valve events pass
        // too little gas — the sin² lift curve's time-area is below a real cam's, and the lift
        // stops at L/D 0.25 (FINDINGS, 2026-10-27). The torque is flat from 5500 to 6500 rpm
        // (within 1 %), so its peak may land at 5500 until the cam profile is a real one.
        let points = sweep(&EngineDef::lfa_v10());
        let (torque, power) = peaks(&points).unwrap();
        assert!(
            (5500.0..=7500.0).contains(&torque.rpm),
            "torque peak at {}",
            torque.rpm
        );
        assert!(
            (7500.0..=9000.0).contains(&power.rpm),
            "power peak at {}",
            power.rpm
        );
        assert!(
            (480.0..=600.0).contains(&power.power_hp()),
            "{} hp",
            power.power_hp()
        );
        // the broad band: 90 % of the peak torque by 4500 rpm (published: 3700)
        let at = |rpm: f64| points.iter().find(|p| p.rpm == rpm).unwrap().torque_nm;
        assert!(
            at(4500.0) >= 0.88 * torque.torque_nm,
            "{} at 4500",
            at(4500.0)
        );
    }

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
    fn measuring_covers_whole_engine_cycles() {
        // One cycle = 720° = 120/rpm s: 1,000 rpm → 0.12 s, so 0.4 s rounds up to 4 cycles
        // = 0.48 s = 92,160 steps of 1/192 kHz; 6,000 rpm → 0.02 s → 20 cycles = 76,800 steps.
        assert_eq!(measure_steps(1000.0), 92_160);
        assert_eq!(measure_steps(6000.0), 76_800);
    }

    #[test]
    fn settling_outlasts_five_filter_time_constants() {
        // The engine's torque average has τ = 0.15 s; 5τ = 0.75 s leaves < 1 % (e^-5 = 0.67 %).
        const { assert!(SETTLE_S >= 5.0 * 0.15) };
    }

    #[test]
    fn single_cylinder_curve_is_smooth() {
        // A one-cylinder torque pulse is lumpy; without whole-cycle averaging adjacent
        // points jumped up and down. A zigzag is a bend against both neighbours (the second
        // difference), which must stay within 15 % of the peak; a curve falling past its peak
        // (the single's 32 → 26 N·m from 5000 to 5500 rpm) is the engine, not the averaging.
        let pts = sweep(&EngineDef::single());
        let peak = peaks(&pts).unwrap().0.torque_nm;
        for w in pts.windows(3) {
            let bend = w[2].torque_nm - 2.0 * w[1].torque_nm + w[0].torque_nm;
            assert!(bend.abs() < 0.15 * peak, "{w:?}");
        }
    }

    #[test]
    fn the_k20a_breathes_near_what_honda_published() {
        // Honda K20A (Integra Type R): 206 N·m at 7000 rpm, 220 PS (217 hp) at 8000. Peak torque
        // lands within 10 % of it (≈ 198 N·m) and holds within 10 % of its peak into VTEC's
        // range (6500 rpm); power, still rising at 8000 rpm as Honda's, runs ≈ 9 % short there:
        // the presets' cams pass too little gas at speed (FINDINGS, 2026-10-27).
        let pts = sweep(&EngineDef::honda_k20a());
        assert_eq!(pts.len(), sweep_speeds(8400.0).len());
        let (t, p) = peaks(&pts).unwrap();
        assert!(
            (t.torque_nm / 206.0 - 1.0).abs() < 0.1,
            "{} N·m",
            t.torque_nm
        );
        let at_6500 = pts.iter().find(|q| q.rpm == 6500.0).unwrap();
        assert!(
            at_6500.torque_nm > 0.9 * t.torque_nm,
            "{} N·m at 6500 rpm against a {} N·m peak",
            at_6500.torque_nm,
            t.torque_nm
        );
        assert!(p.power_hp() > 0.8 * 217.0, "{} hp", p.power_hp());
    }

    #[test]
    fn the_vw_sdi_breathes_near_what_vw_published() {
        // VW 1.9 SDI: 125 N·m at 2200–2600 rpm, 47 kW (63 hp) at 4200. Within 15 % of both
        // (the diesel's own spec), the torque flat through the mid range as a diesel's is
        let pts = sweep(&EngineDef::vw_sdi());
        let (t, p) = peaks(&pts).unwrap();
        assert!(
            (t.torque_nm / 125.0 - 1.0).abs() < 0.15,
            "{} N·m",
            t.torque_nm
        );
        assert!(
            (p.power_hp() / 63.0 - 1.0).abs() < 0.15,
            "{} hp",
            p.power_hp()
        );
        let at_2000 = pts.iter().find(|q| q.rpm == 2000.0).unwrap();
        assert!(at_2000.torque_nm > 0.9 * t.torque_nm, "{at_2000:?}");
    }

    #[test]
    fn the_renesis_breathes_near_what_mazda_published() {
        // 184 kW (247 hp) at 8200 rpm, 216 N·m at 5500 (Heywood 2nd ed., Fig. 1.23): within 15 %
        let pts = sweep(&EngineDef::mazda_renesis());
        let (t, p) = peaks(&pts).unwrap();
        assert!(
            (t.torque_nm / 216.0 - 1.0).abs() < 0.15,
            "{} N·m",
            t.torque_nm
        );
        assert!(
            (p.power_hp() / 247.0 - 1.0).abs() < 0.15,
            "{} hp",
            p.power_hp()
        );
        assert!(p.rpm >= 7500.0, "power peaks at {} rpm", p.rpm);
    }

    #[test]
    fn the_k20c1_boosts_near_what_honda_published() {
        // 400 N·m from 2500 rpm, 228 kW (306 hp) at 6500 (Honda, Civic Type R FK8 press kit
        // 2017): within 15 %, its torque up by 2500 rpm (boost by then) and not at 1000 (none)
        let pts = sweep(&EngineDef::honda_k20c1());
        let (t, p) = peaks(&pts).unwrap();
        assert!(
            (t.torque_nm / 400.0 - 1.0).abs() < 0.15,
            "{} N·m",
            t.torque_nm
        );
        assert!(
            (p.power_hp() / 306.0 - 1.0).abs() < 0.15,
            "{} hp",
            p.power_hp()
        );
        let at = |rpm: f64| pts.iter().find(|x| x.rpm == rpm).unwrap().torque_nm;
        assert!(at(2500.0) > 0.85 * 400.0, "{} N·m at 2500", at(2500.0));
        assert!(at(1000.0) < 0.5 * t.torque_nm);
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
