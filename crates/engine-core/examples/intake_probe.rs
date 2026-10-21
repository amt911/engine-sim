//! Probe: WOT torque vs rpm for 1D intakes with different runner lengths (ram tuning).
use engine_core::engine::*;
use engine_core::intake_fv::IntakeModel;
fn main() {
    let dt = 1.0 / 192_000.0;
    for (runner, model) in [
        (0.3, None),
        (0.15, Some(true)),
        (0.6, Some(true)),
        (0.3, Some(false)),
    ] {
        let mut line = format!("runner {runner} m {model:?}:");
        for rpm in [2000.0, 3000.0, 4000.0, 5000.0, 6500.0] {
            let mut d = EngineDef::inline4();
            if let Some(itb) = model {
                d.intake.model = IntakeModel::Fv;
                d.intake.individual_throttles = itb;
                d.intake.runner_length_m = runner;
            }
            let mut e = Engine::new(d, rpm, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            for _ in 0..(0.6 / dt) as usize {
                e.step(dt, &c);
            }
            let m0 = e.inducted_kg();
            for _ in 0..(0.2 / dt) as usize {
                e.step(dt, &c);
            }
            let ideal = 1.18 * e.def().displacement_m3() * rpm / 120.0 * 0.2;
            line += &format!(
                " {rpm}:{:.0}Nm/VE{:.2}",
                e.torque_nm(),
                (e.inducted_kg() - m0) / ideal
            );
        }
        println!("{line}");
    }
}
