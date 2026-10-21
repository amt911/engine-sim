//! Calibration probe: idle rpm and WOT torque/power for the built-in engines.
//! `cargo run -p engine-core --release --example calibrate`
use engine_core::engine::*;
fn main() {
    let dt = 1.0 / 192_000.0;
    for def0 in [
        EngineDef::single(),
        EngineDef::inline4(),
        EngineDef::lfa_v10(),
    ] {
        for idle in [def0.idle_throttle] {
            let mut d = def0.clone();
            d.idle_throttle = idle;
            let mut e = Engine::new(d, 900.0, 1);
            let c = Controls::default();
            for _ in 0..(3.0 / dt) as usize {
                e.step(dt, &c);
            }
            println!(
                "{} idle {idle}: rpm {:.0} plenum {:.2} bar",
                def0.name,
                e.rpm(),
                e.plenum_pressure_pa() / 1e5
            );
        }
        for rpm in [2000.0, 4000.0, 6800.0, 8700.0] {
            let mut e = Engine::new(def0.clone(), rpm, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            for _ in 0..(1.5 / dt) as usize {
                e.step(dt, &c);
            }
            println!(
                "  WOT {rpm}: {:.1} Nm {:.1} hp",
                e.torque_nm(),
                e.power_w() / 745.7
            );
        }
    }
}
