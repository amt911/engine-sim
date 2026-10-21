//! Probe: restart success over many stall positions.
//! `cargo run -p engine-core --release --example start_sweep`
use engine_core::engine::*;
fn main() {
    let dt = 1.0 / 192_000.0;
    for d in [
        EngineDef::single(),
        EngineDef::inline4(),
        EngineDef::lfa_v10(),
    ] {
        let mut fails = Vec::new();
        for k in 0..60u64 {
            let mut e = Engine::new(d.clone(), 900.0, k);
            let run = |e: &mut Engine, secs: f64, c: &Controls| {
                for _ in 0..(secs / dt) as usize {
                    e.step(dt, c);
                }
            };
            run(&mut e, 0.5 + 0.013 * k as f64, &Controls::default());
            run(
                &mut e,
                5.0,
                &Controls {
                    ignition: false,
                    ..Controls::default()
                },
            );
            let stall = e.crank_deg();
            let mut max_rpm: f64 = 0.0;
            for _ in 0..10 {
                run(
                    &mut e,
                    0.1,
                    &Controls {
                        starter: true,
                        ..Controls::default()
                    },
                );
                max_rpm = max_rpm.max(e.rpm());
            }
            run(&mut e, 1.5, &Controls::default());
            if e.rpm() < 300.0 {
                fails.push(format!(
                    "stall@{stall:.0}° max-crank {max_rpm:.0} → {:.0}",
                    e.rpm()
                ));
            }
        }
        println!(
            "{}: {} / 60 failed {:?}",
            d.name,
            fails.len(),
            &fails[..fails.len().min(5)]
        );
    }
}
