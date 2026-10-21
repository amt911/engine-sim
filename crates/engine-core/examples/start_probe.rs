//! Probe: stall then restart with the starter, printing rpm / crank / cylinder pressures.
//! `cargo run -p engine-core --release --example start_probe`
use engine_core::engine::*;
fn main() {
    let dt = 1.0 / 192_000.0;
    for d in [
        EngineDef::single(),
        EngineDef::inline4(),
        EngineDef::lfa_v10(),
    ] {
        let mut e = Engine::new(d.clone(), 900.0, 1);
        let run = |e: &mut Engine, secs: f64, c: &Controls| {
            for _ in 0..(secs / dt) as usize {
                e.step(dt, c);
            }
        };
        run(&mut e, 1.0, &Controls::default());
        run(
            &mut e,
            4.0,
            &Controls {
                ignition: false,
                ..Controls::default()
            },
        );
        let p: Vec<String> = (0..d.cylinders)
            .map(|i| format!("{:.1}", e.cylinder(i).pressure_pa / 1e5))
            .collect();
        println!(
            "{}: stalled rpm {:.1} crank {:.1}° cyl bar [{}]",
            d.name,
            e.rpm(),
            e.crank_deg(),
            p.join(" ")
        );
        let start = Controls {
            starter: true,
            ..Controls::default()
        };
        for k in 0..10 {
            run(&mut e, 0.2, &start);
            println!(
                "  starter t={:.1}s rpm {:.0} crank {:.0}° p1 {:.1} bar",
                0.2 * (k + 1) as f64,
                e.rpm(),
                e.crank_deg(),
                e.cylinder(0).pressure_pa / 1e5
            );
        }
        run(&mut e, 2.0, &Controls::default());
        println!("  released: rpm {:.0}", e.rpm());
    }
}
