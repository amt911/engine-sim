//! Probe: one failing I4 restart, step by step (combustions, plenum, rpm).
use engine_core::engine::*;
fn main() {
    let dt = 1.0 / 192_000.0;
    let d = EngineDef::inline4();
    let k = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(2u64);
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
    println!(
        "stall crank {:.0}° plenum {:.2} bar",
        e.crank_deg(),
        e.plenum_pressure_pa() / 1e5
    );
    for t in 0..20 {
        let n0 = e.combustions();
        run(
            &mut e,
            0.05,
            &Controls {
                starter: true,
                ..Controls::default()
            },
        );
        let p: Vec<String> = (0..4)
            .map(|i| format!("{:.1}", e.cylinder(i).pressure_pa / 1e5))
            .collect();
        println!(
            "t={:.2} rpm {:>5.0} fires +{} plenum {:.2} torque {:>6.1} cyl [{}]",
            0.05 * (t + 1) as f64,
            e.rpm(),
            e.combustions() - n0,
            e.plenum_pressure_pa() / 1e5,
            e.torque_nm(),
            p.join(" ")
        );
    }
}
