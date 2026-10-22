//! Probe: first step where the FV-exhaust LFA at closed throttle goes non-finite.
use engine_core::engine::*;
use engine_core::exhaust::ExhaustModel;
fn main() {
    let dt = 1.0 / 192_000.0;
    let mut d = EngineDef::lfa_v10();
    d.exhaust.model = ExhaustModel::Fv;
    let mut e = Engine::new(d, 7000.0, 1);
    let c = Controls {
        throttle: 0.0,
        hold_rpm: Some(7000.0),
        ..Controls::default()
    };
    let mut hist: Vec<String> = Vec::new();
    for k in 0..(0.6 / dt) as usize {
        let x = e.step(dt, &c);
        let worst = (0..10)
            .map(|i| e.cylinder(i).pressure_pa)
            .fold(0.0, f64::max);
        let port = (0..10)
            .map(|i| e.exhaust_port_gauge(i))
            .fold(0.0f64, |m, v| m.max(v.abs()));
        hist.push(format!(
            "k{k} x {x:.3e} max cyl {worst:.3e} max |port| {port:.3e}"
        ));
        if hist.len() > 6 {
            hist.remove(0);
        }
        if !x.is_finite() || !worst.is_finite() || worst > 1e8 {
            for h in &hist {
                println!("{h}");
            }
            let i = (0..10)
                .max_by(|a, b| {
                    e.cylinder(*a)
                        .pressure_pa
                        .total_cmp(&e.cylinder(*b).pressure_pa)
                })
                .unwrap();
            let v = e.cylinder(i);
            println!("cyl {i}: {v:?}");
            return;
        }
    }
    println!("stable for 0.6 s");
}
