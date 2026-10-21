//! Probe: raw acoustic source (pre-DSP) statistics per operating point, and the
//! context of the largest sample-to-sample jump.
//! `cargo run -p engine-core --release --example source_probe`
use engine_core::engine::*;
fn main() {
    let dt = 1.0 / 192_000.0;
    for d in [EngineDef::single(), EngineDef::lfa_v10()] {
        for t in [0.0, 1.0] {
            let c = Controls {
                throttle: t,
                hold_rpm: Some(6000.0),
                ..Controls::default()
            };
            let mut e = Engine::new(d.clone(), 6000.0, 1);
            for _ in 0..(0.4 / dt) as usize {
                e.step(dt, &c);
            }
            let mut rows = Vec::new();
            for _ in 0..(0.2 / dt) as usize {
                let x = e.step(dt, &c);
                let v = e.cylinder(0);
                rows.push((
                    x,
                    v.cycle_deg,
                    v.pressure_pa,
                    v.exhaust_lift_m,
                    v.intake_lift_m,
                ));
            }
            let xs: Vec<f64> = rows.iter().map(|r| r.0).collect();
            let mean = xs.iter().sum::<f64>() / xs.len() as f64;
            let ac = (xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / xs.len() as f64).sqrt();
            let (i, _) = xs
                .windows(2)
                .enumerate()
                .max_by(|a, b| (a.1[1] - a.1[0]).abs().total_cmp(&(b.1[1] - b.1[0]).abs()))
                .unwrap();
            println!(
                "{} t={t}: mean {mean:.0} ac-rms {ac:.0}, biggest jump at row {i}:",
                d.name
            );
            for r in &rows[i.saturating_sub(3)..(i + 4).min(rows.len())] {
                println!(
                    "   x {:>8.0}  cyl1 {:>6.2}°  p {:>7.0}  ex {:.5}  in {:.5}",
                    r.0, r.1, r.2, r.3, r.4
                );
            }
        }
    }
}
