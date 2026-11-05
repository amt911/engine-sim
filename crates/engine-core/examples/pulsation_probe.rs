//! Probe: how much the flow out of the tail pulses against its mean — the exit flow's rms ripple
//! over its mean and its swing ((max − min)/mean) over 0.5 s held, and its first firing harmonic
//! over the mean.
//! `cargo run -p engine-core --release --example pulsation_probe -- PRESET.yaml RPM LOAD`
use engine_core::engine::*;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let text = std::fs::read_to_string(&a[1]).expect("preset");
    let def = engine_core::config::from_str(&text, engine_core::config::Format::Yaml).expect("yaml");
    let rpm: f64 = a[2].parse().expect("rpm");
    let thr: f64 = a[3].parse().expect("load");
    let dt = 1.0 / 48_000.0;
    let mut e = Engine::new(def, rpm, 1);
    let c = Controls {
        throttle: thr,
        hold_rpm: Some(rpm),
        ..Controls::default()
    };
    for _ in 0..(1.0 / dt) as usize {
        e.step(dt, &c);
    }
    let mut out = Vec::new();
    let mut x = Vec::new();
    for _ in 0..(0.5 / dt) as usize {
        e.step(dt, &c);
        e.exit_flows(&mut out);
        x.push(out.iter().sum::<f64>());
    }
    let n = x.len() as f64;
    let mean = x.iter().sum::<f64>() / n;
    let rms = (x.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n).sqrt();
    let (lo, hi) = x.iter().fold((f64::MAX, f64::MIN), |(l, h), &v| (l.min(v), h.max(v)));
    println!(
        "{} {rpm} {thr}: mean {mean:.4} kg/s, rms ripple {:.0} %, swing {:.0} %",
        a[1].rsplit('/').next().unwrap_or(""),
        100.0 * rms / mean,
        100.0 * (hi - lo) / mean
    );
}
