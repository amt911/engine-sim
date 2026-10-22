//! Tip-in click probe: high-frequency energy (first difference, RMS per 5 ms) around a pedal
//! step from idle, with each sound source on and off, to find which one clicks.
use engine_core::engine::{Controls, Engine, EngineDef};
use engine_core::sound::SoundDef;

fn main() {
    let dt = 1.0 / 192_000.0;
    for (label, f) in [
        ("all", (1.0, 1.0, 1.0)),
        ("no intake", (1.0, 0.0, 1.0)),
        ("no flow", (0.0, 1.0, 1.0)),
        ("no mech", (1.0, 1.0, 0.0)),
        ("none", (0.0, 0.0, 0.0)),
    ] {
        let mut d = EngineDef::inline4();
        let s0 = d.sound.clone();
        d.sound = SoundDef {
            flow_noise: s0.flow_noise * f.0,
            intake: s0.intake * f.1,
            mechanical: s0.mechanical * f.2,
            ..s0
        };
        let mut e = Engine::new(d, 0.0, 1);
        let crank = Controls {
            starter: true,
            ..Controls::default()
        };
        for _ in 0..(1.0 / dt) as usize {
            e.step(dt, &crank);
        }
        for _ in 0..(3.0 / dt) as usize {
            e.step(dt, &Controls::default());
        }
        let wot = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        // 50 ms before, 100 ms after: first difference (a crude high-pass) energy per 5 ms
        let mut y = Vec::new();
        for _ in 0..(0.05 / dt) as usize {
            y.push(e.step(dt, &Controls::default()));
        }
        for _ in 0..(0.10 / dt) as usize {
            y.push(e.step(dt, &wot));
        }
        let w = (0.005 / dt) as usize;
        let rms: Vec<String> = y
            .windows(2)
            .map(|p| (p[1] - p[0]).powi(2))
            .collect::<Vec<_>>()
            .chunks(w)
            .map(|c| {
                format!(
                    "{:.2}",
                    (c.iter().sum::<f64>() / c.len() as f64).sqrt() * 1e3
                )
            })
            .collect();
        let peak = y.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        println!("{label:10} peak {peak:.3} | {}", rms.join(" "));
    }
}
