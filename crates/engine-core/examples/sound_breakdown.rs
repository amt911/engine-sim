//! Probe: raw source rms per texture component (others off), coast vs WOT, at a given rpm.
//! `cargo run -p engine-core --release --example sound_breakdown -- fv 7000`
use engine_core::engine::*;
use engine_core::exhaust::ExhaustModel;
use engine_core::sound::SoundDef;
fn main() {
    let fv = std::env::args().nth(1).as_deref() == Some("fv");
    let rpm: f64 = std::env::args()
        .nth(2)
        .and_then(|a| a.parse().ok())
        .unwrap_or(7000.0);
    let dt = 1.0 / 192_000.0;
    let off = SoundDef {
        flow_noise: 0.0,
        intake: 0.0,
        mechanical: 0.0,
        combustion_jitter: 0.0,
        reverb: 0.0,
    };
    let parts: [(&str, SoundDef); 4] = [
        ("tonal", off.clone()),
        (
            "flow",
            SoundDef {
                flow_noise: 1.0,
                ..off.clone()
            },
        ),
        (
            "intake",
            SoundDef {
                intake: 1.0,
                ..off.clone()
            },
        ),
        (
            "clicks",
            SoundDef {
                mechanical: 1.0,
                ..off.clone()
            },
        ),
    ];
    for t in [0.0, 1.0] {
        let mut line = format!("LFA{} {rpm} t={t}:", if fv { " fv" } else { "" });
        let mut tonal = Vec::new();
        for (name, snd) in &parts {
            let mut d = EngineDef::lfa_v10();
            d.sound = snd.clone();
            if fv {
                d.exhaust.model = ExhaustModel::Fv;
            }
            let mut e = Engine::new(d, rpm, 1);
            let c = Controls {
                throttle: t,
                hold_rpm: Some(rpm),
                ..Controls::default()
            };
            for _ in 0..(0.3 / dt) as usize {
                e.step(dt, &c);
            }
            let xs: Vec<f64> = (0..(0.2 / dt) as usize).map(|_| e.step(dt, &c)).collect();
            if *name == "tonal" {
                tonal = xs.clone();
            }
            // component alone = run minus the tonal run (same seed, same physics)
            let diff: Vec<f64> = xs.iter().zip(&tonal).map(|(a, b)| a - b).collect();
            let src = if *name == "tonal" { &xs } else { &diff };
            let m = src.iter().sum::<f64>() / src.len() as f64;
            let ac = (src.iter().map(|x| (x - m).powi(2)).sum::<f64>() / src.len() as f64).sqrt();
            line += &format!(" {name} {ac:.0}");
        }
        println!("{line}");
    }
}
