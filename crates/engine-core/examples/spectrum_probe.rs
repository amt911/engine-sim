//! Probe: where the radiated source's energy sits (band energies), coast vs WOT, FV exhaust.
//! `cargo run -p engine-core --release --example spectrum_probe -- 7000`
use engine_core::engine::*;
use engine_core::exhaust::ExhaustModel;
use engine_core::sound::SoundDef;
fn main() {
    let rpm: f64 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(7000.0);
    let dt = 1.0 / 192_000.0;
    for t in [0.0, 1.0] {
        let mut d = EngineDef::lfa_v10();
        d.exhaust.model = ExhaustModel::Fv;
        d.sound = SoundDef {
            flow_noise: 0.0,
            intake: 0.0,
            mechanical: 0.0,
            combustion_jitter: 0.0,
            reverb: 0.0,
            leveling: 0.0,
        };
        let mut e = Engine::new(d, rpm, 1);
        let c = Controls {
            throttle: t,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        for _ in 0..(0.3 / dt) as usize {
            e.step(dt, &c);
        }
        let xs: Vec<f64> = (0..(0.1 / dt) as usize).map(|_| e.step(dt, &c)).collect();
        let m = xs.iter().sum::<f64>() / xs.len() as f64;
        // energy per band via simple DFT bins (Goertzel-ish at band centres)
        let band = |f: f64| {
            let w = 2.0 * std::f64::consts::PI * f * dt;
            let (mut re, mut im) = (0.0, 0.0);
            for (n, x) in xs.iter().enumerate() {
                re += (x - m) * (w * n as f64).cos();
                im += (x - m) * (w * n as f64).sin();
            }
            2.0 * (re * re + im * im).sqrt() / xs.len() as f64
        };
        let f0 = rpm / 120.0;
        let mut line = format!("t={t}: ");
        for f in [
            f0 * 10.0,
            f0 * 20.0,
            1000.0,
            2000.0,
            3000.0,
            4000.0,
            6000.0,
            8000.0,
            12000.0,
            16000.0,
        ] {
            line += &format!("{:.0}Hz:{:.0} ", f, band(f));
        }
        let crossings = xs
            .windows(2)
            .filter(|w| (w[0] - m) * (w[1] - m) < 0.0)
            .count();
        line += &format!(
            "| zero-crossing rate {:.0} Hz",
            crossings as f64 / 2.0 / 0.1
        );
        println!("{line}");
    }
}
