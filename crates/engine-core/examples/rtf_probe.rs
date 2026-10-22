//! Benchmark: real-time factor (simulated seconds per wall second, one core) of every preset,
//! with each exhaust/intake model, at the app's step (48 kHz × 4 substeps), WOT at 6000 rpm.
//! A timing job: run it under `scripts/cpu-lock.sh`.
//! `cargo run -p engine-core --release --example rtf_probe`
use engine_core::engine::*;
use engine_core::exhaust::ExhaustModel;
use engine_core::intake_fv::IntakeModel;
use std::time::Instant;
fn main() {
    let dt = 1.0 / (48_000.0 * 4.0);
    let presets = [
        ("single", EngineDef::single()),
        ("inline4", EngineDef::inline4()),
        ("lfa-v10", EngineDef::lfa_v10()),
        ("v8-flatplane", EngineDef::flatplane_v8()),
        ("v8-crossplane", EngineDef::crossplane_v8()),
    ];
    for (name, def) in presets {
        let mut line = format!("{name:14}");
        for (ex, int) in [
            (ExhaustModel::Waveguide, IntakeModel::ZeroD),
            (ExhaustModel::Fv, IntakeModel::ZeroD),
            (ExhaustModel::Fv, IntakeModel::Fv),
        ] {
            let mut d = def.clone();
            d.exhaust.model = ex;
            d.intake.model = int;
            let mut e = Engine::new(d, 6000.0, 1);
            let c = Controls {
                throttle: 1.0,
                hold_rpm: Some(6000.0),
                ..Controls::default()
            };
            for _ in 0..(0.2 / dt) as usize {
                e.step(dt, &c);
            }
            let secs = 1.0;
            let t = Instant::now();
            let mut acc = 0.0;
            for _ in 0..(secs / dt) as usize {
                acc += e.step(dt, &c);
            }
            let rtf = secs / t.elapsed().as_secs_f64();
            line += &format!(" | {ex:?}/{int:?} {rtf:5.2}×");
            std::hint::black_box(acc);
        }
        println!("{line}");
    }
}
