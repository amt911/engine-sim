//! Benchmark: cell-updates per second of the FV pipe solver (one core).
//! `cargo run -p engine-core --release --example fv_bench`
use engine_core::fv::{Boundary, Pipe1d, Prim};
use std::time::Instant;
fn main() {
    let air = Prim {
        rho: 0.6,
        u: 20.0,
        p: 1.2e5,
    };
    let mut p = Pipe1d::new(1.0, 400, air);
    p.right = Boundary::Open { p: 1.0e5, rho: 0.6 };
    let dt = p.max_dt(0.5);
    let steps = 20_000;
    let t = Instant::now();
    for _ in 0..steps {
        p.step(dt);
    }
    let s = t.elapsed().as_secs_f64();
    let updates = 400.0 * steps as f64 / s;
    println!(
        "{:.1} M cell-updates/s ({:.2} µs per 400-cell step)",
        updates / 1e6,
        s / steps as f64 * 1e6
    );
    // LFA: ~240 cells at 24 k FV steps/s needs 5.8 M cell-updates/s
    println!(
        "LFA budget (240 cells × 24 kHz): {:.0} % of one core",
        240.0 * 24_000.0 / updates * 100.0
    );
    // Whole engine, as the app runs it (192 kHz physics): LFA at 8000 rpm WOT.
    use engine_core::exhaust::ExhaustModel;
    for (name, exhaust, fv_intake) in [
        (
            "waveguide exhaust + 0D intake",
            ExhaustModel::Waveguide,
            false,
        ),
        (
            "waveguide exhaust + 10 ITB runners",
            ExhaustModel::Waveguide,
            true,
        ),
        ("FV exhaust + 0D intake", ExhaustModel::Fv, false),
        ("FV exhaust + 10 ITB runners", ExhaustModel::Fv, true),
    ] {
        let mut d = engine_core::engine::EngineDef::lfa_v10();
        d.exhaust.model = exhaust;
        // the preset is on ITBs: the 0D rows switch the intake back explicitly
        d.intake.model = if fv_intake {
            engine_core::intake_fv::IntakeModel::Fv
        } else {
            engine_core::intake_fv::IntakeModel::ZeroD
        };
        d.intake.individual_throttles = true;
        let mut e = engine_core::engine::Engine::new(d, 8000.0, 1);
        let c = engine_core::engine::Controls {
            throttle: 1.0,
            hold_rpm: Some(8000.0),
            ..Default::default()
        };
        let dt = 1.0 / 192_000.0;
        let t = Instant::now();
        for _ in 0..96_000 {
            e.step(dt, &c);
        }
        println!(
            "LFA {name}: {:.2}× real time",
            0.5 / t.elapsed().as_secs_f64()
        );
    }
}
