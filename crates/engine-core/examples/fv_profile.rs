//! Profile target: the cross-plane V8 with both FV models, WOT at 6000 rpm, 2 s simulated.
//! `perf record -g target/release/examples/fv_profile`
use engine_core::engine::*;
use engine_core::exhaust::ExhaustModel;
use engine_core::intake_fv::IntakeModel;
fn main() {
    let dt = 1.0 / (48_000.0 * 4.0);
    let mut d = EngineDef::crossplane_v8();
    d.exhaust.model = ExhaustModel::Fv;
    d.intake.model = IntakeModel::Fv;
    let mut e = Engine::new(d, 6000.0, 1);
    let c = Controls {
        throttle: 1.0,
        hold_rpm: Some(6000.0),
        ..Controls::default()
    };
    let mut acc = 0.0;
    for _ in 0..(2.0 / dt) as usize {
        acc += e.step(dt, &c);
    }
    println!("{acc}");
}
