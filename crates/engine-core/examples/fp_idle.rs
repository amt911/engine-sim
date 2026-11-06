//! Probe: the flat-plane V8's idle with each intake model (rpm, plenum, fires, torque).
use engine_core::engine::*;
use engine_core::exhaust::ExhaustModel;
use engine_core::intake_fv::IntakeModel;
fn main() {
    let dt = 1.0 / 192_000.0;
    for (m, ex) in [
        (IntakeModel::Fv, ExhaustModel::Waveguide),
        (IntakeModel::Fv, ExhaustModel::Fv),
        (IntakeModel::ZeroD, ExhaustModel::Fv),
    ] {
        let mut d = EngineDef::flatplane_v8();
        d.intake.model = m;
        d.exhaust.model = ex;
        if std::env::args().nth(1).is_some() {
            d.idle_throttle = 0.45 * 0.0097;
        }
        let mut e = Engine::new(d.clone(), d.idle_rpm, 1);
        println!("== {m:?} {ex:?}");
        for t in 0..40 {
            let n0 = e.combustions();
            for _ in 0..(0.1 / dt) as usize {
                e.step(dt, &Controls::default());
            }
            println!(
                "t={:.1} rpm {:>5.0} fires +{} plenum {:.3} torque {:>6.1}",
                0.1 * (t + 1) as f64,
                e.rpm(),
                e.combustions() - n0,
                e.plenum_pressure_pa() / 1e5,
                e.torque_nm()
            );
        }
    }
}
