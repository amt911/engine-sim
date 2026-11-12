//! Probe: the bridge-port 13B's idle over 15 s on each exhaust model and rate.
use engine_core::engine::*;
use engine_core::exhaust::ExhaustModel;
fn main() {
    for (fs, ex) in [
        (192_000.0, ExhaustModel::Fv),
        (192_000.0, ExhaustModel::Waveguide),
    ] {
        let dt = 1.0 / fs;
        let mut d = EngineDef::mazda_13b_bridgeport();
        d.exhaust.model = ex;
        let mut e = Engine::new(d.clone(), d.idle_rpm, 1);
        let mut line = format!("{fs} {ex:?}:");
        for _ in 0..30 {
            for _ in 0..(0.5 / dt) as usize {
                e.step(dt, &Controls::default());
            }
            line += &format!(" {:.0}", e.rpm());
        }
        println!("{line}");
    }
}
