//! Probe: idle rpm vs idle air for one preset. `--example idle_sweep -- lfa|i4|single`
use engine_core::engine::*;
fn main() {
    let which = std::env::args().nth(1).unwrap_or("lfa".into());
    let base = match which.as_str() {
        "i4" => EngineDef::inline4(),
        "single" => EngineDef::single(),
        _ => EngineDef::lfa_v10(),
    };
    let dt = 1.0 / 192_000.0;
    for idle in [0.0015, 0.0025, 0.004, 0.006, 0.01] {
        let mut d = base.clone();
        d.idle_throttle = idle;
        let mut e = Engine::new(d, 1000.0, 1);
        let mut trace = Vec::new();
        for k in 0..30 {
            for _ in 0..(0.1 / dt) as usize {
                e.step(dt, &Controls::default());
            }
            if k % 5 == 4 {
                trace.push(format!("{:.0}", e.rpm()));
            }
        }
        println!(
            "{which} idle {idle}: rpm over 3 s [{}] plenum {:.2}",
            trace.join(" "),
            e.plenum_pressure_pa() / 1e5
        );
    }
}
