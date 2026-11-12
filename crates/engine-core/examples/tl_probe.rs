//! Probe: transmission loss (dB) of the rule-sized road exhaust's packed straight-through
//! silencers ([`engine_core::exhaust::road_exhaust`]) by the 2D model, and the equalizer that
//! carries it, at a few frequencies. `cargo run -p engine-core --release --example tl_probe -- SWEPT_M3 PIPE_D`
use engine_core::exhaust::{Segment, road_exhaust};
use engine_core::silencer2d::{Gas, Perforate, Silencer};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let swept: f64 = a[1].parse().expect("swept");
    let d: f64 = a[2].parse().expect("pipe d");
    let gas = Gas {
        c: 500.0,
        rho: 101_325.0 / (287.0 * 622.0),
    };
    let fs = 192_000.0;
    for seg in road_exhaust(swept, d) {
        if let Segment::Muffler {
            length_m,
            diameter_m,
            tube_diameter_m,
            packing_resistivity,
            ..
        } = seg
        {
            let s = Silencer {
                r1: 0.5 * tube_diameter_m,
                r2: 0.5 * diameter_m,
                len: length_m,
                resistivity: (packing_resistivity > 0.0).then_some(packing_resistivity),
                perforate: Perforate::default(),
            };
            let eq = s.transmission_eq(gas, fs);
            print!("silencer L {length_m} D {diameter_m:.3} tube {tube_diameter_m}:");
            for f in [50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0] {
                print!(
                    "  {f}Hz TL {:.1} eq {:.1}",
                    s.transmission_loss_db(f, gas).unwrap_or(f64::NAN),
                    eq.response_db(f, fs)
                );
            }
            println!();
        }
    }
}
