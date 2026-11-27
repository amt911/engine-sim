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
    let segs = match std::env::var("CAN") {
        // CAN="length,shell,tube": one packed can instead of the road rule
        Ok(c) => {
            let v: Vec<f64> = c.split(',').map(|x| x.parse().expect("can")).collect();
            vec![Segment::Muffler {
                length_m: v[0],
                diameter_m: v[1],
                tube_diameter_m: v[2],
                packing_resistivity: 16_000.0,
                packing_porosity: 0.95,
            }]
        }
        Err(_) => road_exhaust(swept, d),
    };
    for seg in segs {
        if let Segment::Muffler {
            length_m,
            diameter_m,
            tube_diameter_m,
            packing_resistivity,
            ..
        } = seg
        {
            let mach: f64 = std::env::var("MACH")
                .ok()
                .and_then(|m| m.parse().ok())
                .unwrap_or(0.0);
            let s = Silencer {
                r1: 0.5 * tube_diameter_m,
                r2: 0.5 * diameter_m,
                len: length_m,
                resistivity: (packing_resistivity > 0.0).then_some(packing_resistivity),
                perforate: Perforate::default(),
                mach,
            };
            let eq = s.transmission_eq(gas, fs);
            print!("silencer L {length_m} D {diameter_m:.3} tube {tube_diameter_m}:");
            for f in [140.0, 160.0, 185.0, 207.0, 230.0, 250.0, 270.0, 300.0] {
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
