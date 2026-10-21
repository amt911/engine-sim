//! Probe: how much of a 600 Hz pressure ripple crosses a straight FV pipe (cell-size study).
use engine_core::fv::{Boundary, Cons, Pipe1d, Prim};
fn main() {
    for cell in [0.01, 0.02, 0.04] {
        let gas = Prim {
            rho: 0.6,
            u: 0.0,
            p: 101_325.0,
        };
        let len = 1.5;
        let mut p = Pipe1d::new(len, (len / cell) as usize, gas);
        p.right = Boundary::Open {
            p: 101_325.0,
            rho: 1.18,
        };
        let dt = p.max_dt(0.5);
        let (mut amp_in, mut amp_out) = (0.0f64, 0.0f64);
        let mut t = 0.0;
        while t < 0.05 {
            let m = 20.0 + 8.0 * (2.0 * std::f64::consts::PI * 600.0 * t).sin(); // kg/(s·m²)
            p.left = Boundary::Flux(Cons {
                rho: m,
                mom: m * m / 0.6 + p.prim(0).p,
                e: m * 1004.5 * 900.0,
            });
            p.step(dt);
            t += dt;
            if t > 0.03 {
                amp_in = amp_in.max((p.end_flux.0.rho - 20.0).abs());
                amp_out = amp_out.max((p.end_flux.1.rho - 20.0).abs());
            }
        }
        println!("cell {cell} m: ripple in {amp_in:.2} out {amp_out:.2} kg/(s·m²)");
    }
}
