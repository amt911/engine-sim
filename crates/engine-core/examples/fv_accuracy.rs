//! Accuracy probe for the FV pipe scheme: L1 density error against the exact Sod solution
//! (Toro Test 1, t = 0.2) and a smooth acoustic pulse against d'Alembert, at several
//! resolutions, plus the solver's speed. Run it before and after a change to the scheme.
//! `cargo run -p engine-core --release --example fv_accuracy`
use engine_core::fv::{Pipe1d, Prim};
use std::time::Instant;

/// Exact Sod density at (x, t): star values from Toro, Table 4.2.
fn sod_exact(x: f64, t: f64) -> f64 {
    let g = 1.4_f64;
    let (cl, ustar) = (g.sqrt(), 0.92745);
    let cstar = cl - 0.5 * (g - 1.0) * ustar;
    let s = (x - 0.5) / t;
    if s < -cl {
        1.0
    } else if s < ustar - cstar {
        let u = 2.0 / (g + 1.0) * (cl + s);
        let c = cl - 0.5 * (g - 1.0) * u;
        (c / cl).powf(2.0 / (g - 1.0))
    } else if s < ustar {
        0.42632
    } else if s < 1.75216 {
        0.26557
    } else {
        0.125
    }
}

fn run_to(p: &mut Pipe1d, t_end: f64, cfl: f64) {
    let mut t = 0.0;
    while t < t_end {
        let dt = p.max_dt(cfl).min(t_end - t);
        p.step(dt);
        t += dt;
    }
}

fn main() {
    for n in [100, 200, 400] {
        let left = Prim {
            rho: 1.0,
            u: 0.0,
            p: 1.0,
        };
        let mut p = Pipe1d::new(1.0, n, left);
        for i in n / 2..n {
            p.cells[i] = Prim {
                rho: 0.125,
                u: 0.0,
                p: 0.1,
            }
            .to_cons();
        }
        run_to(&mut p, 0.2, 0.8);
        let l1: f64 = (0..n)
            .map(|i| (p.prim(i).rho - sod_exact((i as f64 + 0.5) / n as f64, 0.2)).abs())
            .sum::<f64>()
            / n as f64;
        println!("sod n={n:4}: L1(rho) = {l1:.5}");
    }
    let air = Prim {
        rho: 1.2,
        u: 0.0,
        p: 101_325.0,
    };
    let c = (1.4_f64 * 101_325.0 / 1.2).sqrt();
    for cells_per_sigma in [5.0, 2.5, 1.5] {
        let sigma = 0.01;
        let n = (1.0 / (sigma / cells_per_sigma)) as usize;
        let mut p = Pipe1d::new(1.0, n, air);
        let x_of = |i: usize| (i as f64 + 0.5) / n as f64;
        for i in 0..n {
            let x = x_of(i) - 0.2;
            p.cells[i] = Prim {
                p: 101_325.0 + 500.0 * (-0.5 * (x / sigma).powi(2)).exp(),
                ..air
            }
            .to_cons();
        }
        run_to(&mut p, 1e-3, 0.8);
        let (mut peak, mut l1, mut ref1) = (0.0f64, 0.0, 0.0);
        for i in 0..n {
            let x = x_of(i);
            if x < 0.4 {
                continue;
            }
            let exact = 250.0 * (-0.5 * ((x - 0.2 - c * 1e-3) / sigma).powi(2)).exp();
            let got = p.prim(i).p - 101_325.0;
            peak = peak.max(got);
            l1 += (got - exact).abs();
            ref1 += exact.abs();
        }
        println!(
            "pulse {cells_per_sigma} cells/sigma: peak {:.1} % of 250 Pa, L1 error {:.1} %",
            100.0 * peak / 250.0,
            100.0 * l1 / ref1
        );
    }
    // speed: best of 7 runs (other load on the machine only ever slows a run down)
    let mut best = 0.0f64;
    for _ in 0..7 {
        let air = Prim {
            rho: 0.6,
            u: 20.0,
            p: 1.2e5,
        };
        let mut p = Pipe1d::new(1.0, 400, air);
        let dt = p.max_dt(0.5);
        let steps = 4_000;
        let t = Instant::now();
        for _ in 0..steps {
            p.step(dt);
        }
        best = best.max((400 * steps) as f64 / t.elapsed().as_secs_f64());
    }
    println!("speed: {:.1} M cell-steps/s", best / 1e6);
}
