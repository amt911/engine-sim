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
}
