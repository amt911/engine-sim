//! The timing drive's rattle: the random mechanical noise of an idling engine.
//!
//! Each valve's spring pushes its follower against the lobe; through the lobe's slope that is a
//! torque on the camshaft that resists while the valve opens and drives the cam forward while it
//! closes (Norton, *Cam Design and Manufacturing Handbook*, 2nd ed., ch. 9: the spring force
//! dominates the follower's load at idle speeds, its inertia at high ones). The camshaft's
//! sprocket rides the crank through the chain or gears with some backlash; when the cam torque
//! reverses harder than the valvetrain's friction holds it, the sprocket crosses the backlash and
//! hits the other flank — the loose-gear rattle of Singh, Xie & Comparin (*J. Sound Vib.* 131(2),
//! 1989), driven here by the cam's own torque. Its impacts do not repeat from one cycle to the
//! next, as an idling engine's 1–8 kHz does not (design: docs/superpowers/specs/
//! 2026-11-12-timing-drive-rattle-design.md); at idle the valve train is the main mechanical
//! source, "caused mainly by the gear rattling owing to the variation in the camshaft drive
//! torque" (SAE 2003-01-1720).

use crate::valve::ValveTiming;

/// Margin of a valve spring's open load over the follower's inertia force at the redline, so the
/// follower never leaves the lobe: Norton (ch. 9) sizes springs 1.2–1.5 × the peak negative
/// inertia force; the middle (ESTIMATE).
pub const NO_FLOAT_MARGIN: f64 = 1.35;

/// The spring's load with the valve shut, as a share of its open load. ESTIMATE: street engines'
/// springs 90–120 lb on the seat against 220–300 lb open (fastimeperformance.com spring chart),
/// a production-based Ecotec's 83 lb against ~230 lb (BK Racing): 0.36–0.41.
pub const SEAT_SHARE: f64 = 0.4;

/// The moving mass a valve's spring holds on its lobe, kg: the valve, its retainer and keepers,
/// a third of the spring and the follower. ESTIMATE: ~80 g for a 33 mm intake valve with a bucket
/// follower (passenger-car valves 40–55 g, buckets 25–40 g), scaling with the head's area.
pub fn valve_train_mass_kg(valve_d_m: f64) -> f64 {
    0.08 * (valve_d_m / 0.033).powi(2)
}

/// The spring that holds `v`'s follower on its lobe up to `redline_rpm`: (seat load N, rate N/m).
/// The sin² lift `L·sin²(πx)` over `D` crank radians decelerates hardest over the nose,
/// `2π²L/D²` per crank rad², so at the redline's crank speed ω the open load is
/// [`NO_FLOAT_MARGIN`] × m·2π²L·ω²/D², the seat load [`SEAT_SHARE`] of it.
pub fn spring(v: &ValveTiming, redline_rpm: f64) -> (f64, f64) {
    let d = v.duration_deg.to_radians();
    let omega = redline_rpm * std::f64::consts::TAU / 60.0;
    let accel = 2.0 * std::f64::consts::PI.powi(2) * v.max_lift_m * omega * omega / (d * d);
    let open = NO_FLOAT_MARGIN * valve_train_mass_kg(v.diameter_m) * accel;
    let seat = SEAT_SHARE * open;
    let rate = if v.max_lift_m > 0.0 {
        (open - seat) / v.max_lift_m
    } else {
        0.0
    };
    (seat, rate)
}

/// Torque `v`'s spring puts on its camshaft at `cycle_deg` (crank degrees in the 720° cycle),
/// N·m: positive resists the drive (the valve opening), negative pushes the cam on (closing).
/// The cam turns at half the crank's speed, so per cam radian the lift moves twice as far.
pub fn cam_torque(v: &ValveTiming, seat_n: f64, rate_n_per_m: f64, cycle_deg: f64) -> f64 {
    let x = crate::units::wrap(cycle_deg - v.open_deg, 720.0) / v.duration_deg;
    if x >= 1.0 {
        return 0.0;
    }
    let lift = v.lift(cycle_deg);
    let pi = std::f64::consts::PI;
    // d(L·sin²(πx))/dθ_crank = L·π/D·sin(2πx); per cam radian twice that
    let slope = 2.0 * v.max_lift_m * pi / v.duration_deg.to_radians() * (2.0 * pi * x).sin();
    (seat_n + rate_n_per_m * lift) * slope
}

/// A camshaft's moment of inertia with its sprocket, kg·m². ESTIMATE: a steel shaft of ~25 mm
/// over a four-cylinder head (~0.45 m) and its lobes, ~0.5e-3, with a 90 mm sprocket and phaser
/// ~1.5e-3: 2e-3 for four cylinders, in proportion to the cylinders it serves.
pub fn camshaft_inertia_kgm2(cylinders: usize) -> f64 {
    2e-3 * cylinders as f64 / 4.0
}

/// Pitch radius of a cam sprocket, m: twice the crank's (the cam turns at half speed) on a
/// ~45 mm one (ESTIMATE: 18–21 teeth of 8 mm chain on the crank).
pub const SPROCKET_RADIUS_M: f64 = 0.045;

/// Backlash of the drive at the sprocket's pitch circle, m. ESTIMATE: timing gears are set to
/// 0.05–0.15 mm; a chain's slack held by its tensioner, the same order.
pub const BACKLASH_M: f64 = 1e-4;

/// Stiffness of the drive's contact (a tooth on a chain roller or a gear tooth), N/m. ESTIMATE:
/// a gear pair's c′ = 14 N/(mm·µm) over a 10 mm face (ISO 6336), 1.4e8; a chain roller
/// similar.
pub const CONTACT_STIFFNESS: f64 = 1.4e8;

/// Damping ratio of the contact: Kahraman & Singh (*J. Sound Vib.* 142(1), 1990) measured gear
/// mesh damping 0.03–0.17; the middle (ESTIMATE).
pub const CONTACT_DAMPING: f64 = 0.07;

/// A camshaft and sprocket in the drive's backlash: the loose gear of Singh et al. with the cam
/// torque in place of a steady oil drag.
#[derive(Debug, Clone)]
pub struct CamSprocket {
    inertia: f64,
    /// Position relative to the drive along the pitch circle, m (the drive flank at −b/2).
    x: f64,
    /// The sprocket's own angular speed, rad/s.
    omega: f64,
    fresh: bool,
}

impl CamSprocket {
    pub fn new(inertia_kgm2: f64) -> Self {
        Self {
            inertia: inertia_kgm2,
            x: 0.0,
            omega: 0.0,
            fresh: true,
        }
    }

    /// Advance by `dt` with the drive turning the sprocket at `drive_omega` (half the crank's
    /// speed) against `cam_torque_nm` (positive resists) and the valvetrain's friction `drag_nm`;
    /// returns the contact force on the drive, N (the drive flank positive, the other negative,
    /// 0 in the backlash), its mean over the step.
    pub fn step(&mut self, drive_omega: f64, cam_torque_nm: f64, drag_nm: f64, dt: f64) -> f64 {
        let r = SPROCKET_RADIUS_M;
        let half = 0.5 * BACKLASH_M;
        let k = CONTACT_STIFFNESS;
        let m_eff = self.inertia / (r * r);
        let c = 2.0 * CONTACT_DAMPING * (k * m_eff).sqrt();
        if self.fresh {
            // resting on the drive flank under the load it carries
            self.x = -half - (cam_torque_nm + drag_nm).max(0.0) / (r * k);
            self.omega = drive_omega;
            self.fresh = false;
        }
        // keep the contact oscillation under 0.5 rad per sub-step
        let n = ((k / m_eff).sqrt() * dt / 0.5).ceil().max(1.0);
        let h = dt / n;
        let mut mean = 0.0;
        for _ in 0..n as u32 {
            let v_rel = (self.omega - drive_omega) * r;
            let force = if self.x < -half {
                (k * (-half - self.x) - c * v_rel).max(0.0)
            } else if self.x > half {
                -(k * (self.x - half) + c * v_rel).max(0.0)
            } else {
                0.0
            };
            // the drive pushes the sprocket on with `force`; the cam and friction hold it back
            let torque = r * force - cam_torque_nm - drag_nm;
            self.omega += h * torque / self.inertia;
            self.x += h * (self.omega - drive_omega) * r;
            mean += force / n;
        }
        mean
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn k20_intake() -> ValveTiming {
        ValveTiming {
            open_deg: 0.0,
            duration_deg: 240.0,
            max_lift_m: 0.011,
            diameter_m: 0.035,
            high: None,
        }
    }

    #[test]
    fn the_spring_by_hand() {
        // m = 0.08·(35/33)² = 0.089991 kg; D = 240° = 4.18879 rad, ω at 7000 rpm = 733.038 rad/s;
        // a = 2π²·0.011/4.18879²·733.038² = 0.217131/17.54596·537 344.7 = 6649.8 m/s²;
        // open = 1.35·0.089991·6649.8 = 807.86 N, seat 0.4 of it = 323.14 N,
        // rate (807.86 − 323.14)/0.011 = 44 065 N/m
        let (seat, rate) = spring(&k20_intake(), 7000.0);
        assert!((seat - 323.14).abs() < 0.5, "seat {seat}");
        assert!((rate - 44_065.0).abs() < 50.0, "rate {rate}");
    }

    #[test]
    fn the_cam_torque_a_quarter_into_the_lobe_by_hand() {
        // x = 0.25 (60° into 240°): lift L·sin²(π/4) = L/2, dl/dθ_crank = L·π/D·sin(π/2) =
        // 0.011·π/4.18879 = 8.2500e-3 m/rad, per cam rad twice that, 0.016500 m/rad;
        // force 300 + 40 000·0.0055 = 520 N: 520·0.016500 = 8.580 N·m, resisting.
        // At x = 0.75 the valve closes: the same, pushing the cam on.
        let v = k20_intake();
        let t = cam_torque(&v, 300.0, 40_000.0, 60.0);
        assert!((t - 8.580).abs() < 0.01, "{t}");
        let back = cam_torque(&v, 300.0, 40_000.0, 180.0);
        assert!((back + 8.580).abs() < 0.01, "{back}");
        assert_eq!(cam_torque(&v, 300.0, 40_000.0, 300.0), 0.0);
    }

    #[test]
    fn a_spring_gives_back_over_the_lobe_what_it_took() {
        // F depends on the lift alone: ∫F·dl over a lift that returns to 0 is 0
        let v = k20_intake();
        let n = 24_000;
        let work: f64 = (0..n)
            .map(|i| {
                let deg = 240.0 * (i as f64 + 0.5) / n as f64;
                // per crank degree the cam turns half a degree
                cam_torque(&v, 300.0, 40_000.0, deg) * (0.5 * 240.0 / n as f64).to_radians()
            })
            .sum();
        let peak = cam_torque(&v, 300.0, 40_000.0, 60.0);
        assert!(peak > 1.0, "{peak}");
        assert!(work.abs() < 1e-6 * peak, "{work}");
    }

    /// The intake camshaft of a four-cylinder (firing 1-3-4-2) at `rpm` with the crank's speed
    /// rippling `ripple` at the firing rate: the sprocket's contact force every step at 96 kHz.
    fn idle_drive(rpm: f64, ripple: f64, drag_nm: f64, secs: f64) -> Vec<f64> {
        let v = k20_intake();
        let (seat, rate) = spring(&v, 7000.0);
        let mut s = CamSprocket::new(camshaft_inertia_kgm2(4));
        let dt = 1.0 / 96_000.0;
        let w = rpm * std::f64::consts::TAU / 60.0;
        let mut deg = 0.0;
        (0..(secs / dt) as usize)
            .map(|i| {
                let t = i as f64 * dt;
                let crank = w * (1.0 + ripple * (2.0 * w * t).sin());
                deg = (deg + crank * dt * 180.0 / std::f64::consts::PI) % 720.0;
                let torque: f64 = [0.0, 180.0, 360.0, 540.0]
                    .iter()
                    .map(|&ph| 2.0 * cam_torque(&v, seat, rate, deg - ph))
                    .sum();
                s.step(0.5 * crank, torque, drag_nm, dt)
            })
            .collect()
    }

    /// Impacts: the contact force rising from 0 (the sprocket landing on a flank).
    fn impacts(f: &[f64]) -> usize {
        f.windows(2).filter(|w| w[0] == 0.0 && w[1] != 0.0).count()
    }

    #[test]
    fn a_drag_over_the_cam_torques_swing_holds_the_sprocket_on_its_flank() {
        // Singh's criterion: no impact while the load never reverses. Two valves per lobe
        // position, four positions: the torque swings by tens of N·m; a 200 N·m drag holds it.
        let held = idle_drive(800.0, 0.0, 200.0, 1.0);
        assert_eq!(impacts(&held[9_600..]), 0);
        let free = idle_drive(800.0, 0.0, 2.0, 1.0);
        assert!(impacts(&free[9_600..]) > 20, "{}", impacts(&free[9_600..]));
    }

    #[test]
    fn an_idling_drive_rattles_differently_every_cycle() {
        // Real idles' 1–8 kHz envelope correlates from one engine cycle to the next at most
        // r = 0.40 (blind2's hnr −1.7 dB, best of seven recordings: docs/superpowers/specs/
        // 2026-11-12-timing-drive-rattle-design.md). The rattle's force, cycle against cycle.
        let f = idle_drive(800.0, 0.05, 3.0, 4.0);
        let per = (96_000.0 * 120.0 / 800.0) as usize;
        let cycles: Vec<&[f64]> = f.chunks_exact(per).skip(2).collect();
        let corr = |a: &[f64], b: &[f64]| {
            let (ma, mb) = (
                a.iter().sum::<f64>() / a.len() as f64,
                b.iter().sum::<f64>() / b.len() as f64,
            );
            let (mut ab, mut aa, mut bb) = (0.0, 0.0, 0.0);
            for (x, y) in a.iter().zip(b) {
                ab += (x - ma) * (y - mb);
                aa += (x - ma) * (x - ma);
                bb += (y - mb) * (y - mb);
            }
            ab / (aa * bb).sqrt()
        };
        let r: f64 =
            cycles.windows(2).map(|w| corr(w[0], w[1])).sum::<f64>() / (cycles.len() - 1) as f64;
        assert!(impacts(&f) > 100, "{}", impacts(&f));
        assert!(r < 0.40, "cycle correlation {r}");
    }
}
