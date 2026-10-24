//! Offline rendering for files and games: no real-time limit, the finest simulation, scripted
//! scenes. `drive` (idle, three blips, a pull to the limiter, lift-off), `sweep` (full
//! throttle from idle to redline), and `loops`: a grid of held speeds × loads, each a whole
//! number of engine cycles long — at a speed nudged so a cycle is a whole number of samples —
//! so a game can loop it without a seam.

use crate::quality::Quality;
use crate::sim::Simulation;
use engine_core::drivetrain::VehicleDef;
use engine_core::engine::{Controls, EngineDef};

/// A scripted scene.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scene {
    Drive,
    Sweep,
    Loops,
}

impl std::str::FromStr for Scene {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s.to_ascii_lowercase().as_str() {
            "drive" => Ok(Scene::Drive),
            "sweep" => Ok(Scene::Sweep),
            "loops" => Ok(Scene::Loops),
            _ => Err(format!("unknown scene '{s}' (drive, sweep, loops)")),
        }
    }
}

/// The drive scene's throttle at `t` seconds: idle, blips at 2, 3 and 4 s (0.25 s each), full
/// throttle from 5 to 7.5 s, then lift-off to 11 s.
pub fn drive_throttle(t: f64) -> f64 {
    let blip = |t0: f64| (t0..t0 + 0.25).contains(&t);
    if blip(2.0) || blip(3.0) || blip(4.0) || (5.0..7.5).contains(&t) {
        1.0
    } else {
        0.0
    }
}

/// Length of the drive scene, s.
pub const DRIVE_S: f64 = 11.0;

/// The speed nearest `rpm` at which one engine cycle (two turns) is a whole number of samples
/// at `sample_rate`: 120·fs/n rpm for the nearest whole n.
pub fn seamless_rpm(rpm: f64, sample_rate: f64) -> f64 {
    120.0 * sample_rate / cycle_samples(rpm, sample_rate) as f64
}

/// Samples in one engine cycle at a [`seamless_rpm`].
pub fn cycle_samples(rpm: f64, sample_rate: f64) -> usize {
    (120.0 * sample_rate / rpm.max(1.0)).round().max(1.0) as usize
}

/// One loop: `def` held at `rpm` (made seamless) and `throttle`, settled, then `cycles` whole
/// cycles. Returns (the speed used, the samples).
pub fn render_loop(
    def: &EngineDef,
    rpm: f64,
    throttle: f64,
    cycles: usize,
    sample_rate: f64,
    quality: Quality,
) -> (f64, Vec<f32>) {
    let rpm = seamless_rpm(rpm, sample_rate);
    let mut sim =
        Simulation::with_quality(def.clone(), VehicleDef::default(), sample_rate, 1, quality);
    sim.controls = Controls {
        throttle,
        hold_rpm: Some(rpm),
        ..Controls::default()
    };
    // settle: the pipes' waves and the mean flows reach their steady state
    let mut warm = vec![0.0f32; sample_rate as usize];
    sim.render(&mut warm);
    let mut out = vec![0.0f32; cycles * cycle_samples(rpm, sample_rate)];
    sim.render(&mut out);
    (rpm, out)
}

/// The drive scene at `quality`.
pub fn render_drive(def: &EngineDef, sample_rate: f64, quality: Quality) -> Vec<f32> {
    let mut sim = running(def, sample_rate, quality);
    let block = 240;
    let n = (DRIVE_S * sample_rate) as usize / block;
    let mut out = Vec::with_capacity(n * block);
    let mut buf = vec![0.0f32; block];
    for b in 0..n {
        sim.controls.throttle = drive_throttle(b as f64 * block as f64 / sample_rate);
        sim.render(&mut buf);
        out.extend_from_slice(&buf);
    }
    out
}

/// Full throttle from idle to redline in `seconds`, the speed ramped steadily (a dyno pull you
/// can hear).
pub fn render_sweep(def: &EngineDef, sample_rate: f64, quality: Quality, seconds: f64) -> Vec<f32> {
    let mut sim = running(def, sample_rate, quality);
    let block = 240;
    let n = (seconds * sample_rate) as usize / block;
    let (lo, hi) = (def.idle_rpm.max(800.0), def.redline_rpm);
    let mut out = Vec::with_capacity(n * block);
    let mut buf = vec![0.0f32; block];
    for b in 0..n {
        let f = b as f64 / n.max(1) as f64;
        sim.controls = Controls {
            throttle: 1.0,
            hold_rpm: Some(lo + (hi - lo) * f),
            ..Controls::default()
        };
        sim.render(&mut buf);
        out.extend_from_slice(&buf);
    }
    out
}

/// Seconds an engine idles before a scene is recorded: past its start-up transient (pipes and
/// cylinders filling from rest) and the start flare its idle control brings back down (measured:
/// 3 s still left the V10 15 dB over its idle; at 8 s every preset's idle is flat from the start).
const PREROLL_S: f64 = 8.0;

/// A simulation of `def` already running: started and idled for [`PREROLL_S`], unrecorded.
fn running(def: &EngineDef, sample_rate: f64, quality: Quality) -> Simulation {
    let mut sim =
        Simulation::with_quality(def.clone(), VehicleDef::default(), sample_rate, 1, quality);
    let mut idle = vec![0.0f32; (PREROLL_S * sample_rate) as usize];
    sim.render(&mut idle);
    sim
}

/// Output sample rate of the renders, Hz.
pub const RATE: f64 = 48_000.0;
/// Each loop lasts at least this long, s (whole cycles).
const LOOP_S: f64 = 2.0;

/// The loops a game needs: every 500 rpm from idle to redline, each closed, half and full
/// throttle.
pub fn loop_grid(def: &EngineDef) -> Vec<(f64, f64)> {
    let first = (def.idle_rpm / 500.0).ceil() as usize;
    let last = (def.redline_rpm / 500.0).floor() as usize;
    (first..=last)
        .flat_map(|k| [0.0, 0.5, 1.0].map(|load| (k as f64 * 500.0, load)))
        .collect()
}

/// Render `scene` of `def` at `quality` into `out`; the files written.
pub fn render_scene(
    def: &EngineDef,
    scene: Scene,
    out: &std::path::Path,
    quality: Quality,
) -> Result<Vec<std::path::PathBuf>, String> {
    let wav = |path: &std::path::Path, x: &[f32]| {
        crate::headless::write_wav(path, x, RATE as u32)
            .map_err(|e| format!("{}: {e}", path.display()))
    };
    match scene {
        Scene::Drive => {
            wav(out, &render_drive(def, RATE, quality))?;
            Ok(vec![out.to_path_buf()])
        }
        Scene::Sweep => {
            wav(out, &render_sweep(def, RATE, quality, 12.0))?;
            Ok(vec![out.to_path_buf()])
        }
        Scene::Loops => {
            std::fs::create_dir_all(out).map_err(|e| format!("{}: {e}", out.display()))?;
            let mut files = Vec::new();
            let mut index = String::from("rpm,load,samples,file\n");
            for (target, load) in loop_grid(def) {
                let cycles = (LOOP_S * target / 120.0).ceil().max(1.0) as usize;
                let (rpm, x) = render_loop(def, target, load, cycles, RATE, quality);
                let name = format!("rpm{target:05.0}_load{:03.0}.wav", load * 100.0);
                let path = out.join(&name);
                wav(&path, &x)?;
                index.push_str(&format!("{rpm:.4},{load},{},{name}\n", x.len()));
                files.push(path);
            }
            let csv = out.join("loops.csv");
            std::fs::write(&csv, index).map_err(|e| format!("{}: {e}", csv.display()))?;
            files.push(csv);
            Ok(files)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_drive_starts_on_an_engine_already_idling() {
        // the recording begins on a running engine: its first moments sound like the idle
        // after them — no start-up transient, no start flare (V10 before: 2.4× the idle's rms)
        let x = render_drive(&EngineDef::lfa_v10(), RATE, Quality::Medium);
        let rms = |a: f64, b: f64| {
            let s = &x[(a * RATE) as usize..(b * RATE) as usize];
            (s.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / s.len() as f64).sqrt()
        };
        let (start, idle) = (rms(0.0, 0.3), rms(1.0, 1.9));
        assert!(start < 1.3 * idle, "start {start} vs idle {idle}");
    }

    #[test]
    fn scenes_parse_from_the_command_line() {
        assert_eq!("drive".parse(), Ok(Scene::Drive));
        assert_eq!("Sweep".parse(), Ok(Scene::Sweep));
        assert_eq!("loops".parse(), Ok(Scene::Loops));
        assert!("race".parse::<Scene>().is_err());
    }

    #[test]
    fn the_drive_scene_blips_pulls_and_lifts() {
        assert_eq!(drive_throttle(1.0), 0.0);
        assert_eq!(drive_throttle(2.1), 1.0);
        assert_eq!(drive_throttle(2.3), 0.0);
        assert_eq!(drive_throttle(4.2), 1.0);
        assert_eq!(drive_throttle(6.0), 1.0);
        assert_eq!(drive_throttle(8.0), 0.0);
    }

    #[test]
    fn a_seamless_speed_fits_whole_samples_in_a_cycle() {
        // 3000 rpm: a cycle is 0.04 s = 1920 samples at 48 kHz, already whole
        assert_eq!(seamless_rpm(3000.0, 48_000.0), 3000.0);
        assert_eq!(cycle_samples(3000.0, 48_000.0), 1920);
        // 4321 rpm: 120·48000/4321 = 1333.0 samples → 1333 → 4321.08 rpm
        let r = seamless_rpm(4321.0, 48_000.0);
        assert!((r - 4_321.080_27).abs() < 1e-3, "{r}");
        assert_eq!(cycle_samples(r, 48_000.0), 1333);
    }

    #[test]
    fn a_loop_joins_without_a_seam() {
        // a whole number of cycles, settled: the end runs into the start as the next cycle
        // would. Cycle-to-cycle variation stays (it is physics), so no two cycles are equal
        // sample by sample; but the sound repeats at one cycle far better than a little off it,
        // and the file holds exactly that many cycles
        let d = EngineDef::inline4();
        let (rpm, x) = render_loop(&d, 3000.0, 0.5, 4, 48_000.0, Quality::Medium);
        let n = cycle_samples(rpm, 48_000.0);
        assert_eq!(x.len(), 4 * n);
        let mismatch = |lag: usize| {
            let e: f32 = x.windows(lag + 1).map(|w| (w[lag] - w[0]).powi(2)).sum();
            (e / (x.len() - lag) as f32).sqrt()
        };
        let at_cycle = mismatch(n);
        // measured 2026-10-23: 0.0100 at one cycle, 0.0167 sixteen samples off; held 1% fast
        // the least mismatch moves away from the cycle
        for off in [n - 16, n - 8, n + 8, n + 16] {
            assert!(
                at_cycle < mismatch(off),
                "{at_cycle} vs {} at {off}",
                mismatch(off)
            );
        }
        for off in [n - 16, n + 16] {
            assert!(
                at_cycle < 0.7 * mismatch(off),
                "{at_cycle} vs {}",
                mismatch(off)
            );
        }
        // and the level has not drifted between the first cycle and the last
        let rms = |c: &[f32]| (c.iter().map(|v| v * v).sum::<f32>() / c.len() as f32).sqrt();
        let (first, last) = (rms(&x[..n]), rms(&x[3 * n..]));
        assert!(
            (last / first - 1.0).abs() < 0.1,
            "first {first} last {last}"
        );
    }

    #[test]
    fn the_loops_cover_idle_to_redline_at_three_loads() {
        let mut d = EngineDef::inline4();
        d.idle_rpm = 800.0;
        d.redline_rpm = 2000.0;
        let g = loop_grid(&d);
        assert_eq!(
            g,
            [
                (1000.0, 0.0),
                (1000.0, 0.5),
                (1000.0, 1.0),
                (1500.0, 0.0),
                (1500.0, 0.5),
                (1500.0, 1.0),
                (2000.0, 0.0),
                (2000.0, 0.5),
                (2000.0, 1.0),
            ]
        );
    }

    #[test]
    fn a_scene_lands_in_its_file() {
        let dir = std::env::temp_dir().join(format!("engine-sim-offline-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("sweep.wav");
        let files = render_scene(&EngineDef::single(), Scene::Sweep, &path, Quality::Low).unwrap();
        assert_eq!(files, std::slice::from_ref(&path));
        let wav = hound::WavReader::open(&path).unwrap();
        assert_eq!(wav.spec().sample_rate, 48_000);
        assert_eq!(wav.duration(), 12 * 48_000);
        // a folder that cannot be made is an error, not a panic
        let blocked = path.join("loops");
        assert!(render_scene(&EngineDef::single(), Scene::Loops, &blocked, Quality::Low).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
