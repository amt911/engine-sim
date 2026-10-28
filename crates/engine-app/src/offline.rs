//! Offline rendering for files and games: no real-time limit, the finest simulation, scripted
//! scenes. `drive` (idle, three blips, a pull to the limiter, lift-off), `sweep` (full
//! throttle from idle to redline), and `loops`: a grid of held speeds × loads, each a whole
//! number of engine cycles long — at a speed nudged so a cycle is a whole number of samples —
//! so a game can loop it without a seam; `held:RPM[:LOAD[:SECONDS]]`, one such held speed (to
//! compare with a recording of a real engine held there); `gears`, a full-throttle pull through
//! every gear of the engine's own car, from a clutch launch to the top gear's limit.

use crate::quality::Quality;
use crate::sim::Simulation;
use engine_core::drivetrain::VehicleDef;
use engine_core::engine::{Controls, EngineDef};

/// A scripted scene.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Scene {
    Drive,
    Sweep,
    Loops,
    /// Full throttle through every gear of the engine's car ([`render_gears`]).
    Gears,
    /// One speed held at a load (0–1) for about `seconds` (whole cycles, seamless).
    Held {
        rpm: f64,
        load: f64,
        seconds: f64,
    },
}

impl std::str::FromStr for Scene {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        let lower = s.to_ascii_lowercase();
        if let Some(args) = lower.strip_prefix("held:") {
            return held(args)
                .ok_or_else(|| format!("bad held scene '{s}': held:RPM[:LOAD 0-1[:SECONDS > 0]]"));
        }
        match lower.as_str() {
            "drive" => Ok(Scene::Drive),
            "sweep" => Ok(Scene::Sweep),
            "loops" => Ok(Scene::Loops),
            "gears" => Ok(Scene::Gears),
            _ => Err(format!(
                "unknown scene '{s}' (drive, sweep, loops, gears, held:RPM[:LOAD[:SECONDS]])"
            )),
        }
    }
}

/// `RPM[:LOAD[:SECONDS]]` of a held scene; None unless the speed is positive, the load within
/// 0–1 and the length positive.
fn held(args: &str) -> Option<Scene> {
    let v: Vec<f64> = args
        .split(':')
        .map(|a| a.parse().ok())
        .collect::<Option<_>>()?;
    let (rpm, load, seconds) = match v[..] {
        [r] => (r, 1.0, 4.0),
        [r, l] => (r, l, 4.0),
        [r, l, t] => (r, l, t),
        _ => return None,
    };
    (rpm > 0.0 && (0.0..=1.0).contains(&load) && seconds > 0.0).then_some(Scene::Held {
        rpm,
        load,
        seconds,
    })
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
    sim.set_limiter(false);
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

/// Idle in neutral before the launch, s.
const PULL_IDLE_S: f64 = 1.0;
/// The launch: the clutch comes up at this share of the redline.
const LAUNCH_SHARE: f64 = 0.45;
/// Upshift at this share of the redline (just under the ECU's limiter).
const SHIFT_SHARE: f64 = 0.97;
/// A shift: throttle off and clutch down this long, s (a quick manual shift).
const SHIFT_S: f64 = 0.3;
/// Lift-off at the end, s.
const LIFT_S: f64 = 2.0;
/// A gear whose speed stops climbing (the car's drag holds it under the shift point) is left
/// once it has gained less than `PLATEAU_GAIN` in this long, s: the next gear, or the lift-off
/// in top gear — as a driver would.
pub const PLATEAU_S: f64 = 2.0;
const PLATEAU_GAIN: f64 = 0.005;

/// The driver of a full-throttle pull through the gears: what the pedals and the lever do, step
/// by step, from the engine speed.
#[derive(Debug, Clone)]
pub struct GearPull {
    gears: usize,
    redline: f64,
    phase: PullPhase,
    gear: usize,
    t: f64,
    /// The pull's best speed so far in this gear and when it was last bettered, s.
    best: f64,
    since_best: f64,
    /// Engine speed when the clutch came up after the last shift, rpm.
    takeup: f64,
    /// The gearbox's published gear-change time, s (0: a driver's manual shift).
    shift_time: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum PullPhase {
    Idle,
    /// Clutch down in neutral before a gear goes in (selecting one with the clutch still
    /// engaged stalls the engine).
    Clutch,
    Launch,
    Pull,
    Shift,
    Lift,
    Done,
}

impl GearPull {
    /// `shift_time_s`: [`engine_core::drivetrain::VehicleDef::shift_time_s`].
    pub fn new(gears: usize, redline_rpm: f64, shift_time_s: f64) -> Self {
        Self {
            gears: gears.max(1),
            redline: redline_rpm,
            phase: PullPhase::Idle,
            gear: 0,
            t: 0.0,
            best: 0.0,
            since_best: 0.0,
            takeup: 0.0,
            shift_time: shift_time_s.max(0.0),
        }
    }

    /// How long a shift keeps the clutch open: half an automated gearbox's gear-change time
    /// (the other half closes it), else a quick manual shift's [`SHIFT_S`].
    fn open_s(&self) -> f64 {
        if self.shift_time > 0.0 {
            0.5 * self.shift_time
        } else {
            SHIFT_S
        }
    }

    /// This step's (throttle, pedals and lever) at engine speed `rpm`, with the clutch `locked`
    /// (engine and wheels turning together) or still slipping, `dt` s after the last.
    pub fn next(&mut self, rpm: f64, locked: bool, dt: f64) -> (f64, crate::sim::DriveInput) {
        let to = |p: &mut Self, phase| {
            p.phase = phase;
            p.t = 0.0;
            p.best = 0.0;
            p.since_best = 0.0;
        };
        if self.phase == PullPhase::Pull && locked {
            if rpm > self.best * (1.0 + PLATEAU_GAIN) {
                self.best = rpm;
                self.since_best = 0.0;
            } else {
                self.since_best += dt;
            }
        }
        match self.phase {
            PullPhase::Idle if self.t >= PULL_IDLE_S - 1e-9 => to(self, PullPhase::Clutch),
            PullPhase::Clutch if self.t >= SHIFT_S - 1e-9 => {
                self.gear = 1;
                to(self, PullPhase::Launch);
            }
            PullPhase::Launch if rpm >= LAUNCH_SHARE * self.redline => to(self, PullPhase::Pull),
            PullPhase::Pull
                if locked
                    && (rpm >= SHIFT_SHARE * self.redline
                        || self.since_best >= PLATEAU_S - 1e-9) =>
            {
                let next = if self.gear < self.gears {
                    PullPhase::Shift
                } else {
                    PullPhase::Lift
                };
                to(self, next);
            }
            PullPhase::Shift if self.t >= self.open_s() - 1e-9 => {
                self.gear += 1;
                self.takeup = rpm;
                to(self, PullPhase::Pull);
            }
            PullPhase::Lift if self.t >= LIFT_S - 1e-9 => to(self, PullPhase::Done),
            _ => {}
        }
        let (throttle, clutch_pedal) = match self.phase {
            PullPhase::Idle | PullPhase::Lift | PullPhase::Done => (0.0, false),
            PullPhase::Launch => (1.0, true),
            // while the clutch slips the driver holds the revs: at a launch near the launch revs
            // (on the throttle up to 15 % over them, off by 45 %); after a shift where the
            // clutch came up (half throttle there, full 10 % under, off 10 % over), as the
            // clutch still short of the engine's torque would let it flare to the limiter.
            // An automated gearbox holds the engine's torque low until its clutch locks
            // (Glielmo et al. 2006, Fig. 10). Locked, flat out
            PullPhase::Pull if !locked && self.gear == 1 => {
                let launch = LAUNCH_SHARE * self.redline;
                (
                    ((1.15 * launch - rpm) / (0.3 * launch)).clamp(0.0, 1.0),
                    false,
                )
            }
            PullPhase::Pull if !locked && self.shift_time > 0.0 => (0.0, false),
            PullPhase::Pull if !locked => (
                (0.5 + (self.takeup - rpm) / (0.2 * self.takeup)).clamp(0.0, 1.0),
                false,
            ),
            PullPhase::Pull => (1.0, false),
            PullPhase::Shift | PullPhase::Clutch => (0.0, true),
        };
        self.t += dt;
        (
            throttle,
            crate::sim::DriveInput {
                gear: self.gear,
                clutch_pedal,
                brake: 0.0,
                // after the launch an automated gearbox closes its clutch in half its shift
                clutch_engage_s: if self.gear > 1 {
                    0.5 * self.shift_time
                } else {
                    0.0
                },
            },
        )
    }

    /// The pull has lifted off and ended.
    pub fn done(&self) -> bool {
        self.phase == PullPhase::Done
    }
}

/// The gears scene at `quality`: the engine in its own car, idling, then pulled at full throttle
/// through every gear ([`GearPull`]). Returns the samples and the highest gear reached.
pub fn render_gears(def: &EngineDef, sample_rate: f64, quality: Quality) -> (Vec<f32>, usize) {
    let mut sim =
        Simulation::with_quality(def.clone(), def.vehicle.clone(), sample_rate, 1, quality);
    sim.set_limiter(false);
    let mut idle = vec![0.0f32; (PREROLL_S * sample_rate) as usize];
    sim.render(&mut idle);
    let block = 240;
    let dt = block as f64 / sample_rate;
    let mut pull = GearPull::new(
        def.vehicle.gear_ratios.len(),
        def.redline_rpm,
        def.vehicle.shift_time_s,
    );
    let (mut out, mut buf, mut top) = (Vec::new(), vec![0.0f32; block], 0);
    // a bound for a car that cannot pull at all (each gear's limit plus the top gear's cap)
    let cap = ((def.vehicle.gear_ratios.len() as f64 * 30.0 + 10.0) / dt) as usize;
    for _ in 0..cap {
        let (throttle, drive) = pull.next(sim.snapshot().rpm, sim.clutch_locked(), dt);
        if pull.done() {
            break;
        }
        sim.controls.throttle = throttle;
        sim.drive = drive;
        top = top.max(drive.gear);
        sim.render(&mut buf);
        out.extend_from_slice(&buf);
    }
    (out, top)
}

/// Seconds an engine idles before a scene is recorded: past its start-up transient (pipes and
/// cylinders filling from rest) and the start flare its idle control brings back down (measured:
/// 3 s still left the V10 15 dB over its idle; at 8 s every preset's idle is flat from the start).
const PREROLL_S: f64 = 8.0;

/// A simulation of `def` already running: started and idled for [`PREROLL_S`], unrecorded.
fn running(def: &EngineDef, sample_rate: f64, quality: Quality) -> Simulation {
    let mut sim =
        Simulation::with_quality(def.clone(), VehicleDef::default(), sample_rate, 1, quality);
    sim.set_limiter(false);
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

/// Scale every buffer by one factor so the loudest sample of them all sits at `ceiling`; buffers
/// already under it are left alone. An offline render's output stage does not limit (it would
/// squeeze the loud passages a live device needs squeezed): the scene is scaled once instead, and
/// its loud and quiet moments keep the ratio the physics gave them.
pub fn normalize<B: AsMut<[f32]>>(bufs: &mut [B], ceiling: f32) {
    let peak = bufs
        .iter_mut()
        .flat_map(|b| b.as_mut().iter().map(|x| x.abs()))
        .fold(0.0f32, f32::max);
    if peak > ceiling {
        let k = ceiling / peak;
        for b in bufs.iter_mut() {
            b.as_mut().iter_mut().for_each(|x| *x *= k);
        }
    }
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
            let mut x = [render_drive(def, RATE, quality)];
            normalize(&mut x, engine_audio::dsp::LIMITER_THRESHOLD as f32);
            wav(out, &x[0])?;
            Ok(vec![out.to_path_buf()])
        }
        Scene::Gears => {
            let mut x = [render_gears(def, RATE, quality).0];
            normalize(&mut x, engine_audio::dsp::LIMITER_THRESHOLD as f32);
            wav(out, &x[0])?;
            Ok(vec![out.to_path_buf()])
        }
        Scene::Sweep => {
            let mut x = [render_sweep(def, RATE, quality, 12.0)];
            normalize(&mut x, engine_audio::dsp::LIMITER_THRESHOLD as f32);
            wav(out, &x[0])?;
            Ok(vec![out.to_path_buf()])
        }
        Scene::Held { rpm, load, seconds } => {
            let cycles = (seconds * rpm / 120.0).ceil().max(1.0) as usize;
            let mut x = [render_loop(def, rpm, load, cycles, RATE, quality).1];
            normalize(&mut x, engine_audio::dsp::LIMITER_THRESHOLD as f32);
            wav(out, &x[0])?;
            Ok(vec![out.to_path_buf()])
        }
        Scene::Loops => {
            std::fs::create_dir_all(out).map_err(|e| format!("{}: {e}", out.display()))?;
            let mut files = Vec::new();
            let mut index = String::from("rpm,load,samples,file\n");
            // every loop first, then one scale for all: their levels stay comparable
            let grid = loop_grid(def);
            let mut loops: Vec<(f64, Vec<f32>)> = grid
                .iter()
                .map(|&(target, load)| {
                    let cycles = (LOOP_S * target / 120.0).ceil().max(1.0) as usize;
                    render_loop(def, target, load, cycles, RATE, quality)
                })
                .collect();
            let mut bufs: Vec<Vec<f32>> =
                loops.iter_mut().map(|(_, x)| std::mem::take(x)).collect();
            normalize(&mut bufs, engine_audio::dsp::LIMITER_THRESHOLD as f32);
            for ((&(target, load), (rpm, _)), x) in grid.iter().zip(&loops).zip(&bufs) {
                let name = format!("rpm{target:05.0}_load{:03.0}.wav", load * 100.0);
                let path = out.join(&name);
                wav(&path, x)?;
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
    fn a_scene_is_scaled_once_so_its_loudest_moment_just_fits() {
        // one factor for every buffer: their levels keep their ratios, the loudest peak lands on
        // the ceiling, and a scene already under it is left alone
        let mut bufs = vec![vec![0.5f32, -2.0, 1.0], vec![0.25f32, 0.1]];
        normalize(&mut bufs, 0.8);
        let close = |a: &[f32], b: &[f32]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6);
        assert!(close(&bufs[0], &[0.2, -0.8, 0.4]), "{bufs:?}");
        assert!(close(&bufs[1], &[0.1, 0.04]), "{bufs:?}");
        let mut quiet = vec![vec![0.3f32, -0.5]];
        normalize(&mut quiet, 0.8);
        assert_eq!(quiet[0], vec![0.3f32, -0.5]);
    }

    #[test]
    fn scenes_parse_from_the_command_line() {
        assert_eq!("drive".parse(), Ok(Scene::Drive));
        assert_eq!("Sweep".parse(), Ok(Scene::Sweep));
        assert_eq!("loops".parse(), Ok(Scene::Loops));
        assert_eq!("gears".parse(), Ok(Scene::Gears));
        assert!("race".parse::<Scene>().is_err());
        // one held speed: held:RPM[:LOAD[:SECONDS]], full load and 4 s unless given
        assert_eq!(
            "held:7400".parse(),
            Ok(Scene::Held {
                rpm: 7400.0,
                load: 1.0,
                seconds: 4.0
            })
        );
        assert_eq!(
            "Held:3000:0.5:8".parse(),
            Ok(Scene::Held {
                rpm: 3000.0,
                load: 0.5,
                seconds: 8.0
            })
        );
        for bad in [
            "held",
            "held:",
            "held:x",
            "held:7400:2",
            "held:7400:1:0",
            "held:-5",
            "held:1:1:1:1",
        ] {
            assert!(bad.parse::<Scene>().is_err(), "{bad}");
        }
    }

    #[test]
    fn a_held_scene_is_whole_cycles_at_its_speed() {
        // 3000 rpm is 25 cycles/s: a cycle is 1920 samples at 48 kHz, so 0.5 s is 12.5 cycles,
        // rounded up to 13 whole cycles = 24 960 samples
        let dir = std::env::temp_dir().join(format!("engine-sim-held-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("held.wav");
        let scene = Scene::Held {
            rpm: 3000.0,
            load: 1.0,
            seconds: 0.5,
        };
        let files = render_scene(&EngineDef::single(), scene, &path, Quality::Low).unwrap();
        assert_eq!(files, std::slice::from_ref(&path));
        let wav = hound::WavReader::open(&path).unwrap();
        assert_eq!(wav.duration(), 24_960);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn an_automated_gearbox_shifts_in_its_published_time_on_a_closed_throttle() {
        // a 0.2 s gearbox (the LFA's ASG): the clutch opens with the torque cut for 0.1 s, then
        // closes in 0.1 s with the engine's torque held low until it locks (Glielmo et al.
        // 2006, Fig. 10), and only then does the throttle come back
        let dt = 0.01;
        let mut p = GearPull::new(5, 6000.0, 0.2);
        let mut step = |rpm: f64, locked: bool| p.next(rpm, locked, dt);
        for _ in 0..131 {
            step(900.0, true);
        }
        let (_, d) = step(1500.0, false);
        // the launch is the driver's: a driver's clutch release
        assert_eq!(d.clutch_engage_s, 0.0);
        step(2700.0, false);
        let (thr, d) = step(5820.0, true);
        assert_eq!((thr, d.clutch_pedal), (0.0, true));
        for _ in 0..9 {
            assert!(step(5500.0, false).1.clutch_pedal);
        }
        // 0.1 s on: second gear in, the clutch closing in 0.1 s, the throttle still shut
        let (thr, d) = step(5400.0, false);
        assert_eq!((d.gear, d.clutch_pedal, d.clutch_engage_s), (2, false, 0.1));
        assert_eq!(thr, 0.0);
        assert_eq!(step(5000.0, false).0, 0.0);
        // locked: flat out
        assert_eq!(step(4000.0, true).0, 1.0);
    }

    #[test]
    fn after_a_shift_the_driver_holds_the_revs_while_the_clutch_takes_up() {
        // an upshift lands the engine above the next gear's speed: the clutch slips until they
        // meet, and the driver is already back on the throttle (the launch's governor, which
        // holds the revs near 45 % of the redline while the clutch slips from rest, is for the
        // launch only; after a shift it shut the throttle until the clutch locked, then opened
        // it at once: the engine cut and refired as the clutch took up)
        let dt = 0.01;
        let mut p = GearPull::new(5, 6000.0, 0.0);
        let mut step = |rpm: f64, locked: bool| p.next(rpm, locked, dt);
        for _ in 0..131 {
            step(900.0, true);
        }
        step(1500.0, false);
        step(2700.0, false);
        step(5820.0, true);
        for _ in 0..30 {
            step(5000.0, false);
        }
        // second gear in at 5000 rpm, the clutch still slipping: on the throttle, but holding
        // the revs where the clutch came up — more as the clutch pulls them down, none if they
        // flare (with the clutch still short of the engine's torque, full throttle ran the
        // engine to its limiter)
        let (thr, d) = step(5000.0, false);
        assert_eq!((d.gear, d.clutch_pedal), (2, false));
        assert!(thr > 0.2 && thr < 1.0, "{thr}");
        let (lower, _) = step(4500.0, false);
        assert!(lower > thr, "{lower} vs {thr}");
        assert_eq!(step(5600.0, false).0, 0.0);
        assert_eq!(step(4000.0, false).0, 1.0);
        // locked: flat out
        assert_eq!(step(4000.0, true).0, 1.0);
    }

    #[test]
    fn a_gear_pull_launches_shifts_at_the_limit_and_lifts() {
        // 5 gears, redline 6000: launch at 45 % of it with the clutch, full throttle; at 97 %
        // (5820 rpm) a 0.3 s shift (throttle off, clutch down, next gear); in top gear it pulls
        // to the shift point, then lifts off for 2 s and the scene ends
        let dt = 0.01;
        let mut p = GearPull::new(5, 6000.0, 0.0);
        let step = |p: &mut GearPull, rpm: f64| p.next(rpm, true, dt);
        // idle in neutral first
        let (thr, d) = step(&mut p, 900.0);
        assert_eq!((thr, d.gear, d.clutch_pedal), (0.0, 0, false));
        for _ in 0..100 {
            step(&mut p, 900.0);
        }
        // the clutch goes down in neutral first (selecting a gear while it is still engaged
        // stalls the engine), then first gear, revving for the launch
        let (thr, d) = step(&mut p, 900.0);
        assert_eq!((thr, d.gear, d.clutch_pedal), (0.0, 0, true));
        for _ in 0..30 {
            step(&mut p, 900.0);
        }
        let (thr, d) = step(&mut p, 1500.0);
        assert_eq!((thr, d.gear, d.clutch_pedal), (1.0, 1, true));
        // at launch revs the clutch comes up
        let (thr, d) = step(&mut p, 2700.0);
        assert_eq!((thr, d.gear, d.clutch_pedal), (1.0, 1, false));
        // pulling until the shift point
        assert_eq!(step(&mut p, 5000.0).1.gear, 1);
        let (thr, d) = step(&mut p, 5820.0);
        assert_eq!((thr, d.clutch_pedal), (0.0, true));
        for _ in 0..30 {
            step(&mut p, 5000.0);
        }
        let (thr, d) = step(&mut p, 4000.0);
        assert_eq!((thr, d.gear, d.clutch_pedal), (1.0, 2, false));
        // through to top gear
        for g in 2..5 {
            step(&mut p, 5820.0);
            for _ in 0..31 {
                step(&mut p, 4000.0);
            }
            assert_eq!(step(&mut p, 4000.0).1.gear, g + 1);
        }
        assert!(!p.done());
        // top gear at the limit: lift off, then done after 2 s
        let (thr, d) = step(&mut p, 5820.0);
        assert_eq!((thr, d.gear), (0.0, 5));
        for _ in 0..199 {
            step(&mut p, 5000.0);
        }
        assert!(!p.done());
        step(&mut p, 5000.0);
        assert!(p.done());
    }

    #[test]
    fn a_slipping_clutch_holds_the_launch_revs_and_never_shifts() {
        // the clutch takes its time to bite: while it slips the driver holds the engine near
        // the launch revs instead of letting it fly to the limiter, and does not shift
        let mut p = GearPull::new(5, 6000.0, 0.0);
        for _ in 0..140 {
            p.next(900.0, false, 0.01);
        }
        let (thr, d) = p.next(2700.0, false, 0.01);
        assert_eq!((d.gear, d.clutch_pedal), (1, false));
        assert!(thr > 0.0);
        // flared to the shift point with the clutch still slipping: throttle off, no shift
        let (thr, d) = p.next(5900.0, false, 0.01);
        assert_eq!((thr, d.gear, d.clutch_pedal), (0.0, 1, false));
        // slipping for long does not count as a plateau
        for _ in 0..400 {
            p.next(3000.0, false, 0.01);
        }
        assert_eq!(p.next(3000.0, false, 0.01).1.gear, 1);
        // locked: full throttle
        assert_eq!(p.next(3000.0, true, 0.01).0, 1.0);
    }

    #[test]
    fn a_gear_pull_that_stops_climbing_moves_on() {
        // drag can hold a gear under the limit: once its speed stops climbing for PLATEAU_S the
        // pull moves on (here the only gear, so it lifts off and ends)
        let mut p = GearPull::new(1, 6000.0, 0.0);
        for _ in 0..200 {
            p.next(900.0, true, 0.01);
        }
        p.next(3000.0, true, 0.01);
        let mut t = 0.0;
        while !p.done() && t < 120.0 {
            p.next(4000.0, true, 0.01);
            t += 0.01;
        }
        assert!(p.done(), "never ended");
        assert!(t < PLATEAU_S + LIFT_S + 0.1, "{t} s");
    }

    #[test]
    fn the_gears_scene_drives_the_car_through_its_gears() {
        // the real simulation: the car pulls through every gear to the top one (the LFA: its
        // engine stalled when first gear went in with the clutch still engaged)
        let def = EngineDef::lfa_v10();
        let (x, gears) = render_gears(&def, RATE, Quality::Low);
        assert!(x.iter().all(|v| v.is_finite()));
        assert_eq!(gears, def.vehicle.gear_ratios.len(), "top gear reached");
        // a real pull: well over 15 s through six gears (it once flew through them in 3 s at
        // walking pace, the clutch slipping)
        assert!(
            x.len() > (15.0 * RATE) as usize,
            "{} s",
            x.len() as f64 / RATE
        );
        assert!(x.len() > (5.0 * RATE) as usize);
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
        // the pulses' band: a 16-sample running mean (first null at 3 kHz) — the valve jets'
        // broadband hiss above it is turbulence, as aperiodic as a real engine's
        let low: Vec<f32> = x
            .windows(16)
            .map(|w| w.iter().sum::<f32>() / 16.0)
            .collect();
        let mismatch = |lag: usize| {
            let e: f32 = low.windows(lag + 1).map(|w| (w[lag] - w[0]).powi(2)).sum();
            (e / (low.len() - lag) as f32).sqrt()
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
