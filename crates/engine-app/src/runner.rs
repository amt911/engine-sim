//! The simulation thread. Audio is the master clock: the thread renders blocks while the
//! ring buffer holds less than `TARGET_LATENCY_S` of audio, otherwise it sleeps briefly.
//! Without an audio device it paces itself against the wall clock.

use crate::quality::{Governor, Quality, QualityChoice, pick};
use crate::sim::{DriveInput, Simulation, Snapshot};
use engine_audio::device::{self, Output};
use engine_core::drivetrain::VehicleDef;
use engine_core::engine::{Controls, EngineDef};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const BLOCK: usize = 128;
const TARGET_LATENCY_S: f64 = 0.04;
const RING_SECONDS: f64 = 0.25;
const FALLBACK_RATE: u32 = 48_000;
const SNAPSHOT_EVERY: Duration = Duration::from_millis(15);

pub enum Command {
    Controls(Controls),
    Engine(Box<EngineDef>),
    Drive(DriveInput),
    Vehicle(Box<VehicleDef>),
    Quality(QualityChoice),
    Quit,
}

pub struct SimHandle {
    tx: Sender<Command>,
    snapshot: Arc<Mutex<Snapshot>>,
    thread: Option<JoinHandle<()>>,
    _output: Option<Output>,
    /// Sample rate in use, or why there is no sound.
    pub audio: Result<u32, String>,
}

impl SimHandle {
    pub fn spawn(
        def: EngineDef,
        vehicle: VehicleDef,
        quality: QualityChoice,
        repaint: Option<eframe::egui::Context>,
    ) -> Self {
        let (output, producer, audio) = match device::open(RING_SECONDS) {
            Ok((out, prod)) => {
                let sr = out.sample_rate;
                (Some(out), Some(prod), Ok(sr))
            }
            Err(e) => (None, None, Err(e.to_string())),
        };
        let rate = *audio.as_ref().unwrap_or(&FALLBACK_RATE);
        // Auto starts at Medium while the benchmark runs
        let start = match quality {
            QualityChoice::Fixed(q) => q,
            QualityChoice::Auto => Quality::Medium,
        };
        let sim = Simulation::with_quality(def, vehicle, rate as f64, 1, start);
        let dropouts = output.as_ref().map(|o| o.dropouts.clone());
        let snapshot = Arc::new(Mutex::new(sim.snapshot()));
        let (tx, rx) = channel();
        let shared = snapshot.clone();
        let thread = std::thread::Builder::new()
            .name("sim".into())
            .spawn(move || run(sim, quality, rx, producer, shared, repaint, dropouts))
            .expect("spawn sim thread");
        Self {
            tx,
            snapshot,
            thread: Some(thread),
            _output: output,
            audio,
        }
    }

    pub fn send(&self, c: Command) {
        let _ = self.tx.send(c);
    }

    pub fn snapshot(&self) -> Snapshot {
        self.snapshot
            .lock()
            .map(|s| s.clone())
            .unwrap_or_else(|p| p.into_inner().clone())
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        let _ = self.tx.send(Command::Quit);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Seconds of audio a benchmark renders, after a short warm-up.
const BENCH_S: f64 = 0.15;

/// Real-time factor of `def` at `quality` on this machine: seconds of audio per second of
/// work, at full throttle held at 80 % of the redline (the dearest steady point).
pub fn measure(def: &EngineDef, rate: f64, quality: Quality) -> f64 {
    let mut sim = Simulation::with_quality(def.clone(), VehicleDef::default(), rate, 1, quality);
    sim.controls = Controls {
        throttle: 1.0,
        hold_rpm: Some(0.8 * def.redline_rpm),
        ..Controls::default()
    };
    let mut warm = vec![0.0f32; (0.05 * rate) as usize];
    sim.render(&mut warm);
    let mut out = vec![0.0f32; (BENCH_S * rate) as usize];
    let t = Instant::now();
    sim.render(&mut out);
    BENCH_S / t.elapsed().as_secs_f64().max(1e-9)
}

/// Benchmark `def` on its own thread (the sim keeps playing meanwhile); the level arrives on
/// the returned channel.
fn benchmark(def: EngineDef, rate: f64) -> Receiver<Quality> {
    let (tx, rx) = channel();
    std::thread::spawn(move || {
        let _ = tx.send(pick(|q| measure(&def, rate, q)));
    });
    rx
}

fn run(
    mut sim: Simulation,
    mut choice: QualityChoice,
    rx: Receiver<Command>,
    mut producer: Option<rtrb::Producer<f32>>,
    shared: Arc<Mutex<Snapshot>>,
    repaint: Option<eframe::egui::Context>,
    dropouts: Option<Arc<std::sync::atomic::AtomicU64>>,
) {
    let rate = sim.sample_rate();
    let target = (rate * TARGET_LATENCY_S) as usize;
    let mut block = [0.0f32; BLOCK];
    let start = Instant::now();
    let mut produced: u64 = 0;
    let mut last_snap = Instant::now();
    let mut governor = Governor::new();
    let mut bench = (choice == QualityChoice::Auto).then(|| benchmark(sim.def().clone(), rate));
    loop {
        for cmd in rx.try_iter() {
            match cmd {
                Command::Controls(c) => sim.controls = c,
                Command::Engine(d) => {
                    sim.replace_engine(*d);
                    if choice == QualityChoice::Auto {
                        bench = Some(benchmark(sim.def().clone(), rate));
                    }
                }
                Command::Drive(d) => sim.drive = d,
                Command::Vehicle(v) => sim.replace_vehicle(*v),
                Command::Quality(q) => {
                    choice = q;
                    governor = Governor::new();
                    match q {
                        QualityChoice::Fixed(level) => {
                            bench = None;
                            sim.set_quality(level);
                        }
                        QualityChoice::Auto => bench = Some(benchmark(sim.def().clone(), rate)),
                    }
                }
                Command::Quit => return,
            }
        }
        if let Some(level) = bench.as_ref().and_then(|b| b.try_recv().ok()) {
            bench = None;
            governor = Governor::new();
            sim.set_quality(level);
        }
        let due = match &producer {
            Some(p) => p.buffer().capacity() - p.slots() < target,
            None => (start.elapsed().as_secs_f64() * rate) as u64 > produced,
        };
        if due {
            let t = Instant::now();
            sim.render(&mut block);
            let work = t.elapsed().as_secs_f64();
            // Auto: a level that keeps falling behind steps down (never while benchmarking)
            if choice == QualityChoice::Auto
                && bench.is_none()
                && governor.overloaded(BLOCK as f64 / rate, work)
                && let Some(lower) = sim.quality().lower()
            {
                sim.set_quality(lower);
            }
            produced += BLOCK as u64;
            if let Some(p) = &mut producer {
                for s in block {
                    let _ = p.push(s);
                }
            }
        } else {
            std::thread::sleep(Duration::from_micros(500));
        }
        if last_snap.elapsed() >= SNAPSHOT_EVERY {
            last_snap = Instant::now();
            if let Ok(mut s) = shared.try_lock() {
                *s = sim.snapshot();
                s.dropouts = dropouts
                    .as_ref()
                    .map_or(0, |d| d.load(std::sync::atomic::Ordering::Relaxed));
            }
            if let Some(ctx) = &repaint {
                ctx.request_repaint();
            }
        }
    }
}
