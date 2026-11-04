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
    // edits are paced (a dragged slider sends one per frame); a benchmark begun before the last
    // edit measured another engine: its answer is dropped
    let mut edits = EditGate::new();
    let mut bench_stale = false;
    loop {
        for cmd in rx.try_iter() {
            match cmd {
                Command::Controls(c) => sim.controls = c,
                Command::Engine(d) => edits.offer(*d),
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
                        QualityChoice::Auto if bench.is_none() => {
                            bench = Some(benchmark(sim.def().clone(), rate));
                        }
                        QualityChoice::Auto => {}
                    }
                }
                Command::Quit => return,
            }
        }
        let now = Instant::now();
        if let Some(d) = edits.take(now) {
            sim.replace_engine(d);
            bench_stale = bench.is_some();
        }
        if choice == QualityChoice::Auto && edits.bench_due(now, bench.is_some()) {
            bench = Some(benchmark(sim.def().clone(), rate));
        }
        if let Some(level) = bench.as_ref().and_then(|b| b.try_recv().ok()) {
            bench = None;
            if !std::mem::take(&mut bench_stale) {
                governor = Governor::new();
                sim.set_quality(level);
            }
        }
        // what the audio buffer holds, in samples (none without a device)
        let filled = producer.as_ref().map(|p| p.buffer().capacity() - p.slots());
        let due = match filled {
            Some(f) => f < target,
            None => (start.elapsed().as_secs_f64() * rate) as u64 > produced,
        };
        if due {
            let t = Instant::now();
            sim.render(&mut block);
            let work = t.elapsed().as_secs_f64();
            // Auto: a level that keeps falling behind, or lets the buffer run nearly dry, steps
            // down (never while benchmarking)
            let fill = filled.map_or(1.0, |f| f as f64 / target as f64);
            if choice == QualityChoice::Auto
                && bench.is_none()
                && (governor.overloaded(BLOCK as f64 / rate, work) | governor.starved(fill))
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

/// Least time between two rebuilds while edits keep coming (a slider dragged sends one per frame).
const EDIT_EVERY: Duration = Duration::from_millis(250);
/// How long the engine must stay unedited before the quality benchmark runs on it.
const BENCH_AFTER: Duration = Duration::from_secs(1);

/// Paces the engine edits that reach the sim: a rebuild at most every [`EDIT_EVERY`], always ending
/// on the latest definition, and the quality benchmark only once the edits stop.
struct EditGate<T> {
    pending: Option<T>,
    last_applied: Option<Instant>,
    bench_wanted: bool,
}

impl<T> EditGate<T> {
    fn new() -> Self {
        Self {
            pending: None,
            last_applied: None,
            bench_wanted: false,
        }
    }

    /// A new definition: it replaces any not yet applied.
    fn offer(&mut self, def: T) {
        self.pending = Some(def);
    }

    /// The definition to rebuild with now, if one is waiting and the last rebuild is old enough.
    fn take(&mut self, now: Instant) -> Option<T> {
        if self
            .last_applied
            .is_some_and(|t| now.saturating_duration_since(t) < EDIT_EVERY)
        {
            return None;
        }
        let def = self.pending.take()?;
        self.last_applied = Some(now);
        self.bench_wanted = true;
        Some(def)
    }

    /// Whether to start the benchmark now: wanted after an edit, none running, nothing waiting,
    /// and the engine still for [`BENCH_AFTER`].
    fn bench_due(&mut self, now: Instant, benching: bool) -> bool {
        let still = self
            .last_applied
            .is_some_and(|t| now.saturating_duration_since(t) >= BENCH_AFTER);
        let due = self.bench_wanted && self.pending.is_none() && !benching && still;
        if due {
            self.bench_wanted = false;
        }
        due
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(t0: Instant, n: u64) -> Instant {
        t0 + Duration::from_millis(n)
    }

    #[test]
    fn a_drag_rebuilds_at_most_every_250_ms_and_ends_on_the_last_value() {
        let t0 = Instant::now();
        let mut g = EditGate::new();
        let mut applied = Vec::new();
        // a slider dragged for a second: a new value every 16 ms frame
        for i in 0..63u64 {
            g.offer(i);
            applied.extend(g.take(ms(t0, 16 * i)));
        }
        applied.extend(g.take(ms(t0, 1300)));
        assert!(applied.len() <= 6, "{applied:?}");
        assert_eq!(applied.first(), Some(&0), "the first edit is heard at once");
        assert_eq!(applied.last(), Some(&62), "it ends on the last value");
        assert_eq!(g.take(ms(t0, 2000)), None, "nothing left");
    }

    #[test]
    fn the_benchmark_waits_until_the_engine_is_still() {
        let t0 = Instant::now();
        let mut g = EditGate::new();
        assert!(!g.bench_due(t0, false), "no edit, no benchmark");
        g.offer(1);
        assert_eq!(g.take(t0), Some(1));
        g.offer(2);
        assert!(
            !g.bench_due(ms(t0, 1500), false),
            "an edit waits: not still"
        );
        assert_eq!(g.take(ms(t0, 1500)), Some(2));
        assert!(!g.bench_due(ms(t0, 2499), false));
        assert!(!g.bench_due(ms(t0, 2500), true), "one benchmark at a time");
        assert!(g.bench_due(ms(t0, 2500), false));
        assert!(!g.bench_due(ms(t0, 4000), false), "once");
    }
}
