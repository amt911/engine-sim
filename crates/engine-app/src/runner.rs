//! The simulation thread. Audio is the master clock: the thread renders blocks while the
//! ring buffer holds less than `TARGET_LATENCY_S` of audio, otherwise it sleeps briefly.
//! Without an audio device it paces itself against the wall clock.

use crate::sim::{Simulation, Snapshot};
use engine_audio::device::{self, Output};
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
    Engine(EngineDef),
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
    pub fn spawn(def: EngineDef, repaint: Option<eframe::egui::Context>) -> Self {
        let (output, producer, audio) = match device::open(RING_SECONDS) {
            Ok((out, prod)) => {
                let sr = out.sample_rate;
                (Some(out), Some(prod), Ok(sr))
            }
            Err(e) => (None, None, Err(e.to_string())),
        };
        let rate = *audio.as_ref().unwrap_or(&FALLBACK_RATE);
        let sim = Simulation::new(def, rate as f64, 1);
        let snapshot = Arc::new(Mutex::new(sim.snapshot()));
        let (tx, rx) = channel();
        let shared = snapshot.clone();
        let thread = std::thread::Builder::new()
            .name("sim".into())
            .spawn(move || run(sim, rx, producer, shared, repaint))
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

fn run(
    mut sim: Simulation,
    rx: Receiver<Command>,
    mut producer: Option<rtrb::Producer<f32>>,
    shared: Arc<Mutex<Snapshot>>,
    repaint: Option<eframe::egui::Context>,
) {
    let rate = sim.sample_rate();
    let target = (rate * TARGET_LATENCY_S) as usize;
    let mut block = [0.0f32; BLOCK];
    let start = Instant::now();
    let mut produced: u64 = 0;
    let mut last_snap = Instant::now();
    loop {
        for cmd in rx.try_iter() {
            match cmd {
                Command::Controls(c) => sim.controls = c,
                Command::Engine(d) => sim.replace_engine(d),
                Command::Quit => return,
            }
        }
        let due = match &producer {
            Some(p) => p.buffer().capacity() - p.slots() < target,
            None => (start.elapsed().as_secs_f64() * rate) as u64 > produced,
        };
        if due {
            sim.render(&mut block);
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
            }
            if let Some(ctx) = &repaint {
                ctx.request_repaint();
            }
        }
    }
}
