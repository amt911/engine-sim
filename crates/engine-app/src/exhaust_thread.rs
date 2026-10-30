//! The exhaust net on its own thread, a block of steps at a time. The engine reads the waves
//! arriving at its valves from its own copy of the primaries' delay lines
//! (`engine_core::exhaust::ArrivalProbe`): what arrives in the next `block` steps is already
//! in them, so the net may run a block or two behind on another core and give the very same
//! samples (`Engine::detach_exhaust`). One persistent worker; per block the two threads trade
//! one message each way through bounded channels — a thousand hand-offs a second, not one per
//! physics step, so a shared core (a laptop) costs a block's wait, not a step's.

use engine_core::exhaust::{BlockIn, BlockOut, ExhaustLink, ExhaustNet};
use rtrb::{Consumer, Producer, RingBuffer};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread::JoinHandle;

/// Room in each of the runners' rings, values (several steps of the largest engine).
const RING: usize = 4096;

/// Spin a while (a step takes microseconds), then yield the core.
struct Backoff(u32);

impl Backoff {
    fn wait(&mut self) {
        self.0 += 1;
        if self.0 < 4000 {
            std::hint::spin_loop();
        } else {
            std::thread::yield_now();
        }
    }
}

/// Send `values` as one message.
fn send(p: &mut Producer<f64>, values: &[f64]) {
    let mut b = Backoff(0);
    loop {
        if let Ok(chunk) = p.write_chunk_uninit(values.len()) {
            chunk.fill_from_iter(values.iter().copied());
            return;
        }
        b.wait();
    }
}

/// Receive one message of `out.len()` values.
fn recv(c: &mut Consumer<f64>, out: &mut [f64]) {
    let mut b = Backoff(0);
    loop {
        if let Ok(chunk) = c.read_chunk(out.len()) {
            let (a, rest) = chunk.as_slices();
            out[..a.len()].copy_from_slice(a);
            out[a.len()..].copy_from_slice(rest);
            chunk.commit_all();
            return;
        }
        b.wait();
    }
}

/// Whether the sim may hand its exhaust (and intake runners) to their own threads
/// (`--threads`). The runners still trade messages every step (off by default for them, see
/// `RunnerThread`); the exhaust trades a block at a time.
pub static STEP_THREADS: AtomicBool = AtomicBool::new(false);

fn asked() -> bool {
    STEP_THREADS.load(Ordering::Relaxed)
}

fn cores() -> usize {
    std::thread::available_parallelism().map_or(1, |n| n.get())
}

/// What the engine sends the worker.
enum Job {
    Run(BlockIn),
    Stop,
}

/// The worker's end of a hand-off: the net, its jobs and where its results go.
struct Block {
    net: ExhaustNet,
    jobs: mpsc::Receiver<Job>,
    results: mpsc::SyncSender<BlockOut>,
}

/// A worker thread that runs exhaust nets handed to it, a block of steps at a time.
pub struct ExhaustThread {
    to_worker: Option<mpsc::Sender<Block>>,
    from_worker: mpsc::Receiver<ExhaustNet>,
    handle: Option<JoinHandle<()>>,
}

impl ExhaustThread {
    /// With a core to spare beyond the UI's, the audio callback's and the sim's.
    pub fn worthwhile() -> bool {
        Self::worth(cores())
    }

    fn worth(cores: usize) -> bool {
        cores >= 4
    }

    pub fn spawn() -> std::io::Result<Self> {
        let (to_worker, blocks) = mpsc::channel::<Block>();
        let (back, from_worker) = mpsc::channel::<ExhaustNet>();
        let handle = std::thread::Builder::new()
            .name("exhaust".into())
            .spawn(move || {
                while let Ok(mut b) = blocks.recv() {
                    while let Ok(Job::Run(input)) = b.jobs.recv() {
                        let mut out = BlockOut::default();
                        b.net.run_block(&input, &mut out);
                        if b.results.send(out).is_err() {
                            break;
                        }
                    }
                    if back.send(b.net).is_err() {
                        return;
                    }
                }
            })?;
        Ok(Self {
            to_worker: Some(to_worker),
            from_worker,
            handle: Some(handle),
        })
    }

    /// Hand `net` to the worker, in blocks of `block` steps; the link the engine talks to it
    /// through.
    pub fn link(&self, net: ExhaustNet, block: usize) -> Box<dyn ExhaustLink> {
        // two blocks in flight at most, each way (the engine never runs further ahead)
        let (jobs_tx, jobs) = mpsc::sync_channel(2);
        let (results, results_rx) = mpsc::sync_channel(2);
        if let Some(tx) = &self.to_worker {
            let _ = tx.send(Block { net, jobs, results });
        }
        Box::new(ThreadLink {
            block,
            jobs: jobs_tx,
            results: results_rx,
        })
    }

    /// End the hand-off (after the engine flushed): the net back from the worker.
    pub fn back(&self, link: &mut dyn ExhaustLink) -> Option<ExhaustNet> {
        link.stop();
        self.from_worker.recv().ok()
    }
}

impl Drop for ExhaustThread {
    fn drop(&mut self) {
        // closing the channel ends the worker's loop
        self.to_worker = None;
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The engine's end: each block's inputs out, its results back in order.
struct ThreadLink {
    block: usize,
    jobs: mpsc::SyncSender<Job>,
    results: mpsc::Receiver<BlockOut>,
}

impl ExhaustLink for ThreadLink {
    fn block(&self) -> usize {
        self.block
    }
    fn submit(&mut self, input: &mut BlockIn) {
        let _ = self.jobs.send(Job::Run(std::mem::take(input)));
    }
    fn collect(&mut self, out: &mut BlockOut) {
        if let Ok(o) = self.results.recv() {
            *out = o;
        }
    }
    fn stop(&mut self) {
        let _ = self.jobs.send(Job::Stop);
    }
}

use engine_core::fv::Cons;
use engine_core::intake_fv::{MAX_SUBCYCLES, RunnerLink, RunnerResult, RunnerSet, Upstream};

/// A job for the runners: k, dt, the throttle area, the upstream state (p, ρ), each runner's
/// valve flow (mass, energy).
fn runner_job_len(r: usize) -> usize {
    5 + 2 * r
}

/// A result: the largest stable step, the mass fluxes then the energy fluxes (room for the
/// most substeps), each runner's last cell (3 values) and middle pressure.
fn runner_result_len(r: usize) -> usize {
    1 + 2 * MAX_SUBCYCLES * r + 3 * r + r
}

struct RunnerBlock {
    set: RunnerSet,
    jobs: Consumer<f64>,
    results: Producer<f64>,
}

/// A worker thread that steps the runners handed to it, one block at a time.
pub struct RunnerThread {
    to_worker: Option<mpsc::Sender<RunnerBlock>>,
    from_worker: mpsc::Receiver<RunnerSet>,
    handle: Option<JoinHandle<()>>,
}

impl RunnerThread {
    /// A second worker: worth it with the exhaust's thread, the sim's, the UI's and the audio
    /// callback's cores already taken.
    pub fn worthwhile() -> bool {
        Self::worth(asked(), cores())
    }

    fn worth(asked: bool, cores: usize) -> bool {
        asked && cores >= 6
    }

    pub fn spawn() -> std::io::Result<Self> {
        let (to_worker, blocks) = mpsc::channel::<RunnerBlock>();
        let (back, from_worker) = mpsc::channel::<RunnerSet>();
        let handle = std::thread::Builder::new()
            .name("runners".into())
            .spawn(move || {
                while let Ok(mut b) = blocks.recv() {
                    let r = b.set.pipes.len();
                    let mut job = vec![0.0; runner_job_len(r)];
                    let mut acc = vec![(0.0, 0.0); r];
                    let mut res = RunnerResult::default();
                    let mut msg = vec![0.0; runner_result_len(r)];
                    loop {
                        recv(&mut b.jobs, &mut job);
                        if job[0] < 0.0 {
                            break;
                        }
                        let (k, dt, area) = (job[0] as usize, job[1], job[2]);
                        let up = Upstream {
                            p: job[3],
                            rho: job[4],
                        };
                        for (i, a) in acc.iter_mut().enumerate() {
                            *a = (job[5 + 2 * i], job[6 + 2 * i]);
                        }
                        b.set.fv_step(k, dt, area, up, &acc, &mut res);
                        msg[0] = res.min_dt;
                        msg[1..1 + res.flux.len()].copy_from_slice(&res.flux);
                        let at_e = 1 + MAX_SUBCYCLES * r;
                        msg[at_e..at_e + res.eflux.len()].copy_from_slice(&res.eflux);
                        let at = 1 + 2 * MAX_SUBCYCLES * r;
                        for (i, c) in res.last.iter().enumerate() {
                            msg[at + 3 * i] = c.rho;
                            msg[at + 3 * i + 1] = c.mom;
                            msg[at + 3 * i + 2] = c.e;
                        }
                        msg[at + 3 * r..].copy_from_slice(&res.mid_p);
                        send(&mut b.results, &msg);
                    }
                    if back.send(b.set).is_err() {
                        return;
                    }
                }
            })?;
        Ok(Self {
            to_worker: Some(to_worker),
            from_worker,
            handle: Some(handle),
        })
    }

    /// Hand `set` to the worker for a block; the link the intake talks to it through.
    pub fn link(&self, set: RunnerSet) -> Box<dyn RunnerLink> {
        let r = set.pipes.len();
        let (jobs_p, jobs_c) = RingBuffer::new(RING);
        let (res_p, res_c) = RingBuffer::new(RING.max(4 * runner_result_len(r)));
        if let Some(tx) = &self.to_worker {
            let _ = tx.send(RunnerBlock {
                set,
                jobs: jobs_c,
                results: res_p,
            });
        }
        Box::new(RunnerThreadLink {
            r,
            k: 0,
            job: vec![0.0; runner_job_len(r)],
            msg: vec![0.0; runner_result_len(r)],
            jobs: jobs_p,
            results: res_c,
        })
    }

    /// End the block: the runners back from the worker.
    pub fn back(&self, link: &mut dyn RunnerLink) -> Option<RunnerSet> {
        link.stop();
        self.from_worker.recv().ok()
    }
}

impl Drop for RunnerThread {
    fn drop(&mut self) {
        self.to_worker = None;
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

struct RunnerThreadLink {
    r: usize,
    k: usize,
    job: Vec<f64>,
    msg: Vec<f64>,
    jobs: Producer<f64>,
    results: Consumer<f64>,
}

impl RunnerLink for RunnerThreadLink {
    fn submit(&mut self, k: usize, dt: f64, throttle_area: f64, up: Upstream, acc: &[(f64, f64)]) {
        self.k = k;
        self.job[0] = k as f64;
        self.job[1] = dt;
        self.job[2] = throttle_area;
        self.job[3] = up.p;
        self.job[4] = up.rho;
        for (i, &(m, e)) in acc.iter().enumerate() {
            self.job[5 + 2 * i] = m;
            self.job[6 + 2 * i] = e;
        }
        send(&mut self.jobs, &self.job);
    }
    fn collect(&mut self, out: &mut RunnerResult) {
        recv(&mut self.results, &mut self.msg);
        let (r, k) = (self.r, self.k);
        out.min_dt = self.msg[0];
        out.flux.clear();
        out.flux.extend_from_slice(&self.msg[1..1 + r * k]);
        let at_e = 1 + MAX_SUBCYCLES * r;
        out.eflux.clear();
        out.eflux.extend_from_slice(&self.msg[at_e..at_e + r * k]);
        let at = 1 + 2 * MAX_SUBCYCLES * r;
        out.last.clear();
        out.last.extend((0..r).map(|i| Cons {
            rho: self.msg[at + 3 * i],
            mom: self.msg[at + 3 * i + 1],
            e: self.msg[at + 3 * i + 2],
        }));
        out.mid_p.clear();
        out.mid_p.extend_from_slice(&self.msg[at + 3 * r..]);
    }
    fn stop(&mut self) {
        self.job.fill(0.0);
        self.job[0] = -1.0;
        send(&mut self.jobs, &self.job);
    }
}

#[cfg(test)]
mod opt_in_tests {
    use super::*;

    #[test]
    fn the_exhaust_thread_needs_a_spare_core_and_the_runners_an_ask() {
        // the exhaust trades a block at a time, so it is worth a spare core by default; the
        // runners still trade every step (measured 2026-10-26: on six shared cores the per-step
        // hand-off made the LFA 2.5× slower than one thread), so only `--threads` turns them on
        assert!(ExhaustThread::worth(4));
        assert!(!ExhaustThread::worth(3));
        assert!(!RunnerThread::worth(false, 24));
        assert!(RunnerThread::worth(true, 6));
        assert!(!RunnerThread::worth(true, 5));
    }
}
