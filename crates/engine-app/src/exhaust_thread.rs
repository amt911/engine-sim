//! The exhaust net on its own thread: the cylinders of step n+1 run while it advances step n
//! (`engine_core::engine::Engine::detach_exhaust` explains why that is exact). One persistent
//! worker; the net is handed to it for a render block and comes back at the end. Per step the
//! two threads trade whole messages through lock-free rings (one commit each: a value at a
//! time would bounce a cache line between the cores per value) — no lock, no allocation.

use engine_core::exhaust::{ExhaustLink, ExhaustNet};
use rtrb::{Consumer, Producer, RingBuffer};
use std::sync::mpsc;
use std::thread::JoinHandle;

/// Job header values.
const STEP: f64 = 0.0;
const STEP_MEAN: f64 = 1.0;
const STOP: f64 = 2.0;
/// Room in each ring, values (several steps of the largest engine).
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

/// A job: header, valve flows (n), tail Mach (m), mean flows per valve (n) and per tail (m).
fn job_len(n: usize, m: usize) -> usize {
    1 + 2 * n + 2 * m
}

/// The worker's end of a block: the net and the rings.
struct Block {
    net: ExhaustNet,
    jobs: Consumer<f64>,
    arriving: Producer<f64>,
    tail_out: Producer<f64>,
}

/// A worker thread that runs exhaust nets handed to it, one block at a time.
pub struct ExhaustThread {
    to_worker: Option<mpsc::Sender<Block>>,
    from_worker: mpsc::Receiver<ExhaustNet>,
    handle: Option<JoinHandle<()>>,
}

impl ExhaustThread {
    /// Worth it only with cores to spare: the UI, the audio callback and the sim thread already
    /// take three.
    pub fn worthwhile() -> bool {
        std::thread::available_parallelism().is_ok_and(|n| n.get() >= 4)
    }

    pub fn spawn() -> std::io::Result<Self> {
        let (to_worker, blocks) = mpsc::channel::<Block>();
        let (back, from_worker) = mpsc::channel::<ExhaustNet>();
        let handle = std::thread::Builder::new()
            .name("exhaust".into())
            .spawn(move || {
                while let Ok(mut b) = blocks.recv() {
                    let (n, m) = (b.net.valves(), b.net.last_tails().len());
                    let mut job = vec![0.0; job_len(n, m)];
                    let mut next = vec![0.0; n];
                    loop {
                        recv(&mut b.jobs, &mut job);
                        if job[0] == STOP {
                            break;
                        }
                        let (flows, rest) = job[1..].split_at(n);
                        let (mach, rest) = rest.split_at(m);
                        let (cyl, branch) = rest.split_at(n);
                        let mean = (job[0] == STEP_MEAN).then_some((cyl, branch));
                        b.net.prepare(mach, mean);
                        b.net.arriving_next(&mut next);
                        send(&mut b.arriving, &next);
                        b.net.step(flows);
                        send(&mut b.tail_out, b.net.last_tails());
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

    /// Hand `net` to the worker for a block; the link the engine talks to it through.
    pub fn link(&self, net: ExhaustNet) -> Box<dyn ExhaustLink> {
        let (n, m) = (net.valves(), net.last_tails().len());
        let (jobs_p, jobs_c) = RingBuffer::new(RING);
        let (arr_p, arr_c) = RingBuffer::new(RING);
        let (tail_p, tail_c) = RingBuffer::new(RING);
        if let Some(tx) = &self.to_worker {
            let _ = tx.send(Block {
                net,
                jobs: jobs_c,
                arriving: arr_p,
                tail_out: tail_p,
            });
        }
        Box::new(ThreadLink {
            job: vec![0.0; job_len(n, m)],
            jobs: jobs_p,
            arriving: arr_c,
            tails: tail_c,
        })
    }

    /// End the block (after the engine flushed): the net back from the worker.
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

/// The engine's end: each step's job out, the next waves and the tails back.
struct ThreadLink {
    job: Vec<f64>,
    jobs: Producer<f64>,
    arriving: Consumer<f64>,
    tails: Consumer<f64>,
}

impl ExhaustLink for ThreadLink {
    fn submit(&mut self, flows: &[f64], mach: &[f64], mean: Option<(&[f64], &[f64])>) {
        let (n, m) = (flows.len(), mach.len());
        self.job[0] = if mean.is_some() { STEP_MEAN } else { STEP };
        self.job[1..1 + n].copy_from_slice(flows);
        self.job[1 + n..1 + n + m].copy_from_slice(mach);
        if let Some((cyl, branch)) = mean {
            self.job[1 + n + m..1 + 2 * n + m].copy_from_slice(cyl);
            self.job[1 + 2 * n + m..].copy_from_slice(branch);
        }
        send(&mut self.jobs, &self.job);
    }
    fn arriving_next(&mut self, out: &mut [f64]) {
        recv(&mut self.arriving, out);
    }
    fn tails(&mut self, out: &mut [f64]) {
        recv(&mut self.tails, out);
    }
    fn stop(&mut self) {
        self.job.fill(0.0);
        self.job[0] = STOP;
        send(&mut self.jobs, &self.job);
    }
}

use engine_core::fv::Cons;
use engine_core::intake_fv::{MAX_SUBCYCLES, RunnerLink, RunnerResult, RunnerSet};

/// A job for the runners: k, dt, the throttle area, each runner's valve flow (mass, energy).
fn runner_job_len(r: usize) -> usize {
    3 + 2 * r
}

/// A result: the largest stable step, the fluxes (room for the most substeps), each runner's
/// last cell (3 values) and middle pressure.
fn runner_result_len(r: usize) -> usize {
    1 + MAX_SUBCYCLES * r + 3 * r + r
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
        std::thread::available_parallelism().is_ok_and(|n| n.get() >= 6)
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
                        for (i, a) in acc.iter_mut().enumerate() {
                            *a = (job[3 + 2 * i], job[4 + 2 * i]);
                        }
                        b.set.fv_step(k, dt, area, &acc, &mut res);
                        msg[0] = res.min_dt;
                        msg[1..1 + res.flux.len()].copy_from_slice(&res.flux);
                        let at = 1 + MAX_SUBCYCLES * r;
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
    fn submit(&mut self, k: usize, dt: f64, throttle_area: f64, acc: &[(f64, f64)]) {
        self.k = k;
        self.job[0] = k as f64;
        self.job[1] = dt;
        self.job[2] = throttle_area;
        for (i, &(m, e)) in acc.iter().enumerate() {
            self.job[3 + 2 * i] = m;
            self.job[4 + 2 * i] = e;
        }
        send(&mut self.jobs, &self.job);
    }
    fn collect(&mut self, out: &mut RunnerResult) {
        recv(&mut self.results, &mut self.msg);
        let (r, k) = (self.r, self.k);
        out.min_dt = self.msg[0];
        out.flux.clear();
        out.flux.extend_from_slice(&self.msg[1..1 + r * k]);
        let at = 1 + MAX_SUBCYCLES * r;
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
