//! Linear acoustic waveguides for the exhaust (J. O. Smith, *Physical Audio Signal
//! Processing*: digital waveguides, Kelly–Lochbaum junctions). Pressures are gauge (Pa).
//!
//! A pipe is two delay lines: `fwd` carries waves away from its near end, `bwd` carries
//! waves back to it. Junctions and ends exchange waves between pipes every step.

/// Fractional delay line (linear interpolation; Laakso et al. 1996).
#[derive(Debug, Clone)]
pub struct DelayLine {
    buf: Vec<f64>,
    pos: usize,
    /// Whole steps of the delay and the fraction past them (the delay is fixed: worked out once;
    /// the ring wraps by comparison — a modulo per read was most of a lined muffler's cost).
    whole: usize,
    frac: f64,
    /// The delay it was built with, steps: [`DelayLine::set_scale`] scales it.
    base: f64,
    /// Each value's own arrival time, when the line is non-linear ([`DelayLine::nonlinear`]).
    arrivals: Option<Box<Arrivals>>,
}

/// The values of a non-linear line in flight: `t[j]` is when `buf[j]` arrives, in steps of the
/// line's clock `now`; `head` the newest value already arrived. A value is a cluster of the
/// samples that merged into it (see [`DelayLine::push`]): `mass` samples, `speed` the sum of
/// their speeds (relative to c) and `sent` the sum of speed × send time.
#[derive(Debug, Clone)]
struct Arrivals {
    t: Vec<f64>,
    mass: Vec<f64>,
    speed: Vec<f64>,
    sent: Vec<f64>,
    now: f64,
    head: usize,
    delay: f64,
    k: f64,
}

/// Shortest gap kept between two arrivals, steps, where a value would arrive with or before one
/// already arrived (it cannot merge with it any more).
const SHOCK_GAP: f64 = 1e-6;

/// Narrowest tent a value is read with, steps ([`DelayLine::arrived`]): a shock is captured
/// over about two steps, as finite volumes capture one over about two cells. A real exhaust
/// shock is microseconds thick, under one 5.2 µs step, so this width is the grid's, not the
/// gas's. It must not depend on how closely the values crowd (they crowd most where they merge),
/// or a shock gathering one sample more or less would make the sound jump and a pipe's echoes
/// would never settle. Its cost: the line smooths a little, −0.23 dB at 10 kHz.
const MIN_KERNEL: f64 = 2.0;
/// Widest tent, steps: where an expansion spreads the values further apart than this, they are
/// read with their spacing.
const MAX_KERNEL: f64 = 4.0;
/// No value arrives sooner than this many steps after it is sent: the read a step ahead
/// (`read_next`) must not depend on values not yet sent, nor on clusters still merging.
const MIN_LINE_STEPS: f64 = 2.0 + MERGE_FREEZE;
/// A value arriving within this many steps of the next read no longer merges ([`DelayLine::push`]).
const MERGE_FREEZE: f64 = 2.0;

impl DelayLine {
    /// `delay` in steps, ≥ 1.
    pub fn new(delay: f64) -> Self {
        Self::with_room(delay, delay)
    }
    /// [`DelayLine::new`] with room to stretch to a delay of `room` steps
    /// ([`DelayLine::set_scale`]).
    pub fn with_room(delay: f64, room: f64) -> Self {
        let delay = delay.max(1.0);
        let whole = delay.floor();
        Self {
            buf: vec![0.0; room.max(delay).ceil() as usize + 2],
            pos: 0,
            whole: whole as usize,
            frac: delay - whole,
            base: delay,
            arrivals: None,
        }
    }
    /// The delay becomes `scale` × the one it was built with (the gas warmed or cooled: the
    /// delay goes as 1/c), within the room it was built with. A linear line reads its values at
    /// the new age; a non-linear one sends new values at the new delay while those in flight
    /// keep theirs. Scaled slowly (a small fraction of a step per call), nothing overtakes.
    pub fn set_scale(&mut self, scale: f64) {
        let room = match &self.arrivals {
            // the loudest values still travel at most twice as fast and half as slow
            Some(_) => (self.buf.len() as f64 - 4.0) / 2.0,
            None => self.buf.len() as f64 - 2.0,
        };
        let delay = (self.base * scale).clamp(1.0, room);
        let delay = match &self.arrivals {
            // a non-linear line's next read must not depend on the value about to be pushed
            Some(_) => delay.max(2.0 * MIN_LINE_STEPS),
            None => delay,
        };
        self.whole = delay.floor() as usize;
        self.frac = delay - delay.floor();
        if let Some(a) = &mut self.arrivals {
            a.delay = delay;
        }
    }
    /// A line on which each value rides at its own speed, c·(1 + k·x): a simple wave's value
    /// moves at c + (γ+1)/2·u (Riemann; Msallam et al., *Acta Acustica* 86, 2000, eqs. 1–2), so
    /// with u = x/ρc, k = (γ+1)/(2ρc·c). Loud compressions arrive early and a pulse steepens.
    /// The speed stays within ½c–2c (the delay within 2× either way); a line shorter than 4
    /// steps stays linear (its next read must not depend on the value about to be pushed).
    pub fn nonlinear(delay: f64, k: f64) -> Self {
        Self::nonlinear_with_room(delay, k, delay)
    }
    /// [`DelayLine::nonlinear`] with room to stretch to a delay of `room` steps.
    pub fn nonlinear_with_room(delay: f64, k: f64, room: f64) -> Self {
        let delay = delay.max(1.0);
        if delay < 2.0 * MIN_LINE_STEPS {
            return Self::with_room(delay, room);
        }
        let cap = (2.0 * room.max(delay)).ceil() as usize + 4;
        // as if the line had always carried silence: value j was sent at step j − cap
        let sent: Vec<f64> = (0..cap).map(|j| j as f64 - cap as f64).collect();
        let t: Vec<f64> = sent.iter().map(|n| n + delay).collect();
        let head = t.iter().rposition(|&x| x <= 0.0).unwrap_or(0);
        Self {
            buf: vec![0.0; cap],
            pos: 0,
            whole: delay.floor() as usize,
            frac: delay - delay.floor(),
            base: delay,
            arrivals: Some(Box::new(Arrivals {
                t,
                mass: vec![1.0; cap],
                speed: vec![1.0; cap],
                sent,
                now: 0.0,
                head,
                delay,
                k,
            })),
        }
    }
    /// The same line, still empty, with room for `factor` × its built delay.
    fn roomier(&self, factor: f64) -> Self {
        let room = self.base * factor;
        match &self.arrivals {
            Some(a) => Self::nonlinear_with_room(self.base, a.k, room),
            None => Self::with_room(self.base, room),
        }
    }
    /// The delay when faint, steps.
    fn delay(&self) -> f64 {
        self.whole as f64 + self.frac
    }
    /// What a non-linear line holds at its clock time `time`: the mass-weighted average of the
    /// values arriving near it, each weighted by a tent as wide as its spacing to its neighbours
    /// (one step at least, [`MAX_KERNEL`] at most; a Nadaraya–Watson estimate). With evenly
    /// spaced samples that is linear interpolation; and two samples about to merge into a
    /// cluster (arriving together) weigh exactly as the cluster will, so a shock gathering one
    /// sample more or less does not make the sound jump.
    fn arrived(&self, a: &Arrivals, time: f64) -> f64 {
        let n = self.buf.len();
        let next = |j: usize| if j + 1 == n { 0 } else { j + 1 };
        let prev = |j: usize| if j == 0 { n - 1 } else { j - 1 };
        let mut j = a.head;
        while next(j) != self.pos && a.t[next(j)] <= time {
            j = next(j);
        }
        // from a few values ahead to a few behind
        let mut first = j;
        loop {
            let p = prev(first);
            if p == self.pos || a.t[p] <= time - MAX_KERNEL {
                break;
            }
            first = p;
        }
        let (mut sum, mut weight) = (0.0, 0.0);
        // slide over the values with their neighbours' times: a value's tent is half the gap
        // between its two neighbours wide (MIN_KERNEL..MAX_KERNEL)
        let (mut i, mut t_before) = (first, a.t[prev(first)]);
        let mut ti = a.t[i];
        loop {
            if ti > time + MAX_KERNEL {
                break;
            }
            let last = next(i) == self.pos;
            let t_after = if last { ti + 1.0 } else { a.t[next(i)] };
            let h = (0.5 * (t_after - t_before)).clamp(MIN_KERNEL, MAX_KERNEL);
            let d = (time - ti).abs();
            if d < h {
                let w = a.mass[i] * (1.0 - d / h);
                sum += w * self.buf[i];
                weight += w;
            }
            if last {
                break;
            }
            t_before = ti;
            i = next(i);
            ti = t_after;
        }
        if weight > 0.0 {
            return sum / weight;
        }
        // a gap wider than the kernels (a strong expansion): between its two sides
        let k = next(j);
        let w = (time - a.t[j]) / (a.t[k] - a.t[j]);
        self.buf[j] + (self.buf[k] - self.buf[j]) * w
    }
    /// The value pushed `delay` steps ago (interpolated). Call before `push` each step.
    pub fn read(&self) -> f64 {
        if let Some(a) = &self.arrivals {
            return self.arrived(a, a.now);
        }
        let n = self.buf.len();
        // index of the value pushed k steps ago (k ≤ whole + 1 < n)
        let back = |k: usize| {
            let j = self.pos + n - k;
            self.buf[if j >= n { j - n } else { j }]
        };
        back(self.whole) * (1.0 - self.frac) + back(self.whole + 1) * self.frac
    }
    /// What [`DelayLine::read`] will return after the next `push`, before that push: the
    /// values it interpolates were pushed at least a step earlier when the delay is ≥ 2 steps
    /// (`None` below that, where the next read depends on the value about to be pushed).
    pub fn read_next(&self) -> Option<f64> {
        self.read_ahead(1)
    }
    /// What [`DelayLine::read`] will return after `ahead` more pushes, before them: known
    /// while those pushes cannot arrive by then, `ahead <` [`DelayLine::whole_steps`] (`None`
    /// otherwise). The same arithmetic as `read`, so the same bits.
    pub fn read_ahead(&self, ahead: usize) -> Option<f64> {
        if ahead >= self.whole_steps() {
            return None;
        }
        if let Some(a) = &self.arrivals {
            return Some(self.arrived(a, a.now + ahead as f64));
        }
        let n = self.buf.len();
        let back = |k: usize| {
            let j = self.pos + n - k;
            self.buf[if j >= n { j - n } else { j }]
        };
        Some(
            back(self.whole - ahead) * (1.0 - self.frac) + back(self.whole + 1 - ahead) * self.frac,
        )
    }
    /// Whether its values travel at their own speed ([`DelayLine::nonlinear`]).
    pub fn is_nonlinear(&self) -> bool {
        self.arrivals.is_some()
    }
    /// A non-linear line's values travel at c + k·x from now on (a linear line stays linear).
    pub fn set_k(&mut self, k: f64) {
        if let Some(a) = &mut self.arrivals {
            a.k = k;
        }
    }
    /// Whole steps a value pushed now takes at least to arrive: the delay's, or on a non-linear
    /// line half of it (its loudest values travel at most twice as fast).
    pub fn whole_steps(&self) -> usize {
        if self.arrivals.is_some() {
            self.whole / 2
        } else {
            self.whole
        }
    }
    /// Send `x` into the line. On a non-linear line, a value that would overtake the one ahead
    /// of it before the far end (characteristics crossing: a shock) merges with it, as sticky
    /// particles do: the cluster keeps the samples' sum and moves as their centre of mass, at the
    /// mean of their speeds — the Rankine–Hugoniot speed of the simple wave's shock, which
    /// gives the entropy solution of Burgers' equation (E, Rykov & Sinai, *Commun. Math. Phys.*
    /// 177, 1996; Brenier & Grenier, *SIAM J. Numer. Anal.* 35, 1998). The crest a shock
    /// overruns is dissipated, as in a real one; the wave's content ∫p dt is kept.
    pub fn push(&mut self, x: f64) {
        let n = self.buf.len();
        let Some(a) = &mut self.arrivals else {
            self.buf[self.pos] = x;
            self.pos += 1;
            if self.pos == n {
                self.pos = 0;
            }
            return;
        };
        let prev = |j: usize| if j == 0 { n - 1 } else { j - 1 };
        let next = |j: usize| if j + 1 == n { 0 } else { j + 1 };
        let s = (1.0 + a.k * x).clamp(0.5, 2.0);
        let (mut v, mut mass, mut speed, mut sent) = (x, 1.0, s, s * a.now);
        let arrival = |mass: f64, speed: f64, sent: f64| (sent + a.delay * mass) / speed;
        let mut t = arrival(mass, speed, sent);
        // merge with the values ahead it would overtake, while they are still in flight (one
        // arriving by the next step is already read: `read_next`)
        loop {
            let j = prev(self.pos);
            if j == a.head || a.t[j] <= a.now + 1.0 + MERGE_FREEZE || a.t[j] < t {
                break;
            }
            v = (v * mass + self.buf[j] * a.mass[j]) / (mass + a.mass[j]);
            mass += a.mass[j];
            speed += a.speed[j];
            sent += a.sent[j];
            t = arrival(mass, speed, sent);
            self.pos = j;
        }
        let j = prev(self.pos);
        t = t.max(a.t[j] + SHOCK_GAP);
        self.buf[self.pos] = v;
        a.t[self.pos] = t;
        a.mass[self.pos] = mass;
        a.speed[self.pos] = speed;
        a.sent[self.pos] = sent;
        self.pos = next(self.pos);
        a.now += 1.0;
        while next(a.head) != self.pos && a.t[next(a.head)] <= a.now {
            a.head = next(a.head);
        }
    }
}

/// A uniform pipe: area, one-way delay, and a per-trip amplitude loss.
#[derive(Debug, Clone)]
pub struct Pipe {
    pub area_m2: f64,
    pub fwd: DelayLine,
    pub bwd: DelayLine,
    gain: f64,
    /// Per-trip factor without flow (`gain` is it times the flow's friction loss).
    still_gain: f64,
    length_m: f64,
    diameter_m: f64,
    /// The speed of sound it was built for, m/s, and its gas's now ([`Pipe::set_sound_speed`]).
    sound_speed: f64,
    gas_c: f64,
    /// Its gas's mean pressure, Pa ([`Pipe::set_mean_pressure`]).
    mean_pa: f64,
    dt: f64,
    /// The walls' loss above 200 Hz on each line ([`Pipe::viscothermal`]).
    walls: Option<Box<[WallLoss; 2]>>,
    /// Further segments after `fwd` and `bwd` ([`Pipe::steepening`]), each direction's in order.
    chain: Option<Box<[Vec<Stage>; 2]>>,
    /// A silencer's transmission on each direction's waves as they enter (forward, backward).
    filter: Option<Box<[crate::silencer2d::Cascade; 2]>>,
}

/// A segment of a [`Pipe::steepening`] pipe: its share of the walls' loss, then its line.
#[derive(Debug, Clone)]
struct Stage {
    wall: Option<WallLoss>,
    line: DelayLine,
}

/// Longest segment of a steepening pipe, m. A full-load blowdown pulse forms a shock within about
/// x̄ = c²/(βωu) (60 kPa rising in 1 ms in 900 K gas: 600²/(1.2·6283·254) = 0.19 m; Hamilton &
/// Blackstock, *Nonlinear Acoustics*, ch. 4), so the walls act on a front every one or two
/// formation distances. Measured on the LFA's exhaust driven periodically (2026-10-26): cycle to
/// cycle it settles within 3.9 % unsegmented, 0.34 % at 0.2 m, under 1 % at 0.4 m, at half the
/// cost of 0.2 m.
const STEEPENING_SEGMENT_M: f64 = 0.2;

/// Kirchhoff's wall loss, Np/m, of a plane wave at `f` Hz in a pipe of `radius_m` holding gas at
/// an atmosphere whose speed of sound is `c`: α = √(πfν)·(1 + (γ−1)/√Pr)/(a·c) (Kinsler, Frey,
/// Coppens & Sanders, *Fundamentals of Acoustics*, 4th ed., §9.9), ν from Sutherland's
/// viscosity ([`crate::exhaust::gas_viscosity`]).
pub(crate) fn kirchhoff_alpha(f: f64, radius_m: f64, c: f64) -> f64 {
    let gamma = crate::gas::GAMMA;
    let t = c * c / (gamma * crate::gas::R_AIR);
    let rho = crate::units::ATM_PA / (crate::gas::R_AIR * t);
    let nu = crate::exhaust::gas_viscosity(t) / rho;
    let thermal = 1.0 + (gamma - 1.0) / crate::narrow_tube::PRANDTL.sqrt();
    (std::f64::consts::PI * f * nu).sqrt() * thermal / (radius_m * c)
}

/// The frequency the pipes' flat loss ([`crate::exhaust::LOSS_PER_M`]) stands for, Hz.
const WALL_REF_HZ: f64 = 200.0;
/// Corner frequencies of the wall filter's shelves, Hz.
const WALL_POLES_HZ: [f64; 4] = [250.0, 1000.0, 4000.0, 16_000.0];

/// A pipe's wall loss above [`WALL_REF_HZ`] as four first-order shelves (bilinear), each
/// (b0, b1, a1, state), their depths fitted to Kirchhoff's √f law once, when the pipe is built.
#[derive(Debug, Clone)]
struct WallLoss([(f64, f64, f64, f64); 4]);

impl WallLoss {
    fn new(length_m: f64, diameter_m: f64, c: f64, dt: f64) -> Option<Self> {
        let radius = 0.5 * diameter_m;
        let alpha_ref = kirchhoff_alpha(WALL_REF_HZ, radius, c);
        // the extra log-magnitude to match, Np
        let target = |f: f64| -(length_m * (kirchhoff_alpha(f, radius, c) - alpha_ref)).max(0.0);
        let f_max = (0.4 / dt).min(20_000.0);
        if target(f_max) > -1e-3 {
            return None;
        }
        let freqs: Vec<f64> = (0..48)
            .map(|i| 100.0 * (f_max / 100.0).powf(i as f64 / 47.0))
            .collect();
        let sections = |depth: &[f64; 4]| -> [(f64, f64, f64); 4] {
            std::array::from_fn(|i| shelf(WALL_POLES_HZ[i], depth[i], dt))
        };
        let error = |depth: &[f64; 4]| -> f64 {
            let s = sections(depth);
            freqs
                .iter()
                .map(|&f| {
                    let got: f64 = s.iter().map(|&c| shelf_log_mag(c, f, dt)).sum();
                    (got - target(f)).powi(2)
                })
                .sum()
        };
        // coordinate descent, a golden-section search per depth (ln of the zero over the pole)
        let mut depth = [0.0; 4];
        let g = 0.5 * (5f64.sqrt() - 1.0);
        for _ in 0..8 {
            for i in 0..4 {
                let (mut lo, mut hi) = (0.0, 6.0);
                for _ in 0..30 {
                    let (a, b) = (hi - g * (hi - lo), lo + g * (hi - lo));
                    let mut da = depth;
                    da[i] = a;
                    let mut db = depth;
                    db[i] = b;
                    if error(&da) < error(&db) {
                        hi = b;
                    } else {
                        lo = a;
                    }
                }
                depth[i] = 0.5 * (lo + hi);
            }
        }
        Some(Self(sections(&depth).map(|(b0, b1, a1)| (b0, b1, a1, 0.0))))
    }
    fn process(&mut self, mut x: f64) -> f64 {
        for (b0, b1, a1, st) in &mut self.0 {
            let y = *b0 * x + *st;
            *st = *b1 * x - *a1 * y;
            x = y;
        }
        x
    }
}

/// A first-order low shelf, unity at DC, its zero `e^depth` above its pole at `pole_hz`:
/// (1 + s/ω_z)/(1 + s/ω_p) by the bilinear transform. Returns (b0, b1, a1).
pub(crate) fn shelf(pole_hz: f64, depth: f64, dt: f64) -> (f64, f64, f64) {
    let k = 2.0 / dt;
    let wp = 2.0 * std::f64::consts::PI * pole_hz;
    let wz = wp * depth.exp();
    let d = 1.0 + k / wp;
    ((1.0 + k / wz) / d, (1.0 - k / wz) / d, (1.0 - k / wp) / d)
}

/// ln |H(e^{jωΔt})| of a first-order section (b0, b1, a1) at `f` Hz.
fn shelf_log_mag((b0, b1, a1): (f64, f64, f64), f: f64, dt: f64) -> f64 {
    let w = 2.0 * std::f64::consts::PI * f * dt;
    let (c, s) = (w.cos(), w.sin());
    let num = (b0 + b1 * c).powi(2) + (b1 * s).powi(2);
    let den = (1.0 + a1 * c).powi(2) + (a1 * s).powi(2);
    0.5 * (num / den).ln()
}

impl Pipe {
    /// `loss_per_m`: fractional amplitude loss per metre travelled (wall friction, heat).
    pub fn new(length_m: f64, diameter_m: f64, sound_speed: f64, dt: f64, loss_per_m: f64) -> Self {
        let delay = length_m / sound_speed / dt;
        Self {
            area_m2: std::f64::consts::PI * 0.25 * diameter_m * diameter_m,
            fwd: DelayLine::new(delay),
            bwd: DelayLine::new(delay),
            gain: (1.0 - loss_per_m).powf(length_m),
            still_gain: (1.0 - loss_per_m).powf(length_m),
            length_m,
            diameter_m,
            sound_speed,
            gas_c: sound_speed,
            mean_pa: crate::units::ATM_PA,
            dt,
            walls: None,
            chain: None,
            filter: None,
        }
    }
    /// The waves crossing this pipe, either way, pass `transmission` (a silencer's, as
    /// [`crate::silencer2d::Silencer::transmission_eq`]).
    pub fn with_filter(mut self, transmission: crate::silencer2d::Cascade) -> Self {
        self.filter = Some(Box::new([transmission.clone(), transmission]));
        self
    }
    /// The pipe's waves steepen and its walls round them off together, as they do along a real
    /// pipe: it is cut into segments of at most [`STEEPENING_SEGMENT_M`], each its share of the
    /// walls' loss ([`Pipe::viscothermal`]) and then a non-linear line ([`Pipe::nonlinear`]) —
    /// the operator splitting of Lee & Hamilton (*JASA* 97, 1995; Hamilton & Blackstock ch. 11).
    /// A front never gets steeper than the walls let it: the shock has a thickness, where a
    /// loss only at the pipe's mouth left it none.
    pub fn steepening(self) -> Self {
        let delay = self.fwd.delay();
        let mut n = (self.length_m / STEEPENING_SEGMENT_M).ceil().max(1.0) as usize;
        while n > 1 && delay / (n as f64) < 2.0 * MIN_LINE_STEPS {
            n -= 1;
        }
        let mut pipe = self.nonlinear();
        if n == 1 {
            return pipe.viscothermal();
        }
        let gamma = crate::gas::GAMMA;
        let rho_c = gamma * crate::units::ATM_PA / pipe.sound_speed;
        let k = (gamma + 1.0) / (2.0 * rho_c * pipe.sound_speed);
        let seg = delay / n as f64;
        let wall = || {
            WallLoss::new(
                pipe.length_m / n as f64,
                pipe.diameter_m,
                pipe.sound_speed,
                pipe.dt,
            )
        };
        let stages = || -> Vec<Stage> {
            (1..n)
                .map(|_| Stage {
                    wall: wall(),
                    line: DelayLine::nonlinear(seg, k),
                })
                .collect()
        };
        let chain = Box::new([stages(), stages()]);
        pipe.fwd = DelayLine::nonlinear(seg, k);
        pipe.bwd = DelayLine::nonlinear(seg, k);
        pipe.walls = wall().zip(wall()).map(|(a, b)| Box::new([a, b]));
        pipe.chain = Some(chain);
        pipe
    }
    /// The walls take the higher frequencies faster than the flat loss: Kirchhoff's √f law above
    /// [`WALL_REF_HZ`], where the flat loss stands for it ([`kirchhoff_alpha`]). It rounds a
    /// steepening pulse off as the walls round real ones (Msallam et al. 2000, §2.3.3).
    pub fn viscothermal(mut self) -> Self {
        let make = || WallLoss::new(self.length_m, self.diameter_m, self.sound_speed, self.dt);
        self.walls = make().zip(make()).map(|(a, b)| Box::new([a, b]));
        self
    }
    /// The pipe's waves steepen as they travel ([`DelayLine::nonlinear`]): k = (γ+1)/(2ρc·c),
    /// ρc of the pipe's gas at an atmosphere.
    pub fn nonlinear(mut self) -> Self {
        let gamma = crate::gas::GAMMA;
        let rho_c = gamma * crate::units::ATM_PA / self.sound_speed;
        let k = (gamma + 1.0) / (2.0 * rho_c * self.sound_speed);
        let delay = self.fwd.delay();
        self.fwd = DelayLine::nonlinear(delay, k);
        self.bwd = DelayLine::nonlinear(delay, k);
        self
    }
    /// The pipe's gas is at a mean pressure of `p` Pa from now on (a turbine's inlet, upstream of
    /// it): its waves steepen at k = (γ+1)/(2γp) ([`Pipe::nonlinear`]) — a pressure swing is
    /// that much less of the gas's own — and a mass flow is ṁ/(ρcA), ρc = γp/c, of Mach number
    /// ([`Pipe::set_mean_flow`], at its next call).
    pub fn set_mean_pressure(&mut self, p: f64) {
        self.mean_pa = p;
        let gamma = crate::gas::GAMMA;
        let rho_c = gamma * p / self.sound_speed;
        let k = (gamma + 1.0) / (2.0 * rho_c * self.sound_speed);
        self.fwd.set_k(k);
        self.bwd.set_k(k);
        if let Some(chain) = &mut self.chain {
            for st in chain.iter_mut().flatten() {
                st.line.set_k(k);
            }
        }
    }
    /// Room in its lines for gas as slow as `c_min` m/s ([`Pipe::set_sound_speed`]). Call it
    /// last, on a pipe that has not carried a wave yet.
    pub fn with_room(mut self, c_min: f64) -> Self {
        self.make_room(c_min);
        self
    }
    /// [`Pipe::with_room`] in place.
    pub fn make_room(&mut self, c_min: f64) {
        let factor = (self.sound_speed / c_min).max(1.0);
        self.fwd = self.fwd.roomier(factor);
        self.bwd = self.bwd.roomier(factor);
        if let Some(chain) = &mut self.chain {
            for st in chain.iter_mut().flatten() {
                st.line = st.line.roomier(factor);
            }
        }
    }
    /// The gas in the pipe now carries sound at `c` m/s: its waves take length/c to cross it
    /// (within the room it was built with, [`Pipe::with_room`]). A steepening line's k =
    /// (γ+1)/(2ρc·c) = (γ+1)/(2γp) does not depend on c; the walls' √f loss keeps its build-time
    /// fit (α ∝ √ν/c moves a few % of a few dB).
    pub fn set_sound_speed(&mut self, c: f64) {
        self.gas_c = c;
        let scale = self.sound_speed / c;
        self.fwd.set_scale(scale);
        self.bwd.set_scale(scale);
        if let Some(chain) = &mut self.chain {
            for st in chain.iter_mut().flatten() {
                st.line.set_scale(scale);
            }
        }
    }
    /// The speed of sound of its gas now, m/s.
    pub fn gas_sound_speed(&self) -> f64 {
        self.gas_c
    }
    /// The pipe's length, m, and diameter, m.
    pub fn size(&self) -> (f64, f64) {
        (self.length_m, self.diameter_m)
    }
    /// Whether its waves steepen ([`Pipe::nonlinear`], [`Pipe::steepening`]).
    pub fn is_nonlinear(&self) -> bool {
        self.chain.is_some() || self.fwd.is_nonlinear()
    }
    /// Extra per-trip amplitude factor (e.g. a muffler in this pipe).
    pub fn with_extra_gain(mut self, g: f64) -> Self {
        self.gain *= g;
        self.still_gain *= g;
        self
    }

    /// The pipe's mean mass flow, kg/s, of gas at its sound speed and an atmosphere: turbulent
    /// wall friction, ½ρu|u|·f/D per unit volume, linearized about the mean velocity U, is a
    /// resistance R = ρ·f·U/D on the wave's velocity, which damps it by α = R/(2ρc) = f·M/(2D)
    /// per metre (M = U/c; Darcy f of [`crate::exhaust_fv::PIPE_FRICTION`]):
    /// the faster the gas, the more a pipe's resonances are worn down.
    pub fn set_mean_flow(&mut self, mass_flow: f64) {
        let rho_c = crate::gas::GAMMA * self.mean_pa / self.gas_c;
        let mach = (mass_flow / (rho_c * self.area_m2)).abs();
        let alpha = crate::exhaust_fv::PIPE_FRICTION * mach / (2.0 * self.diameter_m);
        self.gain = self.still_gain * (-alpha * self.length_m).exp();
    }

    /// Wave arriving at the far end this step.
    pub fn at_far(&self) -> f64 {
        self.last(0).read() * self.gain
    }
    /// Wave arriving back at the near end this step.
    pub fn at_near(&self) -> f64 {
        self.last(1).read() * self.gain
    }
    /// The line a direction ends in (`0` away from the near end, `1` back to it).
    fn last(&self, dir: usize) -> &DelayLine {
        match self.chain.as_ref().and_then(|c| c[dir].last()) {
            Some(stage) => &stage.line,
            None if dir == 0 => &self.fwd,
            None => &self.bwd,
        }
    }
    /// [`Pipe::at_near`] as it will read after the next `send` (at the current gain), if the
    /// pipe is at least two steps long.
    pub fn at_near_next(&self) -> Option<f64> {
        self.last(1).read_next().map(|x| x * self.gain)
    }
    /// [`Pipe::at_near`] as it will read after `ahead` more `send`s (at the current gain), if
    /// those sends are not among the values it interpolates.
    pub fn at_near_ahead(&self, ahead: usize) -> Option<f64> {
        self.bwd.read_ahead(ahead).map(|x| x * self.gain)
    }
    /// Send this step's outgoing waves: `from_near` into `fwd`, `from_far` into `bwd`.
    pub fn send(&mut self, from_near: f64, from_far: f64) {
        if let Some(chain) = &mut self.chain {
            // each segment takes what the one before it delivers this step (read before pushed)
            for (dir, stages) in chain.iter_mut().enumerate() {
                let first = if dir == 0 { &self.fwd } else { &self.bwd };
                for i in (0..stages.len()).rev() {
                    let x = if i == 0 {
                        first.read()
                    } else {
                        stages[i - 1].line.read()
                    };
                    let st = &mut stages[i];
                    let x = match &mut st.wall {
                        Some(w) => w.process(x),
                        None => x,
                    };
                    st.line.push(x);
                }
            }
        }
        let (near, far) = match &mut self.walls {
            Some(w) => (w[0].process(from_near), w[1].process(from_far)),
            None => (from_near, from_far),
        };
        match self.filter.as_deref_mut() {
            None => {
                self.fwd.push(near);
                self.bwd.push(far);
            }
            Some([f, b]) => {
                self.fwd.push(f.process(near));
                self.bwd.push(b.process(far));
            }
        }
    }
}

/// Pipes of different areas in series. Each change of area scatters a plane wave (the
/// two-port pressure junction below), which is how a muffler's chambers reflect sound back
/// (Munjal, *Acoustics of Ducts and Mufflers*, ch. 3) and how a stepped pipe shapes it.
#[derive(Debug, Clone)]
pub struct Duct {
    pipes: Vec<Pipe>,
    /// The gas's mean mass flow from the first pipe to the last, kg/s: what its plain area
    /// changes lose ([`area_change_resistance`]).
    mean_flow: f64,
    /// What happens at each inner node, between pipe k and k + 1.
    nodes: Vec<Node>,
}

/// A wall that breathes at a node of a duct: the packing behind a perforate, a pocket of pore
/// gas (compliance C) behind the fibres' resistance R, drawing a volume flow q = Y(s)·p with
/// Y(s) = sC/(1 + sRC) — the lumped element of the locally reacting lining (Munjal ch. 6;
/// [`crate::lining`] for R and C). Discretized by the bilinear transform; the node solves it
/// with the pipes' waves at once, so it is stable for any element.
#[derive(Debug, Clone)]
pub struct Shunt {
    /// q[n] = b0·p[n] + b1·p[n−1] + b2·p[n−2] − a1·q[n−1] − a2·q[n−2], in units of area ×
    /// pressure (×ρc), so the node adds it to the pipes' areas.
    b: [f64; 3],
    a: [f64; 2],
    p: [f64; 2],
    q: [f64; 2],
    /// 1/(A₀ + A₁ + b₀): the node's pipes never change, so the division is done once.
    inv: f64,
}

/// First-order branches k·sT/(1 + sT) driven by one signal, by the bilinear transform: their
/// summed output is `inst`·x[n] + `hist` (the past), so a node can solve for x[n] implicitly.
#[derive(Debug, Clone)]
struct Branches {
    /// per branch (b₀, a₁), y[n] = b₀(x[n] − x[n−1]) − a₁y[n−1]
    coef: Vec<(f64, f64)>,
    y: Vec<f64>,
    x_prev: f64,
    inst: f64,
}

impl Branches {
    fn new(branches: &[crate::narrow_tube::Branch], scale: f64, dt: f64) -> Self {
        let coef: Vec<(f64, f64)> = branches
            .iter()
            .map(|b| {
                let at = 2.0 * b.t_s / dt;
                (scale * b.k * at / (1.0 + at), (1.0 - at) / (1.0 + at))
            })
            .collect();
        let inst = coef.iter().map(|c| c.0).sum();
        Self {
            y: vec![0.0; coef.len()],
            coef,
            x_prev: 0.0,
            inst,
        }
    }

    /// The summed output's part already fixed by the past.
    fn hist(&self) -> f64 {
        self.coef
            .iter()
            .zip(&self.y)
            .map(|(&(b0, a1), &y)| -b0 * self.x_prev - a1 * y)
            .sum()
    }

    /// Take this step's input.
    fn advance(&mut self, x: f64) {
        for (&(b0, a1), y) in self.coef.iter().zip(&mut self.y) {
            *y = b0 * (x - self.x_prev) - a1 * *y;
        }
        self.x_prev = x;
    }
}

/// A series impedance R₀ + Σ k·sT/(1 + sT) in the flow's way (Pa·s/m³): a narrow-channel
/// brick's viscous losses ([`crate::narrow_tube::viscous`]).
#[derive(Debug, Clone)]
pub struct SeriesNet {
    r0: f64,
    rho_c: f64,
    b: Branches,
}

impl SeriesNet {
    /// In gas of impedance `rho_c`.
    pub fn new(r0: f64, branches: &[crate::narrow_tube::Branch], rho_c: f64, dt: f64) -> Self {
        Self {
            r0,
            rho_c,
            b: Branches::new(branches, 1.0, dt),
        }
    }

    /// The flow through it between pipes of impedances `z` with waves `incoming`, writing the
    /// outgoing waves: 2(p₁ − p₂) − (z₁ + z₂)Q = Z·Q (see [`Duct::send`]).
    fn node(&mut self, areas: &[f64; 2], incoming: &[f64; 2], out: &mut [f64; 2]) {
        let z = [self.rho_c / areas[0], self.rho_c / areas[1]];
        let q = (2.0 * (incoming[0] - incoming[1]) - self.b.hist())
            / (z[0] + z[1] + self.r0 + self.b.inst);
        self.b.advance(q);
        *out = [incoming[0] - z[0] * q, incoming[1] + z[1] * q];
    }
}

/// A shunt admittance Σ k·sT/(1 + sT) off the duct (m³/(Pa·s)) in gas of impedance `rho_c`: a
/// narrow-channel brick's thermal losses ([`crate::narrow_tube::thermal`]).
#[derive(Debug, Clone)]
pub struct ShuntNet {
    b: Branches,
}

impl ShuntNet {
    pub fn new(branches: &[crate::narrow_tube::Branch], rho_c: f64, dt: f64) -> Self {
        // ×ρc: the junction speaks in areas, Σ Aᵢ(2pᵢ − p) = ρc·Y·p
        Self {
            b: Branches::new(branches, rho_c, dt),
        }
    }

    fn node(&mut self, areas: &[f64; 2], incoming: &[f64; 2], out: &mut [f64; 2]) {
        let pj = (2.0 * (areas[0] * incoming[0] + areas[1] * incoming[1]) - self.b.hist())
            / (areas[0] + areas[1] + self.b.inst);
        self.b.advance(pj);
        *out = [pj - incoming[0], pj - incoming[1]];
    }
}

/// What happens where two pipes of a [`Duct`] meet.
#[derive(Debug, Clone)]
pub enum Node {
    /// A plain area change (or none).
    Plain,
    /// A wall that breathes: lining, side resonator.
    Shunt(Shunt),
    /// A resistance in the flow's way: `R` (Pa·s/m³, pressure drop over volume flow) in gas
    /// of impedance `rho_c` — a catalyst's monolith.
    Series(f64, f64),
    /// A frequency-dependent impedance in the flow's way.
    SeriesNet(SeriesNet),
    /// A frequency-dependent admittance off the duct.
    ShuntNet(ShuntNet),
}

impl Shunt {
    /// The packing along `length_m` of a duct whose passage faces it over `perimeter_m`, in gas
    /// of impedance `rho_c` (kg/(m²·s)), at step `dt`. Per metre r = σt/(3P) and c = φPt/(γp):
    /// the low-frequency limit of a rigid-backed porous layer (Allard & Atalla).
    pub fn lining(
        packing: crate::lining::Packing,
        perimeter_m: f64,
        length_m: f64,
        rho_c: f64,
        dt: f64,
    ) -> Self {
        let r = packing.flow_resistivity * packing.thickness_m / 3.0 / perimeter_m;
        let c = packing.porosity * perimeter_m * packing.thickness_m
            / (crate::gas::GAMMA * crate::units::ATM_PA);
        // Y(s) = K·s/(1 + sτ), per node; ×ρc to speak in the junction's area units
        let (k, tau, a) = (rho_c * c * length_m, r * c, 2.0 / dt);
        let den = 1.0 + a * tau;
        Self::from_coefficients(
            [k * a / den, -k * a / den, 0.0],
            [(1.0 - a * tau) / den, 0.0],
        )
    }

    /// A Helmholtz resonator off the duct: a neck of `neck_d` and effective length `neck_l`
    /// (end corrections included) into a cavity of `volume`, the neck's resistance `r_neck`
    /// (Pa·s/m³). Its branch draws Y(s) = sC/(1 + sRC + s²MC), M = ρl/Aₙ, C = V/(ρc²)
    /// (Kinsler et al., ch. 10): it short-circuits the duct at ω₀ = c·√(Aₙ/(lV)).
    pub fn helmholtz(
        neck_d: f64,
        neck_l: f64,
        volume: f64,
        r_neck: f64,
        rho_c: f64,
        c: f64,
        dt: f64,
    ) -> Self {
        let rho = rho_c / c;
        let a_n = std::f64::consts::PI * 0.25 * neck_d * neck_d;
        let (m, cap) = (rho * neck_l / a_n, volume / (rho * c * c));
        let (k, a) = (rho_c * cap, 2.0 / dt);
        // bilinear: s = a(1 − z⁻¹)/(1 + z⁻¹)
        let d0 = 1.0 + r_neck * cap * a + m * cap * a * a;
        let d1 = 2.0 - 2.0 * m * cap * a * a;
        let d2 = 1.0 - r_neck * cap * a + m * cap * a * a;
        Self::from_coefficients([k * a / d0, 0.0, -k * a / d0], [d1 / d0, d2 / d0])
    }

    fn from_coefficients(b: [f64; 3], a: [f64; 2]) -> Self {
        Self {
            b,
            a,
            p: [0.0; 2],
            q: [0.0; 2],
            inv: f64::NAN,
        }
    }

    /// Node pressure where pipes of `areas` meet with waves `incoming`, this shunt drawing its
    /// flow: Σ Aᵢ(2pᵢ − p) = q(p). Writes the outgoing waves; advances the shunt.
    fn node(&mut self, areas: &[f64; 2], incoming: &[f64; 2], outgoing: &mut [f64; 2]) {
        let hist = self.b[1] * self.p[0] + self.b[2] * self.p[1]
            - self.a[0] * self.q[0]
            - self.a[1] * self.q[1];
        if self.inv.is_nan() {
            self.inv = 1.0 / (areas[0] + areas[1] + self.b[0]);
        }
        let pj = (2.0 * (areas[0] * incoming[0] + areas[1] * incoming[1]) - hist) * self.inv;
        let q = self.b[0] * pj + hist;
        self.p = [pj, self.p[0]];
        self.q = [q, self.q[0]];
        outgoing[0] = pj - incoming[0];
        outgoing[1] = pj - incoming[1];
    }
}

impl Duct {
    /// `pipes` from the near end to the far end; at least one.
    pub fn new(pipes: Vec<Pipe>) -> Self {
        assert!(!pipes.is_empty(), "a duct needs a pipe");
        Self {
            pipes,
            mean_flow: 0.0,
            nodes: Vec::new(),
        }
    }
    /// Pipes with what happens at each inner node (`nodes[k]` between pipe k and k + 1;
    /// missing ones are plain).
    pub fn with_nodes(pipes: Vec<Pipe>, nodes: Vec<Node>) -> Self {
        Self {
            nodes,
            ..Self::new(pipes)
        }
    }
    /// A duct whose every inner node breathes through a copy of `shunt` (a lined passage
    /// cut into short pipes: the lumped transmission line).
    pub fn with_shunts(pipes: Vec<Pipe>, shunt: Shunt) -> Self {
        let n = pipes.len().saturating_sub(1);
        Self {
            nodes: vec![Node::Shunt(shunt); n],
            ..Self::new(pipes)
        }
    }
    /// This duct, then `next` after it (a hard node between them).
    pub fn then(self, next: Duct) -> Self {
        self.then_through(Node::Plain, next)
    }
    /// This duct, then `next`, joined by `node`.
    pub fn then_through(mut self, node: Node, next: Duct) -> Self {
        let n = self.pipes.len();
        self.nodes.resize(n - 1, Node::Plain);
        self.nodes.push(node);
        let mut rest = next.nodes;
        rest.resize(next.pipes.len() - 1, Node::Plain);
        self.nodes.extend(rest);
        self.pipes.extend(next.pipes);
        self
    }
    /// Area of the near (first) and far (last) pipe, m².
    pub fn end_areas(&self) -> (f64, f64) {
        (
            self.pipes[0].area_m2,
            self.pipes[self.pipes.len() - 1].area_m2,
        )
    }
    /// Wave arriving at the far end this step.
    pub fn at_far(&self) -> f64 {
        self.pipes[self.pipes.len() - 1].at_far()
    }
    /// Wave arriving back at the near end this step.
    pub fn at_near(&self) -> f64 {
        self.pipes[0].at_near()
    }
    /// The mean mass flow through every pipe of the duct, kg/s ([`Pipe::set_mean_flow`]).
    /// Its pipes, near end first.
    pub fn pipes_mut(&mut self) -> &mut [Pipe] {
        &mut self.pipes
    }
    pub fn set_mean_flow(&mut self, mass_flow: f64) {
        self.mean_flow = mass_flow;
        for p in &mut self.pipes {
            p.set_mean_flow(mass_flow);
        }
    }
    /// Send this step's waves into the ends; the inner area changes scatter what arrives at
    /// them this step.
    pub fn send(&mut self, from_near: f64, from_far: f64) {
        let n = self.pipes.len();
        let mut into_near = from_near;
        for k in 0..n {
            let (back, onward) = if k + 1 < n {
                let areas = [self.pipes[k].area_m2, self.pipes[k + 1].area_m2];
                let incoming = [self.pipes[k].at_far(), self.pipes[k + 1].at_near()];
                let mut out = [0.0; 2];
                match self.nodes.get_mut(k) {
                    Some(Node::Shunt(s)) => s.node(&areas, &incoming, &mut out),
                    Some(Node::SeriesNet(n)) => n.node(&areas, &incoming, &mut out),
                    Some(Node::ShuntNet(n)) => n.node(&areas, &incoming, &mut out),
                    Some(Node::Series(r, rho_c)) => {
                        // Q = 2(p₁ − p₂)/(z₁ + z₂ + R), z = ρc/A: each side loses z·Q
                        let (z1, z2) = (*rho_c / areas[0], *rho_c / areas[1]);
                        let q = 2.0 * (incoming[0] - incoming[1]) / (z1 + z2 + *r);
                        out = [incoming[0] - z1 * q, incoming[1] + z2 * q];
                    }
                    _ if self.mean_flow != 0.0 && areas[0] != areas[1] => {
                        // the mean flow loses pressure crossing the change: a resistance
                        let c = self.pipes[k].sound_speed;
                        let r = area_change_resistance(areas[0], areas[1], self.mean_flow, c);
                        let rho_c = crate::gas::GAMMA * crate::units::ATM_PA / c;
                        let (z1, z2) = (rho_c / areas[0], rho_c / areas[1]);
                        let q = 2.0 * (incoming[0] - incoming[1]) / (z1 + z2 + r);
                        out = [incoming[0] - z1 * q, incoming[1] + z2 * q];
                    }
                    _ => {
                        junction(&areas, &incoming, &mut out);
                    }
                }
                (out[0], out[1])
            } else {
                (from_far, 0.0)
            };
            self.pipes[k].send(into_near, back);
            into_near = onward;
        }
    }
}

/// N-port pressure junction (Kelly–Lochbaum, admittance ∝ area):
/// p_J = 2·ΣAᵢ·pᵢ / ΣAᵢ, outgoing pᵢ' = p_J − pᵢ. Returns p_J; writes outgoing waves.
pub fn junction(areas: &[f64], incoming: &[f64], outgoing: &mut [f64]) -> f64 {
    let total: f64 = areas.iter().sum();
    let pj = 2.0 * areas.iter().zip(incoming).map(|(a, p)| a * p).sum::<f64>() / total;
    for (o, p) in outgoing.iter_mut().zip(incoming) {
        *o = pj - p;
    }
    pj
}

/// Acoustic resistance, Pa·s/m³ (pressure drop over volume flow), of a sudden change of area
/// from `a_from` to `a_to` (m², along the flow) crossed by a mean mass flow `mass_flow` (kg/s,
/// negative: the other way) of gas whose speed of sound is `c`, at an atmosphere. The flow loses
/// ½ρKU² there (Borda–Carnot for an expansion, K = (1 − σ)² on the upstream velocity; a
/// contraction K = 0.42(1 − σ) on the downstream one, White, *Fluid Mechanics*, §6.9; σ the area
/// ratio, < 1): a sound wave riding on it changes that loss by ρKU·u, a resistance ρKU/A on its
/// volume flow — the quasi-steady loss of area changes in flow ducts that damps their resonances
/// (Ronneberger 1967; Davies, *JSV* 124 (1988) §3; Munjal, *Acoustics of Ducts and Mufflers*,
/// 2nd ed., §2.13).
pub fn area_change_resistance(a_from: f64, a_to: f64, mass_flow: f64, c: f64) -> f64 {
    let (up, down) = if mass_flow >= 0.0 {
        (a_from, a_to)
    } else {
        (a_to, a_from)
    };
    if up <= 0.0 || down <= 0.0 || up == down {
        return 0.0;
    }
    let rho = crate::gas::GAMMA * crate::units::ATM_PA / (c * c);
    let m = mass_flow.abs();
    // K and the area its velocity is taken in
    let (k, a) = if down > up {
        ((1.0 - up / down).powi(2), up)
    } else {
        (0.42 * (1.0 - down / up), down)
    };
    rho * k * (m / (rho * a)) / a
}

/// Newton iterations of [`lossy_junction`].
const NEWTON_STEPS: usize = 6;

/// [`junction`] where the flow loses pressure entering and leaving through it: branch `i`'s end
/// pressure is the junction's plus `s·u|u|`, `s` = `losses[i].0` while its gas flows into the
/// junction (u > 0) and `losses[i].1` while it flows out (each ½ρK: Borda–Carnot for a sudden
/// expansion, a contraction coefficient the other way; Benson, *Thermodynamics and Gas Dynamics
/// of IC Engines* vol. 1, ch. 8, "pressure loss" junctions). The quasi-steady loss acts on the
/// instantaneous velocity of the waves, u = (incoming − outgoing)/`z`, with mass conserved,
/// ΣAᵢuᵢ = 0. Each branch's end gives P = 2pᵢ − z·uᵢ, so every uᵢ is a monotone function of
/// the junction pressure; Newton's method finds it. Returns p_J; writes outgoing waves.
pub fn lossy_junction(
    areas: &[f64],
    incoming: &[f64],
    losses: &[(f64, f64)],
    z: f64,
    outgoing: &mut [f64],
) -> f64 {
    // the velocity into the junction of branch i when its pressure is p_J (and its slope)
    let velocity = |i: usize, pj: f64| -> (f64, f64) {
        let r = 2.0 * incoming[i] - pj;
        let s = if r >= 0.0 { losses[i].0 } else { losses[i].1 };
        if s <= 0.0 {
            return (r / z, -1.0 / z);
        }
        // s·u|u| + z·u = r, the root with the sign of r
        let root = (z * z + 4.0 * s * r.abs()).sqrt();
        let u = r.signum() * (root - z) / (2.0 * s);
        (u, -1.0 / root)
    };
    let total: f64 = areas.iter().sum();
    let mut pj = 2.0 * areas.iter().zip(incoming).map(|(a, p)| a * p).sum::<f64>() / total;
    // the mass balance is monotone and smooth in p_J: Newton from the lossless junction converges
    // quadratically, well inside NEWTON_STEPS iterations for waves of a few bar
    for _ in 0..NEWTON_STEPS {
        let (mut f, mut df) = (0.0, 0.0);
        for (i, a) in areas.iter().enumerate() {
            let (u, du) = velocity(i, pj);
            f += a * u;
            df += a * du;
        }
        pj -= f / df;
    }
    for (i, o) in outgoing.iter_mut().enumerate() {
        *o = incoming[i] - z * velocity(i, pj).0;
    }
    pj
}

/// Open pipe end radiating into air: reflection −r·LP(p) with a one-pole low-pass at the
/// frequency where ka = 1 (|R| ≈ 1 below it, falling above; Levine & Schwinger 1948).
#[derive(Debug, Clone)]
pub struct OpenEnd {
    r: f64,
    k: f64,
    lp: f64,
    /// Reflection factor of the outflow's jet, (1 − M)/(1 + M).
    jet: f64,
}

impl OpenEnd {
    pub fn new(diameter_m: f64, dt: f64) -> Self {
        /// Speed of sound in the ambient air the pipe radiates into, m/s.
        const C_AIR: f64 = 343.0;
        let fc = C_AIR / (std::f64::consts::PI * diameter_m); // ka = 1, a = d/2
        Self {
            r: 0.98,
            k: 1.0 - (-2.0 * std::f64::consts::PI * fc * dt).exp(),
            lp: 0.0,
            jet: 1.0,
        }
    }
    /// Mean outflow at the end, as a Mach number. At low Strouhal number the jet sheds the
    /// sound energy the flow convects out as vorticity (a Kutta condition at the edge): the
    /// reflection of a pipe without convection falls by (1 − M)/(1 + M) (Munt 1977, Rienstra
    /// 1983; measured by Peters, Hirschberg, Reijnen & Wijnands, JFM 256, 1993). Inflow (a
    /// reversing pulse) gets no such sink.
    pub fn set_outflow_mach(&mut self, mach: f64) {
        let m = mach.clamp(0.0, 0.99);
        self.jet = (1.0 - m) / (1.0 + m);
    }
    /// Given the arriving wave, returns (reflected wave, outflow volume velocity × ρc =
    /// p − reflected). The far-field sound is the time derivative of the latter (monopole).
    pub fn process(&mut self, arriving: f64) -> (f64, f64) {
        self.lp += self.k * (arriving - self.lp);
        let reflected = -self.r * self.jet * self.lp;
        (reflected, arriving - reflected)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_junction_loses_pressure_as_flow_expands_or_contracts_through_it() {
        // A pipe of area 1 into one of area 4 (σ = 0.25), gas of ρ = 0.4, ρc = 240. A 10 kPa wave
        // arriving down the small pipe: the flow into the big one expands, losing
        // ½ρ·(1 − σ)²·u² (Borda–Carnot; White, *Fluid Mechanics*, §6.9): with mass conserved,
        // p_J = 3904.71 Pa (lossless 4000), u = 65.078 m/s, 476.46 Pa lost, the small pipe's
        // reflection −5618.83 Pa (hand, the quadratic solved)
        let expansion = 0.5 * 0.4 * 0.75f64.powi(2);
        let contraction = 0.5 * 0.4 * 0.42 * 0.75;
        let losses = [(expansion, contraction), (0.0, 0.0)];
        let mut out = [0.0; 2];
        let pj = lossy_junction(&[1.0, 4.0], &[10_000.0, 0.0], &losses, 240.0, &mut out);
        assert!((pj - 3904.71).abs() < 0.01, "{pj}");
        assert!((out[0] + 5618.83).abs() < 0.01, "{out:?}");
        assert!((out[1] - 3904.71).abs() < 0.01, "{out:?}");
        // the other way, a sudden contraction (K = 0.42·(1 − σ)): a 10 kPa wave down the big pipe
        // drives 65.759 m/s into the small one and holds p_J at 16054.48 Pa (lossless 16000)
        let pj = lossy_junction(&[1.0, 4.0], &[0.0, 10_000.0], &losses, 240.0, &mut out);
        assert!((pj - 16_054.48).abs() < 0.01, "{pj}");
        assert!((out[0] - 15_782.06).abs() < 0.01, "{out:?}");
        // a 200 kPa wave, where the loss is a third of the junction's pressure: the quadratic
        // 400000 = 5·p_J + 0.1125·(4·p_J/240)²·… gives p_J = 58564.06 Pa (lossless 80000) (hand)
        let pj = lossy_junction(&[1.0, 4.0], &[200_000.0, 0.0], &losses, 240.0, &mut out);
        assert!((pj - 58_564.06).abs() < 0.01, "{pj}");
        // without losses it is the lossless junction
        let mut plain = [0.0; 3];
        let mut lossy = [0.0; 3];
        let p = junction(&[1.0, 1.0, 3.0], &[500.0, -200.0, 50.0], &mut plain);
        let q = lossy_junction(
            &[1.0, 1.0, 3.0],
            &[500.0, -200.0, 50.0],
            &[(0.0, 0.0); 3],
            240.0,
            &mut lossy,
        );
        assert!(
            (p - q).abs() < 1e-9 && plain.iter().zip(&lossy).all(|(a, b)| (a - b).abs() < 1e-9)
        );
    }

    #[test]
    fn a_shock_moves_at_the_mean_of_its_two_sides() {
        // Rankine–Hugoniot for a simple wave (Burgers): a jump from 0 to p moves at the mean of
        // the two sides' speeds, c·(1 + k·p/2) (Whitham, *Linear and Nonlinear Waves*, §2.4).
        // The step 0 → 20 kPa in 1 m of gas at c = 500 (k = 1.2/(283.71·500)) arrives
        // 200/(1 + k·10⁴) = 184.401 steps after it is sent (hand); its middle, 10 kPa, then
        let k = 1.2 / (283.71 * 500.0);
        let mut line = DelayLine::nonlinear(200.0, k);
        let (mut prev, mut crossed) = (0.0, None);
        for n in 0..1000 {
            let out = line.read();
            if prev < 10_000.0 && out >= 10_000.0 {
                crossed = Some(n as f64 - 1.0 + (10_000.0 - prev) / (out - prev) - 500.0);
            }
            prev = out;
            line.push(if n >= 500 { 20_000.0 } else { 0.0 });
        }
        let t = crossed.unwrap();
        assert!((t - 184.401).abs() < 1.0, "{t}");
    }

    #[test]
    fn a_shock_keeps_the_waves_content() {
        // what a line carries in is what comes out, ∫p dt, shocks or not: the merged samples keep
        // their sum (Brenier & Grenier, SIAM J. Numer. Anal. 35, 1998). Reading the shock at the
        // grid's scale (`MIN_KERNEL`) loses a little of the strongest ones — measured 1.9 % at
        // 40 kPa and 3.5 % at 60 kPa — and none below 20 kPa
        let content = |amp: f64| {
            let k = 1.2 / (283.71 * 500.0);
            let mut line = DelayLine::nonlinear(400.0, k);
            let (mut sent, mut got) = (0.0, 0.0);
            for n in 0..20_000 {
                got += line.read();
                let x = if n < 16_000 && n % 800 < 100 {
                    amp * (std::f64::consts::PI * (n % 800) as f64 / 100.0).sin()
                } else {
                    0.0
                };
                sent += x;
                line.push(x);
            }
            got / sent
        };
        let low = content(2e4);
        assert!((low - 1.0).abs() < 1e-3, "{low}");
        let high = content(6e4);
        assert!((0.95..1.0).contains(&high), "{high}");
    }

    #[test]
    fn a_faint_wave_crosses_a_nonlinear_line_as_a_linear_one() {
        // amplitude → 0: the arrival times are the fixed delay's. The line reads its values
        // with a tent two steps wide (`MIN_KERNEL`); on a whole-step delay that is the filter
        // ¼, ½, ¼, |H| = ½ + ½·cos ω: 1 kHz at 192 kHz passes at 0.99973 and 10 kHz at 0.97347
        // (−0.23 dB) (hand)
        let tone = |f: f64, nonlinear: bool| {
            let w = 2.0 * std::f64::consts::PI * f / 192_000.0;
            let mut line = if nonlinear {
                DelayLine::nonlinear(40.0, 8.46e-6)
            } else {
                DelayLine::new(40.0)
            };
            let (mut re, mut im) = (0.0, 0.0);
            for n in 0..19_200 {
                let out = line.read();
                assert!(line.read_next().is_some());
                line.push(1e-3 * (w * n as f64).sin());
                if n >= 9_600 {
                    re += out * (w * n as f64).cos();
                    im += out * (w * n as f64).sin();
                }
            }
            re.hypot(im)
        };
        for (f, gain) in [(1000.0, 0.999_73), (10_000.0, 0.973_47)] {
            let ratio = tone(f, true) / tone(f, false);
            assert!((ratio - gain).abs() < 2e-4, "{f} Hz: {ratio}");
        }
    }

    #[test]
    fn a_loud_sample_travels_at_c_plus_its_own_velocity() {
        // Msallam et al. (Acta Acustica 86, 2000) eq. 1–2: a simple wave's value rides at
        // c + (γ+1)/2·u, u = p/ρc. 1 m of gas at c = 500 m/s (ρc = 1.4·101325/500 = 283.71),
        // steps of 10 µs: 200 steps when faint. A slow ramp of 1 Pa per step (no shock forms):
        // the 10 kPa sample, pushed at step 10000, rides at 500·(1 + 1.2·10000/(283.71·500)) =
        // 542.30 m/s and comes out 184.401 steps later (hand)
        let k = 1.2 / (283.71 * 500.0);
        let mut line = DelayLine::nonlinear(200.0, k);
        let mut prev = 0.0;
        let mut crossed = None;
        for n in 0..12_000 {
            let out = line.read();
            if prev < 10_000.0 && out >= 10_000.0 {
                // linear between the two reads
                crossed = Some(n as f64 - 1.0 + (10_000.0 - prev) / (out - prev));
            }
            prev = out;
            line.push(n as f64);
        }
        let t = crossed.unwrap();
        assert!((t - 10_184.401).abs() < 0.05, "{t}");
    }

    #[test]
    fn a_loud_tone_grows_harmonics_as_fubini_found() {
        // Before a shock forms the simple wave's harmonics are Fubini's: B_n = 2J_n(nσ)/(nσ),
        // σ = x/x̄, x̄ = c²/(βωu₀) (Hamilton & Blackstock, *Nonlinear Acoustics*, ch. 4). Air at
        // c = 343 (ρc = 413.57), 2 kPa at 200 Hz: u₀ = 4.836 m/s, x̄ = 16.133 m; after 2 m σ =
        // 0.12397: B₁ = 0.99808, B₂ = 0.061667, B₂/B₁ = 0.061786 (scipy's Bessel functions)
        let dt = 1e-5;
        let mut pipe = Pipe::new(2.0, 0.05, 343.0, dt, 0.0).nonlinear();
        let w = 2.0 * std::f64::consts::PI * 200.0;
        let (mut h1, mut h2) = ((0.0, 0.0), (0.0, 0.0));
        for n in 0..40_000 {
            let t = n as f64 * dt;
            let out = pipe.at_far();
            pipe.send(2000.0 * (w * t).sin(), 0.0);
            // the last 0.2 s (40 periods), after the wave has crossed
            if n >= 20_000 {
                h1.0 += out * (w * t).cos();
                h1.1 += out * (w * t).sin();
                h2.0 += out * (2.0 * w * t).cos();
                h2.1 += out * (2.0 * w * t).sin();
            }
        }
        let ratio = h2.0.hypot(h2.1) / h1.0.hypot(h1.1);
        assert!((ratio - 0.061_786).abs() < 0.003, "{ratio}");
        // and a linear pipe makes none
        let mut lin = Pipe::new(2.0, 0.05, 343.0, dt, 0.0);
        let mut h = (0.0, 0.0);
        for n in 0..40_000 {
            let t = n as f64 * dt;
            let out = lin.at_far();
            lin.send(2000.0 * (w * t).sin(), 0.0);
            if n >= 20_000 {
                h.0 += out * (2.0 * w * t).cos();
                h.1 += out * (2.0 * w * t).sin();
            }
        }
        assert!(h.0.hypot(h.1) / h1.0.hypot(h1.1) < 1e-6);
    }

    /// Steady transmitted amplitude (re a unit incident sine at `f`) through `duct` with an
    /// anechoic source and termination.
    fn transmission(mut duct: Duct, f: f64, dt: f64) -> f64 {
        let n = (0.3 / dt) as usize;
        let mut peak: f64 = 0.0;
        for k in 0..n {
            let x = (2.0 * std::f64::consts::PI * f * k as f64 * dt).sin();
            if k > n / 2 {
                peak = peak.max(duct.at_far().abs());
            }
            duct.send(x, 0.0);
        }
        peak
    }

    /// A lined duct of `n` segments over `length`: the packing of the test below.
    fn lined(length: f64, n: usize, dt: f64) -> Duct {
        // passage r = 44.1 mm (a 0.006 m³ can of 0.4 m, 25 mm packing): A = 6.1095e-3 m²,
        // P = 0.27708 m; σ = 16 kPa·s/m², φ = 0.95, gas at 622.2 K (c = 500 m/s, ρ = 0.56742)
        let a = 6.109_476_6e-3;
        let d = (4.0 * a / std::f64::consts::PI).sqrt();
        let pipes = (0..n)
            .map(|_| Pipe::new(length / n as f64, d, 500.0, dt, 0.0))
            .collect();
        // each of the n − 1 inner nodes carries the lining of length/(n − 1)
        let shunt = Shunt::lining(
            crate::lining::Packing::default(),
            0.277_081_12,
            length / (n - 1) as f64,
            0.567_42 * 500.0,
            dt,
        );
        Duct::with_shunts(pipes, shunt)
    }

    #[test]
    fn an_area_change_in_flow_resists_by_its_loss() {
        // 0.05 kg/s from 0.002 m² into 0.008 m² of 500 m/s gas: ρ = 1.4·101325/500² = 0.567420,
        // U = 0.05/(0.56742·0.002) = 44.059 m/s, K = (1 − 0.25)² = 0.5625, R = ρKU/A =
        // 0.56742·0.5625·44.059/0.002 = 7031.3 Pa·s/m³ (hand). The other way it contracts:
        // K = 0.42·0.75 = 0.315 on the narrow side's U, R = 0.56742·0.315·44.059/0.002 = 3937.5.
        let r = area_change_resistance(0.002, 0.008, 0.05, 500.0);
        assert!((r - 7031.3).abs() < 0.5, "{r}");
        let back = area_change_resistance(0.002, 0.008, -0.05, 500.0);
        assert!((back - 3937.5).abs() < 0.5, "{back}");
        assert_eq!(area_change_resistance(0.002, 0.002, 0.05, 500.0), 0.0);
        assert_eq!(area_change_resistance(0.002, 0.008, 0.0, 500.0), 0.0);
    }

    #[test]
    fn a_duct_in_flow_passes_less_across_its_area_change() {
        // A wave from a 0.002 m² pipe into a 0.008 m² one: lossless it passes 2z₂/(z₁ + z₂) =
        // 0.4; with R = 7031.3 (above; z₁ = ρc/A₁ = 141855, z₂ = 35464) it passes
        // 2·35464/(141855 + 35464 + 7031) = 0.38475 (hand). Each pipe 1 step long at 500 m/s.
        let dt = 1.0 / 192_000.0;
        let len = 500.0 * dt * 8.0;
        let d1 = (4.0 * 0.002 / std::f64::consts::PI).sqrt();
        let d2 = (4.0 * 0.008 / std::f64::consts::PI).sqrt();
        let passed = |flow: f64| {
            let mut duct = Duct::new(vec![
                Pipe::new(len, d1, 500.0, dt, 0.0),
                Pipe::new(len, d2, 500.0, dt, 0.0),
            ]);
            duct.set_mean_flow(flow);
            let gains = duct.pipes[0].gain * duct.pipes[1].gain;
            let mut peak: f64 = 0.0;
            for k in 0..64 {
                duct.send(if k == 0 { 1.0 } else { 0.0 }, 0.0);
                peak = peak.max(duct.pipes[1].at_far().abs());
            }
            peak / gains
        };
        assert!((passed(0.0) - 0.4).abs() < 1e-9, "{}", passed(0.0));
        assert!((passed(0.05) - 0.38475).abs() < 1e-4, "{}", passed(0.05));
    }

    #[test]
    fn a_packed_duct_attenuates_as_its_transmission_line() {
        // Per metre, by hand: r = σt/3/P = 481.21 Pa·s/m², c = φPt/(γp) = 4.6390e-8 m³/Pa,
        // y = jωc/(1 + jωrc); z = jωρ/A, Y₀ = jωA/(ρc²); γ = √(z·(Y₀ + y)) (Munjal ch. 6, the
        // locally reacting lining): Re γ = 0.6488, 2.4839, 8.4981 Np/m at 1, 2 and 4 kHz. The
        // pocket's compliance is as large as the duct's own here, so the first-order α = Re(y)
        // ρc/2A (0.93, 3.52, 11.55) is 40 % off: the lining slows the wave too. 0.4 m more of
        // duct cuts the transmission by e^(−0.4·Re γ): 2.25, 8.63 and 29.5 dB. Cut into 1 cm
        // pipes the lumped line measures 2.33, 8.78 and 30.99 dB (2 cm: 34.7 dB at 4 kHz).
        let dt = 1.0 / 192_000.0;
        for (f, db) in [(1000.0, 2.254), (2000.0, 8.630), (4000.0, 29.526)] {
            let short = transmission(lined(0.4, 40, dt), f, dt);
            let long = transmission(lined(0.8, 80, dt), f, dt);
            let got = 20.0 * (short / long).log10();
            assert!(
                (got - db).abs() < 0.06 * db + 0.1,
                "{f} Hz: {got:.2} dB, want {db}"
            );
        }
    }

    #[test]
    fn a_longer_flowing_pipe_and_an_extra_loss_by_hand() {
        // M = 0.2 in a 50 mm pipe: α = f·M/(2D) = 0.025·0.2/0.1 = 0.05 Np/m, so 2 m keep
        // e^(−0.1) = 0.904837
        // on top of the still loss (1 − 0.03)² = 0.9409
        let dt = 1.0 / 192_000.0;
        let a = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        let mut p = Pipe::new(2.0, 0.05, 500.0, dt, 0.03);
        p.set_mean_flow(0.2 * 283.71 * a);
        assert!(
            (p.gain / 0.9409 - 0.904_837).abs() < 1e-5,
            "{}",
            p.gain / 0.9409
        );
        // an extra per-trip factor (a muffler's) multiplies a fresh pipe's: 0.97·0.5
        let mut q = Pipe::new(1.0, 0.05, 500.0, dt, 0.03).with_extra_gain(0.5);
        assert!((q.gain - 0.485).abs() < 1e-12, "{}", q.gain);
        // and it stays when the flow sets the gain afresh from the still pipe's
        q.set_mean_flow(0.0);
        assert!((q.gain - 0.485).abs() < 1e-12, "{}", q.gain);
    }

    #[test]
    fn the_next_read_is_known_a_step_early() {
        // a 2.4-step delay: after each push, read() returns what read_next() said before it
        let mut d = DelayLine::new(2.4);
        for k in 0..50 {
            let next = d.read_next().unwrap();
            d.push((k as f64 * 0.37).sin());
            assert_eq!(d.read(), next, "step {k}");
        }
        // shorter than two steps it cannot be: the next read needs the value about to come
        assert_eq!(DelayLine::new(1.5).read_next(), None);
    }

    #[test]
    fn a_delay_line_reads_ahead_what_it_will_read_after_more_pushes() {
        // a 5.3-step delay with a ramp pushed: read_ahead(k) says what read() returns after k
        // more pushes, for every k below the whole delay (5), and nothing beyond
        let mut d = DelayLine::new(5.3);
        for i in 0..20 {
            d.push(f64::from(i) * 1.5 - 4.0);
        }
        for ahead in 0..8 {
            let said = d.read_ahead(ahead);
            let mut e = d.clone();
            for _ in 0..ahead {
                e.push(1000.0);
            }
            if ahead < 5 {
                assert_eq!(said.unwrap().to_bits(), e.read().to_bits(), "{ahead}");
            } else {
                assert_eq!(said, None, "{ahead}");
            }
        }
        assert_eq!(d.whole_steps(), 5);
    }

    #[test]
    fn a_flowing_pipe_wears_its_waves_down() {
        // D = 50 mm, 1 m, c = 500 m/s (ρc = γp/c = 283.71): 0.0557 kg/s is M = 0.2; f = 0.025
        // → α = f·M/(2D) = 0.05 Np/m: a wave keeps e^(−0.05) = 0.951229 of itself, on top of the
        // still pipe's (1 − 0.03)¹
        let dt = 1.0 / 192_000.0;
        let mut p = Pipe::new(1.0, 0.05, 500.0, dt, 0.03);
        let still = 0.97;
        let a = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        p.set_mean_flow(0.2 * 283.71 * a);
        p.send(1.0, 1.0);
        for _ in 0..400 {
            p.send(0.0, 0.0);
            if p.fwd.read() != 0.0 {
                break;
            }
        }
        assert!(
            (p.gain / still - 0.951_229).abs() < 1e-5,
            "{}",
            p.gain / still
        );
        // and the flow's direction does not matter
        p.set_mean_flow(-0.2 * 283.71 * a);
        assert!((p.gain / still - 0.951_229).abs() < 1e-5);
        p.set_mean_flow(0.0);
        assert_eq!(p.gain, still);
    }

    #[test]
    fn an_outflow_takes_sound_energy_out_of_the_open_end() {
        // A slow wave (far below ka = 1) meets the open end: without flow it comes back
        // inverted at r = 0.98; with outflow at Mach M the jet carries off the convected energy,
        // the reflection falling by (1 − M)/(1 + M) at low Strouhal number (Munt 1977; measured
        // by Peters, Hirschberg et al., JFM 256, 1993): M = 0.17 → 0.98·0.83/1.17 = 0.6952.
        let dt = 1.0 / 192_000.0;
        let settle = |m: f64| {
            let mut end = OpenEnd::new(0.05, dt);
            end.set_outflow_mach(m);
            let mut r = 0.0;
            for _ in 0..2000 {
                r = end.process(1.0).0;
            }
            r
        };
        assert!((settle(0.0) + 0.98).abs() < 1e-9);
        assert!((settle(0.17) + 0.695_2).abs() < 1e-4, "{}", settle(0.17));
        // inflow (a reversing pulse) gets no such sink here
        assert!((settle(-0.2) + 0.98).abs() < 1e-9);
    }

    #[test]
    fn a_series_resistance_has_its_transmission_loss() {
        // A resistance R in a uniform duct (area S, gas impedance ρc): TL = 20·log(1 + R·S/2ρc)
        // (Kinsler et al., ch. 10). R = ρc/S: 20·log 1.5 = 3.522 dB at any frequency.
        let dt = 1.0 / 192_000.0;
        let rho_c = 283.71;
        let s_area = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        let duct = || {
            Duct::with_nodes(
                vec![
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                ],
                vec![Node::Series(rho_c / s_area, rho_c)],
            )
        };
        for f in [200.0, 1500.0] {
            let tl = -20.0 * transmission(duct(), f, dt).log10();
            assert!((tl - 3.522).abs() < 0.05, "{f} Hz: {tl:.3} dB");
        }
    }

    #[test]
    fn a_series_network_has_its_transmission_loss() {
        // A parallel RL in series, Z = R·iωT/(1 + iωT), R = 2ρc/S: at ωT = 1 Z·S/2ρc = (1 + i)/2,
        // TL = 20·log|1.5 + 0.5i| = 3.979 dB; far above, Z → R: 20·log 2 = 6.021 dB (Kinsler
        // et al., ch. 10)
        let dt = 1.0 / 192_000.0;
        let rho_c = 283.71;
        let s_area = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        let f1 = 300.0;
        let branch = crate::narrow_tube::Branch {
            k: 2.0 * rho_c / s_area,
            t_s: 1.0 / (2.0 * std::f64::consts::PI * f1),
        };
        let duct = || {
            Duct::with_nodes(
                vec![
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                ],
                vec![Node::SeriesNet(SeriesNet::new(0.0, &[branch], rho_c, dt))],
            )
        };
        for (f, want) in [(f1, 3.979), (30.0 * f1, 6.021)] {
            let tl = -20.0 * transmission(duct(), f, dt).log10();
            assert!((tl - want).abs() < 0.1, "{f} Hz: {tl:.3} dB, want {want}");
        }
    }

    #[test]
    fn a_shunt_network_has_its_transmission_loss() {
        // A series RC to the side, Y = G·iωT/(1 + iωT), G = 2S/ρc: TL = 20·log|1 + Y·ρc/2S|,
        // 3.979 dB at ωT = 1 and 6.021 dB far above (Kinsler et al., ch. 10)
        let dt = 1.0 / 192_000.0;
        let rho_c = 283.71;
        let s_area = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        let f1 = 300.0;
        let branch = crate::narrow_tube::Branch {
            k: 2.0 * s_area / rho_c,
            t_s: 1.0 / (2.0 * std::f64::consts::PI * f1),
        };
        let duct = || {
            Duct::with_nodes(
                vec![
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                ],
                vec![Node::ShuntNet(ShuntNet::new(&[branch], rho_c, dt))],
            )
        };
        for (f, want) in [(f1, 3.979), (30.0 * f1, 6.021)] {
            let tl = -20.0 * transmission(duct(), f, dt).log10();
            assert!((tl - want).abs() < 0.1, "{f} Hz: {tl:.3} dB, want {want}");
        }
    }

    #[test]
    fn a_side_resonator_has_its_transmission_loss() {
        // A Helmholtz resonator on a 50 mm duct: neck 30 mm, effective length 50 mm, 3 L; gas
        // at c = 500 m/s, ρ = 0.56742. Z = j(ωM − 1/ωC), M = ρl/Aₙ, C = V/(ρc²): resonance
        // f₀ = c/2π·√(Aₙ/(lV)) = 172.75 Hz; TL = 20·log|1 + (ρc/S)/(2Z)| (Kinsler ch. 10):
        // 3.468 dB at f₀/2 and at 2f₀, 13.315 dB at 1.2 f₀ — the notch is narrow and deep.
        let dt = 1.0 / 192_000.0;
        let duct = || {
            Duct::with_nodes(
                vec![
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                ],
                vec![Node::Shunt(Shunt::helmholtz(
                    0.03, 0.05, 0.003, 0.0, 283.71, 500.0, dt,
                ))],
            )
        };
        for (f, want) in [(86.37, 3.468), (345.49, 3.468), (207.30, 13.315)] {
            let tl = -20.0 * transmission(duct(), f, dt).log10();
            assert!((tl - want).abs() < 0.25, "{f} Hz: {tl:.3} dB, want {want}");
        }
        assert!(-20.0 * transmission(duct(), 172.75, dt).log10() > 30.0);
    }

    #[test]
    fn a_resistive_neck_flattens_the_resonators_notch() {
        // the same resonator with a neck resistance equal to the duct's ρc/S = 144 492 Pa·s/m³:
        // at f₀ the branch is the resistance alone, TL = 20·log(1 + ½) = 3.522 dB; at 1.2 f₀
        // (X = ωM − 1/ωC = 15 975) TL = 20·log|1 + 72 246/(144 492 + 15 975j)| = 3.493 dB
        let dt = 1.0 / 192_000.0;
        let duct = || {
            Duct::with_nodes(
                vec![
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                ],
                vec![Node::Shunt(Shunt::helmholtz(
                    0.03, 0.05, 0.003, 144_492.3, 283.71, 500.0, dt,
                ))],
            )
        };
        for (f, want) in [(172.75, 3.522), (207.30, 3.493)] {
            let tl = -20.0 * transmission(duct(), f, dt).log10();
            assert!((tl - want).abs() < 0.1, "{f} Hz: {tl:.3} dB, want {want}");
        }
    }

    #[test]
    fn joined_ducts_keep_one_node_per_junction_in_place() {
        // three lined pipes (two shunt nodes), then through a series resistance into two plain
        // pipes: five pipes, four junctions — shunt, shunt, the series one, plain
        let dt = 1.0 / 192_000.0;
        let lined = Duct::with_shunts(
            (0..3)
                .map(|_| Pipe::new(0.1, 0.05, 500.0, dt, 0.0))
                .collect(),
            Shunt::lining(crate::lining::Packing::default(), 0.157, 0.1, 283.71, dt),
        );
        let plain = Duct::new(
            (0..2)
                .map(|_| Pipe::new(0.1, 0.05, 500.0, dt, 0.0))
                .collect(),
        );
        let d = lined.then_through(Node::Series(1000.0, 283.71), plain);
        assert_eq!(d.pipes.len(), 5);
        assert_eq!(d.nodes.len(), 4);
        assert!(matches!(d.nodes[0], Node::Shunt(_)) && matches!(d.nodes[1], Node::Shunt(_)));
        assert!(matches!(d.nodes[2], Node::Series(r, _) if r == 1000.0));
        assert!(matches!(d.nodes[3], Node::Plain));
    }

    #[test]
    fn a_duct_carries_both_ways_and_wears_its_waves_with_flow() {
        // two 1 m, 50 mm pipes through a series resistance: equal pulses from both ends meet
        // at it with no pressure across it, so no flow — each end gets a whole pulse back
        let dt = 1.0 / 192_000.0;
        let pair = || {
            Duct::with_nodes(
                vec![
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                    Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
                ],
                vec![Node::Series(50_000.0, 283.71)],
            )
        };
        let mut d = pair();
        let (mut near, mut far): (f64, f64) = (0.0, 0.0);
        for k in 0..2_000 {
            near = near.max(d.at_near().abs());
            far = far.max(d.at_far().abs());
            let x = if k == 0 { 1.0 } else { 0.0 };
            d.send(x, x);
        }
        assert!(
            (near - 1.0).abs() < 1e-12 && (far - 1.0).abs() < 1e-12,
            "{near} {far}"
        );
        // a mean flow of M = 0.2 through both pipes: α = f·M/(2D) = 0.025·0.2/0.1 = 0.05 Np/m,
        // so a pulse crossing the 2 m keeps e^(−0.1) = 0.904837 (the resistance aside: here a
        // plain duct)
        let a = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        let mut d = Duct::new(vec![
            Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
            Pipe::new(1.0, 0.05, 500.0, dt, 0.0),
        ]);
        d.set_mean_flow(0.2 * 283.71 * a);
        let mut got: f64 = 0.0;
        for k in 0..2_000 {
            got = got.max(d.at_far().abs());
            d.send(if k == 0 { 1.0 } else { 0.0 }, 0.0);
        }
        assert!((got - 0.904_837).abs() < 1e-5, "{got}");
    }

    #[test]
    fn an_expansion_chamber_has_munjals_transmission_loss() {
        // m = 8 (A₂/A₁), L = 0.4 m, c = 500 m/s, lossless: TL = 10·log[1 + ¼(m − 1/m)²·sin²(kL)]
        // (Munjal, *Acoustics of Ducts and Mufflers*, 1987, ch. 3, the simple expansion chamber):
        // kL = π/4 at 156.25 Hz → 9.42 dB; π/2 at 312.5 Hz → 12.17 dB; π at 625 Hz → 0 dB
        let dt = 1.0 / 192_000.0;
        let d1 = 0.05;
        let d2 = d1 * 8f64.sqrt();
        let chamber = || {
            Duct::new(vec![
                Pipe::new(1.0, d1, 500.0, dt, 0.0),
                Pipe::new(0.4, d2, 500.0, dt, 0.0),
                Pipe::new(1.0, d1, 500.0, dt, 0.0),
            ])
        };
        for (f, tl) in [(156.25, 9.42), (312.5, 12.17), (625.0, 0.0)] {
            let got = -20.0 * transmission(chamber(), f, dt).log10();
            assert!((got - tl).abs() < 0.3, "{f} Hz: {got:.2} dB, want {tl}");
        }
        // a straight duct passes everything
        let straight = Duct::new(vec![Pipe::new(2.4, d1, 500.0, dt, 0.0)]);
        assert!((transmission(straight, 312.5, dt) - 1.0).abs() < 1e-3);
    }

    use approx::assert_relative_eq;

    #[test]
    fn delay_line_integer_and_fractional() {
        let mut d = DelayLine::new(3.0);
        let mut out = Vec::new();
        for x in [1.0, 0.0, 0.0, 0.0, 0.0] {
            out.push(d.read());
            d.push(x);
        }
        assert_eq!(out, vec![0.0, 0.0, 0.0, 1.0, 0.0]);
        let mut d = DelayLine::new(2.5);
        let mut out = Vec::new();
        for x in [1.0, 0.0, 0.0, 0.0, 0.0] {
            out.push(d.read());
            d.push(x);
        }
        assert_eq!(out, vec![0.0, 0.0, 0.5, 0.5, 0.0]);
    }

    #[test]
    fn pipe_delay_is_length_over_c() {
        // 0.55 m at 550 m/s = 1 ms = 48 steps at 48 kHz.
        let mut p = Pipe::new(0.55, 0.04, 550.0, 1.0 / 48_000.0, 0.0);
        let mut arrived_at = None;
        for n in 0..100 {
            if p.at_far() > 0.5 && arrived_at.is_none() {
                arrived_at = Some(n);
            }
            p.send(if n == 0 { 1.0 } else { 0.0 }, 0.0);
        }
        assert_eq!(arrived_at, Some(48));
    }

    /// Step at which an impulse sent into `p`'s near end peaks at its far end.
    fn far_peak(p: &mut Pipe, steps: usize) -> usize {
        far_peak_after(p, 0, steps)
    }
    /// [`far_peak`] with the impulse sent after `lead` steps of silence, counted from it.
    fn far_peak_after(p: &mut Pipe, lead: usize, steps: usize) -> usize {
        let mut best = (0, 0.0);
        for n in 0..lead + steps {
            let y = p.at_far();
            if n >= lead && y > best.1 {
                best = (n - lead, y);
            }
            p.send(if n == lead { 1.0 } else { 0.0 }, 0.0);
        }
        best.0
    }

    #[test]
    fn a_pipe_carries_waves_at_its_gas_speed() {
        let dt = 1.0 / 48_000.0;
        // 0.55 m at 275 m/s = 2 ms = 96 steps; at 733.3 m/s 0.75 ms = 36 steps
        let mut p = Pipe::new(0.55, 0.04, 550.0, dt, 0.0).with_room(275.0);
        p.set_sound_speed(275.0);
        assert_eq!(far_peak(&mut p, 200), 96);
        let mut p = Pipe::new(0.55, 0.04, 550.0, dt, 0.0).with_room(275.0);
        p.set_sound_speed(550.0 / 0.75 * 1.0);
        assert_eq!(far_peak(&mut p, 200), 36);
        // without room the line cannot stretch past what it was built for (48 steps)
        let mut p = Pipe::new(0.55, 0.04, 550.0, dt, 0.0);
        p.set_sound_speed(275.0);
        assert_eq!(far_peak(&mut p, 200), 48);
    }

    #[test]
    fn a_steepening_pipe_carries_waves_at_its_gas_speed() {
        // 1.2 m at 400 m/s = 3 ms = 144 steps at 48 kHz (built for 600 m/s: 96); a faint pulse
        let dt = 1.0 / 48_000.0;
        let mut p = Pipe::new(1.2, 0.04, 600.0, dt, 0.0)
            .steepening()
            .with_room(300.0);
        p.set_sound_speed(400.0);
        // once the line carries gas at the new speed (the values sent at the old delay are out)
        let n = far_peak_after(&mut p, 300, 300);
        assert!((143..=145).contains(&n), "peak at {n}");
    }

    #[test]
    fn a_pipe_under_pressure_steepens_its_waves_as_much_less() {
        // A wave steepens at k·x = (γ+1)/(2γp)·x (Msallam et al. 2000, eqs. 1–2): in gas at 3
        // atmospheres (a turbine's inlet) a pulse three times as strong steepens as one at an
        // atmosphere, so it arrives the same shape, three times as strong
        let dt = 1.0 / 96_000.0;
        let build = || Pipe::new(1.5, 0.04, 600.0, dt, 0.0).steepening();
        let (mut atm, mut high) = (build(), build());
        high.set_mean_pressure(3.0 * crate::units::ATM_PA);
        let pulse = |n: usize| 20_000.0 * (std::f64::consts::PI * n as f64 / 40.0).sin().max(0.0);
        for n in 0..600 {
            let x = if n < 40 { pulse(n) } else { 0.0 };
            let (a, b) = (atm.at_far(), high.at_far());
            assert!(
                (3.0 * a - b).abs() < 1e-6 * 20_000.0,
                "step {n}: {a} vs {b}"
            );
            atm.send(x, 0.0);
            high.send(3.0 * x, 0.0);
        }
        // and the strong pulse at an atmosphere arrives otherwise: steeper, sooner
        let (mut plain, mut high) = (build(), build());
        high.set_mean_pressure(3.0 * crate::units::ATM_PA);
        let mut diff: f64 = 0.0;
        for n in 0..600 {
            let x = if n < 40 { 3.0 * pulse(n) } else { 0.0 };
            diff = diff.max((plain.at_far() - high.at_far()).abs());
            plain.send(x, 0.0);
            high.send(x, 0.0);
        }
        assert!(diff > 1000.0, "{diff}");
    }

    #[test]
    fn a_pipe_under_pressure_flows_slower_for_its_mass_flow() {
        // The friction follows the mean Mach number ṁ/(ρcA), ρc = γp/c: at 3 atmospheres a mass
        // flow is a third of the Mach number it is at one
        let dt = 1.0 / 192_000.0;
        let a = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        let mut atm = Pipe::new(1.0, 0.05, 500.0, dt, 0.03);
        let mut high = Pipe::new(1.0, 0.05, 500.0, dt, 0.03);
        high.set_mean_pressure(3.0 * crate::units::ATM_PA);
        atm.set_mean_flow(0.2 * 283.71 * a);
        high.set_mean_flow(3.0 * 0.2 * 283.71 * a);
        assert!(
            (atm.gain - high.gain).abs() < 1e-12,
            "{} {}",
            atm.gain,
            high.gain
        );
    }

    #[test]
    fn room_alone_changes_nothing() {
        let dt = 1.0 / 96_000.0;
        let build = |room: bool, steep: bool| {
            let p = Pipe::new(0.9, 0.045, 600.0, dt, 0.03);
            let p = if steep { p.steepening() } else { p };
            if room { p.with_room(300.0) } else { p }
        };
        for steep in [false, true] {
            let (mut a, mut b) = (build(false, steep), build(true, steep));
            let mut x = 0.3_f64;
            for _ in 0..2000 {
                x = (x * 3.7 * (1.0 - x)).clamp(0.01, 0.99);
                let s = 4000.0 * (x - 0.5);
                assert_eq!(a.at_far().to_bits(), b.at_far().to_bits());
                assert_eq!(a.at_near().to_bits(), b.at_near().to_bits());
                a.send(s, -s);
                b.send(s, -s);
            }
        }
    }

    #[test]
    fn equal_areas_are_transparent_and_area_change_reflects() {
        let mut out = [0.0; 2];
        junction(&[1.0, 1.0], &[1.0, 0.0], &mut out);
        assert_eq!(out, [0.0, 1.0]);
        // A2 = 3·A1: reflection (A1 − A2)/(A1 + A2) = −0.5, transmission 1 + R = 0.5
        junction(&[1.0, 3.0], &[1.0, 0.0], &mut out);
        assert_relative_eq!(out[0], -0.5);
        assert_relative_eq!(out[1], 0.5);
        // three equal pipes: a pulse splits into −1/3 back and 2/3 into each other branch
        let mut out = [0.0; 3];
        junction(&[1.0, 1.0, 1.0], &[1.0, 0.0, 0.0], &mut out);
        assert_relative_eq!(out[0], -1.0 / 3.0);
        assert_relative_eq!(out[1], 2.0 / 3.0);
    }

    #[test]
    fn open_end_inverts_low_frequencies() {
        let mut e = OpenEnd::new(0.05, 1.0 / 192_000.0);
        let mut refl = 0.0;
        for _ in 0..2000 {
            refl = e.process(1.0).0; // DC: fully reflected, inverted
        }
        assert!(refl < -0.95 && refl > -1.0, "{refl}");
        // The source handed to the radiation stage is the outflow volume velocity (× ρc),
        // p⁺ − p_reflected: (1 + r)·p at DC. A far-field monopole is d/dt of it (voice).
        let out = e.process(1.0).1;
        assert!((1.95..=2.0).contains(&out), "{out}");
    }

    #[test]
    fn pipe_losses_scale_with_length() {
        let mut p = Pipe::new(1.0, 0.04, 500.0, 1.0 / 1000.0, 0.02);
        let mut got: f64 = 0.0;
        for n in 0..10 {
            got = got.max(p.at_far());
            p.send(if n == 0 { 1.0 } else { 0.0 }, 0.0);
        }
        assert_relative_eq!(got, 0.98, max_relative = 1e-9);
    }
}
