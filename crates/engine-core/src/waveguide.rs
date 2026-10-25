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
}

impl DelayLine {
    /// `delay` in steps, ≥ 1.
    pub fn new(delay: f64) -> Self {
        let delay = delay.max(1.0);
        let whole = delay.floor();
        Self {
            buf: vec![0.0; delay.ceil() as usize + 2],
            pos: 0,
            whole: whole as usize,
            frac: delay - whole,
        }
    }
    /// The value pushed `delay` steps ago (interpolated). Call before `push` each step.
    pub fn read(&self) -> f64 {
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
        if self.whole < 2 {
            return None;
        }
        let n = self.buf.len();
        let back = |k: usize| {
            let j = self.pos + n - k;
            self.buf[if j >= n { j - n } else { j }]
        };
        Some(back(self.whole - 1) * (1.0 - self.frac) + back(self.whole) * self.frac)
    }
    pub fn push(&mut self, x: f64) {
        self.buf[self.pos] = x;
        self.pos += 1;
        if self.pos == self.buf.len() {
            self.pos = 0;
        }
    }
}

/// A delay line whose every sample keeps its own travel time: a simple wave's crests outrun its
/// troughs. Along a characteristic of a simple wave the sample travels at u + c = c₀(1 + ε·p)
/// (Earnshaw; Hamilton & Blackstock, *Nonlinear Acoustics*, ch. 4), so a sample of p pushed at
/// step m reaches the far end at m + D/(1 + ε·p).
///
/// Where a later, higher sample would arrive before an earlier one the profile has folded into
/// a shock (Hirschberg et al., "Shock waves in trombones", JASA 99, 1996: x_s =
/// 2γp₀c/((γ+1)·max ∂p/∂t)). Weak-shock theory places the shock by the equal-area rule (it
/// travels at the mean of the speeds ahead and behind it; Salomons, J. Theor. Comp. Acoust.
/// 2025, §2): over the folded stretch from its foot to its head, ∫(t − t_s)·dp = 0, so
/// t_s = ∫t·dp / (p_head − p_foot) along the folded profile. The stretch's inner samples are
/// gone; the foot and the head arrive together at t_s, a jump. A shock that reaches the sample
/// ahead of it (or an older shock) takes it in. The fold's energy is what the shock dissipates.
///
/// Linear interpolation in arrival time: with no amplitude it is [`DelayLine`]'s.
#[derive(Debug, Clone)]
pub struct WarpLine {
    /// The samples still travelling, oldest first.
    samples: std::collections::VecDeque<Sample>,
    /// The shocks still travelling, oldest first.
    shocks: std::collections::VecDeque<Shock>,
    delay: f64,
    eps: f64,
    /// Step of the next push (and of the next read: reads come before pushes).
    now: f64,
    pushed: u64,
    /// Sum of the values in `samples` (for [`WarpLine::mean`]).
    sum: f64,
}

#[derive(Debug, Clone, Copy)]
struct Sample {
    x: f64,
    /// When it arrives (a shock's time for its foot and head).
    t: f64,
    /// When it would arrive on its own characteristic.
    t0: f64,
    id: u64,
}

#[derive(Debug, Clone, Copy)]
struct Shock {
    foot: u64,
    head: u64,
    /// ∫t·dp along the folded profile from the foot to the head.
    moment: f64,
}

/// The fastest and slowest a sample may travel against the small-signal speed: beyond these
/// the simple-wave speed is meaningless (the waves are clamped far inside them anyway).
const WARP_FASTEST: f64 = 2.0;
const WARP_SLOWEST: f64 = 0.5;

impl WarpLine {
    /// `delay`: steps at the small-signal speed (≥ 1); `eps`: [`nonlinearity`] of the gas.
    pub fn new(delay: f64, eps: f64) -> Self {
        let delay = delay.max(1.0);
        let n = (delay / WARP_SLOWEST).ceil() as usize + 4;
        let mut samples = std::collections::VecDeque::with_capacity(n);
        // the line starts full of silence, as a plain one
        for m in -(delay.ceil() as i64 + 2)..0 {
            let t = m as f64 + delay;
            samples.push_back(Sample {
                x: 0.0,
                t,
                t0: t,
                id: 0,
            });
        }
        Self {
            samples,
            shocks: std::collections::VecDeque::with_capacity(n),
            delay,
            eps,
            now: 0.0,
            pushed: 1,
            sum: 0.0,
        }
    }

    /// The signal arriving at step `t`, from the samples now travelling.
    fn at(&self, t: f64) -> f64 {
        // the last sample arrived by `t` and the first after it
        let mut before: Option<(f64, f64)> = None;
        for s in &self.samples {
            if s.t <= t {
                before = Some((s.x, s.t));
            } else {
                return match before {
                    Some((x, tb)) => x + (s.x - x) * (t - tb) / (s.t - tb),
                    None => 0.0,
                };
            }
        }
        before.map_or(0.0, |b| b.0)
    }

    /// The value arriving now. Call before `push` each step.
    pub fn read(&self) -> f64 {
        self.at(self.now)
    }

    /// What [`WarpLine::read`] will return after the next `push`, before it: known when even
    /// the fastest sample takes more than a step (`None` otherwise).
    pub fn read_next(&self) -> Option<f64> {
        if self.delay / WARP_FASTEST <= 1.0 {
            return None;
        }
        Some(self.at(self.now + 1.0))
    }

    /// Push a sample of pressure `x`: it travels at c₀(1 + ε·x).
    pub fn push(&mut self, x: f64) {
        self.push_at(x, 1.0 + self.eps * x);
    }

    /// The mean of the values still travelling (zero for an empty line).
    pub fn mean(&self) -> f64 {
        if self.samples.is_empty() {
            0.0
        } else {
            self.sum / self.samples.len() as f64
        }
    }

    /// Push a sample `x` that travels at `speed` times the small-signal speed (a Riemann
    /// variable's characteristic slope, [`crate::riemann::slopes`]).
    pub fn push_at(&mut self, x: f64, speed: f64) {
        let speed = speed.clamp(WARP_SLOWEST, WARP_FASTEST);
        let t0 = self.now + (self.delay / speed).max(1.0);
        let id = self.pushed;
        self.pushed += 1;
        let new = Sample { x, t: t0, t0, id };
        self.sum += x;
        let Some(&last) = self.samples.back() else {
            self.samples.push_back(new);
            self.now += 1.0;
            return;
        };
        if t0 >= last.t {
            self.samples.push_back(new);
        } else {
            // it overtakes: it heads the shock the last sample was in, or a new one
            let mut shock = match self.shocks.back() {
                Some(s) if s.head == last.id => {
                    let s = *s;
                    self.shocks.pop_back();
                    // the old head is now inside the fold (unless it was also the foot)
                    if s.foot != last.id {
                        self.samples.pop_back();
                        self.sum -= last.x;
                    }
                    s
                }
                _ => Shock {
                    foot: last.id,
                    head: last.id,
                    moment: 0.0,
                },
            };
            shock.moment += 0.5 * (last.t0 + t0) * (x - last.x);
            shock.head = id;
            self.samples.push_back(new);
            self.settle(shock);
        }
        self.now += 1.0;
        // drop what has fully arrived: keep one at or before the next read for the interpolation
        while self.samples.len() > 2 && self.samples[1].t <= self.now {
            if let Some(gone) = self.samples.pop_front() {
                self.sum -= gone.x;
            }
        }
        let front = self.samples.front().map_or(0, |s| s.id);
        while self.shocks.front().is_some_and(|s| s.foot < front) {
            self.shocks.pop_front();
        }
    }

    /// Remove the sample at `i`, keeping the running sum.
    fn drop_at(&mut self, i: usize) {
        if let Some(gone) = self.samples.remove(i) {
            self.sum -= gone.x;
        }
    }

    /// Place `shock` (its head the last sample, its foot just before it in the line) by the
    /// equal-area rule, taking in whatever ahead of it it now reaches.
    fn settle(&mut self, mut shock: Shock) {
        loop {
            let n = self.samples.len();
            let (foot, head) = (self.samples[n - 2], self.samples[n - 1]);
            let jump = head.x - foot.x;
            let ts = if jump > 1e-12 {
                shock.moment / jump
            } else {
                0.5 * (foot.t0 + head.t0)
            };
            self.samples[n - 2].t = ts;
            self.samples[n - 1].t = ts;
            // does it reach the sample ahead of its foot?
            if n < 3 || self.samples[n - 3].t <= ts {
                self.shocks.push_back(shock);
                return;
            }
            let ahead = self.samples[n - 3];
            match self.shocks.back() {
                Some(s) if s.head == ahead.id => {
                    // two shocks meet: one shock over both folds
                    let older = *s;
                    self.shocks.pop_back();
                    shock.moment += older.moment + 0.5 * (ahead.t0 + foot.t0) * (foot.x - ahead.x);
                    shock.foot = older.foot;
                    // the older head and this foot are inside the fold now
                    self.drop_at(n - 2);
                    if older.foot != ahead.id {
                        self.drop_at(n - 3);
                    }
                }
                _ => {
                    shock.moment += 0.5 * (ahead.t0 + foot.t0) * (foot.x - ahead.x);
                    shock.foot = ahead.id;
                    self.drop_at(n - 2);
                }
            }
        }
    }
}

/// The coefficient of nonlinearity per pascal of a gas at `p0`: a simple wave's crest of p
/// travels at c₀(1 + ε·p).
pub fn nonlinearity(gamma: f64, p0: f64) -> f64 {
    // u + c = c₀ + (γ+1)/2·u for a simple wave, u = p/(ρ₀c₀) and ρ₀c₀² = γp₀
    (gamma + 1.0) / (2.0 * gamma * p0)
}

/// One direction of a pipe: a plain delay (small signals) or a warped one (finite amplitude).
#[derive(Debug, Clone)]
pub enum Line {
    Plain(DelayLine),
    Warped(WarpLine),
}

impl Line {
    pub fn read(&self) -> f64 {
        match self {
            Line::Plain(d) => d.read(),
            Line::Warped(w) => w.read(),
        }
    }
    pub fn read_next(&self) -> Option<f64> {
        match self {
            Line::Plain(d) => d.read_next(),
            Line::Warped(w) => w.read_next(),
        }
    }
    pub fn push(&mut self, x: f64) {
        match self {
            Line::Plain(d) => d.push(x),
            Line::Warped(w) => w.push(x),
        }
    }
}

/// A uniform pipe: area, one-way delay, and a per-trip amplitude loss.
#[derive(Debug, Clone)]
pub struct Pipe {
    pub area_m2: f64,
    pub fwd: Line,
    pub bwd: Line,
    gain: f64,
    /// Per-trip factor without flow (`gain` is it times the flow's friction loss).
    still_gain: f64,
    length_m: f64,
    diameter_m: f64,
    sound_speed: f64,
}

impl Pipe {
    /// `loss_per_m`: fractional amplitude loss per metre travelled (wall friction, heat).
    pub fn new(length_m: f64, diameter_m: f64, sound_speed: f64, dt: f64, loss_per_m: f64) -> Self {
        let delay = length_m / sound_speed / dt;
        Self {
            area_m2: std::f64::consts::PI * 0.25 * diameter_m * diameter_m,
            fwd: Line::Plain(DelayLine::new(delay)),
            bwd: Line::Plain(DelayLine::new(delay)),
            gain: (1.0 - loss_per_m).powf(length_m),
            still_gain: (1.0 - loss_per_m).powf(length_m),
            length_m,
            diameter_m,
            sound_speed,
        }
    }
    /// Finite-amplitude propagation ([`WarpLine`]) in both directions, for gas at `p0` with
    /// `gamma`: each wave steepens on its own as it travels (the waves crossing it do not bend
    /// it — the simple-wave approximation; their effect on the speed is of the same order).
    pub fn warped(mut self, dt: f64, gamma: f64, p0: f64) -> Self {
        let delay = self.length_m / self.sound_speed / dt;
        let eps = nonlinearity(gamma, p0);
        self.fwd = Line::Warped(WarpLine::new(delay, eps));
        self.bwd = Line::Warped(WarpLine::new(delay, eps));
        self
    }

    /// Extra per-trip amplitude factor (e.g. a muffler in this pipe).
    pub fn with_extra_gain(mut self, g: f64) -> Self {
        self.gain *= g;
        self.still_gain *= g;
        self
    }

    /// The pipe's mean mass flow, kg/s, of gas at its sound speed and an atmosphere: turbulent
    /// wall friction, ½ρu|u|·f/D per unit volume, linearized about the mean velocity U damps a
    /// wave by α = f·M/D per metre (M = U/c; Darcy f of [`crate::exhaust_fv::PIPE_FRICTION`]):
    /// the faster the gas, the more a pipe's resonances are worn down.
    pub fn set_mean_flow(&mut self, mass_flow: f64) {
        let rho_c = crate::gas::GAMMA * crate::units::ATM_PA / self.sound_speed;
        let mach = (mass_flow / (rho_c * self.area_m2)).abs();
        let alpha = crate::exhaust_fv::PIPE_FRICTION * mach / self.diameter_m;
        self.gain = self.still_gain * (-alpha * self.length_m).exp();
    }

    /// Wave arriving at the far end this step.
    pub fn at_far(&self) -> f64 {
        self.fwd.read() * self.gain
    }
    /// Wave arriving back at the near end this step.
    pub fn at_near(&self) -> f64 {
        self.bwd.read() * self.gain
    }
    /// [`Pipe::at_near`] as it will read after the next `send` (at the current gain), if the
    /// pipe is at least two steps long.
    pub fn at_near_next(&self) -> Option<f64> {
        self.bwd.read_next().map(|x| x * self.gain)
    }
    /// Send this step's outgoing waves: `from_near` into `fwd`, `from_far` into `bwd`.
    pub fn send(&mut self, from_near: f64, from_far: f64) {
        self.fwd.push(from_near);
        self.bwd.push(from_far);
    }
}

/// Pipes of different areas in series. Each change of area scatters a plane wave (the
/// two-port pressure junction below), which is how a muffler's chambers reflect sound back
/// (Munjal, *Acoustics of Ducts and Mufflers*, ch. 3) and how a stepped pipe shapes it.
#[derive(Debug, Clone)]
pub struct Duct {
    pipes: Vec<Pipe>,
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
    pub fn set_mean_flow(&mut self, mass_flow: f64) {
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
                    Some(Node::Series(r, rho_c)) => {
                        // Q = 2(p₁ − p₂)/(z₁ + z₂ + R), z = ρc/A: each side loses z·Q
                        let (z1, z2) = (*rho_c / areas[0], *rho_c / areas[1]);
                        let q = 2.0 * (incoming[0] - incoming[1]) / (z1 + z2 + *r);
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
        // M = 0.2 in a 50 mm pipe: α = 0.025·0.2/0.05 = 0.1 Np/m, so 2 m keep e^(−0.2) = 0.818731
        // on top of the still loss (1 − 0.03)² = 0.9409
        let dt = 1.0 / 192_000.0;
        let a = std::f64::consts::PI * 0.25 * 0.05 * 0.05;
        let mut p = Pipe::new(2.0, 0.05, 500.0, dt, 0.03);
        p.set_mean_flow(0.2 * 283.71 * a);
        assert!(
            (p.gain / 0.9409 - 0.818_731).abs() < 1e-5,
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

    /// Amplitude of harmonic `k` of `f` in `x` (a whole number of periods).
    fn harmonic(x: &[f64], f: f64, dt: f64, k: usize) -> f64 {
        let (mut re, mut im) = (0.0, 0.0);
        for (i, v) in x.iter().enumerate() {
            let w = 2.0 * std::f64::consts::PI * f * k as f64 * i as f64 * dt;
            re += v * w.cos();
            im += v * w.sin();
        }
        2.0 * (re * re + im * im).sqrt() / x.len() as f64
    }

    /// A sine of `amp` Pa at `f` through a warped line of `delay` steps; the steady output.
    fn warped_sine(delay: f64, amp: f64, f: f64, dt: f64) -> Vec<f64> {
        let mut line = WarpLine::new(delay, nonlinearity(crate::gas::GAMMA, crate::units::ATM_PA));
        let period = (1.0 / (f * dt)).round() as usize;
        let mut out = Vec::new();
        for i in 0..(delay as usize * 3 + 40 * period) {
            let y = line.read();
            line.push(amp * (2.0 * std::f64::consts::PI * f * i as f64 * dt).sin());
            if i >= delay as usize * 3 {
                out.push(y);
            }
        }
        out.truncate(out.len() / period * period);
        out
    }

    #[test]
    fn a_loud_sine_steepens_as_fubini_solved_it() {
        // Fubini's solution before the shock (Hamilton & Blackstock, *Nonlinear Acoustics*,
        // ch. 4): at σ = x/x̄ the harmonics are B_n = 2·J_n(nσ)/(nσ) of the source amplitude,
        // x̄ = 1/(ε·p_a·ω) in time of flight with ε = (γ+1)/(2γ·p₀). At σ = 0.5, with J₁(0.5) =
        // 0.242268, J₂(1) = 0.114903, J₃(1.5) = 0.060964 (Abramowitz & Stegun, table 9.1):
        // B₁ = 0.969072, B₂ = 0.229806, B₃ = 0.081285.
        let dt = 1.0 / 192_000.0;
        let (amp, f) = (5000.0, 500.0);
        let eps = (1.4 + 1.0) / (2.0 * 1.4 * 101_325.0);
        let delay = 0.5 / (eps * amp * 2.0 * std::f64::consts::PI * f * dt);
        let y = warped_sine(delay, amp, f, dt);
        let b: Vec<f64> = (1..=3).map(|k| harmonic(&y, f, dt, k) / amp).collect();
        assert!((b[0] / 0.969_072 - 1.0).abs() < 0.01, "B1 {}", b[0]);
        assert!((b[1] / 0.229_806 - 1.0).abs() < 0.03, "B2 {}", b[1]);
        assert!((b[2] / 0.081_285 - 1.0).abs() < 0.05, "B3 {}", b[2]);
    }

    #[test]
    fn far_past_the_shock_a_sine_is_a_decaying_sawtooth() {
        // Weak-shock theory (Blackstock, JASA 39, 1966; Hamilton & Blackstock, *Nonlinear
        // Acoustics*, ch. 4): well past the shock distance a sine is a sawtooth whose harmonics
        // are B_n = 2/(n·(1 + σ)) of the source amplitude — the shocks dissipate it. At σ = 5:
        // B₁ = 1/3, B₂ = 1/6. A shock that kept its full height would stay near B₁ = 2/π.
        let dt = 1.0 / 192_000.0;
        let (amp, f) = (5000.0, 500.0);
        let eps = (1.4 + 1.0) / (2.0 * 1.4 * 101_325.0);
        let delay = 5.0 / (eps * amp * 2.0 * std::f64::consts::PI * f * dt);
        let y = warped_sine(delay, amp, f, dt);
        let b1 = harmonic(&y, f, dt, 1) / amp;
        let b2 = harmonic(&y, f, dt, 2) / amp;
        assert!((b1 / (1.0 / 3.0) - 1.0).abs() < 0.05, "B1 {b1}");
        assert!((b2 / (1.0 / 6.0) - 1.0).abs() < 0.08, "B2 {b2}");
    }

    #[test]
    fn past_the_shock_distance_the_front_jumps() {
        // at σ = 2 the crest has overtaken the front: the rise takes a step or two instead of
        // the sine's 0.016 of the amplitude per step
        let dt = 1.0 / 192_000.0;
        let (amp, f) = (5000.0, 500.0);
        let eps = (1.4 + 1.0) / (2.0 * 1.4 * 101_325.0);
        let delay = 2.0 / (eps * amp * 2.0 * std::f64::consts::PI * f * dt);
        let y = warped_sine(delay, amp, f, dt);
        let jump = y.windows(2).map(|w| w[1] - w[0]).fold(0.0, f64::max);
        assert!(jump > 0.5 * amp, "steepest rise {jump}");
        assert!(y.iter().all(|v| v.abs() <= amp * 1.0001), "no overshoot");
    }

    #[test]
    fn a_faint_wave_travels_as_in_a_plain_line() {
        let mut plain = DelayLine::new(37.3);
        let mut warped = WarpLine::new(37.3, nonlinearity(1.4, 101_325.0));
        for k in 0..400 {
            let x = 1e-3 * (k as f64 * 0.21).sin();
            assert!((plain.read() - warped.read()).abs() < 1e-9, "step {k}");
            plain.push(x);
            warped.push(x);
        }
    }

    #[test]
    fn a_warped_line_knows_its_next_read_a_step_early() {
        let mut d = WarpLine::new(24.0, nonlinearity(1.4, 101_325.0));
        for k in 0..200 {
            let next = d.read_next().unwrap();
            d.push(30_000.0 * (k as f64 * 0.37).sin());
            assert_eq!(d.read(), next, "step {k}");
        }
        assert_eq!(WarpLine::new(1.5, 1e-5).read_next(), None);
    }

    #[test]
    fn the_nonlinearity_by_hand() {
        // (γ+1)/(2γ·p₀) for air at one atmosphere: 2.4/(2.8·101 325) = 8.4593e-6 per Pa
        assert!((nonlinearity(1.4, 101_325.0) / 8.4593e-6 - 1.0).abs() < 1e-4);
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
    fn a_flowing_pipe_wears_its_waves_down() {
        // D = 50 mm, 1 m, c = 500 m/s (ρc = γp/c = 283.71): 0.0557 kg/s is M = 0.2; f = 0.025
        // → α = f·M/D = 0.1 Np/m: a wave keeps e^(−0.1) = 0.904837 of itself, on top of the
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
            (p.gain / still - 0.904_837).abs() < 1e-5,
            "{}",
            p.gain / still
        );
        // and the flow's direction does not matter
        p.set_mean_flow(-0.2 * 283.71 * a);
        assert!((p.gain / still - 0.904_837).abs() < 1e-5);
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
        // a mean flow of M = 0.2 through both pipes: α = 0.025·0.2/0.05 = 0.1 Np/m, so a pulse
        // crossing the 2 m keeps e^(−0.2) = 0.818731 (the resistance aside: here a plain duct)
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
        assert!((got - 0.818_731).abs() < 1e-5, "{got}");
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
