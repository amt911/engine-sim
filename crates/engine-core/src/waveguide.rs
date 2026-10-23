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
    delay: f64,
}

impl DelayLine {
    /// `delay` in steps, ≥ 1.
    pub fn new(delay: f64) -> Self {
        let delay = delay.max(1.0);
        Self {
            buf: vec![0.0; delay.ceil() as usize + 2],
            pos: 0,
            delay,
        }
    }
    /// The value pushed `delay` steps ago (interpolated). Call before `push` each step.
    pub fn read(&self) -> f64 {
        let n = self.buf.len();
        let i = self.delay.floor();
        let f = self.delay - i;
        let at = |k: usize| self.buf[(self.pos + n * 2 - k) % n];
        let i = i as usize;
        at(i) * (1.0 - f) + at(i + 1) * f
    }
    pub fn push(&mut self, x: f64) {
        self.buf[self.pos] = x;
        self.pos = (self.pos + 1) % self.buf.len();
    }
}

/// A uniform pipe: area, one-way delay, and a per-trip amplitude loss.
#[derive(Debug, Clone)]
pub struct Pipe {
    pub area_m2: f64,
    pub fwd: DelayLine,
    pub bwd: DelayLine,
    gain: f64,
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
        }
    }
    /// Extra per-trip amplitude factor (e.g. a muffler in this pipe).
    pub fn with_extra_gain(mut self, g: f64) -> Self {
        self.gain *= g;
        self
    }

    /// Wave arriving at the far end this step.
    pub fn at_far(&self) -> f64 {
        self.fwd.read() * self.gain
    }
    /// Wave arriving back at the near end this step.
    pub fn at_near(&self) -> f64 {
        self.bwd.read() * self.gain
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
}

impl Duct {
    /// `pipes` from the near end to the far end; at least one.
    pub fn new(pipes: Vec<Pipe>) -> Self {
        assert!(!pipes.is_empty(), "a duct needs a pipe");
        Self { pipes }
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
    /// Send this step's waves into the ends; the inner area changes scatter what arrives at
    /// them this step.
    pub fn send(&mut self, from_near: f64, from_far: f64) {
        let n = self.pipes.len();
        let mut into_near = from_near;
        for k in 0..n {
            let (back, onward) = if k + 1 < n {
                let mut out = [0.0; 2];
                junction(
                    &[self.pipes[k].area_m2, self.pipes[k + 1].area_m2],
                    &[self.pipes[k].at_far(), self.pipes[k + 1].at_near()],
                    &mut out,
                );
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
        }
    }
    /// Given the arriving wave, returns (reflected wave, outflow volume velocity × ρc =
    /// p − reflected). The far-field sound is the time derivative of the latter (monopole).
    pub fn process(&mut self, arriving: f64) -> (f64, f64) {
        self.lp += self.k * (arriving - self.lp);
        let reflected = -self.r * self.lp;
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
