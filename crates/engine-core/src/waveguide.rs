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
    pub fn push(&mut self, x: f64) {
        self.buf[self.pos] = x;
        self.pos += 1;
        if self.pos == self.buf.len() {
            self.pos = 0;
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
    sound_speed: f64,
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
        }
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
    /// What the wall draws at each inner node (none where it is hard).
    shunts: Vec<Option<Shunt>>,
}

/// A wall that breathes at a node of a duct: the packing behind a perforate, a pocket of pore
/// gas (compliance C) behind the fibres' resistance R, drawing a volume flow q = Y(s)·p with
/// Y(s) = sC/(1 + sRC) — the lumped element of the locally reacting lining (Munjal ch. 6;
/// [`crate::lining`] for R and C). Discretized by the bilinear transform; the node solves it
/// with the pipes' waves at once, so it is stable for any element.
#[derive(Debug, Clone)]
pub struct Shunt {
    /// q[n] = b0·p[n] + b1·p[n−1] − a1·q[n−1], in units of area × pressure (×ρc).
    b0: f64,
    b1: f64,
    a1: f64,
    p_last: f64,
    q_last: f64,
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
        Self {
            b0: k * a / den,
            b1: -k * a / den,
            a1: (1.0 - a * tau) / den,
            p_last: 0.0,
            q_last: 0.0,
        }
    }

    /// Node pressure where pipes of `areas` meet with waves `incoming`, this shunt drawing its
    /// flow: Σ Aᵢ(2pᵢ − p) = q(p). Writes the outgoing waves; advances the shunt.
    fn node(&mut self, areas: &[f64; 2], incoming: &[f64; 2], outgoing: &mut [f64; 2]) {
        let hist = self.b1 * self.p_last - self.a1 * self.q_last;
        let pj = (2.0 * (areas[0] * incoming[0] + areas[1] * incoming[1]) - hist)
            / (areas[0] + areas[1] + self.b0);
        self.q_last = self.b0 * pj + hist;
        self.p_last = pj;
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
            shunts: Vec::new(),
        }
    }
    /// A duct whose every inner node breathes through a copy of `shunt` (a lined passage
    /// cut into short pipes: the lumped transmission line).
    pub fn with_shunts(pipes: Vec<Pipe>, shunt: Shunt) -> Self {
        let n = pipes.len().saturating_sub(1);
        Self {
            shunts: vec![Some(shunt); n],
            ..Self::new(pipes)
        }
    }
    /// This duct, then `next` after it (a hard node between them).
    pub fn then(mut self, next: Duct) -> Self {
        let n = self.pipes.len();
        self.shunts.resize(n - 1, None);
        self.shunts.push(None);
        let mut rest = next.shunts;
        rest.resize(next.pipes.len() - 1, None);
        self.shunts.extend(rest);
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
                match self.shunts.get_mut(k) {
                    Some(Some(s)) => s.node(&areas, &incoming, &mut out),
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
