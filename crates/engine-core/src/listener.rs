//! Where the listener stands: each tail pipe reaches them with its own delay and spreading
//! loss. Two tails a car's width apart, heard from off the axis, arrive out of phase at the
//! low orders. That is why a cross-plane V8's uneven bank pulses are heard as a burble; summed
//! at one point they cancel exactly into even 90° firing.
//!
//! The output is the sound pressure at the listener, in pascals: each tail is a monopole of
//! volume flow Q, p = ρ₀/(4πr)·dQ/dt(t − r/c₀) (Kinsler et al., *Fundamentals of Acoustics*,
//! §7.2; valid while the pipe is small against the wavelength, ka < 1).

/// Speed of sound in the ambient air between the car and the listener, m/s (20 °C).
const C_AMBIENT: f64 = 343.0;
/// Density of that air, kg/m³ (20 °C, 1 atm).
const RHO_AMBIENT: f64 = 1.204;

/// A street's façades: `width_m` apart (the car in the middle), each reflecting a share
/// `reflection` of the sound pressure (ISO 9613-2, Table 4: 1 for flat hard walls, 0.8 for
/// building walls with windows and small installations).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Street {
    pub width_m: f64,
    pub reflection: f64,
}

/// One reflected path: its delay and monopole gain, and the air's absorption along it as a
/// one-pole low-pass at [`air_corner_hz`] of its length.
#[derive(Debug, Clone)]
struct Image {
    delay: usize,
    gain: f64,
    /// One-pole coefficient, its state, and the last signal it heard (the monopole hears
    /// changes).
    k: f64,
    state: f64,
    last: f64,
}

impl Image {
    fn new(delay: usize, gain: f64, path_m: f64, dt: f64) -> Self {
        // a path whose air keeps everything audible (corner at the 20 kHz cap) passes untouched
        let fc = air_corner_hz(path_m);
        let k = if fc >= 20_000.0 {
            1.0
        } else {
            1.0 - (-2.0 * std::f64::consts::PI * fc.min(0.45 / dt) * dt).exp()
        };
        Self {
            delay,
            gain,
            k,
            state: 0.0,
            last: 0.0,
        }
    }
    /// The pressure this path adds for the tail signal `x` it hears now.
    fn hear(&mut self, x: f64) -> f64 {
        self.state += (x - self.state) * self.k;
        let p = self.gain * (self.state - self.last);
        self.last = self.state;
        p
    }
}

/// Atmospheric absorption of ISO 9613-1 (eqs. 3–5) at 20 °C, 70 % RH, 1 atm, dB/m, at octave
/// centres: (Hz, α).
const AIR_ABSORPTION: [(f64, f64); 8] = [
    (125.0, 0.0003),
    (250.0, 0.0011),
    (500.0, 0.0028),
    (1000.0, 0.0050),
    (2000.0, 0.0090),
    (4000.0, 0.0231),
    (8000.0, 0.0776),
    (16000.0, 0.2811),
];

/// The frequency at which a path of `path_m` metres of air takes 3 dB off (α(f)·r = 3 dB),
/// interpolated log-log in [`AIR_ABSORPTION`]; capped at 20 kHz: the corner of the one-pole
/// that stands for the air on that path.
pub fn air_corner_hz(path_m: f64) -> f64 {
    let need = 3.0 / path_m.max(1e-3);
    let mut prev = AIR_ABSORPTION[0];
    if need <= prev.1 {
        return prev.0;
    }
    for &(f, a) in &AIR_ABSORPTION[1..] {
        if need <= a {
            let t = (need.ln() - prev.1.ln()) / (a.ln() - prev.1.ln());
            return (prev.0.ln() + t * (f.ln() - prev.0.ln()))
                .exp()
                .min(20_000.0);
        }
        prev = (f, a);
    }
    20_000.0
}

/// The tails spread evenly across `spacing_m` at the rear (x across the car, y = 0), the
/// listener at `distance_m` from their centre, `angle_deg` off the car's axis (0 = straight
/// behind). Free-field monopoles, each heard after its own path; only the delay differences
/// are kept (the common path is dropped).
#[derive(Debug, Clone)]
pub struct Listener {
    distance: Vec<f64>,
    delay: Vec<usize>,
    /// ρ₀/(4πr) · A/(ρc) / dt per tail: turns the tail signal's step change into pascals.
    gain: Vec<f64>,
    lines: Vec<Vec<f64>>,
    last: Vec<f64>,
    /// Per tail, its images (the ground's, the façades'): each a reflected path.
    images: Vec<Vec<Image>>,
    pos: usize,
    /// For a signal radiated from all the tails alike (at their mean gain, no delay).
    shared_last: f64,
    dt: f64,
    /// The tails' radiation above ka ≈ 2: a one-pole low-pass on dQ/dt at ω_c = 2c/a, and its
    /// state.
    radiation_k: f64,
    radiated: f64,
    /// The same for the intake mouths ([`Listener::radiate_mouth_at`]): their area and corner.
    mouth_area: f64,
    mouth_k: f64,
    /// Where the ear is: (across, along the car from the tails, height above the tails) and the
    /// nearest tail's distance, m.
    ear: (f64, f64, f64, f64),
    /// The engine's own sources (a bank's side of the block with its intake mouths; a common
    /// throttle's mouth), each heard from where it sits. One at the tails' mean distance until
    /// [`Listener::with_engine_at`] places them.
    engine: Vec<EngineSource>,
}

/// One place on the engine that radiates (a bank, a common mouth): its distance to the
/// listener, m, the delay line that holds it back by its path beyond the nearest tail's, and
/// the states of its monopole ([`Listener::radiate_flow_at`]) and its mouth's radiation corner.
#[derive(Debug, Clone)]
struct EngineSource {
    distance: f64,
    line: Vec<f64>,
    pos: usize,
    flow_last: f64,
    mouth_radiated: f64,
    /// Where it sits: across the car's axis and ahead of the tails, m.
    at: (f64, f64),
    /// The car's body between it and the ear, if it screens it ([`Listener::with_body`]).
    barrier: Option<Barrier>,
}

/// A screen's diffraction loss (Maekawa 1968; ISO 9613-2:1996, eq. 14, C₂ = 20):
/// D = 10·log10(3 + 20N), N = 2δ/λ for a path δ longer over the edge than straight through.
/// |H|² = 1/(3·(1 + f/f_c)), f_c = 3c/(40δ): flat 4.8 dB, then 3 dB per octave, built as
/// first-order shelves two octaves apart (each down 6 dB over one octave), within 1.2 dB of the
/// curve above f_c/5.
#[derive(Debug, Clone)]
struct Barrier {
    /// Per shelf: (b0, b1, a1) and its last input and output.
    shelves: Vec<([f64; 3], f64, f64)>,
}

impl Barrier {
    fn new(delta_m: f64, dt: f64) -> Self {
        let fc = 3.0 * C_AMBIENT / (40.0 * delta_m.max(1e-6));
        let k = 2.0 / dt;
        let nyquist = 0.5 / dt;
        let mut shelves = Vec::new();
        let mut p = 1.1 * fc;
        while p < nyquist {
            let (wp, wz) = (
                2.0 * std::f64::consts::PI * p,
                2.0 * std::f64::consts::PI * 2.0 * p,
            );
            // H(s) = (1 + s/ωz)/(1 + s/ωp), bilinear
            let (n0, n1, d0, d1) = (1.0 + k / wz, 1.0 - k / wz, 1.0 + k / wp, 1.0 - k / wp);
            shelves.push(([n0 / d0, n1 / d0, d1 / d0], 0.0, 0.0));
            p *= 4.0;
        }
        Self { shelves }
    }

    fn process(&mut self, x: f64) -> f64 {
        let mut y = x / 3f64.sqrt();
        for (c, xl, yl) in self.shelves.iter_mut() {
            let out = c[0] * y + c[1] * *xl - c[2] * *yl;
            (*xl, *yl) = (y, out);
            y = out;
        }
        y
    }
}

impl EngineSource {
    fn at(distance: f64, delay_steps: usize) -> Self {
        Self {
            distance,
            line: vec![0.0; delay_steps + 1],
            pos: 0,
            flow_last: 0.0,
            mouth_radiated: 0.0,
            at: (0.0, 0.0),
            barrier: None,
        }
    }
}

impl Listener {
    /// `tail_area_m2`: cross-section of each tail pipe; the tails' signals are ρc·u at their
    /// outlets (ρc = [`crate::exhaust::tail_impedance`]), so Q = A·signal/ρc.
    pub fn new(
        tails: usize,
        spacing_m: f64,
        distance_m: f64,
        angle_deg: f64,
        tail_area_m2: f64,
        dt: f64,
        ground: Option<(f64, f64)>,
    ) -> Self {
        Self::with_street(
            tails,
            spacing_m,
            distance_m,
            angle_deg,
            tail_area_m2,
            dt,
            ground,
            None,
        )
    }

    /// [`Listener::new`] in a street: façades parallel to the car's axis, `street.width_m`
    /// apart with the car in the middle, each returning the tails from their images across it
    /// (first and second bounces, each also off the ground when it reflects), weakened by the
    /// wall's reflection factor per bounce and by the air along the longer path (ISO 9613-1).
    #[allow(clippy::too_many_arguments)]
    pub fn with_street(
        tails: usize,
        spacing_m: f64,
        distance_m: f64,
        angle_deg: f64,
        tail_area_m2: f64,
        dt: f64,
        ground: Option<(f64, f64)>,
        street: Option<Street>,
    ) -> Self {
        let (heights, image) = match ground {
            Some(h) => (h, true),
            None => ((0.0, 0.0), false),
        };
        Self::with_heights(
            tails,
            spacing_m,
            distance_m,
            angle_deg,
            tail_area_m2,
            dt,
            heights,
            image,
            street,
        )
    }

    /// [`Listener::with_street`] with the tails and the ear at `heights` (tail, ear) above the
    /// ground whether or not the ground reflects: the direct path climbs the ear's height over
    /// the tails in the free field too, and `ground_image` adds the ground's image.
    #[allow(clippy::too_many_arguments)]
    pub fn with_heights(
        tails: usize,
        spacing_m: f64,
        distance_m: f64,
        angle_deg: f64,
        tail_area_m2: f64,
        dt: f64,
        heights: (f64, f64),
        ground_image: bool,
        street: Option<Street>,
    ) -> Self {
        let n = tails.max(1);
        let (lx, ly) = {
            let a = angle_deg.to_radians();
            (distance_m * a.sin(), -distance_m * a.cos())
        };
        // heights above the ground (tail, ear): the direct path climbs (h_l − h_s), the
        // ground's image path (h_l + h_s); free field has neither
        let (hs, hl) = heights;
        let (rise, image_rise) = (hl - hs, ground_image.then_some(hl + hs));
        let xs: Vec<f64> = (0..n)
            .map(|k| {
                if n == 1 {
                    0.0
                } else {
                    -0.5 * spacing_m + spacing_m * k as f64 / (n - 1) as f64
                }
            })
            .collect();
        let path = |x: f64, h: f64| ((lx - x).powi(2) + ly * ly + h * h).sqrt().max(0.1);
        let r: Vec<f64> = xs.iter().map(|&x| path(x, rise)).collect();
        let r_min = r.iter().copied().fold(f64::INFINITY, f64::min);
        let steps = |ri: f64| ((ri - r_min) / C_AMBIENT / dt).round() as usize;
        let delay: Vec<usize> = r.iter().map(|&ri| steps(ri)).collect();
        let to_q = tail_area_m2 / crate::exhaust::tail_impedance();
        let monopole = |ri: f64| RHO_AMBIENT / (4.0 * std::f64::consts::PI * ri) * to_q / dt;
        let gain = r.iter().map(|&ri| monopole(ri)).collect();
        // Images of each tail at (x', height): the rigid ground (asphalt: reflection ≈ 1 at these
        // frequencies, ISO 9613-2) below it; with a street, its mirror across each façade
        // (x' = ±W − x) and across both (x' = x ± 2W), each with and without the ground.
        let images: Vec<Vec<Image>> = xs
            .iter()
            .map(|&x| {
                let mut out = Vec::new();
                let mut add = |xi: f64, h: f64, factor: f64| {
                    let ri = path(xi, h);
                    out.push(Image::new(steps(ri), factor * monopole(ri), ri, dt));
                };
                if let Some(h) = image_rise {
                    add(x, h, 1.0);
                }
                if let Some(st) = street {
                    let w = st.width_m;
                    let rho = st.reflection;
                    for (xi, f) in [
                        (w - x, rho),
                        (-w - x, rho),
                        (2.0 * w + x, rho * rho),
                        (-2.0 * w + x, rho * rho),
                    ] {
                        add(xi, rise, f);
                        if let Some(h) = image_rise {
                            add(xi, h, f);
                        }
                    }
                }
                out
            })
            .collect();
        let delay_max = delay
            .iter()
            .copied()
            .chain(images.iter().flatten().map(|i| i.delay))
            .max()
            .unwrap_or(0);
        let len = delay_max + 1;
        let mean_distance = r.iter().sum::<f64>() / r.len() as f64;
        // the outlets' radiation corner ([`Listener::radiate`]): ka = 2
        let radius = (tail_area_m2 / std::f64::consts::PI).sqrt().max(1e-3);
        let omega_c = 2.0 * C_AMBIENT / radius;
        let radiation_k = 1.0 - (-omega_c * dt).exp();
        Self {
            distance: r,
            delay,
            gain,
            lines: vec![vec![0.0; len]; n],
            last: vec![0.0; n],
            images,
            pos: 0,
            shared_last: 0.0,
            dt,
            radiation_k,
            radiated: 0.0,
            mouth_area: 0.0,
            mouth_k: 1.0,
            engine: vec![EngineSource::at(mean_distance, 0)],
            ear: (lx, ly, rise, r_min),
        }
    }

    /// The engine (intake mouth, block) as one source `ahead_m` forward of the tails along the
    /// car's axis: heard from there, farther ([`Listener::radiate_flow`],
    /// [`Listener::spread_from_1m`]) and later ([`Listener::from_engine`]). 0 keeps it at the
    /// tails.
    pub fn with_engine_ahead(self, ahead_m: f64) -> Self {
        self.with_engine_at(ahead_m, &[0.0])
    }

    /// The engine's sources `ahead_m` forward of the tails, each `across_m` (positive towards
    /// the listener's side at positive angles) from the car's axis: a V engine's two banks on
    /// either side, heard each from its own distance and after its own path (so what cancels
    /// between identical banks at one point — the half-firing orders — survives). Every source
    /// on the axis at the tails (all 0) keeps them at the tails' mean distance, undelayed.
    pub fn with_engine_at(mut self, ahead_m: f64, across_m: &[f64]) -> Self {
        if ahead_m == 0.0 && across_m.iter().all(|&x| x == 0.0) {
            self.engine = vec![self.engine[0].clone(); across_m.len().max(1)];
            return self;
        }
        let (lx, ly, rise, r_min) = self.ear;
        self.engine = across_m
            .iter()
            .map(|&x| {
                let r = ((lx - x).powi(2) + (ly - ahead_m).powi(2) + rise * rise).sqrt();
                let steps = ((r - r_min) / C_AMBIENT / self.dt).round().max(0.0) as usize;
                EngineSource {
                    at: (x, ahead_m),
                    ..EngineSource::at(r, steps)
                }
            })
            .collect();
        self
    }

    /// The car's body as a screen between the engine's sources and the ear: a roof
    /// `body_height_m` high ending at the tails, the sources `source_height_m` and the ear
    /// `ear_height_m` above the ground. A source whose straight line to the ear passes under
    /// the roof's rear edge is heard over that edge with [`Barrier`]'s loss; an ear ahead of the
    /// tails or above the shadow hears it unscreened. Only the path over the roof is taken
    /// (the sides' diffraction, which a listener far off the axis would also get, is left out).
    pub fn with_body(
        mut self,
        body_height_m: f64,
        source_height_m: f64,
        ear_height_m: f64,
    ) -> Self {
        let (lx, ly, _, _) = self.ear;
        let dt = self.dt;
        for s in self.engine.iter_mut() {
            let (x, a) = s.at;
            s.barrier = None;
            if ly >= 0.0 || a <= 0.0 {
                continue;
            }
            let t = a / (a - ly);
            let (cx, cz) = (
                x + t * (lx - x),
                source_height_m + t * (ear_height_m - source_height_m),
            );
            if cz >= body_height_m {
                continue;
            }
            let d = |p: (f64, f64, f64), q: (f64, f64, f64)| {
                ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2) + (p.2 - q.2).powi(2)).sqrt()
            };
            let (src, ear, edge) = (
                (x, a, source_height_m),
                (lx, ly, ear_height_m),
                (cx, 0.0, body_height_m),
            );
            let delta = d(src, edge) + d(edge, ear) - d(src, ear);
            s.barrier = Some(Barrier::new(delta, dt));
        }
        self
    }

    /// How many places the engine is heard from ([`Listener::with_engine_at`]).
    pub fn engine_sources(&self) -> usize {
        self.engine.len()
    }

    /// The engine's first source's pressure, as computed at its distance, delayed by its path.
    pub fn from_engine(&mut self, p: f64) -> f64 {
        self.from_engine_at(0, p)
    }

    /// Source `k`'s pressure, as computed at its distance, delayed by its path.
    pub fn from_engine_at(&mut self, k: usize, p: f64) -> f64 {
        let s = &mut self.engine[k];
        let n = s.line.len();
        s.line[s.pos] = p;
        s.pos = (s.pos + 1) % n;
        let heard = s.line[s.pos];
        match &mut s.barrier {
            Some(b) => b.process(heard),
            None => heard,
        }
    }

    /// Distance from each tail pipe to the listener, m.
    pub fn distances(&self) -> &[f64] {
        &self.distance
    }

    pub fn tails(&self) -> usize {
        self.gain.len()
    }

    /// A signal in the tails' units (ρc·u) that radiates from all of them alike, at their mean
    /// distance: the sound pressure it adds at the listener, Pa.
    pub fn radiate_shared(&mut self, x: f64) -> f64 {
        let g = self.gain.iter().sum::<f64>() / self.gain.len() as f64;
        let p = g * (x - self.shared_last);
        self.shared_last = x;
        p
    }

    /// A volume flow `q` (m³/s, out of the source) from the engine (at the tails' mean distance
    /// unless [`Listener::with_engine_ahead`] moved it):
    /// the monopole's p = ρ₀/(4πr)·dQ/dt (Kinsler §7.2), Pa.
    pub fn radiate_flow(&mut self, q: f64) -> f64 {
        self.radiate_flow_at(0, q)
    }

    /// [`Listener::radiate_flow`] from the engine's source `k`.
    pub fn radiate_flow_at(&mut self, k: usize, q: f64) -> f64 {
        let s = &mut self.engine[k];
        let p =
            RHO_AMBIENT / (4.0 * std::f64::consts::PI * s.distance) * (q - s.flow_last) / self.dt;
        s.flow_last = q;
        p
    }

    /// [`Listener::radiate_flow`] from an open mouth of area `mouth_area_m2` (the intake): as a
    /// tail ([`Listener::radiate`]), it radiates as a monopole only below ka ≈ 2, flat above
    /// (Levine & Schwinger 1948): H(s) = s/(1 + s/ω_c), ω_c = 2c/a. Pa.
    pub fn radiate_mouth(&mut self, q: f64, mouth_area_m2: f64) -> f64 {
        self.radiate_mouth_at(0, q, mouth_area_m2)
    }

    /// [`Listener::radiate_mouth`] from the engine's source `k` (the mouths share one area).
    pub fn radiate_mouth_at(&mut self, k: usize, q: f64, mouth_area_m2: f64) -> f64 {
        let p = self.radiate_flow_at(k, q);
        if mouth_area_m2 != self.mouth_area {
            let radius = (mouth_area_m2 / std::f64::consts::PI).sqrt().max(1e-3);
            self.mouth_k = 1.0 - (-2.0 * C_AMBIENT / radius * self.dt).exp();
            self.mouth_area = mouth_area_m2;
        }
        let s = &mut self.engine[k];
        s.mouth_radiated += self.mouth_k * (p - s.mouth_radiated);
        s.mouth_radiated
    }

    /// A sound pressure measured 1 m from a compact source (the block) heard from the engine's
    /// first source's place: spherical spreading, p ∝ 1/r (Kinsler §7.2), Pa.
    pub fn spread_from_1m(&self, p_1m: f64) -> f64 {
        self.spread_from_1m_at(0, p_1m)
    }

    /// [`Listener::spread_from_1m`] from the engine's source `k`.
    pub fn spread_from_1m_at(&self, k: usize, p_1m: f64) -> f64 {
        p_1m / self.engine[k].distance
    }

    /// One step: the tails' outlet signals in, the sound pressure at the listener (Pa) out.
    /// Missing tails read as silent ones.
    pub fn process(&mut self, tails: &[f64]) -> f64 {
        let len = self.lines[0].len();
        let mut y = 0.0;
        for (k, line) in self.lines.iter_mut().enumerate() {
            line[self.pos] = tails.get(k).copied().unwrap_or(0.0);
            let heard = line[(self.pos + len - self.delay[k]) % len];
            y += self.gain[k] * (heard - self.last[k]);
            self.last[k] = heard;
            for im in self.images[k].iter_mut() {
                let reflected = line[(self.pos + len - im.delay) % len];
                y += im.hear(reflected);
            }
        }
        self.pos = (self.pos + 1) % len;
        y
    }

    /// [`Listener::process`] with the outlets' own radiation: an unflanged pipe radiates as a
    /// monopole (p ∝ dQ/dt) only while ka ≪ 1; from ka ≈ 2 all the incident power leaves it
    /// (Levine & Schwinger, Phys. Rev. 73, 1948: its reflection falls to zero) and the radiation
    /// stops rising with frequency: H(s) = s/(1 + s/ω_c), ω_c = 2c/a. What the engine plays.
    pub fn radiate(&mut self, tails: &[f64]) -> f64 {
        let y = self.process(tails);
        self.radiated += self.radiation_k * (y - self.radiated);
        self.radiated
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn a_car_body_between_the_engine_and_the_ear_takes_maekawa_s_loss() {
        // Engine 3 m ahead of the tails at 0.6 m, ear 3 m behind them at 1.6 m, roof 1.2 m: the
        // straight line crosses the tails' plane at 1.1 m, under the roof. Over its edge:
        // δ = √(3² + 0.6²) + √(3² + 0.4²) − √(6² + 1²) = 3.059 412 + 3.026 549 − 6.082 763
        // = 0.003 198 m. Maekawa (ISO 9613-2, eq. 14 with C₂ = 20): D = 10·log10(3 + 20N),
        // N = 2δf/c: at 500 Hz N = 0.009 32, D = 5.04 dB; at 8 kHz N = 0.149, D = 7.75 dB.
        let dt = 1.0 / 96_000.0;
        let mut l = Listener::new(1, 0.0, 3.0, 0.0, 1e-3, dt, None)
            .with_engine_ahead(3.0)
            .with_body(1.2, 0.6, 1.6);
        let mut free = Listener::new(1, 0.0, 3.0, 0.0, 1e-3, dt, None).with_engine_ahead(3.0);
        for (f, want) in [(500.0, 5.04), (8000.0, 7.75)] {
            let (mut a, mut b) = (0.0f64, 0.0f64);
            let n = (0.5 / dt) as usize;
            for j in 0..n {
                let x = (2.0 * std::f64::consts::PI * f * j as f64 * dt).sin();
                let (ya, yb) = (l.from_engine(x), free.from_engine(x));
                if j > n / 2 {
                    (a, b) = (a + ya * ya, b + yb * yb);
                }
            }
            let loss = 10.0 * (b / a).log10();
            assert!(
                (loss - want).abs() < 1.2,
                "{f} Hz: {loss:.2} dB, Maekawa {want} dB"
            );
        }
        // an ear ahead of the car's rear end, or over the roof's shadow, hears it unscreened
        let mut over = Listener::new(1, 0.0, 3.0, 0.0, 1e-3, dt, None)
            .with_engine_ahead(3.0)
            .with_body(1.2, 0.6, 4.0);
        assert_eq!(over.from_engine(1.0), free_first(dt));
    }

    fn free_first(dt: f64) -> f64 {
        Listener::new(1, 0.0, 3.0, 0.0, 1e-3, dt, None)
            .with_engine_ahead(3.0)
            .from_engine(1.0)
    }

    use super::*;

    #[test]
    fn a_free_field_ear_keeps_its_height_and_hears_no_image() {
        // tail 0.3 m, ear 1.6 m, 3 m behind: the direct path climbs 1.3 m, √(3² + 1.3²) =
        // 3.2696 m, with or without the ground; only the ground adds its image
        let dt = 1.0 / 48_000.0;
        let free = Listener::with_heights(1, 0.0, 3.0, 0.0, 1e-3, dt, (0.3, 1.6), false, None);
        assert!(
            (free.distances()[0] - 3.2696).abs() < 1e-4,
            "{:?}",
            free.distances()
        );
        assert!(free.images.iter().all(|i| i.is_empty()));
        let ground = Listener::with_heights(1, 0.0, 3.0, 0.0, 1e-3, dt, (0.3, 1.6), true, None);
        assert!((ground.distances()[0] - 3.2696).abs() < 1e-4);
        assert_eq!(ground.images[0].len(), 1);
    }

    #[test]
    fn the_engine_is_heard_from_ahead_of_the_tails_farther_and_later() {
        // ear 3 m straight behind one tail (no ground), engine 4 m ahead: 7 m away, so a source
        // of 1 Pa at 1 m gives 1/7 Pa, after the 4 m extra path: 4/343 s = 0.011662 s = 1119.5
        // → 1120 steps at 96 kHz
        let dt = 1.0 / 96_000.0;
        let mut l =
            Listener::with_street(1, 0.0, 3.0, 0.0, 1e-3, dt, None, None).with_engine_ahead(4.0);
        assert!((l.spread_from_1m(1.0) - 1.0 / 7.0).abs() < 1e-12);
        let first = (0..2000)
            .map(|k| l.from_engine(if k == 0 { 1.0 } else { 0.0 }))
            .position(|p| p != 0.0);
        assert_eq!(first, Some(1120));
        // at the tails (0 m), as before: the tails' distance, no delay
        let mut at =
            Listener::with_street(1, 0.0, 3.0, 0.0, 1e-3, dt, None, None).with_engine_ahead(0.0);
        assert!((at.spread_from_1m(1.0) - 1.0 / 3.0).abs() < 1e-12);
        assert_eq!(at.from_engine(1.0), 1.0);
    }

    #[test]
    fn each_bank_is_heard_from_its_own_side() {
        // ear 3 m to the side of one tail (90°: across 3 m, along 0; no ground), the engine at
        // the tails' station with its banks 0.5 m either side of the car's axis: the near bank
        // 2.5 m away, the far one 3.5 m. A 1 Pa source at 1 m: 1/2.5 and 1/3.5 Pa. The near
        // bank is nearer than the tail (no delay: the common path is dropped); the far one's
        // extra 0.5 m is 0.5/343 s = 1.4577 ms = 139.9 → 140 steps at 96 kHz.
        let dt = 1.0 / 96_000.0;
        let mut l = Listener::with_street(1, 0.0, 3.0, 90.0, 1e-3, dt, None, None)
            .with_engine_at(0.0, &[0.5, -0.5]);
        assert_eq!(l.engine_sources(), 2);
        assert!((l.spread_from_1m_at(0, 1.0) - 1.0 / 2.5).abs() < 1e-12);
        assert!((l.spread_from_1m_at(1, 1.0) - 1.0 / 3.5).abs() < 1e-12);
        let first = |l: &mut Listener, k: usize| {
            (0..2000)
                .map(|i| l.from_engine_at(k, if i == 0 { 1.0 } else { 0.0 }))
                .position(|p| p != 0.0)
        };
        assert_eq!(first(&mut l, 0), Some(0));
        assert_eq!(first(&mut l, 1), Some(140));
        // each bank's mouth is its own monopole: ρ₀/(4πr)·dQ/dt, 1e-3 m³/s in one 10 µs step
        // at 2.5 m: 1.204/(4π·2.5)·1e-3/1e-5 = 3.8324 Pa; at 3.5 m: 2.7374 Pa; each holds its
        // own last flow (a second equal flow at the same source adds nothing)
        let mut l = Listener::with_street(1, 0.0, 3.0, 90.0, 1e-3, 1e-5, None, None)
            .with_engine_at(0.0, &[0.5, -0.5]);
        assert!((l.radiate_flow_at(0, 1e-3) - 3.8324).abs() < 1e-4);
        assert!((l.radiate_flow_at(1, 1e-3) - 2.7374).abs() < 1e-4);
        assert_eq!(l.radiate_flow_at(0, 1e-3), 0.0);
        // one source on the axis, ahead: the single-source calls, as before
        let one = Listener::with_street(1, 0.0, 3.0, 0.0, 1e-3, dt, None, None)
            .with_engine_at(4.0, &[0.0]);
        assert_eq!(one.engine_sources(), 1);
        assert!((one.spread_from_1m(1.0) - 1.0 / 7.0).abs() < 1e-12);
    }

    #[test]
    fn a_flow_source_radiates_as_a_monopole() {
        // p = ρ₀/(4πr)·dQ/dt: Q stepping 0 → 1e-3 m³/s in 10 µs, 3 m away (one tail, straight
        // behind, free field): 1.204/(4π·3)·1e-3/1e-5 = 3.19366 Pa, then nothing while it holds
        let mut l = Listener::new(1, 0.0, 3.0, 0.0, 1e-3, 1e-5, None);
        assert_eq!(l.radiate_flow(0.0), 0.0);
        assert!((l.radiate_flow(1e-3) - 3.193_66).abs() < 1e-4);
        assert_eq!(l.radiate_flow(1e-3), 0.0);
    }
    use approx::assert_relative_eq;

    const DT: f64 = 1.0 / 192_000.0;
    /// 50 mm tail pipe: A = π/4·0.05² = 1.9635e-3 m²
    const A: f64 = 1.963_495e-3;
    /// Tail gas impedance ρc: c = 500 m/s ⇒ T = c²/γR = 622.2 K, ρ = 101 325/(287·622.2)
    /// = 0.56742 kg/m³, ρc = 283.71 Pa·s/m
    const Z: f64 = 283.71;

    /// A tail signal (ρc·u) whose volume flow Q = A·u rises at 1 m³/s per second.
    fn unit_ramp(n: usize) -> f64 {
        n as f64 * DT * Z / A
    }

    #[test]
    fn a_street_returns_the_tail_from_its_facades() {
        // One tail 3 m straight behind, free field below (no ground), between façades 10 m
        // apart (the car in the middle) reflecting ρ = 0.8 (ISO 9613-2, Table 4: walls of
        // buildings with windows). Images at x = ±10 m (one bounce) and ±20 m (two): paths
        // √(10² + 3²) = 10.440 m and √(20² + 3²) = 20.224 m. A tail flow rising steadily is heard
        // first alone, then from 21.7 ms ((10.440 − 3)/343) with each façade's image, and from
        // 49.2 ms with the second bounces: 1 + 2·0.8·3/10.440 + 2·0.64·3/20.224 = 1.6496 times
        // the direct sound (the air's absorption passes a steady signal whole)
        let street = Some(Street {
            width_m: 10.0,
            reflection: 0.8,
        });
        let mut l = Listener::with_street(1, 0.9, 3.0, 0.0, A, DT, None, street);
        let ys: Vec<f64> = (0..(0.08 / DT) as usize)
            .map(|n| l.process(&[unit_ramp(n)]))
            .collect();
        let direct = ys[100];
        let first = (0.021_69 / DT) as usize;
        assert!(
            (ys[first - 20] / direct - 1.0).abs() < 1e-9,
            "before the façades"
        );
        assert!(ys[first + 400] > 1.3 * direct, "the façades' images");
        let steady = ys[ys.len() - 1] / direct;
        assert!((steady / 1.6496 - 1.0).abs() < 0.005, "{steady}");
    }

    #[test]
    fn the_air_takes_the_highs_off_a_long_path() {
        // ISO 9613-1 at 20 °C, 70 % RH: α = 0.0776 dB/m at 8 kHz, so a 38.66 m path loses 3 dB
        // there: its one-pole's corner. Short paths keep everything up to the 20 kHz cap
        assert!((air_corner_hz(3.0 / 0.0776) / 8000.0 - 1.0).abs() < 0.01);
        assert_eq!(air_corner_hz(3.0), 20_000.0);
        assert!(air_corner_hz(100.0) < air_corner_hz(40.0));
    }

    #[test]
    fn one_tail_radiates_as_a_monopole() {
        // p = ρ₀/(4πr)·dQ/dt (Kinsler, Fundamentals of Acoustics, §7.2): ρ₀ = 1.204 kg/m³
        // (20 °C), r = 3 m, dQ/dt = 1 m³/s² → 1.204/(4π·3) = 0.031937 Pa
        let mut l = Listener::new(1, 0.9, 3.0, 0.0, A, DT, None);
        let mut p = 0.0;
        for n in 0..100 {
            p = l.process(&[unit_ramp(n)]);
        }
        assert_relative_eq!(p, 0.031_937, max_relative = 1e-4);
    }

    #[test]
    fn a_listener_on_the_axis_hears_both_tails_together() {
        // tails 0.9 m apart, listener 3 m behind their centre: r = √(3² + 0.45²) = 3.0336 m for
        // both, no delay between them; each gives 1.204/(4π·3.0336) = 0.031584 Pa
        let mut l = Listener::new(2, 0.9, 3.0, 0.0, A, DT, None);
        let mut p = 0.0;
        for n in 0..100 {
            p = l.process(&[unit_ramp(n), 2.0 * unit_ramp(n)]);
        }
        assert_relative_eq!(p, 3.0 * 0.031_584, max_relative = 1e-4);
        for r in l.distances() {
            assert_relative_eq!(*r, 3.0336, max_relative = 1e-4);
        }
    }

    /// Sample index and value of the strongest output for an impulse fed to tail `k`.
    fn arrival(l: &mut Listener, k: usize) -> (usize, f64) {
        let n = l.tails();
        let out: Vec<f64> = (0..4_000)
            .map(|i| {
                let mut x = vec![0.0; n];
                if i == 0 {
                    x[k] = 1.0;
                }
                l.process(&x)
            })
            .collect();
        let (i, v) = out
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .unwrap();
        (i, *v)
    }

    #[test]
    fn a_listener_beside_the_car_hears_the_near_tail_first_and_louder() {
        // Tails at x = ∓0.5 m, listener level with them at 3 m from the centre (θ = 90°):
        // r = 3.5 and 2.5 m. Δr = 1 m → 1/343 s = 559.8 samples at 192 kHz; amplitude ∝ 1/r,
        // so the near tail is 3.5/2.5 = 1.4 times louder.
        let mut l = Listener::new(2, 1.0, 3.0, 90.0, A, DT, None);
        let (far, g_far) = arrival(&mut l, 0);
        let mut l = Listener::new(2, 1.0, 3.0, 90.0, A, DT, None);
        let (near, g_near) = arrival(&mut l, 1);
        assert_eq!(far - near, 560);
        assert_relative_eq!(g_near / g_far, 1.4, max_relative = 1e-9);
    }

    #[test]
    fn a_shared_signal_radiates_at_the_tails_mean_gain() {
        // the beside-the-car tails (r = 3.5 and 2.5 m), one ramp from both: the mean of the
        // two monopoles, 1.204/(4π)·(1/3.5 + 1/2.5)/2 = 0.032850 Pa, steady along the ramp
        let mut l = Listener::new(2, 1.0, 3.0, 90.0, A, DT, None);
        let p: Vec<f64> = (0..100).map(|n| l.radiate_shared(unit_ramp(n))).collect();
        for p in &p[1..] {
            assert_relative_eq!(*p, 0.032_850, max_relative = 1e-4);
        }
    }

    #[test]
    fn the_ground_sends_a_second_arrival_from_the_image_source() {
        // Tail 0.3 m up, listener's ear 1.5 m up, 3 m straight behind, rigid ground (asphalt,
        // Q ≈ 1; ISO 9613-2): direct path √(3² + 1.2²) = 3.2311 m, reflected (image 0.3 m
        // below the ground) √(3² + 1.8²) = 3.4986 m: 0.2675 m later = 149.7 → 150 steps at
        // 192 kHz, at 3.2311/3.4986 = 0.92354 of the direct amplitude, same sign.
        let mut l = Listener::new(1, 0.9, 3.0, 0.0, A, DT, Some((0.3, 1.5)));
        let out: Vec<f64> = (0..400)
            .map(|i| l.process(&[if i == 0 { 1.0 } else { 0.0 }]))
            .collect();
        let direct = out[0];
        assert!(direct > 0.0);
        assert_relative_eq!(out[150] / direct, 0.923_54, max_relative = 1e-4);
        assert_relative_eq!(l.distances()[0], 3.231_1, max_relative = 1e-4);
        // nothing else arrives in between (the impulse's own step back aside)
        assert!(out[2..149].iter().all(|v| *v == 0.0));
        // free field: one arrival
        let mut f = Listener::new(1, 0.9, 3.0, 0.0, A, DT, None);
        let out: Vec<f64> = (0..400)
            .map(|i| f.process(&[if i == 0 { 1.0 } else { 0.0 }]))
            .collect();
        assert!(out[2..].iter().all(|v| *v == 0.0));
        // a step at the tail: one spike from the tail, one from its image, then silence — each
        // path radiates the step's change once
        let mut l = Listener::new(1, 0.9, 3.0, 0.0, A, DT, Some((0.3, 1.5)));
        let ys: Vec<f64> = (0..400).map(|_| l.process(&[1.0])).collect();
        let spikes: Vec<f64> = ys.iter().copied().filter(|y| *y != 0.0).collect();
        assert_eq!(spikes.len(), 2, "{spikes:?}");
        // the image 150 steps (0.2675 m) after the tail
        let at: Vec<usize> = (0..ys.len()).filter(|&i| ys[i] != 0.0).collect();
        assert_eq!(at[1] - at[0], 150, "{at:?}");
        // two tails 0.9 m apart, heard 3 m out at 30°, ears 1.6 m up: the nearer tail (x = +0.45)
        // is 3.0891 m away, its image 3.3856 m: 166 steps later; the farther one's paths are
        // 3.4989 and 3.7633 m (229 and 377 steps after the nearest arrival)
        let mut l = Listener::new(2, 0.9, 3.0, 30.0, A, DT, Some((0.3, 1.6)));
        let ys: Vec<f64> = (0..600).map(|_| l.process(&[0.0, 1.0])).collect();
        let at: Vec<usize> = (0..ys.len()).filter(|&i| ys[i] != 0.0).collect();
        assert_eq!(at, [0, 166], "{at:?}");
        let mut l = Listener::new(2, 0.9, 3.0, 30.0, A, DT, Some((0.3, 1.6)));
        let ys: Vec<f64> = (0..600).map(|_| l.process(&[1.0, 0.0])).collect();
        let at: Vec<usize> = (0..ys.len()).filter(|&i| ys[i] != 0.0).collect();
        assert_eq!(at, [229, 377], "{at:?}");
        assert_relative_eq!(spikes[1] / spikes[0], 0.923_54, max_relative = 1e-4);
    }

    #[test]
    fn a_steady_flow_is_silent() {
        // only the change of the volume flow radiates
        let mut l = Listener::new(1, 0.9, 3.0, 0.0, A, DT, None);
        l.process(&[5.0]);
        assert_eq!(l.process(&[5.0]), 0.0);
    }

    #[test]
    fn a_tail_radiates_like_an_open_pipe_above_ka_of_two() {
        // A monopole's p ∝ dQ/dt holds while the outlet is small against the wavelength. An
        // unflanged pipe passes all incident power out once ka ≳ 2 (Levine & Schwinger 1948:
        // its reflection falls to zero), so the radiation stops rising there: H(s) = s/(1 + s/ω_c),
        // ω_c = 2c/a. For the 50 mm tail (a = 25 mm): ω_c = 2·343/0.025 = 27 440 rad/s; with a
        // steady Q, a 16 kHz tone against a 1 kHz one gives |H(16k)|/|H(1k)| =
        // (100 531/√(1 + 3.6637²))/(6 283.2/√(1 + 0.22898²)) = 26 471/6 124.7 = 4.32
        // (a pure dQ/dt would give 16).
        let amp = |f: f64| {
            let mut l = Listener::new(1, 0.0, 3.0, 0.0, A, DT, None);
            let n = (0.05 / DT) as usize;
            let mut sum = 0.0;
            for i in 0..n {
                let x = (2.0 * std::f64::consts::PI * f * i as f64 * DT).sin();
                let p = l.radiate(&[x]);
                if i > n / 2 {
                    sum += p * p;
                }
            }
            sum.sqrt()
        };
        let ratio = amp(16_000.0) / amp(1_000.0);
        assert!((ratio / 4.32 - 1.0).abs() < 0.03, "ratio {ratio}");
    }

    #[test]
    fn an_intake_mouth_radiates_like_an_open_pipe_above_ka_of_two() {
        // the tails' rule for the intake: a 50 mm mouth (a = 25 mm), a steady volume flow
        // amplitude, 16 kHz against 1 kHz: 4.32 (see the tails' test for the hand value), not
        // the monopole's 16
        let amp = |f: f64| {
            let mut l = Listener::new(1, 0.0, 3.0, 0.0, A, DT, None);
            let n = (0.05 / DT) as usize;
            let mut sum = 0.0;
            for i in 0..n {
                let q = 1e-3 * (2.0 * std::f64::consts::PI * f * i as f64 * DT).sin();
                let p = l.radiate_mouth(q, A);
                if i > n / 2 {
                    sum += p * p;
                }
            }
            sum.sqrt()
        };
        let ratio = amp(16_000.0) / amp(1_000.0);
        assert!((ratio / 4.32 - 1.0).abs() < 0.03, "ratio {ratio}");
    }
}
