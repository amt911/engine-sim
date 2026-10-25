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
    flow_last: f64,
    mean_distance: f64,
    dt: f64,
    /// The tails' radiation above ka ≈ 2: a one-pole low-pass on dQ/dt at ω_c = 2c/a, and its
    /// state.
    radiation_k: f64,
    radiated: f64,
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
        let n = tails.max(1);
        let (lx, ly) = {
            let a = angle_deg.to_radians();
            (distance_m * a.sin(), -distance_m * a.cos())
        };
        // heights above the ground (tail, ear): the direct path climbs (h_l − h_s), the
        // ground's image path (h_l + h_s); free field has neither
        let (rise, image_rise) = match ground {
            Some((hs, hl)) => (hl - hs, Some(hl + hs)),
            None => (0.0, None),
        };
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
            flow_last: 0.0,
            mean_distance,
            dt,
            radiation_k,
            radiated: 0.0,
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

    /// A volume flow `q` (m³/s, out of the source) from a point at the tails' mean distance:
    /// the monopole's p = ρ₀/(4πr)·dQ/dt (Kinsler §7.2), Pa.
    pub fn radiate_flow(&mut self, q: f64) -> f64 {
        let p = RHO_AMBIENT / (4.0 * std::f64::consts::PI * self.mean_distance)
            * (q - self.flow_last)
            / self.dt;
        self.flow_last = q;
        p
    }

    /// A sound pressure measured 1 m from a compact source (the block) heard at the tails'
    /// mean distance: spherical spreading, p ∝ 1/r (Kinsler §7.2), Pa.
    pub fn spread_from_1m(&self, p_1m: f64) -> f64 {
        p_1m / self.mean_distance
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
    use super::*;

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
}
