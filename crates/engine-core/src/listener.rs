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
    pos: usize,
    /// For a signal radiated from all the tails alike (at their mean gain, no delay).
    shared_last: f64,
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
    ) -> Self {
        let n = tails.max(1);
        let (lx, ly) = {
            let a = angle_deg.to_radians();
            (distance_m * a.sin(), -distance_m * a.cos())
        };
        let r: Vec<f64> = (0..n)
            .map(|k| {
                let x = if n == 1 {
                    0.0
                } else {
                    -0.5 * spacing_m + spacing_m * k as f64 / (n - 1) as f64
                };
                ((lx - x).powi(2) + ly * ly).sqrt().max(0.1)
            })
            .collect();
        let r_min = r.iter().copied().fold(f64::INFINITY, f64::min);
        let delay: Vec<usize> = r
            .iter()
            .map(|ri| ((ri - r_min) / C_AMBIENT / dt).round() as usize)
            .collect();
        let to_q = tail_area_m2 / crate::exhaust::tail_impedance();
        let gain = r
            .iter()
            .map(|ri| RHO_AMBIENT / (4.0 * std::f64::consts::PI * ri) * to_q / dt)
            .collect();
        let len = delay.iter().copied().max().unwrap_or(0) + 1;
        Self {
            distance: r,
            delay,
            gain,
            lines: vec![vec![0.0; len]; n],
            last: vec![0.0; n],
            pos: 0,
            shared_last: 0.0,
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
        }
        self.pos = (self.pos + 1) % len;
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
    fn one_tail_radiates_as_a_monopole() {
        // p = ρ₀/(4πr)·dQ/dt (Kinsler, Fundamentals of Acoustics, §7.2): ρ₀ = 1.204 kg/m³
        // (20 °C), r = 3 m, dQ/dt = 1 m³/s² → 1.204/(4π·3) = 0.031937 Pa
        let mut l = Listener::new(1, 0.9, 3.0, 0.0, A, DT);
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
        let mut l = Listener::new(2, 0.9, 3.0, 0.0, A, DT);
        let mut p = 0.0;
        for n in 0..100 {
            p = l.process(&[unit_ramp(n), 2.0 * unit_ramp(n)]);
        }
        assert_relative_eq!(p, 3.0 * 0.031_584, max_relative = 1e-4);
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
        let mut l = Listener::new(2, 1.0, 3.0, 90.0, A, DT);
        let (far, g_far) = arrival(&mut l, 0);
        let mut l = Listener::new(2, 1.0, 3.0, 90.0, A, DT);
        let (near, g_near) = arrival(&mut l, 1);
        assert_eq!(far - near, 560);
        assert_relative_eq!(g_near / g_far, 1.4, max_relative = 1e-9);
    }

    #[test]
    fn a_steady_flow_is_silent() {
        // only the change of the volume flow radiates
        let mut l = Listener::new(1, 0.9, 3.0, 0.0, A, DT);
        l.process(&[5.0]);
        assert_eq!(l.process(&[5.0]), 0.0);
    }
}
