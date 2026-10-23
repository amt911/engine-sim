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
    /// Per tail, its ground image: (delay, gain); empty in free field.
    image: Vec<(usize, f64)>,
    image_last: Vec<f64>,
    pos: usize,
    /// For a signal radiated from all the tails alike (at their mean gain, no delay).
    shared_last: f64,
    flow_last: f64,
    mean_distance: f64,
    dt: f64,
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
        // the rigid ground (asphalt: reflection Q ≈ 1 at these frequencies, ISO 9613-2) returns
        // each tail from its image below it, later and weaker
        let image: Vec<(usize, f64)> = match image_rise {
            Some(h) => xs
                .iter()
                .map(|&x| {
                    let ri = path(x, h);
                    (steps(ri), monopole(ri))
                })
                .collect(),
            None => Vec::new(),
        };
        let delay_max = delay
            .iter()
            .copied()
            .chain(image.iter().map(|i| i.0))
            .max()
            .unwrap_or(0);
        let len = delay_max + 1;
        let mean_distance = r.iter().sum::<f64>() / r.len() as f64;
        Self {
            distance: r,
            delay,
            gain,
            lines: vec![vec![0.0; len]; n],
            last: vec![0.0; n],
            image_last: vec![0.0; image.len()],
            image,
            pos: 0,
            shared_last: 0.0,
            flow_last: 0.0,
            mean_distance,
            dt,
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
            if let Some(&(d, g)) = self.image.get(k) {
                let reflected = line[(self.pos + len - d) % len];
                y += g * (reflected - self.image_last[k]);
                self.image_last[k] = reflected;
            }
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
    }

    #[test]
    fn a_steady_flow_is_silent() {
        // only the change of the volume flow radiates
        let mut l = Listener::new(1, 0.9, 3.0, 0.0, A, DT, None);
        l.process(&[5.0]);
        assert_eq!(l.process(&[5.0]), 0.0);
    }
}
