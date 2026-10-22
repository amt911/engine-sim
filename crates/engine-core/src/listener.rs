//! Where the listener stands: each tail pipe reaches them with its own delay and spreading
//! loss. Two tails a car's width apart, heard from off the axis, arrive out of phase at the
//! low orders. That is why a cross-plane V8's uneven bank pulses are heard as a burble; summed
//! at one point they cancel exactly into even 90° firing.

/// Speed of sound in the ambient air between the car and the listener, m/s (20 °C).
const C_AMBIENT: f64 = 343.0;

/// The tails spread evenly across `spacing_m` at the rear (x across the car, y = 0), the
/// listener at `distance_m` from their centre, `angle_deg` off the car's axis (0 = straight
/// behind). Free-field monopoles: delay r/c and amplitude ∝ 1/r (Kinsler, *Fundamentals of
/// Acoustics*, §7.2), the gains normalised so that their sum is the tail count (a listener on
/// the axis hears the plain sum). Only the delay differences are kept; the common path is
/// dropped.
#[derive(Debug, Clone)]
pub struct Listener {
    delay: Vec<usize>,
    gain: Vec<f64>,
    lines: Vec<Vec<f64>>,
    pos: usize,
}

impl Listener {
    pub fn new(tails: usize, spacing_m: f64, distance_m: f64, angle_deg: f64, dt: f64) -> Self {
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
                ((lx - x).powi(2) + ly * ly).sqrt().max(1e-3)
            })
            .collect();
        let r_min = r.iter().copied().fold(f64::INFINITY, f64::min);
        let delay: Vec<usize> = r
            .iter()
            .map(|ri| ((ri - r_min) / C_AMBIENT / dt).round() as usize)
            .collect();
        let inv: f64 = r.iter().map(|ri| 1.0 / ri).sum();
        let gain = r.iter().map(|ri| n as f64 / ri / inv).collect();
        let len = delay.iter().copied().max().unwrap_or(0) + 1;
        Self {
            delay,
            gain,
            lines: vec![vec![0.0; len]; n],
            pos: 0,
        }
    }

    pub fn tails(&self) -> usize {
        self.gain.len()
    }

    /// One step: the tails' radiated signals in, the pressure at the listener out. Missing
    /// tails read as silent ones.
    pub fn process(&mut self, tails: &[f64]) -> f64 {
        let len = self.lines[0].len();
        let mut y = 0.0;
        for (k, line) in self.lines.iter_mut().enumerate() {
            line[self.pos] = tails.get(k).copied().unwrap_or(0.0);
            y += self.gain[k] * line[(self.pos + len - self.delay[k]) % len];
        }
        self.pos = (self.pos + 1) % len;
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 192_000.0;

    /// Sample index at which an impulse fed to tail `k` comes out.
    fn arrival(l: &mut Listener, k: usize) -> (usize, f64) {
        let n = l.tails();
        let mut out = Vec::new();
        for i in 0..4_000 {
            let mut x = vec![0.0; n];
            if i == 0 {
                x[k] = 1.0;
            }
            out.push(l.process(&x));
        }
        let (i, v) = out
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .unwrap();
        (i, *v)
    }

    #[test]
    fn a_listener_on_the_axis_hears_the_plain_sum() {
        let mut l = Listener::new(2, 0.9, 3.0, 0.0, DT);
        for i in 0..100 {
            let x = [i as f64, 2.0 * i as f64];
            assert_eq!(l.process(&x), 3.0 * i as f64);
        }
    }

    #[test]
    fn a_listener_beside_the_car_hears_the_near_tail_first_and_louder() {
        // Tails at x = ∓0.5 m, listener level with them at 3 m from the centre (θ = 90°):
        // r = 3.5 and 2.5 m, Δr = 1 m → 1/343 s = 559.8 samples at 192 kHz. Gains ∝ 1/r,
        // normalised to sum to 2: 2·(1/3.5)/(1/3.5 + 1/2.5) = 0.8333 and 1.1667.
        let mut l = Listener::new(2, 1.0, 3.0, 90.0, DT);
        let (far, g_far) = arrival(&mut l, 0);
        let mut l = Listener::new(2, 1.0, 3.0, 90.0, DT);
        let (near, g_near) = arrival(&mut l, 1);
        assert_eq!(far - near, 560);
        assert!((g_far - 0.8333).abs() < 1e-3 && (g_near - 1.1667).abs() < 1e-3);
    }

    #[test]
    fn one_tail_passes_straight_through() {
        let mut l = Listener::new(1, 0.9, 3.0, 45.0, DT);
        assert_eq!(l.process(&[2.5]), 2.5);
    }
}
