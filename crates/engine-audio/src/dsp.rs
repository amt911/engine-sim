//! Pure DSP blocks. f32 audio path, f64 where filter state needs it.

use std::f64::consts::PI;

/// One-pole DC blocker y[n] = x[n] − x[n−1] + R·y[n−1]
/// (J. O. Smith, *Introduction to Digital Filters*, "DC Blocker").
/// Cutoff must sit at a few Hz: low-cylinder engines have ~5 Hz fundamentals at idle.
#[derive(Debug, Clone)]
pub struct DcBlocker {
    r: f64,
    x1: f64,
    y1: f64,
}

impl DcBlocker {
    pub fn new(cutoff_hz: f64, sample_rate: f64) -> Self {
        Self {
            r: 1.0 - 2.0 * PI * cutoff_hz / sample_rate,
            x1: 0.0,
            y1: 0.0,
        }
    }
    pub fn process(&mut self, x: f64) -> f64 {
        let y = x - self.x1 + self.r * self.y1;
        self.x1 = x;
        self.y1 = y;
        y
    }
}

/// Upward leveler: lifts quiet passages (idle) toward a reference, leaves louder ones alone.
/// Gain = (ref/level)^(1 − 1/ratio) with ratio 2 below the reference, at most +12 dB; the
/// level is a mean square (symmetric, 20 ms: an RMS, not a peak detector) followed by a fast
/// attack (20 ms, so a blip from idle does not arrive boosted) and a slow release (0.5 s, so
/// the gain does not pump within an idle cycle).
/// The usual broadcast "AGC/leveler" (Zölzer, *DAFX*, ch. 4, dynamics processing).
#[derive(Debug, Clone)]
pub struct Leveler {
    /// How much of the boost to apply, in dB: 1 = all of it, 0 = none (natural dynamics).
    pub amount: f64,
    /// RMS at which the gain is 1: a sine of amplitude `reference`.
    ref_ms: f64,
    mean: f64,
    ms: f64,
    smooth: f64,
    attack: f64,
    release: f64,
}

/// Largest boost of the leveler, as a gain factor (+12 dB).
pub const LEVELER_MAX_GAIN: f64 = 3.981_071_705_534_973;

impl Leveler {
    pub fn new(reference: f64, sample_rate: f64) -> Self {
        let k = |tau: f64| 1.0 - (-1.0 / (tau * sample_rate)).exp();
        Self {
            amount: 1.0,
            ref_ms: 0.5 * reference * reference,
            mean: 0.5 * reference * reference,
            ms: 0.5 * reference * reference,
            smooth: k(0.020),
            attack: k(0.020),
            release: k(0.5),
        }
    }
    pub fn gain(&self) -> f64 {
        // ratio 2: gain = (ref/level)^(1/2) = (ref_ms/ms)^(1/4)
        (self.ref_ms / self.ms.max(1e-30))
            .sqrt()
            .sqrt()
            .clamp(1.0, LEVELER_MAX_GAIN)
            .powf(self.amount.clamp(0.0, 1.0))
    }
    pub fn process(&mut self, x: f64) -> f64 {
        self.mean += self.smooth * (x * x - self.mean);
        let k = if self.mean > self.ms {
            self.attack
        } else {
            self.release
        };
        self.ms += k * (self.mean - self.ms);
        x * self.gain()
    }
}

/// RBJ cookbook 2nd-order low-pass biquad (direct form I).
#[derive(Debug, Clone)]
pub struct LowPass {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
}

impl LowPass {
    pub fn new(cutoff_hz: f64, q: f64, sample_rate: f64) -> Self {
        let w0 = 2.0 * PI * cutoff_hz / sample_rate;
        let (sin, cos) = w0.sin_cos();
        let alpha = sin / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b1 = (1.0 - cos) / a0;
        Self {
            b0: b1 * 0.5,
            b1,
            b2: b1 * 0.5,
            a1: -2.0 * cos / a0,
            a2: (1.0 - alpha) / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }
    pub fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

/// Small room / car-body reverb: 4-line feedback delay network with a Householder matrix
/// and per-line damping (Jot & Chaigne 1991; J. O. Smith, PASP, "FDN reverberation").
#[derive(Debug, Clone)]
pub struct Reverb {
    lines: [Vec<f64>; 4],
    pos: [usize; 4],
    g: [f64; 4],
    damp: [f64; 4],
    damp_k: f64,
}

impl Reverb {
    /// `rt60`: time (s) for the tail to decay by 60 dB.
    pub fn new(sample_rate: f64, rt60: f64) -> Self {
        // Mutually prime-ish lengths (ms) spread the echoes densely.
        const MS: [f64; 4] = [29.7, 37.1, 41.1, 43.7];
        let len = MS.map(|ms| (ms * 1e-3 * sample_rate) as usize);
        Self {
            lines: len.map(|n| vec![0.0; n.max(1)]),
            pos: [0; 4],
            // −60 dB after rt60: each pass through a line of d samples loses 60·d/(rt60·fs) dB.
            g: len.map(|n| 10f64.powf(-3.0 * n as f64 / (rt60 * sample_rate))),
            damp: [0.0; 4],
            damp_k: 1.0 - (-2.0 * PI * 6_000.0 / sample_rate).exp(),
        }
    }
    /// Returns the wet (reverberant) signal for input `x`.
    pub fn process(&mut self, x: f64) -> f64 {
        let mut v = [0.0; 4];
        let mut wet = 0.0;
        for (((line, pos), damp), (g, vi)) in self
            .lines
            .iter()
            .zip(&self.pos)
            .zip(self.damp.iter_mut())
            .zip(self.g.iter().zip(v.iter_mut()))
        {
            let o = line[*pos];
            wet += o;
            *damp += self.damp_k * (o - *damp);
            *vi = g * *damp;
        }
        // Householder reflection: lossless mixing, v − (2/N)·Σv.
        let sum: f64 = v.iter().sum::<f64>() * 0.5;
        for ((line, pos), vi) in self.lines.iter_mut().zip(self.pos.iter_mut()).zip(v) {
            line[*pos] = x + vi - sum;
            *pos = (*pos + 1) % line.len();
        }
        wet * 0.5
    }
}

/// Above this frequency the radiated sound follows the derivative of the source.
pub const RADIATION_CORNER_HZ: f64 = 150.0;

/// The engine voice: raw exhaust pressure (Pa) → speaker sample in [-1, 1].
#[derive(Debug, Clone)]
pub struct Voice {
    dc: DcBlocker,
    reverb: Reverb,
    /// Wet level of the body/room reverb (0 = dry).
    pub reverb_mix: f64,
    prev: f64,
    lp: LowPass,
    sample_rate: f64,
    /// Pressure that maps to full scale before soft clipping, Pa.
    pub full_scale_pa: f64,
    /// Lifts idle toward the full-load level (reference: a full-load engine, ≈ −15 dBFS RMS).
    pub leveler: Leveler,
}

/// Output level (sine amplitude, of full scale) the leveler treats as full load.
pub const LEVELER_REFERENCE: f64 = 0.25;

impl Voice {
    pub fn new(sample_rate: f64) -> Self {
        Self {
            dc: DcBlocker::new(3.0, sample_rate),
            reverb: Reverb::new(sample_rate, 0.35),
            reverb_mix: 0.0,
            prev: 0.0,
            lp: LowPass::new(
                5000.0_f64.min(0.45 * sample_rate),
                std::f64::consts::FRAC_1_SQRT_2,
                sample_rate,
            ),
            sample_rate,
            full_scale_pa: 150_000.0,
            leveler: Leveler::new(LEVELER_REFERENCE, sample_rate),
        }
    }
    pub fn process(&mut self, pressure_pa: f64) -> f32 {
        if !pressure_pa.is_finite() {
            return 0.0;
        }
        // Radiation: a pipe outlet radiates like a monopole, p ∝ d(flow)/dt (J. O. Smith,
        // PASP, "Spherical Waves from a Point Source"). Mix the source with its derivative so
        // components above RADIATION_CORNER_HZ are lifted ∝ frequency.
        let x = self.dc.process(pressure_pa);
        let dx = (x - self.prev) * self.sample_rate / (2.0 * PI * RADIATION_CORNER_HZ);
        self.prev = x;
        let dry = self.lp.process(x + dx);
        let y = self
            .leveler
            .process((dry + self.reverb_mix * self.reverb.process(dry)) / self.full_scale_pa);
        if !y.is_finite() {
            // A runaway filter state is reset rather than left to poison every later sample.
            let amount = self.leveler.amount;
            *self = Voice {
                full_scale_pa: self.full_scale_pa,
                ..Voice::new(self.sample_rate)
            };
            self.leveler.amount = amount;
            return 0.0;
        }
        y.tanh() as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    const FS: f64 = 48_000.0;

    /// Peak output amplitude after settling, for a unit sine at `f`.
    fn sine_gain(mut f_proc: impl FnMut(f64) -> f64, f: f64) -> f64 {
        let n = FS as usize;
        let mut peak: f64 = 0.0;
        for i in 0..n {
            let y = f_proc((2.0 * PI * f * i as f64 / FS).sin());
            if i > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    /// Peak of `lev` on a sine of amplitude `a` at 100 Hz, over the last 0.1 s of `secs`.
    fn leveled_peak(lev: &mut Leveler, a: f64, secs: f64) -> f64 {
        let n = (secs * FS) as usize;
        let mut peak: f64 = 0.0;
        for i in 0..n {
            let y = lev.process(a * (2.0 * PI * 100.0 * i as f64 / FS).sin());
            if i + (0.1 * FS) as usize >= n {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    #[test]
    fn leveler_lifts_quiet_sound_and_leaves_loud_sound_alone() {
        // reference 1.0: a sine at full reference passes at unit gain
        let mut lev = Leveler::new(1.0, FS);
        let loud = leveled_peak(&mut lev, 1.0, 2.0);
        assert!((loud - 1.0).abs() < 0.02, "loud {loud}");
        // 20 dB below the reference (rms 0.071 against 0.707): ratio 2 halves it, +10 dB
        let mut lev = Leveler::new(1.0, FS);
        let q = leveled_peak(&mut lev, 0.1, 6.0);
        assert!(
            (q / 0.1 - 10f64.powf(10.0 / 20.0)).abs() < 0.1,
            "gain {}",
            q / 0.1
        );
        // 40 dB below: the boost stops at +12 dB (from the reference, the 0.5 s release takes
        // seconds to settle)
        let mut lev = Leveler::new(1.0, FS);
        let q = leveled_peak(&mut lev, 0.01, 6.0);
        assert!(
            (q / 0.01 - 10f64.powf(12.0 / 20.0)).abs() < 0.05,
            "gain {}",
            q / 0.01
        );
    }

    #[test]
    fn leveler_amount_scales_the_boost_in_decibels() {
        // half the amount: half the dB (+6 of the +12); none: unity
        for (amount, db) in [(0.5, 6.0), (0.0, 0.0)] {
            let mut lev = Leveler::new(1.0, FS);
            lev.amount = amount;
            let q = leveled_peak(&mut lev, 0.01, 6.0);
            assert!(
                (q / 0.01 - 10f64.powf(db / 20.0)).abs() < 0.03,
                "{amount}: {}",
                q / 0.01
            );
        }
    }

    #[test]
    fn leveler_drops_its_boost_fast_when_the_sound_gets_loud() {
        // a blip from idle: within 60 ms of the step the boost is (nearly) gone
        let mut lev = Leveler::new(1.0, FS);
        leveled_peak(&mut lev, 0.01, 3.0);
        let after = leveled_peak(&mut lev, 1.0, 0.16);
        assert!(after < 1.15, "blip boosted to {after}");
    }

    #[test]
    fn dc_blocker_removes_dc_keeps_low_tones() {
        let mut dc = DcBlocker::new(3.0, FS);
        let mut y = 1.0;
        for _ in 0..FS as usize {
            y = dc.process(1.0);
        }
        assert!(y.abs() < 1e-3, "residual {y}");
        let mut dc = DcBlocker::new(3.0, FS);
        assert!(sine_gain(|x| dc.process(x), 50.0) > 0.98);
    }

    #[test]
    fn low_pass_is_minus_3db_at_cutoff_and_rolls_off() {
        let q = std::f64::consts::FRAC_1_SQRT_2;
        let mut lp = LowPass::new(2000.0, q, FS);
        assert_relative_eq!(
            sine_gain(|x| lp.process(x), 2000.0),
            std::f64::consts::FRAC_1_SQRT_2,
            max_relative = 0.01
        );
        let mut lp = LowPass::new(2000.0, q, FS);
        assert_relative_eq!(
            sine_gain(|x| lp.process(x), 100.0),
            1.0,
            max_relative = 0.01
        );
        // 2nd order: −12 dB/octave → ×4 frequency ≈ 1/16
        let mut lp = LowPass::new(2000.0, q, FS);
        assert!(sine_gain(|x| lp.process(x), 8000.0) < 0.08);
    }

    #[test]
    fn reverb_tail_decays_at_rt60() {
        let mut r = Reverb::new(FS, 0.3);
        let energy = |from: usize, to: usize, r: &mut Reverb, first: bool| {
            let mut e = 0.0;
            for n in from..to {
                let y = r.process(if first && n == 0 { 1.0 } else { 0.0 });
                e += y * y;
            }
            e
        };
        let early = energy(0, 4_800, &mut r, true); // first 0.1 s
        let _ = energy(4_800, 14_400, &mut r, false); // up to 0.3 s
        let late = energy(14_400, 19_200, &mut r, false); // 0.3–0.4 s
        assert!(early > 0.0);
        // After one RT60 the level is −60 dB: energy ratio ~1e-6 (generous bound for the windows)
        assert!(late < 1e-4 * early, "early {early} late {late}");
    }

    #[test]
    fn reverb_is_stable_and_bounded() {
        let mut r = Reverb::new(FS, 2.0);
        let mut peak: f64 = 0.0;
        for n in 0..FS as usize * 3 {
            peak = peak.max(r.process(((n as f64) * 0.05).sin()).abs());
        }
        assert!(peak.is_finite() && peak < 20.0, "{peak}");
    }

    #[test]
    fn voice_reverb_leaves_a_tail_only_when_mixed_in() {
        let tail = |mix: f64| {
            let mut v = Voice::new(FS);
            v.reverb_mix = mix;
            for n in 0..2_000 {
                v.process(if n < 50 { 30_000.0 } else { 0.0 });
            }
            (0..4_000)
                .map(|_| (v.process(0.0) as f64).abs())
                .sum::<f64>()
        };
        assert!(
            tail(0.3) > 10.0 * tail(0.0),
            "{} vs {}",
            tail(0.3),
            tail(0.0)
        );
    }

    #[test]
    fn voice_lifts_an_idle_level_and_leaves_full_load_alone() {
        // 200 Hz pressure tones: one ~30 dB under full load, one at full load
        let out_rms = |amp: f64, leveling: f64| {
            let mut v = Voice::new(FS);
            v.leveler.amount = leveling;
            let n = (6.0 * FS) as usize;
            let mut acc = 0.0;
            for i in 0..n {
                let y = v.process(amp * (2.0 * PI * 200.0 * i as f64 / FS).sin()) as f64;
                if i > n - FS as usize {
                    acc += y * y;
                }
            }
            (acc / FS).sqrt()
        };
        let quiet = out_rms(1_000.0, 1.0) / out_rms(1_000.0, 0.0);
        assert!((quiet - LEVELER_MAX_GAIN).abs() < 0.05, "idle lift {quiet}");
        let loud = out_rms(40_000.0, 1.0) / out_rms(40_000.0, 0.0);
        assert!((loud - 1.0).abs() < 0.01, "full load {loud}");
    }

    #[test]
    fn voice_silence_is_silence_and_nan_does_not_poison() {
        let mut v = Voice::new(FS);
        assert_eq!(v.process(0.0), 0.0);
        assert_eq!(v.process(f64::NAN), 0.0);
        for _ in 0..100 {
            assert!(v.process(1000.0).is_finite());
        }
    }

    proptest! {
        #[test]
        fn voice_output_bounded(xs in proptest::collection::vec(-1e7f64..1e7, 1..500)) {
            let mut v = Voice::new(FS);
            for x in xs {
                let y = v.process(x);
                prop_assert!(y.is_finite() && y.abs() <= 1.0);
            }
        }
    }
}
