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

/// Fourth-order Butterworth low-pass (two biquads, Q = 0.5412 and 1.3066): the anti-alias
/// filter before keeping one physics step in `SUBSTEPS` as an audio sample. Taking every 4th
/// sample of a 192 kHz signal unfiltered folds everything between 24 and 96 kHz back into the
/// audible band.
#[derive(Debug, Clone)]
pub struct AntiAlias {
    stages: [LowPass; 2],
}

impl AntiAlias {
    pub fn new(cutoff_hz: f64, sample_rate: f64) -> Self {
        Self {
            stages: [
                LowPass::new(cutoff_hz, 0.541_196, sample_rate),
                LowPass::new(cutoff_hz, 1.306_563, sample_rate),
            ],
        }
    }
    pub fn process(&mut self, x: f64) -> f64 {
        let y = self.stages[0].process(x);
        self.stages[1].process(y)
    }
}

/// How far ahead the limiter looks, s: long enough to ramp its gain down over a firing pulse's
/// rise without a click, short enough to stay unheard as latency.
pub const LIMITER_LOOKAHEAD_S: f64 = 0.005;
/// Time constant of the limiter's recovery, s.
pub const LIMITER_RELEASE_S: f64 = 0.15;
/// Highest sample the limiter lets out (−1 dBFS).
pub const LIMITER_THRESHOLD: f64 = 0.891;

/// A look-ahead peak limiter: the output stage's protection against overload, which turns the
/// level down for a moment instead of bending the wave (a soft clip adds harmonics: heard as
/// distortion). The gain each sample needs, min(1, T/|x|), is held at its minimum over the
/// look-ahead window, recovers with [`LIMITER_RELEASE_S`], and is averaged over the window while
/// the signal waits as long; every gain the average covers already holds the sample it meets,
/// so no output exceeds [`LIMITER_THRESHOLD`] (the brick-wall look-ahead design, e.g.
/// G. Massberg / Signalsmith "limiter" notes). Below the threshold it is a pure delay.
/// Fixed buffers: nothing allocates after `new`.
#[derive(Debug, Clone)]
pub struct Limiter {
    n: usize,
    /// The signal waiting its turn: a ring of n − 1 samples (the window's last sample is the
    /// one each average covers in all its windows).
    delay: Vec<f64>,
    /// The window's needed gains as a monotonic queue (rising from front to back): positions in
    /// a ring of n, with their sample index.
    queue: Vec<(u64, f64)>,
    head: usize,
    len: usize,
    /// The held, recovering gain's last n values (ring) and their running sum.
    held: Vec<f64>,
    sum: f64,
    release: f64,
    last: f64,
    t: u64,
}

impl Limiter {
    pub fn new(sample_rate: f64) -> Self {
        let n = ((LIMITER_LOOKAHEAD_S * sample_rate).round() as usize).max(1);
        Self {
            n,
            delay: vec![0.0; n - 1],
            queue: vec![(0, 1.0); n],
            head: 0,
            len: 0,
            held: vec![1.0; n],
            sum: n as f64,
            release: 1.0 - (-1.0 / (LIMITER_RELEASE_S * sample_rate)).exp(),
            last: 1.0,
            t: 0,
        }
    }

    /// Samples between a sample going in and coming out.
    pub fn latency(&self) -> usize {
        self.n - 1
    }

    pub fn process(&mut self, x: f64) -> f64 {
        let n = self.n;
        let t = self.t;
        let need = if x.abs() > LIMITER_THRESHOLD {
            LIMITER_THRESHOLD / x.abs()
        } else {
            1.0
        };
        // sliding minimum of the needed gain over the last n samples
        while self.len > 0 && self.queue[(self.head + self.len - 1) % n].1 >= need {
            self.len -= 1;
        }
        self.queue[(self.head + self.len) % n] = (t, need);
        self.len += 1;
        if self.queue[self.head].0 + n as u64 <= t {
            self.head = (self.head + 1) % n;
            self.len -= 1;
        }
        let hold = self.queue[self.head].1;
        // recover towards 1, never above what the window holds
        let g = hold.min(self.last + (1.0 - self.last) * self.release);
        self.last = g;
        // average over the window: the gain that meets the sample now leaving
        let slot = (t % n as u64) as usize;
        self.sum += g - self.held[slot];
        self.held[slot] = g;
        if slot == 0 {
            // a fresh sum each window, so rounding cannot drift
            self.sum = self.held.iter().sum();
        }
        let out = if self.delay.is_empty() {
            x
        } else {
            let k = (t % self.delay.len() as u64) as usize;
            std::mem::replace(&mut self.delay[k], x)
        };
        self.t += 1;
        out * (self.sum / n as f64)
    }
}

/// Sound pressure that plays at full scale: 0 dBFS = 120 dB SPL (20 µPa reference), so an
/// engine heard at 100 dB plays at −20 dBFS and the levels between engines and operating points
/// stay the physical ones.
pub const FULL_SCALE_PA: f64 = 20.0;

/// The engine voice: sound pressure at the listener (Pa, from the physics, already radiated)
/// → speaker sample in [-1, 1]. It only reproduces: DC block (a speaker cannot hold a static
/// pressure), optional room reverb and leveler, a fixed calibration, and a look-ahead limiter
/// against overload ([`Limiter`]: it turns the level down, never bends the wave). No tone shaping.
#[derive(Debug, Clone)]
pub struct Voice {
    dc: DcBlocker,
    reverb: Reverb,
    limiter: Limiter,
    /// Wet level of the body/room reverb (0 = dry).
    pub reverb_mix: f64,
    sample_rate: f64,
    /// Pressure that maps to full scale before soft clipping, Pa.
    pub full_scale_pa: f64,
    /// Lifts idle toward the full-load level (reference: a full-load engine, ≈ −15 dBFS RMS).
    pub leveler: Leveler,
    /// Whether the look-ahead limiter holds the output under its threshold. Live audio needs it
    /// (a device clips); an offline render turns it off and scales the whole scene once, so its
    /// loud passages keep their peaks.
    pub limit: bool,
}

/// Output level (sine amplitude, of full scale) the leveler treats as full load.
pub const LEVELER_REFERENCE: f64 = 0.25;

impl Voice {
    pub fn new(sample_rate: f64) -> Self {
        Self {
            dc: DcBlocker::new(3.0, sample_rate),
            reverb: Reverb::new(sample_rate, 0.35),
            limiter: Limiter::new(sample_rate),
            reverb_mix: 0.0,
            sample_rate,
            full_scale_pa: FULL_SCALE_PA,
            leveler: Leveler::new(LEVELER_REFERENCE, sample_rate),
            limit: true,
        }
    }
    pub fn process(&mut self, pressure_pa: f64) -> f32 {
        if !pressure_pa.is_finite() {
            return 0.0;
        }
        let dry = self.dc.process(pressure_pa);
        let y = self
            .leveler
            .process((dry + self.reverb_mix * self.reverb.process(dry)) / self.full_scale_pa);
        if !y.is_finite() {
            // A runaway filter state is reset rather than left to poison every later sample.
            let amount = self.leveler.amount;
            *self = Voice {
                full_scale_pa: self.full_scale_pa,
                limit: self.limit,
                ..Voice::new(self.sample_rate)
            };
            self.leveler.amount = amount;
            return 0.0;
        }
        if self.limit {
            self.limiter.process(y) as f32
        } else {
            y as f32
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    const FS: f64 = 48_000.0;

    #[test]
    fn a_voice_without_its_limiter_passes_loud_passages_untouched() {
        // offline, nothing is squeezed: twice the pressure is twice the output, even far past
        // full scale (the limiter would have held it at its threshold)
        let mut a = Voice::new(FS);
        let mut b = Voice::new(FS);
        a.limit = false;
        b.limit = false;
        let mut worst: f64 = 0.0;
        let mut peak: f64 = 0.0;
        for i in 0..4800 {
            let x = 60.0 * (2.0 * PI * 200.0 * i as f64 / FS).sin();
            let (ya, yb) = (a.process(x) as f64, b.process(2.0 * x) as f64);
            worst = worst.max((yb - 2.0 * ya).abs());
            peak = peak.max(ya.abs());
        }
        assert!(peak > 2.0, "{peak}");
        assert!(worst < 1e-5, "{worst}");
    }

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
    fn anti_alias_passes_the_audio_band_and_stops_what_would_fold() {
        // 4th-order Butterworth at 20 kHz, run at 192 kHz: flat in the band, −3 dB at the cut,
        // and at 40 kHz (an octave up) at least the analog |H| = 1/√(1 + 2⁸) = 0.0624 (−24 dB):
        // the bilinear transform only steepens it towards Nyquist
        let fs = 192_000.0;
        let gain = |f: f64| {
            let mut a = AntiAlias::new(20_000.0, fs);
            let n = fs as usize / 10;
            let ys: Vec<f64> = (0..n)
                .map(|i| a.process((2.0 * PI * f * i as f64 / fs).sin()))
                .skip(n / 2)
                .collect();
            (2.0 * ys.iter().map(|y| y * y).sum::<f64>() / ys.len() as f64).sqrt()
        };
        assert_relative_eq!(gain(1_000.0), 1.0, max_relative = 0.005);
        assert_relative_eq!(
            gain(20_000.0),
            std::f64::consts::FRAC_1_SQRT_2,
            max_relative = 0.03
        );
        assert!(gain(40_000.0) < 0.0624, "40 kHz: {}", gain(40_000.0));
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
            v.leveler.amount = 0.0;
            v.reverb_mix = mix;
            // a 4 Pa (≈ 106 dB) burst: one period of 1 kHz, zero mean like any radiated sound
            // (a one-sided pulse would leave the DC blocker's own slow recovery as a "tail")
            for n in 0..2_000 {
                let x = if n < 48 {
                    4.0 * (2.0 * PI * n as f64 / 48.0).sin()
                } else {
                    0.0
                };
                v.process(x);
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
        // ~0.2 Pa (≈ 80 dB) idle against ~9 Pa (≈ 113 dB, 0.45 of full scale) full load
        let quiet = out_rms(0.2, 1.0) / out_rms(0.2, 0.0);
        assert!((quiet - LEVELER_MAX_GAIN).abs() < 0.05, "idle lift {quiet}");
        let loud = out_rms(9.0, 1.0) / out_rms(9.0, 0.0);
        assert!((loud - 1.0).abs() < 0.01, "full load {loud}");
    }

    #[test]
    fn voice_plays_pascals_at_a_fixed_calibration_without_tone_shaping() {
        // 0 dBFS = 120 dB SPL = 20 Pa: a 2 Pa tone (100 dB) plays at 0.1 of full scale, at
        // 200 Hz and at 8 kHz alike (under the limiter's threshold the chain is linear)
        for f in [200.0, 8_000.0] {
            let mut v = Voice::new(FS);
            v.leveler.amount = 0.0;
            // amplitude from the rms over whole periods (a sampled 8 kHz sine never hits its peak)
            let ys: Vec<f64> = (0..FS as usize)
                .map(|i| v.process(2.0 * (2.0 * PI * f * i as f64 / FS).sin()) as f64)
                .skip(FS as usize / 2)
                .collect();
            let amp = (2.0 * ys.iter().map(|y| y * y).sum::<f64>() / ys.len() as f64).sqrt();
            assert_relative_eq!(amp, 0.1, max_relative = 0.01);
        }
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

    #[test]
    fn voice_keeps_its_calibration_through_a_reset() {
        // a runaway resets the filters but not the full scale it was given: a 2 Pa tone against 4 Pa
        // full scale peaks at 2/4 = 0.5 after it
        let mut v = Voice::new(FS);
        v.full_scale_pa = 4.0;
        v.leveler.amount = 0.0;
        v.reverb_mix = 0.0;
        // a finite input past what the filters can hold overflows them: the voice resets
        // (the DC blocker's difference of −MAX and MAX is infinite)
        v.process(-f64::MAX);
        assert_eq!(v.process(f64::MAX), 0.0);
        v.leveler.amount = 0.0;
        // the peak of a 2 Pa, 200 Hz tone, once the filters have settled
        let peak = (0..FS as usize)
            .map(|i| v.process(2.0 * (2.0 * PI * 200.0 * i as f64 / FS).sin()) as f64)
            .skip(FS as usize / 2)
            .fold(0.0f64, |m, y| m.max(y.abs()));
        assert_relative_eq!(peak, 0.5, max_relative = 0.01);
    }

    fn sine(amp: f64, f: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| amp * (2.0 * PI * f * i as f64 / FS).sin())
            .collect()
    }

    #[test]
    fn the_limiter_passes_what_fits_untouched_only_later() {
        // a tone under the threshold comes out sample for sample, LIMITER_LOOKAHEAD_S late
        let mut lim = Limiter::new(FS);
        let x = sine(0.5, 200.0, FS as usize);
        let y: Vec<f64> = x.iter().map(|&v| lim.process(v)).collect();
        let d = lim.latency();
        assert_eq!(d, (LIMITER_LOOKAHEAD_S * FS).round() as usize - 1);
        for i in d..x.len() {
            assert_eq!(y[i], x[i - d], "sample {i}");
        }
    }

    #[test]
    fn the_limiter_holds_a_loud_tone_under_its_threshold_without_distorting_it() {
        // a tone 3.5× past the threshold: no sample above it, and once the gain settles the tone
        // is a clean, scaled sine (its 3rd harmonic 60 dB down; a tanh clip leaves ~−15 dB)
        let mut lim = Limiter::new(FS);
        let y: Vec<f64> = sine(3.0, 200.0, 2 * FS as usize)
            .iter()
            .map(|&v| lim.process(v))
            .collect();
        assert!(y.iter().all(|v| v.abs() <= LIMITER_THRESHOLD + 1e-12));
        let tail = &y[FS as usize..];
        let tone = |f: f64| {
            let (c, s) = tail.iter().enumerate().fold((0.0, 0.0), |(c, s), (i, v)| {
                let w = 2.0 * PI * f * i as f64 / FS;
                (c + v * w.cos(), s + v * w.sin())
            });
            (c * c + s * s).sqrt()
        };
        assert!(
            tone(600.0) < 1e-3 * tone(200.0),
            "3rd harmonic {}",
            tone(600.0) / tone(200.0)
        );
        let peak = tail.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        assert!(peak > 0.97 * LIMITER_THRESHOLD, "settled peak {peak}");
    }

    #[test]
    fn the_limiter_lets_go_after_a_burst() {
        // a loud burst, then a quiet tone: within five release times the tone is back at full
        let mut lim = Limiter::new(FS);
        for v in sine(5.0, 200.0, FS as usize / 2) {
            lim.process(v);
        }
        let n = (5.0 * LIMITER_RELEASE_S * FS) as usize;
        let y: Vec<f64> = sine(0.5, 200.0, n + FS as usize / 10)
            .iter()
            .map(|&v| lim.process(v))
            .collect();
        let peak = y[n..].iter().fold(0.0f64, |m, v| m.max(v.abs()));
        assert!((peak - 0.5).abs() < 0.01, "peak {peak}");
    }

    proptest! {
        #[test]
        fn the_limiter_never_lets_a_sample_past_its_threshold(
            xs in proptest::collection::vec(-1e4f64..1e4, 1..3000)
        ) {
            let mut lim = Limiter::new(FS);
            for x in xs {
                let y = lim.process(x);
                prop_assert!(y.abs() <= LIMITER_THRESHOLD * (1.0 + 1e-9), "{}", y);
            }
        }

        #[test]
        fn voice_output_bounded(xs in proptest::collection::vec(-1e7f64..1e7, 1..500)) {
            let mut v = Voice::new(FS);
            for x in xs {
                let y = v.process(x);
                prop_assert!(y.is_finite() && y.abs() <= 1.0);
            }
        }
    }

    /// Steady-state gain of `f` Hz through `step` at FS: RMS of 2 s after 1 s of settling,
    /// over a unit-amplitude sine (integer cycles for the integer-Hz tones used), ×√2.
    fn steady_gain(mut step: impl FnMut(f64) -> f64, f: f64) -> f64 {
        let w = 2.0 * PI * f / FS;
        let mut acc = 0.0;
        for n in 0..(3 * 48_000) {
            let y = step((w * n as f64).sin());
            if n >= 48_000 {
                acc += y * y;
            }
        }
        (acc / 96_000.0).sqrt() * 2f64.sqrt()
    }

    #[test]
    fn the_dc_blocker_is_3_db_down_at_its_corner() {
        // A first-order high-pass H = s/(s + ωc) has |H(jωc)| = 1/√2 = 0.70711 by definition of its
        // corner (J. O. Smith, *Introduction to Digital Filters*, "DC Blocker"; the −3 dB point).
        // The digital pole R = 1 − ωc/fs places it within O(ωc/fs); tolerance 1 % covers that for
        // fc = 3 Hz (ω = 3.9e-4 rad) and 100 Hz (ω = 0.013 rad).
        for fc in [3.0, 100.0] {
            let mut dc = DcBlocker::new(fc, FS);
            let g = steady_gain(|x| dc.process(x), fc);
            assert_relative_eq!(g, std::f64::consts::FRAC_1_SQRT_2, max_relative = 0.01);
        }
    }

    #[test]
    fn the_biquad_low_pass_gain_at_its_cutoff_is_q() {
        // RBJ cookbook low-pass (R. Bristow-Johnson, *Audio EQ Cookbook*) is the analog
        // H(s) = 1/(s² + s/Q + 1) at s = jω/ω0, whose gain at ω = ω0 is |1/(j/Q)| = Q exactly (the
        // bilinear transform in the cookbook is pre-warped at ω0, so the digital gain there is Q too).
        // Q = 1/√2: −3.01 dB (0.7071); Q = 2: +6.02 dB (2.0). Tolerance 0.5 % (the RMS read).
        for (q, expected) in [(std::f64::consts::FRAC_1_SQRT_2, 0.70711), (2.0, 2.0)] {
            let mut lp = LowPass::new(1000.0, q, FS);
            let g = steady_gain(|x| lp.process(x), 1000.0);
            assert_relative_eq!(g, expected, max_relative = 0.005);
        }
    }
}
