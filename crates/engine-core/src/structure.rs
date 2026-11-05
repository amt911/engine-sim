//! The engine block as a radiator: the cylinder pressure shakes the head, liner and block
//! walls, and their surfaces radiate it as combustion noise. Measured on running and
//! "banger-rig" engines as a structure response function (SRF): the sound pressure level at
//! 1 m, dB re 20 µPa, for 1 kPa of cylinder pressure at each frequency; its complement is the
//! structure attenuation, SA = 154 dB − SRF (Priede; Anderton, SAE 790270 — the Lucas filter
//! of the combustion noise meters is the average of such curves).
//!
//! The curve here is the median of the nine engines (DI, IDI and HSDI diesels, 0.5 to over
//! 1.2 L/cylinder) in the classic figure reproduced in *A Review of Structural Attenuation in
//! Combustion Engines*, Hidraulica 3/2021, Fig. 1, read at the 1/3-octave centres 125 Hz to
//! 10 kHz; the engines spread about ±8 dB around it. Fitted by a 7th-order minimum-phase
//! response (rms error 2.1 dB): three zeros at DC, three poles at `FA_HZ`, one at `FB_HZ`, a
//! damped pair at `FC_HZ` — a block passes almost nothing of the quasi-static pressure (its
//! stiffness carries it to the crank), rises with frequency as its walls go modal and falls
//! off above a few kHz as their radiation is mass-controlled.

use std::f64::consts::PI;

/// Peak of the fitted response, dB re 20 µPa at 1 m for 1 kPa (before the shape terms): the
/// review's median (107.85) raised by 6.3 dB, so that a gasoline engine's average cylinder
/// pressure spectrum gives Anderton's own prediction of its combustion noise (SAE 790270
/// eq. 9, ±2.5 dB(A) against measured engines; test
/// `a_petrol_pressure_spectrum_makes_andertons_noise`). The review's curve shape stays.
const GAIN_DB: f64 = 114.15;
/// Triple pole of the rising flank, Hz.
const FA_HZ: f64 = 3291.4;
/// Single pole, Hz.
const FB_HZ: f64 = 131.6;
/// Damped pair of the high-frequency roll-off, Hz, and its quality factor.
const FC_HZ: f64 = 2158.6;
const QC: f64 = 0.72;
/// 20 µPa in Pa and 1 kPa in Pa: the SRF's two references.
const P_REF: f64 = 20e-6;
const P_EXCITE: f64 = 1e3;

/// A second-order section, transposed direct form II.
#[derive(Debug, Clone)]
struct Section {
    b: [f64; 3],
    a: [f64; 2],
    z: [f64; 2],
}

impl Section {
    /// The analog (b₀ + b₁s + b₂s²)/(a₀ + a₁s + a₂s²) by the bilinear transform,
    /// s = (2/dt)(1 − z⁻¹)/(1 + z⁻¹).
    fn bilinear(b: [f64; 3], a: [f64; 3], dt: f64) -> Self {
        let k = 2.0 / dt;
        let k2 = k * k;
        let n = |c: [f64; 3]| {
            [
                c[0] + c[1] * k + c[2] * k2,
                2.0 * c[0] - 2.0 * c[2] * k2,
                c[0] - c[1] * k + c[2] * k2,
            ]
        };
        let (nb, na) = (n(b), n(a));
        Self {
            b: [nb[0] / na[0], nb[1] / na[0], nb[2] / na[0]],
            a: [na[1] / na[0], na[2] / na[0]],
            z: [0.0; 2],
        }
    }

    fn process(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.z[0];
        self.z[0] = self.b[1] * x - self.a[0] * y + self.z[1];
        self.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }
}

/// Cylinder pressure in, the block's radiated sound pressure at 1 m out (both Pa).
#[derive(Debug, Clone)]
pub struct StructureResponse {
    sections: [Section; 3],
    gain: f64,
}

impl StructureResponse {
    /// Stepped every `dt` seconds.
    pub fn new(dt: f64) -> Self {
        let wa = 2.0 * PI * FA_HZ;
        let wb = 2.0 * PI * FB_HZ;
        let wc = 2.0 * PI * FC_HZ;
        // (s/ωa)²/(1 + s/ωa)²
        let rise = Section::bilinear(
            [0.0, 0.0, 1.0 / (wa * wa)],
            [1.0, 2.0 / wa, 1.0 / (wa * wa)],
            dt,
        );
        // (s/ωa)/((1 + s/ωa)(1 + s/ωb))
        let edge = Section::bilinear(
            [0.0, 1.0 / wa, 0.0],
            [1.0, 1.0 / wa + 1.0 / wb, 1.0 / (wa * wb)],
            dt,
        );
        // 1/(1 + s/(Q·ωc) + (s/ωc)²)
        let roll = Section::bilinear([1.0, 0.0, 0.0], [1.0, 1.0 / (QC * wc), 1.0 / (wc * wc)], dt);
        Self {
            sections: [rise, edge, roll],
            gain: 10f64.powf(GAIN_DB / 20.0) * P_REF / P_EXCITE,
        }
    }

    /// One step: the cylinder pressure (gauge or absolute: the block passes no DC), Pa; the
    /// sound pressure 1 m from the block, Pa.
    pub fn process(&mut self, cylinder_pa: f64) -> f64 {
        let mut x = cylinder_pa;
        for s in &mut self.sections {
            x = s.process(x);
        }
        self.gain * x
    }
}

/// The block's wall modes, for the part of its response a smoothed SRF cannot hold: which
/// cylinder struck it. All cylinders shake the same modes, each in proportion to the mode's
/// shape where that cylinder sits, so each has its own voice (the same mode set's frequency
/// response at a different point; Priede's and Anderton's curves average those away). Modes
/// are spaced evenly (a plate's modal density is constant with frequency; Cremer & Heckl,
/// *Structure-Borne Sound*), each a unit-peak resonance scaled so their random-phase sum keeps
/// a flat mean power: the SRF, applied after, sets the level. Below the first mode the block
/// moves whole, the same for every cylinder.
#[derive(Debug, Clone)]
pub struct BlockModes {
    modes: Vec<Section>,
    /// `shape[k][c]`: mode `k`'s scaled participation for cylinder `c`.
    shape: Vec<Vec<f64>>,
    rigid: Section,
}

/// First mode and spacing, Hz (ESTIMATE: a block's first wall modes lie near 300 Hz; the spacing
/// is a density that gives each critical band several modes).
const MODE_FIRST_HZ: f64 = 300.0;
const MODE_SPACING_HZ: f64 = 100.0;
const MODE_COUNT: usize = 80;
/// Modal damping ratio (ESTIMATE: of the order of cast iron with bolted joints). T60 = 6.9/(ζω),
/// 37 ms at 1 kHz: a longer one sounds like a room, the failed 90 ms tail of 2026-11-02.
const MODE_DAMPING: f64 = 0.03;

impl BlockModes {
    /// `n_cyl` cylinders in a row, stepped every `dt` seconds.
    pub fn new(n_cyl: usize, dt: f64) -> Self {
        // the golden ratio keeps the frequencies from lining up
        const GOLD: f64 = 0.618_033_988_75;
        let mut modes = Vec::with_capacity(MODE_COUNT);
        let mut shape = Vec::with_capacity(MODE_COUNT);
        let mut seed = 0x9E37_79B9_7F4A_7C15_u64;
        for k in 0..MODE_COUNT {
            let jitter = (k as f64 * GOLD).fract() - 0.5;
            let f = MODE_FIRST_HZ + (k as f64 + jitter) * MODE_SPACING_HZ;
            let w = 2.0 * PI * f;
            let z = MODE_DAMPING;
            // 2ζωs/(s² + 2ζωs + ω²): unit peak; mean power over the spacing πζf/Δ
            let scale = (MODE_SPACING_HZ / (PI * z * f)).sqrt();
            modes.push(Section::bilinear(
                [0.0, 2.0 * z * w, 0.0],
                [w * w, 2.0 * z * w, 1.0],
                dt,
            ));
            // At these orders a mode's value at a point is as good as random, and it has to be:
            // a smooth pattern along the block keeps neighbours in step, and where modes overlap
            // they cancel (11 dB down at 2–4 kHz, measured).
            shape.push(
                (0..n_cyl)
                    .map(|_| {
                        seed ^= seed << 13;
                        seed ^= seed >> 7;
                        seed ^= seed << 17;
                        let draw = (seed >> 11) as f64 / (1u64 << 53) as f64;
                        scale * std::f64::consts::SQRT_2 * (2.0 * PI * draw).cos()
                    })
                    .collect(),
            );
        }
        let w = 2.0 * PI * MODE_FIRST_HZ;
        let rigid = Section::bilinear([1.0, 0.0, 0.0], [1.0, 1.414 / w, 1.0 / (w * w)], dt);
        Self {
            modes,
            shape,
            rigid,
        }
    }

    /// One step: each cylinder's pressure, Pa; the pressure the block's response should see (the
    /// mean of the cylinders, as [`combustion_drive`], with each one's own voice above the
    /// first mode).
    pub fn process(&mut self, cylinders_pa: &[f64]) -> f64 {
        let n = cylinders_pa.len().max(1) as f64;
        let mean = cylinders_pa.iter().sum::<f64>() / n;
        let mut out = self.rigid.process(mean);
        for (m, sh) in self.modes.iter_mut().zip(&self.shape) {
            let u = sh.iter().zip(cylinders_pa).map(|(a, p)| a * p).sum::<f64>() / n;
            out += m.process(u);
        }
        out
    }
}

/// What drives the block's response: one cylinder's pressure, the engine's own number of
/// cylinders already in the measured attenuation (Anderton, *Relation between combustion system
/// and engine noise*, SAE 790270, eq. 3: P² = (cylinder pressure × bore²)²/S, with S measured on
/// whole engines against one cylinder's pressure spectrum). The cylinders' sum over √n keeps
/// that spectrum's power where the noise is (the bands from a few hundred Hz up hold many cycle
/// orders, the cylinders' parts of them add in power: n·P₁/n), and their firing pattern stays in
/// the sound. (Their mean kept only the firing orders, 1/n of each band's power: −6 dB on a
/// four, −10 on a V10.) The curve is for a 100 mm bore and the noise grows with the
/// bore's fifth power (Anderton: the direct force with its fourth, the attenuation falling with
/// the first), so the drive scales by (bore / 0.1 m)^2.5. Pa.
pub fn combustion_drive(cylinders_pa: impl IntoIterator<Item = f64>, bore_m: f64) -> f64 {
    let (sum, n) = cylinders_pa
        .into_iter()
        .fold((0.0, 0usize), |(s, n), p| (s + p, n + 1));
    if n == 0 {
        return 0.0;
    }
    sum / (n as f64).sqrt() * (bore_m / 0.1).powf(2.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mean output power of a unit sine of `f` Hz into cylinder `c` of 4, alone (undoing the mean).
    fn modal_power(f: f64, c: usize) -> f64 {
        let dt = 1.0 / 48_000.0;
        let mut m = BlockModes::new(4, dt);
        let mut input = [0.0; 4];
        let n = 4800;
        let mut sum = 0.0;
        for i in 0..n {
            input[c] = 4.0 * (2.0 * PI * f * i as f64 * dt).sin();
            let y = m.process(&input);
            if i >= n / 2 {
                sum += y * y;
            }
        }
        sum / (n / 2) as f64 * 2.0
    }

    #[test]
    fn the_wall_modes_keep_the_mean_power_of_what_they_replace() {
        // By hand: a unit-peak resonance's noise bandwidth is πζf, so `Δ/(πζf)` scales each to
        // 1 over its spacing, and a shape of √2·cos(random) has mean square 1: unit mean power.
        let mut sum = 0.0;
        let mut count = 0;
        for k in 0..60 {
            for c in 0..4 {
                sum += modal_power(1000.0 + 50.0 * k as f64 + 7.0, c);
                count += 1;
            }
        }
        let db = 10.0 * (sum / count as f64).log10();
        assert!(db.abs() < 2.0, "{db} dB");
    }

    #[test]
    fn below_its_first_mode_the_block_moves_whole() {
        let (a, b) = (modal_power(60.0, 0), modal_power(60.0, 3));
        assert!((10.0 * (a / b).log10()).abs() < 1.0, "{a} {b}");
    }

    #[test]
    fn each_cylinder_strikes_the_block_in_its_own_voice_and_always_the_same_one() {
        let dt = 1.0 / 48_000.0;
        let strike = |c: usize| {
            let mut m = BlockModes::new(4, dt);
            let mut out = Vec::new();
            for i in 0..1200 {
                let mut p = [0.0; 4];
                if i == 0 {
                    p[c] = 4.0;
                }
                out.push(m.process(&p));
            }
            out.windows(2).map(|w| w[1] - w[0]).collect::<Vec<f64>>()
        };
        let corr = |a: &[f64], b: &[f64]| {
            let d = |x: &[f64], y: &[f64]| x.iter().zip(y).map(|(p, q)| p * q).sum::<f64>();
            d(a, b) / (d(a, a) * d(b, b)).sqrt()
        };
        assert_eq!(strike(2), strike(2));
        for (a, b) in [(0, 1), (0, 2), (1, 3), (2, 3)] {
            let r = corr(&strike(a), &strike(b));
            assert!(r.abs() < 0.5, "cylinders {a} and {b}: {r}");
        }
    }

    #[test]
    fn the_block_is_driven_by_one_cylinders_pressure_scaled_to_the_bore() {
        // Anderton (SAE 790270) eq. 3: P² = (cylinder pressure × bore²)²/S, one cylinder's
        // pressure spectrum (its engine's noise already in S), S measured at 100 mm, noise ∝
        // bore⁵; the cylinders summed over √n: ten equal 2 bar cylinders of 88 mm drive it as
        // 10 · 2e5/√10 · 0.88^2.5 = 632 455.5 · 0.726452 = 459 449.1
        let ten = [2e5; 10];
        assert!((combustion_drive(ten, 0.088) - 459_449.1).abs() < 1.0);
        // at 100 mm, two cylinders: (1e5 + 3e5)/√2 = 282 842.7
        assert!((combustion_drive([1e5, 3e5], 0.1) - 282_842.712_5).abs() < 1e-3);
        assert_eq!(combustion_drive([], 0.1), 0.0);
    }

    /// SRF (dB re 20 µPa at 1 m for 1 kPa) of a sine of `f` Hz through the filter at `fs`.
    fn srf_db(f: f64, fs: f64) -> f64 {
        let mut s = StructureResponse::new(1.0 / fs);
        let n = (0.2 * fs) as usize;
        let mut peak = 0.0f64;
        for i in 0..n {
            let y = s.process(P_EXCITE * (2.0 * PI * f * i as f64 / fs).sin());
            if i > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        20.0 * (peak / P_REF).log10()
    }

    #[test]
    fn the_block_radiates_as_the_nine_measured_engines() {
        // digitised median of Hidraulica 3/2021 Fig. 1 (nine diesel engines), dB re 20 µPa at
        // 1 m for 1 kPa, raised by the 6.3 dB that bring it to Anderton's own prediction
        // (`a_petrol_pressure_spectrum_makes_andertons_noise`); the fit stays within 5.5 dB of
        // its shape (worst 5 dB at 250 Hz, where the engines spread ±10 dB)
        let figure = [
            (125.0, 17.0),
            (200.0, 33.0),
            (250.0, 39.0),
            (500.0, 47.0),
            (1000.0, 58.0),
            (2000.0, 64.0),
            (2500.0, 67.0),
            (3150.0, 65.0),
            (5000.0, 55.0),
            (10_000.0, 45.0),
        ];
        for fs in [96_000.0, 192_000.0] {
            for (f, db) in figure {
                let db = db + 6.3;
                let got = srf_db(f, fs);
                assert!(
                    (got - db).abs() < 5.5,
                    "{f} Hz at {fs}: {got:.1} dB vs {db}"
                );
            }
        }
    }

    #[test]
    fn the_quasi_static_pressure_does_not_radiate() {
        // a constant 50 bar settles to nothing, and the 25 Hz of an idling four-stroke single
        // is far below the 1 kHz band (figure: the curves fall ~20 dB per octave below 250 Hz)
        let mut s = StructureResponse::new(1.0 / 96_000.0);
        let mut y = 0.0;
        for _ in 0..96_000 {
            y = s.process(5e6);
        }
        assert!(y.abs() < 1e-6, "{y}");
        assert!(srf_db(25.0, 96_000.0) < srf_db(1000.0, 96_000.0) - 60.0);
    }

    #[test]
    fn a_petrol_pressure_spectrum_makes_andertons_noise() {
        // Anderton, SAE 790270: a gasoline engine's reduced cylinder pressure spectrum is
        // CP_R(1.0) = 133 dB with combustion index n = 6.3 (Fig. 14): the level in each
        // one-third octave is 133 + 10·n·log₁₀(N/f) dB re 20 µPa. Its combustion noise at 1 m
        // is eq. (9): CP_R + 50·log₁₀B + 10·n·log₁₀(N/1000) − K₁, K₁ = 175.5 dB(A) at n = 6.3
        // (Fig. 13, read ±0.5): for B = 86 mm at 3000 rev/min,
        // 133 + 96.7 + 30.1 − 175.5 = 84.3 dB(A).
        // One cylinder's pressure, its cycle harmonics given each band's level, through the
        // block as the engine drives it.
        let dt = 1.0 / 96_000.0;
        let (rpm, bore) = (3000.0, 0.086);
        let f_cycle = rpm / 120.0;
        let mut rng = 0x2545_F491_4F6C_DD1Du64;
        let mut harmonics = Vec::new();
        let mut k = 1usize;
        loop {
            let f = k as f64 * f_cycle;
            if f > 10_000.0 {
                break;
            }
            // the band's power shared among the harmonics in it: a third octave holds
            // 0.2316·f/f_cycle of them
            let band_db = 133.0 + 63.0 * (rpm / f).log10();
            let p_rms = 20e-6 * 10f64.powf(band_db / 20.0) / (0.2316 * f / f_cycle).max(1.0).sqrt();
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            let phase = (rng >> 11) as f64 / (1u64 << 53) as f64 * std::f64::consts::TAU;
            if f >= 100.0 {
                harmonics.push((f, p_rms * std::f64::consts::SQRT_2, phase));
            }
            k += 1;
        }
        let mut block = StructureResponse::new(dt);
        let mut a = crate::mechanical::AWeighting::new(dt);
        let n = (1.0 / dt) as usize;
        let mut sum = 0.0;
        for i in 0..n {
            let t = i as f64 * dt;
            let p: f64 = harmonics
                .iter()
                .map(|&(f, amp, ph)| amp * (std::f64::consts::TAU * f * t + ph).sin())
                .sum();
            let y = a.process(block.process(combustion_drive([p], bore)));
            if i > n / 4 {
                sum += y * y;
            }
        }
        let db = 10.0 * (sum / (n - n / 4 - 1) as f64 / 4e-10).log10();
        assert!((db - 84.3).abs() < 2.0, "{db:.1} dB(A) against Anderton's 84.3");
    }
}
