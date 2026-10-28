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

/// Peak of the fitted response, dB re 20 µPa at 1 m for 1 kPa (before the shape terms).
const GAIN_DB: f64 = 107.85;
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

/// What drives the block's response: one cylinder's pressure, the engine's own number of
/// cylinders already in the measured attenuation (Anderton, *Relation between combustion system
/// and engine noise*, SAE 790270, eq. 3: P² = (cylinder pressure × bore²)²/S, with S measured on
/// whole engines against one cylinder's pressure). At the firing harmonics the cylinders' mean
/// is any one cylinder's pressure, so the mean of `cylinders_pa` drives it, and their firing
/// pattern stays in the sound. The curve is for a 100 mm bore and the noise grows with the
/// bore's fifth power (Anderton: the direct force with its fourth, the attenuation falling with
/// the first), so the drive scales by (bore / 0.1 m)^2.5. Pa.
pub fn combustion_drive(cylinders_pa: impl IntoIterator<Item = f64>, bore_m: f64) -> f64 {
    let (sum, n) = cylinders_pa
        .into_iter()
        .fold((0.0, 0usize), |(s, n), p| (s + p, n + 1));
    if n == 0 {
        return 0.0;
    }
    sum / n as f64 * (bore_m / 0.1).powf(2.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_block_is_driven_by_one_cylinders_pressure_scaled_to_the_bore() {
        // Anderton (SAE 790270) eq. 3: P² = (cylinder pressure × bore²)²/S, one cylinder's
        // pressure (its engine's noise already in S), S measured at 100 mm, noise ∝ bore⁵:
        // ten equal 2 bar cylinders of 88 mm drive it as 2 bar × 0.88^2.5 = 2e5 × 0.726452
        let ten = [2e5; 10];
        assert!((combustion_drive(ten, 0.088) - 145_290.4).abs() < 1.0);
        // at 100 mm, the mean pressure itself
        assert!((combustion_drive([1e5, 3e5], 0.1) - 2e5).abs() < 1e-9);
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
        // 1 m for 1 kPa; the fit stays within 5.5 dB of it (worst 5 dB at 250 Hz, where the
        // engines spread ±10 dB)
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
}
