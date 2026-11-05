//! The intake's air cleaner: the box the engine draws its air through, its snorkel and its
//! filter element, as a lumped acoustic filter between the flow the engine draws and the flow
//! at the snorkel's mouth (what radiates).
//!
//! Below the box's first standing wave the parts are lumped (Kinsler et al., *Fundamentals of
//! Acoustics*, §10.8; Munjal, *Acoustics of Ducts and Mufflers*, ch. 2): the box a compliance
//! C = V/(ρc²), the snorkel an inertance M = ρ·L_eff/A (end corrections 1.45·r, as
//! [`crate::intake_fv::AirboxDef`]), the filter element a resistance R in series with it. With
//! the drawn volume flow q_in into the box and q_out through the snorkel,
//! q_out/q_in = 1/(1 + sRC + s²MC): a Helmholtz resonance, damped by the filter, and a
//! low-pass above it. Past the box's first axial mode (the box taken as a cube, c/2L) the
//! lumped parts no longer hold and a box passes as an expansion chamber does on average, at
//! its peak loss 20·log₁₀((m + 1/m)/2), m the box's section over the snorkel's (Munjal §3.3):
//! a flat path of that gain, high-passed at the mode, in parallel. A diesel truck's standard
//! air cleaner took 8.5 dB(A) off its open intake (Raff & Perry, JSV 28(3), 1973, Fig. 14).

use serde::{Deserialize, Serialize};

/// Ambient air: density, kg/m³, and speed of sound, m/s (20 °C).
const RHO: f64 = 1.204;
const C: f64 = 343.0;
/// End corrections of the snorkel, × its radius (one flanged end, one unflanged).
const END_CORRECTION: f64 = 1.45;
/// A clean paper element's restriction at the engine's rated air flow, Pa (ESTIMATE: new
/// filters are rated at 1–2 kPa there; linearised).
pub const RATED_RESTRICTION_PA: f64 = 1500.0;
/// Box volume per litre of displacement (ESTIMATE: production boxes of 1.5–2.5 L engines hold
/// 6–12 L; the K20 preset's 8 L for 2.0 L).
pub const BOX_PER_DISPLACEMENT: f64 = 4.0;
/// Snorkel length, m (ESTIMATE, a typical inlet duct).
pub const SNORKEL_LENGTH_M: f64 = 0.3;

/// The air cleaner as built. `fitted: false`: a bare mouth.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct AirCleanerDef {
    pub fitted: bool,
    /// Box volume, m³ (0: the rule, [`AirCleanerDef::rule_sized`]).
    pub volume_m3: f64,
    pub snorkel_length_m: f64,
    /// Snorkel bore, m (0: the throttle's).
    pub snorkel_diameter_m: f64,
    /// The element's flow resistance, Pa per m³/s (0 with `volume_m3` 0: the rule).
    pub filter_resistance_pa_s_m3: f64,
}

impl Default for AirCleanerDef {
    /// Fitted, sized by the rule from the engine at build ([`AirCleanerDef::resolved`]).
    fn default() -> Self {
        Self {
            fitted: true,
            volume_m3: 0.0,
            snorkel_length_m: SNORKEL_LENGTH_M,
            snorkel_diameter_m: 0.0,
            filter_resistance_pa_s_m3: 0.0,
        }
    }
}

impl AirCleanerDef {
    /// The rule-sized cleaner of an engine of `displacement_m3` revving to `redline_rpm` with
    /// a throttle of `throttle_d`: a box of [`BOX_PER_DISPLACEMENT`] × its displacement, a
    /// snorkel of the throttle's bore, an element restricting [`RATED_RESTRICTION_PA`] at the
    /// rated flow displacement·redline/120.
    pub fn rule_sized(displacement_m3: f64, redline_rpm: f64, throttle_d: f64) -> Self {
        let rated = displacement_m3 * redline_rpm / 120.0;
        Self {
            fitted: true,
            volume_m3: BOX_PER_DISPLACEMENT * displacement_m3,
            snorkel_length_m: SNORKEL_LENGTH_M,
            snorkel_diameter_m: throttle_d,
            filter_resistance_pa_s_m3: if rated > 0.0 {
                RATED_RESTRICTION_PA / rated
            } else {
                0.0
            },
        }
    }

    /// This definition with its zeros filled by the rule for that engine.
    pub fn resolved(&self, displacement_m3: f64, redline_rpm: f64, throttle_d: f64) -> Self {
        let rule = Self::rule_sized(displacement_m3, redline_rpm, throttle_d);
        Self {
            fitted: self.fitted,
            volume_m3: if self.volume_m3 > 0.0 {
                self.volume_m3
            } else {
                rule.volume_m3
            },
            snorkel_length_m: self.snorkel_length_m.max(0.0),
            snorkel_diameter_m: if self.snorkel_diameter_m > 0.0 {
                self.snorkel_diameter_m
            } else {
                rule.snorkel_diameter_m
            },
            filter_resistance_pa_s_m3: if self.volume_m3 > 0.0 {
                self.filter_resistance_pa_s_m3
            } else {
                rule.filter_resistance_pa_s_m3
            },
        }
    }

    /// The snorkel's mouth area, m².
    pub fn mouth_area(&self) -> f64 {
        std::f64::consts::PI * 0.25 * self.snorkel_diameter_m * self.snorkel_diameter_m
    }

    fn lumped(&self) -> (f64, f64, f64) {
        let a = self.mouth_area().max(1e-9);
        let r_s = 0.5 * self.snorkel_diameter_m;
        let m = RHO * (self.snorkel_length_m + END_CORRECTION * r_s) / a;
        let c = self.volume_m3 / (RHO * C * C);
        (m, c, self.filter_resistance_pa_s_m3)
    }
}

/// The box's Helmholtz frequency, Hz: 1/(2π√(MC)).
pub fn helmholtz_hz(def: &AirCleanerDef) -> f64 {
    let (m, c, _) = def.lumped();
    1.0 / (std::f64::consts::TAU * (m * c).sqrt())
}

/// The cleaner as a filter on the drawn volume flow, bilinear (pre-warped at the resonance).
#[derive(Debug, Clone)]
pub struct AirCleaner {
    b: [f64; 3],
    a: [f64; 2],
    x: [f64; 2],
    y: [f64; 2],
    /// The chamber path: its gain 2/(m + 1/m), and a one-pole high-pass at the box's first
    /// axial mode (coefficient, last input, last output).
    chamber: f64,
    hp: (f64, f64, f64),
}

impl AirCleaner {
    pub fn new(def: &AirCleanerDef, dt: f64) -> Self {
        let (m, c, r) = def.lumped();
        let w0 = 1.0 / (m * c).sqrt();
        // s → k·(1 − z⁻¹)/(1 + z⁻¹), k pre-warped so the resonance stays put
        let k = w0 / (w0 * dt * 0.5).tan();
        let (a2, a1) = (m * c, r * c);
        let d0 = a2 * k * k + a1 * k + 1.0;
        Self {
            b: [1.0 / d0, 2.0 / d0, 1.0 / d0],
            a: [
                (2.0 - 2.0 * a2 * k * k) / d0,
                (a2 * k * k - a1 * k + 1.0) / d0,
            ],
            x: [0.0; 2],
            y: [0.0; 2],
            chamber: {
                let side = def.volume_m3.cbrt();
                let m = side * side / def.mouth_area().max(1e-9);
                2.0 / (m + 1.0 / m)
            },
            hp: {
                let f_mode = C / (2.0 * def.volume_m3.cbrt().max(1e-3));
                let w = (std::f64::consts::PI * f_mode * dt).tan();
                ((1.0 - w) / (1.0 + w), 0.0, 0.0)
            },
        }
    }

    /// One step: the volume flow at the snorkel's mouth for `q_in` drawn into the box.
    pub fn process(&mut self, q_in: f64) -> f64 {
        let y = self.b[0] * q_in + self.b[1] * self.x[0] + self.b[2] * self.x[1]
            - self.a[0] * self.y[0]
            - self.a[1] * self.y[1];
        self.x = [q_in, self.x[0]];
        self.y = [y, self.y[0]];
        // the chamber's path above the mode: y = a·(y₁ + x − x₁) (bilinear first-order high-pass)
        let (a, x1, y1) = self.hp;
        let h = 0.5 * (1.0 + a) * (q_in - x1) + a * y1;
        self.hp = (a, q_in, h);
        y + self.chamber * h
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_resonates_where_helmholtz_says() {
        // 8 L box, 60 mm × 0.3 m snorkel (end corrections 1.45·r: L_eff = 0.3435 m), no filter:
        // f_H = c/2π·√(A/(V·L_eff)) = 343/6.2832·√(2.8274e-3/(8e-3·0.3435)) = 54.6·1.0143
        // = 55.4 Hz; at four times it the box passes |1/(1 − 16)| = 1/15 (−23.5 dB)
        let def = AirCleanerDef {
            fitted: true,
            volume_m3: 8e-3,
            snorkel_length_m: 0.3,
            snorkel_diameter_m: 0.06,
            filter_resistance_pa_s_m3: 0.0,
        };
        let f_h = helmholtz_hz(&def);
        assert!((f_h - 55.4).abs() < 0.3, "{f_h}");
        // the response at f alone (a lossless box started by a sine rings at f_H for ever:
        // that ringing is left out by projecting the output on f)
        let g = |f: f64| {
            let dt = 1.0 / 48_000.0;
            let mut b = AirCleaner::new(&def, dt);
            let n = (2.0 / dt) as usize;
            let (mut re, mut im) = (0.0, 0.0);
            for k in 0..n {
                let ph = std::f64::consts::TAU * f * k as f64 * dt;
                let y = b.process(ph.sin());
                if k > n / 2 {
                    re += y * ph.sin();
                    im += y * ph.cos();
                }
            }
            let amp = 2.0 * (re * re + im * im).sqrt() / (n - n / 2 - 1) as f64;
            20.0 * amp.log10()
        };
        assert!((g(4.0 * 55.4) + 23.5).abs() < 1.0, "{}", g(4.0 * 55.4));
        assert!(
            g(55.4) > 10.0,
            "a lossless box rings at its resonance: {}",
            g(55.4)
        );
    }

    #[test]
    fn the_filter_damps_the_resonance() {
        let mut def = AirCleanerDef::rule_sized(2.0e-3, 7000.0, 0.06);
        let ring = |d: &AirCleanerDef| {
            let dt = 1.0 / 48_000.0;
            let mut b = AirCleaner::new(d, dt);
            let f = helmholtz_hz(d);
            let n = (2.0 / dt) as usize;
            let mut eo = 0.0;
            for k in 0..n {
                let y = b.process((std::f64::consts::TAU * f * k as f64 * dt).sin());
                if k > n / 2 {
                    eo += y * y;
                }
            }
            eo
        };
        let with = ring(&def);
        def.filter_resistance_pa_s_m3 = 0.0;
        assert!(with < 0.5 * ring(&def));
    }

    #[test]
    fn the_rule_sizes_by_hand() {
        // 2.0 L, 7000 rev/min: rated flow 2e-3·7000/120 = 0.1167 m³/s; 1.5 kPa of restriction
        // there is 12 857 Pa·s/m³; the box 4 × 2 L = 8 L; the snorkel the throttle's bore
        let d = AirCleanerDef::rule_sized(2.0e-3, 7000.0, 0.06);
        assert!((d.volume_m3 - 8e-3).abs() < 1e-12);
        assert!((d.filter_resistance_pa_s_m3 - 12_857.1).abs() < 0.5);
        assert!((d.snorkel_diameter_m - 0.06).abs() < 1e-12);
    }

    #[test]
    fn above_its_modes_a_box_passes_as_an_expansion_chamber() {
        // An 8 L box taken as a cube: 0.2 m a side, 0.04 m² of section against the 60 mm
        // snorkel's 2.827e-3 m²: m = 14.15. Past its first axial mode (343/0.4 = 858 Hz) the
        // lumped low-pass no longer holds; an expansion chamber's transmission averages its
        // peak loss 20·log₁₀((m + 1/m)/2) = 17.0 dB (Munjal, *Acoustics of Ducts and Mufflers*,
        // §3.3): |H| at 4 kHz within ±3 dB of −17.0, not the lumped −74 dB.
        let def = AirCleanerDef {
            fitted: true,
            volume_m3: 8e-3,
            snorkel_length_m: 0.3,
            snorkel_diameter_m: 0.06,
            filter_resistance_pa_s_m3: 0.0,
        };
        let dt = 1.0 / 96_000.0;
        let f = 4000.0;
        let mut b = AirCleaner::new(&def, dt);
        let n = (1.0 / dt) as usize;
        let (mut re, mut im) = (0.0, 0.0);
        for k in 0..n {
            let ph = std::f64::consts::TAU * f * k as f64 * dt;
            let y = b.process(ph.sin());
            if k > n / 2 {
                re += y * ph.sin();
                im += y * ph.cos();
            }
        }
        let db = 20.0 * (2.0 * (re * re + im * im).sqrt() / (n - n / 2 - 1) as f64).log10();
        assert!((db + 17.0).abs() < 3.0, "{db:.1} dB");
    }
}
