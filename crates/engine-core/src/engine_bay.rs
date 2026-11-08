//! The engine bay as a leaky enclosure over the head: the hood, wings and bulkhead close it but
//! for its openings (grille, wheel arches, the gaps round the engine), and the valve ticking the
//! head and its cover radiate leaves through them and the panels (the block's skirt and sump
//! radiate into the open underside instead: [`crate::engine`] sends only the ticking here). Its insertion loss for a
//! diffuse field inside, with the panels' own transmission far under the openings',
//! IL = 10·lg(1 + ᾱ/τ̄) (Vér & Beranek, *Noise and Vibration Control Engineering*, 2nd ed.,
//! ch. 12, enclosures): ᾱ the bay's mean absorption, τ̄ its mean transmission, here the open
//! fraction of its surface. The hood's fibre liner is what absorbs, and only in the highs
//! (Delany & Bazley, *Applied Acoustics* 3, 1970: a rigid-backed porous layer); bare metal and
//! the engine hardly do. So the bay takes a few dB of the block's lows and some 14 dB of its
//! valve ticking — measured hoods take 16–27 dB at the positions they shield (Ibarra et al.,
//! "Noise attenuation of the engine compartment of a vehicle", *Int. J. Vehicle Noise and
//! Vibration* 13(2), 2017).

use serde::{Deserialize, Serialize};

/// Air at 20 °C: density, kg/m³, and speed of sound, m/s.
const RHO: f64 = 1.204;
const C: f64 = 343.0;
/// Normal-incidence absorption of painted metal and the engine's castings (ESTIMATE: 0.01–0.02
/// for painted or smooth metal in the usual tables, e.g. Beranek & Vér, ch. 6).
const BARE_ABSORPTION: f64 = 0.02;
/// Frequencies, Hz, at which the shelf standing for the bay's loss is anchored: its floor and
/// ceiling.
const LOW_HZ: f64 = 63.0;
const HIGH_HZ: f64 = 4000.0;

/// An engine bay (data, `serde`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct EngineBay {
    /// Share of the bay's surface open to the outside (grille, arches, gaps underneath).
    pub open_fraction: f64,
    /// The hood liner: its thickness, m, its flow resistivity, Pa·s/m², and the share of the
    /// bay's surface it covers.
    pub liner_thickness_m: f64,
    pub liner_resistivity: f64,
    pub liner_coverage: f64,
    /// A floor closes the bay underneath (a flat underbody or an engine undertray): the
    /// block's skirt and sump radiate into the bay and leave through it too. Open (false, the
    /// default), they radiate straight out of the underside and only the head's ticking is
    /// muffled.
    pub closed_underside: bool,
}

impl Default for EngineBay {
    /// A road car's bay. ESTIMATES: an open share of 1 % puts the liner's 14 dB at 4 kHz at the
    /// low end of Ibarra et al.'s measured 16–27 dB; a 20 mm fibre liner (the packing's
    /// resistivity, [`crate::lining::Packing`]) over the hood, a quarter of a bay's surface.
    fn default() -> Self {
        Self {
            open_fraction: 0.01,
            liner_thickness_m: 0.02,
            liner_resistivity: crate::lining::Packing::default().flow_resistivity,
            liner_coverage: 0.25,
            closed_underside: false,
        }
    }
}

/// Normal-incidence absorption of a porous layer `thickness_m` thick of flow resistivity
/// `sigma` (Pa·s/m²) on a rigid backing, at `f` Hz: Delany & Bazley's characteristic
/// impedance and wavenumber, the layer's surface impedance Z_s = −j·Z_c·cot(k·d), and
/// α = 1 − |(Z_s − ρc)/(Z_s + ρc)|². Clamped to [0, 1] (the fit is for ρf/σ in 0.01–1).
pub fn liner_absorption(f: f64, thickness_m: f64, sigma: f64) -> f64 {
    let x = RHO * f / sigma;
    let zr = RHO * C * (1.0 + 0.0571 * x.powf(-0.754));
    let zi = -RHO * C * 0.087 * x.powf(-0.732);
    let k0 = 2.0 * std::f64::consts::PI * f / C;
    let kr = k0 * (1.0 + 0.0978 * x.powf(-0.700));
    let ki = -k0 * 0.189 * x.powf(-0.595);
    // cot(a + jb) = (sin 2a − j sinh 2b)/(cosh 2b − cos 2a), a + jb = k·d
    let (a, b) = (2.0 * kr * thickness_m, 2.0 * ki * thickness_m);
    let den = b.cosh() - a.cos();
    let (cr, ci) = (a.sin() / den, -b.sinh() / den);
    // Z_s = −j·Z_c·cot = −j(zr + j zi)(cr + j ci)
    let (pr, pi) = (zr * cr - zi * ci, zr * ci + zi * cr);
    let (sr, si) = (pi, -pr);
    let (nr, ni) = (sr - RHO * C, si);
    let (dr, di) = (sr + RHO * C, si);
    let r2 = (nr * nr + ni * ni) / (dr * dr + di * di);
    (1.0 - r2).clamp(0.0, 1.0)
}

impl EngineBay {
    /// The bay's insertion loss at `f` Hz, dB: 10·lg(1 + ᾱ/τ̄).
    pub fn insertion_loss_db(&self, f: f64) -> f64 {
        let liner = liner_absorption(f, self.liner_thickness_m, self.liner_resistivity);
        let mean = self.liner_coverage * liner + (1.0 - self.liner_coverage) * BARE_ABSORPTION;
        10.0 * (1.0 + mean / self.open_fraction.max(1e-6)).log10()
    }
}

/// The bay's loss as a filter on what the block radiates: its loss at [`LOW_HZ`] as a gain,
/// and a first-order shelf down to its loss at [`HIGH_HZ`], centred (geometrically) where the
/// loss is halfway in dB.
#[derive(Debug, Clone)]
pub struct BayFilter {
    gain: f64,
    section: (f64, f64, f64),
    state: f64,
}

impl BayFilter {
    pub fn new(bay: &EngineBay, dt: f64) -> Self {
        let (lo, hi) = (
            bay.insertion_loss_db(LOW_HZ),
            bay.insertion_loss_db(HIGH_HZ),
        );
        let mid = 0.5 * (lo + hi);
        // the halfway frequency, by bisection in log f (the loss rises with f)
        let (mut a, mut b) = (LOW_HZ.ln(), HIGH_HZ.ln());
        for _ in 0..60 {
            let m = 0.5 * (a + b);
            if bay.insertion_loss_db(m.exp()) < mid {
                a = m;
            } else {
                b = m;
            }
        }
        let depth = (hi - lo).max(0.0) / 20.0 * std::f64::consts::LN_10;
        let pole_hz = (0.5 * (a + b)).exp() * (-0.5 * depth).exp();
        Self {
            gain: 10f64.powf(-lo / 20.0),
            section: crate::waveguide::shelf(pole_hz, depth, dt),
            state: 0.0,
        }
    }

    pub fn process(&mut self, x: f64) -> f64 {
        let (b0, b1, a1) = self.section;
        let y = b0 * x + self.state;
        self.state = b1 * x - a1 * y;
        self.gain * y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_closed_underside_round_trips_and_old_files_leave_it_open() {
        let bay = EngineBay {
            closed_underside: true,
            ..EngineBay::default()
        };
        let yaml = serde_yaml_ng::to_string(&bay).unwrap();
        assert_eq!(serde_yaml_ng::from_str::<EngineBay>(&yaml).unwrap(), bay);
        let json = serde_json::to_string(&bay).unwrap();
        assert_eq!(serde_json::from_str::<EngineBay>(&json).unwrap(), bay);
        // a file written before the field: the underside stays open
        let old: EngineBay = serde_yaml_ng::from_str("open_fraction: 0.01\n").unwrap();
        assert!(!old.closed_underside);
    }

    #[test]
    fn a_fibre_liner_absorbs_the_highs_not_the_lows() {
        // 20 mm of 16 000 Pa·s/m² fibre on a rigid wall, Delany–Bazley worked by hand (complex
        // arithmetic in a calculator, ρ = 1.204, c = 343): α = 0.0227 at 250 Hz, 0.3240 at
        // 1 kHz, 0.9503 at 4 kHz.
        for (f, want) in [(250.0, 0.0227), (1000.0, 0.3240), (4000.0, 0.9503)] {
            let got = liner_absorption(f, 0.02, 16_000.0);
            assert!((got - want).abs() < 2e-4, "{f} Hz: {got}");
        }
    }

    #[test]
    fn the_bays_loss_is_its_absorption_over_its_openings() {
        // ᾱ = 0.25·0.9503 + 0.75·0.02 = 0.25258 at 4 kHz over 1 % open: 10·lg(26.258) = 14.19 dB
        let bay = EngineBay::default();
        assert!((bay.insertion_loss_db(4000.0) - 14.19).abs() < 0.02);
        // a bay with no liner and nothing closed: 10·lg(1 + 0.02) = 0.086 dB
        let open = EngineBay {
            open_fraction: 1.0,
            liner_coverage: 0.0,
            ..EngineBay::default()
        };
        assert!((open.insertion_loss_db(4000.0) - 0.086).abs() < 0.001);
    }

    #[test]
    fn the_filter_follows_the_bays_loss() {
        // steady sines through the filter against the loss computed at each frequency
        let bay = EngineBay::default();
        let dt = 1.0 / 96_000.0;
        for f in [63.0, 250.0, 1000.0, 4000.0] {
            let mut flt = BayFilter::new(&bay, dt);
            let n = (0.5 / dt) as usize;
            let mut peak: f64 = 0.0;
            for k in 0..n {
                let y = flt.process((2.0 * std::f64::consts::PI * f * k as f64 * dt).sin());
                if k > n / 2 {
                    peak = peak.max(y.abs());
                }
            }
            let got = -20.0 * peak.log10();
            let want = bay.insertion_loss_db(f);
            assert!((got - want).abs() < 2.0, "{f} Hz: {got:.1} dB vs {want:.1}");
        }
    }

    #[test]
    fn a_bay_round_trips() {
        let b = EngineBay::default();
        let json: EngineBay = serde_json::from_str(&serde_json::to_string(&b).unwrap()).unwrap();
        let yaml: EngineBay =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&b).unwrap()).unwrap();
        assert_eq!((json, yaml), (b.clone(), b));
    }
}
