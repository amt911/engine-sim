//! Temperature as colour: the *inferno* map (Smith & van der Walt, matplotlib 2015 —
//! perceptually uniform, readable in grey and by colour-blind viewers), sampled at eight points
//! and interpolated, its darkest 15 % left out so cold parts stay visible on the dark background.

/// Coldest and hottest temperatures the map spans, K.
pub const COLD_K: f64 = 300.0;
pub const HOT_K: f64 = 1300.0;

/// inferno at 0, 1/7, …, 1 (sRGB).
const INFERNO: [[f32; 3]; 8] = [
    [0.001, 0.000, 0.014],
    [0.120, 0.047, 0.282],
    [0.316, 0.071, 0.485],
    [0.512, 0.130, 0.471],
    [0.705, 0.212, 0.380],
    [0.875, 0.341, 0.227],
    [0.978, 0.557, 0.035],
    [0.988, 0.998, 0.645],
];

/// The map at `t` in 0..1 (clamped), linear between its samples.
pub fn inferno(t: f32) -> [f32; 3] {
    let x = t.clamp(0.0, 1.0) * 7.0;
    let i = (x.floor() as usize).min(6);
    let f = x - i as f32;
    let (a, b) = (INFERNO[i], INFERNO[i + 1]);
    [0, 1, 2].map(|k| a[k] + f * (b[k] - a[k]))
}

/// The colour of a gas or wall at `kelvin` (300 K → 1300 K over the upper 85 % of the map),
/// opaque. NaN reads as cold.
pub fn temperature(kelvin: f64) -> [f32; 4] {
    let u = ((kelvin - COLD_K) / (HOT_K - COLD_K)) as f32;
    let u = if u.is_finite() {
        u.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let [r, g, b] = inferno(0.15 + 0.85 * u);
    [r, g, b, 1.0]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_map_hits_its_samples_and_clamps() {
        assert_eq!(inferno(0.0), INFERNO[0]);
        assert_eq!(inferno(1.0), INFERNO[7]);
        assert_eq!(inferno(3.0 / 7.0), INFERNO[3]);
        assert_eq!(inferno(-1.0), INFERNO[0]);
        assert_eq!(inferno(9.0), INFERNO[7]);
        // halfway between samples 0 and 1
        let h = inferno(0.5 / 7.0);
        assert!((h[2] - 0.148).abs() < 1e-6, "{h:?}");
    }

    #[test]
    fn temperatures_span_the_upper_map() {
        // 300 K at t = 0.15 (1.05 samples in: 0.05 of the way from sample 1 to 2)
        let c = temperature(300.0);
        let want = inferno(0.15);
        assert_eq!(&c[..3], &want);
        assert_eq!(&temperature(1300.0)[..3], &INFERNO[7]);
        assert_eq!(temperature(5000.0), temperature(1300.0));
        assert_eq!(temperature(f64::NAN), temperature(300.0));
        // hotter is brighter
        let lum = |c: [f32; 4]| 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
        assert!(lum(temperature(900.0)) > lum(temperature(500.0)));
    }
}
