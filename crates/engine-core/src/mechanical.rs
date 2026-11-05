//! Mechanical noise of the engine structure: what the block radiates from the impacts of its
//! moving parts (main journals thrown across their oil clearance when their load reverses,
//! pistons slapping the liner when their side force reverses), as opposed to the combustion's
//! own pressure ([`crate::structure`]).
//!
//! Its level is the measured one. At high speed a petrol engine's noise is mechanical: a
//! 1500 cc engine motored without its head and valvetrain made 88.5 dB(A) at 4000 rev/min, the
//! same as two fired ones (Raff & Perry, "Vehicle noise studies", JSV 28(3), 1973, Fig. 11).
//! Lalor, Grover & Priede ("Engine noise — mechanical origins", SAE 800402, 1980) measure three
//! petrol engines' overall noise against speed, all with the same two slopes (their eqs. 12–13:
//! a clearance-controlled branch at low speed, an inertia-controlled one above a critical speed);
//! the curve here is their 91 mm bore / 77 mm stroke engine of Fig. 7, read off the figure:
//! 75 dB(A) at 1000 rev/min, 90 at 3000, 105 at 6000 (ESTIMATE ±1 dB; the three engines spread
//! ±5 dB with no clean trend in bore — the 98 mm one is the quietest — so no bore scaling). The
//! distance is not printed: ISVR's practice of 1 m from the engine (ESTIMATE).

/// Speed where Fig. 7's two slopes meet, rev/min.
pub const CRITICAL_RPM: f64 = 3000.0;
/// Level at [`CRITICAL_RPM`], dB(A) at 1 m.
pub const CRITICAL_DBA: f64 = 90.0;
/// Slope below [`CRITICAL_RPM`], dB per decade of speed: (90 − 75)/log₁₀3.
pub const LOW_DB_PER_DECADE: f64 = 31.4;
/// Slope above it: (105 − 90)/log₁₀2.
pub const HIGH_DB_PER_DECADE: f64 = 49.8;

/// The measured petrol engine's mechanical (overall, full-load) noise at `rpm`, dB(A) at 1 m;
/// −∞ when the engine is not turning.
pub fn mechanical_dba_at_1m(rpm: f64) -> f64 {
    if rpm <= 0.0 {
        return f64::NEG_INFINITY;
    }
    let decades = (rpm / CRITICAL_RPM).log10();
    let slope = if decades < 0.0 {
        LOW_DB_PER_DECADE
    } else {
        HIGH_DB_PER_DECADE
    };
    CRITICAL_DBA + slope * decades
}

/// The rms sound pressure of a level `db` re 20 µPa, Pa (0 for −∞).
pub fn rms_pa_from_dba(db: f64) -> f64 {
    20e-6 * 10f64.powf(db / 20.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_petrol_curve_by_the_figure() {
        // Lalor, Grover & Priede, SAE 800402, Fig. 7, the 91 mm bore / 77 mm stroke petrol
        // engine, overall dB(A), read off the figure (±1 dB): 75 at 1000 rev/min, 90 at 3000,
        // 105 at 6000
        assert!((mechanical_dba_at_1m(1000.0) - 75.0).abs() < 1.0);
        assert!((mechanical_dba_at_1m(3000.0) - 90.0).abs() < 1.0);
        assert!((mechanical_dba_at_1m(6000.0) - 105.0).abs() < 1.0);
    }

    #[test]
    fn no_speed_no_noise() {
        assert_eq!(rms_pa_from_dba(mechanical_dba_at_1m(0.0)), 0.0);
        assert_eq!(rms_pa_from_dba(mechanical_dba_at_1m(-50.0)), 0.0);
    }

    #[test]
    fn decibels_to_pascals_by_hand() {
        // 94 dB re 20 µPa is 1.002 Pa rms (20e-6 · 10^(94/20) = 1.0024)
        assert!((rms_pa_from_dba(94.0) - 1.0024).abs() < 1e-4);
    }
}
