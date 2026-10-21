//! 0D lumped gas volumes and compressible orifice flow.
//!
//! Ideal gas, constant γ = 1.4 (like engine-sim; NASA γ(T) tables are a later phase).
//! Orifice flow: isentropic nozzle with choked/unchoked regimes, computed from the
//! high-pressure side (Heywood, *ICE Fundamentals*, App. C, eqs. C.8–C.9).

use serde::{Deserialize, Serialize};

pub const R_AIR: f64 = 287.0;
pub const GAMMA: f64 = 1.4;
pub const CV: f64 = R_AIR / (GAMMA - 1.0);
pub const CP: f64 = GAMMA * CV;

/// A control volume: mass, internal energy and (externally imposed) volume.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GasVolume {
    pub mass_kg: f64,
    pub energy_j: f64,
    pub volume_m3: f64,
}

/// A fixed-state reservoir (the atmosphere).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reservoir {
    pub pressure_pa: f64,
    pub temperature_k: f64,
}

impl GasVolume {
    pub fn at(pressure_pa: f64, temperature_k: f64, volume_m3: f64) -> Self {
        let mass_kg = pressure_pa * volume_m3 / (R_AIR * temperature_k);
        Self {
            mass_kg,
            energy_j: mass_kg * CV * temperature_k,
            volume_m3,
        }
    }
    pub fn pressure(&self) -> f64 {
        (GAMMA - 1.0) * self.energy_j / self.volume_m3
    }
    pub fn temperature(&self) -> f64 {
        self.energy_j / (self.mass_kg * CV)
    }
    /// Change the volume keeping mass and energy (work is applied separately).
    pub fn set_volume(&mut self, v: f64) {
        self.volume_m3 = v;
    }
}

/// Mass flow (kg/s, ≥ 0) through an orifice of effective area `cd_area` from `p_up`
/// to `p_down`. Zero when `p_down >= p_up`.
pub fn orifice_mass_flow(cd_area: f64, p_up: f64, t_up: f64, p_down: f64) -> f64 {
    if p_down >= p_up || cd_area <= 0.0 {
        return 0.0;
    }
    let g = GAMMA;
    let base = cd_area * p_up / (R_AIR * t_up).sqrt();
    let crit = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
    let pr = p_down / p_up;
    if pr <= crit {
        base * g.sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)))
    } else {
        base * pr.powf(1.0 / g) * (2.0 * g / (g - 1.0) * (1.0 - pr.powf((g - 1.0) / g))).sqrt()
    }
}

/// Exchange gas between two volumes through an orifice for `dt` seconds.
/// Returns the mass moved from `a` to `b` (negative if it went `b` → `a`).
/// The moved mass carries the upstream specific enthalpy and is clamped so the
/// pressures never cross in one step (keeps explicit stepping stable).
pub fn exchange(a: &mut GasVolume, b: &mut GasVolume, cd_area: f64, dt: f64) -> f64 {
    let (pa, pb) = (a.pressure(), b.pressure());
    if pa >= pb {
        transfer(a, b, cd_area, dt, pa, pb)
    } else {
        -transfer(b, a, cd_area, dt, pb, pa)
    }
}

/// Exchange gas between a volume and a fixed reservoir. Returns mass moved out of `a`
/// (negative = inflow into `a`).
pub fn exchange_with_reservoir(a: &mut GasVolume, r: &Reservoir, cd_area: f64, dt: f64) -> f64 {
    let pa = a.pressure();
    if pa >= r.pressure_pa {
        let h = CP * a.temperature();
        let dm = clamp_flow(
            orifice_mass_flow(cd_area, pa, a.temperature(), r.pressure_pa) * dt,
            pa - r.pressure_pa,
            h,
            1.0 / a.volume_m3,
        )
        .min(0.5 * a.mass_kg);
        a.mass_kg -= dm;
        a.energy_j -= dm * h;
        dm
    } else {
        let h = CP * r.temperature_k;
        let dm = clamp_flow(
            orifice_mass_flow(cd_area, r.pressure_pa, r.temperature_k, pa) * dt,
            r.pressure_pa - pa,
            h,
            1.0 / a.volume_m3,
        );
        a.mass_kg += dm;
        a.energy_j += dm * h;
        -dm
    }
}

/// Move gas from `up` (pressure `pu`) to `down` (pressure `pd`), `pu >= pd`.
fn transfer(
    up: &mut GasVolume,
    down: &mut GasVolume,
    cd_area: f64,
    dt: f64,
    pu: f64,
    pd: f64,
) -> f64 {
    let h = CP * up.temperature();
    let ideal = orifice_mass_flow(cd_area, pu, up.temperature(), pd) * dt;
    let dm = clamp_flow(ideal, pu - pd, h, 1.0 / up.volume_m3 + 1.0 / down.volume_m3)
        .min(0.5 * up.mass_kg);
    up.mass_kg -= dm;
    up.energy_j -= dm * h;
    down.mass_kg += dm;
    down.energy_j += dm * h;
    dm
}

/// Moving `dm` with enthalpy `h` changes p by (γ−1)·h·dm/V on each side (p = (γ−1)U/V),
/// so the pressures meet exactly at dm_eq = Δp / ((γ−1)·h·Σ1/V). Never move more.
fn clamp_flow(ideal: f64, dp: f64, h: f64, inv_volumes: f64) -> f64 {
    let dm_eq = dp / ((GAMMA - 1.0) * h * inv_volumes);
    ideal.min(dm_eq)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    #[test]
    fn state_round_trip() {
        let g = GasVolume::at(2.0e5, 350.0, 1e-3);
        assert_relative_eq!(g.pressure(), 2.0e5, max_relative = 1e-12);
        assert_relative_eq!(g.temperature(), 350.0, max_relative = 1e-12);
        // m = pV/RT = 2e5·1e-3/(287·350) = 1.99104e-3 kg
        assert_relative_eq!(g.mass_kg, 1.99104e-3, max_relative = 1e-5);
    }

    #[test]
    fn choked_flow_matches_reference() {
        // p_down/p_up = 0.25 < 0.528 → choked. Value computed by hand from Heywood C.9.
        let f = orifice_mass_flow(1e-4, 2e5, 300.0, 0.5e5);
        assert_relative_eq!(f, 0.046671, max_relative = 1e-4);
    }

    #[test]
    fn subsonic_flow_matches_reference() {
        // p_down/p_up = 0.8. Heywood C.8 by hand.
        let f = orifice_mass_flow(1e-4, 2e5, 300.0, 1.6e5);
        assert_relative_eq!(f, 0.038215, max_relative = 1e-4);
    }

    #[test]
    fn no_flow_without_pressure_drop() {
        assert_eq!(orifice_mass_flow(1e-4, 1e5, 300.0, 1e5), 0.0);
        assert_eq!(orifice_mass_flow(1e-4, 1e5, 300.0, 2e5), 0.0);
    }

    #[test]
    fn regimes_meet_at_critical_ratio() {
        let crit = (2.0 / (GAMMA + 1.0)).powf(GAMMA / (GAMMA - 1.0));
        let below = orifice_mass_flow(1e-4, 1e5, 300.0, 1e5 * (crit - 1e-9));
        let above = orifice_mass_flow(1e-4, 1e5, 300.0, 1e5 * (crit + 1e-9));
        assert_relative_eq!(below, above, max_relative = 1e-6);
    }

    #[test]
    fn exchange_flows_high_to_low_and_conserves() {
        let mut a = GasVolume::at(3e5, 900.0, 5e-4);
        let mut b = GasVolume::at(1e5, 400.0, 1e-3);
        let (m0, e0) = (a.mass_kg + b.mass_kg, a.energy_j + b.energy_j);
        let dm = exchange(&mut a, &mut b, 5e-4, 2e-5);
        assert!(dm > 0.0);
        assert_relative_eq!(a.mass_kg + b.mass_kg, m0, max_relative = 1e-12);
        assert_relative_eq!(a.energy_j + b.energy_j, e0, max_relative = 1e-12);
        // reversed arguments → negative
        let mut c = GasVolume::at(1e5, 300.0, 1e-3);
        let mut d = GasVolume::at(2e5, 300.0, 1e-3);
        assert!(exchange(&mut c, &mut d, 1e-4, 1e-5) < 0.0);
    }

    #[test]
    fn huge_step_equalizes_without_crossing() {
        let mut a = GasVolume::at(5e5, 1200.0, 5e-4);
        let mut b = GasVolume::at(1e5, 300.0, 5e-4);
        exchange(&mut a, &mut b, 1.0, 1.0);
        assert!(a.pressure() >= b.pressure() * (1.0 - 1e-9));
        assert_relative_eq!(a.pressure(), b.pressure(), max_relative = 1e-6);
    }

    #[test]
    fn reservoir_inflow_and_outflow() {
        let atm = Reservoir {
            pressure_pa: 101_325.0,
            temperature_k: 300.0,
        };
        let mut low = GasVolume::at(0.5e5, 300.0, 1e-3);
        let m = low.mass_kg;
        assert!(exchange_with_reservoir(&mut low, &atm, 1e-4, 1e-4) < 0.0);
        assert!(low.mass_kg > m);
        let mut high = GasVolume::at(3e5, 300.0, 1e-3);
        assert!(exchange_with_reservoir(&mut high, &atm, 1e-4, 1e-4) > 0.0);
        // a huge step settles at atmospheric pressure, no overshoot
        exchange_with_reservoir(&mut high, &atm, 1.0, 1.0);
        assert_relative_eq!(high.pressure(), 101_325.0, max_relative = 1e-6);
    }

    proptest! {
        #[test]
        fn exchange_never_crosses_or_goes_negative(
            pa in 2e4f64..2e6, pb in 2e4f64..2e6, ta in 250.0f64..2500.0, tb in 250.0f64..2500.0,
            va in 1e-5f64..2e-3, vb in 1e-5f64..2e-3, area in 1e-6f64..1e-2, dt in 1e-6f64..1e-3,
        ) {
            let mut a = GasVolume::at(pa, ta, va);
            let mut b = GasVolume::at(pb, tb, vb);
            let sign = (pa - pb).signum();
            exchange(&mut a, &mut b, area, dt);
            prop_assert!(a.mass_kg > 0.0 && b.mass_kg > 0.0);
            prop_assert!(a.energy_j > 0.0 && b.energy_j > 0.0);
            prop_assert!((a.pressure() - b.pressure()) * sign >= -1e-6 * pa.max(pb));
        }
    }
}
