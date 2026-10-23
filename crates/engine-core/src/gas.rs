//! 0D lumped gas volumes and compressible orifice flow.
//!
//! Ideal gas, constant γ = 1.4 (like engine-sim; NASA γ(T) tables are a later phase).
//! Orifice flow: isentropic nozzle with choked/unchoked regimes, computed from the
//! high-pressure side (Heywood, *ICE Fundamentals*, App. C, eqs. C.8–C.9).

use serde::{Deserialize, Serialize};

pub const R_AIR: f64 = 287.0;
pub const GAMMA: f64 = 1.4;
pub const CV: f64 = R_AIR / (GAMMA - 1.0);
/// 2/(γ − 1) for air (γ = 1.4): the isentropic exponents in y = (p/p₀)^((γ−1)/2γ) are integers,
/// ρ ∝ y^5 and p ∝ y^7, so they take multiplications, not `powf`.
pub(crate) const ISENTROPIC_RHO_POW: i32 = 5;
pub(crate) const ISENTROPIC_P_POW: i32 = 7;
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
        // pr^(1/γ) = pr / pr^((γ−1)/γ): one pow instead of two on the hottest path
        let q = pr.powf((g - 1.0) / g);
        base * (pr / q) * (2.0 * g / (g - 1.0) * (1.0 - q)).sqrt()
    }
}

/// A flow decided from a frozen state, to apply later. Lets several connections of one
/// volume be computed from the same start-of-step state (Jacobi), instead of each
/// one seeing the previous one's result — which ping-pongs a volume open to two others.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Flow {
    /// Mass from `a` to `b` (negative: `b` → `a`).
    pub dm: f64,
    /// Specific enthalpy carried, J/kg (upstream side).
    pub h: f64,
}

impl Flow {
    /// Apply to a volume exchanging with a reservoir (the reservoir does not change).
    pub fn apply_to(&self, a: &mut GasVolume) {
        a.mass_kg -= self.dm;
        a.energy_j -= self.dm * self.h;
    }

    pub fn apply(&self, a: &mut GasVolume, b: &mut GasVolume) {
        a.mass_kg -= self.dm;
        a.energy_j -= self.dm * self.h;
        b.mass_kg += self.dm;
        b.energy_j += self.dm * self.h;
    }
}

/// Plan the exchange `exchange` would do, scaled by `relax` (0..=1) of the equalising
/// limit, without changing either volume.
pub fn plan_exchange(a: &GasVolume, b: &GasVolume, cd_area: f64, dt: f64, relax: f64) -> Flow {
    let (pa, pb) = (a.pressure(), b.pressure());
    let (up, down, pu, pd, sign) = if pa >= pb {
        (a, b, pa, pb, 1.0)
    } else {
        (b, a, pb, pa, -1.0)
    };
    let h = CP * up.temperature();
    let ideal = orifice_mass_flow(cd_area, pu, up.temperature(), pd) * dt;
    let dm = ideal
        .min(relax * equalising_mass(pu - pd, h, 1.0 / up.volume_m3 + 1.0 / down.volume_m3))
        .min(0.5 * up.mass_kg);
    Flow { dm: sign * dm, h }
}

/// Exchange gas between two volumes through an orifice for `dt` seconds.
/// Returns the mass moved from `a` to `b` (negative if it went `b` → `a`).
/// The moved mass carries the upstream specific enthalpy and is clamped so the
/// pressures never cross in one step (keeps explicit stepping stable).
pub fn exchange(a: &mut GasVolume, b: &mut GasVolume, cd_area: f64, dt: f64) -> f64 {
    let f = plan_exchange(a, b, cd_area, dt, 1.0);
    f.apply(a, b);
    f.dm
}

/// Exchange gas between a volume and a fixed reservoir. Returns mass moved out of `a`
/// (negative = inflow into `a`).
pub fn exchange_with_reservoir(a: &mut GasVolume, r: &Reservoir, cd_area: f64, dt: f64) -> f64 {
    let f = plan_with_reservoir(a, r, cd_area, dt, 1.0);
    f.apply_to(a);
    f.dm
}

/// Plan the exchange between a volume and a reservoir (mass out of `a`; negative = inflow),
/// limited to `relax` of the equalising mass, without changing `a`.
pub fn plan_with_reservoir(
    a: &GasVolume,
    r: &Reservoir,
    cd_area: f64,
    dt: f64,
    relax: f64,
) -> Flow {
    let pa = a.pressure();
    let inv_v = 1.0 / a.volume_m3;
    if pa >= r.pressure_pa {
        let h = CP * a.temperature();
        let ideal = orifice_mass_flow(cd_area, pa, a.temperature(), r.pressure_pa) * dt;
        let dm = ideal
            .min(relax * equalising_mass(pa - r.pressure_pa, h, inv_v))
            .min(0.5 * a.mass_kg);
        Flow { dm, h }
    } else {
        let h = CP * r.temperature_k;
        let ideal = orifice_mass_flow(cd_area, r.pressure_pa, r.temperature_k, pa) * dt;
        Flow {
            dm: -ideal.min(relax * equalising_mass(r.pressure_pa - pa, h, inv_v)),
            h,
        }
    }
}

/// Signed mass flow (kg/s, + out of `a`) through an orifice into a pipe whose pressure at
/// the port rises with that flow: p(ṁ) = `p_base` + `z`·ṁ (`z` = c/A, plane-wave impedance;
/// `p_base` = p_atm + 2·p⁻). Solved implicitly — an explicit lag makes the port oscillate at
/// Nyquist — with Illinois regula falsi on h(ṁ) = ṁ − f(p(ṁ)), which is monotone.
/// Gas flowing back into `a` comes at `t_back`.
pub fn port_flow(cd_area: f64, a: &GasVolume, p_base: f64, t_back: f64, z: f64) -> f64 {
    let (pa, ta) = (a.pressure(), a.temperature());
    let f = |m: f64| {
        let pv = p_base + z * m;
        if pa >= pv {
            orifice_mass_flow(cd_area, pa, ta, pv)
        } else {
            -orifice_mass_flow(cd_area, pv, t_back, pa)
        }
    };
    let m0 = f(0.0);
    if m0 == 0.0 {
        return 0.0;
    }
    // h(0) and h(m0) bracket the root: h(0) = −m0 and h(m0) has the opposite sign.
    let h = |m: f64| m - f(m);
    let (mut lo, mut hi) = (0.0, m0);
    let (mut hlo, mut hhi) = (h(lo), h(hi));
    if hhi == 0.0 {
        return hi;
    }
    let mut side = 0;
    let tol = 1e-9 * m0.abs();
    for _ in 0..60 {
        let m = (lo * hhi - hi * hlo) / (hhi - hlo);
        let hm = h(m);
        if hm.abs() <= tol {
            return m;
        }
        if (hm > 0.0) == (hhi > 0.0) {
            hi = m;
            hhi = hm;
            if side == -1 {
                hlo *= 0.5;
            }
            side = -1;
        } else {
            lo = m;
            hlo = hm;
            if side == 1 {
                hhi *= 0.5;
            }
            side = 1;
        }
    }
    (lo * hhi - hi * hlo) / (hhi - hlo)
}

/// One step of flow through an exhaust valve into its primary pipe, given the wave arriving at
/// the port (`arriving`, gauge Pa), the port impedance `z` = c/A and the reversion gas
/// temperature. Solves the port implicitly ([`port_flow`]), then limits the step to `relax` of
/// the equalising mass and to half the cylinder's mass. Returns the planned flow (+ = out).
pub fn exhaust_valve_flow(
    gas: &GasVolume,
    area: f64,
    arriving: f64,
    z: f64,
    t_back: f64,
    dt: f64,
    relax: f64,
) -> Flow {
    if area <= 0.0 {
        return Flow {
            dm: 0.0,
            h: CP * gas.temperature(),
        };
    }
    let atm = crate::units::ATM_PA;
    let p_base = (atm + 2.0 * arriving).max(0.05 * atm);
    let m = port_flow(area, gas, p_base, t_back, z);
    let pv = p_base + z * m;
    let h = if m >= 0.0 {
        CP * gas.temperature()
    } else {
        CP * t_back
    };
    let limit = relax * (gas.pressure() - pv).abs() * gas.volume_m3 / ((GAMMA - 1.0) * h);
    let mut dm = (m * dt).clamp(-limit, limit);
    if dm > 0.0 {
        dm = dm.min(0.5 * gas.mass_kg);
    }
    Flow { dm, h }
}

/// Moving `dm` with enthalpy `h` changes p by (γ−1)·h·dm/V on each side (p = (γ−1)U/V),
/// so the pressures meet exactly at dm_eq = Δp / ((γ−1)·h·Σ1/V). Never move more.
fn equalising_mass(dp: f64, h: f64, inv_volumes: f64) -> f64 {
    dp / ((GAMMA - 1.0) * h * inv_volumes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_isentropic_exponents_are_airs() {
        // in y = (p/p₀)^((γ−1)/2γ): ρ ∝ y^(2/(γ−1)), p ∝ y^(2γ/(γ−1)); γ = 1.4 gives 5 and 7
        assert_relative_eq!(
            2.0 / (GAMMA - 1.0),
            f64::from(ISENTROPIC_RHO_POW),
            max_relative = 1e-12
        );
        assert_relative_eq!(
            2.0 * GAMMA / (GAMMA - 1.0),
            f64::from(ISENTROPIC_P_POW),
            max_relative = 1e-12
        );
    }
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

    #[test]
    fn planned_flow_matches_exchange_and_relax_scales_the_limit() {
        let (a0, b0) = (
            GasVolume::at(5e5, 1200.0, 5e-4),
            GasVolume::at(1e5, 300.0, 5e-4),
        );
        let (mut a1, mut b1) = (a0, b0);
        let dm = exchange(&mut a1, &mut b1, 5e-4, 2e-5);
        let f = plan_exchange(&a0, &b0, 5e-4, 2e-5, 1.0);
        assert_relative_eq!(f.dm, dm, max_relative = 1e-12);
        let (mut a2, mut b2) = (a0, b0);
        f.apply(&mut a2, &mut b2);
        assert_eq!((a1, b1), (a2, b2));
        // reverse direction is negative and carries b's enthalpy
        let f = plan_exchange(&b0, &a0, 5e-4, 2e-5, 1.0);
        assert_relative_eq!(f.dm, -dm, max_relative = 1e-12);
        assert_relative_eq!(f.h, CP * 1200.0, max_relative = 1e-12);
        // huge step: relax 0.5 moves half the equalising mass
        let full = plan_exchange(&a0, &b0, 1.0, 1.0, 1.0).dm;
        let half = plan_exchange(&a0, &b0, 1.0, 1.0, 0.5).dm;
        assert_relative_eq!(half, 0.5 * full, max_relative = 1e-12);
    }

    #[test]
    fn planned_reservoir_flow_matches_exchange_and_relaxes() {
        let atm = Reservoir {
            pressure_pa: 1e5,
            temperature_k: 300.0,
        };
        for p in [0.3e5, 3e5] {
            let a0 = GasVolume::at(p, 700.0, 5e-4);
            let mut a1 = a0;
            let dm = exchange_with_reservoir(&mut a1, &atm, 2e-4, 2e-5);
            let f = plan_with_reservoir(&a0, &atm, 2e-4, 2e-5, 1.0);
            assert_relative_eq!(f.dm, dm, max_relative = 1e-12);
            let mut a2 = a0;
            f.apply_to(&mut a2);
            assert_eq!(a1, a2);
            let full = plan_with_reservoir(&a0, &atm, 1.0, 1.0, 1.0).dm;
            let half = plan_with_reservoir(&a0, &atm, 1.0, 1.0, 0.25).dm;
            assert_relative_eq!(half, 0.25 * full, max_relative = 1e-12);
        }
    }

    #[test]
    fn port_flow_solves_the_coupled_equation() {
        let z = 600.0 / (std::f64::consts::PI * 0.25 * 0.04 * 0.04);
        for (p, base) in [
            (5e5, 1e5),
            (1.2e5, 1e5),
            (0.3e5, 1e5),
            (1e5, 1e5),
            (1e5, 1.0001e5),
        ] {
            let a = GasVolume::at(p, 900.0, 5e-4);
            let m = port_flow(7e-4, &a, base, 800.0, z);
            let pv = base + z * m;
            let expected = if p >= pv {
                orifice_mass_flow(7e-4, p, 900.0, pv)
            } else {
                -orifice_mass_flow(7e-4, pv, 800.0, p)
            };
            assert!(
                (m - expected).abs() <= 1e-6 * expected.abs().max(1e-9),
                "p {p}: {m} vs {expected}"
            );
            assert_eq!(m > 0.0, p > base, "direction at p {p}");
        }
        // no pipe impedance → plain orifice flow
        let a = GasVolume::at(3e5, 900.0, 5e-4);
        assert_relative_eq!(
            port_flow(7e-4, &a, 1e5, 800.0, 0.0),
            orifice_mass_flow(7e-4, 3e5, 900.0, 1e5)
        );
    }

    const Z: f64 = 600.0 / (std::f64::consts::PI * 0.25 * 0.04 * 0.04);

    #[test]
    fn exhaust_valve_blowdown_and_reversion() {
        // Blowdown: hot 5 bar cylinder, quiet pipe → outflow with the cylinder's enthalpy.
        let hot = GasVolume::at(5e5, 1200.0, 4e-4);
        let f = exhaust_valve_flow(&hot, 5e-4, 0.0, Z, 800.0, 5e-6, 0.5);
        let m = port_flow(5e-4, &hot, 1.01325e5, 800.0, Z);
        assert_relative_eq!(f.dm, m * 5e-6, max_relative = 1e-12);
        assert_relative_eq!(f.h, CP * 1200.0, max_relative = 1e-12);
        // Reversion: 0.3 bar cylinder → inflow carrying the pipe gas enthalpy (800 K).
        let cold = GasVolume::at(0.3e5, 400.0, 4e-4);
        let f = exhaust_valve_flow(&cold, 5e-4, 0.0, Z, 800.0, 5e-6, 0.5);
        assert!(f.dm < 0.0);
        assert_relative_eq!(f.h, CP * 800.0, max_relative = 1e-12);
        // An arriving compression wave (+0.3 bar) raises the port base by twice its amplitude.
        let with_wave = exhaust_valve_flow(&hot, 5e-4, 0.3e5, Z, 800.0, 5e-6, 0.5);
        let m = port_flow(5e-4, &hot, 1.01325e5 + 0.6e5, 800.0, Z);
        assert_relative_eq!(with_wave.dm, m * 5e-6, max_relative = 1e-12);
    }

    #[test]
    fn exhaust_valve_limits() {
        let hot = GasVolume::at(5e5, 1200.0, 4e-4);
        // Huge step: limited to `relax` of the equalising mass towards the port pressure.
        let f = exhaust_valve_flow(&hot, 5e-4, 0.0, Z, 800.0, 1.0, 0.25);
        let m = port_flow(5e-4, &hot, 1.01325e5, 800.0, Z);
        let pv = 1.01325e5 + Z * m;
        let eq = (5e5 - pv) * 4e-4 / ((GAMMA - 1.0) * CP * 1200.0);
        assert_relative_eq!(f.dm, 0.25 * eq, max_relative = 1e-9);
        // Never more than half the cylinder's mass.
        let f = exhaust_valve_flow(&hot, 5e-4, 0.0, 1e-3, 800.0, 1.0, 1.0);
        assert!(f.dm <= 0.5 * hot.mass_kg * (1.0 + 1e-12));
        // A rarefaction deeper than vacuum is floored at 5 % of an atmosphere.
        let deep = exhaust_valve_flow(&hot, 5e-4, -2.0e5, Z, 800.0, 5e-6, 0.5);
        let m = port_flow(5e-4, &hot, 0.05 * 1.01325e5, 800.0, Z);
        assert_relative_eq!(deep.dm, m * 5e-6, max_relative = 1e-12);
        // Valve shut: nothing moves.
        assert_eq!(
            exhaust_valve_flow(&hot, 0.0, 0.0, Z, 800.0, 5e-6, 0.5).dm,
            0.0
        );
    }

    #[test]
    fn reservoir_flows_carry_enthalpy() {
        let atm = Reservoir {
            pressure_pa: 101_325.0,
            temperature_k: 300.0,
        };
        // inflow: energy rises by dm · cp · T_reservoir
        let mut a = GasVolume::at(0.5e5, 600.0, 1e-3);
        let e0 = a.energy_j;
        let out = exchange_with_reservoir(&mut a, &atm, 1e-4, 1e-5);
        assert_relative_eq!(a.energy_j - e0, -out * CP * 300.0, max_relative = 1e-12);
        // outflow: energy falls by dm · cp · T_volume
        let mut b = GasVolume::at(3e5, 900.0, 1e-3);
        let e0 = b.energy_j;
        let out = exchange_with_reservoir(&mut b, &atm, 1e-4, 1e-5);
        assert_relative_eq!(e0 - b.energy_j, out * CP * 900.0, max_relative = 1e-12);
        // a huge inflow step settles at atmospheric pressure, no overshoot
        let mut c = GasVolume::at(0.2e5, 300.0, 1e-3);
        exchange_with_reservoir(&mut c, &atm, 1.0, 1.0);
        assert_relative_eq!(c.pressure(), 101_325.0, max_relative = 1e-6);
    }

    #[test]
    fn at_most_half_the_mass_leaves_in_one_step() {
        // 50 bar into 1 bar: the equalising mass is (1 − 1/50)/γ = 70% of the gas,
        // so the 50% safety clamp is what binds.
        let atm = Reservoir {
            pressure_pa: 1e5,
            temperature_k: 300.0,
        };
        let mut a = GasVolume::at(50e5, 300.0, 1e-3);
        let m = a.mass_kg;
        let out = exchange_with_reservoir(&mut a, &atm, 1.0, 1.0);
        assert_relative_eq!(out, 0.5 * m, max_relative = 1e-12);
        let mut a = GasVolume::at(50e5, 300.0, 1e-3);
        let mut b = GasVolume::at(1e5, 300.0, 1.0);
        let moved = exchange(&mut a, &mut b, 1.0, 1.0);
        assert_relative_eq!(moved, 0.5 * m, max_relative = 1e-12);
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
