//! Transient fuel: the port wall film (Aquino's x–τ model, SAE 810494; Heywood, *ICE
//! Fundamentals*, §7.6.3), the share of a charge a flame leaves unburnt, and that mixture
//! burning again in a hot exhaust (Heywood §11.6.3).

use crate::combustion::{AFR_STOICH, LHV_GASOLINE};

/// Unburnt mixture in hot exhaust gas oxidizes from about 600 °C (HC; CO needs about 700 °C):
/// Heywood, *ICE Fundamentals*, §11.6.3.
pub const AFTERBURN_IGNITION_K: f64 = 873.0;

/// Share of the film that evaporates over `period_s` with the film's time constant `tau_s`:
/// Aquino's dm_f/dt = −m_f/τ over one intake period, exactly, 1 − e^(−T/τ).
pub fn evaporated_share(period_s: f64, tau_s: f64) -> f64 {
    1.0 - (-period_s.max(0.0) / tau_s.max(1e-9)).exp()
}

/// The liquid fuel on one port's walls, kg.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PortFilm {
    pub film_kg: f64,
}

impl PortFilm {
    /// One intake event: `injected` kg are sprayed, a share `x` of it lands on the film, and the
    /// film gives back its share `share` (see [`evaporated_share`]). Returns the fuel that enters
    /// the cylinder, kg.
    pub fn intake(&mut self, injected: f64, x: f64, share: f64) -> f64 {
        let back = share * self.film_kg;
        self.film_kg += x * injected - back;
        (1.0 - x) * injected + back
    }

    /// The ECU's compensation: the injection that delivers `want` kg through this film, never
    /// negative, and the intake event it makes. Returns (injected, delivered), kg.
    pub fn meter(&mut self, want: f64, x: f64, share: f64) -> (f64, f64) {
        // (1 − x)·inj + share·film = want
        let inj = ((want - share * self.film_kg) / (1.0 - x).max(1e-9)).max(0.0);
        (inj, self.intake(inj, x, share))
    }
}

/// The fuel a charge of `fuel` and `air` kg can burn: all of it when lean, what its air can
/// oxidize when rich.
pub fn burnable(fuel: f64, air: f64) -> f64 {
    fuel.min(air / AFR_STOICH).max(0.0)
}

/// What a flame that released the share `released` of its burnable fuel leaves unburnt:
/// (fuel, air), kg. A rich charge's excess fuel never burns.
pub fn unburnt(fuel: f64, air: f64, released: f64) -> (f64, f64) {
    let burnt = burnable(fuel, air) * released.clamp(0.0, 1.0);
    (fuel - burnt, (air - burnt * AFR_STOICH).max(0.0))
}

/// Unburnt fuel and air in one exhaust primary, kg.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ExhaustMixture {
    pub fuel_kg: f64,
    pub air_kg: f64,
}

impl ExhaustMixture {
    /// Burn for `dt` in gas at `t_k`: above [`AFTERBURN_IGNITION_K`] the burnable fuel goes with
    /// time constant `tau_s`. Returns the heat released, J (fuel burnt × LHV).
    pub fn burn(&mut self, t_k: f64, dt: f64, tau_s: f64) -> f64 {
        if t_k <= AFTERBURN_IGNITION_K {
            return 0.0;
        }
        let burnt = burnable(self.fuel_kg, self.air_kg) * evaporated_share(dt, tau_s);
        self.fuel_kg -= burnt;
        self.air_kg = (self.air_kg - burnt * AFR_STOICH).max(0.0);
        burnt * LHV_GASOLINE
    }

    /// `dm` kg enter a primary holding `m` kg, carrying `fuel` and `air` kg of unburnt mixture.
    /// The primary is well mixed (as [`crate::engine`]'s port temperature is): the gas the
    /// inflow pushes on downstream, or a reversion (`dm` < 0) takes back, carries its share of
    /// the mixture. Returns what left, (fuel, air) kg.
    pub fn flow(&mut self, m: f64, dm: f64, fuel: f64, air: f64) -> (f64, f64) {
        self.fuel_kg += fuel;
        self.air_kg += air;
        let leaves = if dm > 0.0 {
            dm / (m + dm)
        } else {
            (-dm / m.max(1e-12)).min(1.0)
        };
        let out = (self.fuel_kg * leaves, self.air_kg * leaves);
        self.fuel_kg -= out.0;
        self.air_kg -= out.1;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn a_film_evaporates_exponentially_over_a_period() {
        // 1 − e^(−1) = 0.632121 (hand); none in no time, all of it in a long one
        assert_relative_eq!(evaporated_share(0.2, 0.2), 0.632_120_6, max_relative = 1e-6);
        assert_eq!(evaporated_share(0.0, 0.2), 0.0);
        assert_relative_eq!(evaporated_share(100.0, 0.2), 1.0);
    }

    #[test]
    fn a_film_in_equilibrium_delivers_what_is_injected() {
        // steady state of x–τ: film = x·m/e, so the cylinder gets (1 − x)·m + x·m = m
        let (x, e, m) = (0.25, 0.1, 4e-5);
        let mut f = PortFilm { film_kg: x * m / e };
        assert_relative_eq!(f.intake(m, x, e), m, max_relative = 1e-12);
        assert_relative_eq!(f.film_kg, x * m / e, max_relative = 1e-12);
    }

    #[test]
    fn a_cut_leaves_a_geometric_tail() {
        // after a cut the k-th intake gets e·(1 − e)^k of the film: from equilibrium,
        // x·m·(1 − e)^k — with x = 0.25, e = 0.1, m = 4e-5: 1e-5, 9e-6, 8.1e-6 (hand)
        let (x, e, m) = (0.25, 0.1, 4e-5);
        let mut f = PortFilm { film_kg: x * m / e };
        for want in [1e-5, 9e-6, 8.1e-6] {
            assert_relative_eq!(f.intake(0.0, x, e), want, max_relative = 1e-12);
        }
    }

    #[test]
    fn the_ecu_fills_an_empty_film() {
        // from a dry port the ECU sprays want/(1 − x) and gets want: 4e-5/0.75 = 5.3333e-5 kg
        // (hand), of which 1.3333e-5 stays on the wall
        let mut f = PortFilm::default();
        let (inj, got) = f.meter(4e-5, 0.25, 0.1);
        assert_relative_eq!(inj, 5.333_333e-5, max_relative = 1e-6);
        assert_relative_eq!(got, 4e-5, max_relative = 1e-12);
        assert_relative_eq!(f.film_kg, 1.333_333e-5, max_relative = 1e-6);
    }

    #[test]
    fn a_lift_off_cannot_be_compensated() {
        // the film from 4e-5 kg charges gives back x·m = 1e-5 kg; a charge of a quarter wants
        // 1e-5 too, so the ECU sprays nothing and the cylinder still gets 1e-5. A tenth wants
        // 4e-6 and gets 1e-5: 2.5× rich (hand)
        let (x, e, m) = (0.25, 0.1, 4e-5);
        let mut f = PortFilm { film_kg: x * m / e };
        let (inj, got) = f.meter(m / 10.0, x, e);
        assert_eq!(inj, 0.0);
        assert_relative_eq!(got, 1e-5, max_relative = 1e-12);
    }

    #[test]
    fn a_rich_charge_burns_what_its_air_can_oxidize() {
        // 14.7 kg of air burn 1 kg of fuel: 2e-5 kg fuel in 1.47e-4 kg air burns 1e-5 (hand)
        assert_relative_eq!(burnable(2e-5, 1.47e-4), 1e-5, max_relative = 1e-12);
        assert_relative_eq!(burnable(5e-6, 1.47e-4), 5e-6, max_relative = 1e-12);
    }

    #[test]
    fn a_partial_burn_leaves_its_share_unburnt() {
        // lean (5e-6 fuel, 1.47e-4 air), 90 % released: 5e-7 fuel and 1.47e-4 − 4.5e-6·14.7
        // = 8.085e-5 air left (hand)
        let (f, a) = unburnt(5e-6, 1.47e-4, 0.9);
        assert_relative_eq!(f, 5e-7, max_relative = 1e-9);
        assert_relative_eq!(a, 8.085e-5, max_relative = 1e-9);
        // rich (2e-5, 1.47e-4), all released: the extra 1e-5 fuel and no air left
        let (f, a) = unburnt(2e-5, 1.47e-4, 1.0);
        assert_relative_eq!(f, 1e-5, max_relative = 1e-9);
        assert!(a.abs() < 1e-18, "{a}");
        // a misfire sends the whole charge out
        assert_eq!(unburnt(5e-6, 1.47e-4, 0.0), (5e-6, 1.47e-4));
    }

    #[test]
    fn cold_exhaust_does_not_burn_the_mixture() {
        let mut m = ExhaustMixture {
            fuel_kg: 1e-6,
            air_kg: 1e-4,
        };
        assert_eq!(m.burn(800.0, 1e-3, 2e-3), 0.0);
        assert_eq!(m.fuel_kg, 1e-6);
    }

    #[test]
    fn hot_exhaust_burns_the_mixture_with_its_air() {
        // dt = τ: 1 − e^(−1) of 1e-6 kg burns, 6.32121e-7 × 44e6 = 27.813 J (hand), and its
        // air goes with it (×14.7)
        let mut m = ExhaustMixture {
            fuel_kg: 1e-6,
            air_kg: 1e-4,
        };
        assert_relative_eq!(m.burn(1000.0, 2e-3, 2e-3), 27.813_3, max_relative = 1e-5);
        assert_relative_eq!(m.fuel_kg, 3.678_794e-7, max_relative = 1e-6);
        assert_relative_eq!(m.air_kg, 1e-4 - 6.321_206e-7 * 14.7, max_relative = 1e-6);
        // fuel without air does not burn
        let mut rich = ExhaustMixture {
            fuel_kg: 1e-6,
            air_kg: 0.0,
        };
        assert_eq!(rich.burn(1000.0, 2e-3, 2e-3), 0.0);
    }

    #[test]
    fn flow_through_a_primary_displaces_its_mixture() {
        // 1e-3 kg with 2e-7 fuel enters 4e-3 kg holding 1e-6: mixed, 1.2e-6 in 5e-3 kg, and the
        // 1e-3 kg pushed on takes a fifth: 9.6e-7 fuel and 8e-5 air stay (hand)
        let mut m = ExhaustMixture {
            fuel_kg: 1e-6,
            air_kg: 1e-4,
        };
        let (f, a) = m.flow(4e-3, 1e-3, 2e-7, 0.0);
        assert_relative_eq!(m.fuel_kg, 9.6e-7, max_relative = 1e-12);
        assert_relative_eq!(m.air_kg, 8e-5, max_relative = 1e-12);
        assert_relative_eq!(f, 2.4e-7, max_relative = 1e-12);
        assert_relative_eq!(a, 2e-5, max_relative = 1e-12);
        // a reversion of 8e-4 kg takes a fifth of 4e-3 kg back: 1.92e-7 fuel (hand)
        let (f, _) = m.flow(4e-3, -8e-4, 0.0, 0.0);
        assert_relative_eq!(m.fuel_kg, 7.68e-7, max_relative = 1e-12);
        assert_relative_eq!(f, 1.92e-7, max_relative = 1e-12);
    }
}
