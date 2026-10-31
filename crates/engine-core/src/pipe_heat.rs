//! Heat the exhaust gas loses to its pipes' walls, and the walls to the air outside: the gas
//! temperature that sets each pipe's speed of sound
//! (spec: `docs/superpowers/specs/2026-10-31-exhaust-gas-temperature-design.md`).

/// Laminar fully developed Nusselt number at a constant wall temperature (Incropera & DeWitt,
/// *Fundamentals of Heat and Mass Transfer*, eq. 8.55).
const NU_LAMINAR: f64 = 3.66;
/// Reynolds number where pipe flow turns turbulent (Incropera §8.1.1).
const RE_TURBULENT: f64 = 2300.0;
/// The wall: thin stainless sheet (AISI 304: ρ 7900 kg/m³, c 477 J/(kg·K), Incropera Table A.1),
/// 1.5 mm, a header's usual gauge. Heat capacity per unit area, J/(m²·K).
const WALL_HEAT_CAPACITY: f64 = 7900.0 * 477.0 * 0.0015;
/// Outside convection, W/(m²·K): between free convection of gases (2–25) and forced (25–250,
/// Incropera Table 1.1); under a car the air moves a little.
const OUTSIDE_HTC: f64 = 15.0;
/// Emissivity of oxidized steel (Incropera Table A.11).
const EMISSIVITY: f64 = 0.8;
/// Stefan–Boltzmann constant, W/(m²·K⁴).
const STEFAN_BOLTZMANN: f64 = 5.670e-8;
/// Air around the pipes, K.
pub const AMBIENT_K: f64 = 300.0;

/// Heat transfer coefficient from gas at `t_gas` K flowing at `mass_flow` kg/s to the wall of a
/// pipe of `diameter_m`, W/(m²·K): Dittus–Boelter for gas being cooled, Nu = 0.023·Re^0.8·Pr^0.3
/// (Incropera eq. 8.60), laminar Nu = 3.66 below Re 2300; Re = 4ṁ/(πDμ), k = μ·c_p/Pr.
pub fn inside_htc(mass_flow: f64, diameter_m: f64, t_gas: f64) -> f64 {
    let mu = crate::exhaust::gas_viscosity(t_gas);
    let pr = crate::narrow_tube::PRANDTL;
    let k = mu * crate::gas::CP / pr;
    let re = 4.0 * mass_flow.abs() / (std::f64::consts::PI * diameter_m * mu);
    let nu = if re < RE_TURBULENT {
        NU_LAMINAR
    } else {
        (0.023 * re.powf(0.8) * pr.powf(0.3)).max(NU_LAMINAR)
    };
    nu * k / diameter_m
}

/// Gas entering a pipe at `t_in` whose wall is at `t_wall`, after `ntu` = hπDL/(ṁc_p) transfer
/// units: T(x) = T_w + (T_in − T_w)·e^(−x/ℓ) (Incropera eq. 8.42b, constant wall temperature).
/// Returns (mean over the length, outlet), K.
pub fn profile(t_in: f64, t_wall: f64, ntu: f64) -> (f64, f64) {
    let d = t_in - t_wall;
    if ntu < 1e-9 {
        return (t_in, t_in);
    }
    let e = (-ntu).exp();
    (t_wall + d * (1.0 - e) / ntu, t_wall + d * e)
}

/// One pipe's wall and the gas through it.
#[derive(Debug, Clone)]
pub struct PipeHeat {
    length_m: f64,
    diameter_m: f64,
    /// The wall's temperature, K.
    pub wall_k: f64,
    /// Whether the wall has been set to its first operating point's steady state.
    settled: bool,
}

impl PipeHeat {
    pub fn new(length_m: f64, diameter_m: f64) -> Self {
        Self {
            length_m,
            diameter_m,
            wall_k: AMBIENT_K,
            settled: false,
        }
    }

    /// Heat flux into the wall from gas entering at `t_in` with `mass_flow`, W/m², and the gas's
    /// (mean, outlet) temperatures, at the wall's temperature `t_wall`.
    fn exchange(&self, t_in: f64, mass_flow: f64, t_wall: f64) -> (f64, (f64, f64)) {
        let m = mass_flow.abs();
        let area = std::f64::consts::PI * self.diameter_m * self.length_m;
        if m < 1e-12 {
            // still gas: it sits at the wall's temperature and carries no heat
            return (0.0, (t_wall, t_wall));
        }
        let h = inside_htc(m, self.diameter_m, t_in);
        let ntu = h * area / (m * crate::gas::CP);
        let (mean, out) = profile(t_in, t_wall, ntu);
        (m * crate::gas::CP * (t_in - out) / area, (mean, out))
    }

    /// Heat flux the wall at `t_wall` loses outside, W/m²: convection and radiation.
    fn loss(t_wall: f64) -> f64 {
        OUTSIDE_HTC * (t_wall - AMBIENT_K)
            + EMISSIVITY * STEFAN_BOLTZMANN * (t_wall.powi(4) - AMBIENT_K.powi(4))
    }

    /// Gas entering at `t_in` K with `mass_flow` kg/s for `dt` s: the wall warms or cools by the
    /// heat it takes from the gas less what it loses outside (a lumped wall, explicit Euler: `dt`
    /// is a few hundred µs against a time constant of tens of seconds). The first call sets the
    /// wall to that operating point's steady state, so a render does not start with cold pipes.
    /// Returns the gas's (mean, outlet) temperatures, K.
    pub fn step(&mut self, t_in: f64, mass_flow: f64, dt: f64) -> (f64, f64) {
        if !self.settled {
            self.settled = true;
            self.wall_k = self.steady_wall(t_in, mass_flow);
        }
        let (q, temps) = self.exchange(t_in, mass_flow, self.wall_k);
        self.wall_k += dt * (q - Self::loss(self.wall_k)) / WALL_HEAT_CAPACITY;
        temps
    }

    /// The wall temperature where the gas's heat balances the losses outside (bisection: the
    /// balance falls monotonically with the wall's temperature).
    fn steady_wall(&self, t_in: f64, mass_flow: f64) -> f64 {
        let (mut lo, mut hi) = (AMBIENT_K, t_in.max(AMBIENT_K));
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if self.exchange(t_in, mass_flow, mid).0 > Self::loss(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

/// Speed of sound in gas at `t` K, m/s: c = √(γRT).
pub fn sound_speed(t: f64) -> f64 {
    (crate::gas::GAMMA * crate::gas::R_AIR * t).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dittus_boelter_in_a_primary_matches_the_hand_value() {
        // 0.02 kg/s of 900 K gas in a 42 mm pipe: μ = 1.458e-6·900^1.5/1010.4 = 3.8961e-5 Pa·s,
        // Re = 4·0.02/(π·0.042·3.8961e-5) = 15 562, Nu = 0.023·15562^0.8·0.71^0.3 = 0.023·2257.6
        // ·0.90235 = 46.85, k = 3.8961e-5·1004.5/0.71 = 0.05512 W/(m·K): h = 46.85·0.05512/0.042
        // = 61.49 W/(m²·K)
        let h = inside_htc(0.02, 0.042, 900.0);
        assert!((h - 61.49).abs() < 0.3, "h = {h}");
    }

    #[test]
    fn slow_gas_transfers_heat_as_laminar_flow() {
        // 1e-4 kg/s: Re ≈ 78, Nu = 3.66: h = 3.66·0.05512/0.042 = 4.803 W/(m²·K)
        let h = inside_htc(1e-4, 0.042, 900.0);
        assert!((h - 4.803).abs() < 0.02, "h = {h}");
    }

    #[test]
    fn the_gas_cools_toward_the_wall_exponentially() {
        // 1000 K gas, 600 K wall, one transfer unit: outlet 600 + 400/e = 747.15 K, mean
        // 600 + 400·(1 − 1/e) = 852.85 K
        let (mean, out) = profile(1000.0, 600.0, 1.0);
        assert!((out - 747.15).abs() < 0.01, "outlet {out}");
        assert!((mean - 852.85).abs() < 0.01, "mean {mean}");
    }

    #[test]
    fn a_longer_pipe_lets_the_gas_out_colder() {
        let mut short = PipeHeat::new(0.5, 0.042);
        let mut long = PipeHeat::new(1.5, 0.042);
        let (_, out_s) = short.step(1000.0, 0.02, 1e-3);
        let (_, out_l) = long.step(1000.0, 0.02, 1e-3);
        assert!(out_l < out_s - 5.0, "short {out_s}, long {out_l}");
        assert!(out_s < 1000.0 && out_l > short.wall_k.min(long.wall_k));
    }

    #[test]
    fn the_wall_starts_where_its_heat_balances() {
        let mut p = PipeHeat::new(0.8, 0.042);
        p.step(1000.0, 0.02, 1e-3);
        let (q, _) = p.exchange(1000.0, 0.02, p.wall_k);
        let out = PipeHeat::loss(p.wall_k);
        assert!((q - out).abs() < 1e-3 * out, "in {q}, out {out}");
        // a 42 mm primary at full load glows: between the air and the gas
        assert!((600.0..1000.0).contains(&p.wall_k), "wall {}", p.wall_k);
    }

    #[test]
    fn the_wall_warms_over_tens_of_seconds() {
        // settled at a light load, then a full-load gas: its time constant is the heat capacity
        // over the conductances, 5653/(61 + 15 + ~60 radiating) ≈ 40 s
        let mut p = PipeHeat::new(0.8, 0.042);
        p.step(700.0, 0.004, 1e-3);
        let cold = p.wall_k;
        let mut t = 0.0;
        while t < 5.0 {
            p.step(1000.0, 0.02, 1e-3);
            t += 1e-3;
        }
        let after_5s = p.wall_k;
        while t < 300.0 {
            p.step(1000.0, 0.02, 1e-2);
            t += 1e-2;
        }
        let hot = p.wall_k;
        let frac = (after_5s - cold) / (hot - cold);
        // 1 − e^(−5/40) ≈ 0.12: neither instant nor stuck
        assert!(
            (0.05..0.3).contains(&frac),
            "{cold} → {after_5s} → {hot}: {frac}"
        );
    }

    #[test]
    fn still_gas_sits_at_the_wall_temperature() {
        let mut p = PipeHeat::new(0.8, 0.042);
        p.wall_k = 700.0;
        p.settled = true;
        let (mean, out) = p.step(1000.0, 0.0, 1e-3);
        assert_eq!((mean, out), (700.0, 700.0));
    }

    #[test]
    fn sound_speed_of_air_at_room_temperature() {
        // √(1.4·287·293.15) = 343.2 m/s
        assert!((sound_speed(293.15) - 343.2).abs() < 0.1);
    }
}
