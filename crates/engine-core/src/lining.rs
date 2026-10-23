//! Absorptive lining of a silencer: the fibre packing behind the perforated wall of a muffler
//! can, the part that turns the high-frequency content of the exhaust into heat (the "air" a
//! bare expansion chamber passes on).
//!
//! Each lined cell of a [`Pipe1d`] breathes through the perforate into its own pocket of gas in
//! the packing: a resistance (the fibres' flow resistivity across the layer) in series with a
//! compliance (the gas in the pores). This locally reacting liner is the lowest-order model of a
//! dissipative duct (Munjal, *Acoustics of Ducts and Mufflers*, 2nd ed., ch. 6): per metre of
//! duct a shunt admittance y = jωc/(1 + jωrc) next to the duct's own, which makes a plane wave
//! decay by α ≈ Re(y)·ρc/(2A) per metre, rising as ω² until ωrc ≈ 1.
//!
//! - r: a rigid-backed porous layer of thickness t and flow resistivity σ has, at low
//!   frequencies, a surface impedance tending to σt/3 − j·γp/(ωφt) (the classic low-frequency
//!   limit of a thin porous layer, e.g. Allard & Atalla, *Propagating Sound in Porous Media*);
//!   divided by the perimeter it faces the duct with, Pa·s/m² per metre of duct.
//! - c: the pore volume over γp, adiabatic like the rest of the gas (the fibres make it
//!   closer to isothermal at low frequencies, ×1.4 compliance; a later refinement).
//!
//! The exchange is split from the flux step and integrated exactly for its linear part: the
//! pressure difference between cell and pocket relaxes as e^(−t/τ),
//! τ = r·c_cell·c_pocket/(c_cell + c_pocket), so it stays stable for any step. Gas that leaves the duct takes its total
//! enthalpy into the pocket (its axial momentum is lost to the fibres, as heat); gas that comes
//! back brings the pocket's enthalpy and no axial momentum. Mass and energy are conserved.

use crate::fv::{Cons, Pipe1d};
use crate::gas::{CP, CV, GAMMA, GasVolume};
use serde::{Deserialize, Serialize};

/// Fibre packing behind a perforated wall. Zero thickness: no packing (a bare can).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Packing {
    /// Layer thickness, m.
    pub thickness_m: f64,
    /// Flow resistivity, Pa·s/m².
    pub flow_resistivity: f64,
    /// Open fraction of the packed volume.
    pub porosity: f64,
}

impl Default for Packing {
    /// A road-car silencer: 25 mm of glass-fibre wool. Muffler fibre packed at 100–200 g/L has
    /// a flow resistivity of roughly 5–40 kPa·s/m² and a porosity near 0.95 (estimates from
    /// the range of fibrous absorbers, Delany & Bazley 1970; to be checked against PAPERS.md).
    fn default() -> Self {
        Self {
            thickness_m: 0.025,
            flow_resistivity: 16_000.0,
            porosity: 0.95,
        }
    }
}

/// The packing along a run of cells of one pipe.
#[derive(Debug, Clone)]
pub struct Lining {
    first: usize,
    pockets: Vec<GasVolume>,
    /// Resistance of each cell's perforate face, Pa·s/m³.
    resistance: Vec<f64>,
}

impl Lining {
    /// Lines `cells` of `pipe` with `packing` of cross-section `packed_area_m2`, facing the duct
    /// along `perimeter_m`, its pores filled with gas at `p` and `t_k`.
    pub fn new(
        pipe: &Pipe1d,
        cells: std::ops::Range<usize>,
        perimeter_m: f64,
        packed_area_m2: f64,
        packing: Packing,
        (p, t_k): (f64, f64),
    ) -> Self {
        let face = perimeter_m * pipe.dx;
        let pore = packing.porosity * packed_area_m2 * pipe.dx;
        let n = cells.len();
        Self {
            first: cells.start,
            pockets: vec![GasVolume::at(p, t_k, pore); n],
            resistance: vec![packing.flow_resistivity * packing.thickness_m / 3.0 / face; n],
        }
    }

    /// Exchange gas between each lined cell and its pocket over `dt`.
    pub fn step(&mut self, pipe: &mut Pipe1d, dt: f64) {
        for (k, (pocket, &r)) in self.pockets.iter_mut().zip(&self.resistance).enumerate() {
            let i = self.first + k;
            let v_cell = pipe.area[i] * pipe.dx;
            exchange(&mut pipe.cells[i], v_cell, pocket, r, dt);
        }
    }

    /// Gas held in the pores, kg.
    pub fn mass_kg(&self) -> f64 {
        self.pockets.iter().map(|p| p.mass_kg).sum()
    }

    /// Pressure in each pocket, Pa.
    pub fn pressures(&self) -> impl Iterator<Item = f64> + '_ {
        self.pockets.iter().map(GasVolume::pressure)
    }
}

/// One cell (conserved state per volume, volume `v_cell`) and its pocket through resistance
/// `r` (Pa·s/m³) for `dt`: returns the mass that went into the pocket (negative: out of it).
fn exchange(cell: &mut Cons, v_cell: f64, pocket: &mut GasVolume, r: f64, dt: f64) -> f64 {
    let rho = cell.rho;
    let u = cell.mom / rho;
    let ke = 0.5 * rho * u * u;
    let p = (GAMMA - 1.0) * (cell.e - ke);
    let pl = pocket.pressure();
    let (c_cell, c_pocket) = (v_cell / (GAMMA * p), pocket.volume_m3 / (GAMMA * pl));
    let tau = r * c_cell * c_pocket / (c_cell + c_pocket);
    // volume that crossed the face, ∫q dt with q = Δp/r and Δp relaxing as e^(−t/τ)
    let dv = (p - pl) * tau * -(-dt / tau).exp_m1() / r;
    let dm = if dv > 0.0 {
        dv * rho
    } else {
        dv * pocket.mass_kg / pocket.volume_m3
    };
    // total enthalpy per kilogram that crosses: the cell's (h + u²/2) out, the pocket's h in
    let h = if dm > 0.0 {
        (cell.e + p) / rho
    } else {
        CP * pocket.energy_j / (pocket.mass_kg * CV)
    };
    cell.rho -= dm / v_cell;
    cell.e -= dm * h / v_cell;
    if dm > 0.0 {
        cell.mom -= dm * u / v_cell;
    }
    pocket.mass_kg += dm;
    pocket.energy_j += dm * h;
    dm
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fv::{Boundary, Prim};
    use crate::gas::R_AIR;
    use approx::assert_relative_eq;

    #[test]
    fn the_exchange_conserves_mass_and_energy() {
        let v_cell = 1e-4;
        let mut cell = Prim {
            rho: 0.6,
            u: 40.0,
            p: 1.3e5,
        }
        .to_cons();
        let mut pocket = GasVolume::at(1.0e5, 400.0, 2e-5);
        let before = (
            cell.rho * v_cell + pocket.mass_kg,
            cell.e * v_cell + pocket.energy_j,
        );
        for _ in 0..50 {
            exchange(&mut cell, v_cell, &mut pocket, 1e7, 1e-5);
        }
        assert_relative_eq!(
            cell.rho * v_cell + pocket.mass_kg,
            before.0,
            max_relative = 1e-14
        );
        assert_relative_eq!(
            cell.e * v_cell + pocket.energy_j,
            before.1,
            max_relative = 1e-14
        );
    }

    #[test]
    fn the_pressure_difference_relaxes_with_the_rc_time() {
        // still gas, 1.01 bar in the cell against 1 bar in the pocket, both 300 K.
        // c_cell = 1e-4/(1.4·1.01e5) = 7.0721e-10, c_pocket = 1e-4/(1.4·1e5) = 7.1429e-10 m³/Pa;
        // τ = r·c_cell·c_pocket/(c_cell + c_pocket) = 1e6 · 3.5537e-10 = 3.5537e-4 s.
        // After τ the difference is down to 1/e of its 1000 Pa (to first order in Δp/p).
        let v = 1e-4;
        let mut cell = Prim {
            rho: 1.01e5 / (R_AIR * 300.0),
            u: 0.0,
            p: 1.01e5,
        }
        .to_cons();
        let mut pocket = GasVolume::at(1.0e5, 300.0, v);
        let n = 1000;
        for _ in 0..n {
            exchange(&mut cell, v, &mut pocket, 1e6, 3.5537e-4 / n as f64);
        }
        let p = (GAMMA - 1.0) * cell.e;
        assert_relative_eq!(
            (p - pocket.pressure()) / 1000.0,
            (-1.0f64).exp(),
            max_relative = 0.01
        );
        // and one big step lands in the same place: the exponential is exact, not explicit
        let mut cell2 = Prim {
            rho: 1.01e5 / (R_AIR * 300.0),
            u: 0.0,
            p: 1.01e5,
        }
        .to_cons();
        let mut pocket2 = GasVolume::at(1.0e5, 300.0, v);
        exchange(&mut cell2, v, &mut pocket2, 1e6, 3.5537e-4);
        assert_relative_eq!(
            (GAMMA - 1.0) * cell2.e - pocket2.pressure(),
            p - pocket.pressure(),
            max_relative = 0.01
        );
    }

    #[test]
    fn outflow_leaves_its_axial_momentum_in_the_fibres() {
        // gas flowing at 40 m/s into a lower-pressure pocket: the cell keeps its velocity (the
        // mass that left took its own share of momentum), it does not speed up
        let v_cell = 1e-4;
        let mut cell = Prim {
            rho: 1.2,
            u: 40.0,
            p: 1.1e5,
        }
        .to_cons();
        let mut pocket = GasVolume::at(1.0e5, 300.0, 1e-4);
        let dm = exchange(&mut cell, v_cell, &mut pocket, 1e6, 1e-4);
        assert!(dm > 0.0);
        assert_relative_eq!(cell.mom / cell.rho, 40.0, max_relative = 1e-12);
    }

    #[test]
    fn packing_round_trips_and_old_files_get_the_default() {
        let k = Packing {
            thickness_m: 0.03,
            flow_resistivity: 9_000.0,
            porosity: 0.9,
        };
        let json = serde_json::to_string(&k).unwrap();
        assert_eq!(serde_json::from_str::<Packing>(&json).unwrap(), k);
        let yaml = serde_yaml_ng::to_string(&k).unwrap();
        assert_eq!(serde_yaml_ng::from_str::<Packing>(&yaml).unwrap(), k);
        // an exhaust written before the packing existed loads with the road-car default
        let old: crate::exhaust::ExhaustDef =
            serde_yaml_ng::from_str("tail_length_m: 1.2").unwrap();
        assert_eq!(old.muffler_packing, Packing::default());
        assert_eq!(Packing::default().thickness_m, 0.025);
    }

    /// Single-bin DFT amplitude of `x` at `f` (sample step `dt`).
    fn amplitude(x: &[f64], f: f64, dt: f64) -> f64 {
        let w = 2.0 * std::f64::consts::PI * f * dt;
        let (re, im) = x.iter().enumerate().fold((0.0, 0.0), |(re, im), (n, v)| {
            (re + v * (w * n as f64).cos(), im + v * (w * n as f64).sin())
        });
        (re * re + im * im).sqrt()
    }

    #[test]
    fn a_lined_duct_attenuates_a_tone_as_the_transmission_line_predicts() {
        // 2 kHz tone burst (10 cycles, Hann) of 50 Pa in air at 300 K down a 0.01 m² duct, lined
        // over 1–4 m with 25 mm of σ = 16 kPa·s/m² packing (porosity 0.95) along 0.35 m of
        // perimeter. Per metre, by hand: r = σt/3/P = 380.95 Pa·s/m², c = 0.95·0.35·0.025/
        // (1.4·101325) = 5.8599e-8 m³/Pa, ωrc = 0.28052, y = jωc/(1 + jωrc) with Re y =
        // 1.9150e-4; the duct: ρ = 1.17683 kg/m³, c = 347.19 m/s, z = jωρ/A, its own shunt
        // jωA/(ρc²) = j·8.8586e-4. Lossy transmission line: γ = √(z·(jωA/(ρc²) + y)) =
        // 2.9346 + j·48.251 per metre. From x = 1.5 to 2.0 m the tone keeps e^(−0.5·2.9346) =
        // 0.2305 of its amplitude, relative to the same run unlined (the ratio cancels the
        // scheme's own dissipation). The lined wave is shorter (k = 48.25/m against 36.2/m), so
        // it needs fine cells: measured 0.152 at 10 mm, 0.220 at 5 mm, 0.227 at the 2.5 mm here.
        let (p0, t0) = (101_325.0, 300.0);
        let rho0 = p0 / (R_AIR * t0);
        let c0 = (GAMMA * p0 / rho0).sqrt();
        let f = 2000.0;
        let packing = Packing {
            thickness_m: 0.025,
            flow_resistivity: 16_000.0,
            porosity: 0.95,
        };
        let run = |lined: bool| {
            let n = 2400;
            let mut pipe = Pipe1d::segmented(
                &[(6.0, 0.01)],
                0.0025,
                Prim {
                    rho: rho0,
                    u: 0.0,
                    p: p0,
                },
            );
            assert_eq!(pipe.cells.len(), n);
            // the burst, right-running, centred at 0.5 m
            let len = 10.0 * c0 / f;
            for i in 0..n {
                let x = (i as f64 + 0.5) * pipe.dx - 0.5 + 0.5 * len;
                let env = if (0.0..len).contains(&x) {
                    (std::f64::consts::PI * x / len).sin().powi(2)
                } else {
                    0.0
                };
                let dp = 50.0 * env * (2.0 * std::f64::consts::PI * f * x / c0).sin();
                pipe.cells[i] = Prim {
                    rho: rho0 + dp / (c0 * c0),
                    u: dp / (rho0 * c0),
                    p: p0 + dp,
                }
                .to_cons();
            }
            let open = Boundary::Open { p: p0, rho: rho0 };
            (pipe.left, pipe.right) = (open, open);
            let mut lining =
                lined.then(|| Lining::new(&pipe, 400..1600, 0.35, 0.35 * 0.025, packing, (p0, t0)));
            let dt = 0.5 * pipe.dx / c0;
            let (i1, i2) = (600, 800);
            let (mut s1, mut s2) = (Vec::new(), Vec::new());
            for _ in 0..(0.016 / dt) as usize {
                pipe.step(dt);
                if let Some(l) = lining.as_mut() {
                    l.step(&mut pipe, dt);
                }
                s1.push(pipe.prim(i1).p - p0);
                s2.push(pipe.prim(i2).p - p0);
            }
            amplitude(&s2, f, dt) / amplitude(&s1, f, dt)
        };
        let ratio = run(true) / run(false);
        assert_relative_eq!(ratio, 0.2305, max_relative = 0.03);
    }
}
