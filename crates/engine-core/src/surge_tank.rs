//! A surge tank between a group of intake runners and the air, in 1D: a wide quasi-1D pipe along
//! the bank, the runners' trumpets joining it one per cylinder, breathing through an inlet duct
//! open to the atmosphere (Okamoto et al., SAE 2011-01-2024, Fig. 18: the LFA's "400 Hz tuned by
//! surge tank"). A 0D tank would only hold well below the tank's first standing wave; along a bank
//! that wave is in the few-hundred-hertz range the paper tunes.
//!
//! Joints are Benson's constant-pressure junctions (`junction_ghost`, the FV exhaust's
//! collectors): a small node of gas meets the pipe ends around it, each end keeping the
//! characteristic that leaves it. Each runner keeps its own throttle at its trumpet
//! (`throttled_ghost`).

use serde::{Deserialize, Serialize};

use crate::exhaust_fv::{guard, junction_ghost, throttled_ghost};
use crate::fv::{Boundary, Pipe1d, Prim};
use crate::gas::{GasVolume, R_AIR};
use crate::intake_fv::AMBIENT_T_K;
use crate::units::ATM_PA;

/// A surge tank's geometry: its length along the runners and its cross-section, and the inlet
/// duct it breathes through.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SurgeTankDef {
    pub length_m: f64,
    pub area_m2: f64,
    pub inlet_diameter_m: f64,
    pub inlet_length_m: f64,
    /// Where the inlet joins the tank, as a fraction of its length from the first runner's end:
    /// 0 at that end, ½ in the middle (at the runner nearest it). Default 0.
    #[serde(default)]
    pub inlet_at: f64,
    /// Loss coefficient of what the inlet draws through (an air cleaner): Δp = k·½ρu² at the
    /// inlet's open end, the quasi-steady Bernoulli loss Heywood (§6.3.2) gives each intake part.
    /// 0: the inlet opens straight to the air. Default 0.
    #[serde(default)]
    pub inlet_loss: f64,
}

/// Cell length of the tank's and the inlet's pipes, m.
const CELL_M: f64 = 0.01;
/// Length of tank a joint's node holds, m: a cell. A 0D node passes a wave without delay, so a
/// thicker one shortens the tank's acoustic length; a thinner one is unstable (its gas must
/// outlast a sound wave crossing it in a step, c·h ≲ its length, as a cell's does).
const NODE_M: f64 = CELL_M;

#[derive(Debug, Clone)]
pub struct SurgeTank {
    /// The inlet duct: open to the atmosphere at its left end, joined to the tank at its right.
    pub inlet: Pipe1d,
    /// The tank, cut at its joints: `stretches[0]` from the inlet's joint to the first runner's,
    /// one per runner after it, the last ending at a closed wall.
    pub stretches: Vec<Pipe1d>,
    /// Gas at each joint: `nodes[0]` the tank's first end, `nodes[k + 1]` runner k's.
    nodes: Vec<GasVolume>,
    /// The node the inlet joins.
    inlet_node: usize,
    /// Effective-area factor of the inlet's open end through its loss, 1/√(1 + k).
    inlet_psi: f64,
    a_tank: f64,
    a_inlet: f64,
    /// Mass drawn in through the inlet during the last substep, kg.
    pub drawn: f64,
    /// Each runner's draw from its joint over the last runner step: (kg/s, W). Scratch.
    draws: Vec<(f64, f64)>,
}

impl SurgeTank {
    /// A tank for `runners` runners, its joints one per cylinder slot (runner k at
    /// (k + ½)·length/runners from the inlet's end), all gas at rest at ambient.
    pub fn new(def: &SurgeTankDef, runners: usize) -> Self {
        let n = runners.max(1);
        let air = Prim {
            rho: ATM_PA / (R_AIR * AMBIENT_T_K),
            u: 0.0,
            p: ATM_PA,
        };
        let a_tank = def.area_m2.max(1e-6);
        let a_inlet = std::f64::consts::PI * 0.25 * def.inlet_diameter_m.max(1e-3).powi(2);
        // cells of about CELL_M, sized so each pipe is exactly as long as asked
        let pipe = |len: f64, area: f64| {
            let len = len.max(CELL_M);
            let cell = len / (len / CELL_M).round().max(1.0);
            let mut p = Pipe1d::segmented(&[(len, area)], cell, air);
            p.friction = crate::exhaust_fv::PIPE_FRICTION;
            p
        };
        // each joint's node holds NODE_M of the tank: the stretches give that room up (half of it
        // at each runner's joint they touch, the whole of the inlet's, which sits at the
        // tank's end), so the tank holds A·L in all
        let slot = def.length_m.max(CELL_M) / n as f64;
        let mut stretches = vec![pipe(0.5 * slot - 1.5 * NODE_M, a_tank)];
        stretches.extend((1..n).map(|_| pipe(slot - NODE_M, a_tank)));
        stretches.push(pipe(0.5 * slot - 0.5 * NODE_M, a_tank));
        let node = GasVolume::at(ATM_PA, AMBIENT_T_K, a_tank * NODE_M);
        // the inlet keeps its whole length: its joint's extra half cell (below) is a little more
        // volume, where a shorter neck would lose inertia (a 5 cm neck, 10 % of it)
        let mut inlet = pipe(def.inlet_length_m, a_inlet);
        inlet.left = Boundary::Open {
            p: ATM_PA,
            rho: air.rho,
        };
        let inlet_node = if def.inlet_at <= 0.0 {
            0
        } else {
            ((def.inlet_at.min(1.0) * n as f64).floor() as usize).min(n - 1) + 1
        };
        // A joint holds half a cell of every pipe it joins: the tank's (as above) and, at the
        // inlet's joint, the inlet's too. Its gas must outlast what those pipes carry through it
        // in a step; a wide inlet on a node sized for the tank alone was not (a 150 mm inlet on
        // a 0.01 m² tank rang at the mesh rate: 2–10 kHz noise, LFA at 8000 rpm).
        let mut nodes = vec![node; n + 1];
        nodes[inlet_node] = GasVolume::at(ATM_PA, AMBIENT_T_K, (a_tank + 0.5 * a_inlet) * CELL_M);
        Self {
            inlet,
            stretches,
            nodes,
            inlet_psi: 1.0 / (1.0 + def.inlet_loss.max(0.0)).sqrt(),
            inlet_node,
            a_tank,
            a_inlet,
            drawn: 0.0,
            draws: Vec::with_capacity(n),
        }
    }

    /// The largest stable step of the tank's own pipes at Courant number `cfl`.
    pub fn max_dt(&self, cfl: f64) -> f64 {
        self.stretches
            .iter()
            .chain(std::iter::once(&self.inlet))
            .map(|p| p.max_dt(cfl))
            .fold(f64::INFINITY, f64::min)
    }

    /// Advance `h` with `runners` (their valve ends already set) drawing on it through
    /// trumpets of area `a_runner`, each throttled to `psi` = effective area / runner area.
    /// Returns each runner's left-end mass flux (kg/(s·m²), +x: into the runner is negative).
    pub fn substep(&mut self, h: f64, runners: &mut [Pipe1d], a_runner: f64, psi: f64) {
        // the runners first, at their own step, each trumpet reading its node's gas
        let n = self.stretches.len() - 1;
        for (k, r) in runners.iter_mut().enumerate().take(n) {
            let v = &self.nodes[k + 1];
            let (p, rho) = (v.pressure(), v.mass_kg / v.volume_m3);
            r.left = Boundary::State(throttled_ghost(r.prim(0), p, rho, -1.0, psi));
            r.step(h);
        }
        // what each runner drew from its node over that step (mass and energy rates)
        self.draws.clear();
        self.draws.extend(
            runners
                .iter()
                .take(n)
                .map(|r| (r.end_flux.0.rho * a_runner, r.end_flux.0.e * a_runner)),
        );
        // then the tank, as many steps as its own (finer) cells need: multirate, so a fine
        // tank does not drag every runner down to its step; the runners' draw spread evenly
        let m = (h / self.max_dt(0.9)).ceil().max(1.0);
        let ht = h / m;
        self.drawn = 0.0;
        for _ in 0..m as usize {
            self.tank_step(ht);
        }
    }

    /// One step `h` of the tank's own pipes and joints, the runners drawing [`Self::draws`].
    fn tank_step(&mut self, h: f64) {
        let state = |v: &GasVolume| (v.pressure(), v.mass_kg / v.volume_m3);
        let n = self.stretches.len() - 1;
        let last = |p: &Pipe1d| p.prim(p.cells.len() - 1);
        let (p0, r0) = state(&self.nodes[self.inlet_node]);
        self.inlet.right = Boundary::State(junction_ghost(last(&self.inlet), p0, r0, 1.0));
        if self.inlet_psi < 1.0 {
            // the air drawn through the cleaner: the atmosphere behind its loss
            let rho_atm = ATM_PA / (R_AIR * AMBIENT_T_K);
            let first = self.inlet.prim(0);
            self.inlet.left = Boundary::State(throttled_ghost(
                first,
                ATM_PA,
                rho_atm,
                -1.0,
                self.inlet_psi,
            ));
        }
        for k in 0..=n {
            let (pl, rl) = state(&self.nodes[k]);
            let s = &mut self.stretches[k];
            s.left = Boundary::State(junction_ghost(s.prim(0), pl, rl, -1.0));
            s.right = if k == n {
                Boundary::Wall
            } else {
                let (pr, rr) = state(&self.nodes[k + 1]);
                Boundary::State(junction_ghost(last(s), pr, rr, 1.0))
            };
        }
        self.inlet.step(h);
        for s in &mut self.stretches {
            s.step(h);
        }
        // each node takes what its faces carried
        let take = |v: &mut GasVolume, f: crate::fv::Cons, area: f64, sign: f64| {
            v.mass_kg += sign * f.rho * area * h;
            v.energy_j += sign * f.e * area * h;
        };
        take(
            &mut self.nodes[self.inlet_node],
            self.inlet.end_flux.1,
            self.a_inlet,
            1.0,
        );
        // each stretch: its left end leaves node k, its right end enters node k + 1 (the last
        // ends at the wall); runner k draws from node k + 1
        for (k, s) in self.stretches.iter().enumerate() {
            take(&mut self.nodes[k], s.end_flux.0, self.a_tank, -1.0);
            if k < n {
                take(&mut self.nodes[k + 1], s.end_flux.1, self.a_tank, 1.0);
                if let Some(&(dm, de)) = self.draws.get(k) {
                    self.nodes[k + 1].mass_kg -= dm * h;
                    self.nodes[k + 1].energy_j -= de * h;
                }
            }
        }
        for v in &mut self.nodes {
            guard(v);
        }
        self.drawn += self.inlet.end_flux.0.rho * self.a_inlet * h;
    }

    /// Pressure at the closed end's cell, Pa (for the tests' probes).
    pub fn far_end_pressure(&self) -> f64 {
        let s = &self.stretches[self.stretches.len() - 1];
        s.prim(s.cells.len() - 1).p
    }

    /// Total gas mass in the tank, its inlet and its nodes, kg.
    pub fn mass(&self) -> f64 {
        let pipe = |p: &Pipe1d| {
            p.cells
                .iter()
                .zip(&p.area)
                .map(|(c, a)| c.rho * a)
                .sum::<f64>()
                * p.dx
        };
        pipe(&self.inlet)
            + self.stretches.iter().map(pipe).sum::<f64>()
            + self.nodes.iter().map(|v| v.mass_kg).sum::<f64>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The strongest frequency of `x` (sampled every `h`) between 50 Hz and 2 kHz, to 0.5 Hz: a
    /// scan of its spectrum (each candidate's DFT magnitude).
    fn frequency(x: &[f64], h: f64) -> f64 {
        let m = x.iter().sum::<f64>() / x.len() as f64;
        let power = |f: f64| {
            let (c, s) = x.iter().enumerate().fold((0.0, 0.0), |(c, s), (i, v)| {
                let w = 2.0 * std::f64::consts::PI * f * i as f64 * h;
                (c + (v - m) * w.cos(), s + (v - m) * w.sin())
            });
            c * c + s * s
        };
        (100..4000)
            .map(|k| k as f64 * 0.5)
            .max_by(|a, b| power(*a).total_cmp(&power(*b)))
            .unwrap_or(0.0)
    }

    /// Runners with their throttles shut: the tank alone.
    fn shut(n: usize) -> Vec<Pipe1d> {
        let air = Prim {
            rho: ATM_PA / (R_AIR * AMBIENT_T_K),
            u: 0.0,
            p: ATM_PA,
        };
        (0..n).map(|_| Pipe1d::new(0.2, 20, air)).collect()
    }

    /// Raise the tank's gas by +`dp` (isentropically) and record the far-end pressure for `secs`:
    /// it swings back through the inlet at the system's lowest mode.
    fn ring(def: &SurgeTankDef, runners: usize, dp: f64, secs: f64) -> (Vec<f64>, f64) {
        let mut tank = SurgeTank::new(def, runners);
        let lift = |c: &mut crate::fv::Cons| {
            let mut p = c.to_prim();
            p.rho *= ((p.p + dp) / p.p).powf(1.0 / crate::gas::GAMMA);
            p.p += dp;
            *c = p.to_cons();
        };
        for s in tank.stretches.iter_mut() {
            s.cells.iter_mut().for_each(lift);
        }
        for v in tank.nodes.iter_mut() {
            *v = GasVolume::at(
                ATM_PA + dp,
                v.temperature() * ((ATM_PA + dp) / ATM_PA).powf(0.4 / 1.4),
                v.volume_m3,
            );
        }
        let mut runners = shut(runners);
        let h = 0.8 * tank.max_dt(0.8);
        let x = (0..(secs / h) as usize)
            .map(|_| {
                tank.substep(h, &mut runners, 1e-3, 0.0);
                tank.far_end_pressure() - ATM_PA
            })
            .collect();
        (x, h)
    }

    #[test]
    fn a_tank_as_wide_as_its_inlet_is_a_quarter_wave_pipe() {
        // inlet and tank of one bore: a uniform pipe 0.2 + 0.5 = 0.7 m, open at the inlet, closed
        // at the far end: f₁ = c/4L = 347.19/2.8 = 124.0 Hz (c = √(1.4·287·300); no end
        // correction in 1D). The six junction nodes are 0D: each passes the wave without the
        // delay of the cell it stands for, so the pipe sounds up to 6·0.01/(2·0.7) ≈ 4 % short —
        // the lumped junction's known error, hence 5 %.
        let d: f64 = 0.1;
        let def = SurgeTankDef {
            length_m: 0.5,
            area_m2: std::f64::consts::PI * 0.25 * d * d,
            inlet_diameter_m: d,
            inlet_loss: 0.0,
            inlet_at: 0.0,
            inlet_length_m: 0.2,
        };
        let (x, h) = ring(&def, 5, 200.0, 0.1);
        let f = frequency(&x, h);
        assert!((f / 124.0 - 1.0).abs() < 0.05, "{f} Hz");
    }

    #[test]
    fn a_short_wide_tank_behind_a_narrow_inlet_rings_as_helmholtz() {
        // 0.06 m long × 0.04 m² = 2.4 L behind a 70 mm × 0.05 m inlet: a squat tank far below
        // its own waves (c/2L = 2.9 kHz) and too wide to add inertia of its own, so a Helmholtz
        // resonator. With the neck's air distributed along it (the 1D model, no end
        // corrections) the resonance solves kL·tan(kL) = A·L/V = 3.8485e-3·0.05/2.4e-3 = 0.08018:
        // kL = 0.27943, f = kc/2π = 0.27943/0.05·347.19/2π = 308.8 Hz (Kinsler et al.,
        // *Fundamentals of Acoustics*, §10.8; the lumped c/2π·√(A/VL) = 312.9 Hz is its limit)
        let def = SurgeTankDef {
            length_m: 0.06,
            area_m2: 0.04,
            inlet_diameter_m: 0.07,
            inlet_loss: 0.0,
            inlet_at: 0.0,
            inlet_length_m: 0.05,
        };
        let (x, h) = ring(&def, 1, 200.0, 0.05);
        let f = frequency(&x, h);
        assert!((f / 308.8 - 1.0).abs() < 0.03, "{f} Hz");
    }

    #[test]
    fn drawing_through_an_air_cleaner_costs_its_loss() {
        // Air drawn steadily through an inlet whose cleaner has loss k = 35.1: the air reaches
        // the duct through an effective area 1/√(1 + k) of it with no pressure recovery
        // (Benson's partially open end), so the duct's static pressure sits (1 + k)·½ρu² below
        // the atmosphere — the cleaner's k·½ρu² plus the duct's own dynamic head. The runners
        // empty into a sink 8 kPa below the atmosphere, which absorbs the waves (a steady draw).
        let def = SurgeTankDef {
            length_m: 0.485,
            area_m2: 0.01,
            inlet_diameter_m: 0.15,
            inlet_length_m: 0.03,
            inlet_at: 0.5,
            inlet_loss: 35.1,
        };
        let mut tank = SurgeTank::new(&def, 5);
        let mut runners = shut(5);
        let rho = ATM_PA / (R_AIR * AMBIENT_T_K);
        for r in runners.iter_mut() {
            r.right = Boundary::Open {
                p: ATM_PA - 8000.0,
                rho,
            };
        }
        let a_runner = 0.045f64.powi(2) * std::f64::consts::PI / 4.0;
        let h = 0.8 * tank.max_dt(0.8).min(runners[0].max_dt(0.8));
        let n = (0.5 / h) as usize;
        let (mut dp, mut head, mut count) = (0.0, 0.0, 0);
        for step in 0..n {
            tank.substep(h, &mut runners, a_runner, 1.0);
            if step > n / 2 {
                let c = tank.inlet.prim(0);
                dp += ATM_PA - c.p;
                head += 0.5 * c.rho * c.u * c.u;
                count += 1;
            }
        }
        let ratio = dp / head;
        assert!(
            head / count as f64 > 10.0,
            "barely any flow: {} Pa",
            head / count as f64
        );
        assert!((ratio / 36.1 - 1.0).abs() < 0.1, "Δp/(½ρu²) = {ratio}");
    }

    #[test]
    fn a_shut_tank_keeps_its_air() {
        // throttles shut, inlet open: what rings out through the inlet comes back, nothing leaks
        let def = SurgeTankDef {
            length_m: 0.485,
            area_m2: 0.01,
            inlet_diameter_m: 0.07,
            inlet_loss: 0.0,
            inlet_at: 0.0,
            inlet_length_m: 0.2,
        };
        let mut tank = SurgeTank::new(&def, 5);
        let m0 = tank.mass();
        let mut runners = shut(5);
        let h = 0.8 * tank.max_dt(0.8);
        for _ in 0..(0.2 / h) as usize {
            tank.substep(h, &mut runners, 1e-3, 0.0);
        }
        assert!(
            (tank.mass() / m0 - 1.0).abs() < 1e-3,
            "{} vs {m0}",
            tank.mass()
        );
    }
}
