//! Exhaust system as a network of waveguides: one primary per cylinder, primaries merged in
//! collectors (N-port junctions), each collector into its own tail pipe, which radiates.
//! Spec: `docs/superpowers/specs/2026-10-21-exhaust-waveguides.md`.

use crate::waveguide::{OpenEnd, Pipe, junction};
use serde::{Deserialize, Serialize};

/// Speeds of sound along the system: gas cools from the port to the tail (≈ 900 → 600 K).
/// Estimates; c(T) from the gas state is a later refinement.
pub const C_PRIMARY: f64 = 600.0;
pub const C_COLLECTOR: f64 = 560.0;
pub const C_TAIL: f64 = 500.0;
/// Amplitude loss per metre (wall friction and heat transfer), estimate.
pub const LOSS_PER_M: f64 = 0.03;
/// Temperature of exhaust gas pushed back into a cylinder (reversion), K.
pub const REVERSION_TEMP_K: f64 = 800.0;
/// Clamp on the waves (Pa): the linear model is meaningless beyond a few bar.
const WAVE_LIMIT_PA: f64 = 3.0e5;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ExhaustDef {
    pub primary_length_m: f64,
    pub primary_diameter_m: f64,
    /// Groups of cylinders (1-based) merged in each collector; empty = one collector for all.
    pub collectors: Vec<Vec<usize>>,
    pub collector_length_m: f64,
    pub collector_diameter_m: f64,
    pub tail_length_m: f64,
    pub tail_diameter_m: f64,
}

impl Default for ExhaustDef {
    fn default() -> Self {
        Self {
            primary_length_m: 0.7,
            primary_diameter_m: 0.040,
            collectors: Vec::new(),
            collector_length_m: 0.25,
            collector_diameter_m: 0.055,
            tail_length_m: 1.5,
            tail_diameter_m: 0.050,
        }
    }
}

impl ExhaustDef {
    /// Collector groups as 0-based cylinder indices (empty → all cylinders in one).
    pub fn groups(&self, cylinders: usize) -> Vec<Vec<usize>> {
        if self.collectors.is_empty() {
            return vec![(0..cylinders).collect()];
        }
        self.collectors
            .iter()
            .map(|g| g.iter().map(|c| c - 1).collect())
            .collect()
    }
}

#[derive(Debug, Clone)]
struct Branch {
    members: Vec<usize>,
    collector: Pipe,
    tail: Pipe,
    end: OpenEnd,
}

/// Runtime network for one engine at one time step.
#[derive(Debug, Clone)]
pub struct ExhaustNet {
    primaries: Vec<Pipe>,
    /// Last wave sent into each primary from its valve end.
    sent: Vec<f64>,
    branches: Vec<Branch>,
    pub dt: f64,
}

impl ExhaustNet {
    pub fn new(def: &ExhaustDef, cylinders: usize, dt: f64) -> Self {
        let primaries = (0..cylinders)
            .map(|_| {
                Pipe::new(
                    def.primary_length_m,
                    def.primary_diameter_m,
                    C_PRIMARY,
                    dt,
                    LOSS_PER_M,
                )
            })
            .collect();
        let branches = def
            .groups(cylinders)
            .into_iter()
            .map(|members| Branch {
                members,
                collector: Pipe::new(
                    def.collector_length_m,
                    def.collector_diameter_m,
                    C_COLLECTOR,
                    dt,
                    LOSS_PER_M,
                ),
                tail: Pipe::new(
                    def.tail_length_m,
                    def.tail_diameter_m,
                    C_TAIL,
                    dt,
                    LOSS_PER_M,
                ),
                end: OpenEnd::new(def.tail_diameter_m, dt),
            })
            .collect();
        Self {
            primaries,
            sent: vec![0.0; cylinders],
            branches,
            dt,
        }
    }

    pub fn primary_area(&self) -> f64 {
        self.primaries.first().map_or(1.0, |p| p.area_m2)
    }

    /// Wave arriving at cylinder `i`'s valve end this step (p⁻, gauge).
    pub fn arriving(&self, i: usize) -> f64 {
        self.primaries[i].at_near()
    }

    /// Port impedance c/A of the primaries (Pa per kg/s).
    pub fn port_impedance(&self) -> f64 {
        C_PRIMARY / self.primary_area()
    }

    /// Gauge pressure at cylinder `i`'s valve end now (p⁺ + p⁻).
    pub fn valve_pressure(&self, i: usize) -> f64 {
        self.primaries[i].at_near() + self.sent[i]
    }

    /// Advance one step. `flows[i]`: mass flow (kg/s) from cylinder `i` into its primary
    /// (negative = reversion). Valve end: p⁺ = p⁻ + c·ṁ/A. Returns the radiated pressure
    /// summed over the tail pipes.
    pub fn step(&mut self, flows: &[f64]) -> f64 {
        let n = self.primaries.len();
        let mut from_far = vec![0.0; n];
        let mut radiated = 0.0;
        let mut areas = Vec::new();
        let mut incoming = Vec::new();
        let mut outgoing = Vec::new();
        for b in &mut self.branches {
            areas.clear();
            incoming.clear();
            for &m in &b.members {
                areas.push(self.primaries[m].area_m2);
                incoming.push(self.primaries[m].at_far());
            }
            areas.push(b.collector.area_m2);
            incoming.push(b.collector.at_near());
            outgoing.resize(incoming.len(), 0.0);
            junction(&areas, &incoming, &mut outgoing);
            for (k, &m) in b.members.iter().enumerate() {
                from_far[m] = outgoing[k];
            }
            let collector_near = outgoing[b.members.len()];
            let mut two = [0.0; 2];
            junction(
                &[b.collector.area_m2, b.tail.area_m2],
                &[b.collector.at_far(), b.tail.at_near()],
                &mut two,
            );
            let (reflected, out) = b.end.process(b.tail.at_far());
            radiated += out;
            b.collector.send(collector_near, two[0]);
            b.tail.send(two[1], reflected);
        }
        for i in 0..n {
            let a = self.primaries[i].area_m2;
            let p_plus = (self.primaries[i].at_near() + C_PRIMARY * flows[i] / a)
                .clamp(-WAVE_LIMIT_PA, WAVE_LIMIT_PA);
            self.primaries[i].send(p_plus, from_far[i]);
            self.sent[i] = p_plus;
        }
        radiated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 192_000.0;

    fn first_arrival(def: &ExhaustDef, n: usize) -> usize {
        let mut net = ExhaustNet::new(def, n, DT);
        let mut flows = vec![0.0; n];
        for k in 0..20_000 {
            flows[0] = if k < 20 { 0.2 } else { 0.0 };
            if net.step(&flows).abs() > 1.0 {
                return k;
            }
        }
        panic!("nothing radiated");
    }

    #[test]
    fn groups_default_to_one_collector() {
        assert_eq!(ExhaustDef::default().groups(3), vec![vec![0, 1, 2]]);
        let d = ExhaustDef {
            collectors: vec![vec![1, 3], vec![2, 4]],
            ..ExhaustDef::default()
        };
        assert_eq!(d.groups(4), vec![vec![0, 2], vec![1, 3]]);
    }

    #[test]
    fn silence_stays_silent() {
        let mut net = ExhaustNet::new(&ExhaustDef::default(), 4, DT);
        for _ in 0..10_000 {
            assert_eq!(net.step(&[0.0; 4]), 0.0);
        }
    }

    #[test]
    fn pulse_reaches_the_tail_after_the_travel_time() {
        // 0.7/600 + 0.25/560 + 1.5/500 = 1.167 + 0.446 + 3.0 = 4.613 ms = 885.7 steps at 192 kHz
        let k = first_arrival(&ExhaustDef::default(), 4);
        assert!((880..=900).contains(&k), "arrived at step {k}");
    }

    #[test]
    fn longer_primaries_arrive_later() {
        let short = first_arrival(&ExhaustDef::default(), 2);
        let long = first_arrival(
            &ExhaustDef {
                primary_length_m: 1.3,
                ..ExhaustDef::default()
            },
            2,
        );
        // 0.6 m more at 600 m/s = 1 ms = 192 steps
        assert!((185..=200).contains(&(long - short)), "{short} → {long}");
    }

    #[test]
    fn outflow_raises_valve_pressure_and_closed_valve_reflects() {
        let mut net = ExhaustNet::new(&ExhaustDef::default(), 1, DT);
        net.step(&[0.1]);
        // p⁺ = c·ṁ/A = 600 · 0.1 / (π/4 · 0.04²) = 47 746 Pa (hand)
        let p = net.valve_pressure(0);
        assert!((47_000.0..48_500.0).contains(&p), "{p}");
        // no flow afterwards: nothing comes back before the round trip, pressure stays bounded
        for _ in 0..50_000 {
            net.step(&[0.0]);
            assert!(net.valve_pressure(0).abs() < 1.0e5);
        }
    }
}
