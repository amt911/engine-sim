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
/// Clamp on the waves (Pa): the linear model is meaningless beyond a few bar, and two
/// superposed rarefactions must never take the absolute pressure below vacuum.
const WAVE_LIMIT_PA: f64 = 3.0e5;
const WAVE_MIN_PA: f64 = -0.45 * crate::units::ATM_PA;

fn clamp_wave(p: f64) -> f64 {
    p.clamp(WAVE_MIN_PA, WAVE_LIMIT_PA)
}

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
    /// Amplitude a wave keeps crossing the muffler (absorptive silencer, lumped into the tail;
    /// a transfer-matrix muffler is a later refinement — Munjal). 1 = straight pipe.
    pub muffler_transmission: f64,
    /// Exhaust model: linear waveguides (cheap) or non-linear 1D finite volumes (`fv`).
    pub model: ExhaustModel,
    /// Muffler expansion-chamber volume, m³ (finite-volume model).
    pub muffler_volume_m3: f64,
    /// Distance between the outermost tail pipes across the car, m (several tails are spread
    /// evenly between them); where the listener hears each one from.
    pub tail_spacing_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExhaustModel {
    #[default]
    Waveguide,
    Fv,
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
            muffler_transmission: 0.5,
            model: ExhaustModel::Waveguide,
            muffler_volume_m3: 0.006,
            tail_spacing_m: 0.9,
        }
    }
}

impl ExhaustDef {
    /// Collector groups as 0-based cylinder indices (empty → all cylinders in one).
    pub fn groups(&self, cylinders: usize) -> Vec<Vec<usize>> {
        // Invalid indices are dropped (validation reports them; the sim must not panic).
        let groups: Vec<Vec<usize>> = self
            .collectors
            .iter()
            .map(|g| {
                g.iter()
                    .filter(|c| (1..=cylinders).contains(*c))
                    .map(|c| c - 1)
                    .collect::<Vec<_>>()
            })
            .filter(|g| !g.is_empty())
            .collect();
        if groups.is_empty() {
            vec![(0..cylinders).collect()]
        } else {
            groups
        }
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
    /// Radiated pressure at each tail pipe in the last step (one per collector branch).
    tails: Vec<f64>,
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
                )
                .with_extra_gain(def.muffler_transmission),
                end: OpenEnd::new(def.tail_diameter_m, dt),
            })
            .collect();
        Self {
            primaries,
            sent: vec![0.0; cylinders],
            tails: vec![0.0; def.groups(cylinders).len()],
            branches,
            dt,
        }
    }

    /// Radiated pressure at each tail pipe in the last step; they sum to `step`'s result.
    pub fn last_tails(&self) -> &[f64] {
        &self.tails
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
        for (b, tail_out) in self.branches.iter_mut().zip(self.tails.iter_mut()) {
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
                from_far[m] = clamp_wave(outgoing[k]);
            }
            let collector_near = clamp_wave(outgoing[b.members.len()]);
            let mut two = [0.0; 2];
            junction(
                &[b.collector.area_m2, b.tail.area_m2],
                &[b.collector.at_far(), b.tail.at_near()],
                &mut two,
            );
            let (reflected, out) = b.end.process(b.tail.at_far());
            radiated += out;
            *tail_out = out;
            b.collector.send(collector_near, clamp_wave(two[0]));
            b.tail.send(clamp_wave(two[1]), clamp_wave(reflected));
        }
        for i in 0..n {
            let a = self.primaries[i].area_m2;
            let p_plus = clamp_wave(self.primaries[i].at_near() + C_PRIMARY * flows[i] / a);
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
    fn each_tail_reports_its_own_signal_and_they_sum_to_the_radiated() {
        let def = ExhaustDef {
            collectors: vec![vec![1, 3], vec![2, 4]],
            ..ExhaustDef::default()
        };
        let mut net = ExhaustNet::new(&def, 4, DT);
        let mut seen = 0.0;
        for k in 0..20_000 {
            // only cylinder 1 (bank 1) blows: the second tail stays silent
            let flows = [if k % 4000 < 400 { 0.05 } else { 0.0 }, 0.0, 0.0, 0.0];
            let r = net.step(&flows);
            let t = net.last_tails();
            assert_eq!(t.len(), 2);
            assert!((t[0] + t[1] - r).abs() <= 1e-12 * r.abs().max(1.0));
            assert_eq!(t[1], 0.0);
            seen += t[0].abs();
        }
        assert!(seen > 0.0);
    }

    #[test]
    fn tail_spacing_round_trips_and_defaults_for_old_files() {
        let old: ExhaustDef = serde_yaml_ng::from_str("tail_length_m: 1.0\n").unwrap();
        assert_eq!(old.tail_spacing_m, 0.9);
        let d = ExhaustDef {
            tail_spacing_m: 1.4,
            ..ExhaustDef::default()
        };
        let j: ExhaustDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let y: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((j, y), (d.clone(), d));
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
    fn groups_ignore_invalid_indices_instead_of_panicking() {
        // Unvalidated definitions must not panic deep inside the simulation.
        let d = ExhaustDef {
            collectors: vec![vec![0, 1, 9], vec![2]],
            ..ExhaustDef::default()
        };
        assert_eq!(d.groups(2), vec![vec![0], vec![1]]);
        let d = ExhaustDef {
            collectors: vec![vec![0]],
            ..ExhaustDef::default()
        };
        assert_eq!(
            d.groups(2),
            vec![vec![0, 1]],
            "nothing valid → one collector for all"
        );
    }

    #[test]
    fn the_collector_reflection_returns_to_the_valve_after_a_round_trip() {
        // Two primaries into one collector: the junction (areas A, A, Ac) reflects part of a
        // pulse back to its valve with negative sign (opening into more area), after 2·Lp/c.
        let d = ExhaustDef::default();
        let mut net = ExhaustNet::new(&d, 2, DT);
        let round_trip = (2.0 * d.primary_length_m / C_PRIMARY / DT) as usize; // 0.7·2/600 s = 448 steps
        let mut first = None;
        for k in 0..2000usize {
            if net.arriving(0).abs() > 1.0 && first.is_none() {
                first = Some((k, net.arriving(0)));
            }
            net.step(&[if k < 5 { 0.1 } else { 0.0 }, 0.0]);
        }
        let (k, p) = first.expect("reflection never arrived");
        assert!(
            k.abs_diff(round_trip) <= 2,
            "arrived at {k}, expected {round_trip}"
        );
        assert!(
            p < 0.0,
            "expansion into the collector reflects a rarefaction, got {p}"
        );
    }

    #[test]
    fn an_outflow_pulse_radiates_positive_first() {
        let mut net = ExhaustNet::new(&ExhaustDef::default(), 1, DT);
        let mut first = 0.0;
        for k in 0..2000 {
            let r = net.step(&[if k < 20 { 0.2 } else { 0.0 }]);
            if first == 0.0 && r.abs() > 1.0 {
                first = r;
            }
        }
        assert!(first > 0.0, "{first}");
        // two branches add up: the same pulse in each gives twice the output
        let two = ExhaustDef {
            collectors: vec![vec![1], vec![2]],
            ..ExhaustDef::default()
        };
        let one = ExhaustDef {
            collectors: vec![vec![1, 2]],
            ..ExhaustDef::default()
        };
        let total = |d: &ExhaustDef, flows: [f64; 2]| {
            let mut net = ExhaustNet::new(d, 2, DT);
            (0..3000)
                .map(|k| net.step(&if k < 20 { flows } else { [0.0, 0.0] }))
                .sum::<f64>()
        };
        let a = total(&two, [0.2, 0.0]);
        let b = total(&two, [0.2, 0.2]);
        assert!((b - 2.0 * a).abs() < 1e-6 * a.abs(), "{a} {b}");
        assert!(total(&one, [0.2, 0.2]).is_finite());
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

    /// Energy left ringing in the system long after a short pulse (sum of |radiated|).
    fn ringing(def: &ExhaustDef) -> f64 {
        let mut net = ExhaustNet::new(def, 1, DT);
        let mut tail = 0.0;
        for k in 0..40_000 {
            let r = net.step(&[if k < 20 { 0.2 } else { 0.0 }]);
            if k > 20_000 {
                tail += r.abs();
            }
        }
        tail
    }

    #[test]
    fn muffler_damps_resonances() {
        let open = ExhaustDef {
            muffler_transmission: 1.0,
            ..ExhaustDef::default()
        };
        let muffled = ExhaustDef {
            muffler_transmission: 0.5,
            ..ExhaustDef::default()
        };
        // Round trips through the muffler lose 1 − 0.5² = 75 % each way out and back.
        assert!(
            ringing(&muffled) < 0.1 * ringing(&open),
            "{} vs {}",
            ringing(&muffled),
            ringing(&open)
        );
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
