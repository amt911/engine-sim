//! Exhaust system as a network of waveguides: one primary per cylinder, primaries merged in
//! collectors (N-port junctions), each collector into its own tail pipe, which radiates.
//! Spec: `docs/superpowers/specs/2026-10-21-exhaust-waveguides.md`.

use crate::waveguide::{Duct, Node, OpenEnd, Pipe, Shunt, junction};
use serde::{Deserialize, Serialize};

/// Speeds of sound along the system: gas cools from the port to the tail (≈ 900 → 600 K).
/// Estimates; c(T) from the gas state is a later refinement.
pub const C_PRIMARY: f64 = 600.0;
pub const C_COLLECTOR: f64 = 560.0;
pub const C_TAIL: f64 = 500.0;
/// Characteristic impedance ρc of the tail-pipe gas, Pa·s/m: gas at atmospheric pressure whose
/// speed of sound is `C_TAIL` (T = c²/γR = 622 K, ρ = 0.567 kg/m³). Both exhaust models hand
/// the listener ρc·u at each outlet, u the outflow velocity.
pub fn tail_impedance() -> f64 {
    tail_reference_density() * C_TAIL
}

/// Density of the tail-pipe gas, kg/m³ (see [`tail_impedance`]).
pub(crate) fn tail_reference_density() -> f64 {
    let t = C_TAIL * C_TAIL / (crate::gas::GAMMA * crate::gas::R_AIR);
    crate::units::ATM_PA / (crate::gas::R_AIR * t)
}

/// Amplitude loss per metre (wall friction and heat transfer), estimate.
pub const LOSS_PER_M: f64 = 0.03;
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
    /// Extra lumped loss on the tail pipe, amplitude kept per pass (1 = none). The muffler
    /// itself is modelled from `muffler_volume_m3` and `muffler_packing`; this field predates
    /// that and stays for files that set it.
    pub muffler_transmission: f64,
    /// Exhaust model: linear waveguides (cheap) or non-linear 1D finite volumes (`fv`).
    pub model: ExhaustModel,
    /// Muffler expansion-chamber volume, m³ (finite-volume model).
    pub muffler_volume_m3: f64,
    /// Absorptive packing lining the muffler can (finite-volume model).
    pub muffler_packing: crate::lining::Packing,
    /// Distance between the outermost tail pipes across the car, m (several tails are spread
    /// evenly between them); where the listener hears each one from.
    pub tail_spacing_m: f64,
    /// What follows each collector, in order, to the open end. Empty: a can of
    /// `muffler_volume_m3` lined with `muffler_packing`, then the tail pipe (the fields above).
    pub after_collector: Vec<Segment>,
}

/// The mean flow's losses after the collector: turbulent friction Σ f·L/(D·A²) over the pipes
/// (Δp = that × ṁ²/2ρ), a catalyst's laminar resistance (Δp = R·ṁ/ρ), and the open end's area.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExitLosses {
    pub outlet_area: f64,
    pub friction: f64,
    pub laminar: f64,
}

/// One piece of the exhaust after the collector.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Segment {
    /// A plain pipe.
    Pipe { length_m: f64, diameter_m: f64 },
    /// A silencer: a shell of `diameter_m` × `length_m`. With a `tube_diameter_m` > 0 a
    /// perforated straight-through tube of that bore runs inside, the annulus packed with fibre
    /// of `packing_resistivity` (Pa·s/m²) and `packing_porosity` (an absorptive silencer); with
    /// 0 the shell is an empty expansion chamber.
    Muffler {
        length_m: f64,
        diameter_m: f64,
        #[serde(default)]
        tube_diameter_m: f64,
        #[serde(default = "default_resistivity")]
        packing_resistivity: f64,
        #[serde(default = "default_porosity")]
        packing_porosity: f64,
    },
    /// A catalytic converter: a ceramic monolith of square channels, `cells_per_in2` of them
    /// per square inch of its `diameter_m` face, walls `wall_m` thick, `length_m` long.
    Catalyst {
        length_m: f64,
        diameter_m: f64,
        cells_per_in2: f64,
        wall_m: f64,
    },
    /// A Helmholtz resonator off the line between the segments before and after it: a neck of
    /// `neck_diameter_m` × `neck_length_m` into a cavity of `volume_m3`.
    Resonator {
        neck_diameter_m: f64,
        neck_length_m: f64,
        volume_m3: f64,
    },
}

/// A road car's exhaust after one collector, from sizing rules rather than one car's drawings
/// (a starting point to design from; no preset uses it — the rule is not any car's data):
/// a close-coupled catalyst of about the swept volume (standard 400 cpsi / 0.165 mm substrate)
/// and silencers totalling ten times it (a long-standing rule of thumb for muffler volume, e.g.
/// US patent 3,318,412), a fifth in a centre resonator and the rest in the rear box, both
/// straight-through and packed, joined by `pipe_d` pipes. `swept_m3` is this branch's share.
pub fn road_exhaust(swept_m3: f64, pipe_d: f64) -> Vec<Segment> {
    // to a tenth of a millimetre: what a drawing would give (and a file keeps exactly)
    let shell = |volume: f64, length: f64| {
        ((4.0 * volume / (std::f64::consts::PI * length)).sqrt() * 1e4).round() / 1e4
    };
    let pipe = |length_m: f64| Segment::Pipe {
        length_m,
        diameter_m: pipe_d,
    };
    let silencer = |volume: f64, length_m: f64| Segment::Muffler {
        length_m,
        diameter_m: shell(volume, length_m).max(1.5 * pipe_d),
        tube_diameter_m: pipe_d,
        packing_resistivity: default_resistivity(),
        packing_porosity: default_porosity(),
    };
    vec![
        pipe(0.4),
        Segment::Catalyst {
            length_m: 0.13,
            diameter_m: shell(swept_m3, 0.13).max(1.5 * pipe_d),
            cells_per_in2: 400.0,
            wall_m: 0.000_165,
        },
        pipe(1.0),
        silencer(2.0 * swept_m3, 0.35),
        pipe(0.8),
        silencer(8.0 * swept_m3, 0.45),
        pipe(0.3),
    ]
}

/// Viscosity of exhaust gas at `t` K: Sutherland's law for air (μ₀ = 1.716e-5 Pa·s at 273 K,
/// S = 110.4 K; White, *Viscous Fluid Flow*, Table 1-2), Pa·s.
pub(crate) fn gas_viscosity(t: f64) -> f64 {
    1.458e-6 * t.powf(1.5) / (t + 110.4)
}

/// A catalyst monolith's resistance to flow, Pa·s/m³ (pressure drop over volume flow): laminar
/// flow in square channels, f·Re = 56.91 (Shah & London 1978), Δp = 28.45·μ·L·u_ch/d_h², with
/// the channel velocity u_ch = Q/(OFA·A). Cell pitch 25.4 mm/√cpsi, hydraulic diameter pitch −
/// wall, open frontal area (d_h/pitch)².
pub(crate) fn catalyst_resistance(
    length_m: f64,
    diameter_m: f64,
    cells_per_in2: f64,
    wall_m: f64,
    t: f64,
) -> (f64, f64) {
    let pitch = 0.0254 / cells_per_in2.max(1.0).sqrt();
    let dh = (pitch - wall_m).max(1e-5);
    let ofa = (dh / pitch).powi(2);
    let area = std::f64::consts::PI * 0.25 * diameter_m * diameter_m;
    (
        28.45 * gas_viscosity(t) * length_m / (dh * dh) / (ofa * area),
        ofa,
    )
}

fn default_resistivity() -> f64 {
    crate::lining::Packing::default().flow_resistivity
}
fn default_porosity() -> f64 {
    crate::lining::Packing::default().porosity
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
            muffler_transmission: 1.0,
            model: ExhaustModel::Waveguide,
            muffler_volume_m3: 0.006,
            muffler_packing: crate::lining::Packing::default(),
            tail_spacing_m: 0.9,
            after_collector: Vec::new(),
        }
    }
}

impl ExhaustDef {
    /// What follows each collector: the designed segments, or the can and tail of the older
    /// fields (a can no wider than the tail is none; its packing leaves at least the tail's bore).
    pub fn path(&self) -> Vec<Segment> {
        if !self.after_collector.is_empty() {
            return self.after_collector.clone();
        }
        let tail = Segment::Pipe {
            length_m: self.tail_length_m,
            diameter_m: self.tail_diameter_m,
        };
        let tail_area = std::f64::consts::PI * 0.25 * self.tail_diameter_m.powi(2);
        if self.muffler_volume_m3 / CHAMBER_LENGTH_M <= tail_area {
            return vec![tail];
        }
        let (passage, _, packing) =
            chamber_passage(self.muffler_volume_m3, self.muffler_packing, tail_area);
        let can_d = (4.0 * self.muffler_volume_m3 / CHAMBER_LENGTH_M / std::f64::consts::PI).sqrt();
        let tube = if packing.thickness_m > 0.0 {
            (4.0 * passage / std::f64::consts::PI).sqrt()
        } else {
            0.0
        };
        vec![
            Segment::Muffler {
                length_m: CHAMBER_LENGTH_M,
                diameter_m: can_d,
                tube_diameter_m: tube,
                packing_resistivity: packing.flow_resistivity,
                packing_porosity: packing.porosity,
            },
            tail,
        ]
    }

    /// What the mean flow loses on its way out after the collector ([`ExitLosses`]).
    pub fn exit_losses(&self) -> ExitLosses {
        let f = crate::exhaust_fv::PIPE_FRICTION;
        let area = |d: f64| std::f64::consts::PI * 0.25 * d * d;
        let mut friction = 0.0;
        let mut laminar = 0.0;
        for seg in self.path() {
            match seg {
                Segment::Pipe {
                    length_m,
                    diameter_m,
                } => friction += f * length_m / (diameter_m * area(diameter_m).powi(2)),
                Segment::Muffler {
                    length_m,
                    diameter_m,
                    tube_diameter_m,
                    ..
                } => {
                    let d = if tube_diameter_m > 0.0 {
                        tube_diameter_m
                    } else {
                        diameter_m
                    };
                    friction += f * length_m / (d * area(d).powi(2));
                }
                Segment::Catalyst {
                    length_m,
                    diameter_m,
                    cells_per_in2,
                    wall_m,
                } => {
                    laminar +=
                        catalyst_resistance(length_m, diameter_m, cells_per_in2, wall_m, TAIL_T).0
                }
                Segment::Resonator { .. } => {}
            }
        }
        ExitLosses {
            outlet_area: area(self.outlet_diameter()),
            friction,
            laminar,
        }
    }

    /// The tail pipe to edit: (length, diameter) of the designed path's last pipe, or the older
    /// fields when there is no path (or it ends in something else).
    pub fn tail_mut(&mut self) -> (&mut f64, &mut f64) {
        match self.after_collector.last_mut() {
            Some(Segment::Pipe {
                length_m,
                diameter_m,
            }) => (length_m, diameter_m),
            _ => (&mut self.tail_length_m, &mut self.tail_diameter_m),
        }
    }

    /// Bore of the open end, m (the last segment's).
    pub fn outlet_diameter(&self) -> f64 {
        match self.path().last() {
            Some(Segment::Pipe { diameter_m, .. })
            | Some(Segment::Muffler { diameter_m, .. })
            | Some(Segment::Catalyst { diameter_m, .. }) => *diameter_m,
            Some(Segment::Resonator { .. }) | None => self.tail_diameter_m,
        }
    }

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
    /// Muffler can and tail pipe.
    tail: Duct,
    end: OpenEnd,
}

/// Length of the muffler can, m (a typical silencer; its area follows from the volume).
pub(crate) const CHAMBER_LENGTH_M: f64 = 0.4;

/// The gas passage of a muffler can of `volume` (length [`CHAMBER_LENGTH_M`]) lined with
/// `packing`: (area, perimeter, packing as fitted). The packing fills an annulus inside the
/// can wall, leaving at least the tail pipe's own bore open (a straight-through perforate).
pub(crate) fn chamber_passage(
    volume: f64,
    packing: crate::lining::Packing,
    tail_area: f64,
) -> (f64, f64, crate::lining::Packing) {
    let can_area = volume.max(1e-5) / CHAMBER_LENGTH_M;
    let can_r = (can_area / std::f64::consts::PI).sqrt();
    let min_r = (tail_area / std::f64::consts::PI).sqrt();
    let r = (can_r - packing.thickness_m.max(0.0)).max(min_r.min(can_r));
    let fitted = crate::lining::Packing {
        thickness_m: can_r - r,
        ..packing
    };
    (
        std::f64::consts::PI * r * r,
        2.0 * std::f64::consts::PI * r,
        fitted,
    )
}

/// Lined can segments, m: the packing's lumped line converges at about 1 cm (see the
/// waveguide test: 5 % at 4 kHz; 2 cm is 17 % there, where it has already cut 30 dB).
pub const LINING_SEGMENT_M: f64 = 0.01;

/// Gas temperature after the collector, K (the waveguide's tail sound speed, T = c²/(γR)).
const TAIL_T: f64 = C_TAIL * C_TAIL / (crate::gas::GAMMA * crate::gas::R_AIR);

/// What follows a collector, built from [`ExhaustDef::path`]: pipes; silencers whose packing
/// breathes through their perforated tube, cut into 1 cm segments (a lumped lossy line,
/// Munjal ch. 6) or empty shells that reflect at their ends like expansion chambers
/// (ch. 3); catalysts as their open channels plus their laminar resistance; resonators as
/// side branches between their neighbours. `muffler_transmission` stays an extra lumped loss on
/// the last pipe (1 = none).
fn after_collector(def: &ExhaustDef, dt: f64, lining_m: f64) -> Duct {
    let rho_c = crate::gas::GAMMA * crate::units::ATM_PA / C_TAIL;
    let pipe = |len: f64, d: f64| Pipe::new(len, d, C_TAIL, dt, LOSS_PER_M);
    let mut duct: Option<Duct> = None;
    let mut pending = Node::Plain;
    let path = def.path();
    let last_pipe = path
        .iter()
        .rposition(|s| !matches!(s, Segment::Resonator { .. }));
    for (i, seg) in path.iter().enumerate() {
        let piece = match *seg {
            Segment::Pipe {
                length_m,
                diameter_m,
            } => {
                let mut p = pipe(length_m, diameter_m);
                if Some(i) == last_pipe {
                    p = p.with_extra_gain(def.muffler_transmission);
                }
                Duct::new(vec![p])
            }
            Segment::Muffler {
                length_m,
                diameter_m,
                tube_diameter_m,
                packing_resistivity,
                packing_porosity,
            } => {
                if tube_diameter_m > 0.0 && tube_diameter_m < diameter_m {
                    let n = (length_m / lining_m).round().max(2.0) as usize;
                    let pipes = (0..n)
                        .map(|_| pipe(length_m / n as f64, tube_diameter_m))
                        .collect();
                    let packing = crate::lining::Packing {
                        thickness_m: 0.5 * (diameter_m - tube_diameter_m),
                        flow_resistivity: packing_resistivity,
                        porosity: packing_porosity,
                    };
                    let per_node = length_m / (n - 1) as f64;
                    let perimeter = std::f64::consts::PI * tube_diameter_m;
                    Duct::with_shunts(
                        pipes,
                        Shunt::lining(packing, perimeter, per_node, rho_c, dt),
                    )
                } else {
                    Duct::new(vec![pipe(length_m, diameter_m)])
                }
            }
            Segment::Catalyst {
                length_m,
                diameter_m,
                cells_per_in2,
                wall_m,
            } => {
                let (r, ofa) =
                    catalyst_resistance(length_m, diameter_m, cells_per_in2, wall_m, TAIL_T);
                // the channels carry the waves; the walls' friction is the resistance at its end
                let brick = Duct::new(vec![pipe(length_m, diameter_m * ofa.sqrt())]);
                pending = Node::Series(r, rho_c);
                duct = Some(match duct.take() {
                    Some(d) => d.then(brick),
                    None => brick,
                });
                continue;
            }
            Segment::Resonator {
                neck_diameter_m,
                neck_length_m,
                volume_m3,
            } => {
                pending = Node::Shunt(Shunt::helmholtz(
                    neck_diameter_m,
                    neck_length_m,
                    volume_m3,
                    0.0,
                    rho_c,
                    C_TAIL,
                    dt,
                ));
                continue;
            }
        };
        let node = std::mem::replace(&mut pending, Node::Plain);
        duct = Some(match duct.take() {
            Some(d) => d.then_through(node, piece),
            None => piece,
        });
    }
    duct.unwrap_or_else(|| Duct::new(vec![pipe(def.tail_length_m, def.tail_diameter_m)]))
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
    /// The mean flows last handed to the pipes' friction (per cylinder, per branch), kg/s.
    mean_flows: (Vec<f64>, Vec<f64>),
    pub dt: f64,
}

impl ExhaustNet {
    pub fn new(def: &ExhaustDef, cylinders: usize, dt: f64) -> Self {
        Self::with_lining(def, cylinders, dt, LINING_SEGMENT_M)
    }

    /// [`ExhaustNet::new`] with the silencers' packing cut into `lining_m` segments.
    pub fn with_lining(def: &ExhaustDef, cylinders: usize, dt: f64, lining_m: f64) -> Self {
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
                tail: after_collector(def, dt, lining_m),
                end: OpenEnd::new(def.tail_diameter_m, dt),
            })
            .collect();
        Self {
            primaries,
            sent: vec![0.0; cylinders],
            tails: vec![0.0; def.groups(cylinders).len()],
            mean_flows: (Vec::new(), Vec::new()),
            branches,
            dt,
        }
    }

    /// Mean mass flows, kg/s: through each cylinder's primary and each branch's collector and
    /// what follows it (their turbulent friction, [`Pipe::set_mean_flow`]).
    pub fn set_mean_flow(&mut self, per_cylinder: &[f64], per_branch: &[f64]) {
        // reuses the vectors' room: this runs every few dozen steps on the sim thread
        self.mean_flows.0.clear();
        self.mean_flows.0.extend_from_slice(per_cylinder);
        self.mean_flows.1.clear();
        self.mean_flows.1.extend_from_slice(per_branch);
        for (p, m) in self.primaries.iter_mut().zip(per_cylinder) {
            p.set_mean_flow(*m);
        }
        for (b, m) in self.branches.iter_mut().zip(per_branch) {
            b.collector.set_mean_flow(*m);
            b.tail.set_mean_flow(*m);
        }
    }

    /// The mean flows the pipes' friction last followed: (per cylinder, per branch), kg/s.
    pub fn mean_flows(&self) -> (&[f64], &[f64]) {
        (&self.mean_flows.0, &self.mean_flows.1)
    }

    /// Mean outflow Mach number at each tail's open end (see [`OpenEnd::set_outflow_mach`]).
    pub fn set_outflow_mach(&mut self, mach: &[f64]) {
        for (b, m) in self.branches.iter_mut().zip(mach) {
            b.end.set_outflow_mach(*m);
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
                &[b.collector.area_m2, b.tail.end_areas().0],
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

    #[test]
    fn a_catalyst_brick_resists_as_laminar_channels() {
        // 400 cpsi, 0.165 mm walls: pitch 25.4/√400 = 1.27 mm, channel 1.105 mm, open frontal
        // area (1.105/1.27)² = 0.75704. A 0.13 m × 110 mm brick at 600 K (μ = 3.0164e-5):
        // 28.45·μ·L/d_h²/(OFA·A) = 28.45·3.0164e-5·0.13/1.105e-3²/(0.75704·9.5033e-3)
        // = 12 699.6 Pa·s/m³
        let (r, ofa) = catalyst_resistance(0.13, 0.11, 400.0, 0.000_165, 600.0);
        assert!((r / 12_699.6 - 1.0).abs() < 1e-4, "{r}");
        assert!((ofa - 0.757_04).abs() < 1e-5, "{ofa}");
    }

    #[test]
    fn the_exit_losses_add_up_each_piece_by_hand() {
        // Darcy f = 0.025 over each passage: f·L/(d·A²) — a 1 m and a 0.4 m length of 50 mm
        // (the packed silencer's tube) and 0.3 m of an empty 100 mm shell:
        // 0.025·(1.4/(0.05·(1.9635e-3)²) + 0.3/(0.1·(7.8540e-3)²)) = 182 783 m⁻⁴; the catalyst's
        // laminar resistance at the tail's 622.2 K (μ = 3.0888e-5): 13 004.5 Pa·s/m³
        let d = ExhaustDef {
            after_collector: vec![
                Segment::Pipe {
                    length_m: 1.0,
                    diameter_m: 0.05,
                },
                Segment::Muffler {
                    length_m: 0.4,
                    diameter_m: 0.12,
                    tube_diameter_m: 0.05,
                    packing_resistivity: 16_000.0,
                    packing_porosity: 0.95,
                },
                Segment::Muffler {
                    length_m: 0.3,
                    diameter_m: 0.1,
                    tube_diameter_m: 0.0,
                    packing_resistivity: 16_000.0,
                    packing_porosity: 0.95,
                },
                Segment::Catalyst {
                    length_m: 0.13,
                    diameter_m: 0.11,
                    cells_per_in2: 400.0,
                    wall_m: 0.000_165,
                },
                Segment::Pipe {
                    length_m: 0.0,
                    diameter_m: 0.05,
                },
            ],
            ..ExhaustDef::default()
        };
        let l = d.exit_losses();
        assert!(
            (l.friction / 182_783.4 - 1.0).abs() < 1e-5,
            "{}",
            l.friction
        );
        assert!((l.laminar / 13_004.5 - 1.0).abs() < 1e-4, "{}", l.laminar);
        assert!((l.outlet_area - 1.963_495e-3).abs() < 1e-9);
    }

    #[test]
    fn a_silencer_file_without_packing_gets_road_car_fibre() {
        let s: Segment =
            serde_yaml_ng::from_str("kind: muffler\nlength_m: 0.3\ndiameter_m: 0.12\n").unwrap();
        assert_eq!(
            s,
            Segment::Muffler {
                length_m: 0.3,
                diameter_m: 0.12,
                tube_diameter_m: 0.0,
                packing_resistivity: 16_000.0,
                packing_porosity: 0.95,
            }
        );
    }

    #[test]
    fn an_old_can_wider_than_the_tail_is_a_can_and_unpacked_is_empty() {
        // 1.5 L over the 0.4 m can: 3.75e-3 m² of shell against the 1.9635e-3 m² tail — a can
        let mut d = ExhaustDef {
            muffler_volume_m3: 0.0015,
            ..ExhaustDef::default()
        };
        assert_eq!(d.path().len(), 2);
        // without packing the can is an empty chamber: no inner tube
        d.muffler_packing.thickness_m = 0.0;
        let Segment::Muffler {
            tube_diameter_m, ..
        } = d.path()[0]
        else {
            panic!("{:?}", d.path()[0])
        };
        assert_eq!(tube_diameter_m, 0.0);
    }

    #[test]
    fn exhaust_gas_viscosity_follows_sutherland() {
        // 1.458e-6·600^1.5/(600 + 110.4) = 1.458e-6·14696.94/710.4 = 3.0164e-5 Pa·s at 600 K
        // (White's table for air: 3.06e-5)
        assert!((gas_viscosity(600.0) / 3.0164e-5 - 1.0).abs() < 1e-4);
    }

    #[test]
    fn an_old_file_reads_as_a_packed_can_and_a_tail() {
        // the default: a 6 L can of 0.4 m (shell r = √(0.006/0.4/π) = 69.10 mm) lined with
        // 25 mm of packing, so a 88.20 mm perforated tube, then the 1.5 m × 50 mm tail
        let path = ExhaustDef::default().path();
        assert_eq!(path.len(), 2);
        let Segment::Muffler {
            length_m,
            diameter_m,
            tube_diameter_m,
            packing_resistivity,
            packing_porosity,
        } = path[0]
        else {
            panic!("{:?}", path[0])
        };
        assert_eq!(length_m, 0.4);
        assert!((diameter_m - 0.138_198).abs() < 1e-5, "{diameter_m}");
        assert!(
            (tube_diameter_m - 0.088_198).abs() < 1e-5,
            "{tube_diameter_m}"
        );
        assert_eq!((packing_resistivity, packing_porosity), (16_000.0, 0.95));
        assert_eq!(
            path[1],
            Segment::Pipe {
                length_m: 1.5,
                diameter_m: 0.05
            }
        );
        assert_eq!(ExhaustDef::default().outlet_diameter(), 0.05);
        // a can no wider than the tail is no can
        let bare = ExhaustDef {
            muffler_volume_m3: 0.0,
            ..ExhaustDef::default()
        };
        assert_eq!(bare.path().len(), 1);
    }

    #[test]
    fn a_road_exhaust_follows_its_sizing_rules() {
        // 2.0 L in one branch, 50 mm pipes: catalyst of 2.0 L over 0.13 m → Ø 0.1400 m;
        // silencers of 10 × 2.0 L, a fifth (4 L over 0.35 m → Ø 0.1206 m) then the rest (16 L
        // over 0.45 m → Ø 0.2128 m)
        let path = road_exhaust(2.0e-3, 0.05);
        let diameters: Vec<f64> = path
            .iter()
            .filter_map(|s| match s {
                Segment::Catalyst { diameter_m, .. } | Segment::Muffler { diameter_m, .. } => {
                    Some(*diameter_m)
                }
                _ => None,
            })
            .collect();
        assert_eq!(diameters.len(), 3);
        for (got, want) in diameters.iter().zip([0.1400, 0.1206, 0.2128]) {
            assert!((got - want).abs() < 5e-4, "{got} vs {want}");
        }
        assert!(
            matches!(path.last(), Some(Segment::Pipe { diameter_m, .. }) if *diameter_m == 0.05)
        );
    }

    #[test]
    fn a_designed_path_wins_and_round_trips() {
        let def = ExhaustDef {
            after_collector: vec![
                Segment::Catalyst {
                    length_m: 0.12,
                    diameter_m: 0.11,
                    cells_per_in2: 400.0,
                    wall_m: 0.000_165,
                },
                Segment::Pipe {
                    length_m: 0.8,
                    diameter_m: 0.055,
                },
                Segment::Muffler {
                    length_m: 0.5,
                    diameter_m: 0.2,
                    tube_diameter_m: 0.0,
                    packing_resistivity: 16_000.0,
                    packing_porosity: 0.95,
                },
                Segment::Pipe {
                    length_m: 0.4,
                    diameter_m: 0.06,
                },
            ],
            ..ExhaustDef::default()
        };
        assert_eq!(def.path(), def.after_collector);
        assert_eq!(def.outlet_diameter(), 0.06);
        let yaml = serde_yaml_ng::to_string(&def).unwrap();
        assert!(yaml.contains("kind: catalyst"), "{yaml}");
        assert_eq!(serde_yaml_ng::from_str::<ExhaustDef>(&yaml).unwrap(), def);
        let json = serde_json::to_string(&def).unwrap();
        assert_eq!(serde_json::from_str::<ExhaustDef>(&json).unwrap(), def);
    }

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

    /// Peak radiated pressure once settled, a cylinder blowing a tone of `f` Hz into `def`.
    fn tone_through(def: &ExhaustDef, f: f64) -> f64 {
        let mut net = ExhaustNet::with_lining(def, 1, DT, LINING_SEGMENT_M);
        let n = (0.3 / DT) as usize;
        let mut peak: f64 = 0.0;
        for k in 0..n {
            let x = 0.01 * (2.0 * std::f64::consts::PI * f * k as f64 * DT).sin();
            let y = net.step(&[x]);
            if k > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    fn with_path(path: Vec<Segment>) -> ExhaustDef {
        ExhaustDef {
            after_collector: path,
            ..ExhaustDef::default()
        }
    }

    fn packed_can(length_m: f64, tube_diameter_m: f64) -> Segment {
        // the old default can: 138.2 mm shell, 25 mm of 16 kPa·s/m² packing round an 88.2 mm tube
        Segment::Muffler {
            length_m,
            diameter_m: 0.138_198,
            tube_diameter_m,
            packing_resistivity: 16_000.0,
            packing_porosity: 0.95,
        }
    }

    #[test]
    fn a_designed_silencer_attenuates_as_its_packed_line() {
        // the packing lines the tube over the can's length: 0.4 m more of it takes the
        // transmission line's e^(−0.4·Re γ) off a 2 kHz tone, Re γ = 2.4839 Np/m (hand value of
        // the waveguide test for this very passage): 8.63 dB
        let tail = Segment::Pipe {
            length_m: 0.5,
            diameter_m: 0.05,
        };
        let short = tone_through(
            &with_path(vec![packed_can(0.4, 0.088_198), tail.clone()]),
            2000.0,
        );
        let long = tone_through(
            &with_path(vec![packed_can(0.8, 0.088_198), tail.clone()]),
            2000.0,
        );
        let db = 20.0 * (short / long).log10();
        assert!((db - 8.63).abs() < 0.1 * 8.63, "{db} dB");
        // a tube as wide as the shell is no packing: an empty shell, as one with no tube
        let wide = tone_through(
            &with_path(vec![packed_can(0.4, 0.138_198), tail.clone()]),
            2000.0,
        );
        let empty = tone_through(&with_path(vec![packed_can(0.4, 0.0), tail]), 2000.0);
        assert_eq!(wide, empty);
    }

    #[test]
    fn the_net_hands_the_mean_flows_and_the_outflow_to_its_pipes_and_end() {
        // an impulse's ring-down in the tail: with the jet carrying sound off at M = 0.5 each
        // reflection at the open end keeps (1 − M)/(1 + M) = 1/3 of it; with friction from a
        // mean flow every pass wears the waves down. Both leave less ringing after the first
        // arrival than the still, stagnant system.
        let def = ExhaustDef::default();
        let ring = |mach: f64, flow: f64| {
            let mut net = ExhaustNet::new(&def, 1, DT);
            net.set_outflow_mach(&[mach]);
            net.set_mean_flow(&[flow], &[flow]);
            let (mut first, mut late) = (None, 0.0);
            for k in 0..40_000 {
                let y = net.step(&[if k < 20 { 0.2 } else { 0.0 }]);
                if first.is_none() && y.abs() > 1.0 {
                    first = Some(k);
                }
                if let Some(f) = first
                    && k > f + 2_000
                {
                    late += y * y;
                }
            }
            late
        };
        let still = ring(0.0, 0.0);
        assert!(
            ring(0.5, 0.0) < 0.5 * still,
            "outflow: {} vs {still}",
            ring(0.5, 0.0)
        );
        assert!(
            ring(0.0, 0.08) < 0.9 * still,
            "friction: {} vs {still}",
            ring(0.0, 0.08)
        );
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
        // primary, collector, muffler can (0.4 m) and tail: 0.7/600 + 0.25/560 + 0.4/500 + 1.5/500
        // = 1.167 + 0.446 + 0.8 + 3.0 = 5.413 ms = 1039.3 steps at 192 kHz
        let k = first_arrival(&ExhaustDef::default(), 4);
        assert!((1030..=1050).contains(&k), "arrived at step {k}");
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
