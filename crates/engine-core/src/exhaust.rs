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

/// Amplitude loss per metre (wall friction and heat transfer), frequency-independent. Kirchhoff's
/// boundary-layer attenuation α = (1/(r·c))·√(ων/2)·(1 + (γ − 1)/√Pr) (Kinsler et al.,
/// *Fundamentals of Acoustics*, §9.4) gives 0.0295 Np/m in a 42 mm primary of 900 K gas (c 600
/// m/s, ν 1e-4 m²/s) at 200 Hz, about the firing orders' band: 1 − e^(−0.0295) ≈ 0.03.
pub const LOSS_PER_M: f64 = 0.03;
/// Bore a path ends in, m (a silencer's tube if it has one, else its shell); 0 for none.
fn exit_diameter(path: &[Segment]) -> f64 {
    match path.last() {
        Some(Segment::Pipe { diameter_m, .. } | Segment::Catalyst { diameter_m, .. }) => {
            *diameter_m
        }
        Some(Segment::Muffler {
            diameter_m,
            tube_diameter_m,
            ..
        }) => {
            if *tube_diameter_m > 0.0 {
                *tube_diameter_m
            } else {
                *diameter_m
            }
        }
        Some(Segment::Resonator { .. }) => exit_diameter(&path[..path.len() - 1]),
        None => 0.0,
    }
}

/// Clamp on the waves (strength, Pa: [`crate::valve_boundary::wave_pressure`]). Below: the
/// rarefaction that, doubled where it meets its own echo, leaves vacuum (X = ½, X_s = 0:
/// w = −3.5·p₀); a finite-amplitude wave reaches it only where the gas would cavitate. Above:
/// 3 bar of strength is a pressure ratio of 12, past any exhaust pulse.
const WAVE_LIMIT_PA: f64 = 3.0e5;
const WAVE_MIN_PA: f64 = -3.5 * crate::units::ATM_PA;

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
    /// The collectors' paths joined into one duct before the open end (several banks into one
    /// muffler, as on the Lexus LFA: Okamoto et al., SAE 2011-01-2024, Fig. 19). None: every
    /// collector has its own tail. The waveguide exhaust only; the finite-volume one keeps the
    /// paths apart.
    pub merge: Option<Merge>,
}

/// Where the collectors' paths meet: each path ends in a junction with this shared duct, which
/// leaves through one open end.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Merge {
    /// The shared duct after the junction, in order to the open end.
    pub segments: Vec<Segment>,
    /// Diameter of the one open end (the tips together), m.
    pub outlet_diameter_m: f64,
    /// A silencer's bypass valve after the shared duct (none: the duct leads to the open end).
    pub bypass: Option<Bypass>,
}

/// A silencer with a bypass valve (the LFA's main muffler: Lexus, "Sounds Spectacular",
/// valve-actuated dual-stage titanium silencer). Past the shared duct the gas goes round the
/// `closed` path's chambers, or, once the valve opens, also straight through `open`; both meet
/// again in `tips`, which lead to the open end. The valve is a quasi-steady orifice at the
/// entry of `open` (pressure-loss junction, Benson vol. 1 ch. 8), its loss ½ρ(1/(C_c·x) − 1)²·u²
/// for an opening fraction x (Borda–Carnot past the vena contracta, C_c = 0.61; fully open that
/// is ζ = 0.41, a butterfly's range in Idelchik, *Handbook of Hydraulic Resistance*, §9).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Bypass {
    pub closed: Vec<Segment>,
    pub open: Vec<Segment>,
    pub tips: Vec<Segment>,
    /// Outlets after `tips` (its last chamber), each a pipe radiating from its own open end, side
    /// by side at the car's tail: unequal lengths cancel some orders in the air (Torimoto &
    /// Mabuchi, JP 2008-169751 A: the LFA's three tips, the lower one at least twice as long).
    /// Empty: `tips` ends in the merge's outlet.
    pub outlets: Vec<Vec<Segment>>,
    /// Open-area ratio of the perforated baffle the chamber path starts through (the gas
    /// passes small holes into the side chambers, JP 2008-169751 A); 1 for none.
    pub baffle_open_area: f64,
    /// Engine speed the valve opens above, rpm.
    pub open_rpm: f64,
    /// Valve travel, closed to open, s.
    pub travel_s: f64,
}

impl Default for Bypass {
    fn default() -> Self {
        Self {
            closed: Vec::new(),
            open: Vec::new(),
            tips: Vec::new(),
            outlets: Vec::new(),
            baffle_open_area: 1.0,
            open_rpm: 3000.0,
            travel_s: 0.3,
        }
    }
}

impl Valved {
    /// The tips' far end splits into the outlets, each radiating from its own end: returns
    /// (the wave back into the tips, the outlets' summed volume velocity × ρc per unit of their
    /// summed area — what one outlet of that area would radiate).
    fn radiate_outlets(&mut self) -> (f64, f64) {
        let [areas, arriving, leaving] = &mut self.scratch;
        areas[0] = self.tips.end_areas().1;
        arriving[0] = self.tips.at_far();
        for (k, (d, _)) in self.outlets.iter().enumerate() {
            areas[k + 1] = d.end_areas().0;
            arriving[k + 1] = d.at_near();
        }
        junction(areas, arriving, leaving);
        let (mut flow, mut area) = (0.0, 0.0);
        for (k, (d, e)) in self.outlets.iter_mut().enumerate() {
            let (reflected, out) = e.process(d.at_far());
            let a = d.end_areas().1;
            flow += out * a;
            area += a;
            d.send(clamp_wave(leaving[k + 1]), clamp_wave(reflected));
        }
        (leaving[0], flow / area)
    }

    fn set_outflow_mach(&mut self, mach: f64) {
        for (_, e) in &mut self.outlets {
            e.set_outflow_mach(mach);
        }
    }

    /// One step past the shared duct, whose far end sends `arriving` (area `duct_area`): the
    /// valve moves, the junction where the paths split (the valve's loss on `open`), the one
    /// where they meet, and the open end. Returns (the wave back into the shared duct, the
    /// radiated signal).
    fn step(&mut self, end: &mut OpenEnd, arriving: f64, duct_area: f64, dt: f64) -> (f64, f64) {
        let rate = if self.travel_s > 0.0 {
            dt / self.travel_s
        } else {
            1.0
        };
        self.x = if self.opening {
            (self.x + rate).min(1.0)
        } else {
            (self.x - rate).max(0.0)
        };
        let rho = tail_reference_density();
        let z = rho * C_TAIL;
        let shut = self.x <= 0.0;
        let baffle = baffle_loss(self.baffle, rho);
        let (open_near, open_far) = (self.open.at_near(), self.open.at_far());
        // split: the shared duct, the closed path and (unless shut) the valve into the open path
        let mut split = [0.0; 3];
        let (to_open, to_tips_from_open);
        if shut {
            crate::waveguide::lossy_junction(
                &[duct_area, self.closed.end_areas().0],
                &[arriving, self.closed.at_near()],
                &[(0.0, 0.0), (baffle, baffle)],
                z,
                &mut split[..2],
            );
            to_open = open_near; // the shut valve reflects as a wall
        } else {
            let loss = valve_loss(self.x, rho);
            crate::waveguide::lossy_junction(
                &[
                    duct_area,
                    self.closed.end_areas().0,
                    self.open.end_areas().0,
                ],
                &[arriving, self.closed.at_near(), open_near],
                &[(0.0, 0.0), (baffle, baffle), (loss, loss)],
                z,
                &mut split,
            );
            to_open = split[2];
        }
        // meet: the closed path, the open path (a wall while shut) and the tips
        let mut meet = [0.0; 3];
        if shut {
            let mut two = [0.0; 2];
            junction(
                &[self.closed.end_areas().1, self.tips.end_areas().0],
                &[self.closed.at_far(), self.tips.at_near()],
                &mut two,
            );
            (meet[0], meet[2]) = (two[0], two[1]);
            to_tips_from_open = open_far;
        } else {
            junction(
                &[
                    self.closed.end_areas().1,
                    self.open.end_areas().1,
                    self.tips.end_areas().0,
                ],
                &[self.closed.at_far(), open_far, self.tips.at_near()],
                &mut meet,
            );
            to_tips_from_open = meet[1];
        }
        let (reflected, out) = if self.outlets.is_empty() {
            end.process(self.tips.at_far())
        } else {
            self.radiate_outlets()
        };
        self.closed.send(clamp_wave(split[1]), clamp_wave(meet[0]));
        self.open
            .send(clamp_wave(to_open), clamp_wave(to_tips_from_open));
        self.tips.send(clamp_wave(meet[2]), clamp_wave(reflected));
        (clamp_wave(split[0]), out)
    }
}

/// Contraction coefficient of the bypass valve's jet (sharp-edged orifice).
const VALVE_CONTRACTION: f64 = 0.61;

/// Pressure-loss factor s (Δp = s·u|u|) of a valve open by `x` of its pipe's area, in gas of
/// density `rho`; infinite when shut.
fn valve_loss(x: f64, rho: f64) -> f64 {
    if x <= 0.0 {
        return f64::INFINITY;
    }
    0.5 * rho * (1.0 / (VALVE_CONTRACTION * x.min(1.0)) - 1.0).powi(2)
}

/// Pressure-loss factor s of a perforated baffle of `open_area` (its holes' share of the pipe),
/// in gas of density `rho`: each hole an orifice, ½ρ(1/(C_c·σ) − 1)² (Idelchik, *Handbook of
/// Hydraulic Resistance*, §8, thin perforated plates; the same jet as [`valve_loss`]). None
/// (σ ≥ 1): no loss.
fn baffle_loss(open_area: f64, rho: f64) -> f64 {
    if open_area >= 1.0 {
        return 0.0;
    }
    valve_loss(open_area, rho)
}

impl Default for Merge {
    fn default() -> Self {
        Self {
            segments: Vec::new(),
            outlet_diameter_m: 0.07,
            bypass: None,
        }
    }
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
        close_coupled_catalyst(swept_m3, pipe_d),
        pipe(1.0),
        silencer(2.0 * swept_m3, 0.35),
        pipe(0.8),
        silencer(8.0 * swept_m3, 0.45),
        pipe(0.3),
    ]
}

/// A shell of `volume` and `length`, its diameter to a tenth of a millimetre: what a drawing
/// would give (and a file keeps exactly).
fn shell(volume: f64, length: f64) -> f64 {
    ((4.0 * volume / (std::f64::consts::PI * length)).sqrt() * 1e4).round() / 1e4
}

/// The catalyst right after a collector whose branch sweeps `swept_m3`, between `pipe_d` pipes:
/// a brick of about the swept volume (a common sizing; estimate, no car's drawing), 0.13 m of the
/// standard 400 cpsi / 6.5 mil (0.165 mm) substrate, never narrower than 1.5 pipes.
pub fn close_coupled_catalyst(swept_m3: f64, pipe_d: f64) -> Segment {
    Segment::Catalyst {
        length_m: 0.13,
        diameter_m: shell(swept_m3, 0.13).max(1.5 * pipe_d),
        cells_per_in2: 400.0,
        wall_m: 0.000_165,
    }
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

/// The gas a duct carries: sound speed (m/s), density (kg/m³), viscosity (Pa·s), pressure (Pa).
#[derive(Debug, Clone, Copy)]
pub(crate) struct DuctGas {
    pub c: f64,
    pub rho: f64,
    pub mu: f64,
    pub p: f64,
}

/// A catalyst monolith as a ladder of its open channels: lossless pipes of the open area,
/// joined by the channels' viscous series impedance and thermal shunt admittance
/// ([`crate::narrow_tube`], square channels taken as circular of their hydraulic radius), each
/// node carrying a section's worth. Sections are as short as the pipes allow at `dt` (a pipe
/// half a section long must hold a step's travel). The DC resistance is the square channels'
/// laminar one, f·Re = 56.91 (Shah & London 1978) against the circular tube's 64.
pub(crate) fn catalyst_brick(
    length_m: f64,
    diameter_m: f64,
    cells_per_in2: f64,
    wall_m: f64,
    gas: DuctGas,
    dt: f64,
) -> Duct {
    use crate::waveguide::{Node, Pipe, SeriesNet, ShuntNet};
    let pitch = 0.0254 / cells_per_in2.max(1.0).sqrt();
    let dh = (pitch - wall_m).max(1e-5);
    let ofa = (dh / pitch).powi(2);
    let open = std::f64::consts::PI * 0.25 * diameter_m * diameter_m * ofa;
    let d_open = diameter_m * ofa.sqrt();
    let (r0, viscous) = crate::narrow_tube::viscous(0.5 * dh, open, gas.rho, gas.mu);
    let thermal = crate::narrow_tube::thermal(0.5 * dh, open, gas.rho, gas.mu, gas.p);
    let n = ((length_m / (2.0 * gas.c * dt)).floor() as usize).max(2);
    let section = length_m / n as f64;
    let scaled = |bs: &[crate::narrow_tube::Branch], by: f64| {
        bs.iter()
            .map(|b| crate::narrow_tube::Branch {
                k: b.k * by,
                t_s: b.t_s,
            })
            .collect::<Vec<_>>()
    };
    let rho_c = gas.rho * gas.c;
    // n sections of two half pipes: a series node in each, a shunt between them (the n − 1
    // shunts carry the whole brick's thermal admittance)
    let series = SeriesNet::new(
        r0 * (56.91 / 64.0) * section,
        &scaled(&viscous, section),
        rho_c,
        dt,
    );
    let shunt = ShuntNet::new(&scaled(&thermal, length_m / (n - 1) as f64), rho_c, dt);
    let pipes = (0..2 * n)
        .map(|_| Pipe::new(0.5 * section, d_open, gas.c, dt, 0.0))
        .collect();
    let nodes = (0..2 * n - 1)
        .map(|k| {
            if k % 2 == 0 {
                Node::SeriesNet(series.clone())
            } else {
                Node::ShuntNet(shunt.clone())
            }
        })
        .collect();
    Duct::with_nodes(pipes, nodes)
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
            merge: None,
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

    /// Bore the sound leaves through, m: the merged outlet when the banks join, else each tail's.
    /// With outlets ([`Bypass::outlets`]), the bore of their summed area.
    pub fn radiating_diameter(&self) -> f64 {
        let outlets = self
            .merge
            .as_ref()
            .and_then(|m| m.bypass.as_ref())
            .map(|b| b.outlets.as_slice())
            .unwrap_or_default();
        if !outlets.is_empty() {
            return outlets
                .iter()
                .map(|o| exit_diameter(o).powi(2))
                .sum::<f64>()
                .sqrt();
        }
        self.merge
            .as_ref()
            .map_or_else(|| self.outlet_diameter(), |m| m.outlet_diameter_m)
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
    /// ½ρK of each member's flow into and out of the collector, then the collector's own (none)
    /// ([`waveguide::lossy_junction`]).
    losses: Vec<(f64, f64)>,
    /// Muffler can and tail pipe.
    tail: Duct,
    end: OpenEnd,
}

/// ρc of the primaries' gas, Pa·s/m: an ideal gas at an atmosphere whose speed of sound is
/// [`C_PRIMARY`], ρc = γp/c.
fn primary_impedance() -> f64 {
    crate::gas::GAMMA * crate::units::ATM_PA / C_PRIMARY
}

/// The losses of a collector's junction: each primary's flow expands into the collector,
/// losing ½ρ(1 − σ)²u² (Borda–Carnot), and contracts back into its primary losing ½ρ·0.42(1 − σ)u²
/// (sudden contraction; White, *Fluid Mechanics*, §6.9), σ = A_primary/A_collector, ρ the primary
/// gas's. The joining jets expand into the collector each on its own — the constant-pressure
/// junction of Benson (vol. 1, ch. 8) with a loss on each branch. The collector's own flow passes
/// without one.
fn collector_losses(def: &ExhaustDef, members: usize) -> Vec<(f64, f64)> {
    let sigma = (def.primary_diameter_m / def.collector_diameter_m)
        .powi(2)
        .min(1.0);
    let rho = crate::gas::GAMMA * crate::units::ATM_PA / (C_PRIMARY * C_PRIMARY);
    let expansion = 0.5 * rho * (1.0 - sigma).powi(2);
    let contraction = 0.5 * rho * 0.42 * (1.0 - sigma);
    let mut l = vec![(expansion, contraction); members];
    l.push((0.0, 0.0));
    l
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
/// (ch. 3); catalysts as their narrow channels ([`catalyst_brick`]); resonators as
/// side branches between their neighbours. `muffler_transmission` stays an extra lumped loss on
/// the last pipe (1 = none).
pub(crate) fn after_collector(def: &ExhaustDef, dt: f64, lining_m: f64) -> Duct {
    // with a merge the muffler's transmission factor belongs to the shared duct's last pipe
    let transmission = if def.merge.is_some() {
        1.0
    } else {
        def.muffler_transmission
    };
    segments_duct(
        &def.path(),
        transmission,
        (def.tail_length_m, def.tail_diameter_m),
        dt,
        lining_m,
    )
}

/// The duct of `path`'s segments, its last pipe carrying `transmission`; a lone pipe of
/// `fallback` (length, diameter) if the path is empty. The lining's segment length no longer
/// applies here: a packed silencer is its 2D transmission ([`crate::silencer2d`]), not a lumped
/// line.
fn segments_duct(
    path: &[Segment],
    transmission: f64,
    fallback: (f64, f64),
    dt: f64,
    _lining_m: f64,
) -> Duct {
    let rho_c = crate::gas::GAMMA * crate::units::ATM_PA / C_TAIL;
    let pipe = |len: f64, d: f64| Pipe::new(len, d, C_TAIL, dt, LOSS_PER_M);
    let mut duct: Option<Duct> = None;
    let mut pending = Node::Plain;
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
                    p = p.with_extra_gain(transmission);
                }
                Duct::new(vec![p])
            }
            Segment::Muffler {
                length_m,
                diameter_m,
                tube_diameter_m,
                packing_resistivity,
                packing_porosity: _,
            } => {
                if tube_diameter_m > 0.0 && tube_diameter_m < diameter_m {
                    // the packed (or empty) shell round the perforated tube, by the 2D model:
                    // its transmission on the tube's waves both ways
                    let silencer = crate::silencer2d::Silencer {
                        r1: 0.5 * tube_diameter_m,
                        r2: 0.5 * diameter_m,
                        len: length_m,
                        resistivity: (packing_resistivity > 0.0).then_some(packing_resistivity),
                        perforate: crate::silencer2d::Perforate::default(),
                    };
                    let gas = crate::silencer2d::Gas {
                        c: C_TAIL,
                        rho: crate::units::ATM_PA / (crate::gas::R_AIR * TAIL_T),
                    };
                    Duct::new(vec![
                        pipe(length_m, tube_diameter_m)
                            .with_filter(silencer.transmission_eq(gas, 1.0 / dt)),
                    ])
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
                // the channels carry the waves, with their walls' viscous and thermal losses
                let p = crate::units::ATM_PA;
                let gas = DuctGas {
                    c: C_TAIL,
                    rho: p / (crate::gas::R_AIR * TAIL_T),
                    mu: gas_viscosity(TAIL_T),
                    p,
                };
                catalyst_brick(length_m, diameter_m, cells_per_in2, wall_m, gas, dt)
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
    duct.unwrap_or_else(|| Duct::new(vec![pipe(fallback.0, fallback.1)]))
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
    /// Per-step scratch (reused: no allocation on the sim thread): the waves each primary
    /// receives from its collector, and one junction's areas, incoming and outgoing waves.
    scratch: Scratch,
    /// The mean flows last handed to the pipes' friction (per cylinder, per branch), kg/s.
    mean_flows: (Vec<f64>, Vec<f64>),
    /// The duct the branches' tails join, and its one open end ([`ExhaustDef::merge`]).
    shared: Option<Shared>,
    pub dt: f64,
}

/// The branches' common duct and open end, and each branch's last tail wave (scratch).
#[derive(Debug, Clone)]
struct Shared {
    duct: Duct,
    end: OpenEnd,
    /// Each branch tail's near-end wave this step, sent once the junction is solved.
    near: Vec<f64>,
    /// The tails' far-end area over the outlet's: a tail's Mach to the outlet's.
    tail_to_outlet: f64,
    valved: Option<Valved>,
}

/// A [`Bypass`]'s ducts and its valve's state.
#[derive(Debug, Clone)]
struct Valved {
    closed: Duct,
    open: Duct,
    tips: Duct,
    /// [`Bypass::outlets`]: each pipe and its open end; and the junction's scratch (areas,
    /// arriving, leaving: the tips' far end first).
    outlets: Vec<(Duct, OpenEnd)>,
    scratch: [Vec<f64>; 3],
    /// [`Bypass::baffle_open_area`].
    baffle: f64,
    open_rpm: f64,
    travel_s: f64,
    /// Opening fraction, 0 shut … 1 open.
    x: f64,
    /// Whether the engine is above `open_rpm` (the valve's target).
    opening: bool,
}

/// Most steps a block of the net's work may hold, and fewest worth handing off (below it the
/// hand-off costs more than it saves: very short primaries stay on the engine's thread).
pub const MAX_BLOCK: usize = 64;
pub const MIN_BLOCK: usize = 8;

/// Per-step inputs of the exhaust net for a block of steps, in order: what the engine hands
/// over (steps × valves flows, steps × tails Mach, and the mean flows on the steps that carry
/// them).
#[derive(Debug, Clone, Default)]
pub struct BlockIn {
    pub steps: usize,
    pub flows: Vec<f64>,
    pub mach: Vec<f64>,
    /// Per step: the mean flows follow in `mean` (cylinders, then tails, then the engine speed).
    pub has_mean: Vec<bool>,
    pub mean: Vec<f64>,
}

impl BlockIn {
    pub fn clear(&mut self) {
        self.steps = 0;
        self.flows.clear();
        self.mach.clear();
        self.has_mean.clear();
        self.mean.clear();
    }
}

/// What the net did over a [`BlockIn`]: per step, the wave each primary's collector end sent
/// back toward its valve (steps × valves) and the radiated tail signals (steps × tails).
#[derive(Debug, Clone, Default)]
pub struct BlockOut {
    pub steps: usize,
    pub from_far: Vec<f64>,
    pub tails: Vec<f64>,
}

/// Where an [`Engine`](crate::engine::Engine) sends its exhaust's work when the net runs
/// elsewhere (another thread, in the app): the boundary that keeps this crate free of threads.
///
/// The engine hands the net a [`BlockIn`] of up to [`ExhaustLink::block`] steps at a time and
/// takes the [`BlockOut`]s back in order, never more than two blocks behind (`collect` may
/// wait for the oldest). The engine reads the waves arriving at its valves from the primaries'
/// delay lines it keeps a copy of ([`ArrivalProbe`]), not from the net.
pub trait ExhaustLink: Send {
    /// Steps per block (at most the `max` the engine offered).
    fn block(&self) -> usize;
    /// Take the inputs of the next block (leaving `input` cleared, for reuse).
    fn submit(&mut self, input: &mut BlockIn);
    /// The results of the oldest block not yet collected, waiting for them if need be.
    fn collect(&mut self, out: &mut BlockOut);
    /// No more blocks: whatever runs the net may hand it back.
    fn stop(&mut self) {}
}

/// [`ExhaustLink`] that runs the net right here, at `submit`: the reference the threaded one
/// is compared against (and the test of the engine's split step without threads).
pub struct LocalLink {
    pub net: ExhaustNet,
    block: usize,
    done: std::collections::VecDeque<BlockOut>,
}

impl LocalLink {
    pub fn new(net: ExhaustNet, block: usize) -> Self {
        Self {
            net,
            block,
            done: std::collections::VecDeque::new(),
        }
    }
}

impl ExhaustLink for LocalLink {
    fn block(&self) -> usize {
        self.block
    }
    fn submit(&mut self, input: &mut BlockIn) {
        let mut out = BlockOut::default();
        self.net.run_block(input, &mut out);
        input.clear();
        self.done.push_back(out);
    }
    fn collect(&mut self, out: &mut BlockOut) {
        if let Some(o) = self.done.pop_front() {
            *out = o;
        }
    }
}

/// The waves arriving at the valves while the net is a block or two behind: copies of the
/// primaries' near-end delay lines, fed the waves the net sent into them as its results come
/// back. A primary's line carries a wave `whole` steps, so the waves for the next `whole`
/// steps are already in it (or in what the net is still to hand back): the engine reads them
/// from the copy, the same bits the net itself would read.
#[derive(Debug, Clone)]
pub struct ArrivalProbe {
    pipes: Vec<Pipe>,
}

impl ArrivalProbe {
    pub fn new(net: &ExhaustNet) -> Self {
        Self {
            pipes: net.primaries.clone(),
        }
    }

    /// The primaries' friction follows these mean flows from now on (as in
    /// [`ExhaustNet::set_mean_flow`]).
    pub fn set_mean_flow(&mut self, per_cylinder: &[f64]) {
        for (p, m) in self.pipes.iter_mut().zip(per_cylinder) {
            p.set_mean_flow(*m);
        }
    }

    /// The net's results for steps the copy has not taken yet: `from_far` (steps × valves).
    pub fn push(&mut self, from_far: &[f64]) {
        let n = self.pipes.len();
        for step in from_far.chunks_exact(n) {
            for (p, &w) in self.pipes.iter_mut().zip(step) {
                p.bwd.push(w);
            }
        }
    }

    /// The wave arriving at each valve now, with `ahead` steps' results not pushed yet.
    /// `ahead` must stay below [`ExhaustNet::max_block`]`·2`.
    pub fn arriving(&self, ahead: usize, out: &mut [f64]) {
        for (o, p) in out.iter_mut().zip(&self.pipes) {
            *o = p.at_near_ahead(ahead).unwrap_or(0.0);
        }
    }
}

#[derive(Debug, Clone, Default)]
struct Scratch {
    from_far: Vec<f64>,
    areas: Vec<f64>,
    incoming: Vec<f64>,
    outgoing: Vec<f64>,
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
        let branches: Vec<Branch> = def
            .groups(cylinders)
            .into_iter()
            .map(|members| Branch {
                losses: collector_losses(def, members.len()),
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
            scratch: Scratch::default(),
            shared: def.merge.as_ref().map(|m| {
                let fallback = (def.tail_length_m, m.outlet_diameter_m);
                // the muffler's transmission factor belongs to the last duct before the open end
                let last = |valved: bool| {
                    if valved {
                        1.0
                    } else {
                        def.muffler_transmission
                    }
                };
                let duct = segments_duct(
                    &m.segments,
                    last(m.bypass.is_some()),
                    fallback,
                    dt,
                    lining_m,
                );
                let valved = m.bypass.as_ref().map(|b| {
                    let duct =
                        |path: &[Segment], t: f64| segments_duct(path, t, fallback, dt, lining_m);
                    Valved {
                        closed: duct(&b.closed, 1.0),
                        open: duct(&b.open, 1.0),
                        tips: duct(&b.tips, def.muffler_transmission),
                        outlets: b
                            .outlets
                            .iter()
                            .map(|o| (duct(o, 1.0), OpenEnd::new(exit_diameter(o), dt)))
                            .collect(),
                        scratch: std::array::from_fn(|_| vec![0.0; b.outlets.len() + 1]),
                        baffle: b.baffle_open_area,
                        open_rpm: b.open_rpm,
                        travel_s: b.travel_s,
                        x: 0.0,
                        opening: false,
                    }
                });
                let tail_area = std::f64::consts::PI * 0.25 * def.tail_diameter_m.powi(2);
                let outlet_area = std::f64::consts::PI * 0.25 * def.radiating_diameter().powi(2);
                Shared {
                    duct,
                    end: OpenEnd::new(m.outlet_diameter_m, dt),
                    near: vec![0.0; branches.len()],
                    tail_to_outlet: tail_area / outlet_area,
                    valved,
                }
            }),
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
        if let Some(sh) = &mut self.shared {
            let total: f64 = per_branch.iter().sum();
            sh.duct.set_mean_flow(total);
            if let Some(v) = &mut sh.valved {
                // the open path takes the flow in proportion to its opening (an estimate: the
                // split follows the two paths' resistances)
                v.open.set_mean_flow(total * v.x);
                v.closed.set_mean_flow(total * (1.0 - v.x));
                v.tips.set_mean_flow(total);
            }
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
        if let Some(sh) = &mut self.shared {
            // the tails' volume flows add up through the outlet
            let m = mach.iter().sum::<f64>() * sh.tail_to_outlet;
            sh.end.set_outflow_mach(m);
            if let Some(v) = &mut sh.valved {
                v.set_outflow_mach(m);
            }
        }
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

    /// Gauge pressure at cylinder `i`'s valve end now (p⁺ + p⁻, finite-amplitude:
    /// [`crate::valve_boundary::wave_pressure`]).
    pub fn valve_pressure(&self, i: usize) -> f64 {
        let atm = crate::units::ATM_PA;
        crate::valve_boundary::wave_pressure(self.primaries[i].at_near() + self.sent[i], atm) - atm
    }

    /// How many valves (cylinders) feed the net.
    pub fn valves(&self) -> usize {
        self.primaries.len()
    }

    /// The start of a step: the tails' outflow Mach and, when given, the mean flows the pipes'
    /// friction follows — what [`Engine`](crate::engine::Engine) applies before each
    /// [`ExhaustNet::step`].
    pub fn prepare(&mut self, mach: &[f64], mean: Option<(&[f64], &[f64], f64)>) {
        self.set_outflow_mach(mach);
        if let Some((cyl, branch, rpm)) = mean {
            self.set_mean_flow(cyl, branch);
            self.set_engine_speed(rpm);
        }
    }

    /// The engine speed, rpm, that a bypass valve follows (it opens above its `open_rpm`).
    pub fn set_engine_speed(&mut self, rpm: f64) {
        if let Some(v) = self.shared.as_mut().and_then(|sh| sh.valved.as_mut()) {
            v.opening = rpm > v.open_rpm;
        }
    }

    /// A bypass valve's opening fraction (0 shut … 1 open), if the net has one.
    pub fn bypass_opening(&self) -> Option<f64> {
        self.shared.as_ref()?.valved.as_ref().map(|v| v.x)
    }

    /// The waves each primary last sent toward its valve (from its collector end) in the last
    /// [`ExhaustNet::step`].
    pub fn last_from_far(&self) -> &[f64] {
        &self.scratch.from_far
    }

    /// Steps per block the net can run a block behind its engine with: a block must be at most
    /// half a primary's delay (the engine reads the waves for two blocks of steps from the
    /// primaries' lines as they stood when the oldest block's results were back), capped at
    /// [`MAX_BLOCK`]. Under [`MIN_BLOCK`] it is 0: not worth handing off.
    pub fn max_block(&self) -> usize {
        let whole = self
            .primaries
            .iter()
            .map(|p| p.bwd.whole_steps())
            .min()
            .unwrap_or(0);
        let b = (whole / 2).min(MAX_BLOCK);
        if b < MIN_BLOCK { 0 } else { b }
    }

    /// Run `input`'s steps in order ([`ExhaustNet::prepare`] then [`ExhaustNet::step`] each).
    pub fn run_block(&mut self, input: &BlockIn, out: &mut BlockOut) {
        let n = self.primaries.len();
        let m = self.tails.len();
        out.steps = input.steps;
        out.from_far.clear();
        out.tails.clear();
        let mut mean_at = 0;
        for s in 0..input.steps {
            let mean = if input.has_mean[s] {
                let (cyl, rest) = input.mean[mean_at..].split_at(n);
                mean_at += n + m + 1;
                Some((cyl, &rest[..m], rest[m]))
            } else {
                None
            };
            self.prepare(&input.mach[s * m..(s + 1) * m], mean);
            self.step(&input.flows[s * n..(s + 1) * n]);
            out.from_far.extend_from_slice(&self.scratch.from_far);
            out.tails.extend_from_slice(&self.tails);
        }
    }

    /// Advance one step. `flows[i]`: mass flow (kg/s) from cylinder `i` into its primary
    /// (negative = reversion). Valve end: p⁺ = p⁻ + c·ṁ/A. Returns the radiated pressure
    /// summed over the tail pipes.
    pub fn step(&mut self, flows: &[f64]) -> f64 {
        let n = self.primaries.len();
        let Scratch {
            from_far,
            areas,
            incoming,
            outgoing,
        } = &mut self.scratch;
        from_far.clear();
        from_far.resize(n, 0.0);
        let mut radiated = 0.0;
        for (k, (b, tail_out)) in self
            .branches
            .iter_mut()
            .zip(self.tails.iter_mut())
            .enumerate()
        {
            areas.clear();
            incoming.clear();
            for &m in &b.members {
                areas.push(self.primaries[m].area_m2);
                incoming.push(self.primaries[m].at_far());
            }
            areas.push(b.collector.area_m2);
            incoming.push(b.collector.at_near());
            outgoing.resize(incoming.len(), 0.0);
            crate::waveguide::lossy_junction(
                areas,
                incoming,
                &b.losses,
                primary_impedance(),
                outgoing,
            );
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
            b.collector.send(collector_near, clamp_wave(two[0]));
            match &mut self.shared {
                // the tail's far end meets the shared duct below
                Some(sh) => sh.near[k] = clamp_wave(two[1]),
                None => {
                    let (reflected, out) = b.end.process(b.tail.at_far());
                    radiated += out;
                    *tail_out = out;
                    b.tail.send(clamp_wave(two[1]), clamp_wave(reflected));
                }
            }
        }
        if let Some(sh) = &mut self.shared {
            // the tails' far ends and the shared duct's near end: one pressure junction
            areas.clear();
            incoming.clear();
            for b in &self.branches {
                areas.push(b.tail.end_areas().1);
                incoming.push(b.tail.at_far());
            }
            areas.push(sh.duct.end_areas().0);
            incoming.push(sh.duct.at_near());
            outgoing.resize(incoming.len(), 0.0);
            junction(areas, incoming, outgoing);
            // the wave back into the shared duct's far end, and what the open end radiates
            let (reflected, out) = match &mut sh.valved {
                None => sh.end.process(sh.duct.at_far()),
                Some(v) => v.step(
                    &mut sh.end,
                    sh.duct.at_far(),
                    sh.duct.end_areas().1,
                    self.dt,
                ),
            };
            radiated += out;
            let share = out / self.branches.len().max(1) as f64;
            for ((b, tail_out), (near, back)) in self
                .branches
                .iter_mut()
                .zip(self.tails.iter_mut())
                .zip(sh.near.iter().zip(outgoing.iter()))
            {
                *tail_out = share;
                b.tail.send(*near, clamp_wave(*back));
            }
            sh.duct.send(
                clamp_wave(outgoing[self.branches.len()]),
                clamp_wave(reflected),
            );
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
    /// Steady transmitted amplitude (re a unit incident sine at `f`) through `duct` with an
    /// anechoic source and termination.
    fn transmission(mut duct: crate::waveguide::Duct, f: f64, dt: f64) -> f64 {
        let n = (0.3 / dt) as usize;
        let mut peak: f64 = 0.0;
        for k in 0..n {
            let x = (2.0 * std::f64::consts::PI * f * k as f64 * dt).sin();
            if k > n / 2 {
                peak = peak.max(duct.at_far().abs());
            }
            duct.send(x, 0.0);
        }
        peak
    }

    #[test]
    fn the_clamp_spares_every_wave_the_gas_can_carry() {
        use crate::units::ATM_PA;
        use crate::valve_boundary::wave_pressure;
        // a blowdown's echo off an open end, −0.9 atm of strength, is a real wave (it stands at
        // p₀·(1 − 0.9/7)^7 = 0.3816 atm on quiet gas): it passes untouched
        assert_eq!(super::clamp_wave(-0.9 * ATM_PA), -0.9 * ATM_PA);
        assert!((wave_pressure(-0.9 * ATM_PA, ATM_PA) / ATM_PA - 0.3816).abs() < 1e-4);
        // the floor is the wave whose meeting with its own echo leaves vacuum: X = 1 − 3.5/7 = ½,
        // X_s = ½ + ½ − 1 = 0
        let floor = super::clamp_wave(-10.0 * ATM_PA);
        assert_eq!(floor, -3.5 * ATM_PA);
        assert_eq!(wave_pressure(2.0 * floor, ATM_PA), 0.0);
        assert!(wave_pressure(floor, ATM_PA) > 0.0);
    }

    #[test]
    fn a_catalyst_has_the_measured_transmission_loss() {
        // Elnemr (KTH licentiate thesis, 2011, §4.3.1, Figs. 95 and 100): a production catalyst
        // on a cold zero-flow rig — Ø81 mm in, a 50 mm cone, 12 mm gap, an oval monolith
        // 133 × 116 mm (as a circle of its area, Ø124.2 mm) 130 mm long, 13 mm gap, a 44 mm
        // cone, Ø62 mm out. Its cells, measured on the photograph of its face (Fig. 94b, the
        // 133 mm line as scale; a 2D FFT of the grid): 1.43 mm pitch, ≈ 310 cpsi — the common
        // 300 cpsi / 8 mil substrate. Measured TL: 1.3 dB at 50 Hz, 5.0 at 260, 5.5 at 600, 8.0 at
        // 1000, 9.5 at 1200 (read from Fig. 100); TL = 20·log|p_i/p_t| + 10·log(S_i/S_t).
        // The narrow-tube theory of rigid channels is a lower bound: the part's porous ceramic
        // walls, its mounting mat and bends lose more (≈ 1–2 dB of channel loss in theory at
        // 1 kHz over 13 cm). The model must rise with frequency as the part does, never pass it
        // and stay within 5 dB of it; the old model (channels as a lossless pipe and a fixed
        // resistance) fell to 0.9 dB at 1200 Hz.
        use crate::waveguide::{Duct, Pipe};
        let dt = 1.0 / 192_000.0;
        let air = super::DuctGas {
            c: 343.0,
            rho: 1.204,
            mu: 1.81e-5,
            p: 101_325.0,
        };
        let shell = (0.133f64 * 0.116).sqrt();
        let pipe = |l: f64, d: f64| Pipe::new(l, d, air.c, dt, 0.0);
        let cone = |l: f64, d0: f64, d1: f64| {
            (0..4)
                .map(|k| pipe(l / 4.0, d0 + (d1 - d0) * (k as f64 + 0.5) / 4.0))
                .collect::<Vec<_>>()
        };
        let build = || {
            let mut inlet = vec![pipe(1.0, 0.081)];
            inlet.extend(cone(0.05, 0.081, shell));
            inlet.push(pipe(0.012, shell));
            let mut outlet = vec![pipe(0.013, shell)];
            outlet.extend(cone(0.044, shell, 0.062));
            outlet.push(pipe(1.0, 0.062));
            Duct::new(inlet)
                .then(super::catalyst_brick(
                    0.13, shell, 300.0, 0.000_203, air, dt,
                ))
                .then(Duct::new(outlet))
        };
        let area = 20.0 * (0.081f64 / 0.062).log10();
        let mut got = Vec::new();
        for (f, want) in [
            (50.0, 1.3),
            (260.0, 5.0),
            (600.0, 5.5),
            (1000.0, 8.0),
            (1200.0, 9.5),
        ] {
            let tl = -20.0 * transmission(build(), f, dt).log10() + area;
            got.push((f, tl, want));
        }
        let ok = got
            .iter()
            .all(|&(_, tl, want)| tl <= want + 0.5 && want - tl < 5.0);
        assert!(
            ok && got[4].1 > got[2].1 + 1.5,
            "(f, model, measured): {got:.1?}"
        );
    }

    use super::*;

    /// Two one-cylinder banks of plain pipes, joined into one outlet duct when `merged`.
    fn two_banks(merged: bool) -> ExhaustDef {
        ExhaustDef {
            collectors: vec![vec![1], vec![2]],
            after_collector: vec![Segment::Pipe {
                length_m: 0.5,
                diameter_m: 0.05,
            }],
            merge: merged.then(|| Merge {
                segments: vec![Segment::Pipe {
                    length_m: 0.6,
                    diameter_m: 0.07,
                }],
                outlet_diameter_m: 0.07,
                bypass: None,
            }),
            ..ExhaustDef::default()
        }
    }

    #[test]
    fn the_sound_leaves_through_the_merged_outlet_not_the_bank_pipes() {
        // Unmerged, each tail radiates through its own 50 mm pipe; merged, both banks leave
        // through the one 70 mm outlet (two_banks), whatever the bank pipes measure.
        assert_eq!(two_banks(false).radiating_diameter(), 0.05);
        assert_eq!(two_banks(true).radiating_diameter(), 0.07);
    }

    #[test]
    fn a_merge_round_trips_and_old_files_have_none() {
        let old: ExhaustDef = serde_yaml_ng::from_str("primary_length_m: 0.5\n").unwrap();
        assert_eq!(old.merge, None);
        let d = two_banks(true);
        let json: ExhaustDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d));
    }

    #[test]
    fn merged_banks_hear_each_other_and_leave_through_one_outlet() {
        // a pulse from bank A's valve: apart, bank B's valve never hears it; joined in a shared
        // duct (the LFA's one rear muffler, Okamoto et al. SAE 2011-01-2024 Fig. 19), part of it
        // turns back down bank B at the junction (Kelly–Lochbaum: two 50 mm pipes and a 70 mm
        // one, 2·A_a/ΣA of the pulse goes on into each other branch). And the one outlet's
        // sound is shared by the banks' entries, equal halves
        let dt = 1.0 / 96_000.0;
        let heard_by_b = |merged: bool| {
            let mut net = ExhaustNet::new(&two_banks(merged), 2, dt);
            let mut peak: f64 = 0.0;
            for k in 0..(0.03 / dt) as usize {
                let flows = [if k < 20 { 0.05 } else { 0.0 }, 0.0];
                net.step(&flows);
                peak = peak.max(net.arriving(1).abs());
                if merged && k > 400 {
                    let t = net.last_tails();
                    assert!((t[0] - t[1]).abs() <= 1e-12 * t[0].abs().max(1e-30));
                }
            }
            peak
        };
        let (apart, joined) = (heard_by_b(false), heard_by_b(true));
        assert_eq!(apart, 0.0);
        assert!(joined > 1.0, "bank B hears {joined} Pa");
    }

    /// Two banks into a shared duct and a bypass silencer: the closed path a 1.0 m pipe, the open
    /// one 0.1 m, the tips 0.1 m.
    fn valved_banks() -> ExhaustDef {
        let mut d = two_banks(true);
        let pipe = |length_m: f64, diameter_m: f64| Segment::Pipe {
            length_m,
            diameter_m,
        };
        d.merge.as_mut().unwrap().bypass = Some(Bypass {
            closed: vec![pipe(1.0, 0.05)],
            open: vec![pipe(0.1, 0.07)],
            tips: vec![pipe(0.1, 0.07)],
            outlets: Vec::new(),
            baffle_open_area: 1.0,
            open_rpm: 3000.0,
            travel_s: 0.3,
        });
        d
    }

    /// [`valved_banks`] whose final chamber feeds `outlets` 40 mm pipes of the given lengths.
    fn outlet_banks(lengths: &[f64]) -> ExhaustDef {
        let mut d = valved_banks();
        let b = d.merge.as_mut().unwrap().bypass.as_mut().unwrap();
        b.outlets = lengths
            .iter()
            .map(|&length_m| {
                vec![Segment::Pipe {
                    length_m,
                    diameter_m: 0.04,
                }]
            })
            .collect();
        d
    }

    #[test]
    fn outlets_round_trip_and_radiate_through_their_summed_bore() {
        let d = outlet_banks(&[0.2, 0.6]);
        let json: ExhaustDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d.clone()));
        // two 40 mm outlets carry the area of one √2·40 = 56.57 mm bore (π/4·2·0.04²)
        assert!((d.radiating_diameter() - 0.056_569).abs() < 1e-6);
        // a bypass without outlets still ends in the merge's outlet
        assert_eq!(valved_banks().radiating_diameter(), 0.07);
    }

    #[test]
    fn outlets_of_unequal_length_cancel_where_they_are_half_a_wave_apart() {
        // A chamber feeds two open pipes of lengths L₁ = 0.2 m and L₂ = 0.6 m at one pressure
        // p_J; each passes the volume velocity A·p_J/(ρc·sin kL) out of its open end (an open
        // pipe's transfer, Kinsler et al. §10.3), so their sum vanishes where sin kL₂ = −sin kL₁:
        // kL₂ = kL₁ + π, f = c/(2(L₂ − L₁)) = 500/0.8 = 625 Hz (c = C_TAIL). At 400 Hz the sum is
        // 1/sin(1.005) + 1/sin(3.016) = 1.187 + 7.97 = 9.16 times one pipe's term: no cancelling.
        // (The ends' radiation lag shifts the zero a little: the deepest point is sought near it.)
        let dt = 1.0 / 96_000.0;
        let level = |lengths: &[f64], f: f64| {
            let mut net = ExhaustNet::new(&outlet_banks(lengths), 2, dt);
            net.set_engine_speed(5000.0);
            for _ in 0..(0.4 / dt) as usize {
                net.step(&[0.0, 0.0]);
            }
            let mut peak: f64 = 0.0;
            for k in 0..(0.25 / dt) as usize {
                let q = 1e-4 * (2.0 * std::f64::consts::PI * f * k as f64 * dt).sin();
                let out = net.step(&[q, 0.0]);
                if k as f64 * dt > 0.15 {
                    peak = peak.max(out.abs());
                }
            }
            peak
        };
        let two = [0.2, 0.6];
        let open = level(&two, 400.0);
        let (deepest, at) = (540..=660)
            .step_by(15)
            .map(|f| (level(&two, f as f64), f))
            .fold((f64::INFINITY, 0), |a, b| if b.0 < a.0 { b } else { a });
        assert!(
            20.0 * (deepest / open).log10() < -20.0,
            "{} dB at {at} Hz",
            20.0 * (deepest / open).log10()
        );
        // one outlet alone has no such zero there
        let one = level(&[0.2], at as f64) / level(&[0.2], 400.0);
        assert!(20.0 * one.log10() > -10.0, "{}", 20.0 * one.log10());
    }

    #[test]
    fn a_baffle_loses_its_orifices_head_and_none_without_one() {
        // a plate with 10 % open area: ½ρ(1/(0.61·0.1) − 1)² = ½ρ·15.393² = 236.95·½ρ (ρ = 2);
        // a whole-area "baffle" is no baffle
        assert!((baffle_loss(0.1, 2.0) - 236.95).abs() < 0.01);
        assert_eq!(baffle_loss(1.0, 2.0), 0.0);
    }

    #[test]
    fn a_perforated_baffle_damps_the_chamber_path_so_it_cancels_less() {
        // Valve open, the chamber path (1.0 m) and the short one (0.1 m) meet again: a
        // Herschel–Quincke tube, cancelling near where they are half a wave apart,
        // c/(2·0.9 m) ≈ 278 Hz. A 5 %-open baffle on the chamber path drops its wave by the
        // orifice's u|u| loss (Ingard & Ising 1967: a perforate's resistance at high amplitude),
        // so the two no longer cancel as deeply.
        let dt = 1.0 / 96_000.0;
        let dip = |open_area: f64| {
            let mut d = valved_banks();
            d.merge
                .as_mut()
                .unwrap()
                .bypass
                .as_mut()
                .unwrap()
                .baffle_open_area = open_area;
            let level = |f: f64| {
                let mut net = ExhaustNet::new(&d, 2, dt);
                net.set_engine_speed(5000.0);
                for _ in 0..(0.4 / dt) as usize {
                    net.step(&[0.0, 0.0]);
                }
                let mut peak: f64 = 0.0;
                for k in 0..(0.3 / dt) as usize {
                    let q = 0.02 * (2.0 * std::f64::consts::PI * f * k as f64 * dt).sin();
                    let out = net.step(&[q, 0.0]);
                    if k as f64 * dt > 0.2 {
                        peak = peak.max(out.abs());
                    }
                }
                peak
            };
            let deepest = (200..=360)
                .step_by(20)
                .map(|f| level(f as f64))
                .fold(f64::INFINITY, f64::min);
            20.0 * (deepest / level(150.0)).log10()
        };
        let (bare, baffled) = (dip(1.0), dip(0.05));
        assert!(baffled > bare + 6.0, "bare {bare} dB, baffled {baffled} dB");
    }

    #[test]
    fn a_bypass_round_trips_and_merges_without_one_still_load() {
        let old: Merge = serde_yaml_ng::from_str("outlet_diameter_m: 0.07\n").unwrap();
        assert_eq!(old.bypass, None);
        let d = valved_banks();
        let json: ExhaustDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d));
    }

    #[test]
    fn the_valve_loses_a_butterflys_head_fully_open_and_shuts_tight() {
        // ½ρ(1/C_c − 1)² with C_c = 0.61: (0.6393)² = 0.4087 of ½ρ; half open (1/0.305 − 1)² =
        // 2.2787² = 5.1924; shut: no flow
        assert!((valve_loss(1.0, 2.0) - 0.4087).abs() < 1e-4);
        assert!((valve_loss(0.5, 2.0) - 5.1924).abs() < 1e-4);
        assert_eq!(valve_loss(0.0, 2.0), f64::INFINITY);
    }

    #[test]
    fn the_valve_opens_above_its_speed_over_its_travel() {
        // 0.3 s of travel: 0.15 s after the engine passes 3000 rpm it is half open, at 0.3 s
        // fully; back under, it closes the same way
        let dt = 1.0 / 48_000.0;
        let mut net = ExhaustNet::new(&valved_banks(), 2, dt);
        assert_eq!(net.bypass_opening(), Some(0.0));
        let run = |net: &mut ExhaustNet, s: f64| {
            for _ in 0..(s / dt).round() as usize {
                net.step(&[0.0, 0.0]);
            }
        };
        net.set_engine_speed(2900.0);
        run(&mut net, 0.1);
        assert_eq!(net.bypass_opening(), Some(0.0));
        net.set_engine_speed(3500.0);
        run(&mut net, 0.15);
        assert!((net.bypass_opening().unwrap() - 0.5).abs() < 1e-9);
        run(&mut net, 0.2);
        assert_eq!(net.bypass_opening(), Some(1.0));
        net.set_engine_speed(2000.0);
        run(&mut net, 0.15);
        assert!((net.bypass_opening().unwrap() - 0.5).abs() < 1e-9);
        assert_eq!(
            ExhaustNet::new(&two_banks(true), 2, dt).bypass_opening(),
            None
        );
    }

    #[test]
    fn shut_the_sound_goes_round_the_chambers_open_it_takes_the_short_way() {
        // a pulse from bank A reaches the open end first through whichever path is open: the
        // closed path is 0.9 m longer, 0.9/500 m/s (C_TAIL) = 1.8 ms = 172.8 steps at 96 kHz
        let dt = 1.0 / 96_000.0;
        let first = |rpm: f64| {
            let mut net = ExhaustNet::new(&valved_banks(), 2, dt);
            net.set_engine_speed(rpm);
            // let the valve finish its travel in silence
            for _ in 0..(0.4 / dt) as usize {
                net.step(&[0.0, 0.0]);
            }
            let mut out = Vec::new();
            for k in 0..(0.02 / dt) as usize {
                out.push(net.step(&[if k < 5 { 0.05 } else { 0.0 }, 0.0]));
            }
            let peak = out.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            out.iter().position(|v| v.abs() > 0.05 * peak).unwrap() as f64
        };
        let (shut, open) = (first(1000.0), first(5000.0));
        assert!(
            (shut - open - 172.8).abs() < 4.0,
            "shut {shut}, open {open}"
        );
    }

    #[test]
    fn a_block_is_half_a_primary_delay_capped_and_short_primaries_stay_put() {
        // delay = L/c/dt with c = 600 m/s (C_PRIMARY): 0.7 m at 192 kHz is 224 steps, so half is
        // 112, capped at 64; 0.71 m at 48 kHz is 56.8 steps (whole 56: a block of 28 — 0.7 m would sit on
        // the boundary, 55.999… in floating point); 0.205 m is 16.4 steps
        // (whole 16: a block of 8, the least worth it); 0.19 m is 15.2 (whole 15: 7 < 8: none)
        let block = |length_m: f64, fs: f64| {
            let def = ExhaustDef {
                primary_length_m: length_m,
                ..ExhaustDef::default()
            };
            ExhaustNet::new(&def, 4, 1.0 / fs).max_block()
        };
        assert_eq!(block(0.7, 192_000.0), 64);
        assert_eq!(block(0.71, 48_000.0), 28);
        assert_eq!(block(0.205, 48_000.0), 8);
        assert_eq!(block(0.19, 48_000.0), 0);
        assert_eq!(block(0.05, 48_000.0), 0);
    }

    #[test]
    fn the_probe_reads_the_waves_the_net_would_with_results_still_out() {
        // the net steps on its own; the probe is given each step's result only when it is
        // `late − 1` steps old, and a change of the mean flows at step 300: at every step it
        // reads, with its results that late, the very bits the net reads
        let def = ExhaustDef::default();
        let mut net = ExhaustNet::new(&def, 4, 1.0 / 96_000.0);
        let mut probe = ArrivalProbe::new(&net);
        let late = 2 * net.max_block();
        assert!(late >= 16);
        let mut results: Vec<Vec<f64>> = Vec::new();
        let mut pushed = 0;
        let mut got = [0.0; 4];
        for s in 0..600usize {
            while pushed < s.saturating_sub(late - 1) {
                probe.push(&results[pushed]);
                pushed += 1;
            }
            probe.arriving(s - pushed, &mut got);
            for (i, g) in got.iter().enumerate() {
                assert_eq!(g.to_bits(), net.arriving(i).to_bits(), "step {s} valve {i}");
            }
            let flows = [0.02 * (s as f64 * 0.37).sin(); 4];
            net.step(&flows);
            if s == 300 {
                let mean = [0.05, 0.04, 0.03, 0.02];
                net.set_mean_flow(&mean, &[0.1]);
                probe.set_mean_flow(&mean);
            }
            results.push(net.last_from_far().to_vec());
        }
        assert!(results.iter().flatten().any(|w| *w != 0.0));
    }

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
        // the packed can by the 2D model, through the net: 0.4 m more of it takes 33.82 −
        // 17.34 = 16.48 dB off a 1 kHz tone in tail gas (500 m/s, 0.56742 kg/m³) by the
        // independent prototype (papers/silencers/lining-study/modal2d.py, scipy's Bessel
        // functions; validated against JSV 262 Fig. 7), within the equalizer's 1.5 dB
        let tail = Segment::Pipe {
            length_m: 0.5,
            diameter_m: 0.05,
        };
        let short = tone_through(
            &with_path(vec![packed_can(0.4, 0.088_198), tail.clone()]),
            1000.0,
        );
        let long = tone_through(
            &with_path(vec![packed_can(0.8, 0.088_198), tail.clone()]),
            1000.0,
        );
        let db = 20.0 * (short / long).log10();
        assert!((db - 16.48).abs() < 1.5, "{db} dB");
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
    fn a_collector_loses_the_head_of_the_flow_entering_and_leaving_it() {
        // 40 mm primaries into a 55 mm collector: σ = (40/55)² = 0.52893; gas at c = 600 m/s,
        // ρ = 1.4·101325/600² = 0.39404. ½ρ(1 − σ)² = 0.043721 (Borda–Carnot) and
        // ½ρ·0.42·(1 − σ) = 0.038981 (sudden contraction) (hand)
        let l = collector_losses(&ExhaustDef::default(), 2);
        assert_eq!(l.len(), 3);
        assert!(
            (l[0].0 - 0.043_721).abs() < 1e-6 && (l[1].1 - 0.038_981).abs() < 1e-6,
            "{l:?}"
        );
        assert_eq!(l[2], (0.0, 0.0));
        // and the gas they flow in: ρc = 1.4·101325/600 = 236.425 Pa·s/m (hand)
        assert!((primary_impedance() - 236.425).abs() < 1e-3);
        // in the net: the loss grows as u², so a loud pulse comes back from the junction
        // relatively weaker than a faint one (a lossless junction reflects both alike)
        let reflected = |flow: f64| {
            let mut net = ExhaustNet::new(&ExhaustDef::default(), 2, DT);
            let mut peak: f64 = 0.0;
            for k in 0..600usize {
                peak = peak.max(net.arriving(0).abs());
                net.step(&[if k < 20 { flow } else { 0.0 }, 0.0]);
            }
            peak / flow
        };
        let (faint, loud) = (reflected(1e-4), reflected(0.2));
        assert!(loud < 0.9 * faint, "faint {faint} loud {loud}");
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
        // p⁺ = c·ṁ/A = 600 · 0.1 / (π/4 · 0.04²) = 47 746 Pa of strength (hand), standing at
        // 1 atm·((1 + 47 746/(7·101 325))^7 − 1) = 58 547 Pa gauge (finite amplitude)
        let p = net.valve_pressure(0);
        assert!((58_000.0..59_000.0).contains(&p), "{p}");
        // no flow afterwards: nothing comes back before the round trip, pressure stays bounded
        for _ in 0..50_000 {
            net.step(&[0.0]);
            assert!(net.valve_pressure(0).abs() < 1.0e5);
        }
    }
}
