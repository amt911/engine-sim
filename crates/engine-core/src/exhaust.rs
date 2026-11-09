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
/// Running radial clearance of a hot exhaust butterfly round its disc, m: ESTIMATE, a gap left
/// for the disc's thermal growth (≈ 0.5 mm on 50–90 mm valves).
pub const BUTTERFLY_CLEARANCE_M: f64 = 0.0005;

/// Share of a butterfly's pipe a radial gap of `clearance_m` round its disc leaves open: the ring
/// π·D·c over the bore π·D²/4, 4c/D (capped at the whole pipe; 0 with no gap or no pipe).
pub fn valve_leak_share(clearance_m: f64, bore_m: f64) -> f64 {
    if bore_m <= 0.0 || clearance_m <= 0.0 {
        return 0.0;
    }
    (4.0 * clearance_m / bore_m).min(1.0)
}

/// Bore a path ends in, m (a silencer's tube if it has one, else its shell); 0 for none.
fn exit_diameter(path: &[Segment]) -> f64 {
    match path.last() {
        Some(Segment::Pipe { diameter_m, .. } | Segment::Catalyst { diameter_m, .. }) => {
            *diameter_m
        }
        Some(Segment::Measured { bore_m, .. }) => *bore_m,
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

pub(crate) fn clamp_wave(p: f64) -> f64 {
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
    /// Extra length of each collector's line over `collector_length_m`, m, in collector order
    /// (missing: none). Two banks are never exactly alike: their pulses meet a little apart, and
    /// a V engine's half orders come from that.
    pub collector_extra_length_m: Vec<f64>,
    /// Each primary's length off `primary_length_m` as built, m, in cylinder order (missing:
    /// none). Equal-length headers are equal only to their fabrication: pulses that reach the
    /// collector a few tens of microseconds apart put energy in the cycle orders between the
    /// firing harmonics and take it off their highs.
    pub primary_length_deviation_m: Vec<f64>,
    pub tail_length_m: f64,
    pub tail_diameter_m: f64,
    /// Open ends the tail's flow leaves through side by side, each the tail's bore (1 = the pipe
    /// itself). The pipe's waves and its radiated pulses stay the pipe's; the jet blows at the
    /// flow over their summed area.
    pub tail_outlets: u32,
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
    /// The waveguide's smooth pipes steepen their waves into shocks and their walls take the
    /// highs (Msallam et al., *Acta Acustica* 86, 2000; Kirchhoff, Kinsler §9.9): see
    /// [`Pipe::steepening`]. Off: linear pipes (cheaper; what lower quality levels run).
    pub steepening: bool,
    /// Each pipe's gas takes its own temperature, so its own speed of sound: the port's outflow
    /// cooled along the system by its walls, which warm and cool over seconds
    /// ([`crate::pipe_heat`]). Off: the fixed speeds [`C_PRIMARY`], [`C_COLLECTOR`], [`C_TAIL`].
    pub gas_temperature: bool,
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
    /// Radial clearance round the valve's disc, m: a butterfly keeps a gap for its thermal
    /// growth, so shut it still passes a jet through [`valve_leak_share`] of its pipe (0:
    /// sealed). Default 0.5 mm: ESTIMATE, a hot exhaust butterfly's running clearance.
    pub valve_clearance_m: f64,
    /// The silencer's outer wall over its inlet chamber (the first of `closed`): its thickness,
    /// m (0: no wall radiates), density, kg/m³, and area, m². The gas in the chamber shakes the
    /// wall, which radiates as well as the tips ("break-out" noise, Munjal, *Acoustics of Ducts
    /// and Mufflers*, 2nd ed., §9.9): with the valve shut and the chambers taking 60 dB the
    /// wall is what passes the mid range ([`Shell`]).
    pub shell_thickness_m: f64,
    pub shell_density_kg_m3: f64,
    pub shell_area_m2: f64,
    /// The wall's first resonance, Hz (0: a limp wall, mass law only). Below it the wall moves
    /// by its stiffness and passes less than the mass law would (Kinsler et al., §6.4).
    pub shell_resonance_hz: f64,
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
            valve_clearance_m: BUTTERFLY_CLEARANCE_M,
            shell_thickness_m: 0.0,
            shell_density_kg_m3: 7850.0,
            shell_area_m2: 0.0,
            shell_resonance_hz: 0.0,
        }
    }
}

/// A silencer's outer wall as a limp mass driven by the gas inside and loaded by the gas on
/// both sides: m·dv/dt = p − (Z₁ + Z₂)·v (Kinsler et al., *Fundamentals of Acoustics*, 4th ed.,
/// §6.4: a partition of surface mass m between media of impedances Z₁, Z₂; its transmission
/// falls 6 dB per octave above (Z₁ + Z₂)/(2πm), the mass law), held by a stiffness k that puts
/// its first resonance at f₀ = √(k/m)/2π: m·dv/dt = p − k·x − (Z₁ + Z₂ + c)·v, with the
/// structure's own damping c = η·√(k·m) at a loss factor η of [`SHELL_LOSS_FACTOR`]. Higher
/// modes are left out.
#[derive(Debug, Clone)]
pub struct Shell {
    /// Surface mass, kg/m² (0: no wall).
    mass: f64,
    area: f64,
    /// Z₁ + Z₂: the gas inside and the air outside, Pa·s/m.
    z: f64,
    /// Stiffness per unit area, Pa/m (0: limp).
    k: f64,
    v: f64,
    x: f64,
}

/// Structural loss factor of a seamed silencer shell (ESTIMATE: welded steel structures run
/// 0.01–0.1, Beranek & Vér, *Noise and Vibration Control Engineering*, ch. 14; seams and the
/// packing against the wall put a silencer at the top of the range).
const SHELL_LOSS_FACTOR: f64 = 0.1;

/// ρc of the air outside, Pa·s/m (1.204 kg/m³ · 343 m/s, 20 °C).
const Z_AIR: f64 = 1.204 * 343.0;

impl Shell {
    /// A wall `thickness_m` thick of `density` kg/m³ over `area_m2`, its first resonance at
    /// `resonance_hz` (0: limp), the gas inside of impedance `z_gas`.
    pub fn new(
        thickness_m: f64,
        density: f64,
        area_m2: f64,
        resonance_hz: f64,
        z_gas: f64,
    ) -> Self {
        let mass = thickness_m * density;
        Self {
            mass,
            area: area_m2,
            z: z_gas + Z_AIR,
            k: (2.0 * std::f64::consts::PI * resonance_hz).powi(2) * mass,
            v: 0.0,
            x: 0.0,
        }
    }

    /// One step driven by the pressure `p` inside (Pa): the volume velocity the wall pushes into
    /// the air outside, m³/s. Semi-implicit, so a light wall cannot overshoot.
    pub fn step(&mut self, p: f64, dt: f64) -> f64 {
        if self.mass <= 0.0 || self.area <= 0.0 {
            return 0.0;
        }
        let (m, k) = (self.mass, self.k);
        let c = self.z + SHELL_LOSS_FACTOR * (k * m).sqrt();
        // semi-implicit: v from the force with x advanced by v
        self.v = (self.v + dt / m * (p - k * self.x)) / (1.0 + dt * c / m + dt * dt * k / m);
        self.x += self.v * dt;
        self.v * self.area
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
        // shut, the disc still leaves its clearance open
        let opening = self.x.max(self.leak);
        let shut = opening <= 0.0;
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
            let loss = valve_loss(opening, rho);
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
        // the inlet chamber's wall, its volume velocity as the outlets' u·ρc over their area
        let out_area: f64 = if self.outlets.is_empty() {
            self.tips.end_areas().1
        } else {
            self.outlets.iter().map(|(d, _)| d.end_areas().1).sum()
        };
        let chamber = split[1] + self.closed.at_near();
        let out = out + self.shell.step(chamber, dt) * z / out_area;
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
    /// A silencer element whose maker published its transmission loss, `transmission_loss_db`
    /// as (frequency Hz, dB) points in rising frequency, carried on a straight bore of `bore_m`
    /// × `length_m` ([`crate::silencer2d::measured_eq`]: linear between points in log frequency,
    /// falling to 0 at 0 Hz below the first, held above the last). For a chamber whose shape
    /// makes its loss flat across the orders: the LFA's spherical final chamber (Torimoto &
    /// Mabuchi, JP 2008-169751 A, Fig. 6: ≈ 15 dB at 100 Hz, 20 dB from 200 Hz to 1 kHz, where
    /// the flat chamber it is compared with swings 10–33 dB), which no 1D chamber gives: an
    /// expansion chamber in 1D ripples with its axial standing waves, 0 dB at every half-wave.
    Measured {
        length_m: f64,
        bore_m: f64,
        transmission_loss_db: Vec<[f64; 2]>,
    },
}

/// A road car's exhaust after one collector, from sizing rules rather than one car's drawings
/// (every road preset without a drawn silencer carries it, `EngineDef::with_road_exhaust`):
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
            collector_extra_length_m: Vec::new(),
            primary_length_deviation_m: Vec::new(),
            tail_length_m: 1.5,
            tail_diameter_m: 0.050,
            tail_outlets: 1,
            muffler_transmission: 1.0,
            model: ExhaustModel::Waveguide,
            muffler_volume_m3: 0.006,
            muffler_packing: crate::lining::Packing::default(),
            tail_spacing_m: 0.9,
            after_collector: Vec::new(),
            merge: None,
            steepening: true,
            gas_temperature: false,
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
                Segment::Measured {
                    length_m, bore_m, ..
                } => friction += f * length_m / (bore_m * area(bore_m).powi(2)),
            }
        }
        ExitLosses {
            outlet_area: area(self.outlet_diameter()) * f64::from(self.tail_outlets.max(1)),
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
            Some(Segment::Measured { bore_m, .. }) => *bore_m,
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

    /// Bore of the one jet with the area of all the outlets, m ([`Self::tail_outlets`]).
    pub fn jet_diameter(&self) -> f64 {
        self.radiating_diameter() * f64::from(self.tail_outlets.max(1)).sqrt()
    }

    /// Length of cylinder `i`'s (0-based) primary, m: `primary_length_m` and its deviation,
    /// if any.
    pub fn primary_length(&self, i: usize) -> f64 {
        self.primary_length_m
            + self
                .primary_length_deviation_m
                .get(i)
                .copied()
                .unwrap_or(0.0)
    }

    /// Length of collector `branch`'s line, m: `collector_length_m` and its extra, if any.
    pub fn collector_length(&self, branch: usize) -> f64 {
        self.collector_length_m
            + self
                .collector_extra_length_m
                .get(branch)
                .copied()
                .unwrap_or(0.0)
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
    /// A turbine between the collector and the tail, once the engine has said so
    /// ([`ExhaustNet::set_turbine_resistance`]).
    turbine: Option<crate::waveguide::TurbineJunction>,
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
pub(crate) fn after_collector(
    def: &ExhaustDef,
    dt: f64,
    lining_m: f64,
    memo: &mut crate::silencer2d::EqMemo,
) -> Duct {
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
        def.steepening,
        memo,
    )
}

/// Area ratio from which a step out into a wider passage is an abrupt expansion that sheds the
/// flow (a sudden-expansion loss (1 − σ)² ≥ 0.11 at σ = 1/1.5; below it the jet stays mostly
/// attached and its noise is left to the pipes).
pub const EXPANSION_RATIO_MIN: f64 = 1.5;

/// Where along the exhaust an [`ExpansionSite`] is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SiteAt {
    /// A primary (0-based cylinder) jetting into its collector.
    Collector { primary: usize },
    /// The step into segment `index` of the collector's path ([`ExhaustDef::path`]).
    Segment { index: usize },
    /// A branch's path jetting into the shared duct of a [`Merge`].
    MergeJunction,
    /// The step into segment `index` of the merge's shared duct.
    MergedSegment { index: usize },
    /// The open end: the branch's tail (or the merge's outlet) jetting into the air.
    Exit,
}

/// An abrupt area expansion of the exhaust, where the flow separates and sheds vortices: a
/// source of duct flow noise ([`crate::jet_noise::DuctJet`]; Davies & Holland 1999 §4). `branch`
/// is the collector it belongs to (None: after a merge); `d_jet` the passage the flow leaves,
/// `d_down` the one it enters (for the exit: the tail itself, which carries its waves out).
#[derive(Debug, Clone, PartialEq)]
pub struct ExpansionSite {
    pub branch: Option<usize>,
    pub at: SiteAt,
    pub d_jet: f64,
    pub d_down: f64,
}

/// The bore the flow passes through in `seg`: a straight-through silencer's tube, an empty can's
/// shell, a catalyst's face. None for a side resonator (off the flow).
fn passage_diameter(seg: &Segment) -> Option<f64> {
    match *seg {
        Segment::Pipe { diameter_m, .. } | Segment::Catalyst { diameter_m, .. } => Some(diameter_m),
        Segment::Measured { bore_m, .. } => Some(bore_m),
        Segment::Muffler {
            diameter_m,
            tube_diameter_m,
            ..
        } => Some(if tube_diameter_m > 0.0 && tube_diameter_m < diameter_m {
            tube_diameter_m
        } else {
            diameter_m
        }),
        Segment::Resonator { .. } => None,
    }
}

/// The expansions along `segments` entered from a passage of `d_from`, pushed as `at(index)`;
/// returns the last passage's diameter.
fn path_sites(
    segments: &[Segment],
    mut d_from: f64,
    branch: Option<usize>,
    at: impl Fn(usize) -> SiteAt,
    out: &mut Vec<ExpansionSite>,
) -> f64 {
    for (index, seg) in segments.iter().enumerate() {
        let Some(d) = passage_diameter(seg) else {
            continue;
        };
        if d * d >= EXPANSION_RATIO_MIN * d_from * d_from {
            out.push(ExpansionSite {
                branch,
                at: at(index),
                d_jet: d_from,
                d_down: d,
            });
        }
        d_from = d;
    }
    d_from
}

/// Every abrupt area expansion of `def`'s exhaust for an engine of `cylinders`, in a fixed
/// order both exhaust models share: for each collector (in `def.collectors` order, or one for
/// all), its primaries' jets into it, the expansions along its path, and its exit (none when a
/// [`Merge`] takes the paths); then each branch's step into the merge, the merge's own
/// expansions and its outlet. The bypass silencer's inner chambers are not listed.
pub fn expansion_sites(def: &ExhaustDef, cylinders: usize) -> Vec<ExpansionSite> {
    let groups = def.groups(cylinders);
    let path = def.path();
    let mut out = Vec::new();
    let mut branch_exit = Vec::with_capacity(groups.len());
    let (dp, dc) = (def.primary_diameter_m, def.collector_diameter_m);
    for (b, group) in groups.iter().enumerate() {
        if dc * dc >= EXPANSION_RATIO_MIN * dp * dp {
            for &primary in group {
                out.push(ExpansionSite {
                    branch: Some(b),
                    at: SiteAt::Collector { primary },
                    d_jet: dp,
                    d_down: dc,
                });
            }
        }
        let d_end = path_sites(
            &path,
            dc,
            Some(b),
            |index| SiteAt::Segment { index },
            &mut out,
        );
        branch_exit.push(d_end);
        if def.merge.is_none() {
            out.push(ExpansionSite {
                branch: Some(b),
                at: SiteAt::Exit,
                d_jet: d_end,
                d_down: d_end,
            });
        }
    }
    if let Some(m) = &def.merge {
        let d_shared = m
            .segments
            .iter()
            .find_map(passage_diameter)
            .unwrap_or(m.outlet_diameter_m);
        for (b, &d_end) in branch_exit.iter().enumerate() {
            if d_shared * d_shared >= EXPANSION_RATIO_MIN * d_end * d_end {
                out.push(ExpansionSite {
                    branch: Some(b),
                    at: SiteAt::MergeJunction,
                    d_jet: d_end,
                    d_down: d_shared,
                });
            }
        }
        // the first shared segment is the junction's own duct: its expansions start after it
        let rest = m.segments.get(1..).unwrap_or(&[]);
        path_sites(
            rest,
            d_shared,
            None,
            |i| SiteAt::MergedSegment { index: i + 1 },
            &mut out,
        );
        out.push(ExpansionSite {
            branch: None,
            at: SiteAt::Exit,
            d_jet: m.outlet_diameter_m,
            d_down: m.outlet_diameter_m,
        });
    }
    out
}

/// The duct of `path`'s segments, its last pipe carrying `transmission`; a lone pipe of
/// `fallback` (length, diameter) if the path is empty. The lining's segment length no longer
/// applies here: a packed silencer is its 2D transmission ([`crate::silencer2d`]), not a lumped
/// line.
/// `pipe` steepening ([`Pipe::steepening`]) when `steep`, else as it is.
fn shaped(pipe: Pipe, steep: bool) -> Pipe {
    if steep { pipe.steepening() } else { pipe }
}

fn segments_duct(
    path: &[Segment],
    transmission: f64,
    fallback: (f64, f64),
    dt: f64,
    _lining_m: f64,
    steep: bool,
    memo: &mut crate::silencer2d::EqMemo,
) -> Duct {
    let rho_c = crate::gas::GAMMA * crate::units::ATM_PA / C_TAIL;
    let plain = |len: f64, d: f64| Pipe::new(len, d, C_TAIL, dt, LOSS_PER_M);
    // a smooth-walled pipe: its waves steepen and its walls take the highs (Kirchhoff)
    let pipe = |len: f64, d: f64| shaped(plain(len, d), steep);
    let mut duct: Option<Duct> = None;
    let mut pending = Node::Plain;
    let mut starts = Vec::with_capacity(path.len());
    let last_pipe = path
        .iter()
        .rposition(|s| !matches!(s, Segment::Resonator { .. }));
    for (i, seg) in path.iter().enumerate() {
        starts.push(duct.as_ref().map_or(0, Duct::pipe_count));
        let piece =
            match *seg {
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
                        // the perforated tube is no smooth wall: linear, its loss the packing's
                        Duct::new(vec![
                            plain(length_m, tube_diameter_m)
                                .with_filter(silencer.transmission_eq_in(gas, 1.0 / dt, memo)),
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
                Segment::Measured {
                    length_m,
                    bore_m,
                    ref transmission_loss_db,
                } => {
                    // the published loss on the waves both ways, like a packed silencer's
                    Duct::new(vec![plain(length_m, bore_m).with_filter(
                        crate::silencer2d::measured_eq(transmission_loss_db, 1.0 / dt),
                    )])
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
    let mut duct = duct.unwrap_or_else(|| Duct::new(vec![pipe(fallback.0, fallback.1)]));
    duct.segment_starts = starts;
    duct
}

/// A pipe's gas as the 3D view shows it ([`ExhaustNet::probe`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeState {
    /// The gas's mean temperature, K: from the pipe's speed of sound, T = c²/(γR).
    pub temperature_k: f64,
    /// The gas's mean speed along the pipe, m/s, + out towards the tail: the mean mass flow over
    /// ρA at atmospheric pressure and that temperature.
    pub velocity_m_s: f64,
    /// The wall's temperature, K, when the gas temperature model runs ([`crate::pipe_heat`]).
    pub wall_k: Option<f64>,
}

impl PipeState {
    fn of(pipe: &Pipe, mass_flow: f64, wall_k: Option<f64>) -> Self {
        let c = pipe.gas_sound_speed();
        let t = c * c / (crate::gas::GAMMA * crate::gas::R_AIR);
        let (_, d) = pipe.size();
        let rho = crate::units::ATM_PA / (crate::gas::R_AIR * t);
        let area = std::f64::consts::PI * 0.25 * d * d;
        Self {
            temperature_k: t,
            velocity_m_s: if rho * area > 0.0 {
                mass_flow / (rho * area)
            } else {
                0.0
            },
            wall_k,
        }
    }
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
    /// The gas's temperature along the pipes ([`ExhaustDef::gas_temperature`]).
    heat: Option<Heat>,
    /// Flow noise at the exhaust's area expansions ([`ExhaustNet::set_duct_flow`]).
    duct_flow: DuctFlow,
    pub dt: f64,
}

/// The exhaust's duct flow noise: one [`crate::jet_noise::DuctJet`] per [`ExpansionSite`], in
/// [`expansion_sites`] order, at a mix `gain` (0: off, nothing computed).
#[derive(Debug, Clone, Default)]
struct DuctFlow {
    gain: f64,
    sites: Vec<SiteSource>,
    /// Each site's mass flow handed in from outside ([`ExhaustNet::set_site_flows`]), kg/s;
    /// empty: the net's own.
    given: Vec<f64>,
}

#[derive(Debug, Clone)]
struct SiteSource {
    site: ExpansionSite,
    jet: crate::jet_noise::DuctJet,
}

impl DuctFlow {
    fn new(def: &ExhaustDef, cylinders: usize) -> Self {
        let sites = expansion_sites(def, cylinders)
            .into_iter()
            .enumerate()
            .map(|(i, site)| SiteSource {
                site,
                jet: crate::jet_noise::DuctJet::new(0xD0C7_F10E ^ (i as u64 + 1)),
            })
            .collect();
        Self {
            gain: 0.0,
            sites,
            given: Vec::new(),
        }
    }

    /// Site `i`'s noise this step as the plane-wave pressure for a 1 m² duct (scale by 1/√A),
    /// its mass flow `own` (kg/s, + towards the tail) unless one was given, in gas of sound
    /// speed `c` at an atmosphere.
    fn pressure(&mut self, i: usize, own: f64, c: f64, dt: f64) -> f64 {
        let m_dot = self.given.get(i).copied().unwrap_or(own);
        let src = &mut self.sites[i];
        let rho = crate::gas::GAMMA * crate::units::ATM_PA / (c * c);
        let a_jet = 0.25 * std::f64::consts::PI * src.site.d_jet * src.site.d_jet;
        let u = m_dot / (rho * a_jet);
        self.gain * src.jet.step(u, rho, c, src.site.d_jet, src.site.d_down, dt)
    }
}

/// The pipes' walls and their gas ([`crate::pipe_heat`]): one state per pipe, in the order
/// [`ExhaustNet::update_gas`] walks them (the gas's), made on its first walk.
#[derive(Debug, Clone, Default)]
struct Heat {
    walls: Vec<crate::pipe_heat::PipeHeat>,
    /// Each cylinder's port outflow temperature, K ([`ExhaustNet::set_port_temperature`]).
    port_k: Vec<f64>,
    /// Steps since the walls last moved.
    steps: usize,
    /// The next wall in the walk, and the time it covers, s.
    next: usize,
    elapsed: f64,
    /// Scratch, reused (no allocation on the sim thread): each primary's and each branch's
    /// outlet gas (temperature, mass flow).
    outs: Vec<f64>,
    tails: Vec<(f64, f64)>,
}

impl Heat {
    /// Gas at `t_in` K and `mass_flow` kg/s through `pipe`: its wall moves over the walk's
    /// time, the pipe's waves take the gas's mean speed of sound. Returns the outlet temperature.
    fn pass(&mut self, pipe: &mut Pipe, t_in: f64, mass_flow: f64) -> f64 {
        if self.next == self.walls.len() {
            let (length, diameter) = pipe.size();
            self.walls
                .push(crate::pipe_heat::PipeHeat::new(length, diameter));
        }
        let wall = &mut self.walls[self.next];
        self.next += 1;
        let (mean, out) = wall.step(t_in, mass_flow, self.elapsed);
        // a path no gas runs through (a shut bypass) holds the gas of the chamber it opens
        // from, which the shell around it keeps warm (an estimate: the lumped wall alone would
        // cool it to the air)
        let (mean, out) = if mass_flow.abs() < 1e-9 {
            (t_in, t_in)
        } else {
            (mean, out)
        };
        pipe.set_sound_speed(crate::pipe_heat::sound_speed(mean));
        out
    }
    /// A duct's pipes in turn; returns the outlet temperature.
    fn duct(&mut self, duct: &mut Duct, t_in: f64, mass_flow: f64) -> f64 {
        duct.pipes_mut()
            .iter_mut()
            .fold(t_in, |t, p| self.pass(p, t, mass_flow))
    }
}

/// Mass-weighted mean temperature of gas streams (temperature K, mass flow kg/s): where they
/// mix, enthalpy adds up at one c_p. With no flow, their plain mean.
fn mixed(streams: impl Iterator<Item = (f64, f64)>) -> f64 {
    let (tm, m, t, n) = streams.fold(
        (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64),
        |(tm, m, t, n), (ti, mi)| (tm + ti * mi.abs(), m + mi.abs(), t + ti, n + 1.0),
    );
    if m > 1e-12 { tm / m } else { t / n.max(1.0) }
}

/// The slowest gas the pipes make room for: air at [`crate::pipe_heat::AMBIENT_K`].
fn coldest_sound_speed() -> f64 {
    crate::pipe_heat::sound_speed(crate::pipe_heat::AMBIENT_K)
}

/// The branches' common duct and open end, and each branch's last tail wave (scratch).
#[derive(Debug, Clone)]
pub(crate) struct Shared {
    duct: Duct,
    end: OpenEnd,
    /// Each branch tail's near-end wave this step, sent once the junction is solved.
    near: Vec<f64>,
    /// The tails' far-end area over the outlet's: a tail's Mach to the outlet's.
    tail_to_outlet: f64,
    valved: Option<Valved>,
}

impl Shared {
    /// The duct the branches' tails join and its open end, from [`ExhaustDef::merge`] (none
    /// without one), for `n_branches` tails.
    pub(crate) fn new(
        def: &ExhaustDef,
        n_branches: usize,
        dt: f64,
        lining_m: f64,
        memo: &mut crate::silencer2d::EqMemo,
    ) -> Option<Self> {
        def.merge.as_ref().map(|m| {
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
                def.steepening,
                memo,
            );
            let valved = m.bypass.as_ref().map(|b| {
                let mut duct = |path: &[Segment], t: f64| {
                    segments_duct(path, t, fallback, dt, lining_m, def.steepening, memo)
                };
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
                    leak: valve_leak_share(b.valve_clearance_m, exit_diameter(&b.open)),
                    x: 0.0,
                    opening: false,
                    shell: Shell::new(
                        b.shell_thickness_m,
                        b.shell_density_kg_m3,
                        b.shell_area_m2,
                        b.shell_resonance_hz,
                        tail_reference_density() * C_TAIL,
                    ),
                }
            });
            let tail_area = std::f64::consts::PI * 0.25 * def.tail_diameter_m.powi(2);
            let outlet_area = std::f64::consts::PI * 0.25 * def.radiating_diameter().powi(2);
            Shared {
                duct,
                end: OpenEnd::new(m.outlet_diameter_m, dt),
                near: vec![0.0; n_branches],
                tail_to_outlet: tail_area / outlet_area,
                valved,
            }
        })
    }

    /// The tails' mean outflow Mach numbers (`mach`, one per tail): their volume flows add up
    /// through the outlet ([`OpenEnd::set_outflow_mach`]).
    pub(crate) fn set_outflow_mach(&mut self, mach: &[f64]) {
        let m = mach.iter().sum::<f64>() * self.tail_to_outlet;
        self.end.set_outflow_mach(m);
        if let Some(v) = &mut self.valved {
            v.set_outflow_mach(m);
        }
    }

    /// The engine speed, rpm, that a bypass valve follows (it opens above its `open_rpm`).
    pub(crate) fn set_engine_speed(&mut self, rpm: f64) {
        if let Some(v) = &mut self.valved {
            v.opening = rpm > v.open_rpm;
        }
    }

    /// A bypass valve's opening fraction (0 shut … 1 open), if there is one.
    pub(crate) fn bypass_opening(&self) -> Option<f64> {
        self.valved.as_ref().map(|v| v.x)
    }

    /// The pressure junction of the tails' far ends (`tails`: each one's far-end area and
    /// arriving wave) with this duct's near end: fills `areas`, `incoming` and `outgoing` (the
    /// tails first, this duct last).
    pub(crate) fn solve_merge(
        &self,
        tails: impl Iterator<Item = (f64, f64)>,
        areas: &mut Vec<f64>,
        incoming: &mut Vec<f64>,
        outgoing: &mut Vec<f64>,
    ) {
        areas.clear();
        incoming.clear();
        for (a, w) in tails {
            areas.push(a);
            incoming.push(w);
        }
        areas.push(self.duct.end_areas().0);
        incoming.push(self.duct.at_near());
        outgoing.resize(incoming.len(), 0.0);
        junction(areas, incoming, outgoing);
    }

    /// This duct's far end through its open end or its valved silencer, with `exit` (a source
    /// at the outlet) added both ways, and `near` sent into its near end: what it radiates.
    pub(crate) fn radiate(&mut self, near: f64, exit: f64, dt: f64) -> f64 {
        let arriving = self.duct.at_far() + exit;
        let (reflected, out) = match &mut self.valved {
            None => self.end.process(arriving),
            Some(v) => v.step(&mut self.end, arriving, self.duct.end_areas().1, dt),
        };
        self.duct
            .send(clamp_wave(near), clamp_wave(reflected + exit));
        out
    }
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
    /// [`Bypass::valve_clearance_m`].
    leak: f64,
    /// Opening fraction, 0 shut … 1 open.
    x: f64,
    /// Whether the engine is above `open_rpm` (the valve's target).
    opening: bool,
    /// [`Bypass::shell_thickness_m`]: the inlet chamber's wall.
    shell: Shell,
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

/// The means an [`ExhaustNet`] follows, handed every few dozen steps ([`ExhaustNet::prepare`]):
/// mass flows, kg/s, per cylinder and per branch; the engine speed, rpm; the gas's mean pressure
/// before the collectors' end, Pa ([`ExhaustNet::set_mean_pressure`]); and a turbine's slope
/// there, Pa per kg/s (0: none, [`ExhaustNet::set_turbine_resistance`]).
#[derive(Debug, Clone, Copy)]
pub struct NetMeans<'a> {
    pub cylinders: &'a [f64],
    pub branches: &'a [f64],
    pub rpm: f64,
    pub upstream_pa: f64,
    pub turbine_dp_dm: f64,
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

    /// The primaries' gas is at `p` Pa from now on (as in [`ExhaustNet::set_mean_pressure`]).
    pub fn set_mean_pressure(&mut self, p: f64) {
        for pipe in &mut self.pipes {
            pipe.set_mean_pressure(p);
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

    /// One step taken: the primaries' friction follows its mean flow as the net's does
    /// ([`Pipe::follow_gain`]); once per step, where the net sends.
    pub fn advance(&mut self) {
        for p in &mut self.pipes {
            p.follow_gain();
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
        Self::with_memo(
            def,
            cylinders,
            dt,
            lining_m,
            &mut crate::silencer2d::EqMemo::default(),
        )
    }

    /// [`ExhaustNet::with_lining`], its silencers' equalizers taken from `memo` when designed
    /// before (the same, bit for bit) and kept there.
    pub fn with_memo(
        def: &ExhaustDef,
        cylinders: usize,
        dt: f64,
        lining_m: f64,
        memo: &mut crate::silencer2d::EqMemo,
    ) -> Self {
        let primaries = (0..cylinders)
            .map(|i| {
                Pipe::new(
                    def.primary_length(i),
                    def.primary_diameter_m,
                    C_PRIMARY,
                    dt,
                    LOSS_PER_M,
                )
            })
            .map(|p| shaped(p, def.steepening))
            .collect();
        let branches: Vec<Branch> = def
            .groups(cylinders)
            .into_iter()
            .enumerate()
            .map(|(bi, members)| Branch {
                losses: collector_losses(def, members.len()),
                members,
                collector: shaped(
                    Pipe::new(
                        def.collector_length(bi),
                        def.collector_diameter_m,
                        C_COLLECTOR,
                        dt,
                        LOSS_PER_M,
                    ),
                    def.steepening,
                ),
                tail: after_collector(def, dt, lining_m, memo),
                end: OpenEnd::new(def.tail_diameter_m, dt),
                turbine: None,
            })
            .collect();
        Self {
            primaries,
            sent: vec![0.0; cylinders],
            tails: vec![0.0; def.groups(cylinders).len()],
            mean_flows: (Vec::new(), Vec::new()),
            scratch: Scratch::default(),
            shared: Shared::new(def, branches.len(), dt, lining_m, memo),
            branches,
            heat: def.gas_temperature.then(Heat::default),
            duct_flow: DuctFlow::new(def, cylinders),
            dt,
        }
        .with_room_for_cold_gas()
    }

    /// With [`ExhaustDef::gas_temperature`], room in every pipe for gas down to the air's
    /// temperature.
    fn with_room_for_cold_gas(mut self) -> Self {
        if self.heat.is_some() {
            let c = coldest_sound_speed();
            self.for_each_pipe(|p| p.make_room(c));
        }
        self
    }

    /// Every pipe of the net.
    fn for_each_pipe(&mut self, mut f: impl FnMut(&mut Pipe)) {
        self.primaries.iter_mut().for_each(&mut f);
        for b in &mut self.branches {
            f(&mut b.collector);
            b.tail.pipes_mut().iter_mut().for_each(&mut f);
        }
        if let Some(sh) = &mut self.shared {
            sh.duct.pipes_mut().iter_mut().for_each(&mut f);
            if let Some(v) = &mut sh.valved {
                for d in [&mut v.closed, &mut v.open, &mut v.tips] {
                    d.pipes_mut().iter_mut().for_each(&mut f);
                }
                for (d, _) in &mut v.outlets {
                    d.pipes_mut().iter_mut().for_each(&mut f);
                }
            }
        }
    }

    /// The speed of sound each pipe's gas carries now, m/s, near end first: the primaries,
    /// each branch's collector and tail, then the shared duct and a bypass's paths and tips.
    pub fn gas_sound_speeds(&mut self) -> Vec<f64> {
        let mut v = Vec::new();
        self.for_each_pipe(|p| v.push(p.gas_sound_speed()));
        v
    }

    /// The port temperatures last handed ([`ExhaustNet::set_port_temperature`]), K.
    pub fn port_temperatures(&self) -> &[f64] {
        self.heat.as_ref().map_or(&[], |h| &h.port_k)
    }

    /// Each cylinder's port outflow temperature, K: the gas entering its primary
    /// ([`ExhaustDef::gas_temperature`]; ignored without it).
    pub fn set_port_temperature(&mut self, per_cylinder: &[f64]) {
        if let Some(h) = &mut self.heat {
            h.port_k.clear();
            h.port_k.extend_from_slice(per_cylinder);
        }
    }

    /// The gas through every pipe in the order it flows, at the last mean flows: each wall
    /// moves over the steps since the last walk, each pipe takes its gas's speed of sound.
    fn update_gas(&mut self) {
        let Some(heat) = &mut self.heat else {
            return;
        };
        heat.elapsed = heat.steps as f64 * self.dt;
        heat.steps = 0;
        heat.next = 0;
        let (per_cyl, per_branch) = (&self.mean_flows.0, &self.mean_flows.1);
        let flow = |v: &[f64], i: usize| v.get(i).copied().unwrap_or(0.0);
        // before the engine hands a port temperature: the primaries' fixed speed's gas
        let fallback = C_PRIMARY * C_PRIMARY / (crate::gas::GAMMA * crate::gas::R_AIR);
        let mut outs = std::mem::take(&mut heat.outs);
        let mut tails = std::mem::take(&mut heat.tails);
        outs.clear();
        tails.clear();
        for (i, p) in self.primaries.iter_mut().enumerate() {
            let t = heat.port_k.get(i).copied().unwrap_or(fallback);
            outs.push(heat.pass(p, t, flow(per_cyl, i)));
        }
        for (bi, b) in self.branches.iter_mut().enumerate() {
            let m = flow(per_branch, bi);
            let t = mixed(b.members.iter().map(|&i| (outs[i], flow(per_cyl, i))));
            let t = heat.pass(&mut b.collector, t, m);
            tails.push((heat.duct(&mut b.tail, t, m), m));
        }
        if let Some(sh) = &mut self.shared {
            let m: f64 = per_branch.iter().sum();
            let t = heat.duct(&mut sh.duct, mixed(tails.iter().copied()), m);
            if let Some(v) = &mut sh.valved {
                let (mo, mc) = (m * v.x, m * (1.0 - v.x));
                let to = heat.duct(&mut v.open, t, mo);
                let tc = heat.duct(&mut v.closed, t, mc);
                let t = heat.duct(&mut v.tips, mixed([(to, mo), (tc, mc)].into_iter()), m);
                // the tips share the flow evenly (an estimate: they are about alike)
                let mt = m / v.outlets.len().max(1) as f64;
                for (d, _) in &mut v.outlets {
                    heat.duct(d, t, mt);
                }
            }
        }
        heat.outs = outs;
        heat.tails = tails;
    }

    /// The mean pressure, Pa, of the gas in the primaries and the collectors: a turbine's inlet
    /// pressure when a turbine follows the collectors, the atmosphere without one (what follows
    /// the collectors stays at the atmosphere's). See [`Pipe::set_mean_pressure`].
    pub fn set_mean_pressure(&mut self, p: f64) {
        for pipe in &mut self.primaries {
            pipe.set_mean_pressure(p);
        }
        for b in &mut self.branches {
            b.collector.set_mean_pressure(p);
        }
    }

    /// A turbine follows each collector, its inlet's pressure rising with the flow at `dp_dm` Pa
    /// per kg/s ([`crate::turbo::TurboState::turbine_dp_dm`]): the collector meets the tail
    /// through it ([`crate::waveguide::TurbineJunction`]). None until a positive slope is given (a
    /// net without a turbo is left as it was).
    pub fn set_turbine_resistance(&mut self, dp_dm: f64) {
        let dt = self.dt;
        for b in &mut self.branches {
            match &mut b.turbine {
                Some(t) => t.set_resistance(dp_dm),
                None if dp_dm > 0.0 => {
                    let mut t = crate::waveguide::TurbineJunction::new(dt);
                    t.set_resistance(dp_dm);
                    b.turbine = Some(t);
                }
                None => {}
            }
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
        self.update_gas();
    }

    /// The mean flows the pipes' friction last followed: (per cylinder, per branch), kg/s.
    /// Each primary's gas (by cylinder), then each branch's collector's, into `out` (cleared;
    /// no allocation once it has room). Read-only: for the UI, at its rate, never per step.
    pub fn probe(&self, out: &mut Vec<PipeState>) {
        out.clear();
        let (per_cyl, per_branch) = (&self.mean_flows.0, &self.mean_flows.1);
        let flow = |v: &[f64], i: usize| v.get(i).copied().unwrap_or(0.0);
        // the walls are walked primaries first (crate::pipe_heat), so the first n are theirs
        let walls = self.heat.as_ref().map_or(&[][..], |h| &h.walls[..]);
        for (i, p) in self.primaries.iter().enumerate() {
            out.push(PipeState::of(
                p,
                flow(per_cyl, i),
                walls.get(i).map(|w| w.wall_k),
            ));
        }
        for (k, b) in self.branches.iter().enumerate() {
            out.push(PipeState::of(&b.collector, flow(per_branch, k), None));
        }
    }

    pub fn mean_flows(&self) -> (&[f64], &[f64]) {
        (&self.mean_flows.0, &self.mean_flows.1)
    }

    /// Mean outflow Mach number at each tail's open end (see [`OpenEnd::set_outflow_mach`]).
    pub fn set_outflow_mach(&mut self, mach: &[f64]) {
        if let Some(sh) = &mut self.shared {
            sh.set_outflow_mach(mach);
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
    pub fn prepare(&mut self, mach: &[f64], mean: Option<NetMeans>) {
        self.set_outflow_mach(mach);
        if let Some(m) = mean {
            self.set_mean_pressure(m.upstream_pa);
            self.set_turbine_resistance(m.turbine_dp_dm);
            self.set_mean_flow(m.cylinders, m.branches);
            self.set_engine_speed(m.rpm);
        }
    }

    /// The engine speed, rpm, that a bypass valve follows (it opens above its `open_rpm`).
    pub fn set_engine_speed(&mut self, rpm: f64) {
        if let Some(sh) = &mut self.shared {
            sh.set_engine_speed(rpm);
        }
    }

    /// A bypass valve's opening fraction (0 shut … 1 open), if the net has one.
    pub fn bypass_opening(&self) -> Option<f64> {
        self.shared.as_ref()?.bypass_opening()
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
        // a steepening primary's waves arrive when their own speed says, through its segments:
        // the copy the probe keeps of the lines cannot run ahead of the net; nor can it follow
        // the gas's temperature, which the walls' state sets
        if self.heat.is_some() || self.primaries.iter().any(Pipe::is_nonlinear) {
            return 0;
        }
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
                mean_at += n + m + 3;
                Some(NetMeans {
                    cylinders: cyl,
                    branches: &rest[..m],
                    rpm: rest[m],
                    upstream_pa: rest[m + 1],
                    turbine_dp_dm: rest[m + 2],
                })
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
    /// The duct flow noise's mix: 1 = Davies' law at [`crate::jet_noise::DUCT_FLOW_K`], 0 = off
    /// (the net exactly as without it).
    pub fn set_duct_flow(&mut self, gain: f64) {
        self.duct_flow.gain = gain.max(0.0);
    }

    /// Each expansion site's instantaneous mass flow, kg/s (+ towards the tail), in
    /// [`expansion_sites`] order, from outside (an FV exhaust's cells) for the coming steps;
    /// until then the net uses the flow its own waves carry.
    pub fn set_site_flows(&mut self, flows: &[f64]) {
        self.duct_flow.given.clear();
        self.duct_flow.given.extend_from_slice(flows);
    }

    /// The duct flow noise's mix ([`ExhaustNet::set_duct_flow`]).
    pub fn duct_flow_gain(&self) -> f64 {
        self.duct_flow.gain
    }

    /// The flow out of each open end at the last step as A·Δp/c, kg/s of the tail's reference
    /// gas (the junctions keep A·Δp: an inflow ṁ reads ṁ·C_PRIMARY/C_TAIL here), one per branch
    /// or one for a merge's outlet. Its ripple over its mean is the pulsation's.
    pub fn exit_flows(&self, out: &mut Vec<f64>) {
        out.clear();
        match &self.shared {
            Some(sh) => out.push(sh.duct.far_flow()),
            None => out.extend(self.branches.iter().map(|b| b.tail.far_flow())),
        }
    }

    /// How many expansion sites make duct flow noise.
    pub fn duct_sites(&self) -> usize {
        self.duct_flow.sites.len()
    }

    pub fn step(&mut self, flows: &[f64]) -> f64 {
        if let Some(h) = &mut self.heat {
            h.steps += 1;
        }
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
        let dt = self.dt;
        let df = &mut self.duct_flow;
        let noise = df.gain > 0.0;
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
            // each primary's jet into the collector: its noise half back up the primary, half on
            let mut collector_add = 0.0;
            if noise {
                for i in 0..df.sites.len() {
                    let site = &df.sites[i].site;
                    let (Some(bk), SiteAt::Collector { primary }) = (site.branch, site.at) else {
                        continue;
                    };
                    let Some(j) = b.members.iter().position(|&m| m == primary) else {
                        continue;
                    };
                    if bk != k {
                        continue;
                    }
                    let own = areas[j] * (incoming[j] - outgoing[j]) / C_PRIMARY;
                    let p = df.pressure(i, own, C_PRIMARY, dt);
                    outgoing[j] += p / areas[j].sqrt();
                    collector_add += p / b.collector.area_m2.sqrt();
                }
            }
            for (k, &m) in b.members.iter().enumerate() {
                from_far[m] = clamp_wave(outgoing[k]);
            }
            let collector_near = if noise {
                clamp_wave(outgoing[b.members.len()] + collector_add)
            } else {
                clamp_wave(outgoing[b.members.len()])
            };
            let mut two = [0.0; 2];
            let areas = [b.collector.area_m2, b.tail.end_areas().0];
            let incoming = [b.collector.at_far(), b.tail.at_near()];
            match &mut b.turbine {
                Some(t) => t.process(&areas, &incoming, C_PRIMARY, &mut two),
                None => {
                    junction(&areas, &incoming, &mut two);
                }
            }
            // the path's expansions and its exit, each on the flow through it
            let mut exit = 0.0;
            if noise {
                let own = areas[0] * (incoming[0] - two[0]) / C_COLLECTOR;
                for i in 0..df.sites.len() {
                    let site = &df.sites[i].site;
                    if site.branch != Some(k) {
                        continue;
                    }
                    let at = site.at;
                    match at {
                        SiteAt::Segment { index } => match b.tail.segment_node(index) {
                            None => {
                                let p = df.pressure(i, own, C_TAIL, dt);
                                two[0] += p / areas[0].sqrt();
                                two[1] += p / areas[1].sqrt();
                            }
                            Some(node) => {
                                // the flow through that node (the last step's: one sample)
                                let p = df.pressure(i, b.tail.node_flow(node), C_TAIL, dt);
                                let (a1, a2) = b.tail.node_areas(node);
                                b.tail.inject(node, p / a1.sqrt(), p / a2.sqrt());
                            }
                        },
                        SiteAt::Exit => {
                            let p = df.pressure(i, b.tail.far_flow(), C_TAIL, dt);
                            exit = p / b.tail.end_areas().1.sqrt();
                        }
                        _ => {}
                    }
                }
            }
            b.collector.send(collector_near, clamp_wave(two[0]));
            match &mut self.shared {
                // the tail's far end meets the shared duct below
                Some(sh) => sh.near[k] = clamp_wave(two[1]),
                None => {
                    let arriving = if noise {
                        b.tail.at_far() + exit
                    } else {
                        b.tail.at_far()
                    };
                    let (reflected, out) = b.end.process(arriving);
                    radiated += out;
                    *tail_out = out;
                    let back = if noise { reflected + exit } else { reflected };
                    b.tail.send(clamp_wave(two[1]), clamp_wave(back));
                }
            }
        }
        if let Some(sh) = &mut self.shared {
            // the tails' far ends and the shared duct's near end: one pressure junction
            sh.solve_merge(
                self.branches
                    .iter()
                    .map(|b| (b.tail.end_areas().1, b.tail.at_far())),
                areas,
                incoming,
                outgoing,
            );
            // the merge's sites: each branch's jet into the shared duct, then its expansions
            // and its outlet on all the branches' flow
            let mut exit = 0.0;
            if noise {
                let shared_index = self.branches.len();
                for i in 0..df.sites.len() {
                    let (branch, at) = (df.sites[i].site.branch, df.sites[i].site.at);
                    match (branch, at) {
                        (Some(bk), SiteAt::MergeJunction) => {
                            let own = self.branches.get(bk).map_or(0.0, |b| b.tail.far_flow());
                            let p = df.pressure(i, own, C_TAIL, dt);
                            outgoing[bk] += p / areas[bk].sqrt();
                            outgoing[shared_index] += p / areas[shared_index].sqrt();
                        }
                        (None, SiteAt::MergedSegment { index }) => {
                            if let Some(node) = sh.duct.segment_node(index) {
                                let p = df.pressure(i, sh.duct.node_flow(node), C_TAIL, dt);
                                let (a1, a2) = sh.duct.node_areas(node);
                                sh.duct.inject(node, p / a1.sqrt(), p / a2.sqrt());
                            }
                        }
                        (None, SiteAt::Exit) => {
                            let p = df.pressure(i, sh.duct.far_flow(), C_TAIL, dt);
                            exit = p / sh.duct.end_areas().1.sqrt();
                        }
                        _ => {}
                    }
                }
            }
            // the wave back into the shared duct's far end, and what the open end radiates
            let out = sh.radiate(outgoing[self.branches.len()], exit, self.dt);
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
    use super::{ExpansionSite, SiteAt, expansion_sites};

    fn site(branch: Option<usize>, at: SiteAt, d_jet: f64, d_down: f64) -> ExpansionSite {
        ExpansionSite {
            branch,
            at,
            d_jet,
            d_down,
        }
    }

    /// An inline-4 net driven 0.2 s by firing-like pulses (each primary 0.06 kg/s for a fifth of
    /// its 20 ms cycle, staggered by 5 ms), its radiated samples.
    fn pulsed_inline4(setup: impl Fn(&mut super::ExhaustNet)) -> Vec<f64> {
        let dt = 1.0 / 48_000.0;
        let def = crate::engine::EngineDef::inline4_catalyst_only().exhaust;
        let mut net = super::ExhaustNet::new(&def, 4, dt);
        setup(&mut net);
        (0..(0.2 / dt) as usize)
            .map(|n| {
                let t = n as f64 * dt;
                let flows: Vec<f64> = (0..4)
                    .map(|k| {
                        let ph = ((t + 0.005 * k as f64) / 0.02).fract();
                        if ph < 0.2 { 0.06 } else { 0.0 }
                    })
                    .collect();
                net.step(&flows)
            })
            .collect()
    }

    fn bits_hash(v: &[f64]) -> u64 {
        v.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, x| {
            (h ^ x.to_bits()).wrapping_mul(0x0100_0000_01b3)
        })
    }

    #[test]
    fn duct_flow_off_is_the_old_net() {
        // captured on the code before the duct flow noise (commit 2a93c48), with the noise off;
        // re-captured when the open end began drawing air in through a Borda loss (its pulses
        // stop for 4/5 of each cycle and the tail breathes in) and took Levine & Schwinger's end
        // correction (2026-11-08): the duct noise still adds nothing
        let h = bits_hash(&pulsed_inline4(|n| n.set_duct_flow(0.0)));
        assert_eq!(h, 0xe303_a51b_71df_aec2, "hash {h:#x}");
    }

    /// Mean square of `x` between `lo` and `hi` Hz, from Goertzel bins every 50 Hz.
    fn band_power(x: &[f64], dt: f64, lo: f64, hi: f64) -> f64 {
        let mut f = lo;
        let mut sum = 0.0;
        while f < hi {
            let w = 2.0 * std::f64::consts::PI * f * dt;
            let (mut s1, mut s2) = (0.0, 0.0);
            for &v in x {
                let s0 = v + 2.0 * w.cos() * s1 - s2;
                s2 = s1;
                s1 = s0;
            }
            sum += s1 * s1 + s2 * s2 - 2.0 * w.cos() * s1 * s2;
            f += 50.0;
        }
        sum / (x.len() * x.len()) as f64
    }

    fn steady_inline4(setup: impl Fn(&mut super::ExhaustNet), flow: f64) -> Vec<f64> {
        let dt = 1.0 / 48_000.0;
        let def = crate::engine::EngineDef::inline4().exhaust;
        let mut net = super::ExhaustNet::new(&def, 4, dt);
        setup(&mut net);
        (0..(0.6 / dt) as usize)
            .map(|_| net.step(&[flow; 4]))
            .collect()
    }

    #[test]
    fn an_expansion_with_flow_hisses() {
        let dt = 1.0 / 48_000.0;
        let off = steady_inline4(|n| n.set_duct_flow(0.0), 0.03);
        let on = steady_inline4(|n| n.set_duct_flow(1.0), 0.03);
        // the last 0.2 s: the pipes' ringing from the flow's start has died away
        let (a, b) = (
            band_power(&off[19_200..], dt, 1000.0, 5000.0),
            band_power(&on[19_200..], dt, 1000.0, 5000.0),
        );
        assert!(b > 2.0 * a && b > 0.0, "on {b:.3e} off {a:.3e}");
    }

    #[test]
    fn no_flow_no_hiss() {
        let off = steady_inline4(|n| n.set_duct_flow(0.0), 0.0);
        let on = steady_inline4(|n| n.set_duct_flow(1.0), 0.0);
        assert_eq!(bits_hash(&on), bits_hash(&off));
    }

    #[test]
    fn reversed_flow_is_silent_at_every_site() {
        let off = steady_inline4(|n| n.set_duct_flow(0.0), 0.03);
        let on = steady_inline4(
            |n| {
                n.set_duct_flow(1.0);
                n.set_site_flows(&[-0.02; 6]);
            },
            0.03,
        );
        assert_eq!(bits_hash(&on), bits_hash(&off));
    }

    #[test]
    fn a_steady_flow_leaves_by_the_exit() {
        // 0.03 kg/s into each of four primaries, held. The junctions keep A·Δp (one ρc for all
        // the pipes), so the tail's A·Δp/c reads the flow in its own reference gas:
        // 0.12 · C_PRIMARY/C_TAIL = 0.12 · 600/500 = 0.144 kg/s (the walls' loss: ±5 %)
        let dt = 1.0 / 48_000.0;
        let def = crate::engine::EngineDef::inline4().exhaust;
        let mut net = super::ExhaustNet::new(&def, 4, dt);
        for _ in 0..(1.0 / dt) as usize {
            net.step(&[0.03; 4]);
        }
        let mut out = Vec::new();
        net.exit_flows(&mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0] / 0.144 - 1.0).abs() < 0.05, "{}", out[0]);
    }

    #[test]
    fn the_inline4_expands_where_by_hand() {
        // presets/inline4.yaml: 4 primaries of 40 mm into one 55 mm collector ((55/40)² = 1.89
        // ≥ 1.5: each primary's jet expands), the collector into a 139.9 mm catalyst (6.5×),
        // the catalyst into the 50 mm tail (a contraction: none), the tail's exit
        let ex = crate::engine::EngineDef::inline4_catalyst_only().exhaust;
        let want = vec![
            site(Some(0), SiteAt::Collector { primary: 0 }, 0.04, 0.055),
            site(Some(0), SiteAt::Collector { primary: 1 }, 0.04, 0.055),
            site(Some(0), SiteAt::Collector { primary: 2 }, 0.04, 0.055),
            site(Some(0), SiteAt::Collector { primary: 3 }, 0.04, 0.055),
            site(Some(0), SiteAt::Segment { index: 0 }, 0.055, 0.1399),
            site(Some(0), SiteAt::Exit, 0.05, 0.05),
        ];
        assert_eq!(expansion_sites(&ex, 4), want);
    }

    #[test]
    fn the_lfa_sites_by_hand() {
        // presets/lfa-v10.yaml: banks 1–5 and 6–10, primaries 42 mm into 65 mm collectors
        // (2.4×), each into a 153.4 mm catalyst, its 70 mm pipe into the shared 85 mm duct
        // ((85/70)² = 1.47 < 1.5: no expansion site), one merged exit of 103.9 mm. The bypass
        // silencer's chambers are not sites (this plan's ruling).
        let ex = crate::engine::EngineDef::lfa_v10().exhaust;
        let got = expansion_sites(&ex, 10);
        let collectors = got
            .iter()
            .filter(|s| matches!(s.at, SiteAt::Collector { .. }))
            .count();
        assert_eq!(collectors, 10);
        assert_eq!(
            got.iter()
                .filter(|s| s.at == SiteAt::Segment { index: 0 })
                .count(),
            2
        );
        assert!(!got.iter().any(|s| s.at == SiteAt::MergeJunction));
        let exits: Vec<_> = got.iter().filter(|s| s.at == SiteAt::Exit).collect();
        assert_eq!(exits.len(), 1);
        assert_eq!(exits[0].branch, None);
        assert!((exits[0].d_jet - 0.103_923_048_454_132_62).abs() < 1e-12);
        // the second bank's collector sites carry cylinders 6–10 (indices 5–9)
        assert_eq!(got[6].at, SiteAt::Collector { primary: 5 });
        assert_eq!(got[6].branch, Some(1));
    }

    #[test]
    fn a_pipe_no_wider_than_its_primaries_has_only_its_exit() {
        // primaries and collector alike (no expansion), one plain pipe of the same bore
        let mut ex = crate::engine::EngineDef::inline4().exhaust;
        ex.primary_diameter_m = 0.05;
        ex.collector_diameter_m = 0.05;
        ex.after_collector = vec![super::Segment::Pipe {
            length_m: 1.0,
            diameter_m: 0.05,
        }];
        assert_eq!(
            expansion_sites(&ex, 4),
            vec![site(Some(0), SiteAt::Exit, 0.05, 0.05)]
        );
    }

    #[test]
    fn an_empty_chamber_is_a_site_and_a_straight_through_one_is_not() {
        let mut ex = crate::engine::EngineDef::inline4().exhaust;
        ex.after_collector = vec![
            super::Segment::Muffler {
                length_m: 0.3,
                diameter_m: 0.15,
                tube_diameter_m: 0.05,
                packing_resistivity: 16000.0,
                packing_porosity: 0.95,
            },
            super::Segment::Muffler {
                length_m: 0.3,
                diameter_m: 0.15,
                tube_diameter_m: 0.0,
                packing_resistivity: 0.0,
                packing_porosity: 1.0,
            },
            super::Segment::Pipe {
                length_m: 0.5,
                diameter_m: 0.05,
            },
        ];
        let got = expansion_sites(&ex, 4);
        // the 55 mm collector into the 50 mm tube: no; the tube into the empty 150 mm can: yes
        assert!(!got.iter().any(|s| s.at == SiteAt::Segment { index: 0 }));
        assert!(got.contains(&site(Some(0), SiteAt::Segment { index: 1 }, 0.05, 0.15)));
    }

    #[test]
    fn a_limp_shell_moves_by_the_mass_law() {
        // A wall of 4.5 kg/m² (1 mm of titanium) between gas of ρc 250 and air of ρc 413.4
        // (1.204·343), driven by 1 Pa at 500 Hz: v = p/|Z₁ + Z₂ + jωm| (Kinsler et al.,
        // *Fundamentals of Acoustics*, §6.4, a limp partition) = 1/|663.4 + j·14137.2| =
        // 1/14152.8 = 7.0658e-5 m/s; over 0.1 m² that is 7.0658e-6 m³/s.
        let dt = 1.0 / 192_000.0;
        let mut s = Shell::new(0.001, 4500.0, 0.1, 0.0, 250.0);
        let mut peak: f64 = 0.0;
        let n = (0.2 / dt) as usize;
        for k in 0..n {
            let p = (2.0 * std::f64::consts::PI * 500.0 * k as f64 * dt).sin();
            let q = s.step(p, dt);
            if k > n / 2 {
                peak = peak.max(q.abs());
            }
        }
        assert!((peak / 7.0658e-6 - 1.0).abs() < 0.01, "{peak:e}");
        // no shell given: nothing radiates
        let mut none = Shell::new(0.0, 4500.0, 0.1, 0.0, 250.0);
        assert_eq!(none.step(1.0, dt), 0.0);
    }

    #[test]
    fn a_stiff_shell_passes_less_below_its_resonance() {
        // The same wall held at f₀ = 2000 Hz, driven by 1 Pa at 100 Hz: k = (2π·2000)²·4.5 =
        // 7.1061e8 Pa/m, its damping 0.1·√(k·m) = 5654.9, so v = 1/|(663.4 + 5654.9) +
        // j(ω·4.5 − k/ω)| = 1/|6318.3 − j·1128145.9| (Kinsler et al., §1.7, a driven damped
        // oscillator); over 0.1 m², 8.8640e-8 m³/s — 1/400 of what the limp wall passes.
        let dt = 1.0 / 192_000.0;
        let mut s = Shell::new(0.001, 4500.0, 0.1, 2000.0, 250.0);
        let mut peak: f64 = 0.0;
        let n = (0.5 / dt) as usize;
        for k in 0..n {
            let p = (2.0 * std::f64::consts::PI * 100.0 * k as f64 * dt).sin();
            let q = s.step(p, dt);
            if k > n / 2 {
                peak = peak.max(q.abs());
            }
        }
        assert!((peak / 8.8640e-8 - 1.0).abs() < 0.01, "{peak:e}");
    }

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

    #[test]
    fn a_block_reads_each_steps_means_where_the_engine_packed_them() {
        // Three steps, the first and the last carrying means (cylinders, branches, rpm, the
        // turbine's inlet pressure and slope packed one after another): the block must give the
        // same as handing them over step by step, bit for bit
        let def = ExhaustDef::default();
        let dt = 1.0 / 48_000.0;
        let mut direct = ExhaustNet::new(&def, 4, dt);
        let mut blocked = direct.clone();
        let (n, m) = (direct.primaries.len(), direct.tails.len());
        let means = [
            (vec![0.010; n], vec![0.040; m], 900.0, 1.2e5, 0.0),
            (vec![0.030; n], vec![0.120; m], 3000.0, 1.8e5, 800.0),
        ];
        let mut input = BlockIn {
            steps: 3,
            ..BlockIn::default()
        };
        for s in 0..3 {
            input
                .flows
                .extend((0..n).map(|i| 0.02 * (s * n + i + 1) as f64));
            input.mach.extend(std::iter::repeat_n(0.01, m));
            let carry = s != 1;
            input.has_mean.push(carry);
            if carry {
                let (c, b, rpm, up, dp) = &means[s / 2];
                input.mean.extend_from_slice(c);
                input.mean.extend_from_slice(b);
                input.mean.extend_from_slice(&[*rpm, *up, *dp]);
            }
        }
        let mut out = BlockOut::default();
        blocked.run_block(&input, &mut out);
        let mut tails = Vec::new();
        for s in 0..3 {
            let mean = (s != 1).then(|| {
                let (c, b, rpm, up, dp) = &means[s / 2];
                NetMeans {
                    cylinders: c,
                    branches: b,
                    rpm: *rpm,
                    upstream_pa: *up,
                    turbine_dp_dm: *dp,
                }
            });
            direct.prepare(&input.mach[s * m..(s + 1) * m], mean);
            direct.step(&input.flows[s * n..(s + 1) * n]);
            tails.extend_from_slice(&direct.tails);
        }
        assert_eq!(out.tails, tails);
        // the last step's flows are the ones the net follows, its turbine joined the collectors
        assert_eq!(blocked.mean_flows, direct.mean_flows);
        assert_eq!(blocked.mean_flows.0, means[1].0);
        assert!(blocked.branches.iter().all(|b| b.turbine.is_some()));
    }

    #[test]
    fn a_turbines_inlet_pressure_reaches_the_primaries_collectors_and_probe() {
        let mut net = ExhaustNet::new(&ExhaustDef::default(), 4, 1.0 / 48_000.0);
        let mut probe = ArrivalProbe::new(&net);
        net.set_mean_pressure(2.0e5);
        assert!(net.primaries.iter().all(|p| p.mean_pressure() == 2.0e5));
        assert!(
            net.branches
                .iter()
                .all(|b| b.collector.mean_pressure() == 2.0e5)
        );
        probe.set_mean_pressure(1.5e5);
        assert!(probe.pipes.iter().all(|p| p.mean_pressure() == 1.5e5));
    }

    #[test]
    fn a_turbine_follows_the_collectors_only_with_a_positive_slope() {
        let mut net = ExhaustNet::new(&ExhaustDef::default(), 4, 1.0 / 48_000.0);
        net.set_turbine_resistance(0.0);
        assert!(net.branches.iter().all(|b| b.turbine.is_none()));
        net.set_turbine_resistance(500.0);
        assert!(!net.branches.is_empty());
        assert!(net.branches.iter().all(|b| b.turbine.is_some()));
    }

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
            // sealed: the paths heard one at a time (the leak has its own test)
            valve_clearance_m: 0.0,
            ..Bypass::default()
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
    fn a_shell_round_trips_and_old_bypasses_have_none() {
        let old: Bypass = serde_yaml_ng::from_str("open_rpm: 3000.0\n").unwrap();
        assert_eq!((old.shell_thickness_m, old.shell_resonance_hz), (0.0, 0.0));
        let mut d = valved_banks();
        let b = d.merge.as_mut().unwrap().bypass.as_mut().unwrap();
        (b.shell_thickness_m, b.shell_density_kg_m3) = (0.001, 4510.0);
        (b.shell_area_m2, b.shell_resonance_hz) = (0.14, 300.0);
        let json: ExhaustDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let yaml: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((json, yaml), (d.clone(), d));
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
        // a bypass written before the clearance was a field gets a butterfly's 0.5 mm
        let old: Bypass = serde_yaml_ng::from_str("open_rpm: 3000.0\n").unwrap();
        assert_eq!(old.valve_clearance_m, 0.0005);
        let mut d = valved_banks();
        d.merge
            .as_mut()
            .unwrap()
            .bypass
            .as_mut()
            .unwrap()
            .valve_clearance_m = 0.001;
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
    fn a_butterflys_leak_is_its_clearance_round_its_disc() {
        // the gap is a ring π·D·c of the bore's π·D²/4: 4c/D. By hand: 0.5 mm round an 85 mm
        // disc is 2/85 = 0.023529; round a 50 mm one 0.04; no gap or no pipe, nothing leaks
        assert!((valve_leak_share(0.0005, 0.085) - 0.023529).abs() < 1e-6);
        assert!((valve_leak_share(0.0005, 0.05) - 0.04).abs() < 1e-12);
        assert_eq!(valve_leak_share(0.0, 0.085), 0.0);
        assert_eq!(valve_leak_share(0.0005, 0.0), 0.0);
        // a gap as wide as the disc is no valve: all open
        assert_eq!(valve_leak_share(0.05, 0.085), 1.0);
    }

    #[test]
    fn a_shut_valve_leaks_round_its_disc() {
        // a butterfly keeps radial clearance for its thermal growth: shut, a pulse still takes
        // the short way through the gap, 172.8 steps (1.8 ms at 96 kHz) before the chamber path
        // (`shut_the_sound_goes_round_the_chambers_open_it_takes_the_short_way`). Sealed, nothing
        // arrives that early
        let dt = 1.0 / 96_000.0;
        let heard = |clearance_m: f64| {
            let mut d = valved_banks();
            d.merge
                .as_mut()
                .unwrap()
                .bypass
                .as_mut()
                .unwrap()
                .valve_clearance_m = clearance_m;
            let mut net = ExhaustNet::new(&d, 2, dt);
            net.set_engine_speed(1000.0);
            for _ in 0..(0.4 / dt) as usize {
                net.step(&[0.0, 0.0]);
            }
            (0..(0.02 / dt) as usize)
                .map(|k| net.step(&[if k < 5 { 0.05 } else { 0.0 }, 0.0]))
                .collect::<Vec<f64>>()
        };
        let first = |out: &[f64]| {
            let peak = out.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            out.iter().position(|v| v.abs() > 0.05 * peak).unwrap()
        };
        let (sealed, leaking) = (heard(0.0), heard(0.0005));
        let early = first(&sealed) as f64 - first(&leaking) as f64;
        assert!(
            (early - 172.8).abs() < 4.0,
            "the leak arrives {early} steps early"
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
                // linear primaries: a steepening one is never run ahead (max_block)
                steepening: false,
                gas_temperature: false,
                ..ExhaustDef::default()
            };
            ExhaustNet::new(&def, 4, 1.0 / fs).max_block()
        };
        assert_eq!(block(0.7, 192_000.0), 64);
        assert_eq!(block(0.71, 48_000.0), 28);
        assert_eq!(block(0.205, 48_000.0), 8);
        assert_eq!(block(0.19, 48_000.0), 0);
        assert_eq!(block(0.05, 48_000.0), 0);
        // steepening primaries: never handed off
        assert_eq!(
            ExhaustNet::new(&ExhaustDef::default(), 4, 1.0 / 192_000.0).max_block(),
            0
        );
    }

    #[test]
    fn the_probe_reads_the_waves_the_net_would_with_results_still_out() {
        // the net steps on its own; the probe is given each step's result only when it is
        // `late − 1` steps old, and a change of the mean flows at step 300: at every step it
        // reads, with its results that late, the very bits the net reads
        // the probe runs linear primaries ahead (a steepening net is never handed off)
        let def = ExhaustDef {
            steepening: false,
            gas_temperature: false,
            ..ExhaustDef::default()
        };
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
            probe.advance();
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
    fn side_by_side_outlets_share_the_jet_and_not_the_pipe() {
        // Two 50 mm outlets: 2·π/4·0.05² = 3.927e-3 m², the area of one Ø70.71 mm jet; the bore
        // the pipe's sound leaves through stays the pipe's.
        let mut d = ExhaustDef {
            tail_outlets: 2,
            ..ExhaustDef::default()
        };
        d.after_collector.clear();
        assert!((d.exit_losses().outlet_area - 3.926_991e-3).abs() < 1e-8);
        assert!((d.jet_diameter() - 0.070_711).abs() < 1e-6);
        assert_eq!(d.radiating_diameter(), 0.05);
        let old: ExhaustDef = serde_yaml_ng::from_str("tail_diameter_m: 0.05\n").unwrap();
        assert_eq!(old.tail_outlets, 1);
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
    fn a_measured_chamber_takes_its_published_loss() {
        // a chamber whose maker measured its transmission loss (JP 2008-169751 A, Fig. 6: the
        // LFA's spherical final chamber, 15 dB at 100 Hz, ≈ 20 dB flat from 200 Hz to 1 kHz)
        // takes that loss off a tone, where a 1D chamber of its size ripples ±10 dB with its
        // axial standing waves (the flat chamber's dips the figure contrasts the sphere with)
        let tail = Segment::Pipe {
            length_m: 0.5,
            diameter_m: 0.05,
        };
        let plain = with_path(vec![
            Segment::Pipe {
                length_m: 0.2,
                diameter_m: 0.05,
            },
            tail.clone(),
        ]);
        let measured = with_path(vec![
            Segment::Measured {
                length_m: 0.2,
                bore_m: 0.05,
                transmission_loss_db: vec![
                    [100.0, 15.0],
                    [200.0, 19.0],
                    [300.0, 20.0],
                    [1000.0, 19.0],
                ],
            },
            tail,
        ]);
        // the element itself: its published loss within the equalizer's fit, never a gain
        let fs = 1.0 / DT;
        let eq = crate::silencer2d::measured_eq(
            &[[100.0, 15.0], [200.0, 19.0], [300.0, 20.0], [1000.0, 19.0]],
            fs,
        );
        for (f, want) in [(100.0, 15.0), (300.0, 20.0), (1000.0, 19.0), (2000.0, 19.0)] {
            let db = eq.response_db(f, fs);
            assert!(
                (db + want).abs() < 2.0,
                "{f} Hz: {db:.1} dB, published −{want}"
            );
        }
        // below the first point the loss falls away: a compliance passes the lowest orders
        let db = eq.response_db(50.0, fs);
        assert!(db > -12.0 && db < -4.0, "50 Hz: {db:.1} dB");
        let mut f = 20.0;
        while f < 0.45 * fs {
            assert!(eq.response_db(f, fs) <= 0.1, "{f} Hz: gain");
            f *= 1.1;
        }
        // through the net, between the open end's echoes: the measured line is the quieter
        for f in [300.0, 1000.0] {
            let db = 20.0 * (tone_through(&measured, f) / tone_through(&plain, f)).log10();
            assert!(
                db < -12.0 && db > -32.0,
                "{f} Hz: {db:.1} dB through the line"
            );
        }
    }

    #[test]
    fn a_measured_chamber_round_trips() {
        let def = with_path(vec![
            Segment::Measured {
                length_m: 0.2,
                bore_m: 0.085,
                transmission_loss_db: vec![[100.0, 15.0], [1000.0, 19.0]],
            },
            Segment::Pipe {
                length_m: 0.1,
                diameter_m: 0.06,
            },
        ]);
        let json: ExhaustDef = serde_json::from_str(&serde_json::to_string(&def).unwrap()).unwrap();
        assert_eq!(json, def);
        let yaml: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&def).unwrap()).unwrap();
        assert_eq!(yaml, def);
        assert_eq!(def.outlet_diameter(), 0.06);
        assert_eq!(exit_diameter(&def.after_collector[..1]), 0.085);
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
        // a faint pulse (0.2 g/s, a few hundred pascals): it travels at c, where a loud one
        // steepens and arrives sooner (`waveguide::DelayLine::nonlinear`)
        let mut net = ExhaustNet::new(def, n, DT);
        let mut flows = vec![0.0; n];
        let out: Vec<f64> = (0..20_000)
            .map(|k| {
                flows[0] = if k < 20 { 2e-4 } else { 0.0 };
                net.step(&flows)
            })
            .collect();
        half_rise(&out, 1e-3).expect("nothing radiated")
    }

    /// When a pulse arrives: the step its first lobe (the first run of one sign past `floor`)
    /// reaches half its own extreme. A line's reading spreads a pulse by a step or two either
    /// way (`waveguide::MIN_KERNEL`); the half-rise of the lobe is where its edge is, where
    /// the first step over a floor would move with the spreading.
    fn half_rise(x: &[f64], floor: f64) -> Option<usize> {
        let i0 = x.iter().position(|v| v.abs() > floor)?;
        let sign = x[i0].signum();
        let lobe = x[i0..]
            .iter()
            .take_while(|v| v.signum() == sign || v.abs() <= floor)
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max);
        x[i0..]
            .iter()
            .position(|v| v.abs() >= 0.5 * lobe)
            .map(|i| i0 + i)
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
    fn a_loud_periodic_drive_gives_a_periodic_exhaust() {
        // The LFA's exhaust driven by the same blowdown every cycle (0.15 kg/s peaks, 8000 rpm:
        // 2880 steps a cycle) must answer with the same sound every cycle. Its waves steepen
        // into shocks in the primaries. Shocks read with no width made the sound jump whenever a
        // sample more or less joined one, and the echoes between valve and junction never
        // settled (15 % from cycle to cycle). The finite-volume exhaust, driven the same way,
        // settles within 0.15 %; this within 0.34 % (both measured 2026-10-26)
        let def = crate::engine::EngineDef::lfa_v10().exhaust;
        let dt = 1.0 / 192_000.0;
        let mut net = ExhaustNet::new(&def, 10, dt);
        let period = 2880;
        let mut prev = vec![0.0; period];
        let (mut diff, mut peak) = (0.0f64, 0.0f64);
        let mut flows = [0.0; 10];
        for n in 0..period * 60 {
            for (c, f) in flows.iter_mut().enumerate() {
                let ph = ((n + c * period / 10) % period) as f64 / period as f64;
                *f = if ph < 0.25 {
                    0.15 * (std::f64::consts::PI * ph / 0.25).sin().powi(2)
                } else {
                    0.0
                };
            }
            let r = net.step(&flows);
            let k = n % period;
            if n >= period * 59 {
                diff = diff.max((r - prev[k]).abs());
                peak = peak.max(r.abs());
            }
            prev[k] = r;
        }
        assert!(diff < 0.01 * peak, "cycle to cycle {diff} of {peak}");
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
    fn a_longer_bank_line_delays_its_pulses_by_its_extra_length() {
        // 0.2 m more collector on the second bank: its pulses reach the open end 0.2/560 s
        // later, 68.57 steps at 192 kHz (C_COLLECTOR = 560 m/s); the first bank's are unchanged
        let def = ExhaustDef {
            collectors: vec![vec![1], vec![2]],
            collector_extra_length_m: vec![0.0, 0.2],
            ..ExhaustDef::default()
        };
        // a faint pulse: it travels at c (a loud one steepens and arrives sooner)
        let arrival = |d: &ExhaustDef, cyl: usize| {
            let mut net = ExhaustNet::new(d, 2, DT);
            let out: Vec<f64> = (0..20_000)
                .map(|k| {
                    let mut f = [0.0; 2];
                    if k < 20 {
                        f[cyl] = 2e-4;
                    }
                    net.step(&f)
                })
                .collect();
            half_rise(&out, 1e-3).unwrap() as f64
        };
        let even = ExhaustDef {
            collector_extra_length_m: Vec::new(),
            ..def.clone()
        };
        assert_eq!(arrival(&def, 0), arrival(&even, 0));
        let late = arrival(&def, 1) - arrival(&even, 1);
        assert!((late - 68.57).abs() <= 1.0, "{late} steps");
    }

    #[test]
    fn a_primary_built_long_delays_its_own_pulses_only() {
        // cylinder 2's primary 0.1 m long of the drawing: its pulses reach the open end
        // 0.1/600 s later, 32 steps at 192 kHz (C_PRIMARY = 600 m/s); cylinder 1's are unchanged
        let def = ExhaustDef {
            collectors: vec![vec![1, 2]],
            primary_length_deviation_m: vec![0.0, 0.1],
            ..ExhaustDef::default()
        };
        let arrival = |d: &ExhaustDef, cyl: usize| {
            let mut net = ExhaustNet::new(d, 2, DT);
            let out: Vec<f64> = (0..20_000)
                .map(|k| {
                    let mut f = [0.0; 2];
                    if k < 20 {
                        f[cyl] = 2e-4;
                    }
                    net.step(&f)
                })
                .collect();
            half_rise(&out, 1e-3).unwrap() as f64
        };
        let drawn = ExhaustDef {
            primary_length_deviation_m: Vec::new(),
            ..def.clone()
        };
        assert_eq!(arrival(&def, 0), arrival(&drawn, 0));
        let late = arrival(&def, 1) - arrival(&drawn, 1);
        assert!((late - 32.0).abs() <= 1.0, "{late} steps");
    }

    #[test]
    fn the_gas_cools_along_the_exhaust() {
        // 1100 K gas leaving both ports at 20 g/s through the valved banks (bypass shut): each
        // pipe is cooler than the one feeding it, all between the air and the port
        let def = ExhaustDef {
            gas_temperature: true,
            ..valved_banks()
        };
        let mut net = ExhaustNet::new(&def, 2, DT);
        let built = net.gas_sound_speeds();
        assert_eq!(&built[..3], &[C_PRIMARY, C_PRIMARY, C_COLLECTOR]);
        net.set_port_temperature(&[1100.0, 1100.0]);
        net.set_mean_flow(&[0.02, 0.02], &[0.02, 0.02]);
        let c = net.gas_sound_speeds();
        let (port, air) = (
            crate::pipe_heat::sound_speed(1100.0),
            crate::pipe_heat::sound_speed(300.0),
        );
        assert!(c.iter().all(|&x| x > air && x < port), "{c:?}");
        // primaries, bank 0's collector, then what follows it down to the tips
        assert_eq!(c[0], c[1]);
        assert!(c[2] < c[0], "{c:?}");
        assert!(c[2..].iter().all(|&x| x < c[0]), "{c:?}");
        assert!(c[c.len() - 1] < c[2], "{c:?}");
        // every pipe left its fixed speed, and a hotter port warms the gas all the way down
        assert!(c.iter().zip(&built).all(|(a, b)| a != b), "{c:?}");
        // the gas in the pipes takes time to follow (its exchange rate): 50 ms
        net.set_port_temperature(&[1300.0, 1300.0]);
        for _ in 0..(0.05 / DT) as usize {
            net.step(&[0.0, 0.0]);
        }
        net.set_mean_flow(&[0.02, 0.02], &[0.02, 0.02]);
        let hotter = net.gas_sound_speeds();
        assert!(hotter.iter().zip(&c).all(|(h, c)| h > c), "{hotter:?}");
    }

    #[test]
    fn streams_mix_by_their_mass_flows() {
        // 30 g/s at 1000 K and 10 g/s at 700 K: (30·1000 + 10·700)/40 = 925 K; no flow: the mean
        assert!((mixed([(1000.0, 0.03), (700.0, 0.01)].into_iter()) - 925.0).abs() < 1e-9);
        assert!((mixed([(1000.0, 0.0), (700.0, 0.0)].into_iter()) - 850.0).abs() < 1e-9);
    }

    #[test]
    fn without_gas_temperature_the_pipes_keep_their_fixed_speeds() {
        let mut net = ExhaustNet::new(&valved_banks(), 2, DT);
        let built = net.gas_sound_speeds();
        net.set_port_temperature(&[1100.0, 1100.0]);
        net.set_mean_flow(&[0.02, 0.02], &[0.02, 0.02]);
        assert_eq!(net.gas_sound_speeds(), built);
        assert_eq!(
            net.max_block(),
            ExhaustNet::new(&valved_banks(), 2, DT).max_block()
        );
    }

    #[test]
    fn a_net_with_gas_temperature_stays_on_the_engines_thread() {
        let linear = ExhaustDef {
            steepening: false,
            ..valved_banks()
        };
        let def = ExhaustDef {
            gas_temperature: true,
            ..linear.clone()
        };
        assert!(ExhaustNet::new(&linear, 2, DT).max_block() > 0);
        assert_eq!(ExhaustNet::new(&def, 2, DT).max_block(), 0);
    }

    #[test]
    fn gas_temperature_round_trips_and_defaults_to_off() {
        let old: ExhaustDef = serde_yaml_ng::from_str("tail_length_m: 1.0\n").unwrap();
        assert!(!old.gas_temperature);
        let on = ExhaustDef {
            gas_temperature: true,
            ..ExhaustDef::default()
        };
        let yaml = serde_yaml_ng::to_string(&on).unwrap();
        assert_eq!(serde_yaml_ng::from_str::<ExhaustDef>(&yaml).unwrap(), on);
        let json = serde_json::to_string(&on).unwrap();
        assert_eq!(serde_json::from_str::<ExhaustDef>(&json).unwrap(), on);
    }

    #[test]
    fn primary_deviations_round_trip_and_default_to_none() {
        let old: ExhaustDef = serde_yaml_ng::from_str("tail_length_m: 1.0\n").unwrap();
        assert!(old.primary_length_deviation_m.is_empty());
        let d = ExhaustDef {
            primary_length_deviation_m: vec![0.016, -0.024],
            ..ExhaustDef::default()
        };
        let j: ExhaustDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let y: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((j, y), (d.clone(), d));
    }

    #[test]
    fn bank_extra_lengths_round_trip_and_default_to_none() {
        let old: ExhaustDef = serde_yaml_ng::from_str("tail_length_m: 1.0\n").unwrap();
        assert!(old.collector_extra_length_m.is_empty());
        let d = ExhaustDef {
            collector_extra_length_m: vec![0.0, 0.1],
            ..ExhaustDef::default()
        };
        let j: ExhaustDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let y: ExhaustDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!((j, y), (d.clone(), d));
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
        // faint, so it travels at c (a loud pulse steepens and comes back sooner)
        let back: Vec<f64> = (0..2000usize)
            .map(|k| {
                let p = net.arriving(0);
                net.step(&[if k < 5 { 1e-4 } else { 0.0 }, 0.0]);
                p
            })
            .collect();
        let k = half_rise(&back, 1e-3).expect("reflection never arrived");
        let p = back[k];
        assert!(
            k.abs_diff(round_trip) <= 4,
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

#[cfg(test)]
mod probe {
    //! Scratch probe (ignored): the LFA line's linear transfer, one cylinder's tone to the
    //! radiated sum, at the bank orders of 6700 rpm. `cargo test -p engine-core probe -- --ignored --nocapture`.
    use super::*;
    use crate::engine::EngineDef;
    const DT: f64 = 1.0 / 192_000.0;

    fn tone(def: &ExhaustDef, f: f64, rpm: f64) -> f64 {
        let mut net = ExhaustNet::new(def, 10, DT);
        net.set_engine_speed(rpm);
        let mut flows = [0.0; 10];
        let warm = (0.35 / DT) as usize;
        let n = (0.25 / DT) as usize;
        let mut peak: f64 = 0.0;
        for k in 0..warm + n {
            flows[0] = if k >= warm {
                0.01 * (2.0 * std::f64::consts::PI * f * (k - warm) as f64 * DT).sin()
            } else {
                0.0
            };
            let y = net.step(&flows);
            if k > warm + n / 2 {
                peak = peak.max(y.abs());
            }
        }
        20.0 * peak.log10()
    }

    fn report(name: &str, def: &ExhaustDef) {
        let f0 = 6700.0 / 120.0 * 5.0;
        let v: Vec<String> = (1..=8)
            .map(|n| format!("{:6.1}", tone(def, f0 * n as f64, 6700.0)))
            .collect();
        println!("{name:28} bank orders 1..8 (279 Hz..): {}", v.join(" "));
    }

    fn sweep(name: &str, def: &ExhaustDef) {
        let mut s = String::new();
        let mut f = 200.0;
        while f <= 1300.0 {
            s.push_str(&format!("{f:.0}:{:.0} ", tone(def, f, 6700.0)));
            f += 25.0;
        }
        println!("{name:28} {s}");
    }

    #[test]
    #[ignore]
    fn print_lfa_line_sweeps() {
        let mut lin = EngineDef::lfa_v10().exhaust;
        lin.steepening = false;
        lin.merge = None;
        let pipe = |l: f64| Segment::Pipe {
            length_m: l,
            diameter_m: 0.07,
        };
        sweep("no merge", &lin);
        let mut d = lin.clone();
        d.after_collector = vec![pipe(1.98)];
        sweep("no cat", &d);
        let mut d = lin.clone();
        d.after_collector[0] = Segment::Muffler {
            length_m: 0.13,
            diameter_m: 0.1534,
            tube_diameter_m: 0.0,
            packing_resistivity: 0.0,
            packing_porosity: 1.0,
        };
        sweep("cat as empty chamber", &d);
        for l in [1.5, 2.5] {
            let mut d = lin.clone();
            d.after_collector[1] = pipe(l);
            sweep(&format!("tail {l}"), &d);
        }
        let mut d = lin.clone();
        d.after_collector = vec![pipe(2.11)];
        sweep("no cat, tail 2.11", &d);
    }
    #[test]
    #[ignore]
    fn print_lfa_line_transfer() {
        let base = EngineDef::lfa_v10().exhaust;
        let mut lin = base.clone();
        lin.steepening = false;
        report("full (steepening)", &base);
        report("full linear", &lin);
        let mut d = lin.clone();
        d.merge = None;
        report("no merge", &d);
        let mut d = lin.clone();
        d.merge.as_mut().unwrap().bypass = None;
        report("no bypass", &d);
        let mut d = lin.clone();
        d.after_collector = vec![Segment::Pipe {
            length_m: 1.98,
            diameter_m: 0.07,
        }];
        report("no catalyst", &d);
        let mut d = lin.clone();
        d.merge = None;
        d.after_collector = vec![Segment::Pipe {
            length_m: 1.98,
            diameter_m: 0.07,
        }];
        report("no merge, no catalyst", &d);
        for l in [0.45, 0.55, 0.675, 0.8] {
            let mut d = lin.clone();
            d.merge = None;
            d.primary_length_m = l;
            report(&format!("no merge, primaries {l}"), &d);
        }
        for l in [0.1, 0.3, 0.6] {
            let mut d = lin.clone();
            d.merge = None;
            d.collector_length_m = l;
            report(&format!("no merge, collector {l}"), &d);
        }
        // sweep of the full linear net
        let mut s = String::new();
        let mut f = 100.0;
        while f <= 1500.0 {
            s.push_str(&format!("{f:.0}:{:.1} ", tone(&lin, f, 6700.0)));
            f += 50.0;
        }
        println!("full linear sweep {s}");
    }

    #[test]
    fn the_probe_reads_each_pipes_gas_by_hand() {
        // a primary at the default 600 m/s: T = 600²/(1.4·287) = 895.968 K, ρ = 101 325 /
        // (287 · 895.968) = 0.394042 kg/m³; 0.01 kg/s through a 40 mm bore (12.566 cm²) is
        // 20.1952 m/s
        let mut net = ExhaustNet::new(&ExhaustDef::default(), 4, 1.0 / 48_000.0);
        net.set_mean_flow(&[0.01, 0.0, -0.01, 0.0], &[0.02]);
        let mut out = Vec::new();
        net.probe(&mut out);
        assert_eq!(out.len(), 5, "four primaries and one collector");
        assert!(
            (out[0].temperature_k - 895.968).abs() < 1e-3,
            "{:?}",
            out[0]
        );
        assert!((out[0].velocity_m_s - 20.1952).abs() < 1e-3, "{:?}", out[0]);
        assert_eq!(out[1].velocity_m_s, 0.0);
        assert!(
            (out[2].velocity_m_s + 20.1952).abs() < 1e-3,
            "reversion runs backwards"
        );
        assert_eq!(
            out[0].wall_k, None,
            "no wall model unless the gas temperature runs"
        );
        assert!(out[4].velocity_m_s > 0.0);
    }

    #[test]
    fn the_probe_allocates_nothing_once_it_has_room() {
        let net = ExhaustNet::new(&ExhaustDef::default(), 4, 1.0 / 48_000.0);
        let mut out = Vec::with_capacity(16);
        let ptr = out.as_ptr();
        net.probe(&mut out);
        net.probe(&mut out);
        assert_eq!((out.len(), out.capacity(), out.as_ptr()), (5, 16, ptr));
    }
}
