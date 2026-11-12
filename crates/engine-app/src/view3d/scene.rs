//! The engine's parts as placed unit meshes: the still ones built once from the definition, the
//! moving ones refilled every frame from the cylinders' state. Same proportions as the 2D view
//! (`view::cylinder`): piston 0.6 bore tall with its crown 0.35 bore over the pin, head 0.35
//! bore, valves at ±0.25 bore across the bore. Drawn parts beyond the 2D view's (rings, wrist
//! pin, rod eyes, crank webs with counterweights, the liners' back halves, a rotary's apex seals,
//! eccentric and plugs) are drawing proportions, not data: the file holds none of them.
//!
//! World frame: the crank along +x, cylinders 1.2 bores apart; a bank's frame is the world turned
//! about x by half the bank angle (bank 0 to −, bank 1 to +), so its cylinder axis is
//! (0, cos t, sin t). A rotary's shaft is along x too, each rotor in the y–z plane.

use engine_core::engine::{CylinderView, EngineDef};
use glam::{Mat3, Mat4, Quat, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Part {
    Block,
    Head,
    Liner,
    Crank,
    /// One throw's crankpin (its webs are [`Part::Web`]); the realistic throw draws all three.
    Throw,
    Web,
    Rod,
    RodEye,
    Piston,
    PistonRing,
    WristPin,
    IntakeValve,
    ExhaustValve,
    /// A valve's stem (its head is the valve itself; the realistic valve draws both).
    IntakeStem,
    ExhaustStem,
    /// A valve's spring and its cam (realistic detail only: they draw nothing at Low).
    IntakeSpring,
    ExhaustSpring,
    IntakeCam,
    ExhaustCam,
    Housing,
    SidePlate,
    Rotor,
    ApexSeal,
    Eccentric,
    Plug,
    Pipe,
    Joint,
    Silencer,
    Catalyst,
    Resonator,
    Runner,
    Trumpet,
    Plenum,
    Throttle,
    Airbox,
    Turbine,
    Compressor,
    Intercooler,
    Charge,
    /// The realistic turbo (`assets/3d-parts/scripts/turbo.py`): its housings, drawn at the
    /// realistic detail instead of the plain drums of [`Part::Turbine`] and [`Part::Compressor`],
    /// and its moving parts, posed each frame ([`turbo_moving`]).
    TurbineHousing,
    CompressorHousing,
    CentreHousing,
    CompressorWheel,
    TurbineWheel,
    WastegateFlap,
}

impl Part {
    /// The valve a stem belongs to (any other part: itself), for its card and handles.
    pub fn as_valve(self) -> Part {
        match self {
            Part::IntakeStem => Part::IntakeValve,
            Part::ExhaustStem => Part::ExhaustValve,
            p => p,
        }
    }

    /// A spring or a cam: drawn at the realistic detail only.
    pub fn is_valve_train(self) -> bool {
        matches!(
            self,
            Part::IntakeSpring | Part::ExhaustSpring | Part::IntakeCam | Part::ExhaustCam
        )
    }

    /// Which unit mesh draws it.
    pub fn mesh(self) -> MeshKind {
        match self {
            Part::Block | Part::Head | Part::Plenum | Part::Airbox | Part::Intercooler => {
                MeshKind::Cuboid
            }
            Part::Liner => MeshKind::Liner,
            Part::Web => MeshKind::Web,
            Part::Housing => MeshKind::HousingRing,
            Part::SidePlate => MeshKind::Housing,
            Part::Rotor => MeshKind::Rotor,
            Part::Joint => MeshKind::Sphere,
            Part::Piston => MeshKind::Piston,
            Part::PistonRing => MeshKind::PistonRing,
            Part::WristPin => MeshKind::WristPin,
            Part::Rod => MeshKind::Rod,
            Part::Throw => MeshKind::Throw,
            Part::ApexSeal => MeshKind::ApexSeal,
            Part::Eccentric => MeshKind::Eccentric,
            Part::Plug => MeshKind::Plug,
            Part::IntakeValve | Part::ExhaustValve => MeshKind::Valve,
            Part::IntakeStem | Part::ExhaustStem => MeshKind::ValveStem,
            Part::IntakeSpring | Part::ExhaustSpring => MeshKind::ValveSpring,
            Part::IntakeCam => MeshKind::IntakeCam,
            Part::ExhaustCam => MeshKind::ExhaustCam,
            Part::RodEye => MeshKind::RodEye,
            Part::Turbine | Part::Compressor => MeshKind::TurboDrum,
            Part::TurbineHousing => MeshKind::TurbineHousing,
            Part::CompressorHousing => MeshKind::CompressorHousing,
            Part::CentreHousing => MeshKind::CentreHousing,
            Part::CompressorWheel => MeshKind::CompressorWheel,
            Part::TurbineWheel => MeshKind::TurbineWheel,
            Part::WastegateFlap => MeshKind::WastegateFlap,
            _ => MeshKind::Cylinder,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MeshKind {
    Cylinder,
    Cuboid,
    Housing,
    HousingRing,
    Rotor,
    Liner,
    Web,
    Sphere,
    Piston,
    PistonRing,
    WristPin,
    Rod,
    RodEye,
    Throw,
    Valve,
    ValveStem,
    ValveSpring,
    IntakeCam,
    ExhaustCam,
    ApexSeal,
    Eccentric,
    Plug,
    TurboDrum,
    TurbineHousing,
    CompressorHousing,
    CentreHousing,
    CompressorWheel,
    TurbineWheel,
    WastegateFlap,
}

impl MeshKind {
    pub const ALL: [MeshKind; 29] = [
        MeshKind::Cylinder,
        MeshKind::Cuboid,
        MeshKind::Housing,
        MeshKind::HousingRing,
        MeshKind::Rotor,
        MeshKind::Liner,
        MeshKind::Web,
        MeshKind::Sphere,
        MeshKind::Piston,
        MeshKind::PistonRing,
        MeshKind::WristPin,
        MeshKind::Rod,
        MeshKind::RodEye,
        MeshKind::Throw,
        MeshKind::Valve,
        MeshKind::ValveStem,
        MeshKind::ValveSpring,
        MeshKind::IntakeCam,
        MeshKind::ExhaustCam,
        MeshKind::ApexSeal,
        MeshKind::Eccentric,
        MeshKind::Plug,
        MeshKind::TurboDrum,
        MeshKind::TurbineHousing,
        MeshKind::CompressorHousing,
        MeshKind::CentreHousing,
        MeshKind::CompressorWheel,
        MeshKind::TurbineWheel,
        MeshKind::WastegateFlap,
    ];
}

/// A placed unit mesh. `color` is sRGB; its fourth value is 1 − the part's own light: 1 for every
/// lit part, down to 0 for a charge burning (drawn at full colour whatever the lighting).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Instance {
    pub part: Part,
    pub model: Mat4,
    pub color: [f32; 4],
}

#[derive(Debug, Clone, PartialEq)]
pub struct Scene {
    pub static_parts: Vec<Instance>,
    /// Centre and radius of a sphere holding every part in every position, pipes included.
    pub bounds: (Vec3, f32),
    /// The same for the engine itself (block, head, crank train): what the camera starts on.
    pub focus: (Vec3, f32),
    /// The exhaust pipes the simulation reports gas for, in its order.
    pub pipes: Vec<super::ducts::PipePath>,
}

/// Cylinder pitch along the crank, bores (a drawing choice: the file has no bore spacing).
pub const PITCH_BORES: f64 = 1.2;

/// The head's height over the deck, bores: up to the valve springs' seats (README: 0.0566 m at
/// 86 mm), where the realistic valve train starts. A drawing choice, the 2D view is unchanged.
pub const HEAD_B: f32 = super::parts::SPRING_SEAT_B;

/// The tallest lift a valve's lobe makes (a VTEC valve's high lobe included): what its cam is
/// built for.
pub fn lobe_lift_m(v: &engine_core::valve::ValveTiming) -> f64 {
    v.high
        .map_or(v.max_lift_m, |h| h.max_lift_m.max(v.max_lift_m))
}

/// The cam's turn from its nose-down pose (on the valve's tip), rad: the flank angle that gives
/// the valve's lift on this lobe, on the side of the lobe's centre `cycle_deg` is (the cam turns
/// at half the crank's speed, so half the crank angle from the lobe's centre); on the base
/// circle it keeps turning with the crank. Phasers and VTEC move the real centre, not this one.
fn cam_turn(v: &engine_core::valve::ValveTiming, lift_m: f64, cycle_deg: f64) -> f32 {
    let centre = v.open_deg + 0.5 * v.duration_deg;
    let d = (cycle_deg - centre + 360.0).rem_euclid(720.0) - 360.0;
    let side = if d < 0.0 { -1.0 } else { 1.0 };
    let top = lobe_lift_m(v).max(1e-9);
    let ratio = (lift_m / top) as f32;
    let flank = super::parts::cam_angle(ratio);
    let nominal = (0.5 * d as f32).to_radians().abs();
    side * if ratio > 1e-4 {
        flank
    } else {
        nominal.max(flank)
    }
}

// One colour per kind of part, as engine cut-away drawings do: cast iron and aluminium greys
// for the structure, the liners teal, the crank train violet, bearings and rings brass.
const STEEL: [f32; 4] = [0.50, 0.53, 0.58, 1.0];
const DARK: [f32; 4] = [0.22, 0.23, 0.27, 1.0];
const LINER: [f32; 4] = [0.20, 0.62, 0.74, 1.0];
const CRANK: [f32; 4] = [0.40, 0.25, 0.58, 1.0];
const ROD: [f32; 4] = [0.56, 0.42, 0.74, 1.0];
const PISTON: [f32; 4] = [0.82, 0.83, 0.86, 1.0];
const BRASS: [f32; 4] = [0.93, 0.72, 0.22, 1.0];
const INTAKE: [f32; 4] = [0.31, 0.59, 1.0, 1.0];
const EXHAUST: [f32; 4] = [1.0, 0.43, 0.31, 1.0];

/// Turn of each bank about the crank, rad.
pub(crate) fn bank_tilt(def: &EngineDef, bank: usize) -> f32 {
    if def.layout.bank_angle_deg <= 0.0 {
        return 0.0;
    }
    let h = (0.5 * def.layout.bank_angle_deg.clamp(0.0, 180.0)).to_radians() as f32;
    if bank == 0 { -h } else { h }
}

/// Which side of its bank's head (+1 = +z, −1 = −z in the bank's frame) a cylinder's exhaust
/// leaves: outboard on a V or flat engine; on an inline engine the back (−z), behind the liners'
/// halves, so the camera's side shows the pistons (and the intake).
pub(crate) fn exhaust_side(def: &EngineDef, bank: usize) -> f32 {
    if bank == 1 && def.layout.bank_angle_deg > 0.0 {
        1.0
    } else {
        -1.0
    }
}

/// A unit cylinder (radius 1, y 0..1) stretched from `a` to `b` with `radius`.
pub(crate) fn rod_between(a: Vec3, b: Vec3, radius: f32) -> Mat4 {
    let d = b - a;
    let len = d.length().max(1e-9);
    let q = Quat::from_rotation_arc(Vec3::Y, d / len);
    Mat4::from_scale_rotation_translation(Vec3::new(radius, len, radius), q, a)
}

/// A unit cube (side 1, centred) stretched from `a` to `b`, `width` along x and `depth` across.
fn bar_between(a: Vec3, b: Vec3, width: f32, depth: f32) -> Mat4 {
    let d = b - a;
    let len = d.length().max(1e-9);
    let q = Quat::from_rotation_arc(Vec3::Y, d / len);
    Mat4::from_scale_rotation_translation(Vec3::new(width, len, depth), q, 0.5 * (a + b))
}

/// A short cylinder along the crank axis, centred on `c`.
fn along_x(c: Vec3, radius: f32, length: f32) -> Mat4 {
    let h = Vec3::new(0.5 * length, 0.0, 0.0);
    rod_between(c - h, c + h, radius)
}

/// Rotary: maps a rotor's drawing plane (the 2D view's x, y) to the world's (z, y), the mesh's
/// extrusion axis to x.
fn rotary_frame() -> Mat3 {
    Mat3::from_cols(Vec3::Z, Vec3::X, Vec3::Y)
}

/// The still parts: per bank the block's back wall (the cut-away layer will open the rest), the
/// liners' back halves and the head; the main shaft; a rotary's housings, side plates and plugs.
pub fn build(def: &EngineDef) -> Scene {
    let g = def.geometry;
    let mut out = Vec::new();
    if let Some(rotor) = g.rotor {
        let (e, r, w) = (
            rotor.eccentricity_m as f32,
            rotor.generating_radius_m as f32,
            rotor.width_m as f32,
        );
        let rotors = def.cylinders.div_ceil(3).max(1);
        for k in 0..rotors {
            let x = k as f32 * 1.5 * w;
            // the rotor housing round the rotor (open through, its bore the epitrochoid) and the
            // side plate behind it; both meshes have generating radius 1 and e/R baked in
            let frame = |x0: f32, width: f32| {
                Mat4::from_translation(Vec3::new(x0, 0.0, 0.0))
                    * Mat4::from_mat3(rotary_frame())
                    * Mat4::from_scale(Vec3::new(r, width, r))
            };
            out.push(Instance {
                part: Part::Housing,
                model: frame(x - 0.5 * w, w),
                color: LINER,
            });
            out.push(Instance {
                part: Part::SidePlate,
                model: frame(x - 0.62 * w, 0.12 * w),
                color: STEEL,
            });
            // two plugs over the waist (the minor axis), leading and trailing
            for dz in [-0.18 * r, 0.18 * r] {
                let base = Vec3::new(x, r - e + 0.2 * r, dz);
                out.push(Instance {
                    part: Part::Plug,
                    model: rod_between(base, base + Vec3::new(0.0, 0.18 * r, 0.0), 0.05 * r),
                    color: BRASS,
                });
            }
        }
        let len = rotors as f32 * 1.5 * w;
        out.push(Instance {
            part: Part::Crank,
            model: rod_between(
                Vec3::new(-w, 0.0, 0.0),
                Vec3::new(len, 0.0, 0.0),
                0.3 * e.max(0.1 * r),
            ),
            color: CRANK,
        });
        let centre = Vec3::new(0.5 * len, 0.0, 0.0);
        let radius = (0.5 * len + w).hypot(1.6 * r) * 1.05;
        let charge_to = super::ducts::intake(def, &mut out);
        let focus = enclosing(&out);
        let mut pipes = Vec::new();
        super::ducts::exhaust(def, charge_to, &mut out, &mut pipes);
        return Scene {
            bounds: (centre, holding(&out, centre, radius)),
            focus,
            pipes,
            static_parts: out,
        };
    }
    let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
    let clear_h = (g.clearance_volume() / g.piston_area()) as f32;
    let deck = r + l + 0.35 * b + clear_h;
    let pitch = PITCH_BORES as f32 * b;
    let place = def.layout.place(def.cylinders);
    let banks = place.iter().map(|p| p.0 + 1).max().unwrap_or(1);
    let cols = place.iter().map(|p| p.1 + 1).max().unwrap_or(1);
    let span = (cols as f32 - 1.0) * pitch;
    let skirt = l - r - 0.25 * b;
    for bank in 0..banks {
        let n = place.iter().filter(|p| p.0 == bank).count();
        let len = (n.max(1) as f32 - 1.0) * pitch + b;
        let x0 = -0.5 * b;
        let rot = Mat4::from_rotation_x(bank_tilt(def, bank));
        // the back wall: behind the liners, from the skirt to the deck
        let wall =
            Mat4::from_translation(Vec3::new(x0 + 0.5 * len, 0.5 * (skirt + deck), -0.7 * b))
                * Mat4::from_scale(Vec3::new(len + 0.2 * b, deck - skirt, 0.1 * b));
        out.push(Instance {
            part: Part::Block,
            model: rot * wall,
            color: STEEL,
        });
        for &(_, pos) in place.iter().filter(|p| p.0 == bank) {
            let x = pos as f32 * pitch;
            out.push(Instance {
                part: Part::Liner,
                model: rot
                    * Mat4::from_translation(Vec3::new(x, skirt, 0.0))
                    * Mat4::from_scale(Vec3::new(0.54 * b, deck - skirt, 0.54 * b)),
                color: LINER,
            });
        }
        // up to the springs' seats, so the realistic valve train stands on it
        let head = Mat4::from_translation(Vec3::new(x0 + 0.5 * len, deck + 0.5 * HEAD_B * b, 0.0))
            * Mat4::from_scale(Vec3::new(len + 0.2 * b, HEAD_B * b, 1.3 * b));
        out.push(Instance {
            part: Part::Head,
            model: rot * head,
            color: DARK,
        });
    }
    out.push(Instance {
        part: Part::Crank,
        model: rod_between(
            Vec3::new(-0.7 * b, 0.0, 0.0),
            Vec3::new(span + 0.7 * b, 0.0, 0.0),
            0.17 * b,
        ),
        color: BRASS,
    });
    // the cams stand 1.37 bore over the deck, their noses 0.33 bore past that
    let reach = deck + (super::parts::STEM_B + super::parts::CAM_BASE_B + 0.11) * b;
    let radius = (0.5 * span + 0.7 * b).hypot(reach.max(1.2 * r + 0.5 * b)) * 1.05;
    let centre = Vec3::new(0.5 * span, 0.0, 0.0);
    let charge_to = super::ducts::intake(def, &mut out);
    let focus = enclosing(&out);
    let mut pipes = Vec::new();
    super::ducts::exhaust(def, charge_to, &mut out, &mut pipes);
    Scene {
        bounds: (centre, holding(&out, centre, radius)),
        focus,
        pipes,
        static_parts: out,
    }
}

/// The cut-away plane for this engine (normal, offset; the side n·p > offset is cut away): an
/// inline engine is opened lengthwise through its cylinders' axes (its front half, toward the
/// camera's start, goes); a V, flat or rotary engine across its last cylinder or rotor, which
/// shows the V or the rotor in section.
pub fn cut_plane(def: &EngineDef) -> (Vec3, f32) {
    let g = def.geometry;
    if let Some(rotor) = g.rotor {
        let rotors = def.cylinders.div_ceil(3).max(1);
        return (Vec3::X, (rotors - 1) as f32 * 1.5 * rotor.width_m as f32);
    }
    if def.layout.bank_angle_deg > 0.0 {
        let cols = def
            .layout
            .place(def.cylinders)
            .iter()
            .map(|p| p.1)
            .max()
            .unwrap_or(0);
        return (Vec3::X, cols as f32 * PITCH_BORES as f32 * g.bore_m as f32);
    }
    (Vec3::Z, 0.0)
}

/// A radius about `centre` of at least `radius` that holds every part's both ends (a unit mesh
/// spans y 0..1 or −½..½ in its own frame: both ends are checked).
fn holding(parts: &[Instance], centre: Vec3, radius: f32) -> f32 {
    parts
        .iter()
        .flat_map(|i| {
            [
                i.model.transform_point3(Vec3::ZERO),
                i.model.transform_point3(Vec3::Y),
            ]
        })
        .map(|p| (p - centre).length() * 1.05)
        .fold(radius, f32::max)
}

/// The sphere round the box that holds every part's both ends: the camera's start, centred on
/// what is drawn rather than on the crank.
fn enclosing(parts: &[Instance]) -> (Vec3, f32) {
    let (lo, hi) = parts
        .iter()
        .flat_map(|i| {
            [
                i.model.transform_point3(Vec3::ZERO),
                i.model.transform_point3(Vec3::Y),
            ]
        })
        .fold(
            (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
            |(lo, hi), p| (lo.min(p), hi.max(p)),
        );
    if lo.x > hi.x {
        return (Vec3::ZERO, 1.0);
    }
    (0.5 * (lo + hi), 0.5 * (hi - lo).length().max(1e-3))
}

/// The moving parts for the cylinders' current state, written into `out` (cleared first; it
/// keeps its capacity, so a steady frame allocates nothing).
/// The turbo's moving parts for its housing placed at `housing` (`ducts::turbo`): both wheels on
/// the shaft turned `wheel_angle` rad about +x, and the wastegate flap opened `flap_rad` about its
/// hinge (+z), out of the turbine's outlet face. Drawn at the realistic detail only.
pub fn turbo_moving(
    housing: Mat4,
    ports: &super::parts::TurboPorts,
    wheel_angle: f32,
    flap_rad: f32,
    out: &mut Vec<Instance>,
) {
    let spin = Mat4::from_rotation_x(wheel_angle);
    let at = |p: Vec3| housing * Mat4::from_translation(p);
    out.push(Instance {
        part: Part::CompressorWheel,
        model: at(Vec3::new(-ports.wheel_x, 0.0, 0.0)) * spin,
        color: [0.80, 0.81, 0.84, 1.0],
    });
    out.push(Instance {
        part: Part::TurbineWheel,
        model: at(Vec3::new(ports.wheel_x, 0.0, 0.0)) * spin,
        color: [0.55, 0.50, 0.45, 1.0],
    });
    out.push(Instance {
        part: Part::WastegateFlap,
        model: at(ports.hinge) * Mat4::from_rotation_z(flap_rad),
        color: [0.40, 0.40, 0.42, 1.0],
    });
}

pub fn moving(def: &EngineDef, cylinders: &[CylinderView], out: &mut Vec<Instance>) {
    out.clear();
    let g = def.geometry;
    if let Some(rotor) = g.rotor {
        let (e, r, w) = (
            rotor.eccentricity_m,
            rotor.generating_radius_m,
            rotor.width_m,
        );
        for (k, chambers) in cylinders.chunks(3).enumerate() {
            let Some(first) = chambers.first() else {
                continue;
            };
            let (c, apex) = crate::view::rotor_pose(first.cycle_deg, e, r);
            let theta = (apex[0].1 - c.1).atan2(apex[0].0 - c.0) as f32;
            let x = k as f32 * 1.5 * w as f32;
            let at2d = |p: (f64, f64)| {
                Vec3::new(x, 0.0, 0.0) + rotary_frame() * Vec3::new(p.0 as f32, 0.0, p.1 as f32)
            };
            let centre = at2d(c);
            let (r, w) = (r as f32, w as f32);
            out.push(Instance {
                part: Part::Rotor,
                model: Mat4::from_translation(centre - Vec3::new(0.45 * w, 0.0, 0.0))
                    * Mat4::from_mat3(rotary_frame())
                    * Mat4::from_rotation_y(-theta)
                    * Mat4::from_scale(Vec3::new(0.97 * r, 0.9 * w, 0.97 * r)),
                color: PISTON,
            });
            // the eccentric the rotor turns on (its frame turned with the lobe, which leads the
            // rotor's centre: the realistic shaft takes it; a cylinder looks the same), and a
            // seal at each apex
            let lobe = c.1.atan2(c.0) as f32;
            out.push(Instance {
                part: Part::Eccentric,
                model: Mat4::from_translation(Vec3::new(x - 0.525 * w, 0.0, 0.0))
                    * Mat4::from_mat3(rotary_frame())
                    * Mat4::from_rotation_y(-lobe)
                    * Mat4::from_translation(Vec3::new(e as f32, 0.0, 0.0))
                    * Mat4::from_scale(Vec3::new(0.35 * r, 1.05 * w, 0.35 * r)),
                color: CRANK,
            });
            for a in apex {
                out.push(Instance {
                    part: Part::ApexSeal,
                    model: Mat4::from_translation(at2d(a))
                        * Mat4::from_scale(Vec3::new(0.95 * w, 0.05 * r, 0.05 * r)),
                    color: BRASS,
                });
            }
        }
        return;
    }
    let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
    let clear_h = (g.clearance_volume() / g.piston_area()) as f32;
    let deck = r + l + 0.35 * b + clear_h;
    let pitch = PITCH_BORES as f32 * b;
    let place = def.layout.place(def.cylinders);
    for (i, c) in cylinders.iter().enumerate() {
        let Some(&(bank, pos)) = place.get(i) else {
            break;
        };
        let rot = Mat4::from_rotation_x(bank_tilt(def, bank));
        let x = pos as f32 * pitch;
        let th = (c.cycle_deg as f32).to_radians();
        let (s, co) = th.sin_cos();
        // in the bank's frame: y up the bore, z across it
        let pin = Vec3::new(x, r * co, r * s);
        let wrist = Vec3::new(x, r + l - c.piston_m as f32, 0.0);
        let crown = wrist.y + 0.35 * b;
        let mut put = |part, m: Mat4, color| {
            out.push(Instance {
                part,
                model: rot * m,
                color,
            })
        };
        // the charge between the crown and the head, coloured by its temperature, glowing
        // while it burns
        let mut gas = super::colormap::temperature(c.temperature_k);
        if c.burning {
            gas[3] = 0.0;
        }
        put(
            Part::Charge,
            Mat4::from_translation(Vec3::new(x, crown, 0.0))
                * Mat4::from_scale(Vec3::new(0.49 * b, (deck - crown).max(1e-4), 0.49 * b)),
            gas,
        );
        put(
            Part::Piston,
            Mat4::from_translation(Vec3::new(x, crown - 0.6 * b, 0.0))
                * Mat4::from_scale(Vec3::new(0.49 * b, 0.6 * b, 0.49 * b)),
            PISTON,
        );
        // three rings in their lands under the crown
        for k in 1..=3 {
            put(
                Part::PistonRing,
                Mat4::from_translation(Vec3::new(x, crown - 0.07 * b * k as f32, 0.0))
                    * Mat4::from_scale(Vec3::new(0.497 * b, 0.025 * b, 0.497 * b)),
                BRASS,
            );
        }
        put(Part::WristPin, along_x(wrist, 0.11 * b, 0.82 * b), BRASS);
        // the rod: its shank, the big end round the pin and the small end round the wrist pin
        put(Part::Rod, bar_between(pin, wrist, 0.14 * b, 0.2 * b), ROD);
        put(Part::RodEye, along_x(pin, 0.27 * b, 0.22 * b), ROD);
        put(Part::RodEye, along_x(wrist, 0.17 * b, 0.2 * b), ROD);
        // the throw: its pin and a web each side, counterweight opposite the pin; the pin's frame
        // turns with the throw (a cylinder looks the same), so the realistic throw can take it
        let web_frame = Mat3::from_cols(Vec3::new(0.0, -s, co), Vec3::X, Vec3::new(0.0, co, s));
        // one throw per crank position: the first bank's (a V's banks share its pin; a flat
        // engine's opposed cylinders stand at one position here, so the other bank's pin is a
        // plain pin, not a second throw through the first)
        if bank == 0 {
            put(
                Part::Throw,
                Mat4::from_translation(pin - Vec3::new(0.13 * b, 0.0, 0.0))
                    * Mat4::from_mat3(web_frame)
                    * Mat4::from_scale(Vec3::new(0.13 * b, 0.26 * b, 0.13 * b)),
                CRANK,
            );
        } else {
            put(Part::Crank, along_x(pin, 0.13 * b, 0.26 * b), BRASS);
        }
        for x0 in [x - 0.13 * b - 0.12 * b, x + 0.13 * b] {
            put(
                Part::Web,
                Mat4::from_translation(Vec3::new(x0, 0.0, 0.0))
                    * Mat4::from_mat3(web_frame)
                    * Mat4::from_scale(Vec3::new(r, 0.12 * b, r)),
                CRANK,
            );
        }
        let side = exhaust_side(def, bank);
        for (dz, lift, part, stem, spring, camp, timing, color) in [
            (
                -0.25 * side,
                c.intake_lift_m,
                Part::IntakeValve,
                Part::IntakeStem,
                Part::IntakeSpring,
                Part::IntakeCam,
                &def.intake_valve,
                INTAKE,
            ),
            (
                0.25 * side,
                c.exhaust_lift_m,
                Part::ExhaustValve,
                Part::ExhaustStem,
                Part::ExhaustSpring,
                Part::ExhaustCam,
                &def.exhaust_valve,
                EXHAUST,
            ),
        ] {
            let face = deck - lift as f32;
            let z = dz * b;
            // the head (disc) and its stem up through the head
            put(
                part,
                Mat4::from_translation(Vec3::new(x, face, z))
                    * Mat4::from_scale(Vec3::new(0.17 * b, 0.04 * b, 0.17 * b)),
                color,
            );
            put(
                stem,
                Mat4::from_translation(Vec3::new(x, face, z))
                    * Mat4::from_scale(Vec3::new(0.03 * b, HEAD_B * b, 0.03 * b)),
                color,
            );
            // the realistic valve train: the spring on its seat in the head, closing up with the
            // lift; the cam on the closed tip, turning at half the crank's speed
            put(
                spring,
                Mat4::from_translation(Vec3::new(x, deck + HEAD_B * b, z))
                    * Mat4::from_scale(Vec3::new(
                        b,
                        b * super::parts::spring_squash(lift as f32 / b),
                        b,
                    )),
                STEEL,
            );
            let tip = deck + super::parts::STEM_B * b;
            put(
                camp,
                Mat4::from_translation(Vec3::new(x, tip + super::parts::CAM_BASE_B * b, z))
                    * Mat4::from_rotation_x(
                        std::f32::consts::PI + cam_turn(timing, lift, c.cycle_deg),
                    )
                    * Mat4::from_scale(Vec3::splat(b)),
                color,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn views(def: &EngineDef, cycle_deg: impl Fn(usize) -> f64) -> Vec<CylinderView> {
        (0..def.cylinders)
            .map(|i| {
                let d = cycle_deg(i);
                CylinderView {
                    cycle_deg: d,
                    piston_m: def.geometry.piston_position(d.to_radians()),
                    pressure_pa: 1e5,
                    temperature_k: 300.0,
                    intake_lift_m: 0.0,
                    exhaust_lift_m: 0.0,
                    burning: false,
                }
            })
            .collect()
    }

    fn of(out: &[Instance], part: Part) -> Vec<Mat4> {
        out.iter()
            .filter(|i| i.part == part)
            .map(|i| i.model)
            .collect()
    }

    fn deck_of(d: &EngineDef) -> f32 {
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        r + l + 0.35 * b + (g.clearance_volume() / g.piston_area()) as f32
    }

    #[test]
    fn the_head_reaches_the_springs_seats() {
        let d = EngineDef::inline4();
        let b = d.geometry.bore_m as f32;
        let s = build(&d);
        let head = s
            .static_parts
            .iter()
            .find(|i| i.part == Part::Head)
            .unwrap()
            .model;
        // the box spans y -0.5..0.5 of its unit: its top is the seat, 0.0566 m at 86 mm = 0.658 bore
        let top = head.transform_point3(Vec3::new(0.0, 0.5, 0.0)).y;
        assert!(
            (top - (deck_of(&d) + 0.0566 / 0.086 * b)).abs() < 1e-4 * b,
            "{top}"
        );
        // its bottom stays on the deck
        let low = head.transform_point3(Vec3::new(0.0, -0.5, 0.0)).y;
        assert!((low - deck_of(&d)).abs() < 1e-6, "{low}");
        // and the springs stand on that top, the cams over the valve tips (1.15 bore) by their
        // base circle (0.22 bore)
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        for part in [Part::IntakeSpring, Part::ExhaustSpring] {
            for m in of(&out, part) {
                assert!((m.w_axis.y - top).abs() < 1e-6);
            }
        }
        for part in [Part::IntakeCam, Part::ExhaustCam] {
            for m in of(&out, part) {
                let want = deck_of(&d) + (1.15 + 0.22) * b;
                assert!((m.w_axis.y - want).abs() < 2e-3 * b, "{}", m.w_axis.y);
            }
        }
        // the stems reach the head's top
        for m in of(&out, Part::IntakeStem) {
            assert!((m.y_axis.y - 0.0566 / 0.086 * b).abs() < 1e-4 * b);
        }
    }

    #[test]
    fn the_spring_is_compressed_by_the_lift() {
        let d = EngineDef::inline4();
        let b = d.geometry.bore_m as f32;
        let mut v = views(&d, |_| 0.0);
        v[0].intake_lift_m = 0.00946 * b as f64 / 0.086;
        let mut out = Vec::new();
        moving(&d, &v, &mut out);
        // hand: (0.0378 − 0.00946)/0.0378 = 0.74974 of the free height, other springs free
        let springs = of(&out, Part::IntakeSpring);
        assert!((springs[0].y_axis.y / b - 0.74974).abs() < 1e-3);
        assert!((springs[1].y_axis.y / b - 1.0).abs() < 1e-6);
    }

    #[test]
    fn the_cam_rides_the_tip_of_the_valve_it_opens() {
        // at any point of the lift the lowest point of the (turned) lobe is the valve's tip:
        // closed tip 1.15 bore over the face, less the lift
        let d = EngineDef::inline4();
        let b = d.geometry.bore_m as f32;
        let lobe = |intake: bool| {
            let v = if intake {
                &d.intake_valve
            } else {
                &d.exhaust_valve
            };
            super::super::parts::cam((lobe_lift_m(v) / b as f64) as f32).unwrap()
        };
        let centre = d.intake_valve.open_deg + 0.5 * d.intake_valve.duration_deg;
        for (cycle, ratio) in [
            (centre, 1.0),
            (centre - 50.0, 0.6),
            (centre + 50.0, 0.6),
            (centre - 75.0, 0.2),
        ] {
            let mut v = views(&d, |_| cycle);
            let lift = ratio * lobe_lift_m(&d.intake_valve);
            v[0].intake_lift_m = lift;
            let mut out = Vec::new();
            moving(&d, &v, &mut out);
            let m = of(&out, Part::IntakeCam)[0];
            let low = lobe(true)
                .vertices
                .iter()
                .map(|p| m.transform_point3(Vec3::from(p.pos) * Vec3::splat(1.0)).y)
                .fold(f32::MAX, f32::min);
            let tip = deck_of(&d) + 1.15 * b - lift as f32;
            assert!((low - tip).abs() < 0.004 * b, "{cycle}: {low} vs {tip}");
        }
    }

    #[test]
    fn the_cam_turns_at_half_the_crank_speed() {
        let d = EngineDef::inline4();
        // on the base circle the lobe turns half the crank angle: 40° of crank, 20° of cam
        let angle = |cycle: f64| {
            let mut out = Vec::new();
            moving(&d, &views(&d, |_| cycle), &mut out);
            let m = of(&out, Part::IntakeCam)[0];
            m.y_axis.z.atan2(m.y_axis.y)
        };
        let far = d.intake_valve.open_deg + 0.5 * d.intake_valve.duration_deg + 200.0;
        let turn = (angle(far + 40.0) - angle(far)).to_degrees();
        assert!((turn - 20.0).abs() < 0.5, "{turn}");
    }

    #[test]
    fn inline4_cylinders_are_1_2_bores_apart() {
        let d = EngineDef::inline4();
        let b = d.geometry.bore_m as f32;
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        let xs: Vec<f32> = of(&out, Part::Piston).iter().map(|m| m.w_axis.x).collect();
        assert_eq!(xs.len(), 4);
        for (k, x) in xs.iter().enumerate() {
            assert!((x - 1.2 * b * k as f32).abs() < 1e-6, "{xs:?}");
        }
    }

    #[test]
    fn a_piston_at_tdc_has_its_crown_r_plus_l_plus_0_35_bore_over_the_crank() {
        // the piston mesh's base sits 0.6 b under the crown: base y = r + l + 0.35b − 0.6b
        let d = EngineDef::inline4();
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        let y = of(&out, Part::Piston)[0].w_axis.y;
        assert!((y - (r + l - 0.25 * b)).abs() < 1e-6, "{y}");
        // at 180° it is a stroke lower
        moving(&d, &views(&d, |_| 180.0), &mut out);
        let y2 = of(&out, Part::Piston)[0].w_axis.y;
        assert!((y - y2 - 2.0 * r).abs() < 1e-6, "{y} {y2}");
    }

    #[test]
    fn v8_banks_lean_45_degrees() {
        // 90° V: bank 0's axis is (0, cos 45°, −sin 45°); its piston base at TDC lies on it
        let d = EngineDef::crossplane_v8();
        assert_eq!(d.layout.bank_angle_deg, 90.0);
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        let place = d.layout.place(d.cylinders);
        let pistons = of(&out, Part::Piston);
        let h = r + l - 0.25 * b;
        let s = std::f32::consts::FRAC_1_SQRT_2;
        for (k, m) in pistons.iter().enumerate() {
            let sign = if place[k].0 == 0 { -1.0 } else { 1.0 };
            assert!((m.w_axis.y - h * s).abs() < 1e-6, "{k}: {}", m.w_axis);
            assert!(
                (m.w_axis.z - sign * h * s).abs() < 1e-6,
                "{k}: {}",
                m.w_axis
            );
        }
    }

    #[test]
    fn a_rod_runs_from_its_pin_to_its_wrist() {
        // at 90° the pin is r across the bore (z = r) at the crank's height and the wrist pin
        // √(l² − r²) up the bore: the shank's centre is halfway
        let d = EngineDef::inline4();
        let (r, l) = (d.geometry.crank_radius() as f32, d.geometry.rod_m as f32);
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 90.0), &mut out);
        let rod = of(&out, Part::Rod)[0];
        let mid = rod.w_axis.truncate();
        let want = Vec3::new(0.0, 0.5 * (l * l - r * r).sqrt(), 0.5 * r);
        assert!((mid - want).length() < 1e-6, "{mid} vs {want}");
        // and the big end sits on the pin
        let eye = of(&out, Part::RodEye)[0].w_axis.truncate();
        assert!(
            (eye - Vec3::new(-0.11 * d.geometry.bore_m as f32, 0.0, r)).length() < 1e-6,
            "{eye}"
        );
    }

    #[test]
    fn a_throws_webs_turn_with_its_pin() {
        // the web's outline puts the pin at (0, 1) in its plane: in the bank's frame that is the
        // pin direction (0, cos θ, sin θ) at the throw radius
        let d = EngineDef::inline4();
        let r = d.geometry.crank_radius() as f32;
        let mut out = Vec::new();
        for deg in [0.0f32, 90.0, 210.0] {
            moving(&d, &views(&d, |_| deg as f64), &mut out);
            let web = of(&out, Part::Web)[0];
            let p = web.transform_point3(Vec3::new(0.0, 0.0, 1.0));
            let (s, c) = deg.to_radians().sin_cos();
            assert!(
                (p.y - r * c).abs() < 1e-6 && (p.z - r * s).abs() < 1e-6,
                "{deg}: {p}"
            );
        }
    }

    #[test]
    fn every_piston_has_three_rings_under_its_crown() {
        let d = EngineDef::inline4();
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        let rings = of(&out, Part::PistonRing);
        assert_eq!(rings.len(), 12);
        // at TDC the crown is r + l + 0.35 b; the top ring 0.07 b under it
        assert!((rings[0].w_axis.y - (r + l + 0.35 * b - 0.07 * b)).abs() < 1e-6);
    }

    #[test]
    fn bounds_cover_every_part() {
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            EngineDef::mazda_renesis(),
            EngineDef::crossplane_v8(),
        ] {
            let s = build(&d);
            let mut out = Vec::new();
            for deg in [0.0, 90.0, 180.0, 270.0, 540.0] {
                moving(&d, &views(&d, |_| deg), &mut out);
                for i in s.static_parts.iter().chain(&out) {
                    let p = i.model.w_axis.truncate();
                    assert!(
                        (p - s.bounds.0).length() <= s.bounds.1,
                        "{}: {p} out of {:?}",
                        d.name,
                        s.bounds
                    );
                }
            }
        }
    }

    #[test]
    fn rotary_has_housing_and_rotor_not_pistons() {
        let d = EngineDef::mazda_renesis();
        let s = build(&d);
        let mut out = Vec::new();
        moving(&d, &views(&d, |i| 240.0 * i as f64), &mut out);
        assert_eq!(of(&s.static_parts, Part::Housing).len(), 2);
        assert_eq!(of(&s.static_parts, Part::SidePlate).len(), 2);
        assert_eq!(of(&out, Part::ApexSeal).len(), 6);
        assert_eq!(of(&out, Part::Rotor).len(), 2);
        assert!(of(&out, Part::Piston).is_empty());
    }

    #[test]
    fn a_rotor_centre_rides_its_eccentric() {
        // the rotor's centre is e from the shaft (the y–z plane), wherever the shaft is
        let d = EngineDef::mazda_renesis();
        let e = d.geometry.rotor.unwrap().eccentricity_m as f32;
        let mut out = Vec::new();
        for deg in [0.0, 100.0, 333.0] {
            moving(&d, &views(&d, |_| deg), &mut out);
            let c = of(&out, Part::Rotor)[0].w_axis;
            assert!((c.y.hypot(c.z) - e).abs() < 1e-6, "{deg}: {c}");
        }
    }

    #[test]
    fn the_turbos_wheels_turn_on_its_shaft_and_its_flap_swings_out() {
        // a housing placed at t, at twice its size; the wheels a quarter turn round, the flap 0.3
        // rad open
        let ports = super::super::parts::turbo_ports().unwrap();
        let t = Vec3::new(1.0, 2.0, 3.0);
        let housing = Mat4::from_scale_rotation_translation(Vec3::splat(2.0), Quat::IDENTITY, t);
        let mut out = Vec::new();
        turbo_moving(housing, &ports, std::f32::consts::FRAC_PI_2, 0.3, &mut out);
        let model = |p: Part| out.iter().find(|i| i.part == p).unwrap().model;
        // a quarter turn about +x takes a blade tip at +y to +z, on each wheel's back face
        let tip = Vec3::new(0.0, 0.028, 0.0);
        let want = |x: f32| t + 2.0 * Vec3::new(x, 0.0, 0.028);
        assert!(
            (model(Part::CompressorWheel).transform_point3(tip) - want(-ports.wheel_x)).length()
                < 1e-5
        );
        assert!(
            (model(Part::TurbineWheel).transform_point3(tip) - want(ports.wheel_x)).length() < 1e-5
        );
        // the flap hangs from its hinge (its disc 0.0092 m under it at the reference): opened
        // about +z by 0.3 rad it comes out along +x by 0.0092·sin 0.3
        let disc = Vec3::new(0.0, -0.0092, 0.0);
        let at = model(Part::WastegateFlap).transform_point3(disc);
        let hinge = t + 2.0 * ports.hinge;
        let expect = hinge + 2.0 * Vec3::new(0.0092 * 0.3f32.sin(), -0.0092 * 0.3f32.cos(), 0.0);
        assert!((at - expect).length() < 1e-5, "{at} vs {expect}");
        assert_eq!(out.len(), 3);
    }

    #[test]
    fn moving_does_not_reallocate() {
        // a buffer with room keeps it: same capacity, same storage (a fresh or shrunk Vec
        // would not keep 1000)
        let d = EngineDef::lfa_v10();
        let mut out = Vec::with_capacity(1000);
        let ptr = out.as_ptr();
        for deg in [0.0, 90.0] {
            moving(&d, &views(&d, |_| deg), &mut out);
            assert!(!out.is_empty());
            assert_eq!((out.capacity(), out.as_ptr()), (1000, ptr));
        }
    }

    #[test]
    fn the_camera_starts_on_the_engine_and_its_intake() {
        // centred on what is drawn above and below the crank, not on the crank, and holding it
        for d in [
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            EngineDef::mazda_renesis(),
        ] {
            let s = build(&d);
            let (c, r) = s.focus;
            assert!(
                c.y > 0.0,
                "{}: the intake rises over the crank: {c}",
                d.name
            );
            let plenum_or_trumpets = s
                .static_parts
                .iter()
                .filter(|i| matches!(i.part, Part::Plenum | Part::Trumpet | Part::Runner));
            for i in plenum_or_trumpets {
                let p = i.model.transform_point3(Vec3::ZERO);
                assert!(
                    (p - c).length() <= r + 1e-5,
                    "{}: {p} out of {c} r {r}",
                    d.name
                );
            }
        }
    }

    #[test]
    fn bounds_cover_the_pipes() {
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::honda_k20c1(),
            EngineDef::subaru_ej25(),
            EngineDef::lfa_v10(),
            EngineDef::crossplane_v8(),
            EngineDef::mazda_renesis(),
        ] {
            let s = build(&d);
            for i in &s.static_parts {
                for p in [
                    i.model.transform_point3(Vec3::ZERO),
                    i.model.transform_point3(Vec3::Y),
                ] {
                    assert!((p - s.bounds.0).length() <= s.bounds.1, "{}: {p}", d.name);
                }
            }
        }
    }

    #[test]
    fn the_charge_fills_the_chamber_and_glows_while_it_burns() {
        let d = EngineDef::inline4();
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        let clear = (g.clearance_volume() / g.piston_area()) as f32;
        let mut v = views(&d, |_| 0.0);
        v[0].temperature_k = 2000.0;
        v[0].burning = true;
        let mut out = Vec::new();
        moving(&d, &v, &mut out);
        let charges: Vec<_> = out.iter().filter(|i| i.part == Part::Charge).collect();
        assert_eq!(charges.len(), 4);
        // at TDC: from the crown (r + l + 0.35 b) up the clearance height to the deck
        let (s, _, t) = charges[0].model.to_scale_rotation_translation();
        assert!((t.y - (r + l + 0.35 * b)).abs() < 1e-6, "{t}");
        assert!((s.y - clear).abs() < 1e-6, "{s}");
        assert_eq!(charges[0].color[3], 0.0, "burning: its own light");
        assert_eq!(charges[1].color[3], 1.0);
        assert_eq!(
            charges[0].color[..3],
            super::super::colormap::temperature(2000.0)[..3]
        );
        // at BDC the charge is a stroke taller
        moving(&d, &views(&d, |_| 180.0), &mut out);
        let c = out.iter().find(|i| i.part == Part::Charge).unwrap();
        let (s2, _, _) = c.model.to_scale_rotation_translation();
        assert!((s2.y - clear - 2.0 * r).abs() < 1e-5, "{s2}");
    }

    #[test]
    fn the_cut_goes_through_the_cylinders() {
        // inline: the plane of the axes (z = 0); a V: across its last pair of cylinders, 3
        // pitches along for the cross-plane (4 per bank); a rotary: across its second rotor
        assert_eq!(cut_plane(&EngineDef::inline4()), (Vec3::Z, 0.0));
        let v8 = EngineDef::crossplane_v8();
        let x = 3.0 * PITCH_BORES as f32 * v8.geometry.bore_m as f32;
        let (n, d) = cut_plane(&v8);
        assert_eq!(n, Vec3::X);
        assert!((d - x).abs() < 1e-6);
        let rx = EngineDef::mazda_renesis();
        let w = rx.geometry.rotor.unwrap().width_m as f32;
        let (n, d) = cut_plane(&rx);
        assert_eq!(n, Vec3::X);
        assert!((d - 1.5 * w).abs() < 1e-6);
    }
}
