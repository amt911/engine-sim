//! The intake and exhaust drawn from the definition: every pipe at its defined length and bore,
//! laid as compact runs ([`super::routing`]) — the sim's pipes are 1D, so their path here is a
//! drawing choice, never the car's routing.
//!
//! Exhaust: each primary leaves its port on the head's exhaust side, steps out and runs down
//! beside the block; each collector starts where its primaries end; the after-collector chain
//! (pipes, catalysts, silencers, resonators) runs behind the engine, then the bank merge and the
//! tails.

use super::routing::{Run, route};
use super::scene::{Instance, PITCH_BORES, Part, bank_tilt, exhaust_side, rod_between};
use engine_core::engine::EngineDef;
use engine_core::exhaust::Segment;
use glam::{Mat4, Vec3};

const PIPE: [f32; 4] = [0.62, 0.55, 0.50, 1.0];
const CATALYST: [f32; 4] = [0.36, 0.33, 0.31, 1.0];
const SILENCER: [f32; 4] = [0.72, 0.72, 0.74, 1.0];

/// Lane spacing of a folded pipe of bore `d`.
fn lane(d: f32) -> f32 {
    1.5 * d
}

/// Draw `runs` as a tube of `radius`, with a ball at every bend.
fn tube(runs: &[Run], radius: f32, part: Part, color: [f32; 4], out: &mut Vec<Instance>) {
    for (k, r) in runs.iter().enumerate() {
        out.push(Instance {
            part,
            model: rod_between(r.from, r.to, radius),
            color,
        });
        if k > 0 {
            out.push(Instance {
                part: Part::Joint,
                model: Mat4::from_scale_rotation_translation(
                    Vec3::splat(radius),
                    glam::Quat::IDENTITY,
                    r.from,
                ),
                color,
            });
        }
    }
}

/// Lay the after-collector `chain` from `start`, running straight back along +x; returns the
/// end and the bore of its last piece.
fn chain(segments: &[Segment], start: Vec3, out: &mut Vec<Instance>) -> (Vec3, f32) {
    let mut at = start;
    let mut bore = 0.05f32;
    let mut runs = Vec::new();
    for s in segments {
        runs.clear();
        let (part, color, len, d) = match *s {
            Segment::Pipe {
                length_m,
                diameter_m,
            } => (Part::Pipe, PIPE, length_m, diameter_m),
            Segment::Muffler {
                length_m,
                diameter_m,
                ..
            } => (Part::Silencer, SILENCER, length_m, diameter_m),
            Segment::Catalyst {
                length_m,
                diameter_m,
                ..
            } => (Part::Catalyst, CATALYST, length_m, diameter_m),
            Segment::Measured {
                length_m, bore_m, ..
            } => (Part::Pipe, PIPE, length_m, bore_m),
            Segment::Resonator {
                neck_diameter_m,
                neck_length_m,
                volume_m3,
            } => {
                // a side branch: its neck sideways off the line, its volume a ball at the end
                let (dn, ln) = (neck_diameter_m as f32, neck_length_m as f32);
                let end = at + Vec3::new(0.0, 0.0, ln.max(0.0));
                out.push(Instance {
                    part: Part::Resonator,
                    model: rod_between(at, end, 0.5 * dn),
                    color: SILENCER,
                });
                let r = (3.0 * volume_m3.max(0.0) as f32 / (4.0 * std::f32::consts::PI)).cbrt();
                out.push(Instance {
                    part: Part::Joint,
                    model: Mat4::from_scale_rotation_translation(
                        Vec3::splat(r),
                        glam::Quat::IDENTITY,
                        end + Vec3::new(0.0, 0.0, r),
                    ),
                    color: SILENCER,
                });
                continue;
            }
        };
        let (len, d) = (len as f32, d as f32);
        // cans and catalysts stay straight; pipes fold in lanes of 0.8 m
        let max_run = if part == Part::Pipe {
            0.8
        } else {
            len.max(1e-3)
        };
        at = route(
            at,
            Vec3::X,
            Vec3::NEG_Y,
            lane(d.max(bore)),
            len,
            max_run,
            &mut runs,
        );
        tube(&runs, 0.5 * d, part, color, out);
        bore = d;
    }
    (at, bore)
}

/// The exhaust's instances, appended to `out`; returns the tail ends (for the scene's bounds).
pub fn exhaust(def: &EngineDef, out: &mut Vec<Instance>) -> Vec<Vec3> {
    let g = def.geometry;
    let ex = &def.exhaust;
    let n = def.cylinders;
    let b = g.bore_m as f32;
    let (r, l) = (g.crank_radius() as f32, g.rod_m as f32);
    let deck = r + l + 0.35 * b + (g.clearance_volume() / g.piston_area()) as f32;
    let pitch = PITCH_BORES as f32 * b;
    let place = def.layout.place(n);
    let dp = ex.primary_diameter_m as f32;
    let mut runs = Vec::new();
    // each primary's end, by cylinder
    let mut ends = vec![Vec3::ZERO; n];
    for (i, end) in ends.iter_mut().enumerate() {
        runs.clear();
        let start = if let Some(rotor) = g.rotor {
            // a rotary's exhaust port is in its housing's periphery, below the waist
            let k = i / 3;
            let x = k as f32 * 1.5 * rotor.width_m as f32;
            let reach = (rotor.generating_radius_m + rotor.eccentricity_m) as f32;
            Vec3::new(x, -0.3 * reach, reach)
        } else {
            let (bank, pos) = place.get(i).copied().unwrap_or((0, i));
            let side = exhaust_side(def, bank);
            Mat4::from_rotation_x(bank_tilt(def, bank)).transform_point3(Vec3::new(
                pos as f32 * pitch,
                deck + 0.175 * b,
                side * 0.65 * b,
            ))
        };
        let out_dir = if start.z >= 0.0 { Vec3::Z } else { Vec3::NEG_Z };
        let len = ex.primary_length(i) as f32;
        // a short stub out of the port, then down beside the block, folding outwards
        let stub = route(
            start,
            out_dir,
            Vec3::NEG_Y,
            lane(dp),
            len.min(0.6 * b),
            0.6 * b,
            &mut runs,
        );
        let rest = (len - 0.6 * b).max(0.0);
        let drop = (deck + r + 0.8 * b).max(0.1);
        *end = route(stub, Vec3::NEG_Y, out_dir, lane(dp), rest, drop, &mut runs);
        tube(&runs, 0.5 * dp, Part::Pipe, PIPE, out);
    }
    // collectors: a group's members as the sim joins them; a cylinder no group lists still
    // draws its primary (above) and joins the last group here
    let mut groups = ex.groups(n);
    for c in 0..n {
        if !groups.iter().any(|g| g.contains(&c))
            && let Some(last) = groups.last_mut()
        {
            last.push(c);
        }
    }
    let dc = ex.collector_diameter_m as f32;
    let mut branch_ends = Vec::new();
    for (k, members) in groups.iter().enumerate() {
        runs.clear();
        let start = members.iter().map(|&c| ends[c]).sum::<Vec3>() / members.len().max(1) as f32;
        let len = ex.collector_length(k) as f32;
        let end = route(start, Vec3::NEG_Y, Vec3::X, lane(dc), len, 0.4, &mut runs);
        tube(&runs, 0.5 * dc, Part::Pipe, PIPE, out);
        branch_ends.push(end);
    }
    // the chain behind the collectors: per branch, then the merge's own, then the tails
    let mut tails_from = Vec::new();
    for &end in &branch_ends {
        let (at, bore) = chain(&ex.after_collector, end, out);
        tails_from.push((at, bore));
    }
    if let Some(m) = &ex.merge
        && !tails_from.is_empty()
    {
        let at = tails_from.iter().map(|t| t.0).sum::<Vec3>() / tails_from.len() as f32;
        let (end, bore) = chain(&m.segments, at, out);
        tails_from = vec![(end, bore.max(m.outlet_diameter_m as f32))];
    }
    let mut tail_ends = Vec::new();
    for (at, bore) in tails_from {
        let legacy = ex.after_collector.is_empty();
        let (len, d) = if legacy {
            (ex.tail_length_m as f32, ex.tail_diameter_m as f32)
        } else {
            (0.15, bore)
        };
        let outlets = ex.tail_outlets.max(1);
        let spacing = ex.tail_spacing_m as f32;
        for o in 0..outlets {
            runs.clear();
            let dz = (o as f32 - 0.5 * (outlets - 1) as f32) * spacing;
            let start = at + Vec3::new(0.0, 0.0, dz);
            let end = route(start, Vec3::X, Vec3::NEG_Y, lane(d), len, 0.8, &mut runs);
            tube(&runs, 0.5 * d, Part::Pipe, PIPE, out);
            tail_ends.push(end);
        }
    }
    tail_ends
}

const DUCT: [f32; 4] = [0.55, 0.62, 0.70, 1.0];
const AIRBOX: [f32; 4] = [0.16, 0.16, 0.18, 1.0];
const THROTTLE: [f32; 4] = [0.78, 0.79, 0.82, 1.0];

/// A box of `size` (x, y, z) centred on `c`.
fn boxed(c: Vec3, size: Vec3) -> Mat4 {
    Mat4::from_scale_rotation_translation(size, glam::Quat::IDENTITY, c)
}

/// The intake's instances, appended to `out`: a runner from each port (the head's other side
/// from the exhaust, so into a V's valley), rising to a plenum box of the defined volume that
/// spans the engine, its throttle body at one end, and the air box with its inlet ducts; with
/// individual throttles, no plenum: a trumpet on each runner.
pub fn intake(def: &EngineDef, out: &mut Vec<Instance>) {
    let g = def.geometry;
    let it = &def.intake;
    let n = def.cylinders;
    let b = g.bore_m as f32;
    let (r, l) = (g.crank_radius() as f32, g.rod_m as f32);
    let deck = r + l + 0.35 * b + (g.clearance_volume() / g.piston_area()) as f32;
    let pitch = PITCH_BORES as f32 * b;
    let place = def.layout.place(n);
    let dr = it.runner_diameter_m as f32;
    let len = it.runner_length_m as f32;
    let mut runs = Vec::new();
    let mut ends = Vec::with_capacity(n);
    for i in 0..n {
        runs.clear();
        let start = if let Some(rotor) = g.rotor {
            // a rotary's intake ports are in its side housings: the intake leaves the far side
            let x = (i / 3) as f32 * 1.5 * rotor.width_m as f32;
            let reach = (rotor.generating_radius_m + rotor.eccentricity_m) as f32;
            Vec3::new(x, -0.3 * reach, -reach)
        } else {
            let (bank, pos) = place.get(i).copied().unwrap_or((0, i));
            let side = -exhaust_side(def, bank);
            Mat4::from_rotation_x(bank_tilt(def, bank)).transform_point3(Vec3::new(
                pos as f32 * pitch,
                deck + 0.175 * b,
                side * 0.65 * b,
            ))
        };
        let out_dir = if start.z >= 0.0 { Vec3::Z } else { Vec3::NEG_Z };
        // out of the port, then up, folding outwards
        let stub = route(
            start,
            out_dir,
            Vec3::Y,
            lane(dr),
            len.min(0.5 * b),
            0.5 * b,
            &mut runs,
        );
        let end = route(
            stub,
            Vec3::Y,
            out_dir,
            lane(dr),
            (len - 0.5 * b).max(0.0),
            0.35,
            &mut runs,
        );
        tube(&runs, 0.5 * dr, Part::Runner, DUCT, out);
        if it.individual_throttles {
            // a trumpet on each runner: flared to 1.4 bores at its mouth
            out.push(Instance {
                part: Part::Trumpet,
                model: rod_between(end, end + 0.6 * dr * Vec3::Y, 0.7 * dr),
                color: THROTTLE,
            });
        }
        ends.push(end);
    }
    if it.individual_throttles || ends.is_empty() {
        return;
    }
    // the plenum: as long as the runners' spread along the crank (at least a bore), its other
    // two sides equal, holding the defined volume, just above the runners' ends
    let lo = ends.iter().map(|e| e.x).fold(f32::MAX, f32::min);
    let hi = ends.iter().map(|e| e.x).fold(f32::MIN, f32::max);
    let lx = (hi - lo + 2.0 * dr).max(b);
    let v = def.plenum_volume_m3.max(1e-6) as f32;
    let side = (v / lx).sqrt();
    let mean = ends.iter().sum::<Vec3>() / ends.len() as f32;
    let centre = Vec3::new(0.5 * (lo + hi), mean.y + 0.5 * side, mean.z);
    out.push(Instance {
        part: Part::Plenum,
        model: boxed(centre, Vec3::new(lx, side, side)),
        color: DUCT,
    });
    // the throttle body on the plenum's front end, its bore the throttle's
    let dt = def.throttle_diameter_m as f32;
    let tb = Vec3::new(lo - 0.5 * (lx - (hi - lo)), centre.y, centre.z);
    out.push(Instance {
        part: Part::Throttle,
        model: rod_between(tb, tb - Vec3::new(1.5 * dt, 0.0, 0.0), 0.5 * dt + 0.006),
        color: THROTTLE,
    });
    // the air box ahead of the throttle, holding its volume (a cube), with its inlet ducts
    if let Some(ab) = &it.airbox {
        let a = (ab.volume_m3.max(1e-6) as f32).cbrt();
        let c = tb - Vec3::new(1.5 * dt + 0.5 * a, 0.0, 0.0);
        out.push(Instance {
            part: Part::Airbox,
            model: boxed(c, Vec3::splat(a)),
            color: AIRBOX,
        });
        let dd = ab.duct_diameter_m as f32;
        for k in 0..ab.ducts {
            runs.clear();
            let dz = (k as f32 - 0.5 * (ab.ducts as f32 - 1.0)) * 1.5 * dd;
            let start = c - Vec3::new(0.5 * a, 0.0, -dz);
            route(
                start,
                Vec3::NEG_X,
                Vec3::Y,
                lane(dd),
                ab.duct_length_m as f32,
                0.5,
                &mut runs,
            );
            tube(&runs, 0.5 * dd, Part::Runner, AIRBOX, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Runs of instances drawn as `part` tubes (the ball joints left out), as (from, to).
    fn tubes(out: &[Instance], part: Part) -> Vec<(Vec3, Vec3)> {
        out.iter()
            .filter(|i| i.part == part)
            .map(|i| {
                (
                    i.model.transform_point3(Vec3::ZERO),
                    i.model.transform_point3(Vec3::Y),
                )
            })
            .collect()
    }

    /// Where each primary ends: the primaries are laid first, cylinder by cylinder, so walking
    /// the pipe runs and cutting at each primary's length finds them.
    fn primary_ends(d: &EngineDef, pipes: &[(Vec3, Vec3)]) -> Vec<Vec3> {
        let mut ends = Vec::new();
        let mut acc = 0.0;
        for (a, b) in pipes {
            acc += (*b - *a).length();
            if (acc - d.exhaust.primary_length(ends.len()) as f32).abs() < 1e-4 {
                ends.push(*b);
                acc = 0.0;
                if ends.len() == d.cylinders {
                    break;
                }
            }
        }
        ends
    }

    fn length(t: &[(Vec3, Vec3)]) -> f32 {
        t.iter().map(|(a, b)| (*b - *a).length()).sum()
    }

    #[test]
    fn each_primary_is_its_length_long() {
        // only the primaries: an engine with no collector length and no chain
        let mut d = EngineDef::inline4();
        d.exhaust.primary_length_deviation_m = vec![0.0, 0.02, -0.01, 0.0];
        d.exhaust.collector_length_m = 0.0;
        d.exhaust.collector_extra_length_m.clear();
        d.exhaust.after_collector.clear();
        d.exhaust.merge = None;
        d.exhaust.tail_length_m = 0.0;
        let mut out = Vec::new();
        exhaust(&d, &mut out);
        let pipes = tubes(&out, Part::Pipe);
        let want: f32 = (0..4).map(|i| d.exhaust.primary_length(i) as f32).sum();
        assert!(
            (length(&pipes) - want).abs() < 1e-4,
            "{} vs {want}",
            length(&pipes)
        );
        // and the first starts at cylinder 1's port: out of the head's back at 0.65 bore
        let b = d.geometry.bore_m as f32;
        assert!((pipes[0].0.z + 0.65 * b).abs() < 1e-6 && pipes[0].0.x.abs() < 1e-6);
    }

    #[test]
    fn a_collector_starts_where_its_primaries_end() {
        let mut d = EngineDef::inline4();
        d.exhaust.collectors = vec![vec![1, 4], vec![2, 3]];
        d.exhaust.after_collector.clear();
        d.exhaust.tail_length_m = 0.0;
        let mut out = Vec::new();
        exhaust(&d, &mut out);
        let pipes = tubes(&out, Part::Pipe);
        let ends = primary_ends(&d, &pipes);
        assert_eq!(ends.len(), 4);
        let want = 0.5 * (ends[0] + ends[3]);
        assert!(
            pipes.iter().any(|(a, _)| (*a - want).length() < 1e-5),
            "no collector at {want}"
        );
    }

    #[test]
    fn a_silencer_is_drawn_at_its_diameter() {
        let mut d = EngineDef::inline4();
        d.exhaust.after_collector = vec![
            Segment::Pipe {
                length_m: 0.3,
                diameter_m: 0.05,
            },
            Segment::Muffler {
                length_m: 0.35,
                diameter_m: 0.1206,
                tube_diameter_m: 0.05,
                packing_resistivity: 16_000.0,
                packing_porosity: 0.95,
            },
        ];
        d.exhaust.merge = None;
        let mut out = Vec::new();
        exhaust(&d, &mut out);
        let cans: Vec<_> = out.iter().filter(|i| i.part == Part::Silencer).collect();
        assert_eq!(cans.len(), 1, "one straight can");
        let (s, _, _) = cans[0].model.to_scale_rotation_translation();
        assert!(
            (s.x - 0.0603).abs() < 1e-6 && (s.y - 0.35).abs() < 1e-6,
            "{s}"
        );
    }

    #[test]
    fn two_outlets_draw_two_tails_apart() {
        let mut d = EngineDef::inline4();
        d.exhaust.tail_outlets = 2;
        d.exhaust.tail_spacing_m = 0.1;
        d.exhaust.merge = None;
        let mut out = Vec::new();
        let ends = exhaust(&d, &mut out);
        assert_eq!(ends.len(), 2);
        assert!(
            ((ends[1] - ends[0]).length() - 0.1).abs() < 1e-6,
            "{ends:?}"
        );
    }

    #[test]
    fn a_cylinder_left_out_of_the_groups_still_gets_its_primary() {
        // groups 1-2 and 3: cylinder 4 is drawn and joins the last group, whose collector then
        // starts between cylinders 3's and 4's primary ends
        let mut d = EngineDef::inline4();
        d.exhaust.collectors = vec![vec![1, 2], vec![3]];
        let mut out = Vec::new();
        exhaust(&d, &mut out);
        let pipes = tubes(&out, Part::Pipe);
        let ends = primary_ends(&d, &pipes);
        assert_eq!(ends.len(), 4, "cylinder 4's primary is drawn");
        let want = 0.5 * (ends[2] + ends[3]);
        assert!(
            pipes.iter().any(|(a, _)| (*a - want).length() < 1e-5),
            "no collector at {want}"
        );
    }

    #[test]
    fn a_rotarys_primaries_leave_its_housing() {
        let d = EngineDef::mazda_renesis();
        let rotor = d.geometry.rotor.unwrap();
        let reach = (rotor.generating_radius_m + rotor.eccentricity_m) as f32;
        let mut out = Vec::new();
        exhaust(&d, &mut out);
        let first = tubes(&out, Part::Pipe)[0].0;
        assert!(first.y.hypot(first.z) <= 1.1 * reach, "{first}");
    }

    #[test]
    fn legacy_muffler_fields_still_draw_a_tail() {
        let mut d = EngineDef::single();
        d.exhaust.after_collector.clear();
        d.exhaust.merge = None;
        d.exhaust.tail_length_m = 0.4;
        let mut out = Vec::new();
        let ends = exhaust(&d, &mut out);
        assert_eq!(ends.len(), 1);
        assert!(out.iter().any(|i| i.part == Part::Pipe));
    }

    #[test]
    fn every_preset_draws_a_finite_exhaust() {
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::honda_k20a(),
            EngineDef::lfa_v10(),
            EngineDef::crossplane_v8(),
            EngineDef::mazda_renesis(),
        ] {
            let mut out = Vec::new();
            let ends = exhaust(&d, &mut out);
            assert!(!ends.is_empty(), "{}", d.name);
            assert!(out.iter().all(|i| i.model.is_finite()), "{}", d.name);
        }
    }

    #[test]
    fn the_plenum_holds_its_volume() {
        let d = EngineDef::inline4();
        let mut out = Vec::new();
        intake(&d, &mut out);
        let p: Vec<_> = out.iter().filter(|i| i.part == Part::Plenum).collect();
        assert_eq!(p.len(), 1);
        let (s, _, _) = p[0].model.to_scale_rotation_translation();
        assert!(
            ((s.x * s.y * s.z) as f64 - d.plenum_volume_m3).abs() < 1e-9,
            "{s}"
        );
    }

    #[test]
    fn each_runner_is_its_length_long() {
        let d = EngineDef::inline4();
        let mut out = Vec::new();
        intake(&d, &mut out);
        let runners = tubes(&out, Part::Runner);
        let want = 4.0 * d.intake.runner_length_m as f32;
        assert!(
            (length(&runners) - want).abs() < 1e-4,
            "{} vs {want}",
            length(&runners)
        );
    }

    #[test]
    fn itbs_draw_trumpets_not_a_plenum() {
        let d = EngineDef::lfa_v10();
        assert!(d.intake.individual_throttles);
        let mut out = Vec::new();
        intake(&d, &mut out);
        assert!(!out.iter().any(|i| i.part == Part::Plenum));
        assert_eq!(out.iter().filter(|i| i.part == Part::Trumpet).count(), 10);
    }

    #[test]
    fn an_air_box_has_its_volume_and_ducts() {
        let mut d = EngineDef::inline4();
        d.intake.airbox = Some(engine_core::intake_fv::AirboxDef {
            volume_m3: 8e-3,
            ducts: 2,
            duct_length_m: 0.3,
            duct_diameter_m: 0.06,
        });
        let mut out = Vec::new();
        intake(&d, &mut out);
        let ab: Vec<_> = out.iter().filter(|i| i.part == Part::Airbox).collect();
        assert_eq!(ab.len(), 1);
        let (s, _, _) = ab[0].model.to_scale_rotation_translation();
        assert!((s.x * s.y * s.z - 8e-3).abs() < 1e-9, "{s}");
        // the ducts: two runs of 0.3 m after the runners' 4 × runner length
        let runners = length(&tubes(&out, Part::Runner));
        let want = 4.0 * d.intake.runner_length_m as f32 + 2.0 * 0.3;
        assert!((runners - want).abs() < 1e-4, "{runners} vs {want}");
    }
}
