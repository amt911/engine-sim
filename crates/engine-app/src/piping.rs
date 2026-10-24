//! The engine's pipes as a 2D schematic, from its definition (until the 3D view): what the air
//! flows through from the inlet to the cylinders and the gas from the cylinders to the outlets,
//! each part drawn at its real length and bore. Flow runs left to right; each cylinder is a row.
//! Pure geometry in metres; `view::piping` paints it.

use engine_core::engine::EngineDef;
use engine_core::exhaust::Segment;
use engine_core::intake_fv::IntakeModel;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Part {
    /// Where the air comes from (a marker).
    Air,
    Inlet,
    Tank,
    Plenum,
    Throttle,
    Runner,
    Cylinder,
    Primary,
    Collector,
    Pipe,
    Catalyst,
    Muffler,
    Resonator,
    /// Where the gas leaves (a marker).
    Outlet,
    /// A pipe end turning into the next part: drawn as a line from (x0, y0) to (x1, y1).
    Merge,
}

/// One drawn part: its kind, its box (metres; x along the flow, y down the rows) and a label.
#[derive(Debug, Clone, PartialEq)]
pub struct Item {
    pub part: Part,
    pub x0: f64,
    pub x1: f64,
    pub y0: f64,
    pub y1: f64,
    pub label: String,
}

fn mm(m: f64) -> String {
    format!("{:.0}", m * 1000.0)
}

/// Length of the markers for the air and the outlets, m.
const MARK: f64 = 0.05;
/// Length over which primaries bend into their collector, m (drawing only).
const MERGE: f64 = 0.08;

/// The schematic of `def`: each cylinder a row of 1.6 bores, the intake to its left and the
/// exhaust to its right.
pub fn schematic(def: &EngineDef) -> Vec<Item> {
    let n = def.cylinders.max(1);
    let bore = def.geometry.bore_m;
    let row = 1.6 * bore;
    let mid = |i: usize| (i as f64 + 0.5) * row;
    let mut out = Vec::new();
    let mut push = |part, x0: f64, x1: f64, yc: f64, h: f64, label: String| {
        out.push(Item {
            part,
            x0,
            x1,
            y0: yc - 0.5 * h,
            y1: yc + 0.5 * h,
            label,
        })
    };
    for i in 0..n {
        push(
            Part::Cylinder,
            0.0,
            bore,
            mid(i),
            bore,
            format!("{}", i + 1),
        );
    }

    // intake
    let intake = &def.intake;
    let (rl, rd) = (intake.runner_length_m, intake.runner_diameter_m);
    let throttle = |x: f64, yc: f64| (Part::Throttle, x - 0.01, x, yc, 1.2 * rd, String::new());
    let mut later = Vec::new();
    match intake.model {
        IntakeModel::Fv => {
            for i in 0..n {
                later.push((
                    Part::Runner,
                    -rl,
                    0.0,
                    mid(i),
                    rd,
                    format!("runner {} mm Ø{}", mm(rl), mm(rd)),
                ));
            }
            if intake.individual_throttles {
                for i in 0..n {
                    later.push(throttle(-rl, mid(i)));
                }
            }
            if intake.individual_throttles && !intake.surge_tanks.is_empty() {
                let groups = intake.surge_tanks.len();
                let per = n / groups;
                for (g, t) in intake.surge_tanks.iter().enumerate() {
                    // the tank runs along its bank's rows; its depth is the square of its section
                    let depth = t.area_m2.sqrt();
                    let x1 = -rl - 0.01;
                    // centred on its bank's rows
                    let yc = (g as f64 + 0.5) * per as f64 * row;
                    let top = yc - 0.5 * t.length_m;
                    later.push((
                        Part::Tank,
                        x1 - depth,
                        x1,
                        yc,
                        t.length_m,
                        format!("tank {} mm", mm(t.length_m)),
                    ));
                    let y_in = top + t.inlet_at.clamp(0.0, 1.0) * t.length_m;
                    let x_in = x1 - depth;
                    later.push((
                        Part::Inlet,
                        x_in - t.inlet_length_m,
                        x_in,
                        y_in,
                        t.inlet_diameter_m,
                        format!("inlet Ø{}", mm(t.inlet_diameter_m)),
                    ));
                    later.push((
                        Part::Air,
                        x_in - t.inlet_length_m - MARK,
                        x_in - t.inlet_length_m,
                        y_in,
                        t.inlet_diameter_m,
                        "air".into(),
                    ));
                }
            } else if intake.individual_throttles {
                for i in 0..n {
                    later.push((
                        Part::Air,
                        -rl - 0.01 - MARK,
                        -rl - 0.01,
                        mid(i),
                        rd,
                        String::new(),
                    ));
                }
            } else {
                let x1 = -rl;
                later.push((
                    Part::Plenum,
                    x1 - 0.15,
                    x1,
                    0.5 * n as f64 * row,
                    n as f64 * row,
                    "plenum".into(),
                ));
                later.push(throttle(x1 - 0.15, 0.5 * n as f64 * row));
                later.push((
                    Part::Air,
                    x1 - 0.16 - MARK,
                    x1 - 0.16,
                    0.5 * n as f64 * row,
                    rd,
                    "air".into(),
                ));
            }
        }
        IntakeModel::ZeroD => {
            let yc = 0.5 * n as f64 * row;
            later.push((
                Part::Plenum,
                -0.15,
                -0.02,
                yc,
                n as f64 * row,
                "plenum".into(),
            ));
            later.push(throttle(-0.15, yc));
            later.push((Part::Air, -0.16 - MARK, -0.16, yc, 0.04, "air".into()));
        }
    }

    // exhaust
    let ex = &def.exhaust;
    let (pl, pd) = (ex.primary_length_m, ex.primary_diameter_m);
    for i in 0..n {
        later.push((
            Part::Primary,
            bore,
            bore + pl,
            mid(i),
            pd,
            format!("primary {} mm Ø{}", mm(pl), mm(pd)),
        ));
    }
    let groups: Vec<Vec<usize>> = if ex.collectors.is_empty() {
        vec![(1..=n).collect()]
    } else {
        ex.collectors.clone()
    };
    let path = ex.path();
    let mut merges = Vec::new();
    for g in &groups {
        let yc = g.iter().map(|&c| mid(c.saturating_sub(1))).sum::<f64>() / g.len().max(1) as f64;
        // the primaries bend into their collector over a short merge
        let x_end = bore + pl;
        let mut x = x_end + MERGE;
        for &c in g {
            merges.push(Item {
                part: Part::Merge,
                x0: x_end,
                y0: mid(c.saturating_sub(1)),
                x1: x,
                y1: yc,
                label: String::new(),
            });
        }
        let (cl, cd) = (ex.collector_length_m, ex.collector_diameter_m);
        later.push((
            Part::Collector,
            x,
            x + cl,
            yc,
            cd,
            format!("collector {} mm Ø{}", mm(cl), mm(cd)),
        ));
        x += cl;
        for seg in &path {
            let (part, len, d, label) = match seg {
                Segment::Pipe {
                    length_m,
                    diameter_m,
                } => (Part::Pipe, *length_m, *diameter_m, "pipe"),
                Segment::Muffler {
                    length_m,
                    diameter_m,
                    ..
                } => (Part::Muffler, *length_m, *diameter_m, "silencer"),
                Segment::Catalyst {
                    length_m,
                    diameter_m,
                    ..
                } => (Part::Catalyst, *length_m, *diameter_m, "catalyst"),
                Segment::Resonator { volume_m3, .. } => {
                    // a side branch: drawn as a cube of its volume beside the line
                    let side = volume_m3.cbrt();
                    later.push((
                        Part::Resonator,
                        x,
                        x + side,
                        yc - 0.5 * side - 0.05,
                        side,
                        "resonator".into(),
                    ));
                    continue;
                }
            };
            later.push((
                part,
                x,
                x + len,
                yc,
                d,
                format!("{label} {} mm Ø{}", mm(len), mm(d)),
            ));
            x += len;
        }
        later.push((
            Part::Outlet,
            x,
            x + MARK,
            yc,
            ex.outlet_diameter(),
            "outlet".into(),
        ));
    }
    for (part, x0, x1, yc, h, label) in later {
        push(part, x0, x1, yc, h, label);
    }
    out.extend(merges);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn count(items: &[Item], part: Part) -> usize {
        items.iter().filter(|i| i.part == part).count()
    }

    fn first(items: &[Item], part: Part) -> &Item {
        items.iter().find(|i| i.part == part).unwrap()
    }

    #[test]
    fn the_lfa_is_drawn_as_toyota_built_it() {
        // ten throttled runners from two surge tanks, each fed by its inlet; ten cylinders, ten
        // primaries into two collectors, each followed by its tail pipe to an outlet
        let s = schematic(&EngineDef::lfa_v10());
        for (part, n) in [
            (Part::Air, 2),
            (Part::Inlet, 2),
            (Part::Tank, 2),
            (Part::Throttle, 10),
            (Part::Runner, 10),
            (Part::Cylinder, 10),
            (Part::Primary, 10),
            (Part::Collector, 2),
            (Part::Pipe, 2),
            (Part::Outlet, 2),
        ] {
            assert_eq!(count(&s, part), n, "{part:?}");
        }
        // at their real lengths: runners 325 mm, primaries 675 mm, the tank 485 mm along the bank
        let w = |i: &Item| i.x1 - i.x0;
        assert!((w(first(&s, Part::Runner)) - 0.325).abs() < 1e-9);
        assert!((w(first(&s, Part::Primary)) - 0.675).abs() < 1e-9);
        let tank = first(&s, Part::Tank);
        assert!((tank.y1 - tank.y0 - 0.485).abs() < 1e-9);
        assert!(first(&s, Part::Primary).label.contains("675"));
    }

    #[test]
    fn a_plain_engine_has_a_plenum_and_its_silencer() {
        // the inline-4: a 0D plenum behind one throttle; four cylinders, one collector, then the
        // default can and tail pipe
        let s = schematic(&EngineDef::inline4());
        assert_eq!(count(&s, Part::Plenum), 1);
        assert_eq!(count(&s, Part::Throttle), 1);
        assert_eq!(count(&s, Part::Cylinder), 4);
        assert_eq!(count(&s, Part::Primary), 4);
        assert_eq!(count(&s, Part::Collector), 1);
        assert!(count(&s, Part::Muffler) + count(&s, Part::Pipe) >= 1);
        assert_eq!(count(&s, Part::Outlet), 1);
    }

    #[test]
    fn the_flow_runs_left_to_right() {
        // every engine: whatever feeds a cylinder ends where it starts, and what follows it
        // starts where it ends; the exhaust chain after a collector never runs backwards
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::honda_k20a(),
            EngineDef::lfa_v10(),
            EngineDef::flatplane_v8(),
            EngineDef::crossplane_v8(),
        ] {
            let s = schematic(&d);
            let cyl = first(&s, Part::Cylinder);
            for i in &s {
                let before = matches!(
                    i.part,
                    Part::Air
                        | Part::Inlet
                        | Part::Tank
                        | Part::Plenum
                        | Part::Throttle
                        | Part::Runner
                );
                let after = matches!(
                    i.part,
                    Part::Primary
                        | Part::Collector
                        | Part::Pipe
                        | Part::Catalyst
                        | Part::Muffler
                        | Part::Resonator
                        | Part::Outlet
                );
                assert!(!before || i.x1 <= cyl.x0 + 1e-9, "{}: {i:?}", d.name);
                assert!(!after || i.x0 >= cyl.x1 - 1e-9, "{}: {i:?}", d.name);
                assert!(
                    i.x1 >= i.x0 && (i.part == Part::Merge || i.y1 >= i.y0),
                    "{}: {i:?}",
                    d.name
                );
            }
        }
    }
}
