//! The catalogue of complete breathing systems, sized to the engine
//! (`docs/superpowers/specs/2026-11-04-parts-catalogue-design.md`).
//!
//! Its two empirical numbers come from the real engines among the presets: each preset's exhaust
//! is its car's, documented where the preset is built. They are the mean gas speed in a pipe at
//! the torque peak, and the constant of the wave-tuning law.

use crate::engine::EngineDef;
use crate::exhaust::{C_PRIMARY, ExhaustDef};
use serde::{Deserialize, Serialize};

/// Volumetric efficiency at the torque peak. ESTIMATE: road engines fill 85–95 % of their swept
/// volume there (Heywood, *Internal Combustion Engine Fundamentals*, ch. 7); the sizing only
/// needs the flow's scale, and the same value enters the survey and the sizing, so it cancels.
pub const ETA_V: f64 = 0.9;
/// Exhaust over intake temperature. ESTIMATE: ~1000 K exhaust over 300 K air, the gas whose sound
/// speed the model's primaries carry ([`C_PRIMARY`]).
pub const T_RATIO: f64 = 1000.0 / 300.0;

/// The rpm a system is tuned for: the torque peak, a share of the redline (spec: 0.75 for road and
/// sport systems, 0.85 for race ones; Blair tunes the primaries for the torque peak).
pub fn tuned_rpm(redline_rpm: f64, race: bool) -> f64 {
    redline_rpm * if race { 0.85 } else { 0.75 }
}

/// Mean gas speed, m/s, in a pipe of `area_m2` carrying the exhaust of `cylinders` cylinders of
/// `swept_m3` each at `rpm`: `n·V_d·η_v·(N/120)·(T_exh/T_in) / A` (a four-stroke breathes once
/// every two turns).
pub fn mean_speed(cylinders: usize, swept_m3: f64, rpm: f64, area_m2: f64) -> f64 {
    cylinders as f64 * swept_m3 * ETA_V * (rpm / 120.0) * T_RATIO / area_m2.max(1e-12)
}

/// The blowdown's crank span, degrees: from the exhaust valve's opening to the next TDC
/// (`180 + EVO before BDC`), when its pulse must go out and come back as a suction.
pub fn blowdown_deg(def: &EngineDef) -> f64 {
    // the cycle starts at the firing TDC: BDC at 180°, the next TDC at 360°
    360.0 - def.exhaust_valve.open_deg
}

/// The two numbers, and their spread over the surveyed engines (min, max) for the warnings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Survey {
    /// Mean gas speed in a primary at the torque peak, m/s (median).
    pub v_primary: f64,
    pub v_primary_spread: (f64, f64),
    /// Mean gas speed in a collector, its bank's cylinders, m/s (median): faster than a primary's
    /// (the pulses take turns in it).
    pub v_collector: f64,
    pub v_collector_spread: (f64, f64),
    /// Mean gas speed in the tail, all cylinders, m/s (median).
    pub v_tail: f64,
    pub v_tail_spread: (f64, f64),
    /// The wave-tuning law `L = k·c·θ/(12·N)` (`θ` the blowdown span, `c` the primaries' sound
    /// speed, `N` rpm): the half round trip of the blowdown pulse; its form is the wave's, its
    /// constant `k` read off the real engines (median).
    pub tuning_k: f64,
    pub tuning_k_spread: (f64, f64),
}

/// The real engines surveyed: road cars whose exhausts the presets carry as built. Not the single
/// (a race pipe), the diesels, the rotaries; the K20C1's primaries feed its turbine, so they set
/// no tuning (its speeds count).
pub fn surveyed() -> Vec<(EngineDef, bool)> {
    vec![
        (EngineDef::honda_k20a(), true),
        (EngineDef::honda_k20c1(), false),
        (EngineDef::lfa_v10(), true),
        (EngineDef::subaru_ej25(), true),
        (EngineDef::flatplane_v8(), true),
        (EngineDef::crossplane_v8(), true),
    ]
}

/// Read the two numbers off the surveyed engines.
pub fn survey() -> Survey {
    let (mut primary, mut collector, mut tail, mut tuning) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for (d, tuned) in surveyed() {
        let n = tuned_rpm(d.redline_rpm, false);
        let swept = d.geometry.displacement();
        let x = &d.exhaust;
        primary.push(mean_speed(1, swept, n, area(x.primary_diameter_m)));
        let per = d.cylinders / x.groups(d.cylinders).len().max(1);
        collector.push(mean_speed(per, swept, n, area(x.collector_diameter_m)));
        // the tails: one per branch unless the branches merge, each with its outlets
        let branches = if x.merge.is_some() {
            1
        } else {
            x.groups(d.cylinders).len().max(1)
        };
        let tails = (branches * x.tail_outlets.max(1) as usize) as f64;
        tail.push(mean_speed(
            d.cylinders,
            swept,
            n,
            tails * area(x.tail_diameter_m),
        ));
        if tuned {
            tuning.push(x.primary_length_m * 12.0 * n / (C_PRIMARY * blowdown_deg(&d)));
        }
    }
    let (v_primary, v_primary_spread) = median_spread(&mut primary);
    let (v_tail, v_tail_spread) = median_spread(&mut tail);
    let (v_collector, v_collector_spread) = median_spread(&mut collector);
    let (tuning_k, tuning_k_spread) = median_spread(&mut tuning);
    Survey {
        v_primary,
        v_primary_spread,
        v_collector,
        v_collector_spread,
        v_tail,
        v_tail_spread,
        tuning_k,
        tuning_k_spread,
    }
}

/// A pipe's bore area, m².
pub fn area(diameter_m: f64) -> f64 {
    std::f64::consts::PI * 0.25 * diameter_m * diameter_m
}

/// The median (the mean of the middle two of an even count) and the range.
fn median_spread(v: &mut [f64]) -> (f64, (f64, f64)) {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n == 0 {
        return (0.0, (0.0, 0.0));
    }
    let mid = if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    };
    (mid, (v[0], v[n - 1]))
}

/// The editor's slider range of each exhaust measure, m: every sized measure stays inside it.
pub const PRIMARY_LENGTH_M: (f64, f64) = (0.1, 2.0);
pub const PRIMARY_DIAMETER_M: (f64, f64) = (0.020, 0.090);
pub const COLLECTOR_LENGTH_M: (f64, f64) = (0.01, 1.5);
pub const COLLECTOR_DIAMETER_M: (f64, f64) = (0.020, 0.120);
pub const TAIL_DIAMETER_M: (f64, f64) = (0.020, 0.120);
/// A silencer's length and shell, m (the editor's 50–1200 mm and 40–400 mm).
pub const SILENCER_LENGTH_M: (f64, f64) = (0.05, 1.2);
pub const SILENCER_DIAMETER_M: (f64, f64) = (0.040, 0.400);

/// Header types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Headers {
    /// One cylinder: its own pipe.
    Single,
    /// Every cylinder of a bank into one collector.
    FourOne,
    /// Pairs 360° apart into secondaries, the secondaries into one (tri-Y).
    FourTwoOne,
    /// Short stubs into a log: a cheap cast manifold.
    Log,
}

/// What follows the headers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum System {
    /// Catalyst and two chamber silencers.
    Road,
    /// Catalyst and one straight-through packed silencer.
    Sport,
    /// An open pipe.
    Race,
}

/// The headers this engine can take: a single only its own pipe; a tri-Y needs pairs 360° apart
/// in each bank.
pub fn headers_for(def: &EngineDef) -> Vec<Headers> {
    if def.cylinders <= 1 {
        return vec![Headers::Single];
    }
    let mut out = vec![Headers::FourOne];
    if banks(def).iter().all(|b| pairs(def, b).is_some()) {
        out.push(Headers::FourTwoOne);
    }
    out.push(Headers::Log);
    out
}

/// Each bank's cylinders (1-based), from the layout.
fn banks(def: &EngineDef) -> Vec<Vec<usize>> {
    let mut out: Vec<Vec<usize>> = Vec::new();
    for (i, (bank, _)) in def.layout.place(def.cylinders).into_iter().enumerate() {
        if out.len() <= bank {
            out.resize(bank + 1, Vec::new());
        }
        out[bank].push(i + 1);
    }
    out.retain(|b| !b.is_empty());
    out
}

/// A bank's cylinders paired 360° apart in the firing order (a tri-Y's secondaries), if they pair.
fn pairs(def: &EngineDef, bank: &[usize]) -> Option<Vec<Vec<usize>>> {
    if bank.len() < 2 || bank.len() % 2 == 1 {
        return None;
    }
    let off = def.phase_offsets_deg();
    let mut left: Vec<usize> = bank.to_vec();
    let mut out = Vec::new();
    while let Some(a) = left.pop() {
        let at = left
            .iter()
            .position(|&b| ((off[a - 1] - off[b - 1]).abs() - 360.0).abs() < 1e-6)?;
        let b = left.remove(at);
        out.push(vec![b.min(a), b.max(a)]);
    }
    out.sort();
    Some(out)
}

/// A pipe's bore carrying `cylinders` cylinders' exhaust at `speed`, clamped to `range`.
fn bore_for(def: &EngineDef, cylinders: usize, rpm: f64, speed: f64, range: (f64, f64)) -> f64 {
    let a = mean_speed(cylinders, def.geometry.displacement(), rpm, 1.0) / speed.max(1e-9);
    (4.0 * a / std::f64::consts::PI)
        .sqrt()
        .clamp(range.0, range.1)
}

/// Silencer attenuation at the firing frequency, dB. ESTIMATE: a road car meets its drive-by limit
/// about 20 dB under an open pipe at the firing order; a sport silencer half that. To be confirmed
/// by the user's ear (spec).
pub fn silencer_target_db(system: System) -> f64 {
    match system {
        System::Road => 20.0,
        System::Sport => 10.0,
        System::Race => 0.0,
    }
}

/// An expansion chamber's area ratio for `tl_db` at its peak: Munjal (*Acoustics of Ducts and
/// Mufflers*, ch. 3) `TL = 10·log10(1 + ¼(m − 1/m)²·sin²kL)`, at `kL = π/2`, solved for m.
pub fn chamber_ratio(tl_db: f64) -> f64 {
    // ¼(m − 1/m)² = 10^(TL/10) − 1 → m − 1/m = x → m = (x + √(x² + 4))/2
    let x = 2.0 * (10f64.powf(tl_db.max(0.0) / 10.0) - 1.0).sqrt();
    0.5 * (x + (x * x + 4.0).sqrt())
}

/// The exhaust of `h` and `s` sized for `def` (its other fields kept).
pub fn size_exhaust(def: &EngineDef, h: Headers, s: System, sv: &Survey) -> ExhaustDef {
    use crate::exhaust::{Merge, Segment};
    let n_t = tuned_rpm(def.redline_rpm, s == System::Race);
    let mut x = def.exhaust.clone();
    let tuned = |n: f64| {
        (sv.tuning_k * C_PRIMARY * blowdown_deg(def) / (12.0 * n))
            .clamp(PRIMARY_LENGTH_M.0, PRIMARY_LENGTH_M.1)
    };
    x.primary_diameter_m = bore_for(def, 1, n_t, sv.v_primary, PRIMARY_DIAMETER_M);
    x.primary_length_m = tuned(n_t);
    x.primary_length_deviation_m.clear();
    x.collector_extra_length_m.clear();
    x.merge = None;
    let banks = banks(def);
    // the collectors: a bank's cylinders together, or (tri-Y) its pairs, or one pipe (a single)
    let (groups, per_collector) = match h {
        Headers::FourTwoOne => {
            let p: Vec<Vec<usize>> = banks
                .iter()
                .filter_map(|b| pairs(def, b))
                .flatten()
                .collect();
            (p, 2)
        }
        _ if banks.len() > 1 => {
            let n = banks.iter().map(Vec::len).max().unwrap_or(1);
            (banks.clone(), n)
        }
        _ => (Vec::new(), def.cylinders.max(1)),
    };
    x.collectors = groups;
    x.collector_diameter_m = bore_for(
        def,
        per_collector,
        n_t,
        sv.v_collector,
        COLLECTOR_DIAMETER_M,
    );
    x.collector_length_m = match h {
        // ESTIMATE: a merge's own length; the primaries carry the tuning
        Headers::Single | Headers::FourOne => 0.15,
        // the secondaries tuned by the same law (the second pulse a pair's partner sends)
        Headers::FourTwoOne => tuned(n_t),
        // ESTIMATE: a cast log runs along the bank, about a third of a tuned primary
        Headers::Log => 0.3 * x.primary_length_m,
    }
    .clamp(COLLECTOR_LENGTH_M.0, COLLECTOR_LENGTH_M.1);
    if h == Headers::Log {
        // a log's stubs: the shortest primaries the editor takes
        x.primary_length_m = PRIMARY_LENGTH_M.0;
    }
    // the system: one per branch, or one after the merge (a tri-Y's)
    let merged = h == Headers::FourTwoOne;
    let branch_cyl = if merged { def.cylinders } else { per_collector };
    let tail = bore_for(def, branch_cyl, n_t, sv.v_tail, TAIL_DIAMETER_M);
    let firing_hz = branch_cyl as f64 * n_t / 120.0;
    let can = |f: f64, tl: f64, tube: f64| Segment::Muffler {
        // a quarter wave at f in the tail's gas: the chamber's first peak (kL = π/2)
        length_m: (crate::exhaust::C_TAIL / (4.0 * f))
            .clamp(SILENCER_LENGTH_M.0, SILENCER_LENGTH_M.1),
        diameter_m: (tail * chamber_ratio(tl).sqrt())
            .clamp(SILENCER_DIAMETER_M.0.max(tube * 1.2), SILENCER_DIAMETER_M.1),
        tube_diameter_m: tube,
        packing_resistivity: crate::exhaust::default_resistivity(),
        packing_porosity: crate::exhaust::default_porosity(),
    };
    let pipe = |len: f64| Segment::Pipe {
        length_m: len,
        diameter_m: tail,
    };
    // ESTIMATE: a catalyst's brick has four times the pipe's area (the presets' 0.14 m on 0.05 m)
    let cat = Segment::Catalyst {
        length_m: 0.13,
        diameter_m: 2.0 * tail,
        cells_per_in2: 400.0,
        wall_m: 0.000165,
    };
    let system = match s {
        System::Road => vec![
            cat,
            pipe(0.4),
            can(firing_hz, silencer_target_db(s), 0.0),
            pipe(0.6),
            can(2.0 * firing_hz, silencer_target_db(s), 0.0),
            pipe(0.4),
        ],
        System::Sport => vec![
            cat,
            pipe(0.6),
            can(firing_hz, silencer_target_db(s), tail),
            pipe(0.5),
        ],
        System::Race => vec![pipe(1.0)],
    };
    x.tail_length_m = 1.0;
    x.tail_diameter_m = tail;
    x.muffler_transmission = 1.0;
    if merged {
        x.after_collector = vec![Segment::Pipe {
            length_m: 0.05,
            diameter_m: x.collector_diameter_m,
        }];
        x.merge = Some(Merge {
            segments: system,
            outlet_diameter_m: tail,
            bypass: None,
        });
    } else {
        x.after_collector = system;
    }
    x
}

/// The systems chosen for an engine, remembered so a change of the engine re-sizes them.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Systems {
    pub exhaust: Option<ExhaustChoice>,
}

/// The exhaust chosen, and the measures the user set by hand since (kept on a re-size).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExhaustChoice {
    pub headers: Headers,
    pub system: System,
    #[serde(default)]
    pub kept: Vec<String>,
}

/// The exhaust measures a choice sizes, by their field names.
pub const MEASURES: [&str; 6] = [
    "primary_length_m",
    "primary_diameter_m",
    "collector_length_m",
    "collector_diameter_m",
    "tail_length_m",
    "tail_diameter_m",
];

/// A measure outside what the surveyed real engines use for this engine.
#[derive(Debug, Clone, PartialEq)]
pub struct Warning {
    pub measure: &'static str,
    pub value: f64,
    pub range: (f64, f64),
}

/// Choose an exhaust: size it, remember it, forget earlier hand edits.
pub fn choose_exhaust(def: &mut EngineDef, h: Headers, s: System) {
    def.systems.get_or_insert_with(Systems::default).exhaust = Some(ExhaustChoice {
        headers: h,
        system: s,
        kept: Vec::new(),
    });
    resize(def);
}

/// A measure of an exhaust by its name (the tail's through [`ExhaustDef::tail_mut`]).
fn measure_mut<'a>(x: &'a mut ExhaustDef, name: &str) -> Option<&'a mut f64> {
    match name {
        "primary_length_m" => Some(&mut x.primary_length_m),
        "primary_diameter_m" => Some(&mut x.primary_diameter_m),
        "collector_length_m" => Some(&mut x.collector_length_m),
        "collector_diameter_m" => Some(&mut x.collector_diameter_m),
        "tail_length_m" => Some(x.tail_mut().0),
        "tail_diameter_m" => Some(x.tail_mut().1),
        _ => None,
    }
}

/// The headers a choice takes on this engine: its own if the engine can take them, else a 4-1
/// (a single's choice on an engine raised to four cylinders), or a single's pipe on one cylinder.
fn fitting(def: &EngineDef, h: Headers) -> Headers {
    let can = headers_for(def);
    if can.contains(&h) {
        h
    } else if def.cylinders <= 1 {
        Headers::Single
    } else {
        Headers::FourOne
    }
}

/// Re-size the chosen systems for the engine as it is now, keeping the hand-set measures.
pub fn resize(def: &mut EngineDef) {
    let Some(choice) = def.systems.as_ref().and_then(|s| s.exhaust.clone()) else {
        return;
    };
    let mut sized = size_exhaust(def, fitting(def, choice.headers), choice.system, &survey());
    for name in &choice.kept {
        if let (Some(v), Some(to)) = (
            measure_mut(&mut def.exhaust, name).map(|v| *v),
            measure_mut(&mut sized, name),
        ) {
            *to = v;
        }
    }
    def.exhaust = sized;
}

/// Record as hand-set every exhaust measure that differs from what the choice sizes.
pub fn note_hand_edits(def: &mut EngineDef) {
    let Some(choice) = def.systems.as_ref().and_then(|s| s.exhaust.clone()) else {
        return;
    };
    let mut sized = size_exhaust(def, fitting(def, choice.headers), choice.system, &survey());
    let mut here = def.exhaust.clone();
    let kept: Vec<String> = MEASURES
        .iter()
        .filter(|&&m| {
            let (a, b) = (
                measure_mut(&mut here, m).map(|v| *v),
                measure_mut(&mut sized, m).map(|v| *v),
            );
            matches!((a, b), (Some(a), Some(b)) if (a - b).abs() > 1e-9)
        })
        .map(|m| (*m).to_string())
        .collect();
    if let Some(c) = def.systems.as_mut().and_then(|s| s.exhaust.as_mut()) {
        c.kept = kept;
    }
}

/// The exhaust measures outside the surveyed engines' range, scaled to this engine.
pub fn out_of_range(def: &EngineDef) -> Vec<Warning> {
    let sv = survey();
    let n = tuned_rpm(def.redline_rpm, false);
    let x = &def.exhaust;
    let swept = def.geometry.displacement();
    // a bore from a speed range: the faster engines' small end, the slower ones' large end
    let bores = |cyl: usize, (lo, hi): (f64, f64)| {
        let d = |v: f64| (4.0 * mean_speed(cyl, swept, n, 1.0) / (std::f64::consts::PI * v)).sqrt();
        (d(hi), d(lo))
    };
    let groups = x.groups(def.cylinders).len().max(1);
    let per = def.cylinders.div_ceil(groups);
    let branch = if x.merge.is_some() {
        def.cylinders
    } else {
        per
    };
    let theta = blowdown_deg(def);
    let lengths = (
        sv.tuning_k_spread.0 * C_PRIMARY * theta / (12.0 * n),
        sv.tuning_k_spread.1 * C_PRIMARY * theta / (12.0 * n),
    );
    let mut here = x.clone();
    let checks = [
        ("primary_diameter_m", bores(1, sv.v_primary_spread)),
        ("collector_diameter_m", bores(per, sv.v_collector_spread)),
        ("tail_diameter_m", bores(branch, sv.v_tail_spread)),
        ("primary_length_m", lengths),
    ];
    checks
        .into_iter()
        .filter_map(|(m, (lo, hi))| {
            let v = *measure_mut(&mut here, m)?;
            // 2 % of slack: the surveyed engines themselves sit on the edges
            (v < 0.98 * lo || v > 1.02 * hi).then_some(Warning {
                measure: m,
                value: v,
                range: (lo, hi),
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_k20a_s_primary_speed_by_hand() {
        // K20A: 86 × 86 mm (0.4996 L a cylinder), redline 8400 → tuned 6300 rpm, 40 mm primary
        // (1.2566e-3 m²): 0.4996e-3 · 0.9 · 6300/120 · 3.333 / 1.2566e-3 = 62.6 m/s
        let d = EngineDef::honda_k20a();
        let n = tuned_rpm(d.redline_rpm, false);
        assert!((n - 6300.0).abs() < 1e-9);
        let swept = d.geometry.displacement();
        let a = std::f64::consts::PI * 0.25 * d.exhaust.primary_diameter_m.powi(2);
        let v = mean_speed(1, swept, n, a);
        assert!((v - 62.6).abs() < 0.5, "{v}");
        // its blowdown: the exhaust opens at 140.5° (39.5° before BDC) → 219.5°
        assert!((blowdown_deg(&d) - 219.5).abs() < 1e-9);
        // a race system tunes higher: 0.85 × 8400
        assert!((tuned_rpm(8400.0, true) - 7140.0).abs() < 1e-9);
    }

    #[test]
    fn the_survey_holds_its_engines() {
        let s = survey();
        // the tuning constant: the median of K20A 0.358, LFA 0.484, EJ25 0.323, flat-plane
        // V8 0.258, cross-plane V8 0.323 (each L·12·N/(c·θ), worked by hand) = 0.323
        assert!((s.tuning_k - 0.323).abs() < 0.005, "{}", s.tuning_k);
        assert!(s.tuning_k_spread.0 < 0.27 && s.tuning_k_spread.1 > 0.47);
        // primary speeds 57–69 m/s by hand (K20A 62.6, LFA 58.6, EJ25 56.8, flat V8 68.5,
        // cross V8 62.7, K20C1 52.5 at its 7000 rpm redline)
        assert!(s.v_primary > 55.0 && s.v_primary < 66.0, "{}", s.v_primary);
        assert!(s.v_primary_spread.0 <= s.v_primary && s.v_primary <= s.v_primary_spread.1);
        assert!(s.v_tail > 0.0 && s.v_tail_spread.0 <= s.v_tail && s.v_tail <= s.v_tail_spread.1);
        // collectors, by hand: the K20A's 4 into 55 mm at 6300 rpm, 4·62.6·(40/55)² = 132 m/s;
        // the real ones run about twice a primary's speed
        assert!(
            s.v_collector > 1.5 * s.v_primary,
            "{} {}",
            s.v_collector,
            s.v_primary
        );
    }

    fn sized(d: &EngineDef, h: Headers, s: System) -> ExhaustDef {
        size_exhaust(d, h, s, &survey())
    }

    #[test]
    fn a_two_litre_four_gets_its_primary_by_continuity() {
        // the inline-4 (86 × 86, redline 7000 → tuned 5250): a primary carrying one cylinder at
        // the survey's speed v: A = 0.4996e-3·0.9·5250/120·3.333/v; at v = 60.57 m/s,
        // A = 1.0826e-3 m², D = 37.1 mm
        let mut d = EngineDef::inline4();
        d.redline_rpm = 7000.0;
        let x = sized(&d, Headers::FourOne, System::Road);
        assert!(
            (x.primary_diameter_m - 0.0371).abs() < 0.0006,
            "{}",
            x.primary_diameter_m
        );
        // its length: k·c·θ/(12·N) = 0.3235·600·θ/(12·5250), θ its blowdown span
        let theta = blowdown_deg(&d);
        let l = 0.3235 * 600.0 * theta / (12.0 * 5250.0);
        assert!(
            (x.primary_length_m - l).abs() < 0.01,
            "{} {l}",
            x.primary_length_m
        );
    }

    #[test]
    fn the_tuning_law_lengthens_primaries_for_a_slower_engine() {
        let mut d = EngineDef::inline4();
        d.redline_rpm = 8000.0;
        let fast = sized(&d, Headers::FourOne, System::Road).primary_length_m;
        d.redline_rpm = 4000.0;
        let slow = sized(&d, Headers::FourOne, System::Road).primary_length_m;
        assert!((slow / fast - 2.0).abs() < 1e-9, "{fast} {slow}");
    }

    #[test]
    fn munjals_chamber_gives_its_target_loss() {
        // m for 20 dB at kL = π/2: (m − 1/m) = 2·√(10² − 1) = 19.900, m = 19.950
        assert!(
            (chamber_ratio(20.0) - 19.950).abs() < 0.005,
            "{}",
            chamber_ratio(20.0)
        );
        assert!((chamber_ratio(0.0) - 1.0).abs() < 1e-12);
        // the road silencer of a 2.0 L four tuned at 5250 rpm: firing 175 Hz, a quarter wave in
        // the tail's 500 m/s gas: 0.714 m; its shell √m = 4.47 times the tail's bore
        let mut d = EngineDef::inline4();
        d.redline_rpm = 7000.0;
        let x = sized(&d, Headers::FourOne, System::Road);
        let cans: Vec<_> = x
            .after_collector
            .iter()
            .filter_map(|s| match *s {
                crate::exhaust::Segment::Muffler {
                    length_m,
                    diameter_m,
                    ..
                } => Some((length_m, diameter_m)),
                _ => None,
            })
            .collect();
        assert_eq!(cans.len(), 2, "two silencers");
        assert!((cans[0].0 - 0.714).abs() < 0.005, "{:?}", cans);
        let tail = x.outlet_diameter();
        assert!(
            (cans[0].1 / tail - 19.950f64.sqrt()).abs() < 0.02,
            "{:?} {tail}",
            cans
        );
        assert!(
            x.after_collector
                .iter()
                .any(|s| matches!(s, crate::exhaust::Segment::Catalyst { .. }))
        );
        // race: no catalyst, no silencer
        let race = sized(&d, Headers::FourOne, System::Race);
        assert!(
            race.after_collector
                .iter()
                .all(|s| matches!(s, crate::exhaust::Segment::Pipe { .. }))
        );
    }

    #[test]
    fn a_single_gets_single_headers_and_a_v8_headers_per_bank() {
        assert_eq!(headers_for(&EngineDef::single()), vec![Headers::Single]);
        let i4 = headers_for(&EngineDef::inline4());
        assert!(
            i4.contains(&Headers::FourOne)
                && i4.contains(&Headers::FourTwoOne)
                && i4.contains(&Headers::Log)
        );
        let v8 = EngineDef::crossplane_v8();
        let x = sized(&v8, Headers::FourOne, System::Road);
        assert_eq!(x.groups(8).len(), 2, "a collector per bank");
        // a tri-Y pairs cylinders 360° apart
        let tri = sized(&EngineDef::inline4(), Headers::FourTwoOne, System::Road);
        let offsets = EngineDef::inline4().phase_offsets_deg();
        for g in tri.groups(4) {
            assert_eq!(g.len(), 2);
            assert!(
                ((offsets[g[0]] - offsets[g[1]]).abs() - 360.0).abs() < 1e-9,
                "{g:?}"
            );
        }
        assert!(tri.merge.is_some(), "the secondaries merge");
    }

    #[test]
    fn a_turbo_engine_keeps_its_turbine() {
        let d = EngineDef::honda_k20c1();
        let mut e = d.clone();
        e.exhaust = sized(&d, Headers::FourOne, System::Road);
        assert!(e.turbo.is_some());
        assert!(
            crate::config::validate(&e).is_ok(),
            "{:?}",
            crate::config::validate(&e)
        );
    }

    proptest::proptest! {
        #[test]
        fn every_choice_is_a_valid_engine_in_range(
            cyl in 1usize..=12, layout in 0usize..3, redline in 4000.0f64..9000.0, sys in 0usize..3,
        ) {
            let mut d = match layout {
                0 => EngineDef::inline4(),
                1 => EngineDef::crossplane_v8(),
                _ => EngineDef::subaru_ej25(),
            };
            d.set_cylinders(if layout == 0 { cyl } else { (cyl.max(2) / 2) * 2 });
            d.redline_rpm = redline;
            let system = [System::Road, System::Sport, System::Race][sys];
            for h in headers_for(&d) {
                let mut e = d.clone();
                e.exhaust = size_exhaust(&d, h, system, &survey());
                proptest::prop_assert!(crate::config::validate(&e).is_ok(), "{:?} {:?}", h, crate::config::validate(&e));
                let x = &e.exhaust;
                let within = |v: f64, r: (f64, f64)| v >= r.0 - 1e-12 && v <= r.1 + 1e-12;
                proptest::prop_assert!(within(x.primary_length_m, PRIMARY_LENGTH_M));
                proptest::prop_assert!(within(x.primary_diameter_m, PRIMARY_DIAMETER_M));
                proptest::prop_assert!(within(x.collector_length_m, COLLECTOR_LENGTH_M));
                proptest::prop_assert!(within(x.collector_diameter_m, COLLECTOR_DIAMETER_M));
                proptest::prop_assert!(within(x.outlet_diameter(), TAIL_DIAMETER_M));
            }
        }
    }

    #[test]
    fn a_chosen_exhaust_round_trips_and_presets_carry_none() {
        let mut d = EngineDef::single();
        d.set_cylinders(4);
        choose_exhaust(&mut d, Headers::FourOne, System::Road);
        assert!(d.systems.is_some());
        let yaml = serde_yaml_ng::to_string(&d).unwrap();
        let back: EngineDef = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(back.exhaust, d.exhaust, "exhaust");
        assert_eq!(back.systems, d.systems, "systems");
        assert_eq!(back, d);
        let json = serde_json::to_string(&d).unwrap();
        assert_eq!(serde_json::from_str::<EngineDef>(&json).unwrap(), d);
        // a preset writes no `systems`
        assert!(
            !serde_yaml_ng::to_string(&EngineDef::lfa_v10())
                .unwrap()
                .contains("systems")
        );
    }

    #[test]
    fn changing_cylinders_resizes_and_keeps_hand_edits() {
        let mut d = EngineDef::single();
        choose_exhaust(&mut d, Headers::Single, System::Road);
        d.set_cylinders(4);
        let sized = size_exhaust(&d, Headers::FourOne, System::Road, &survey());
        // the single's choice becomes the four's 4-1 (a single's headers make no sense on four)
        assert_eq!(d.exhaust.collector_diameter_m, sized.collector_diameter_m);
        // a hand edit stays through a re-size
        *d.exhaust.tail_mut().0 = 2.5;
        note_hand_edits(&mut d);
        d.set_cylinders(3);
        assert_eq!(*d.exhaust.tail_mut().0, 2.5);
        // 3 → 4 gives the four's sizing again (but the kept tail)
        d.set_cylinders(4);
        assert_eq!(d.exhaust.primary_length_m, sized.primary_length_m);
        assert_eq!(d.exhaust.collector_diameter_m, sized.collector_diameter_m);
    }

    #[test]
    fn the_forty_mm_collector_on_four_cylinders_is_flagged() {
        // the user's case: the single raised to four keeps its 40 mm collector (no choice made)
        let mut d = EngineDef::single();
        d.set_cylinders(4);
        let w = out_of_range(&d);
        assert!(
            w.iter().any(|w| w.measure == "collector_diameter_m"),
            "{w:?}"
        );
        // the K20A, a surveyed real engine, has none out of range
        assert!(
            out_of_range(&EngineDef::honda_k20a()).is_empty(),
            "{:?}",
            out_of_range(&EngineDef::honda_k20a())
        );
        // a kept hand edit out of range is still flagged
        let mut d = EngineDef::inline4();
        choose_exhaust(&mut d, Headers::FourOne, System::Road);
        d.exhaust.collector_diameter_m = 0.03;
        note_hand_edits(&mut d);
        resize(&mut d);
        assert_eq!(d.exhaust.collector_diameter_m, 0.03);
        assert!(
            out_of_range(&d)
                .iter()
                .any(|w| w.measure == "collector_diameter_m")
        );
    }

    #[test]
    fn unknown_kept_names_are_ignored() {
        let mut d = EngineDef::inline4();
        choose_exhaust(&mut d, Headers::FourOne, System::Road);
        if let Some(c) = d.systems.as_mut().and_then(|s| s.exhaust.as_mut()) {
            c.kept.push("no_such_measure".into());
        }
        resize(&mut d);
        assert!(crate::config::validate(&d).is_ok());
    }
}
