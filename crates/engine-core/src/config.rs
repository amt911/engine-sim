//! Engine files: YAML / JSON (de)serialization of [`EngineDef`] with a versioned envelope,
//! and validation. Pure string ↔ data; file IO lives in the app.

use crate::engine::EngineDef;
use serde::{Deserialize, Serialize};

/// Current file format version. Bump on a breaking change and add a migration + test.
pub const FORMAT_VERSION: u32 = 2;
pub const MAX_CYLINDERS: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    Yaml,
    Json,
}

impl Format {
    /// From a file name's extension (`.yaml`, `.yml`, `.json`), case-insensitive.
    pub fn from_path(path: &str) -> Option<Format> {
        let ext = std::path::Path::new(path)
            .extension()?
            .to_str()?
            .to_ascii_lowercase();
        match ext.as_str() {
            "yaml" | "yml" => Some(Format::Yaml),
            "json" => Some(Format::Json),
            _ => None,
        }
    }
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum ConfigError {
    #[error("cannot parse engine file: {0}")]
    Parse(String),
    #[error("unsupported engine file version {0} (this build reads up to {FORMAT_VERSION})")]
    UnsupportedVersion(u64),
    #[error("invalid engine: {}", .0.join("; "))]
    Invalid(Vec<String>),
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EngineFile {
    version: u32,
    engine: EngineDef,
}

pub fn to_string(def: &EngineDef, format: Format) -> Result<String, ConfigError> {
    let file = EngineFile {
        version: FORMAT_VERSION,
        engine: def.clone(),
    };
    match format {
        Format::Yaml => {
            serde_yaml_ng::to_string(&file).map_err(|e| ConfigError::Parse(e.to_string()))
        }
        Format::Json => {
            serde_json::to_string_pretty(&file).map_err(|e| ConfigError::Parse(e.to_string()))
        }
    }
}

/// Parse and validate an engine file.
pub fn from_str(text: &str, format: Format) -> Result<EngineDef, ConfigError> {
    // Parse into a generic tree first: read the version, migrate old layouts, then type it.
    let parse = |e: String| ConfigError::Parse(e);
    let mut tree: serde_json::Value = match format {
        Format::Yaml => serde_yaml_ng::from_str(text).map_err(|e| parse(e.to_string()))?,
        Format::Json => serde_json::from_str(text).map_err(|e| parse(e.to_string()))?,
    };
    let version = tree
        .get("version")
        .and_then(serde_json::Value::as_u64)
        .ok_or_else(|| parse("missing `version`".into()))?;
    if version == 0 || version > FORMAT_VERSION as u64 {
        return Err(ConfigError::UnsupportedVersion(version));
    }
    if version == 1 {
        migrate_v1(&mut tree);
    }
    let file: EngineFile = serde_json::from_value(tree).map_err(|e| parse(e.to_string()))?;
    validate(&file.engine)?;
    Ok(file.engine)
}

/// v1 → v2: the per-cylinder 0D runner + outlet became an exhaust system of waveguides.
fn migrate_v1(tree: &mut serde_json::Value) {
    tree["version"] = 2.into();
    let Some(engine) = tree
        .get_mut("engine")
        .and_then(serde_json::Value::as_object_mut)
    else {
        return;
    };
    let mut exhaust = serde_json::Map::new();
    for (old, new) in [
        ("exhaust_runner_length_m", "primary_length_m"),
        ("exhaust_runner_diameter_m", "primary_diameter_m"),
        ("exhaust_outlet_diameter_m", "tail_diameter_m"),
    ] {
        if let Some(v) = engine.remove(old) {
            exhaust.insert(new.into(), v);
        }
    }
    if exhaust.is_empty() {
        return;
    }
    // Merge into an `exhaust` object the file may already have (its keys win).
    match engine
        .get_mut("exhaust")
        .and_then(serde_json::Value::as_object_mut)
    {
        Some(existing) => {
            for (k, v) in exhaust {
                existing.entry(k).or_insert(v);
            }
        }
        None => {
            engine.insert("exhaust".into(), exhaust.into());
        }
    }
}

/// Every problem found, so the editor can show them all at once.
pub fn validate(def: &EngineDef) -> Result<(), ConfigError> {
    // Bounds are generous physical limits (wider than the editor's sliders); NaN fails all.
    let mut errs = Vec::new();
    let mut check = |ok: bool, msg: &str| {
        if !ok {
            errs.push(msg.to_string());
        }
    };
    let within = |x: f64, lo: f64, hi: f64| x.is_finite() && (lo..=hi).contains(&x);
    let g = &def.geometry;
    check(
        (1..=MAX_CYLINDERS).contains(&def.cylinders),
        "cylinders must be between 1 and 16",
    );
    check(within(g.bore_m, 0.02, 0.3), "bore must be 20–300 mm");
    check(within(g.stroke_m, 0.02, 0.3), "stroke must be 20–300 mm");
    check(
        within(g.rod_m, 0.0, 1.0) && g.rod_m > g.stroke_m * 0.5,
        "rod must be longer than half the stroke (and < 1 m)",
    );
    check(
        within(g.compression_ratio, 1.01, 30.0),
        "compression ratio must be 1–30",
    );
    for (name, v) in [
        ("intake valve", &def.intake_valve),
        ("exhaust valve", &def.exhaust_valve),
    ] {
        let ok = within(v.open_deg, -720.0, 720.0)
            && within(v.duration_deg, 1.0, 719.0)
            && within(v.max_lift_m, 0.0005, 0.05)
            && within(v.diameter_m, 0.005, 0.2);
        check(
            ok,
            &format!("{name}: opening in ±720°, duration 1–719°, lift 0.5–50 mm, Ø 5–200 mm"),
        );
    }
    check(
        within(def.spark_advance_deg, -90.0, 90.0),
        "spark advance must be −90…90° BTDC",
    );
    check(
        within(def.burn_duration_deg, 5.0, 180.0),
        "burn duration must be 5–180°",
    );
    check(
        within(def.flywheel_inertia_kgm2, 0.005, 10.0),
        "flywheel inertia must be 0.005–10 kg·m²",
    );
    check(
        within(def.plenum_volume_m3, 1e-5, 0.1) && within(def.throttle_diameter_m, 0.005, 0.5),
        "plenum 0.01–100 L and throttle Ø 5–500 mm",
    );
    check(
        within(def.idle_throttle, 0.0, 1.0),
        "idle throttle must be in [0, 1]",
    );
    let x = &def.exhaust;
    check(
        within(x.primary_length_m, 0.05, 10.0)
            && within(x.collector_length_m, 0.01, 10.0)
            && within(x.tail_length_m, 0.05, 20.0)
            && [
                x.primary_diameter_m,
                x.collector_diameter_m,
                x.tail_diameter_m,
            ]
            .iter()
            .all(|d| within(*d, 0.005, 0.3)),
        "exhaust pipes: lengths 0.01–20 m, Ø 5–300 mm",
    );
    check(
        within(x.muffler_transmission, 0.05, 1.0),
        "muffler transmission must be 0.05–1",
    );
    if !x.collectors.is_empty() {
        let mut all: Vec<usize> = x.collectors.iter().flatten().copied().collect();
        all.sort_unstable();
        check(
            all == (1..=def.cylinders).collect::<Vec<_>>()
                && x.collectors.iter().all(|g| !g.is_empty()),
            "exhaust collectors must group every cylinder exactly once",
        );
    }
    check(
        within(def.redline_rpm, 500.0, 30_000.0),
        "redline must be 500–30000 rpm",
    );
    check(
        within(def.starter_torque_nm, 0.0, 5000.0),
        "starter torque must be 0–5000 N·m",
    );
    check(
        def.fmep_bar.iter().all(|c| within(*c, 0.0, 20.0)),
        "fmep coefficients must be 0–20 bar",
    );
    if !def.firing_order.is_empty() {
        let mut o = def.firing_order.clone();
        o.sort_unstable();
        check(
            o == (1..=def.cylinders).collect::<Vec<_>>(),
            "firing order must list every cylinder exactly once",
        );
    }
    if errs.is_empty() {
        Ok(())
    } else {
        Err(ConfigError::Invalid(errs))
    }
}

/// A sensible even-firing order for `n` inline cylinders (1-based).
pub fn default_firing_order(n: usize) -> Vec<usize> {
    match n {
        3 => vec![1, 3, 2],
        4 => vec![1, 3, 4, 2],
        5 => vec![1, 2, 4, 5, 3],
        6 => vec![1, 5, 3, 6, 2, 4],
        8 => vec![1, 6, 2, 5, 8, 3, 7, 4],
        10 => vec![1, 6, 5, 10, 2, 7, 3, 8, 4, 9],
        12 => vec![1, 7, 5, 11, 3, 9, 6, 12, 2, 8, 4, 10],
        _ => (1..=n).step_by(2).chain((2..=n).step_by(2)).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn presets() -> Vec<EngineDef> {
        vec![
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
        ]
    }

    #[test]
    fn format_from_path() {
        assert_eq!(Format::from_path("a/b/v10.yaml"), Some(Format::Yaml));
        assert_eq!(Format::from_path("v10.YML"), Some(Format::Yaml));
        assert_eq!(Format::from_path("v10.json"), Some(Format::Json));
        assert_eq!(Format::from_path("v10.toml"), None);
        assert_eq!(Format::from_path("yaml"), None);
    }

    #[test]
    fn round_trip_yaml_and_json() {
        for def in presets() {
            for f in [Format::Yaml, Format::Json] {
                let text = to_string(&def, f).unwrap();
                assert_eq!(from_str(&text, f).unwrap(), def, "{} {f:?}", def.name);
            }
        }
    }

    #[test]
    fn written_file_carries_version() {
        let y = to_string(&EngineDef::single(), Format::Yaml).unwrap();
        assert!(y.starts_with("version: 2\n"), "{y}");
        let j = to_string(&EngineDef::single(), Format::Json).unwrap();
        assert!(j.contains("\"version\": 2"), "{j}");
    }

    #[test]
    fn partial_file_takes_defaults_and_auto_firing_order() {
        let def = from_str(
            "version: 1\nengine:\n  name: My twin\n  cylinders: 2\n",
            Format::Yaml,
        )
        .unwrap();
        assert_eq!(def.name, "My twin");
        assert_eq!(def.cylinders, 2);
        assert_eq!(def.geometry, EngineDef::single().geometry);
        assert_eq!(def.phase_offsets_deg(), vec![0.0, 360.0]);
    }

    #[test]
    fn unknown_version_and_garbage_are_errors() {
        let e = from_str("version: 99\nengine: {}\n", Format::Yaml).unwrap_err();
        assert_eq!(e, ConfigError::UnsupportedVersion(99));
        assert!(matches!(
            from_str("{ not json", Format::Json),
            Err(ConfigError::Parse(_))
        ));
        assert!(
            matches!(
                from_str("engine: {}\n", Format::Yaml),
                Err(ConfigError::Parse(_))
            ),
            "version is required"
        );
    }

    #[test]
    fn validation_lists_every_problem() {
        let mut d = EngineDef::single();
        d.cylinders = 0;
        d.geometry.bore_m = -1.0;
        d.geometry.rod_m = 0.01; // shorter than the crank radius
        d.geometry.compression_ratio = 1.0;
        d.intake_valve.duration_deg = 0.0;
        d.redline_rpm = 0.0;
        let Err(ConfigError::Invalid(errs)) = validate(&d) else {
            panic!("expected invalid")
        };
        for needle in [
            "cylinders",
            "bore",
            "rod",
            "compression",
            "intake valve",
            "redline",
        ] {
            assert!(
                errs.iter().any(|e| e.contains(needle)),
                "missing '{needle}' in {errs:?}"
            );
        }
        assert!(validate(&EngineDef::lfa_v10()).is_ok());
    }

    #[test]
    fn validation_rejects_nan_and_absurd_values() {
        // Each of these used to pass validation and then run silent or never fire.
        type Mutate = Box<dyn Fn(&mut EngineDef)>;
        let cases: Vec<(&str, Mutate)> = vec![
            ("fmep", Box::new(|d| d.fmep_bar[0] = f64::NAN)),
            ("spark", Box::new(|d| d.spark_advance_deg = 900.0)),
            ("spark", Box::new(|d| d.spark_advance_deg = f64::NAN)),
            ("starter", Box::new(|d| d.starter_torque_nm = f64::NAN)),
            ("inertia", Box::new(|d| d.flywheel_inertia_kgm2 = 1e-12)),
            ("bore", Box::new(|d| d.geometry.bore_m = 1e-6)),
            (
                "compression",
                Box::new(|d| d.geometry.compression_ratio = 1e9),
            ),
            ("burn", Box::new(|d| d.burn_duration_deg = 1e9)),
            ("exhaust", Box::new(|d| d.exhaust.primary_length_m = 1e7)),
            (
                "exhaust",
                Box::new(|d| d.exhaust.tail_diameter_m = f64::NAN),
            ),
            (
                "intake valve",
                Box::new(|d| d.intake_valve.open_deg = f64::NAN),
            ),
            ("redline", Box::new(|d| d.redline_rpm = 1e9)),
        ];
        for (needle, mutate) in cases {
            let mut d = EngineDef::single();
            mutate(&mut d);
            let Err(ConfigError::Invalid(errs)) = validate(&d) else {
                panic!("{needle}: accepted")
            };
            assert!(
                errs.iter().any(|e| e.contains(needle)),
                "{needle}: {errs:?}"
            );
        }
    }

    #[test]
    fn version_1_files_migrate_to_the_exhaust_system() {
        // v1 (≤ v0.2.0) had one 0D runner per cylinder plus an outlet orifice.
        let v1 = "version: 1\nengine:\n  name: Old\n  cylinders: 4\n  exhaust_runner_length_m: 0.7\n  exhaust_runner_diameter_m: 0.041\n  exhaust_outlet_diameter_m: 0.052\n";
        let d = from_str(v1, Format::Yaml).unwrap();
        assert_eq!(d.name, "Old");
        assert_eq!(d.exhaust.primary_length_m, 0.7);
        assert_eq!(d.exhaust.primary_diameter_m, 0.041);
        assert_eq!(d.exhaust.tail_diameter_m, 0.052);
        let j = r#"{"version": 1, "engine": {"exhaust_runner_length_m": 0.9}}"#;
        assert_eq!(
            from_str(j, Format::Json).unwrap().exhaust.primary_length_m,
            0.9
        );
        // a v2 file must not use the old keys
        let bad = "version: 2\nengine:\n  exhaust_runner_length_m: 0.7\n";
        assert!(matches!(
            from_str(bad, Format::Yaml),
            Err(ConfigError::Parse(_))
        ));
    }

    #[test]
    fn v1_migration_merges_into_an_existing_exhaust_object() {
        let v1 = "version: 1\nengine:\n  exhaust_runner_length_m: 0.7\n  exhaust:\n    tail_length_m: 2.0\n";
        let d = from_str(v1, Format::Yaml).unwrap();
        assert_eq!(d.exhaust.primary_length_m, 0.7);
        assert_eq!(d.exhaust.tail_length_m, 2.0);
    }

    #[test]
    fn huge_versions_are_reported_as_written() {
        let e = from_str("version: 4294967297\nengine: {}\n", Format::Yaml).unwrap_err();
        assert_eq!(e, ConfigError::UnsupportedVersion(4_294_967_297));
    }

    #[test]
    fn collectors_must_partition_the_cylinders() {
        let mut d = EngineDef::lfa_v10();
        assert!(validate(&d).is_ok());
        d.exhaust.collectors = vec![vec![1, 3, 5, 7, 9], vec![2, 4, 6, 8]];
        assert!(validate(&d).is_err(), "cylinder 10 missing");
        d.exhaust.collectors = vec![vec![1, 3, 5, 7, 9, 2], vec![2, 4, 6, 8, 10]];
        assert!(validate(&d).is_err(), "cylinder 2 twice");
        d.exhaust.collectors = vec![];
        assert!(validate(&d).is_ok(), "empty = one collector for all");
    }

    #[test]
    fn typos_are_errors_not_silently_ignored() {
        let e = from_str("version: 1\nengine:\n  cylinder: 8\n", Format::Yaml).unwrap_err();
        assert!(
            matches!(&e, ConfigError::Parse(m) if m.contains("cylinder")),
            "{e:?}"
        );
        let e = from_str(
            "version: 1\nengine:\n  geometry: { bore: 0.09 }\n",
            Format::Yaml,
        )
        .unwrap_err();
        assert!(
            matches!(&e, ConfigError::Parse(m) if m.contains("bore")),
            "{e:?}"
        );
        let e = from_str("version: 1\nextra: 2\nengine: {}\n", Format::Yaml).unwrap_err();
        assert!(matches!(e, ConfigError::Parse(_)));
    }

    #[test]
    fn firing_order_must_be_a_permutation() {
        let mut d = EngineDef::inline4();
        d.firing_order = vec![1, 3, 3, 2];
        assert!(validate(&d).is_err());
        d.firing_order = vec![1, 3, 5, 2];
        assert!(validate(&d).is_err());
        d.firing_order = vec![];
        assert!(validate(&d).is_ok(), "empty = automatic");
        d.cylinders = MAX_CYLINDERS + 1;
        assert!(validate(&d).is_err());
    }

    #[test]
    fn default_firing_orders_are_permutations_and_classic() {
        assert_eq!(default_firing_order(4), vec![1, 3, 4, 2]);
        assert_eq!(default_firing_order(6), vec![1, 5, 3, 6, 2, 4]);
        for n in 1..=MAX_CYLINDERS {
            let mut o = default_firing_order(n);
            o.sort();
            assert_eq!(o, (1..=n).collect::<Vec<_>>(), "n = {n}");
        }
    }
}
