//! Engine files: YAML / JSON (de)serialization of [`EngineDef`] with a versioned envelope,
//! and validation. Pure string ↔ data; file IO lives in the app.

use crate::engine::EngineDef;
use serde::{Deserialize, Serialize};

/// Current file format version. Bump on a breaking change and add a migration + test.
pub const FORMAT_VERSION: u32 = 1;
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
    UnsupportedVersion(u32),
    #[error("invalid engine: {}", .0.join("; "))]
    Invalid(Vec<String>),
}

#[derive(Serialize, Deserialize)]
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
    // Read the version first so a newer file says so instead of failing on a field.
    #[derive(Deserialize)]
    struct Header {
        version: u32,
    }
    let parse = |e: String| ConfigError::Parse(e);
    let header: Header = match format {
        Format::Yaml => serde_yaml_ng::from_str(text).map_err(|e| parse(e.to_string()))?,
        Format::Json => serde_json::from_str(text).map_err(|e| parse(e.to_string()))?,
    };
    if header.version == 0 || header.version > FORMAT_VERSION {
        return Err(ConfigError::UnsupportedVersion(header.version));
    }
    let file: EngineFile = match format {
        Format::Yaml => serde_yaml_ng::from_str(text).map_err(|e| parse(e.to_string()))?,
        Format::Json => serde_json::from_str(text).map_err(|e| parse(e.to_string()))?,
    };
    validate(&file.engine)?;
    Ok(file.engine)
}

/// Every problem found, so the editor can show them all at once.
pub fn validate(def: &EngineDef) -> Result<(), ConfigError> {
    let mut errs = Vec::new();
    let mut check = |ok: bool, msg: &str| {
        if !ok {
            errs.push(msg.to_string());
        }
    };
    let g = &def.geometry;
    let pos = |x: f64| x.is_finite() && x > 0.0;
    check(
        (1..=MAX_CYLINDERS).contains(&def.cylinders),
        "cylinders must be between 1 and 16",
    );
    check(pos(g.bore_m), "bore must be positive");
    check(pos(g.stroke_m), "stroke must be positive");
    check(
        g.rod_m.is_finite() && g.rod_m > g.stroke_m * 0.5,
        "rod must be longer than half the stroke",
    );
    check(
        g.compression_ratio.is_finite() && g.compression_ratio > 1.0,
        "compression ratio must be above 1",
    );
    for (name, v) in [
        ("intake valve", &def.intake_valve),
        ("exhaust valve", &def.exhaust_valve),
    ] {
        let ok = v.duration_deg > 0.0
            && v.duration_deg < 720.0
            && pos(v.max_lift_m)
            && pos(v.diameter_m)
            && v.open_deg.is_finite();
        check(
            ok,
            &format!("{name}: duration in (0, 720)°, lift and diameter positive"),
        );
    }
    check(pos(def.burn_duration_deg), "burn duration must be positive");
    check(
        pos(def.flywheel_inertia_kgm2),
        "flywheel inertia must be positive",
    );
    check(
        pos(def.plenum_volume_m3) && pos(def.throttle_diameter_m),
        "plenum volume and throttle diameter must be positive",
    );
    check(
        (0.0..=1.0).contains(&def.idle_throttle),
        "idle throttle must be in [0, 1]",
    );
    check(
        pos(def.exhaust_runner_length_m)
            && pos(def.exhaust_runner_diameter_m)
            && pos(def.exhaust_outlet_diameter_m),
        "exhaust runner and outlet sizes must be positive",
    );
    check(pos(def.redline_rpm), "redline must be positive");
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
        assert!(y.starts_with("version: 1\n"), "{y}");
        let j = to_string(&EngineDef::single(), Format::Json).unwrap();
        assert!(j.contains("\"version\": 1"), "{j}");
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
