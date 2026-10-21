//! Engine files on disk: load/save by extension (YAML or JSON).

use engine_core::config::{self, Format};
use engine_core::engine::EngineDef;
use std::path::Path;

fn format_of(path: &Path) -> Result<Format, String> {
    Format::from_path(&path.to_string_lossy())
        .ok_or_else(|| format!("{}: use a .yaml, .yml or .json file", path.display()))
}

pub fn load_engine(path: &Path) -> Result<EngineDef, String> {
    let format = format_of(path)?;
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    config::from_str(&text, format).map_err(|e| format!("{}: {e}", path.display()))
}

pub fn save_engine(path: &Path, def: &EngineDef) -> Result<(), String> {
    let format = format_of(path)?;
    let text = config::to_string(def, format).map_err(|e| e.to_string())?;
    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controls::Preset;

    fn tmp(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("engine-sim-files-{}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        d.join(name)
    }

    #[test]
    fn save_then_load_both_formats() {
        for name in ["v10.yaml", "v10.json"] {
            let p = tmp(name);
            save_engine(&p, &EngineDef::lfa_v10()).unwrap();
            assert_eq!(load_engine(&p).unwrap(), EngineDef::lfa_v10());
        }
    }

    #[test]
    fn errors_name_the_problem() {
        let e = load_engine(Path::new("/definitely/not/here.yaml")).unwrap_err();
        assert!(e.contains("here.yaml"), "{e}");
        let e = save_engine(&tmp("x.txt"), &EngineDef::single()).unwrap_err();
        assert!(e.contains(".yaml") && e.contains(".json"), "{e}");
        let p = tmp("bad.yaml");
        std::fs::write(&p, "version: 1\nengine:\n  cylinders: 0\n").unwrap();
        assert!(load_engine(&p).unwrap_err().contains("cylinders"));
    }

    #[test]
    fn shipped_preset_files_match_the_built_ins() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../presets");
        for p in Preset::ALL {
            let file = dir.join(format!("{}.yaml", p.slug()));
            assert_eq!(
                load_engine(&file).unwrap(),
                p.def(),
                "{} is stale: regenerate with --save",
                file.display()
            );
        }
    }
}
