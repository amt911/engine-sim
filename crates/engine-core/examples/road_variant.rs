//! Probe helper: a preset with the rule-sized road exhaust ([`engine_core::exhaust::road_exhaust`])
//! after each collector, written as YAML.
//! `cargo run -p engine-core --release --example road_variant -- IN.yaml OUT.yaml`
use engine_core::config::{Format, from_str, to_string};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut def =
        from_str(&std::fs::read_to_string(&a[1]).expect("in"), Format::Yaml).expect("yaml");
    let branches = def.exhaust.groups(def.cylinders as usize).len().max(1);
    let swept = def.displacement_m3() / branches as f64;
    let d = def.exhaust.tail_diameter_m;
    def.exhaust.after_collector = engine_core::exhaust::road_exhaust(swept, d);
    std::fs::write(&a[2], to_string(&def, Format::Yaml).expect("write")).expect("out");
}
