# Changelog — progress tags

Each tag on `main` is a runnable snapshot of the simulator (for the timelapse).
`git checkout <tag> && cargo run -p engine-app --release`.

| Tag | Date | What you can do |
| --- | --- | --- |
| `v0.0.1` | 2026-10-21 | Repo bootstrap: AGENTS.md, docs, design spec, empty Cargo workspace that builds |
| `v0.1.0` | 2026-10-21 | **First sound**: single, inline-4 and Lexus LFA V10 (published specs: 486 N·m / 531 hp vs real 480 / 552), animated cylinders, throttle (W), starter (S), ignition, spark advance, rpm/torque/power gauges, pressure trace, real-time audio |
| `v0.2.0` | 2026-10-21 | **Editor + files**: every engine parameter as a slider (mm, cc, °, L), firing order field with validation, save/load YAML or JSON (dialogs or drag-and-drop), `--engine`/`--save` CLI, preset files in `presets/` |
| `v0.3.0` | 2026-10-21 | **Exhaust waveguides**: primaries, collectors (LFA: two 5-into-1 banks), tail pipes, radiating open ends; implicit valve-port coupling; exhaust editor section; file format v2 (v1 files migrate) |
| `v0.4.0` | 2026-10-22 | **Drivetrain**: clutch (slip/lock with overload hysteresis), gearbox, vehicle with aero/rolling/brake, speedometer; dyno sweep (`--dyno`, GUI plot) |
| `v0.5.0` | 2026-10-22 | **Sound texture**: turbulent flow noise ∝ outflow, induction roar, valvetrain clicks, cycle-to-cycle combustion variability, body/room reverb; Sound section in the editor |
| `v0.6.0` | 2026-10-22 | **Non-linear 1D exhaust (option)**: `exhaust.model: fv` — finite-volume pipes (HLLC/MUSCL, quasi-1D areas, wall friction), characteristic junctions, muffler can; overrun fuel cut and ECU dashpot; reverse-flow Cd |
