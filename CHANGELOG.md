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
| `v0.7.0` | 2026-10-22 | **1D intake**: `intake.model: fv` — runners with trumpets, from individual throttle bodies or a plenum (editor: model, ITB, runner length/Ø); the LFA now breathes through ten ITBs (intake ram and runner resonance); junction inflow from stagnation |
| `v0.8.0` | 2026-10-22 | **V8s and layouts**: flat-plane (European) and cross-plane (American) V8 presets, so you can hear the burble vs. the scream; bank angle and banks, V and flat engines drawn tilted; a listener position that hears each tail pipe with its own delay and distance; audible idle (voice leveler); no more "air" hiss at high rpm |
| `v0.9.0` | 2026-10-22 | **ECU idle control**: every engine idles at its own speed (editor: *Idle speed*) through an idle-air PI and a spark reserve, so the single no longer stalls and an engine survives a blip; one spark per charge whatever the timing does; Woschni wall heat; throttle actuator (0.1 s) and dashpot, so no click on tip-in |
| `v0.9.1` | 2026-10-22 | **Overrun without "air"**: exhaust sources radiate volume (not mass) flow, primaries carry their own gas temperature, the FV exhaust valve no longer chatters on overrun |
| `v0.10.0` | 2026-10-22 | **Revs like the real thing**: rotating inertia from measured data (LFA idle → 9000 rpm in 0.6 s), ECU fuel cut-offs with hysteresis (rhythmic limiter, back to idle after a lift-off), dashpot in plate travel; spectrogram diagnostics (`drive_render`, `scripts/spectrogram.py`) |
| `v0.10.1` | 2026-10-22 | **Import keeps the vehicle**: an engine file carries its own gearbox and vehicle; loading a YAML no longer inherits the selected preset's gears |
| `v0.11.0` | 2026-10-23 | **Sharper 1D waves**: WENO5-Z + SSP-RK3 finite volumes (a sound pulse keeps 89% of its peak instead of 57%, fifth order in space); sound scorecard (`scripts/scorecard.sh`) to measure every change |
| `v0.11.1` | 2026-10-23 | **Steady FV junctions**: the junction root finder no longer gives up beside an infinite slope, so still gas stays still in the non-linear exhaust (`exhaust.model: fv`) |
| `v0.12.0` | 2026-10-23 | **Pascals at the listener**: the tails radiate as monopoles to a listener at a distance and angle (delay, 1/r), turbulent jet noise from each tail (SAE ARP876 level, Strouhal-peaked), no output leveler |
| `v0.12.1` | 2026-10-23 | **Lighter physics**: 29 % less CPU per audio second, same sound (fewer `pow`, the exhaust port's solve starts from its last flow) — the LFA V10 back above real time |
| `v0.13.0` | 2026-10-23 | **Quality levels**: Auto/Low/Medium/High/Ultra (panel or `--quality`); Auto benchmarks the engine and keeps ≥ 1.5× real time, and steps down if it falls behind; Medium steps 1D pipes with MUSCL-RK2, Low trades them for 0D/waveguides |
