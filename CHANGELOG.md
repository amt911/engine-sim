# Changelog — progress tags

Each tag on `main` is a runnable snapshot of the simulator (for the timelapse).
`git checkout <tag> && cargo run -p engine-app --release`.

| Tag | Date | What you can do |
| --- | --- | --- |
| `v0.0.1` | 2026-10-21 | Repo bootstrap: AGENTS.md, docs, design spec, empty Cargo workspace that builds |
| `v0.1.0` | 2026-10-21 | **First sound**: single, inline-4 and Lexus LFA V10 (published specs: 486 N·m / 531 hp vs real 480 / 552), animated cylinders, throttle (W), starter (S), ignition, spark advance, rpm/torque/power gauges, pressure trace, real-time audio |
