# engine-sim

Real-time engine sound simulator in Rust: build an engine visually, watch it run, hear the physics.

```bash
cargo run -p engine-app --release                                  # opens the app
cargo run -p engine-app --release -- --engine presets/lfa-v10.yaml # load an engine file
```

Keys: **W** / Up = full throttle, **S** = starter. Edit the engine in the left panel; save or load
it as YAML/JSON (or drop a file on the window).

- Wishes: [`docs/specs.md`](docs/specs.md) · Design: [`docs/superpowers/specs/`](docs/superpowers/specs/)
- Research (physics, DSP, papers): [`docs/research/simulador-motor-rust.md`](docs/research/simulador-motor-rust.md)
- Progress tags: [`CHANGELOG.md`](CHANGELOG.md)
- Agents: [`AGENTS.md`](AGENTS.md)

Linux deps: `alsa-lib` (Arch) / `libasound2-dev` (Debian/Ubuntu).
