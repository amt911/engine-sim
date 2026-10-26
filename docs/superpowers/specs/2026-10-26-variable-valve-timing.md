# Variable valve timing: VTEC lobes and cam phasers

Date: 2026-10-26. Status: design. Requested by the user ("un sistema VTEC/VVT-i o similar, para
trastear con configuraciones raras"; "sobre todo para el de 4 cilindros Honda").

## Why

Real engines change their valve events with speed and load:
- **Honda VTEC:** two cam lobes per valve, the high one locked in by oil pressure above a speed.
- **Toyota VVT-i, Honda VTC:** a hydraulic phaser turns a camshaft continuously, following an
  ECU map.

The preset K20A (DC5 Integra Type R) has both on its intake: VTEC at 6000 rpm, and VTC. The LFA
has dual VVT-i (intake and exhaust).

Today every valve has one fixed `ValveTiming`, so none of their character (the K20A's VTEC "step"
in torque and sound) can appear. The valve events already drive the port flow, so the physics
follows once the events move.

## Model (data: engine definition, serde, every field defaulted)

- `ValveTiming::high: Option<CamLobe>`: a second lobe (opening, duration, lift) used while VTEC is
  engaged. The valve's diameter is shared. With no `high`, the valve has one lobe, as today.
- `EngineDef::vtec: Option<Vtec>` with:
  - `engage_rpm` and `hysteresis_rpm`: engaged at or above `engage_rpm`, disengaged below
    `engage_rpm − hysteresis_rpm`;
  - `min_throttle`: the pedal it needs. A real VTEC solenoid also waits for load, oil pressure
    and coolant temperature; only the pedal is modelled.
- `EngineDef::intake_phaser`, `EngineDef::exhaust_phaser`: `Option<Phaser>` with:
  - `range_deg`: crank degrees it can advance the cam;
  - `map`: points of (rpm, advance at full load, advance at no load), interpolated in speed and
    linearly in pedal;
  - `rate_deg_s`: how fast the cam can turn.

  An advance opens and closes the valve earlier by the same angle. The cam's profile is
  unchanged.
- **Runtime.** The engine holds the VTEC state and each phaser's angle. Each step the phaser
  slews toward its map's target at `rate_deg_s`, clamped to [0, range].
  - The effective timing of each valve is its lobe (high while VTEC is engaged), opened
    `advance` degrees earlier.
  - It feeds every user of the valve events: flow area, IVC/EVC angles, the exhaust-opening
    events and the view.

## Sources for the numbers

- VVT-i range up to 60° CA: Toyota, SAE 960579.
- Phaser rate 25–40 °/s: rate limits in control patents (US 7,467,611); 24° in about 0.9 s
  (US 7,409,935).
- K20A (JDM ITR):
  - VTEC at 6000 rpm, 217 hp at 8000, 206 N·m at 7000 (Wikipedia, Honda K engine).
  - Cam lobes, low 244°/240° and 8.84/8.59 mm, VTEC 280°/278° and 12.65/12.14 mm
    (motorreviewer, secondary; to replace with the service manual, PAPERS.md).
- VTC at full load sits where it "will optimize the torque in response to engine speed" (Niizato &
  Hayashi, Honda R&D Technical Review 12-2, 2000, §6.1). The preset's full-load map is this model's
  own torque optimum: dyno sweeps at constant advances of 0–50° in 10° steps, low lobe below 6000
  rpm and VTEC lobe above. The two lobes' best curves cross near 6000 rpm, where the real engine
  switches.
- The real K20A switches its exhaust lobes too (Acura RSX service manual: exhaust mid and
  secondary lobes). With the secondary source's exhaust timing, the VTEC exhaust lobe lost torque at
  every speed in this model, so the preset keeps one exhaust lobe until a primary source gives its
  timing. The model supports a `high` lobe on either valve.
- LFA: dual VVT-i, intake and exhaust (Motor-Fan 2009, Yamaha interview; ja.wikipedia). Its ranges
  are not published, so the preset keeps fixed cams until a source appears.

## Tests (values by hand or from the sources)

- **Lobes.** With VTEC engaged a valve uses its high lobe: lift and area at mid-opening are the
  high lobe's, by hand from the sin² profile.
- **Phase.** An advance of a° moves the whole lift curve a° earlier: lift(θ) advanced equals
  lift(θ + a).
- **VTEC switching.** Engaged at `engage_rpm` with the pedal down, held down to
  `engage_rpm − hysteresis`, released below it, and never engaged under `min_throttle`.
- **Phaser.** Slews at `rate_deg_s`: from 0 to a 30° target at 30 °/s takes 1 s. The target is
  the map's interpolation, by hand; it is clamped to the range.
- **Files.** JSON and YAML round-trip. An old file with no VVT fields loads with none.
- **Physics.**
  - The K20A at full load makes more torque at 7000 rpm on the high lobe than forced onto the low
    lobe, and less at 3000 rpm: the trade VTEC exists for.
  - Its dyno peak stays within 10 % of the published 206 N·m, and holds within 10 % of that peak
    into VTEC's range at 6500 rpm. The torque peak lands at 4000 rpm, not Honda's 7000: VTC lifts
    the mid-range, and above 7000 rpm the model breathes less than the engine (power ~16 % short
    at 8000, as before VVT).

## Out of scope (later)

- The K20A's exhaust VTEC lobe in the preset (the model has it; the timing has no primary source).
- Continuously variable lift (Valvetronic).
- Cam-profile editing beyond the three numbers per lobe.
- Oil-pressure dynamics of the solenoid.
