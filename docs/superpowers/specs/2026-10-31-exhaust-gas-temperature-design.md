# Exhaust gas temperature along the pipes — design

Status: written unattended (user asleep, roadmap delegated 2026-10-31: "continúa esta noche con
cosas del roadmap"). It is item 2 of the user's Ange physics list (2026-10-26: heat transfer along
the pipes; Ange: "sounds better" once corrected). Decisions marked *(my call)* are open to the
user's review.

## Why

The waveguide exhaust has three fixed speeds of sound: 600 m/s in the primaries, 560 in the
collectors and 500 after them (`exhaust.rs`, marked "estimates; c(T) from the gas state is a
later refinement"). A real exhaust's gas temperature depends on load and speed. It is about
900–1100 K at the port at full load and hundreds of kelvin less at idle (the cylinder model gives the port value), and it cools
along the pipe toward the wall. So the pipes' resonances move with load by up to about 20 %
(c ∝ √T), and a long thin pipe is colder than a short fat one. With fixed speeds, every
operating point sounds tuned to one temperature.

## Goals

- Each pipe's speed of sound follows its gas temperature, from the port's measured outflow
  temperature cooled along the system by convection to the wall.
- The walls have thermal inertia. A pull heats the pipes over seconds, and a lift-off pumps air
  that the hot walls warm. The sound must not jump when the fuel is cut.
- Deterministic, no allocation on the sim thread, the cost well under 1 % of a step (the update
  runs on the mean-flow tick, every 64 steps).

## Non-goals

- Temperature along a pipe inside the wave equation (the FV exhaust's job). Each pipe has one
  speed, from its mean temperature.
- Wall-filter refits: the Kirchhoff √f wall shelves keep their build-time fit *(my call: refitting
  costs ~10⁵ evaluations; the error is the √c dependence of α, a few % of a few dB)*.
- Impedance jumps from temperature at junctions (ρc = γp/c differs between pipes). Junctions keep
  scattering by area only *(my call; next step if the option earns its place)*.

## Anti-goals

- Changing the LFA the user approved on 2026-10-31. The option is **off by default**
  (`exhaust.gas_temperature: false`). With it off, every render is bit-identical to v0.30.0.

## Physics (cited)

- **Port temperature:** the cycle-mean enthalpy flux over the cycle-mean mass flux of each
  exhaust valve's outflow (the cylinder model already hands each flow's specific enthalpy `h`).
  It is smoothed over a cycle like `cyl_mean_flow`.
- **Inside heat transfer:** Dittus–Boelter, Nu = 0.023 Re^0.8 Pr^0.3 for gas being cooled
  (Incropera & DeWitt, *Fundamentals of Heat and Mass Transfer*, eq. 8.60), Re = 4ṁ/(πDμ), and
  k = μc_p/Pr. Below Re 2300 the laminar Nu = 3.66 (eq. 8.55) applies. Pulsating exhaust flow
  transfers more heat than steady flow; that factor is left at 1 *(my call; cited range 1–3 is
  too wide to pick blind)*.
- **Gas along a pipe at wall temperature T_w:** T(x) = T_w + (T_in − T_w)·e^(−x/ℓ), with
  ℓ = ṁc_p/(hπD) (Incropera eq. 8.42b, constant surface temperature). The pipe's mean
  temperature is T_w + (T_in − T_w)(1 − e^(−N))/N, where N = L/ℓ, and its outlet temperature
  is T_w + (T_in − T_w)e^(−N).
- **Wall:** a lumped mass. Thin steel wall (the header question below, 1.5 mm *(my call)*),
  ρ 7900 kg/m³, c 477 J/(kg·K) (Incropera Table A.1, AISI 304). Its energy balance per unit area:
  m″c·dT_w/dt = q″_gas − h_o(T_w − T_∞) − εσ(T_w⁴ − T_∞⁴). The outside convection h_o is
  15 W/(m²·K), a free-to-light-forced value (Incropera Table 1.1, gases 2–25 free, 25–250
  forced) *(my call)*. The emissivity ε is 0.8, oxidized steel (Incropera Table A.11).
  T_∞ = 300 K. The wall starts at the steady state of the first operating point, so a render
  does not start cold.
- **Junctions:** the gas mixes. The outlet temperature is the mass-weighted mean of what flows in.
- **Speed of sound:** c = √(γRT_mean).

## Shape

- `ExhaustDef::gas_temperature: bool`, default false, serde, round-trip tested.
- `crate::pipe_heat`: pure functions (h, ℓ, mean and outlet temperatures) and a `Wall` state.
  Unit-tested against hand calculations.
- `DelayLine::set_delay` / `Pipe::set_sound_speed`. A line built for gas temperature keeps room
  for the coldest gas (300 K, about 1.4× the 600 m/s delay). Linear lines read at the new delay.
  Non-linear lines send new values at the new delay; values in flight keep theirs (the change
  per tick is a tiny fraction of a step, so nothing overtakes).
- `ExhaustNet::set_gas` walks primaries → collectors → their ducts → the shared duct, the bypass
  paths and the tips, in the order the gas flows, carrying the outlet temperature and the mean
  flow. It is called on the mean-flow tick with the port temperatures and the time since the last
  tick.
- A net with gas temperature on is not handed to another thread (`max_block` 0, as for a
  steepening net) *(my call: the probe would need the walls' state too)*.

## Done when

- Unit tests for the heat functions (hand values), `set_delay` (an impulse arrives at the new
  delay), the net's temperatures (a primary with no flow sits at the wall's temperature; a longer
  pipe exits colder; the wall warms over seconds after a step in port temperature).
- With the option on, the LFA's primaries sit at 850–1050 K at full load and 8000 rpm, and lower
  at idle. With it off, renders are bit-identical to main.
- Gate green, PR with a manual test plan and the verification comment. A/B clips (off/on) for the
  user, gears and idle.
