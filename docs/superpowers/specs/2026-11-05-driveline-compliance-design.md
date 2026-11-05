# An elastic driveline — design

Status: **draft for the user's approval** (written unattended, 2026-11-05). It changes how the car
feels to drive, so nothing is built before the user's yes.

## Why

The authenticity measurements (`scripts/authenticity.py`, `docs/FACTS.md`, 2026-11-04) show that
our renders repeat far more from one engine cycle to the next than real engines, between 250 Hz
and 4 kHz. One measured cause:

- Every cycle's combustion varies a little, so a crank that turns freely speeds up and slows down
  at random from cycle to cycle.
- K20A free-running (3377 rpm) against held at one speed (3200 rpm, 5 % load): free-running is
  7–10 dB more random.

  | Band | Free-running | Held |
  | --- | --- | --- |
  | 63–250 Hz | −17.4 dB | −24.5 dB |
  | 250 Hz–1 kHz | −10.2 dB | −20.1 dB |
  | 1–4 kHz | −7.3 dB | −10.6 dB |

**In gear, our crank behaves like the held one**: `drivetrain::Vehicle` adds the car's mass to the
crank as a rigid inertia (`(m + J_w/r²)·r²/G²`). With that much inertia, a cycle's varying impulse
cannot move the crank.

A real driveline is elastic: the half-shafts, tyres and clutch damper. Its first torsional mode,
"shuffle", sits at **2–7 Hz**:

- 2–7 Hz depending on the driveline (Farshidianfar et al., Int. J. Heavy Vehicle Systems 2002,
  core.ac.uk/download/9562189.pdf; "Driveline shuffle in rear wheel vehicles", Inderscience
  artid 1170, 2003);
- 1–10 Hz by gear and stiffness (Michigan Tech, "Control-oriented modeling of a vehicle drivetrain
  for shuffle and clunk mitigation").

Above that frequency the crank is cut off from the car's inertia. At 3000 rpm the engine cycle is
25 Hz, well above shuffle, so in gear the crank should vary nearly as freely as in neutral.

## The model

Two inertias joined by a spring and a damper, instead of one rigid inertia:

- **Engine side:** the crank and flywheel, as now.
- **Car side:** the car's reflected inertia `J_v = (m + J_w/r²)·r²/G²`, carrying the road load as
  now.
- **Coupling:** a torsional spring `k = k_w/G²` and damper, referred to the crank. `k_w` is the
  driveline stiffness at the wheels, with the half-shafts dominant.
  - The torque `T = k·(θ_e − θ_v) + c·(ω_e − ω_v)` loads the crank and drives the car.
  - With the clutch slipping, nothing changes: the clutch torque drives the car side directly, as
    today.

**Values.** No citable Nm/rad value was found tonight. So `k_w` is fitted to a stated shuffle
frequency, and that frequency is the editable field:

- `vehicle.shuffle_hz`, default 3.5 Hz in first gear: the middle of the published 2–7 Hz range.
  The default is an ESTIMATE until a measured value is in hand (PAPERS.md).
- `vehicle.shuffle_damping` (ζ), default 0.15: an ESTIMATE, to be cited before merge.

In higher gears `G` is smaller, so `k` and `J_v` change and the shuffle frequency comes out of the
same `k_w`. Real cars behave this way: the higher the gear, the higher the shuffle frequency.

## What the user would notice

- **In gear at steady speed:** the engine's sound gets the cycle-to-cycle variation it has when
  revving free, which the measurements say is missing.
- **On tip-in and lift-off:** a short fore-aft oscillation of a few hertz, as in a real car. The
  rpm needle and the sound wobble once or twice before settling. Real, but the user may dislike
  it. ζ sets how long it lasts.

## Tests (values from outside the code)

- The two-inertia system's natural frequency by hand: `f = (1/2π)·√(k·(1/J_e + 1/J_v))` for a
  stated `k`, `J_e` and `J_v`.
- Steady state: the twist equals the transmitted torque over `k`, and the car's speed equals the
  crank's over the ratio (no drift).
- Energy: with ζ = 0 and no load, spring plus kinetic energy stays constant within 0.1 % over
  10 s.
- Sound: in gear at 3000 rpm with 30 % load, the authenticity measure's 250 Hz–1 kHz share comes
  within 3 dB of the free-running engine at the same speed. If it does not, stop and report.
- Shifts: the existing shift and thump tests stay green unchanged.

## Out of scope

Clutch-damper springs, dual-mass flywheels and tyre slip dynamics are later refinements, not
needed to give the crank its freedom.
