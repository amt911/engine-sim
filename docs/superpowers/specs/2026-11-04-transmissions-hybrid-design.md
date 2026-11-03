# Transmissions and hybrid powertrains — design (DRAFT for the user's review)

Status: **draft, not approved.** Written unattended on the night of 2026-11-03 so the user can
react to something concrete; nothing is built until the user approves it (or changes it). Roadmap
item from the user, 2026-10-31: "more transmission types (manual, sequential, automatic, DSG, CVT,
eCVT…) and hybrid powertrains whose electric noise is audible, with a simulated battery shown in
the UI — leave these for the end". The questions marked **Q** are the user's to answer.

## Intent

Every way an engine can reach the wheels should sound and feel like it does in a car: a manual's
clutch and lift-off, a sequential's flat-shift crack, a DSG's torque-filled upshift, a
torque-converter automatic's slur and creep, a CVT's engine held at one speed while the car
accelerates (the "rubber band"), and a hybrid's engine starting and stopping under an electric
motor's whine — with the battery's charge visible. Physics first: each box is a model with cited
constants, not a sound effect.

## What exists

`engine-core::drivetrain`: a vehicle (mass, tyre, drag, rolling), a friction clutch (slip/lock),
a stepped gearbox with a final drive, the driver's gear lever, clutch pedal and brake; the
offline `gears` scene shifts through the gears; the ECU's torque cut on upshift.

## Proposed transmissions (each a `transmission:` kind in the vehicle file; the current gearbox
becomes `manual`)

| Kind | Model | What you hear / feel |
| --- | --- | --- |
| **Manual** (exists) | friction clutch + gear pairs, driver shifts | lift, clutch, rev-match |
| **Sequential** | dog box: no synchro, shift by an actuator in ~50 ms with an ignition cut (flat shift) | crack of the cut, no lift |
| **DSG (dual clutch)** | two wet clutches, odd/even shafts; the next gear pre-selected, torque handed over in ~200 ms (one clutch slips off as the other slips on); launch control | seamless upshift with a torque dip, a "fart" on WOT upshifts (cut + retard) |
| **Torque-converter automatic** | Kotwicki's torque-converter model (capacity factor K and torque ratio vs speed ratio), planetary steps, lock-up clutch, shift schedule by load and speed | slurred shifts, creep at idle, flare on kick-down |
| **CVT** | belt/pulley ratio as a state with a rate limit; a controller holding the engine at its best-efficiency or best-power speed | engine speed steady while the car speeds up |
| **eCVT (power-split hybrid)** | Toyota THS planetary: engine, MG1, MG2 and the wheels through one planetary set (Willis equation) | engine speed decoupled from road speed, MG whine |

**Q1:** all six, or a first set? (Proposed order: sequential → DSG → automatic → CVT → eCVT, each a
PR, as each adds one box to the existing drivetrain.)

## Hybrid

- **Electric machine:** a permanent-magnet synchronous motor with a torque–speed envelope
  (constant torque to base speed, constant power above), efficiency map (ESTIMATE from published
  Prius/Golf GTE data), and its **sound physically**: the magnetic force harmonics at the motor's
  electrical orders (pole pairs × shaft speed, slot harmonics) and the inverter's PWM carrier and
  its side bands (switching frequency ± 2·electrical frequency), radiated through a housing
  response — the whine that rises with speed.
- **Battery:** an equivalent circuit (open-circuit voltage vs state of charge, internal
  resistance, one RC pair), current from the motor's electrical power, state of charge integrated;
  limits on charge/discharge power.
- **Energy manager (simulated ECU):** engine start/stop by power demand and charge, regenerative
  braking, electric-only crawl, boost on WOT.
- **Layouts:** parallel P2 (motor between engine and gearbox: mild/plug-in hybrids), series
  (engine as generator), power-split (eCVT above). **Q2:** which first? (Proposed: P2 parallel —
  it reuses the stepped gearboxes.)
- **UI:** a battery gauge (state of charge, power in/out), the motor's speed and torque, and the
  engine's on/off state; in the 3D view later, the motor between engine and gearbox.

## Goals, non-goals, anti-goals

- **Goals:** each transmission drivable from the keyboard as today (W/S, gear keys, a D/R/N
  selector for the automatics); the offline `gears`/`drive` scenes for each; hybrid presets (a
  Prius-like power-split, a P2 plug-in).
- **Non-goals:** detailed gear rattle and whine of the gear teeth (a later realism item);
  battery thermal ageing.
- **Anti-goals:** sampled or synthesized-to-taste motor whine (it must come from the motor's
  electrical orders); a shift that only sounds right because of a hand-shaped volume envelope.

## Verification (each PR)

Unit tests with hand values (planetary speed relations, converter K and ratio at stall and
coupling point, CVT ratio rate limit, battery SOC from a known current, PWM side-band
frequencies); a drive scene per transmission rendered and checked (shift times, rpm traces);
dyno/0–100 km/h within published figures where they exist; the user's ear on clips.

## Sources to cite (to read before building)

Kotwicki, "Dynamic models for torque converter equipped vehicles", SAE 820393; Guzzella &
Sciarretta, *Vehicle Propulsion Systems* (3rd ed.) ch. 4–5 (hybrids, power split); Naunheimer et
al., *Automotive Transmissions* (2nd ed.) (DCT, AT, CVT); Gieras, *Noise of Polyphase Electric
Motors* (magnetic and PWM noise); Plett, *Battery Management Systems* vol. 1 (equivalent
circuits).
