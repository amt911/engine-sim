# Transmissions and hybrid powertrains — design

Status: **approved by the user, 2026-11-04** ("he leído el spec, me parece bien"), with one
addition: **transmission noise** ("también tienes que incluir ruido de transmisión, por ejemplo, las
secuenciales hacen ruido"), see below. Q1 and Q2 take the proposed answers. Roadmap
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

**Q1 (answered: the proposal):** all six, in this order, each a PR, as each adds one box to the
existing drivetrain: **gear noise** (on the existing manual) → sequential → DSG → automatic → CVT →
eCVT.

## Transmission noise (user, 2026-11-04)

Every box makes its own sound, from its own mechanics, mixed with the engine's through the same
radiation path as the block (`structure`):

- **Gear whine:** the mesh's transmission error — the stiffness of the teeth in contact changing
  as the contact passes from one pair to the next — excites the box at the **mesh frequency**
  (teeth × shaft speed) and its harmonics, rising with load (Welbourn 1979; Smith, *Gear Noise and
  Vibration*, 2nd ed., ch. 2–4). Straight-cut (spur) gears have a contact ratio below 2, so the
  stiffness swings hard: the loud whine of a **sequential / dog box** or a reverse gear. Helical
  gears overlap many teeth: a road car's quiet box. Each gear pair has its tooth counts and helix
  angle in the file; the engaged pair and the final drive whine, at their own orders.
- **Gear rattle:** the unloaded gears on a manual's or a dog box's shafts hammer across their
  backlash, driven by the crank's speed fluctuation: impacts at idle and low load (Singh, Xie &
  Comparin, JSV 131(2), 1989). A dual-mass flywheel lowers it.
- **Shift noise:** a dog box's dogs crash in (an impact, no synchro); a synchro's cone grinds only
  when the driver forces it; a DSG's clutches hand over silently but its mechatronic pump hums.
- **CVT:** the push belt's or chain's elements entering the pulleys at their pitch frequency (the
  chain CVT's whine), changing with the ratio.
- **eCVT / hybrid:** the planetary's own mesh orders, under the motors' whine (below).

Anti-goal as for the motor: no sampled or hand-shaped gear sound; every tone at its mesh order,
every rattle an impact of the modelled gear.

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
  (engine as generator), power-split (eCVT above). **Q2 (answered: the proposal):** P2 parallel
  first — it reuses the stepped gearboxes.
- **UI:** a battery gauge (state of charge, power in/out), the motor's speed and torque, and the
  engine's on/off state; in the 3D view later, the motor between engine and gearbox.

## Goals, non-goals, anti-goals

- **Goals:** each transmission drivable from the keyboard as today (W/S, gear keys, a D/R/N
  selector for the automatics); the offline `gears`/`drive` scenes for each; hybrid presets (a
  Prius-like power-split, a P2 plug-in).
- **Non-goals:** battery thermal ageing; the gearbox's housing modes in detail (it radiates
  through the block's response).
- **Anti-goals:** sampled or synthesized-to-taste motor whine (it must come from the motor's
  electrical orders); a shift that only sounds right because of a hand-shaped volume envelope.

## Verification (each PR)

Unit tests with hand values (mesh frequencies from tooth counts and speed, spur vs helical
contact ratio, rattle only with backlash and speed fluctuation, planetary speed relations, converter K and ratio at stall and
coupling point, CVT ratio rate limit, battery SOC from a known current, PWM side-band
frequencies); a drive scene per transmission rendered and checked (shift times, rpm traces);
dyno/0–100 km/h within published figures where they exist; the user's ear on clips.

## Sources to cite (to read before building)

Welbourn, "Fundamental knowledge of gear noise — a survey", IMechE C117/79 (1979); Smith, *Gear
Noise and Vibration* (2nd ed., 2003); Singh, Xie & Comparin, "Analysis of automotive neutral gear
rattle", JSV 131(2) 1989; Kotwicki, "Dynamic models for torque converter equipped vehicles", SAE 820393; Guzzella &
Sciarretta, *Vehicle Propulsion Systems* (3rd ed.) ch. 4–5 (hybrids, power split); Naunheimer et
al., *Automotive Transmissions* (2nd ed.) (DCT, AT, CVT); Gieras, *Noise of Polyphase Electric
Motors* (magnetic and PWM noise); Plett, *Battery Management Systems* vol. 1 (equivalent
circuits).
