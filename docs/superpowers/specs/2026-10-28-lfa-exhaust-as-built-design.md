# LFA exhaust as built — design

Date: 2026-10-28. Status: roadmap phase 2 (realism), delegated by the user; this document records the
calls made. Branch: `feat/exhaust-bank-merge` (PR #71, reworked).

## Why

The listener hears either two 5-cylinder engines (v0.27.1: two unmixed bank tails, half order +15 dB
over the firing order) or a pure firing-order whine (PR #71: perfectly merged banks, half orders
−33…−61 dB). Real LFA recordings sit at −13.5…−22 dB (three recordings, four conditions; agent2
report 2026-10-28). The preset also has no main muffler, so the overrun shock that wave steepening
produces is radiated raw. That shock is what blocks `feat/nonlinear-delay`, the measured fix for the
near-sine exhaust (H2–H6 within ~3 dB of the real tip with steepening).

## What the real car has (sources)

- Okamoto et al., SAE 2011-01-2024, Fig. 19: per bank an equal-length 5-into-1 manifold, a front TWC
  and a rear TWC (metal substrate, few cells, large), planar A/F and O₂ sensors.
- Toyota press release 2009-10-21: "equal-length exhaust manifolds combined with an equal-length
  dual exhaust system routed through a multistage titanium main muffler"; exhaust pipes stacked
  with the torque tube in the centre tunnel.
- Toyota EPC illustration 17410/17430 (user, 2026-10-28) and Motor-Fan cutaway (user): the two bank
  pipes join in a Y at the muffler inlet. The transverse main muffler has a centre chamber, two end
  chambers reached through tubes (flow reverses), a vacuum-actuated bypass valve (17230K/17338):
  open, the centre chamber feeds the three tips directly; closed, the gas goes round the end
  chambers and back.

## Model

1. **Bug first** (PR #71): the listener and tail jets take the outlet bore from the path's last
   pipe instead of `merge.outlet_diameter_m` (engine.rs listener build; −6.02 dB). Regression test:
   two in-phase 60 mm tails and one outlet of twice the area radiate the same volume flow below the
   first resonance.
2. **Per-bank path, as built**: manifold → front TWC → rear TWC → pipe to the Y (existing
   `Segment`s). The two banks' paths are equal by design; their *gas temperatures* are not
   identical (separate fuel control per bank, Okamoto §7). Each bank's waveguide delay uses its own
   mean gas temperature (already computed per primary), so the bank-difference mode arises from
   physics, not from a fixed offset. If the measured per-bank spread stays < 0.5 %, the design
   records it and does not invent one.
3. **Muffler as a small acoustic network** after the Y (`Merge` generalised):
   - centre chamber (expansion), three tip pipes radiating from their own positions (listener sums
     three monopoles at their offsets, as for the old tail pair);
   - valve-closed path: two extended-tube reversing chambers (Munjal ch. 3, transfer matrices
     realised as waveguide sections with area jumps and end corrections);
   - bypass valve as a variable-area orifice between centre chamber and tips (quasi-steady
     orifice, the same model as the throttle), opening by engine speed at 3000 rpm: Lexus UK
     media "Sounds Spectacular" (valve-actuated dual-stage titanium silencer that changes flow
     with engine speed), Torque.com.sg and Adrian Flux (closed below 3000 rpm, then the gas
     bypasses the chambers into one resonance chamber). The 3000 rpm figure is press, not a
     Lexus specification; the actuator travel time (vacuum diaphragm) is an estimate.
4. **Dimensions**: unpublished. Estimated from the cutaway against known parts (tip and pipe
   bores, the 1895 mm body width) and marked ESTIMATE in the preset with how they were derived.

## Not in scope

Steepening (`feat/nonlinear-delay`) is the next branch; this one only makes it possible. Intake
air path, valve-jet noise and the listener ground model are later items of the agent2 ranking.

## Verification

- Unit: transmission loss of one reversing chamber against Munjal's closed form; bypass orifice
  flow against the throttle model; radiation bug regression.
- Measured on the LFA (probe at the default listener and 0.5 m from a tip, 7400 rpm WOT and decays):
  half orders −13…−22 dB re firing; overrun at least 8 dB below WOT at the same rpm with the valve
  closed; firing order within 2 dB of v0.27.1 at WOT.
- Dyno: `the_lfa_breathes_as_lexus_published` still passes (back-pressure of the muffler).
- Performance: LFA at High ≤ +5 % CPU.
