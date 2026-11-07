# Intake port exchange independent of the step — plan (draft, 2026-11-07)

**Status:** draft for the user's review; nothing implemented. Spec: the phased engine spec
(rotaries, `2026-10-31-rotary-design.md`); the defect is in FINDINGS 2026-11-07, "The rotaries'
port exchange depends on the step".

## Problem, measured

- The intake valve trades with its runner's last FV cell through `plan_exchange(.., VALVE_RELAX)`.
  Each step it moves at most half the mass that would equalise the cell and the chamber.
- A rotor's port (2250 mm² against a 45 mm runner) hits that cap at idle: about 2.4× at 96 kHz,
  about 1.2× at 192 kHz. A cap per step is a cap per second that scales as 1/dt, so the engine
  breathes differently at each quality.
- RENESIS at 96 kHz:
  - from the start: idles at 1317 rpm instead of 800, then stalls after a blip;
  - at 192 kHz: idles at 789 and recovers;
  - at 96 kHz with `VALVE_RELAX = 1`: also idles at 789 and recovers.
- The 13B bridge port with `VALVE_RELAX = 1` stalls. Its two open ports on a small chamber need
  the cap to keep the explicit coupling stable.
- Already in place:
  - `IntakeFv::coupled_to_port`: stiff ports step the runner on `STIFF_PORT_SUBSTEPS`.
  - `set_valve_boundary`: the FV right-end flux is capped at half the last cell.
  - `take_unserved`: what the valve took beyond the runner's reach goes back to the cylinder.

## Approach

The intake port becomes a boundary of its runner solved by characteristics. This is how the
exhaust valve already works (`valve_boundary`: Benson vol. 1 §6.6, Jenny's constant-pressure
outflow and the partially open end for inflow):

1. Outflow from the runner into the chamber uses the runner's incoming Riemann variable against
   the chamber's pressure, through the port's effective area. Inflow (reversion) uses the chamber
   as the upstream stagnation state, mirroring `valve_boundary::solve` with the roles of pipe and
   cylinder swapped.
2. The boundary's mass flux sets `Boundary::Flux` on the runner's right face for the FV step. It
   is integrated in the cylinder over the same sub-steps the FV uses, so both sides see one flux
   and nothing is capped per step.
3. 0D intake keeps `plan_exchange` with a large plenum, where the cap never binds.
4. Delete `VALVE_RELAX` on the intake path once the RENESIS, the bridge port and the R26B are
   stable at 96 and 192 kHz. Keep it where a 0D volume still trades with a cylinder.

## Tests (each seen failing first)

- `the_renesis_idles_alike_at_96_and_192_khz`: the mean idle and the post-blip low agree within
  5 %. Today: 1317 against 789, a stall against 541.
- `the_bridge_port_does_not_stall_at_96_khz`, and the same for the R26B: no stall over 12 s of
  idle and a blip.
- `an_intake_port_boundary_passes_the_isentropic_mass_flow_by_hand`: a steady runner state against
  a chamber at a hand-computed pressure ratio. The expected flow comes from the orifice relation,
  not from the code.
- Every existing breathing test (the dyno against published figures, the idle settles, the VE
  sweeps) passes unchanged at 192 kHz. A piston engine's sound moves less than 0.5 dB
  per third-octave (the cap never bound there; checked before and after).

## Risks

- This touches every engine's breathing. Before any merge, render clips of every preset (idle,
  blip, WOT) before and after, and get the user's ear.
- Hot path cost: one Newton solve per open intake valve per step, the exhaust's cost. Measure
  `--headless` timing on the LFA (Ultra) and on Low quality.
