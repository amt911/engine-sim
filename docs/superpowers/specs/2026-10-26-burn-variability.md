# Cycles vary as much as measured engines' do

Date: 2026-10-26. Status: design. Roadmap: sound realism ("too synthetic").

## Why

Each cycle's flame sets each exhaust pulse. A real engine's cycles differ, so its sound is not a
loop. Measured on the LFA's idle (v0.22.1 render vs the clean recording wR8FwJVFXOQ), the share
of each octave band that repeats from one cycle to the next (synchronous average over 4–8
cycles, tracked cycle by cycle):

| band | 125 Hz | 250 Hz | 500 Hz | 1 kHz | 2 kHz | 4 kHz |
|---|---|---|---|---|---|---|
| real LFA | 0.52 | 0.50 | 0.19 | 0.02 | 0.00 | 0.00 |
| model (v0.22.1) | 0.99 | 0.98 | 0.90 | 0.70 | 0.45 | 0.22 |

Above 500 Hz the real engine does not repeat at all; the model repeats most of it. The cause is
in the flame: its burn angles varied 3 % (σ/mean) cycle to cycle, where measured engines vary
about 12 % (Heywood Fig. 9-35, flame development and rapid-burning angles, many chambers and
conditions). The turbulence draw at the plug was 3 %; Heywood §8.2 (Fig. 8-9) measures the
cycle-to-cycle variation of the mean flow at a point as comparable to the turbulence intensity
itself, and Ozdor, Dulger & Sher (SAE 940987) find the kernel's development, carried by that
flow, dominating cyclic variability.

## Model

- **Turbulence at the plug:** σ 0.175 of u′, set where the burn angles' σ/mean comes out at
  Heywood's 0.12 (with the A/F and residual draws already there).
- **Charge per cycle:** σ 1.2 % of the energy a cylinder burns. Soltau (1960, reviewed by Ozdor et
  al.) measured 1.0–1.5 % cycle-to-cycle variation of indicated work at WOT; near MBT the burn's
  timing moves the work little (Heywood §9.4.1, Matekunas 1983; the model: 0.3 %), so the charge
  carries the rest.

## Tests (values from outside the code)

- Burn angles: σ/mean of the development and rapid-burning angles within 0.09–0.15 at WOT and at
  light load (Heywood Fig. 9-35).
- WOT, inline-4 at 2500 rpm: COV of peak pressure 4–10 % (Soltau 5.5 %; Heywood Fig. 9-31's
  factor-of-two burn rates), COV of indicated work 0.8–2 % (Soltau 1.0–1.5 %).
- The single flame-ratio unit test: u′ × 1.175 → angles × 1.175^(−2/3) = 0.89807.

## Result

The model's 1 kHz+ bands now repeat as little as the real engine's (1 kHz −0.01, 2 kHz 0.05). The
125–500 Hz bands still repeat more than the recording (0.98 / 0.77 / 0.67); part of the
recording's loss is the environment (wind, a moving camera), so that gap is not a target yet.

## Out of scope

Partial burns at the stability limit (Heywood §9.4.3; COV of IMEP to 10 % at light load), which
the flame's S_L zero only covers as misfires.
