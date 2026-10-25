# Transient fuel: wall film, partial burns, afterburning in the exhaust

Date: 2026-10-25. Status: design. Roadmap: sound realism (the user, 2026-10-25: "when it
reaches the rev limiter you can tell it is artificial, and when it lifts off too").

## Why

Today a fuel cut-off (overrun, rev limiter) stops combustion on the next cycle and a resume
burns perfectly from its first cycle. A real port-injected engine does neither:

- Part of each injection lands on the port walls as a liquid film that evaporates later
  (Heywood §7.6, Fig. 7-28; Aquino, SAE 810494: the x–τ model). After a cut the film keeps
  feeding a lean, decaying mixture for several cycles. At a resume the film must build up
  first, so the first cycles run lean.
- A lean charge burns slowly. The flame speed already follows φ (Metghalchi–Keck, v0.18.0). A
  flame that is still burning when the exhaust valve opens leaves fuel unburnt (a partial burn),
  and below the lean limit it does not take at all (a misfire). Partial burns are also the known
  gap in our light-load COV (2.6 % against Singotia's 13 %).
- The unburnt fuel and air leave through the exhaust. Mixed with hot exhaust gas above ≈ 600 °C
  (HC oxidation, Heywood §11.6.3), it burns there: the pops and crackles of a lift-off and a
  limiter. Chung et al. (2000) trace such misfired gas through a manifold (it arrives at the
  confluence point as twin peaks).

## Model

1. **Fuel per cylinder per cycle.**
   - The ECU meters the injection for a target φ from the air it expects: the trapped fresh
     mass.
   - A fraction x of the injection deposits on the port film. The film gives back film/τ_cycles
     each cycle (Aquino's x–τ, discretized per intake event).
   - The cylinder gets (1 − x)·injection + the film's share. φ is that fuel over the
     stoichiometric fuel of the trapped air.
   - The cut-offs cut the injection only; the film keeps evaporating.
2. **ECU transient compensation** (Aquino; Maloney, SAE 1999-01-0553). The ECU inverts an x–τ
   model of the film to meter for the target. Its model is the engine's own (no invented
   mismatch): the imperfection comes from what no compensation can do.
   - An injection cannot be negative. At a lift-off the film gives back more than the smaller
     charge needs, so the mixture goes rich.
   - A fuel cut is never compensated. Its tail is what we hear.
   - In steady state the compensation is exact and the film is in equilibrium: φ = target.
3. **Energy and burn.** The burn releases the fuel's energy when lean and the air's when rich
   (stoichiometric limit). The flame speed takes this cycle's φ (already). The burn stops at EVO:
   whatever the Wiebe has not released by then leaves as unburnt mixture. Where the
   Metghalchi–Keck laminar speed falls to zero (φ outside ≈ 0.54–1.72, the fit extrapolated past
   its 0.8–1.5 range) the spark does not take: a misfire, with no invented limit. A slow flame
   near it is the partial burn. The combustion efficiency's 4 % stays incomplete products (CO,
   H₂), not tracked fuel.
4. **Unburnt mixture in the exhaust.** Each branch keeps a combustible inventory: fuel mass and its
   air. It enters with the exhaust valve's flow, as the unburnt share of that cylinder's charge.
   Once the branch's gas is hot enough (> 873 K, Heywood §11.6.3), it burns over a short
   deflagration of a few ms in the primary, near the port where the gas is hottest (a time
   constant; estimate, flagged). Both shares also leave the primary with the gas that the next
   valve flow displaces.
   - The heat release becomes a source of volume flow in the primary, Q = (γ−1)/γ · q̇/p: an
     isobaric expansion. In the waveguide's units (reference-gas mass) this is q̇/(c_p·T_ref),
     the same weight the valve's enthalpy flux has. It also heats the primary's gas.
   - Waveguide exhaust only. The FV exhaust takes mass with its energy and has no heat source
     yet.
   - It sounds through the same waveguide.
5. **Parameters (data, with defaults).**
   - `intake.film_fraction` x: an estimate for warm port injection until the ETH/SAE data
     arrive (PAPERS.md).
   - `intake.film_tau_s` τ, in seconds: Heywood gives "of order 2 s" for central injection;
     port injection is faster.
   - Both are serialized, with round-trip tests.

## Tests (values from outside the code)

- **x–τ:**
  - The steady state has φ = target.
  - After a cut, the cylinder's fuel decays as x·(1 − 1/n)^k per cycle: a closed form of the
    discrete model.
  - A lift-off to a quarter of the charge cannot be compensated: the first cycle's fuel is the
    film's give-back, x·m_old, against m_old/4.
- **Partial burn:** a flame whose Wiebe is 90 % done at EVO releases 90 % of the energy and sends
  10 % of the fuel out unburnt.
- **Misfire below the lean limit:** no energy; all the fuel goes out.
- **Afterburn:**
  - Heat in equals the burnt fuel × LHV (energy balance).
  - Nothing burns below 873 K.
  - The source is a volume flow, Q = (γ−1)/γ · q̇/p, checked by hand.
- **Journey:**
  - An LFA lift-off from 8000 rpm shows pops (afterburn events) in its first second.
  - Steady WOT shows none.
  - The limiter shows a few per bounce.

## Out of scope

Catalyst light-off and oxidation, the backfire into the intake, the ignition-cut limiter
strategy (data needed: the LFA uses a fuel cut).
