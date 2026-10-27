# The exhaust valve as Benson's characteristic boundary

Date: 2026-10-27. Status: design. Requested by the user ("todo eso que ha hecho Ange y que está
basado en física hay que implementarlo", item 3 of the Ange roadmap; "continúa con lo de la
frontera de Benson").

## Why

Both exhaust models couple the exhaust valve to its primary as a linear impedance:
p_port = p_atm + 2p⁻ + (c/A)·ṁ. The FV pipe solves the gas nonlinearly, but its valve still
couples through that impedance (`port_wave`). The valve is where the strongest gradients are,
and Benson's boundary is the physical one there:

- a constant-pressure expansion with its entropy rise on outflow;
- choking at the throat;
- the pipe's gas re-entering through a nozzle on reversion.

Measured on the solved boundary, a 3-bar, 1300 K blowdown into a quiet pipe through a valve as
wide as it reaches 2.38 bar against the linear port's 2.48: 4 %. So this is accuracy, not the
cure for the high-speed torque loss. That loss is breathing (FINDINGS, 2026-10-27: valve
time-area and lift).

## Model (Benson, *The Thermodynamics and Gas Dynamics of Internal Combustion Engines*, vol. 1)

Notation: Riemann variables λ = A ± (κ−1)/2·U, with A = a/a_ref, U = u/a_ref; reference state
p_ref, a_ref; ψ = F_valve/F_pipe, the effective valve area over the pipe's (capped at 1:
beyond it the pipe, not the valve, controls the flow).

### Outflow, cylinder → pipe: Jenny's constant-pressure model

- Isentropic expansion from the cylinder's stagnation state (p_c, a_c) to the valve throat.
- Then a constant-pressure, irreversible expansion to fill the pipe if the throat is subsonic, or
  a pressure drop if it is choked (Benson §4.1; §6.6.1, Figs. 6.32–6.34).
- With π = (p/p_c)^((κ−1)/2κ), π_α = λ_in·(p_ref/p_c)^((κ−1)/2κ) and the entropy change
  C = a_A2/a_A1:
  - −U_B = 2C/(κ−1)·(π − π_α) (6.45);
  - subsonic: f₁(π) = 0 (6.47); sonic: f₂(π) = 0 (6.51);
  - the boundary between them is π_cr (6.55);
  - C from (6.56), by successive substitution starting at 1/π_α (6.61);
  - then λ_out = 2π·(p_c/p_ref)^((κ−1)/2κ) − λ_in (6.62).
- Mass flow: ṁ = κ·(p_ref/a_ref)·F·U/A²·P (6.63), from the pipe state.

### Inflow, pipe → cylinder: the partially open end, scaled to the cylinder

- Equations (6.33)–(6.42) with λ_in multiplied by (p_ref/p_c)^((κ−1)/2κ) and λ_out divided by
  it (§6.6.1, p. 298).
- Subsonic: f(A) = 0 (6.35); sonic: (6.38)–(6.41).

### Entropy

- The outflow carries its entropy level into the pipe (A_A = a_A/a_ref of the gas leaving): hot
  blowdown gas is a different entropy level from the pipe's.
- The FV pipe takes it as the ghost cell's temperature.
- The waveguide has one reference gas, so it takes the boundary's pressure and flow only (see
  *Coupling*).

## Coupling

- **FV exhaust.** The first cell's state gives λ_in (its incoming characteristic). The boundary
  returns λ_out, A_A and ṁ. The ghost cell is set from them: p from (λ_in + λ_out)/2, u from
  their difference, T from A_A. The cylinder takes ṁ with the enthalpy of whichever side is
  upstream.
- **Waveguide.** The arriving wave p⁻ (linear) is turned into λ_in, as the simple wave of that
  pressure: λ_in = 2·(1 + p⁻/p_ref)^((κ−1)/2κ) − 1. The boundary gives the port pressure
  p_port and ṁ. The wave sent down is p⁺ = p_port − p_ref − p⁻ (superposition, as the waveguide
  is linear), and the source flow follows ṁ as now.
- **κ** is the pipes' (1.4: they are cold-air pipes); the cylinder's real gas enters through its
  stagnation p_c and a_c = √(γ_c·R_c·T_c). A real-gas pipe is a later phase.

## Numerics and cost

- Benson solves f(π) by interval halving (Fig. 6.34). Here the same brackets
  ([S, 1] subsonic, [π_α, S] sonic) with a safeguarded Newton step, warm-started from the last
  step's π: one or two iterations a step.
- C converges in two or three substitutions.
- The valve is solved once per step per open valve. Measure the cost with `scripts/cpu-lock.sh`
  renders before and after (budget: a few % of a step).

## Tests (values by hand or from Benson)

- **Steady limit.** A quiet pipe (λ_in = 1 at p_ref): outflow at small ψ gives the isentropic
  nozzle's mass flow through F_valve (Heywood App. C), subsonic and choked.
- **Defining laws.** Outflow keeps the stagnation enthalpy (4.1); the throat's isentropic flow
  fills the pipe (4.2). Inflow is the pipe gas through a nozzle into the cylinder. The incoming
  characteristic is kept. (Fig. 6.25's worked case is drawn schematically: its f(A) scale does
  not match eq. 6.35's, so it is not used as a value.)
- **Acoustic limit.** For a weak outflow, δp = a·ṁ/F, the characteristic impedance.
- **Continuity across the sonic boundary.** λ_out and ṁ are continuous as π_α crosses π_cr.
- **Direction.** π_α > 1 inflow, < 1 outflow, = 1 no flow (Benson p. 299).
- **Engine.** The FV exhaust with the boundary keeps every engine test green, and its dyno moves
  by a few percent at most from the impedance port's (the 4 % above). Every change is measured
  and reported.
- **Stability.** The FV chatter test keeps passing (overrun reversals ≤ 25 in 7 cycles), and so
  do the overrun and idle tests.

## Out of scope

- The intake valve (backflow into the runner is small at WOT; the same routine applies later).
- A real-gas (variable κ) pipe.
- Valve-jet noise changes (the jet model keeps reading the port state).
