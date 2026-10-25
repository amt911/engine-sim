# Finite-amplitude real-time exhaust: Riemann-variable delay lines

Date: 2026-10-25. Status: design (roadmap phase "nonlinear exhaust", see
`2026-10-21-sound-realism.md`). Replaces the linear pressure waveguide at the real-time
levels; the FV model stays as the reference solver.

## Why

Measured against a recording taken at an LFA's tail pipe (YouTube pm7E5F7K_ms, 6.6–7.4 s,
~7400 rpm WOT), relative to the first firing harmonic:

| | firing 2nd/3rd | firing 4th–6th | bank orders | other orders |
|---|---|---|---|---|
| real | −11 / −6 dB | −10 / −22 / −24 | −16 | −21 |
| main (linear waveguide) | 0 / −12 | −19 / −38 / −36 | −1 | −37 |
| FV (Euler, WENO5) | −1 / −9 | −26 / −36 / −36 | −16 | −41 |

The linear waveguide cannot steepen a pulse. Hirschberg et al. (JASA 99, 1996) give the shock
distance x_s = 2γp₀c / ((γ+1)·max ∂p/∂t). For the LFA at 4100 rpm WOT that is ≈ 1.5 m, while the
exhaust path is ≈ 2.6 m long, so real pulses do steepen. Pressure-wave warping (`WarpLine`,
already merged into the branch, unused) is exact for weak simple waves: it passes the Fubini
and Blackstock checks. The LFA's waves reach ~1 bar, though (ε·p ≈ 0.9), and the valve
boundary p = c·ṁ/A is linear. Wired in, it put 75 % of the WOT energy above 2 kHz.

## Model (Benson, *Thermodynamics and Gas Dynamics of IC Engines* Vol. 1, 1982)

- Homentropic flow, reference state (p_ref, a_ref), ratio of heats κ:
  - A = a/a_ref = (p/p_ref)^((κ−1)/2κ) (eq. 2.34).
  - λ = A + (κ−1)/2·U and β = A − (κ−1)/2·U (eqs. 6.1–6.2).
  - A = (λ+β)/2, U = (λ−β)/(κ−1).
- λ travels at dX/dZ = bλ − aβ and β at aλ − bβ, with a = (3−κ)/(2(κ−1)) and
  b = (κ+1)/(2(κ−1)) (eqs. 6.5–6.8; for κ = 1.4: 3λ − 2β).
- Each pipe holds two lines, λ forward and β backward. They are WarpLines whose sample speed is
  the characteristic slope, with the opposite variable taken as its mean over the line at
  emission. This is the one approximation: it is exact for a simple wave, and its error is in
  the cross term of wave interaction. It is checked against FV in test 3 below.
- Shocks use WarpLine's equal-area placement (Salomons 2025, §2). Benson §5.4 is the
  graphical counterpart.
- Wall friction and heat transfer: the existing per-pipe loss stays, applied to the
  deviation of λ and β from the rest value. A rest value of 1 means no flow: losses act on
  (λ−1).

## Boundaries (Benson)

- **Open end** (§2.8.2, §3.4 for a partially open end): A = 1 at the end for outflow. Inflow
  from the atmosphere comes from a reservoir at constant stagnation conditions (§3.3).
- **Exhaust valve, outflow** (§4.1, constant-pressure model, eqs. 4.1–4.11):
  - Inputs: the cylinder stagnation state (p₀, a₀), ψ = valve effective area / pipe area,
    and the incoming β.
  - Unknowns: the pipe-end state (A₂, U₂). Solved with Newton on the subsonic-throat equation
    (4.9), with the choked branch §4.1.3.
  - Outputs: the outgoing λ and the valve mass flow. The cylinder model already computes the
    mass flow; here it comes from the boundary instead, so the cylinder and the pipe agree.
- **Exhaust valve, reverse flow** (§4.4, inflow from a pipe through a valve).
- **Junctions** (§5.5.1, constant-pressure branch model): all pipe ends share one A, and the
  mass flows sum to zero. Solved for the common A from each pipe's incoming variable.
- **Area changes, silencers, catalysts**: keep the existing linear scattering on the
  *perturbation* about the local mean for now. Benson §5.5.2 gives an approximate nonlinear
  area change, which comes second.

## Entropy

Hot exhaust gas has a different entropy from the reference. Benson ch. 7 treats non-homentropic
flow with path lines. First phase: each pipe gets its own a_ref from the mean gas temperature
it carries, the same thing the waveguide does today with C_PRIMARY, C_COLLECTOR and C_TAIL.
Phase two would follow ch. 7.

## Cost

Per pipe and step: two WarpLine pushes and reads (amortized O(1)), plus the opposite-line mean
kept as a running sum. Boundaries: a few Newton iterations per valve and per junction. This is
expected to stay well under FV-Medium, and must fit the target laptop (Ryzen 7 5850U, 15 W) at
Medium. Measure in cycles per audio second for every preset, as in `2026-10-24-realtime-ultra.md`.

## Tests (values from outside the code)

1. The Riemann-variable round trip and the characteristic slopes at rest and in a simple wave:
   hand values for κ = 1.4.
2. Open end: a compression returns as a rarefaction. Closed end: the same sign. Values from
   Benson §2.8 (eqs. on p. 93).
3. A pulse in a long pipe compared with the FV solver (independent Euler, WENO5): arrival time,
   peak and the steepest slope within tolerances.
4. Valve boundary: a choked outflow from 3 bar into a pipe at rest, giving the pipe state of
   Benson §4.1.3 worked by hand.
5. Junction: a pulse into a 3-way junction of equal areas gives the hand value from the
   constant-pressure model.
6. The LFA fingerprint (the table above), with a script in the repo: firing harmonics 4–6 must
   rise toward the recording. Pass: the 4th within 6 dB of the recording, and nothing above 2 kHz
   beyond the recording's share plus 10 points.
7. The overrun "fly" check: the harmonic share on overrun must fall from 87 %.

## Out of scope here

Cycle-to-cycle irregularity ("other orders" −37 dB against −21 real), throttle-gap jet noise
(Tam 1990) and the turbocharger (Benson ch. 9) come after this.
