# Collector junctions lose pressure as real ones do

Date: 2026-10-26. Status: design. Roadmap: sound realism; lifting off sounds as real ones do.

## Why

The waveguide's collector junction (Kelly–Lochbaum) is lossless. A real junction is not: the gas
that leaves a primary expands into the collector and loses part of its dynamic head, as it does
at any sudden enlargement. The gas that returns contracts into the primary and loses some more.
AngeTheGreat found the same on a real engine ("Improving My Engine Sound Simulator With Real
Data", 15:00–19:06): his pipes rang until he added the boundaries' losses.

Measured on the LFA (`--render loops`), how much quieter lifting off at a held speed is than full
load:

| rpm | 2000 | 3000 | 4000 | 5000 | 6000 | 7000 | 8000 | 9000 |
|---|---|---|---|---|---|---|---|---|
| model v0.23.0, dB | 8.1 | 12.1 | 5.2 | 5.2 | 0.0 | 19.4 | 3.1 | 10.5 |

Real LFA recordings drop 8–19 dB when lifting off, mostly 8–10. That is fairly even across the
range; the model swings with its resonances.

## Model

- **The loss.** The end pressure of each branch of a collector junction is the junction's plus
  s·u|u|, with u the instantaneous velocity of the waves at that end: u = (p_in − p_out)/ρc.
  Mass is conserved, ΣAᵢuᵢ = 0.
  - Each branch end gives P = 2p_in − ρc·u. Each u is therefore a monotone function of the
    junction pressure, and Newton's method finds it in a few iterations.
  - With no losses it is the lossless junction exactly.
- **The coefficients.**
  - A primary's flow into the collector: s = ½ρ(1 − σ)², a sudden expansion (Borda–Carnot).
  - The flow back into the primary: s = ½ρ·0.42(1 − σ), a sudden contraction (White, *Fluid
    Mechanics*, §6.9).
  - σ = A_primary/A_collector, ρ the primary gas's. Each joining jet expands into the collector
    on its own: Benson's constant-pressure junction (vol. 1, ch. 8) with a loss per branch.
  - The collector's own branch has no loss.
- **Quasi-steady.** The loss acts on the waves' instantaneous velocity, not the mean flow, so it
  damps on overrun too: steady loss coefficients applied instantaneously to unsteady flow, as in
  Bassett, Winterbone & Pearson (Proc. IMechE C 215, 2001) and the references they review.

## Tests (values from outside the code)

- **Expansion.** A pipe of area 1 into one of area 4, gas of ρ 0.4 and ρc 240, a 10 kPa wave
  down the small pipe: p_J = 3904.71 Pa (lossless 4000), u = 65.078 m/s, the reflection
  −5618.83 Pa. Hand values, from the quadratic solved.
- **Contraction.** The same wave down the big pipe: p_J = 16054.48 Pa, the small pipe's
  outgoing wave 15782.06 Pa.
- **No loss.** With no losses it equals the lossless junction (1e-9).

## Result

| rpm | 2000 | 3000 | 4000 | 5000 | 6000 | 7000 | 8000 | 9000 |
|---|---|---|---|---|---|---|---|---|
| v0.23.0, dB | 8.1 | 12.1 | 5.2 | 5.2 | 0.0 | 19.4 | 3.1 | 10.5 |
| junction loss, dB | 7.5 | 12.2 | 5.0 | 6.1 | 2.3 | 17.0 | 6.6 | 13.4 |

A step towards the real car's even 8–10 dB, not all of it. The rest is open: the primaries'
gas cools on overrun, while the waveguide's delays stay fixed at 600 m/s. Cost: +7 % (LFA
Medium, 2.82 → 3.03 s for 4 s).
