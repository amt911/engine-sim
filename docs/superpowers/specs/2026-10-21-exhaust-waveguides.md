# Exhaust as waveguides — spec (2026-10-21)

Status: approved by delegation. Roadmap phase 3a of the research doc. Goal: the LFA must
sound like an LFA; the 0D runner low-passes the pulses (~100 Hz corner) and ten smooth pulses
cancel at WOT.

## Model (J. O. Smith, PASP; Baldan et al. 2015; research doc §"De la presión simulada")

Linear acoustic waveguides downstream of the non-linear valve flow:

- **Pipe** = two delay lines (p⁺ forward, p⁻ backward), length L/c samples at the physics rate,
  linear-interpolated fractional delay, per-trip loss g and a one-pole low-pass (wall losses).
- **Valve end** (plane-wave impedance boundary): with mass flow ṁ into the pipe,
  u = (p⁺ − p⁻)/(ρc) and ṁ = ρAu ⇒ **p⁺ = p⁻ + c·ṁ/A**. Valve shut (ṁ = 0) ⇒ p⁺ = p⁻, a
  closed end. ṁ comes from the compressible orifice law between the cylinder and the pipe
  pressure p_atm + p⁺ + p⁻ (explicit: uses last step's p⁺).
- **Junction** (collector, N-port, Kelly–Lochbaum): admittance Yᵢ ∝ Aᵢ;
  p_J = 2·ΣYᵢ·pᵢⁱⁿ / ΣYᵢ; pᵢᵒᵘᵗ = p_J − pᵢⁱⁿ.
- **Open end**: reflection −|R| with a one-pole low-pass (|R| → 1 at low ka, falls at high ka;
  Levine & Schwinger 1948, Silva et al. arXiv:0811.3625). The transmitted wave (1+R)·p⁺ is the
  radiated source, then the voice's radiation term (d/dt) applies.
- Speed of sound from a fixed hot-exhaust temperature for now (c ≈ 550 m/s); c(T) later.

## Data (`EngineDef.exhaust`, serde, defaults keep old files loading)

```yaml
exhaust:
  primary_length_m: 0.6
  primary_diameter_m: 0.042
  collectors: [[1,3,5,7,9],[2,4,6,8,10]]   # groups of cylinders (1-based); [] = all in one
  collector_length_m: 0.3
  collector_diameter_m: 0.065
  tail_length_m: 1.2
  tail_diameter_m: 0.06
```
Each collector feeds its own tail (dual exhaust when there are two groups).

## Cases

- Closed pipe impulse: a pulse injected with the valve shut returns after 2L/c with the same
  sign; with an open end it returns inverted (sign of R).
- Junction conserves "power": equal-area 2-port junction is transparent (no reflection).
- Changing primary length audibly/measurably changes the spectrum (peak shift).
- LFA WOT high-rpm level clearly above idle; firing order audible: 5th order dominant.
- Torque curves move with primary length (tuning), within physical sanity.
