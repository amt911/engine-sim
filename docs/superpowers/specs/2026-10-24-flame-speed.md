# Burn timing from the flame — spec (2026-10-24)

## Why

Measured against three real Lexus LFA recordings (YouTube; steady idle seconds), the sim's idle is too
steady. The 2 ms envelope contrast in 250 Hz–1 kHz (p98 / median, per second) is 7.6–8.9 dB in the
recordings and 4.6–5.2 dB in the sim. The user hears this as "synthetic". The model's combustion
timing is fixed:
- a 5° ignition delay;
- the preset's `burn_duration_deg`;
- a uniform ±1.5° / ±5 % jitter at every speed, load and dilution.

Real engines burn slower and less repeatably at idle. The large residual-gas fraction there slows the
laminar flame, and the early flame's development time sets both the delay and its cycle-to-cycle
dispersion (Heywood §9.4).

## Physics

- **Laminar flame speed** — Metghalchi & Keck, Combust. Flame 48 (1982), eq. 24, RMFD-303 (indolene,
  a gasoline reference):
  - S_L = S_L0(φ) · (T_u/298)^α · (p/1 atm)^β · (1 − 2.1 f)
  - S_L0 = B_m + B₂(φ − φ_m)², with B_m = 27.58 cm/s, B₂ = −78.34 cm/s, φ_m = 1.13 (Table 3A)
  - α = 2.18 − 0.8(φ − 1) and β = −0.16 + 0.22(φ − 1) (eqs. 16–17)
  - f is the diluent (residual) mass fraction, fitted for 0–0.2. Beyond 0.2 the fit is extrapolated
    and floored at the value for f = 0.4.
- **Burn angles** — Hires, Tabaczynski & Novak, SAE 780232 (1978):
  - eq. 13: Δθ_ig ∝ (S_p ν)^⅓ (h/S_L)^⅔
  - eq. 32: Δθ_c ∝ (b/h*)(S_p ν*)^⅓ (h_i/S_L*)^⅔
  - S_p is the mean piston speed, ν the kinematic viscosity of the unburned gas, and h the chamber
    height (clearance height at the spark).
- **Ratio form.** Hires' constants (C_ig = 2.35, C_c = 10.4) belong to his 400 CID engine, in mixed
  units. Instead, a preset's `burn_duration_deg` and the 5° delay are its values at a reference state,
  and the correlations scale them from there:
  - S_p = 10 m/s;
  - unburned gas at 700 K and 20 atm (ν from Sutherland and ideal-gas density);
  - φ = 1, f = 0.08 (a typical full-load residual, Heywood Fig. 6-21);
  - h = the engine's own clearance height at TDC.
  The burn uses φ = 1 (the model runs stoichiometric) and the cylinder's own state at the spark (p,
  T, f = residual / trapped mass).
- **Cyclic dispersion.** The jitter amplitudes (±1.5° on the start, ±5 % on the duration) also hold at
  the reference. The start jitter scales with Δθ_ig, because the dispersion comes from the
  early-flame time scale (Hires' eddy burn-up time). The energy variation stays as is.

## Acceptance

- Unit tests with hand values:
  - MK at the reference conditions and at a diluted one;
  - Hires scaling: ×8 S_L gives Δθ × 1/4; ×8 S_p gives × 2.
- Engine: the burn at idle is longer and more dispersed than at WOT (sim measurement).
- Dyno tests still pass (LFA within the published ranges; K20A; inline-4).
- Idle and start journeys still pass.
- Idle envelope contrast (250 Hz–1 kHz) moves toward the recordings' 7.6–8.9 dB (reported, not
  gated: the recordings are phone/YouTube audio).
