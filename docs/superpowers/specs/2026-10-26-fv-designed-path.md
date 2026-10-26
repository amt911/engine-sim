# The FV exhaust follows the designed path

Date: 2026-10-26. Status: design. Roadmap: sound realism, the FV (non-linear) exhaust used by the
offline Reference renders (v0.22.0).

## Why

The FV exhaust builds its own branch after each collector: the collector, a packed can sized by
`muffler_volume_m3`, and the tail. It ignores the designed path (`ExhaustDef::path`):
catalysts, silencers, resonators. The waveguide follows that path.

The FV model is meant to add non-linearity, but here it changes the geometry too. Measured:
- **Idle.** FV is 10–12 dB louder than the waveguide at idle on the inline-4 and the LFA. The
  catalyst every road preset now carries explains half of the inline-4's gap; FV never gets it.
- **Resonances.** Driven by the same small valve flow (the linear regime, where the two must
  agree), FV and the waveguide put the LFA's resonances in different places: +6 to +25 dB at
  100–300 Hz in FV.
- **NaN.** FV with `muffler_volume_m3 = 0` returns NaN.

## Model

- **FV where the waves are strong.** Primaries, each collector's junction and the collector pipe
  stay as now.
- **The designed path, linear, after the collector.** `exhaust::after_collector(def)` builds a
  duct, the same one the waveguide uses, with its open end and its outflow Mach. Past the
  collector the waves are an order weaker, so they stay linear. The same split appears in
  `exhaust_riemann` and in engine-wave practice.
- **The node at the collector's far end.** Pressure is common to both sides and the volume
  flow passes.
  - FV side: along the characteristic reaching the end from inside, p + Z·v = P = p_i + Z·v_i
    (Z = ρc of the last cell; the same rule as `fv::open_end`).
  - Duct side: p = p_atm + p⁺ + p⁻ and A_fv·v = A_d·(p⁺ − p⁻)/ρc_d, so
    p = p_atm + 2p⁻ + ρc_d·(A_fv/A_d)·v.
  - Together: v = (P − p_atm − 2p⁻)/(Z + ρc_d·A_fv/A_d), and p⁺ = p − p_atm − p⁻.
  - The ghost carries the interior gas expanded to p on outflow, and the duct's gas on inflow.
- **Rates.** FV steps every `substeps` physics steps and the duct every physics step. The p⁺ the
  FV sends is interpolated between its last two pipe steps, as the radiated value already is
  (no staircase into the duct). The p⁻ arriving is read at each FV step.
- **Removed.** The FV can and its lining go: the path's silencers carry them, validated against
  Munjal. That also removes the NaN.

## Tests (values from outside the code)

- **The node, by hand.** A step p⁻ arriving at a collector of gas at rest gives v and p⁺ from
  the two-impedance formula (a closed form in the test).
- **The linear regime.** The LFA's exhaust (with its catalyst), driven by a small valve flow at
  the first primary, carried by FV and by the waveguide. Band levels from 50 Hz to 1.5 kHz must
  agree within 3 dB. The two now differ only in the primaries and collector, where the waveguide
  is the linear limit of FV.
- **The catalyst.** In FV, the path's catalyst takes the 1–3 kHz band down as it does in the
  waveguide (within 3 dB of the waveguide's insertion loss).
- **No NaN.** An exhaust with no can stays finite.
- **Journeys.** The idle-to-WOT range of FV renders is within 6 dB of the waveguide's, on the
  inline-4 and the LFA.

## Out of scope

Non-linear silencers (the waves there are weak), and the real-time Riemann exhaust (parked).
