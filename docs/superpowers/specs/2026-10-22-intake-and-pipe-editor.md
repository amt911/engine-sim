# 1D intake and visual pipe editor — spec (2026-10-22)

Status: approved by delegation (user, 2026-10-21: "que se pudiera modelar un tubo de escape o el
intake incluso"). Builds on the FV exhaust (`fv.rs`, `exhaust_fv.rs`).

## 1D intake

Replace the single 0D plenum + throttle by a network of FV pipes, mirroring the exhaust:

```text
atmosphere → [snorkel → airbox (wide FV can)] → plenum (0D, optional)
           → per-cylinder runner (FV, trumpet = bellmouth entry loss model) → intake valve
           with throttle either common (before the plenum) or individual (one per runner: ITBs)
```

- **Throttle as an orifice between two pipe cells** (compressible orifice law, existing
  `orifice_mass_flow`) placed at a runner position (ITB) or before the plenum (single throttle).
- **Trumpet / bellmouth:** open-end boundary with an entry loss coefficient (sharp edge ≈ 0.5,
  bellmouth ≈ 0.05) and an end correction; the radiated induction sound is the trumpet's outflow,
  like the tail pipe.
- **Valve end:** the intake valve couples the runner's last cell to the cylinder (same planned
  exchange as the exhaust valve).
- **Data:** `EngineDef.intake: IntakeDef { model: zero_d | fv, runner_length_m, runner_diameter_m,
  trumpet: { length_m, flare }, throttle: common | individual { diameter_m }, plenum_volume_m3,
  airbox_volume_m3, snorkel_length_m, snorkel_diameter_m }`, migration from v2 files.
- **LFA preset:** 10 individual throttle bodies, short runners, airbox (estimates marked).

Cases: intake tuning moves the torque curve with runner length (ram effect, peak shifts to lower
rpm with longer runners); induction sound present at WOT and audible as a separate source; mass
conservation through the network; idle and overrun MAP still realistic (dashpot).

## Visual pipe editor

- The engine view shows the pipe network: primaries, collectors, muffler can, tails; runners,
  trumpets, plenum, airbox — drawn to scale from the definition.
- Click a pipe to select it; drag its end to change length; scroll to change diameter; context
  buttons to add/remove a collector group, a resonator (Helmholtz side branch), an X/H crossover
  between banks.
- Everything edits `EngineDef` (validated), so it saves to YAML and the sound updates live.

Cases: dragging a primary end changes `primary_length_m` and the timbre; the network drawn matches
the definition (kittest render); invalid edits are refused with the validation message.
