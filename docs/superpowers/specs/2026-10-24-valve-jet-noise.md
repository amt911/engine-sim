# Exhaust-valve jet noise — spec (2026-10-24)

**Why.** The engines still sound synthetic (user, 2026-10-24): the model carries the exhaust pulses
cleanly and has no broadband sound riding them. At each blowdown the gas crosses the exhaust valve's
curtain near or at sonic speed — a jet whose turbulent mixing noise is generated in the port and
travels down the pipes with the pulse (the "rasp" of a real exhaust), modulated cycle by cycle.

**Physics.**
- Jet speed at the valve: isentropic expansion from the cylinder (p_cyl, T_cyl) to the port
  (p_port = atmosphere + the waves standing at the valve), capped at c*.
  Diameter: the curtain's equivalent, d = √(4·A_curtain/π).
- Level: the same jet-mixing correlation as the tail jets (SAE ARP876, Lighthill u^7.5), with the
  port's gas as the ambient medium. Its acoustic power W = 4πr²·p_rms²/(ρc) (free field) goes into
  the primary as plane waves (below cut-on, 1.84c/πD ≈ 8 kHz for 42 mm in 900 K gas), half
  downstream: p_rms = √(½·W·ρc/A).
- Spectrum: Tam, Golebiowski & Seiner (AIAA 96-1716) fine-scale similarity spectrum G(f/f_F), eq. (6),
  realised by a fitted IIR, its peak following the jet's St = f_F·d/u.

**Where.** A pressure added to the wave each primary receives from its valve, `ExhaustNet` (and the
exhaust thread's job), from `Engine::step` per cylinder. Zero when the valve is shut or the flow
reverses.

**Tests.** Level by hand for a choked case; silent when shut or reversing; spectrum shape vs eq. (6)
at three points; an engine-level test that the tail carries broadband energy synchronous with the
blowdowns (envelope correlation with the port flow).
