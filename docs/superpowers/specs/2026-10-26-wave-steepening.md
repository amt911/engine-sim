# Exhaust waves steepen, and the walls round them off

Date: 2026-10-26. Status: on hold — overrun too loud (see FINDINGS, "Wave steepening"). Roadmap: sound realism ("too synthetic"), the real-time
non-linear exhaust (task #20) without the finite-volume cost.

## Why

At 7500 rpm and full load, the LFA's third-octave bands (relative to each recording's peak), the
real car (YouTube recording) against the model (`--render loops`):

| band | 2.5 kHz | 3.2 kHz | 4 kHz | 5 kHz | 6.4 kHz | 10 kHz |
|---|---|---|---|---|---|---|
| real LFA | −7 | −13 | −15 | −15 | −19 | −28 |
| model (v0.22.1) | −18 | −32 | −25 | −33 | −36 | −37 |

The real car has 10–19 dB more between 2.5 and 6 kHz: the harmonics of a pulse that steepens.
A blowdown pulse of 0.3 bar in a 900 K primary moves its crest about a third faster than its foot,
so within a primary's length it turns into a shock. The linear waveguide keeps every wave's shape.
The finite-volume exhaust has the steepening but costs 2–3× the waveguide (offline only).

AngeTheGreat measured the other half on a real engine ("Improving My Engine Sound Simulator With
Real Data", 15:00 and 18:49). His simulated pipes rang at every harmonic; the real one did not, because
the walls and boundaries take the highs. Adding pipe friction and heat transfer was what fixed his
sound.

## Model

- **Steepening.** Each sample of a pipe's wave travelling away from its near end arrives after
  L/(c·(1 + k·p)), with k = (γ+1)/(2ρc·c). That is the simple wave's speed c + (γ+1)/2·u, with
  u = p/ρc (Msallam, Dequidt, Caussé & Tassart, *Acta Acustica* 86, 2000, eqs. 1–6).
  - The waves coming back are reflections from an open end or a widening junction. Those are
    rarefactions, which spread instead of forming shocks, so that direction stays linear.
- **Shocks.** A sample that would overtake the one ahead merges with it, as sticky particles do:
  the sum and the speed-weighted send time are kept. The cluster moves as the centre of mass, at
  the Rankine–Hugoniot speed of the simple wave's shock. That is the entropy solution of Burgers'
  equation (E, Rykov & Sinai 1996; Brenier & Grenier 1998), and it dissipates the crest a shock
  overruns.
  - Msallam's own rule only stopped the overtaking. That kept the crests, so energy piled up
    between valve and junction and the exhaust never repeated (15 % cycle to cycle).
- **Reading a line.** A line's output is the mass-weighted average of the values arriving near
  each instant, with tents at least two steps wide (Nadaraya–Watson).
  - With evenly spaced values that is a mild filter ¼, ½, ¼: −0.23 dB at 10 kHz.
  - Two samples about to merge weigh exactly as their cluster will. So the sound does not jump
    when a shock gathers one sample more or less, and the echoes settle.
  - A real exhaust shock is microseconds thick, under one 5.2 µs step. The two-step width is
    the grid's, as finite volumes capture a shock over about two cells.
- **Walls.** The loss above 200 Hz follows Kirchhoff's √f law for each pipe's diameter and gas:
  α = √(πfν)·(1 + (γ−1)/√Pr)/(a·c) (Kinsler et al., §9.9).
  - The flat `LOSS_PER_M` stands for α at 200 Hz. The excess is four first-order shelves,
    fitted once, when the pipe is built.
- **Segments.** Primaries and collectors are cut into segments of at most 0.4 m, each with its
  share of the walls' loss and then a non-linear line: Lee & Hamilton's operator splitting.
  - A full-load pulse forms a shock in about 0.19 m.
  - Measured on the LFA's exhaust driven periodically, cycle to cycle: unsegmented 3.9 %,
    0.4 m under 1 %, 0.2 m 0.34 %. The finite-volume exhaust: 0.15 %.
  - The designed path's smooth pipes take one segment each. The packed silencers' perforated
    tubes keep their packing's loss.

## Tests (values from outside the code)

- **Linear limit.** A faint wave crosses a non-linear line as the linear one does, through the
  ¼, ½, ¼ filter: 0.99973 at 1 kHz and 0.97347 at 10 kHz (by hand).
- **Speed of a loud sample.** In 1 m of gas at c = 500, a 10 kPa sample (slow ramp, no shock)
  arrives after 184.401 steps instead of 200 (by hand).
- **Shock speed.** A step from 0 to 20 kPa arrives at the mean of its two sides' speeds:
  184.401 steps (Rankine–Hugoniot).
- **Content.** ∫p dt is kept within 0.1 % up to 20 kPa. At 60 kPa it is within 5 % (measured
  3.5 %: the grid-scale capture of the strongest shocks).
- **Fubini.** A 2 kPa, 200 Hz tone over 2 m grows B₂/B₁ = 0.0618 (Hamilton & Blackstock ch. 4).
- **Walls.** One metre of a 42 mm, 896 K pipe takes a further 0.313, 0.879 and 1.536 dB at 1, 4
  and 10 kHz (by hand), and nothing at 100 Hz.
- **Periodic.** The LFA's exhaust driven by the same blowdown every cycle repeats within 1 %.
- **Travel-time tests.** These now send faint pulses: a loud one arrives sooner, as it should.

## Result

LFA, 7500 rpm, full load (`--render loops`), third-octave bands relative to the peak:

| band | 3.2 kHz | 4 kHz | 5 kHz | 6.4 kHz | 8 kHz | 10 kHz |
|---|---|---|---|---|---|---|
| real LFA | −13 | −15 | −15 | −19 | −22 | −28 |
| v0.22 | −32 | −25 | −33 | −36 | −36 | −37 |
| steepening (0.2 m) | −18 | −16 | −17 | −18 | −18 | −19 |

## Cost

LFA at Medium, 4 s of sound on one core: 2.78 s before, 3.56 s after (+28 %).

## Out of scope

- Frequency-dependent loss at the junctions and the valve (Ange's other boundary finding).
- Mean-flow convection (c ± U) in the delay.
