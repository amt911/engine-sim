# The LFA's intake as built: surge tank, ten throttles at the ports, sound exciter — design

Status: **approved** by the user (2026-10-28), revised the same day after reading Okamoto's Figs. 18–19 at full resolution (below).

## Why

The user hears the K20A and the V8s as convincing and the LFA as synthetic. The LFA preset models
its intake as ten trumpets open to the atmosphere, one throttle at each. The engine as Toyota
built it is different, and its intake is a designed sound source:

- Toyota, *Lexus Debuts LFA* (news release, 21 Oct 2009): "An acoustically tuned surge tank
  linked to 10 individual throttle bodies creates a powerful induction sound."
- Okamoto et al., *New Toyota 4.8L V10 … for Lexus LFA*, SAE 2011-01-2024:
  - §2 / Fig. 3: "Surge tank resonance frequency tuned for powerful engine sound. Adoption of
    so-called 'sound exciter'."
  - §3.3: the ten throttles sit "inside the intake manifold, close to the combustion chamber of
    each cylinder".
  - Fig. 18 *Sound design and intake sound tuning*: air cleaner case → multi-use surge tank
    ("400 Hz, tuned by surge tank") → equal-tone tuned intake; the sound exciter carries the
    surge tank's sound into the cabin through a passage "tuned by passage" to 800 Hz.

Most real LFA recordings the user knows are in-car, where the exciter is heard. With no surge
tank, no 400 Hz resonance and no exciter, the preset misses a part of the LFA's sound that its
makers designed in.

## What (physics, nothing by ear)

1. **Intake topology "throttles at the ports inside a plenum".** It draws air from the atmosphere
   through an inlet duct (1D, finite volume: its inertance is what makes the Helmholtz resonance
   with the tank) into a surge tank (0D volume). Ten runners (1D FV, as now) lead from the tank to
   the ports. Each runner's throttle sits at its port end and is the throttled boundary that today
   faces the atmosphere. The induction sound radiates from the inlet duct's mouth (the air
   cleaner), not from ten trumpets. Plenum engines with one throttle and ITB engines open to the
   air keep their present models.
2. **The LFA preset uses it**:
   - the tank's volume and the inlet duct's length and bore are chosen so the Helmholtz
     frequency f = (c/2π)·√(A/(V·L_eff)) is Okamoto's 400 Hz (end corrections per Munjal);
   - the split between V and the neck is not published: V is set to the engine's displacement
     (a common sizing; an estimate, stated), and the neck follows from 400 Hz.
3. **Sound exciter (second step, its own PR)**: a listener inside the cabin hears the tank
   through a passage tuned to 800 Hz (a quarter-wave or Helmholtz passage per Fig. 18). Only for
   an in-car listener position, which the app does not have yet; it would need its own small
   design.

## Feasibility first

Before the full implementation: the inlet duct + tank + runners in the model must show a
resonance at 400 Hz ± 10 % in the induction sound at the mouth (a sweep render), and its level
must matter beside the exhaust at the default listener (a third-octave comparison with and
without). If the intake stays more than 15 dB under the exhaust in every band, building it is
not worth it for the exterior sound, and only the exciter (in-car) would bring it.

## Acceptance

- The LFA at WOT: the 400 Hz band of the induction sound and the order fingerprint move toward
  the real LFA (`papers/recordings`), measured, not tuned.
- Torque within 2 % of today's: the tank changes the waves, not the breathing at WOT, where the
  throttles are wide open.
- Other presets unchanged (bit-identical renders).
- Cost ≤ 3 % (one inlet duct and one 0D volume per engine).

## Open decisions (defaults taken unless told otherwise)

- Tank volume = displacement (4.8 L), the neck from 400 Hz.
- The air cleaner as a plain inlet (its element's resistance is not published).
- The exciter is not part of this step.

## Revision after Okamoto's Figs. 18 and 19 (2026-10-28)

**The 400 Hz is not a Helmholtz resonance.** With a tank of at least the displacement (4.8 L) and an
inlet that can carry the WOT airflow (~0.4 kg/s, ≥ 70 mm), the end corrections alone put the
Helmholtz frequency under ~200 Hz. A tank tuned to 400 Hz must be using a standing wave along
its length: c/(2L) = 400 Hz gives L ≈ 0.43 m, a manifold spanning a V10's banks. So the tank is
modelled as a 1D duct (finite volume, closed ends), not as a 0D volume, and its modes come out of
the physics.

**Layout from Fig. 18:** two air cleaner cases at the front, each with a long curved duct to the
central tank above the engine; ten equal runners leave the tank, five per bank (the throttles at
their port ends, §3.3); the sound exciter's passage (tuned to 800 Hz) starts at the tank and
opens into the cabin. Lengths not given in figures are scaled from the drawings against the
car's published dimensions (wheelbase 2,605 mm) and stated as estimates.

**The exhaust as built (Fig. 19), part of this work:** per bank an equal-length 5-into-1 manifold,
the front and the rear three-way catalysts close together, then two long parallel pipes to one
large transverse rear silencer (the "multistage titanium main muffler" of Toyota's release),
three tail pipes. The preset now ends in a 1.2 m pipe after a single catalyst. The exhaust is
corrected first (preset data with the existing segments; each bank's share of the shared
silencer as its own volume, an approximation stated), measured against the real LFA, then the
intake.

### Order of work

1. Exhaust as in Fig. 19 (preset data): order fingerprint and the shift's low-frequency burst
   against the real LFA, before and after.
2. Intake: the tank as a closed 1D duct with the runners branching off it (side-branch coupling
   in the finite volumes), two inlet ducts from the atmosphere; the induction sound radiated from
   the two inlet mouths.
3. Sound exciter and an in-cabin listener: a separate small design.
