# Timing-drive rattle — the random mechanical noise of an idling engine

Approved direction: user, 2026-11-12 ("Sí, rama propia"), after the idle diagnosis below.

## Why

Every preset's idle "no se escucha" (user, 2026-11-12). Measured against fixed-gain recordings:
the 1–8 kHz envelope of a real idle does not repeat from one engine cycle to the next (blind2's
`hnr`, best over an rpm search: −1.7 to −10.8 dB on exterior, exhaust and engine-bay microphones —
Golf, Skoda, Cruze, BMW 114i, Alfa 159, Crown Victoria, Cuda), the sim's does (+0.1 to +2.7). The
sim's mechanical noise is all cycle-locked impacts (piston slap, main bearings); its combustion
already varies as measured (idle COV of the work 7–9.5 %), recording backgrounds sit 12–30 dB
under the idles, and the cylinder at exhaust opening is near an atmosphere at a fired idle (no
blowdown jet to add). At idle the valve train is the main mechanical source, "caused mainly by
the gear rattling owing to the variation in the camshaft drive torque" (SAE 2003-01-1720,
four-cylinder diesel at idle).

## Physics

1. **Cam torque.** Each valve's spring pushes its follower with `F = F_seat + k·lift`; through the
   lobe that is a torque on the camshaft `T = F · d(lift)/dθ_cam` — resisting while the valve
   opens, driving the cam forward while it closes (Norton, *Cam Design and Manufacturing Handbook*,
   ch. 9; inertia and friction neglected at idle speeds, where the spring force dominates). The
   lift profile is the sim's own (`valve.rs`). Spring loads: ESTIMATE from production passenger-car
   valve springs, seat and open load per valve scaled with valve area (to be pinned with a source in
   the implementation, e.g. Norton's tables or a manufacturer's spring data).
2. **Drive with backlash.** The cam sprocket and camshaft (inertia `J_cam`) ride the crank's speed
   through the chain or gears with a backlash `b`; when the cam torque reverses faster than the drive
   carries it the sprocket crosses the backlash and hits the other flank — the impact model of the
   gearbox's loose gears (Singh, Xie & Comparin, JSV 131(2) 1989), generalised from a constant oil
   drag to the time-varying cam torque. Shared code: one backlash-impact element used by both the
   gearbox and the timing drive (reuse-first; extracted from `gear_noise::LooseGear`).
3. **Radiation.** The impact force through the timing cover/head into the engine bay, like the
   valve ticking (`engine_bay`), the housing as a panel radiator above its first mode
   (`gear_noise::housing_first_mode_hz`).

Its level comes out of the physics, not from a fitted gain. The measured whole-engine mechanical
curve (Lalor, `mechanical.rs`) stays the total: the cycle-locked impacts keep the remainder (as the
combustion's share is taken today), so adding the rattle does not raise the engine's measured level.

## Tests (each seen failing once; expected values from outside the code)

- Cam torque of one valve on a hand-integrable lift profile against its closed form; the torque over
  a lobe integrates to zero (a spring returns what it took).
- The sprocket rattles only when the cam torque reversal beats the drive (Singh's criterion).
- At an idle speed ripple and cam torque the impact sequence does not repeat cycle to cycle: the
  1–8 kHz envelope's cycle correlation within the recordings' range (hnr ≤ −1.7 dB).
- Its share falls with speed and load (SAE 2003-01-1720: the idle problem).
- The engine's total mechanical A-weighted level still reads the Lalor curve within 1 dB.
- Serialization round trip of any new field (JSON, YAML) with defaults for old files.

## Acceptance

All presets: idle `hnr` into the recordings' range, and the four realism gates no worse than the
base on any preset (`scripts/realism/gates.sh`). Then clips (idle, drive) for the user's ear.

## Non-goals

Injector and high-pressure pump ticking (periodic; SAE 2011-01-0930) and accessory drives — next,
if the idle still lacks them.

## Outcome (2026-11-12): rejected by its own test

Implemented the cam torque (closed-form tests pass) and the sprocket in its backlash. Over a grid
(backlash 0.05–0.3 mm, friction 0.5–8 N·m, crank ripple 0–10 %, firing-to-firing COV 0–10 %) the
rattle locks to the cam torque: cycle correlation of its 1–8 kHz envelope 0.85–1.00 (hnr +7 to
+40 dB), never the recordings' ≤ 0.40. The idle's random 1–8 kHz is not this rattle. Real diesels'
idles read more periodic (Ram +1.7, Volvo −1.8 dB) than petrol ones (−1.7 to −10.8): the next
candidate is the petrol's own source, the choked throttle at idle (Lighthill: ~0.02 W of jet noise
from a ~5 mm sonic jet before the air box and bay). Code kept on this branch for reference only.
