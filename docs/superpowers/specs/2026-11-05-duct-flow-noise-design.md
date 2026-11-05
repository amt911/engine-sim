# Duct flow noise, U⁴, one universal constant — design

Status: approach approved by the user 2026-11-05 ("me parece bien la primera; si no suena bien
tiramos con la segunda"). This document needs the user's approval before the plan.

## Why

Every preset sounds "synthetic": a burst of treble at each firing ("una ametralladora") over a
firing hum. Measured (scratchpad tools, see `docs/agents/debugging.md` → *Sound realism*):

| | 0.8–5 kHz envelope modulation at the firing rhythm |
|---|---|
| 209 steady real segments | 12 % median (p25–p75 ≈ 6–24 %) |
| Ange's new simulator, multi-cylinder demos | 10–20 % |
| our 8 presets, 3000 rpm | 30–150 % |
| our inline-4 open, 6000 WOT (Ange's demo condition) | 30–54 % |

The metric separates every ear verdict so far (clips the user judged real: 1–7 %; judged
"machine gun": 8–58 %). Our render plus steady broadband noise filling the gap to a real recording
was judged "mucho mejor".

Davies & Holland 1999 (JSV 223(3), `papers/acoustics/flow-noise/davies1999.pdf`, §1 and §4):
flow noise, "varying roughly with the fourth power of the mass flow, normally provides a major
contribution to tailpipe sound power emissions above say 300 Hz"; in ducts at M < 0.3 the
measured power scales as **W ∝ U⁴/c** while only plane waves propagate and **W ∝ U⁶/c³** where
higher modes propagate; the sources are vortex shedding at abrupt area expansions and shear-layer
mixing, amplified by acoustic feedback (Davies & Holland 2001, JSV 239(4): 10–20 dB at the
chamber/tailpipe resonances). Our sources follow free-jet laws (U⁸, `jet_noise.rs`) or a steady
spoiler correlation at collector junctions only (Nelson–Morfey; built and reverted 2026-11-01,
45–49 dB under the render), which is why they are inaudible.

## Goal

Every engine, with no per-engine setting, gets broadband flow noise from the sites where a real
exhaust makes it, at a level set by one physical law and one universal constant, so its treble is
a near-steady hiss that follows the flow — not a burst per firing.

## Non-goals and anti-goals

- No per-preset or per-engine knob, no level set by ear for one engine. (The user rejected a
  fitted noise floor for petrol idle on 2026-11-03; this is not one: the level follows each
  site's own instantaneous flow by Davies' law.)
- Not the intake side (later, same model).
- Not the flow–resonance feedback of Davies 2001 as a mechanism (its 10–20 dB is inside the one
  constant, see Risks).
- Not changing the pulse physics (port rise measured 1.8 ms / 33° at 3000 rpm, 1.3 ms at 6000:
  plausible).

## Model

**Sites.** Every abrupt area expansion along the exhaust path where the flow separates: primary →
collector junction, each collector → merge, pipe → silencer or catalyst shell (inlet cone), shell →
pipe (outlet), and the tail exit. One source per site, in the exhaust net, for both the waveguide
and the FV model.

**Level.** At a site with upstream (jet) area A_j, the instantaneous through-flow velocity
U(t) = ṁ(t)/(ρ A_j) (from the net's own flow, not a cycle mean; zero or reversed flow → no source),
local gas density ρ and sound speed c:

- below the duct's first cut-on f_co = 1.84 c / (π D) (D the downstream duct diameter):
  W = K · ρ · A_j · U⁴ / c (Davies & Holland 1999 §4);
- above f_co: the same W carried on with U⁶/c³, continuous at f_co:
  W = K · ρ · A_j · U⁴ / c · (U/c)² / M_co², where M_co is defined so the two meet (the plan pins
  the exact join; the slope change is Davies').

K is dimensionless and the **same for every engine**.

**Spectrum.** Broadband about a Strouhal number St = f·D_j/U ≈ 0.2–0.5 (shear-layer/vortex
shedding at an expansion; Davies 2001 observes St ≈ 0.5 for the chamber vortex). The plan pins
St and the shape against the reverted `flow_noise.rs` (Nelson–Morfey's K(St) table, still in
git at 089298f) — reuse its band synthesis.

**Injection.** Plane-wave pressure p = √(W·ρc / A_d) into the downstream duct (area A_d), half
each way, as the reverted junction code did; seeded RNG per site, deterministic.

## Calibration of K (once, for all engines)

1. Render every preset held at three points (idle-ish, 3000 at 30 %, 6000 at 100 %), with K = 0
   and with a K sweep.
2. Measure on each render the 0.8–5 kHz firing-rhythm modulation (and the band balance
   300–1k / 1–3k / 3–8k re 50–300 Hz).
3. Choose the single K whose median modulation across all presets × points equals the median of
   the 209 real steady segments (12 %), checking the band balance stays inside the real
   segments' p25–p75.
4. The script lives in `scripts/` with the real-segment numbers it targets; `docs/FACTS.md`
   records K, the date and the medians.

If no single K brings all presets into the real p25–p75 of modulation, the mechanism is wrong:
stop, and go to option 2 (constants only from papers we do not yet have: Kårekull 2015, Sakurai
SAE 2004-01-0399).

## Acceptance

- Every preset's modulation inside the real p25–p75 at the three points.
- The user's ear: loudness-matched before/after clips of the inline-4, K20A and LFA (the LFA's
  sound was approved on 2026-10-31 and must not get worse) before merging.
- Real-time cost: measured on the hot loop; must keep Ultra in real time where it is today.
- Tests: the U⁴/c law and the U⁶/c³ join from a hand calculation; a site with no flow is silent;
  determinism (same seed, same samples); YAML/JSON round trip if a field is added (none planned
  besides the existing `sound.*` mix, which gains one `duct_flow` entry defaulting to 1.0).

## Risks

- K absorbs Davies 2001's resonance amplification and microphone/environment effects of the
  recordings: one number for all is an assumption the calibration tests (step 3's stop rule).
- Real-segment modulation includes mechanical noise near the engine; onboard exhaust-mic segments
  are the cleaner subset and the calibration reports both.
- Our 1–5 kHz level vs rpm already scales like the recordings (≈ 40 dB/decade both), so that
  slope cannot validate the mechanism; the modulation metric is the test.

## Related, separate track (pure physics, Ange's list, no calibration)

Frequency-dependent reflection at pipe ends and real wall friction and heat transfer, aimed at our
excess above 3 kHz (3–8 kHz re 50–300 Hz: ours +2 dB open at 6000, Ange −13…−18 dB). Own design.
