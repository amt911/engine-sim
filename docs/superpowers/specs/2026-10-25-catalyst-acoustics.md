# Catalyst acoustics: the monolith's narrow channels

Date: 2026-10-25. Status: design. Roadmap: sound realism (the user: "everything sounds too
synthetic … generalize it to every engine").

## Why

Every road car has catalysts, and they shape what leaves the tail. Measured against a real one,
ours is wrong.

Elnemr (KTH licentiate thesis, 2011, §4.3.1, Figs. 94–102) measured the transmission loss of a
production catalyst ("CAT 1") on a two-load rig. Geometry: inlet Ø81 mm, 50 mm cone, an oval
monolith 133 × 116 mm and 130 mm long, 44 mm cone, outlet Ø62 mm. The cell density is not given.

| f (Hz) | 50 | 260 | 400 | 600 | 850 | 1000 | 1200 |
|---|---|---|---|---|---|---|---|
| Measured TL (dB) | 1.3 | 5 | 4.5 | 5.5 | 7 | 8 | 9.5 |
| Ours (400 cpsi) | 0.6 | 2.0 | 3.1 | 3.9 | 3.5 | 2.5 | 0.9 |

Ours falls where the measurement rises. A lossless expansion of those areas gives at most 3.7 dB.
The rest is the channels' viscous and thermal losses, which grow with frequency, and we model
none of them. The monolith is today a narrowed pipe with a frequency-independent laminar
resistance.

The LFA preset has no catalyst at all. In a quick A/B, adding one moved the V10's half-order
(bank) tone at 318 Hz by −11.5 dB, the direction of the real recording (≈14 dB under ours there).

## Model

1. **Narrow-tube propagation** (Zwikker & Kosten 1949; Stinson, JASA 89, 1991; Dokumaci, JSV
   210, 1998, for catalysts). A channel of hydraulic radius a carries plane waves with:
   - an effective density ρ_eff = ρ/(1 − F(k_v·a)), where k_v = √(−iω/ν) and
     F(x) = 2J₁(x)/(x·J₀(x));
   - an effective compressibility (A/γp)·(1 + (γ − 1)·F(k_t·a)), where k_t = √(−iω·Pr/ν).

   Square channels are taken as circular of the same hydraulic radius; Dokumaci 1998 finds the
   difference small. Low-frequency limits, as checks: Poiseuille's 8μ/a² resistance and a 4/3
   mass excess.
2. **In the time domain.** The excess series impedance i·ω·(ρ_eff − ρ), in the dimensionless
   x = ωa²/ν, is fitted as a Foster network: r₀ plus parallel-RL branches with fixed,
   log-spaced time constants. That fit is linear least squares, and every element must come out
   positive (passive). The excess shunt admittance is fitted the same way with series-RC
   branches.
   - The fit is dimensionless, so one set of constants serves every cell size, gas temperature
     and pressure.
   - The coefficients are made by `scripts/fit_narrow_tube.py` (numpy; Kelvin-function series).
   - A Rust test compares the network with the exact function, computed in the test from the
     Kelvin series, within a stated tolerance over the band.
3. **The brick as a ladder.** Lossless pipes of the open area, about 1 cm per section. Each node
   carries the series network (a new `Node`, the dual of `Shunt`) and the thermal shunt. The
   steady flow resistance is the network's DC value: it equals the Shah & London laminar
   resistance used today (f·Re = 56.91 for square channels, against 64 for circular; the
   difference is documented and tested).
4. **Mean flow.** At M ≤ 0.1 Elnemr's Fig. 100 shows no flow effect, so it is ignored, as Dokumaci
   finds for laminar channel flow.

## Tests (values from outside the code)

- **Narrow-tube function:** the low-frequency limits (8μ/a², 4/3 ρ); the boundary-layer limit
  (Kirchhoff's α at s ≫ 1); the fitted network against the exact function within ±3 % in
  magnitude for 0.3 ≤ s ≤ 60.
- **Series network node:** the transmission loss of a known series impedance in a duct,
  20·log|1 + Z·S/(2ρc)| (Kinsler ch. 10), at two frequencies.
- **Validation:** Elnemr's CAT 1 measured transmission loss at 50, 260, 600, 1000 and 1200 Hz,
  within ±1.5 dB. The cell density is unknown, so it is taken as the common 400 cpsi with
  6.5 mil walls (0.165 mm), stated in the test.
- **Engines:** a catalyst lowers the 1–4 kHz band at the tail and leaves the cycle-mean flow and
  torque unchanged (its pressure drop is the existing resistance).

## Then (separate PR)

Every road-car preset gets its catalysts. The LFA has close-coupled ones per bank (Lexus). Sizes
come from the swept volume rule already in `road_exhaust`, flagged as a rule. Renders and
fingerprints are compared before and after.

## Out of scope

Heat transfer from the gas to the monolith (light-off), and diesel particulate filters.
