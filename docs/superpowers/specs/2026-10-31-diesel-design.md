# Naturally aspirated diesel — design

Status: written unattended (user away, roadmap delegated 2026-10-31: "continúa y no pares"). It is
item 4 of the user's queue (2026-10-22: "compression ignition, fuel-quantity load with no throttle,
ignition delay, premixed + diffusion burn (clatter), low redline. Naturally aspirated first, turbo
later"). Decisions marked *(my call)* are open to the user's review.

## Why

Every engine today is a spark-ignition, premixed, throttled petrol engine. A diesel sounds
different for physical reasons the model can already carry once its combustion is right:

- the fuel is injected late and lights by itself after an **ignition delay**; what mixed during the
  delay burns almost at once (**premixed** phase, a sharp pressure rise), the rest as it mixes
  (**diffusion** phase, slow). The sharp rise is the diesel knock / clatter: the block radiates it
  through the existing Anderton structure response (SAE 790270 was written for diesel noise);
- there is **no throttle**: the pedal sets the fuel per cycle; the manifold stays at atmosphere,
  so the intake and exhaust pulses at idle are as strong as at full load and the idle is loud;
- high compression (18–22:1), long stroke, heavy flywheel, low redline (a governor).

## Goals

- `EngineDef::combustion`: `Spark` (today's model, the default) or `Diesel { cetane, … }`.
  Serde with a default, round-trip tested (JSON and YAML). Old files load unchanged; with
  `Spark` every render is bit-identical to main.
- In `Diesel`: air-only induction; fuel injected at a start-of-injection angle (SOI) from a
  map over speed and fuel; the ignition delay integrated from the cylinder's own p, T; heat
  released by the Watson double Wiebe; the pedal sets fuel; an idle and a top-speed governor on
  the fuel; no overrun of fuel at closed pedal above idle (it is already zero fuel).
- A preset: **VW 1.9 SDI** (AEY/ASY): direct injection, naturally aspirated, 1896 cc,
  79.5 × 95.5 mm, CR 19.5, 47 kW at 4200 rpm, 125 N·m at 2200–2600 rpm, firing 1-3-4-2
  (VW SSP 183 *(to cite precisely when implementing)*; Wikipedia "VW EA827/EA188 1.9 SDI").
- Tests with values from outside the code: the delay correlation against Heywood's worked
  numbers, the Wiebe functions against their closed forms, the preset against its published
  power and torque, and the clatter: peak dp/dθ of the diesel ≥ 3× the K20A's at the same mean
  effective pressure (Heywood Fig. 10-1/10-4: 0.5–1 MPa/° for DI vs ≈ 0.1–0.2 for SI).

## Non-goals

- Turbo (next), common rail with pilot/multiple injections (one injection per cycle *(my call)*),
  indirect injection / prechamber, NOx/soot chemistry. Glow plugs and cold start.
- A spray model. The double Wiebe is the standard zero-D stand-in.

## Physics (cited)

- **Ignition delay** (Hardenberg & Hase, SAE 790493; Heywood, *ICE Fundamentals* 1988, eq. 10.37; 2nd ed. eq. 10.35):
  τ_id [°CA] = (0.36 + 0.22·S̄_p)·exp[E_A·(1/(R̃T) − 1/17 190) + (21.2/(p − 12.4))^0.63],
  E_A = 618 840/(CN + 25) J/mol, p in bar, T in K, S̄_p in m/s. Applied as a Livengood–Wu
  integral ∫dθ/τ_id(p(θ), T(θ)) = 1 from SOI (Heywood §9.6.3), so the delay follows the
  compression it happens in.
- **Heat release** (Watson, Pilley & Marzouk, SAE 800029; Heywood 1988 eqs. 14.47–14.49, a, b, c within its ranges 0.8–0.95, 0.25–0.45, 0.25–0.5): premixed fraction
  β = 1 − 0.926·φ^0.37/τ_id^0.26 (τ_id in ms, φ the overall equivalence ratio); premixed
  f₁(t) = 1 − (1 − t^C₁)^C₂, C₁ = 2 + 1.25·10⁻⁸·(τ_id·N)^2.4, C₂ = 5000; diffusion
  f₂(t) = 1 − exp(−C₃·t^C₄), C₃ = 14.2/φ^0.644, C₄ = 0.79·C₃^0.25; t = (θ − θ_soc)/Δθ_c,
  Δθ_c = 125° *(Watson's normalising duration)*. Released fraction β·f₁ + (1 − β)·f₂.
- **Fuel**: diesel LHV 42.5 MJ/kg, stoichiometric A/F 14.5 (Heywood App. D). Full-load fuel
  from the smoke limit, λ ≥ 1.4 at full load *(my call, Heywood §1.8: DI NA engines run
  λ 1.4–1.6 at full load)*.
- **SOI map**: static timing a few degrees BTDC at idle, advancing with speed (the injection pump's
  advance): *(my call)* linear from 2° BTDC at idle to 12° BTDC at rated speed, minus 2° at low
  fuel; the preset may override.
- **Governors**: idle: a PI on fuel holding idle rpm (the existing idle trim acts on spark: in
  `Diesel` it acts on fuel); top: fuel ramps to zero over the last 300 rpm before the redline
  (a mechanical governor's droop *(my call)*).

## Shape

- `crate::combustion`: `ignition_delay_deg(p_bar, t_k, piston_speed, cetane)`,
  `watson_premixed_fraction(phi, tau_ms)`, `watson_burned(t, beta, tau_n, phi)`; pure, unit-tested.
- `EngineDef::combustion: Combustion` (`#[serde(default)]`). `Engine` branches once per
  cylinder event: at IVC the charge is air (+ residual); at SOI the fuel is added; each step
  the delay integral advances; at its end a `Burn` starts with the Watson shape instead of the
  single Wiebe (the `Burn` keeps its released fraction as today, so unburnt fuel at EVO and the
  fuel ledger keep working).
- The throttle plate stays fully open in `Diesel`; `Controls::throttle` is the pedal → fuel.
- UI: nothing new beyond the preset (the pedal is the same control) *(my call)*.

## Done when

- Unit tests above; serde round trip; bit-identical renders for every existing preset.
- VW SDI preset: dyno within 15 % of 64 PS / 125 N·m (the model runs ~12 % low on power for the
  petrol presets); idles at its idle speed; the clatter test passes.
- Gate green, PR with a manual test plan and verification; clips (idle, drive) sent to the user.
