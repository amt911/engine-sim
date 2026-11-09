# Physics validation inventory

**Purpose.** When the engine sounds wrong we want to rule out whole components. That works only if
each scientific formula is pinned by a test against a value from **outside the code**: a closed
form, a textbook or paper table or figure, or a hand calculation, with the reference. This file is
the inventory of what exists today (2026-11-08, branch `test/physics-references`, read-only
analysis, nothing was run) and of the gaps, ranked by how much they matter for sound.

**Scope.** Every physics/DSP module in `crates/engine-core/src/*.rs` and `crates/engine-audio/src/*.rs`.
Config/serde plumbing, UI and `units.rs` conversions are skipped (units: round-trip and the 94 dB
re 20 µPa check only). Paths are relative to `crates/engine-core/src/` unless prefixed `audio:`
(= `crates/engine-audio/src/`). `file:line` is the `fn` line of the test.

**Legend**

- ✅ **external** — a test asserts a value that does not come from running the code: closed form,
  table, figure read off a paper, or a number worked by hand from the cited formula. Tags in the
  *Reference* column: `[hand]` a hand evaluation of the cited formula (checks the implementation,
  not that the formula is the right model), `[closed]` an exact closed form or limit, `[table]` a
  published table, `[fig]` a value read off a published figure (tolerance = reading error),
  `[meas]` a published measurement, `[ind]` an independent implementation (other language).
- 🟡 **self-consistent only** — tests exist, but they check conservation, symmetry, monotonicity,
  "stays finite", loose plausibility bands, or compare the code with itself (another path, or the
  same function used to build the expectation).
- ❌ **untested** — nothing found that exercises the model's numbers.

**Reading depth.** I read the bodies of the tests that carry the physics (all of `gas`, `fv`,
`waveguide`, `valve_boundary`, `jet_noise`, `listener`, `thermo`, `combustion`, `pipe_heat`,
`turbo`, `friction`, `mechanical`, `structure` headers, the named tests in `exhaust`, `exhaust_fv`,
`intake_fv`, `engine`). For the remaining ~150 `engine.rs` behaviour tests (idle, restart, round
trips, presets) I judged from name and leading comment only; they are scenario/robustness tests
and none was counted as ✅. Where I am not sure, the cell says so.

Most of the code base is better than the usual: nearly every physics test has a hand calculation
with its numbers in a comment. The weakness is therefore not "no external values" but
(a) **model-level validation**: tests check that the *implementation* matches the cited formula,
rarely that the formula reproduces a **measured or exact-solution curve**; and (b) a few
sound-critical models have only qualitative tests (list in *Gaps, ranked*).

---

## 1. Gas dynamics (0D gas, 1D finite-volume solver, nonlinear waveguide)

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Isentropic exponents, critical pressure ratio, choked factor | `gas.rs` consts | `gas.rs:179` | p*/p0 = 0.5283 (Anderson, *Modern Compressible Flow*, Table A.1); exponents 5 and 7 for γ = 1.4 `[table][closed]` | ✅ |
| Orifice mass flow, choked and subsonic (Heywood App. C, eq. C.8–C.9) | `gas.rs::orifice_mass_flow` | `gas.rs:217`, `gas.rs:224`; continuity at the switch `gas.rs:237` | 0.046671 kg/s choked, 0.038215 kg/s at p-ratio 0.8 `[hand]` | ✅ |
| 0D volume exchange, reservoirs, enthalpy carry, half-mass cap | `gas.rs` | `gas.rs:245`, `:260`, `:269`, `:286`, `:309`, `:330`, `:352`, proptest `:371` | conservation, no crossing, no negative mass | 🟡 |
| Ideal-gas state round trip | `gas.rs::GasVolume` | `gas.rs:208` | m = pV/RT = 1.99104e-3 kg `[hand]` | ✅ |
| HLLC Riemann flux (Toro, *Riemann Solvers*, ch. 10) | `fv.rs::hllc` | Sod: `fv.rs:752`; sonic point of a transonic rarefaction `fv.rs:1443`; moving contact `fv.rs:1486`; consistency `fv.rs:737` | Sod/Toro Test 1: p* = 0.30313, u* = 0.92745, ρ*L = 0.42632, ρ*R = 0.26557 (2–5 % tol., N = 400) `[table]`; sonic flux 171.3 kg/(m²s) from Toro eq. 4.56/4.63 `[closed]`; contact exact (Toro §10.4) | ✅ Tests 1, 3, 4, 5 (`fv.rs:818`, `:833`, `:855`, 3 % tol., N = 1000, ends pinned to the initial states); Test 2 (123, near-vacuum) is a **finding**: `fv.rs:801` (`#[ignore]`) |
| HLLC symmetry / near-vacuum robustness | `fv.rs` | `fv.rs:1471`, `:1293`, `:1503` | symmetry, stays physical | 🟡 |
| WENO5-Z reconstruction (Borges et al. 2008) | `fv.rs` | `fv.rs:1154` exact for quadratics, `:1178` order at a crest, `:1203`, `:1215` acoustic pulse, `:1372` advected bump, `:1383` cheap scheme 2nd order | exactness for quadratics `[closed]`; measured orders (3.55 for acoustic pulse with RK3 time error, not the nominal 5; stated in the test comment) | ✅ (order verification; self-convergence) |
| SSP-RK3 time stepping (Shu & Osher 1988) | `fv.rs` | only through the combined convergence tests above | none stand-alone | 🟡 |
| Area step in a pipe | `fv.rs` | `fv.rs:888` | T = 2A1/(A1+A2) = 0.4 and 1.6 (linear theory, Munjal / Kinsler) `[closed]` | ✅ |
| Wall friction, Darcy | `fv.rs` | `fv.rs:901` | Δp = f(L/D)·½ρu² = 540 Pa `[hand]` | ✅ |
| Acoustic wall loss α (Np/m), mean-flow-following | `fv.rs` | `fv.rs:933`, `:972`, `:1018` | e^(−αd) = 0.93922; ODE solution 17.15 / 0.556 m/s `[hand]` | ✅ (implements α as given; α's own value vs Kirchhoff is ❌) |
| Sound speed / pulse travel | `fv.rs` | `fv.rs:1086` | x = 0.2 + c·t = 0.544 m `[closed]` | ✅ |
| Open-end boundary: inflow by Bernoulli, outflow at ambient pressure | `fv.rs::open_end` | `fv.rs:624`, `:646` | v = −9.945 m/s, p = p0 − 59.34 Pa `[hand]` | ✅ |
| Ghost cells, wall/open/reservoir ends | `fv.rs` | `fv.rs:1556`, `:1601`, `:1055` | structural | 🟡 |
| Nonlinear steepening in FV | `fv.rs` | `fv.rs:1623` | "large pulses steepen, small do not" (front width comparison) | 🟡 |
| Closed-tube conservation, positivity | `fv.rs` | `fv.rs:1076`, `:796`, `:1503` | conservation | 🟡 |
| Waveguide simple-wave speed c + (γ+1)/2·u | `waveguide.rs` | `waveguide.rs:1448`, `:2081` | 500·(1 + 1.2·p/(ρc·500)) (Msallam et al., Acta Acustica 86, 2000, eq. 1–2) `[hand]` | ✅ |
| Shock speed (Rankine–Hugoniot for a simple wave) | `waveguide.rs` | `waveguide.rs:1368`, `:1389`, `:1417` | arrival at 184.401 steps (Whitham §2.4) `[hand]`; content kept, faint wave linear: 🟡 | ✅ |
| Fubini harmonics of a loud tone | `waveguide.rs` | `waveguide.rs:1472` | B1 = 0.99808, B2/B1 = 0.061786 (Hamilton & Blackstock ch. 4; scipy Bessel) `[ind]` | ✅ |
| Delay line, fractional delay, pipe delay L/c | `waveguide.rs` | `waveguide.rs:2003`, `:2021`, `:1634`, `:1647` | 0.55 m / 550 m/s = 48 steps `[closed]` | ✅ |

## 2. Valves, ports and boundaries

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Valve lift (sin² bump), curtain/port flow area | `valve.rs::flow_area` | `valve.rs:215`, `:455`, `:463`, `:469`, `:484` | 6.3884e-4 m² at 10 mm lift, 35 mm valve, Cd 0.7 `[hand]` | ✅ (impl) |
| Curtain-to-port area blend vs L/D | `valve.rs` | `valve.rs:428` | ~95 % of geometric min below L/D 0.2, ~90 % at 0.25 (Heywood Fig. 6-18, p. 229) `[fig]`, two points plus a slope-continuity check | ✅ (two points) |
| Rotor edge uncovering a round port (circular segment) | `valve.rs` | `valve.rs:236`; `geometry.rs:121` | segment area 2.4567e-4 → 1.7197e-4 m² `[closed]`; flank speed R/2 per radian `[hand]` | ✅ |
| VTEC lobe switch, phaser, hysteresis | `valve.rs` | `valve.rs:272`, `:292`, `:306`, `:325` | lift at mid-window 8.8 / 12.65 mm, 12.5° by hand `[hand]` | ✅ (logic) |
| Valve as pipe boundary, small-valve limit (nozzle) | `valve_boundary.rs` (Benson vol. 1 §4.1, §6.6) | `valve_boundary.rs:453`, `:632` | 1.834e-3 / 5.113e-3 kg/s (Heywood App. C) `[hand]` | ✅ |
| Wave superposition by amplitude ratio (Blair §2.1–2.2) | `valve_boundary.rs` | `valve_boundary.rs:434`, `:819`, `:838`, `:864` | X = 1.5^(1/7) → 2.2006 bar vs 2.0 linear; λ = 1.028571; 0.3439 bar `[hand]` | ✅ |
| Inflow root (Benson 6.35–6.37) | `valve_boundary.rs` | `valve_boundary.rs:487`, `:828` | A = 1.075171, U = 0.124145, A⁵U/φ = 0.892 (bisection by hand) `[hand]` | ✅ |
| Weak outflow = acoustic impedance ρ·a | `valve_boundary.rs` | `valve_boundary.rs:919` | δp = ρ·a·u (Kinsler §9.2) `[closed]` | ✅ |
| Finite-ψ outflow, Jenny constant-pressure model (Benson eq. 6.43–6.62) | `valve_boundary.rs` | `valve_boundary.rs:471`, `:516`, `:549`, `:570`, `:809`, `:851`, `:883`, proptest `:938` | energy continuity, continuity through the sonic point, solver converges | 🟡 (no published Benson/Jenny worked example) |
| Fully open valve (ψ ≥ 1) = isentropic pipe flow from the cylinder | `valve_boundary.rs` | `valve_boundary.rs:960` | π = 0.95: p/p_c = 0.698337, u/a_c = 0.698212 (energy + isentropy by hand) `[closed]` | ✅ |
| Newton/Illinois/secant root finders | `valve_boundary.rs`, `exhaust_fv.rs` | `valve_boundary.rs:590`, `:610`, `:785`; `exhaust_fv.rs:1307`, `:1376` | x³−2x−5 root 2.0945514815 (Newton's own); ∛2 `[closed]` | ✅ |
| 0D junction (Benson constant-pressure) ghost states | `exhaust_fv.rs` | `exhaust_fv.rs:1177`, `:1203`, `:1237`, `:1457`, `:1656`; throttled end `:1393` | critical ratio 0.5283 (Anderson eq. 5.21) `[closed]`; stagnation-state entropy/enthalpy by definition; orifice equality | ✅ for the choked ratio, 🟡 for the rest |
| One-to-one junction transparent | `exhaust_fv.rs` | `exhaust_fv.rs:1082` | equals the same pipe without a junction `[closed limit]` | ✅ |
| Collector end: pressure and volume flow to duct | `exhaust_fv.rs` | `exhaust_fv.rs:791` | p = p_atm + 568.4 Pa, p⁺ = −431.6 Pa `[hand]` | ✅ |
| Valve discharge reverse/forward Cd ratio (Blair measured maps) | `engine.rs` | `engine.rs:9089`, `:9109` | qualitative (reverse choked inflow ∝ area) | 🟡 |
| Throttle with trumpet Cd, runner cap | `intake_fv.rs` | `intake_fv.rs:1263`, `:1272` | loss 3 → Cd 0.5 (4 cm² at 0.5 = 2 cm² at 1) `[hand]` | ✅ (impl) |
| Valve-seat impact pulse (half-sine impulse) | `engine.rs` | `engine.rs:5950` | ∫ = 2Fτ/π = 27.885 mN·s `[closed]` | ✅ for the impulse; the peak force (292 N) and its scaling with speed are not validated ❌ |
| Port-cell enthalpy/backflow bookkeeping | `intake_fv.rs`, `exhaust_fv.rs` | `intake_fv.rs:1523`, `:1574`; `exhaust_fv.rs:1786` | regression of known bugs (mass/enthalpy kept) | 🟡 |

## 3. Combustion, thermodynamics, heat transfer, fuel

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Wiebe burn fraction | `combustion.rs::wiebe` | `combustion.rs:316`, `:408` | 1−e^(−5/8) = 0.464739 at half duration `[closed]` | ✅ |
| Watson double Wiebe (diesel) | `combustion.rs` | `combustion.rs:430`, `:440` | β = 0.23347; f1(0.05) = 0.42025; f2(0.3) = 0.92991 `[hand]` | ✅ (impl) |
| Hardenberg–Hase ignition delay (Heywood 1988 eq. 10.37) | `combustion.rs` | `combustion.rs:417` | 7.533 °CA `[hand]` | ✅ (impl) |
| Laminar flame speed (Metghalchi & Keck 1982) | `combustion.rs` | `combustion.rs:333`, `:357` | 26.256 cm/s at φ = 1; 0.231562 m/s at φ = 1.2, 4 atm `[table]` | ✅ |
| Burn-angle scaling (Hires, Tabaczynski & Novak, SAE 780232) | `combustion.rs` | `combustion.rs:375` | (S_p ν)^⅓ (h/S_L)^⅔ = 1e-3 `[hand]` (scaling only; absolute angles are fitted) | ✅ (scaling) |
| Spark retard torque loss (Heywood §9.2.4 eq. 9.29) | `engine.rs` | `engine.rs:4967` | 0.99 at 5°, 0.97 at 10°, 0.69 at 40° `[fig/eq]` | ✅ |
| Chamber first acoustic mode (Draper) | `combustion.rs` | `combustion.rs:285` | 1.841·c/(πB) = 5897 Hz `[hand]` | ✅ |
| Combustion step struck by a premixed share | `combustion.rs` | `combustion.rs:249` | (γ−1)Q/V = 4 MPa step, a tenth to the chamber `[hand]`; the factor 1/10 is the code's own estimate | 🟡 (factor unsourced) |
| Woschni heat-transfer coefficient (SAE 670931; Heywood eq. 12.19) | `engine.rs::woschni_htc`, `woschni_velocity` | `engine.rs:6290` | 453.76 W/(m²K) at B 0.1, 5 MPa, 2000 K, 10 m/s; C1 = 6.18/2.28, 3.24e-3 `[hand]` | ✅ (impl; not compared to a measured heat flux) |
| Wall heat share of fuel energy | `engine.rs` | `engine.rs:6364` | 15–35 % at part load (Heywood §12.1) `[range]` | 🟡 (loose band) |
| NASA/JANAF species c_p | `thermo.rs` | `thermo.rs:307`, `:318` | CO₂ 60.350 J/(mol K) at 2000 K etc. (NIST-JANAF), within 0.3 % `[table]` | ✅ |
| Mixture γ vs T (Heywood Fig. 4.13) | `thermo.rs` | `thermo.rs:337` | 1.400 / 1.335 (air 300 / 1000 K); 1.355 / 1.267 (φ = 1 unburned) ±0.005 `[fig]` | ✅ |
| Gas constants of burned/unburned (Heywood Table 4.7) | `thermo.rs` | `thermo.rs:348` | R = 289.06 / 273.43 / 287.0 `[table]` | ✅ |
| Mixing (mass-weighted), datum, internal-energy inversion | `thermo.rs` | `thermo.rs:357`, `:368`, `:391`, `:402`, `:428` | linearity, continuity | 🟡 |
| Fuel–air Otto cycle | `thermo.rs` | `thermo.rs:446` | η in 0.45–0.49 vs Heywood Fig. 5.9 ≈ 47 % `[fig]` | ✅ (band 4 points wide) |
| Cylinder pressure from first law, motored compression | `engine.rs` | `engine.rs:6974` | peak in 8–26 bar; ideal adiabatic 25 bar quoted but not asserted | 🟡 (loose) |
| Net indicated efficiency at WOT | `engine.rs` | `engine.rs:7162` | 35–40 % real engines (Heywood ch. 5) `[range]` | 🟡 (loose band) |
| Cycle-to-cycle variation (Heywood §9.4, Fig. 9-34/9-35) | `engine.rs` | `engine.rs:4709`, `:4732`, `:4789`, `:4807`, `:5111` | σ of burn angles ≈ 0.12 of mean etc. `[fig]` | ✅ (loose) |
| Port wall film (Aquino x–τ, SAE 810494) | `fuel.rs` | `fuel.rs:206`, `:246`, `:255`, `:266`, `:277` | 1−e^(−1) = 0.632121; geometric tail x·m·(1−e)^k; 5.3333e-5 kg `[closed]` | ✅ |
| Unburnt share, afterburn in hot exhaust (Heywood §11.6.3) | `fuel.rs` | `fuel.rs:289`, `:296`, `:332`, `:400`, `:423`, `:440` | 14.7:1, 44 MJ/kg, e-folding, 27.8133 J `[hand]` | ✅ (impl; kinetics time constant unsourced) |
| Exhaust-port gas heat transfer (PORT_HTC = 200 W/m²K, "Heywood §12.6: 100–500") | `engine.rs:2403` | none found for the constant; mixing test `engine.rs:6087` | – | ❌ |
| Rotary chamber volume (Heywood Fig. 1.23) | `geometry.rs` | `geometry.rs:187`, `:201`; `engine.rs:7724` | 3√3·e·R·B = 654.7 cm³ `[closed]` | ✅ |
| Slider-crank kinematics | `geometry.rs` | `geometry.rs:131`, `:145`, `:152`, `:162`, proptest `:172`, `:178` | 0.049623 m at 90° `[hand]`; V = πB²S/4 | ✅ |

## 4. Exhaust acoustics (pipes, junctions, chambers, silencers, catalysts, open end)

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Area step / Kelly–Lochbaum junction | `waveguide.rs::junction` | `waveguide.rs:2216`, `:1302`; collector `exhaust.rs:3921`, `exhaust.rs:2796` | r = (A1−A2)/(A1+A2) = −½; 2A1/ΣA; out = −1/3, 2/3 `[closed]`; `waveguide.rs:2395` adds T-junction −⅓/⅔, A₂ = 3A₁ −½/½, A₂ = 99A₁ −0.98/0.02, contraction +½/1.5 (exact) | ✅ |
| Borda–Carnot expansion and sudden-contraction loss at junctions (White §6.9; K = 0.42(1−σ)) | `waveguide.rs::lossy_junction` | `waveguide.rs:1328`; `exhaust.rs:3892` | p_J = 3904.71 Pa, reflection −5618.83 Pa `[hand]` | ✅ (impl; K = 0.42 is a White correlation, not re-checked) |
| Area change in flow as a resistance (Ronneberger 1967; Davies 1988; Munjal §2.13) | `waveguide.rs::area_change_resistance` | `waveguide.rs:1546`, `:1560`, `:1727` | R = 7031.3 Pa·s/m³; transmission 0.38475 `[hand]` | ✅ (impl) |
| Expansion chamber transmission loss, no flow | `waveguide.rs`, `silencer2d.rs` | `waveguide.rs:1977`; `silencer2d.rs:699` | TL = 10log[1 + ¼(m−1/m)² sin²kL]: 9.42 / 12.17 / 0 dB (m = 8); 15.0 dB at kL = π/2 (m = 11.257) (Munjal ch. 3) `[closed]` | ✅ |
| Expansion chamber TL **with mean flow** (Munjal §3.x, Mach terms) | waveguide pipes do not convect | none | – | ❌ |
| Series resistance / series and shunt networks TL | `waveguide.rs` | `waveguide.rs:1781`, `:1803`, `:1831`, `:2155` | 20log(1 + RS/2ρc) = 3.522 dB; 3.979, 6.021 dB (Kinsler ch. 10) `[closed]` | ✅ |
| Side Helmholtz resonator TL | `waveguide.rs::helmholtz` | `waveguide.rs:1858`, `:1883` | f0 = 172.75 Hz; TL 3.468 / 13.315 dB; resistive 3.522 / 3.493 dB (Kinsler ch. 10) `[closed]` | ✅ |
| Pipe friction damping α = f·M/(2D) | `waveguide.rs`, `exhaust_fv::PIPE_FRICTION` | `waveguide.rs:1612`, `:1670`, `:1930`, `:2114` | e^(−0.05) = 0.951229 `[hand]`; **the factor f = 0.025 is a constant** | ✅ (impl) / ❌ for f(Re, roughness) vs Moody/Colebrook |
| Pipe viscothermal loss | `waveguide.rs`, `narrow_tube.rs` | `waveguide.rs:2246` (loss scaling `1−0.02` per m); Kirchhoff α(1 kHz, 600 K air, a = 21 mm) = 0.0576 Np/m `waveguide.rs:2347` ✅ (2 %); wall filter vs √f law over 3 m: 4 kHz `:2384` ✅ (10 %), 1 kHz `:2390` **finding** (0.0851 vs 0.0956 Np, −11 %) | Kirchhoff via Tiikoja et al. SAE 2016-01-1816 Eq. 6 `[closed]`; air properties Çengel & Ghajar Table A-15 interpolated `[table]` | ✅ α; ✅ at 4 kHz; 🟡 at 1 kHz |
| Lined duct (locally reacting liner, Munjal ch. 6) | `lining.rs`, `waveguide.rs` | `lining.rs:327`, `:145`, `:243`; `waveguide.rs:1591` | Re γ = 0.6488 / 2.4839 / 8.4981 Np/m at 1/2/4 kHz; 2.25 / 8.63 / 29.5 dB over 0.4 m `[hand]` | ✅ |
| Packed perforated dissipative silencer (2D mode matching, Selamet et al.) | `silencer2d.rs` | `silencer2d.rs:715`; `exhaust.rs:3551` | JSV 262 Fig. 7: 21, 29, 36, 30, 22, 18 dB at 0.5–3 kHz `[fig]`; 16.48 dB per 0.4 m at 1 kHz from an independent scipy prototype `[ind]` | ✅ |
| Silencer TL → cascade of peaking biquads (Välimäki & Liski) | `silencer2d.rs` | `silencer2d.rs:735`, `:758`, `:782`; `exhaust.rs:3364` | within 1.5 dB at fitted points; |H| ≤ 1; RBJ gain at centre `[closed]` | ✅ (fit quality) |
| Measured chamber TL imposed (LFA final chamber, JP 2008-169751 A Fig. 6) | `exhaust.rs` | `exhaust.rs:3364` | 15 dB at 100 Hz, ≈ 20 dB flat 200 Hz–1 kHz `[meas]` (input, so it tests the mechanism) | ✅ (mechanism) |
| Catalyst: laminar channel resistance (Poiseuille/hydraulic) | `exhaust.rs` | `exhaust.rs:3183`; `narrow_tube.rs:118` | 12 699.6 Pa·s/m³; 8μ/(a²A) = 87 272.7 `[hand][closed]` | ✅ |
| Catalyst viscothermal channels (Zwikker & Kosten, Stinson, Dokumaci) | `narrow_tube.rs` | `narrow_tube.rs:126` | Foster fit within 1.5 % of F(z) = 2J1(z)/(zJ0(z)) **evaluated by the test-local `f_zk`**; `f_zk` itself now checked at real z = 1, 2 against J0/J1 tables (`narrow_tube.rs:152`, 0.5 %); no complex-argument table found | 🟡 (real axis ✅) |
| Catalyst measured TL (Elnemr, KTH 2011, Fig. 100) | `exhaust.rs` | `exhaust.rs:2610` | 1.3 dB at 50 Hz, 5.0 at 260, 5.5 at 600, 8.0 at 1000, 9.5 at 1200; model within 5 dB and never above `[meas]` | ✅ (loose: 5 dB window) |
| Orifice/butterfly head loss ½ρ(1/Cc − 1)² and leak ring 4c/D | `exhaust.rs` | `exhaust.rs:2930`, `:3002`, `:3066`, `:3078` | 236.95·½ρ; 0.4087, 5.1924 of ½ρ; 0.023529 `[hand]` | ✅ (impl; Cc = 0.61 orifice value) |
| Perforated baffle damping a Herschel–Quincke path | `exhaust.rs` | `exhaust.rs:2938`, `:3040` | "cancels less" (qualitative); arrival 172.8 steps `[hand]` | 🟡 / ✅ for timing |
| Two open pipes cancel where half a wave apart | `exhaust.rs` | `exhaust.rs:2889` | f = c/2(L2−L1) = 625 Hz; 9.16× at 400 Hz (Kinsler §10.3) `[closed]` | ✅ |
| Wall transmission (limp mass law, stiff panel) | `exhaust.rs` | `exhaust.rs:2536`, `:2559` | v = 7.0658e-5 m/s; 8.864e-8 m³/s (Kinsler §6.4, §1.7) `[closed]` | ✅ |
| Exit losses, tail exit orifice (choked/subsonic inversion) | `exhaust.rs`, `engine.rs::tail_exit` | `exhaust.rs:3194`; `engine.rs:10316`, `:10344` | 0.96711 kg/s → 2 atm back-pressure, jet 456.435 m/s; 0.209798 kg/s subsonic `[hand]` | ✅ |
| Open end reflection, convective loss (1−M)/(1+M) (Munt 1977; Peters et al. JFM 256) | `waveguide.rs::OpenEnd` | `waveguide.rs:1759`; `exhaust_fv.rs:826` | 0.98·0.83/1.17 = 0.6952; 1/3 at M = 0.5 `[closed]` | ✅ |
| Open end reflection vs ka (Levine & Schwinger 1948) | `waveguide.rs::OpenEnd` (two-pole H(s) = 1/(1 + 1.2266τs + 0.2523τ²s²), τ = a/c; no extra loss factor) | `open_end_inverts_low_frequencies` (|R| = 1 at DC); |R| at ka = 1, ½, ¼ vs Silva et al. 2009 fit of Levine–Schwinger, 2 % ✅; end correction 0.6133 a, 2 % ✅ (fixed 2026-11-08, was 0.419 a); ka = 2 **finding** (`#[ignore]`) | ka = 2: expected 0.346, code **0.392 (+13 %; was 0.447)** | ✅ for ka ≤ 1 and the end correction; ka = 2 open |
| Exhaust gas viscosity (Sutherland) | `exhaust.rs` | `exhaust.rs:3294` | 3.0164e-5 Pa·s at 600 K (White's table 3.06e-5) `[table]` | ✅ |
| Gas cooling along pipes, Dittus–Boelter | `pipe_heat.rs` | `pipe_heat.rs:158`, `:168`, `:175`, `:246` | Nu = 0.023 Re^0.8 Pr^0.3: h = 61.49 W/m²K; laminar Nu = 3.66; exp(−NTU) `[hand][closed]` | ✅ |
| Wall thermal model (steel pipe, radiation) | `pipe_heat.rs` | `pipe_heat.rs:194`, `:205` | ≈ 40 s time constant ("≈", approximate) | 🟡 |
| FV vs waveguide in the linear regime | `exhaust_fv.rs` | `exhaust_fv.rs:873` | agree within +4.9, +5.9, −3.4 dB (measured gap, not a reference) | 🟡 |
| Exhaust system as a whole vs measured car | `engine.rs` | `engine.rs:9804` | Davies & Holland JSV 223 (1999) Fig. 1(c): ≈ 107 dB(A) at 6000 rpm, 0.25 m, within 3 dB `[meas]` | ✅ (system level, one point) |
| Network travel time and bookkeeping | `exhaust.rs` | `exhaust.rs:3988`, `:3674`, `:3706` | 5.413 ms = 1039.3 steps `[hand]` | ✅ |
| Backward-compat "captured" net | `exhaust.rs` | `exhaust.rs:2360` | captured on an older commit (see golden-number list) | 🟡 |

## 5. Intake

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Airbox + ducts Helmholtz (FV ringing) | `intake_fv.rs` | `intake_fv.rs:1391`, `:1340` | 64.6 Hz with L' = L + 0.85a + 0.6a (Kinsler §10.8–10.9; Rayleigh, Levine–Schwinger end corrections) `[closed]` | ✅ |
| Airbox low-pass by transfer matrices | `intake_fv.rs` | `intake_fv.rs:1430` | mouth/valve volume velocity from Munjal ch. 2 `[hand]` | ✅ |
| Lumped air cleaner (C, M, R) | `air_cleaner.rs` | `air_cleaner.rs:186`, `:226`, `:248`, `:258` | f_H = 55.4 Hz, −23.5 dB at 4 f_H; peak loss 17.0 dB (Munjal §3.3) `[closed]` | ✅ |
| Air cleaner insertion loss vs measurement (Raff & Perry 1973: 8.5 dB(A)) | `engine.rs` | `engine.rs:10153` | band 5–15 dB(A) around 8.5 `[meas]` | 🟡 (loose, one truck diesel) |
| Runner (ram) tuning: quarter-wave resonance of an FV runner | `intake_fv.rs` | `intake_fv.rs:1080` (mentions ~340 Hz for 0.25 m but asserts smoothness); `engine.rs:10584` (longer runners favour low rpm) | f = c/4L not asserted | ❌ / 🟡 |
| Throttle flow, plenum, mass balance | `intake_fv.rs` | `intake_fv.rs:1024`, `:1117`, `:1130`, `:1151`, `:960` | conservation | 🟡 |
| Intake mouth radiation above ka = 2 | `intake_fv.rs`/`listener.rs` | `listener.rs:699` | ratio 4.32 from the code's own one-pole H(s) (see §9) | 🟡 |
| Volumetric efficiency vs rpm (Heywood Fig. 6-?/§6.3) | `engine.rs` | `engine.rs:10606` (VE "stays physical"), `:4912` | qualitative | 🟡 |

## 6. Noise sources

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Jet mixing noise level (SAE ARP876 / Stone; Lighthill V⁸; Hoch density exponent) | `jet_noise.rs` | `jet_noise.rs:476`, `:488`, `:498`, `:539`, `:518` | 56.28 dB at V = 100 m/s, D = 50 mm, R = 7.5 m; +22.58 dB per doubling; +3.056 dB hot slow jet `[hand]` of the cited correlation (not a measured data point) | ✅ (impl) |
| Valve jet level and choked-throat state | `jet_noise.rs` | `jet_noise.rs:378`, `:401` | M = 1.0541; 110.227 dB; 437.5 Pa `[hand]` | ✅ (impl) |
| Tam fine-scale spectrum (Tam, Golebiowski & Seiner 1996 eq. 6) | `jet_noise.rs` | `jet_noise.rs:419` | −5.57 / −2.25 / −11.8 / −26.6 dB at 0.1, 3, 10, 30 f_F `[closed]` | ✅ |
| Strouhal peak 0.2·V/D band-pass | `jet_noise.rs` | `jet_noise.rs:550`, `:583`, `:598` | 0.6897 an octave off `[closed]`; St = 0.2 is a model choice | ✅ (filter) |
| Duct flow noise W = K ρ A U⁴/c (Davies & Holland; Nelson–Morfey style) | `jet_noise.rs` | `jet_noise.rs:630`, `:640`, `:647`, `:685` | 1.5e-3 W; cut-on 1.84c/(πD) = 5857 Hz; 12.247 Pa `[hand]`; K itself is a fitted constant | ✅ (impl) / 🟡 for K |
| Noise generator (xorshift64*) | `jet_noise.rs` | `jet_noise.rs:567` | first draws from Vigna's algorithm run independently `[ind]` | ✅ |
| Combustion noise: Anderton (SAE 790270) | `structure.rs` | `structure.rs:307`, `:377` | 84.3 dB(A) at B = 86 mm, 3000 rpm (eq. 9, Fig. 13–14) `[fig]`; 459 449.1 `[hand]` | ✅ |
| Structure response function (nine engines, Hidraulica 3/2021 Fig. 1) | `structure.rs` | `structure.rs:334`, `:259`, `:275`, `:364` | digitised median, within 5.5 dB `[fig]` | ✅ (digitised, ±5 dB) |
| Mechanical noise level (Raff & Perry 1973; Lalor et al. SAE 800402) | `mechanical.rs` | `mechanical.rs:201`, `:221`, `:234` | 75 / 90 / 105 dB(A) at 1000 / 3000 / 6000 rpm (read off Fig. 7, ±1 dB); 2.5/5/4 dB load deltas `[fig]` | ✅ |
| Mechanical-noise impact trains reproduce the curve | `engine.rs` | `engine.rs:9953`, `:9999`, `:10032` | within 1 dB of the same curve | 🟡 (consistency with the input curve) |
| A-weighting, dB↔Pa | `mechanical.rs` | `mechanical.rs:263`, `:250` | −19.1 dB at 100 Hz (IEC 61672-1 Table 3); 94 dB = 1.0024 Pa `[table]` | ✅ |
| Compressor whine at blade passing | `turbo.rs` | `turbo.rs:436` | 25 mPa rms "the reference" at 440 m/s tip speed (no source), 2^−2.75 scaling | 🟡 |
| Blow-off valve jet, valve tick scaling, texture sources | `engine.rs` | `engine.rs:5210`, `:5962`, `:8534`, `:10481` | hand for jet (choked nozzle); qualitative for the rest | 🟡 |
| Engine bay insertion loss (Vér & Beranek ch. 12; Delany–Bazley) | `engine_bay.rs` | `engine_bay.rs:137`, `:148`, `:162` | α = 0.0227 / 0.3240 / 0.9503; 14.19 dB `[hand]` | ✅ (impl) |

## 7. Mechanics and friction

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Rod inertia force, side force | `mechanical.rs` | `mechanical.rs:284`, `:297`, `:311` | −2201 N at TDC (m ω² r (1+λ)) `[closed]`; side-force sign only 🟡; piston mass 600 kg/m³·B³ is an estimate | ✅ / 🟡 |
| Friction by components (Sandoval & Heywood 2003; Patton–Nitschke–Heywood) | `friction.rs` | `friction.rs:250`, `:286`, `:301`, `:323`, `:358`, `:373` | Ford 5.4 at 2500 rpm WOT: crank ≈ 5, piston ≈ 38, gas ≈ 14, valvetrain ≈ 9, aux ≈ 19 kPa (Sandoval Fig. 3-3) `[fig]`; component terms by hand from App. A-2 `[hand]` (coefficients typed twice) | ✅ |
| Crank dynamics J·dω/dt = T_gas − T_fric − T_load, flywheel, starter | `engine.rs` | `engine.rs:7059`, `:6959`, `:7093` | qualitative (revs, stalls, idles) | 🟡 |
| Torque/power vs published (LFA 136 Nm/L etc.) | `engine.rs` | `engine.rs:6993`, `:7878`, `:8319` | `15–80 N·m` band for a 500 cc single; presets match published specs by assertion of inputs | 🟡 |
| Turbo compressor ratio from tip speed (Euler work with slip) | `turbo.rs` | `turbo.rs:428`, `:462`, `:472` | 1.366767 → 2.984905; 106976.2 J/kg; 8598.91 W `[closed]` | ✅ |
| Turbine as nozzle (swallowing capacity) | `turbo.rs` | `turbo.rs:478`, `:614` | 0.251110 / 0.181306 kg/s `[hand]` | ✅ |
| Inducer choke (Dixon & Hall ch. 7) | `turbo.rs` | `turbo.rs:636`, `:645` | 0.270387 kg/s `[hand]` | ✅ |
| Compressor map (efficiency islands, surge) | `turbo.rs` (no map, efficiency constant) | none | – | ❌ |
| Shaft inertia/lag, wastegate, charge-air volume | `turbo.rs` | `turbo.rs:541`, `:586`, `:667` | τ = C/G = 2.902e-4 s `[hand]`; others qualitative | ✅ (one) / 🟡 |

## 8. Drivetrain

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Road load ½ρ·CdA·v² + Crr·m·g + brake (Guzzella & Onder eq. 2.1–2.3) | `drivetrain.rs` | `drivetrain.rs:637` | 242.1 N, 4742.1 N `[hand]` | ✅ |
| Gear ratios, top speed | `drivetrain.rs` | `drivetrain.rs:648`, `:846`, `:967` | 323.6 km/h vs 325 published (Lexus) `[meas]` | ✅ |
| Clutch Coulomb capacity, lock/slip hysteresis | `drivetrain.rs` | `drivetrain.rs:553`–`:855` | capacity ×engagement²; ¼ at half engagement `[hand]`; structural otherwise | 🟡 |
| Reflected inertia, locked car speed | `drivetrain.rs` | `drivetrain.rs:680`, `:701`, `:834` | (m + J/r²)r²/G² `[closed]` | ✅ (impl) |
| Driveline shuffle (torsional spring-damper) | `drivetrain.rs` | `drivetrain.rs:1021`, `:1035`, `:1049`, `:1069`, `:1130` | f = (1/2π)√(k/J) by hand; torque/k twist; energy kept `[closed]` | ✅ |

## 9. DSP and listener (`audio:dsp.rs`, `listener.rs`, `sound.rs`)

| Model | Source in code | Test (file:line) | Reference value | Status |
| --- | --- | --- | --- | --- |
| Monopole radiation p = ρ0/(4πr)·dQ/dt (Kinsler §7.2) | `listener.rs` | `listener.rs:120`, `:132`, `:489`, `:167`, `:180` | 0.031937 Pa at 3 m; 3.19366 Pa; 1/7 Pa at 7 m `[closed]` | ✅ |
| Propagation delay and 1/r | `listener.rs` | `listener.rs:434`, `:454` | 1120 steps; 140 steps at 96 kHz `[hand]` | ✅ |
| Ground reflection (image source, ISO 9613-2) | `listener.rs` | `listener.rs:620` | 150 steps, ratio 0.92354 `[hand]` | ✅ |
| Street façades, ρ = 0.8 (ISO 9613-2 Table 4) | `listener.rs` | `listener.rs:512` | 1.6496 total, delays 21.7 / 49.2 ms `[hand]` | ✅ |
| Air absorption (ISO 9613-1) | `listener.rs` | `listener.rs:540` | 0.0776 dB/m at 8 kHz, 20 °C, 70 % RH → 3 dB at 38.66 m; one frequency only | ✅ (one point; full α(f) curve ❌) |
| Open-pipe radiation filter above ka = 2 | `listener.rs` | `listener.rs:673`, `:699` | ratio 4.32 computed from the same H(s) = s/(1+s/ω_c), ω_c = 2c/a that the code implements; the model (Levine–Schwinger |R|, flanged/unflanged directivity) is not checked | 🟡 |
| Tail directivity | `listener.rs` | none (all tails are omnidirectional monopoles) | e.g. unflanged-pipe directivity (Levine & Schwinger) | ❌ |
| Low-pass and Butterworth | `audio:dsp.rs` | `audio:dsp.rs:491`, `:511` | −3 dB at cutoff; |H(40 kHz)| ≥ 0.0624 (analog 4th-order Butterworth) `[closed]` | ✅ |
| DC blocker | `audio:dsp.rs` | `audio:dsp.rs:479` | residual < 1e-3; > 0.98 at 50 Hz; −3 dB corner `audio:dsp.rs:761` (1/√2 within 1 %, fc = 3 and 100 Hz) ✅; RBJ low-pass gain at cutoff = Q `audio:dsp.rs:774` ✅ | ✅ corners; reverb RT60 and leveler timing still 🟡 |
| Voice calibration 0 dBFS = 120 dB SPL | `audio:dsp.rs` | `audio:dsp.rs:615`, `:642` | 2 Pa → 0.1 of full scale `[hand]` | ✅ |
| Reverb RT60 | `audio:dsp.rs` | `audio:dsp.rs:535`, `:554` | energy ratio < 1e-4 (generous; −60 dB after 0.3 s would be 1e-6) | 🟡 |
| Look-ahead limiter, leveler | `audio:dsp.rs` | `audio:dsp.rs:669`, `:682`, `:709`, `:726`; `:430`, `:455`, `:470` | THD < −60 dB; ratio in dB halves with amount (design properties) | 🟡 |
| Silencing/NaN safety | `audio:dsp.rs` | `audio:dsp.rs:632`, `:737` | bounded | 🟡 |
| `engine-audio/src/device.rs` (cpal) | – | none (needs hardware; covered by `scripts/smoke.sh`) | – | ❌ (by design) |

---

## Counts

Counted per row of the tables above (a row with "✅ / 🟡" counts as the first symbol; "✅ (impl)"
rows count as ✅).

| Status | Rows |
| --- | --- |
| ✅ external | 98 |
| 🟡 self-consistent only | 35 (+4 mixed rows counted by first symbol or listed under other) |
| ❌ untested | 7 |

(Counts are of model groups, not of tests; there are ≈ 560 tests in the two crates.)

## Suspicious "golden" numbers

Tests whose expected value is not derived from outside the code:

1. `exhaust.rs:2360` `duct_flow_off_is_the_old_net` — "captured on the code before the duct flow noise
   (commit 2a93c48)". A regression golden: it freezes behaviour, not physics. Fine as a
   refactor guard; it validates nothing.
2. `exhaust_fv.rs:940` `a_junction_at_rest_gives_its_own_pressure` — "captured from a run" (the
   failing state of a root finder). The expected value (rest pressure) is physical; only the
   input was captured. Acceptable.
3. `exhaust_fv.rs:873` `in_the_linear_regime_the_fv_net_sounds_like_the_waveguide` — expectations are
   the measured gap between two of the project's own models (+4.9, +5.9, −3.4 dB), not a reference.
4. `exhaust.rs:3618` `a_loud_periodic_drive_gives_a_periodic_exhaust` — thresholds (0.15 %, 0.34 %)
   "both measured 2026-10-26" from the code's output; a stability guard, not physics.
5. `turbo.rs:436` — "25 mPa rms at 440 m/s tip speed: the reference" has no citation.
6. `combustion.rs:249` and `engine.rs:9953`/`:9999`/`:10032` — the "tenth of the premixed step"
   factor and the impact trains compare the code with its own input curve (consistency, not
   validation).
7. `narrow_tube.rs:126` — "exact" F(z) is the test module's own `f_zk` (not production). Now checked at real z = 1, 2 against Bessel tables (`narrow_tube.rs:152`); the complex arguments the test uses are still unchecked.
8. Loose bands that act as plausibility only: `engine.rs:6974` (8–26 bar), `engine.rs:6993`
   (15–80 N·m), `engine.rs:7162` (35–40 %), `engine.rs:6364` (15–35 %), `engine.rs:10153` (5–15 dB(A)),
   `audio:dsp.rs:535` (1e-4).

No test was found whose expected number is clearly just pasted from the code's own output with no
comment; the style of the code base (hand values in comments) makes that unlikely.

---

## Gaps, ranked (by effect on sound)

Order: pulse generation (cylinder, valves) → pipe wave propagation → junctions → silencers and
chambers → open-end radiation → listener → noise sources. "API" says whether the benchmark is
reachable with the existing public API (no new production code).

1. **Cylinder pressure trace / first law (❌/🟡).** The source of every pulse. Only a loose band
   (`engine.rs:6974`). Benchmarks: (a) motored compression p·V^n with the thermo-model γ(T): for
   adiabatic closed cycle with no heat transfer (Woschni switched off, if the constants allow it)
   p/p1 = (V1/V)^γ̄ and the temperature rise T2/T1 = (V1/V2)^(γ̄−1) with the mean γ̄ taken from
   Heywood Fig. 4.13; (b) constant-volume burn: Δp = (γ−1)Q/V at one step (closed form, Heywood ch. 5)
   against `combustion` + cylinder; (c) fuel-air Otto cycle peak pressure and temperature from
   Heywood Fig. 5.9 / Table 4.7 (equilibrium states are in Ferguson & Kirkpatrick, *Internal
   Combustion Engines*, ch. 3 charts). API: `Engine::cylinder(i).pressure_pa` + `Controls` exist;
   a disabled-friction/heat-loss option may be needed (not checked).
2. **Valve flow coefficient curve (🟡: two points).** Heywood Fig. 6-18 gives C_D(L/D) for intake
   valves (≈ 0.6–0.65 near L/D 0.25–0.3, rising to ≈ 0.45 effective area ratio...) and flow
   coefficients of Blair's maps for reverse flow. Benchmark: tabulate C_D·A_curtain at 5–6 values
   of L/D from the figure and test `flow_area`; add the reverse-flow ratio from Blair's
   *Design and Simulation of Four-Stroke Engines* (ch. 6). API: `ValveTiming::flow_area`.
3. **Benson/Jenny valve boundary at finite ψ (🟡, partly done).** ψ ≥ 1 limit is now tested
   (`valve_boundary.rs:960`, closed form). **Correction to the earlier text:** R = (1−ψ)/(1+ψ) does
   *not* apply: a valve orifice at zero mean flow has no linear limit (u = √(2Δp/ρ), a square
   root), so the small-amplitude reflection depends on amplitude; the area-step rule is covered
   by the junction test (`waveguide.rs:2395`). Remaining: reflected amplitude at finite ψ < 1 and
   finite pulse from Benson vol. 1 ch. 4 or Blair worked examples (a published pulse figure).
4. **Toro's Riemann tests 2–5 (✅ 3, 4, 5; Test 2 finding).** Done: `fv.rs:801`–`:855`. Initial data
   confirmed against Tasker et al. 2008 (arXiv:0711.4141 §2.1–2.5) and Cholla's Test 4 page; star
   values: Test 3 (460.894, 19.5975) against Toro et al. (arXiv:2412.03522 Table 1), Tests 2–5
   reproduced by an independent bisection solver. **Corrections to the earlier text:** the inventory
   had Test 5 as p* = 46.0950, u* = 8.68975; the right values are p* = 460.894, u* ≈ 0 (Test 5 is Test 3
   with a −19.59745 background velocity, so its contact stays at x0 = 0.8). Test 4 (collision) has
   p* = 1691.64, u* = 8.68975; its initial data (5.99924, 19.5975, 460.894 | 5.99242, −6.19633,
   46.0950) are the earlier "Test 5 p*" numbers. The Table 4.2 values of Tests 2 and 4 were not found in a
   source text, only reproduced by the independent solver. **Finding:** Test 2 (123 problem): star
   pressure at x = 0.5 is 0.002190 vs exact 0.00189 (+16 %; 0.0022–0.0024 across the whole
   plateau, +18 to +27 %), N = 1000 — outside the 10 % set beforehand; the scheme overestimates the
   near-vacuum pressure. Test boundary note: walls at the ends are wrong for Tests 4–5 (gas moves
   away from them and a vacuum forms); the helper pins both ends to the initial states.
5. **Open-end reflection vs ka (Levine & Schwinger) (✅ ka ≤ 1; findings above).** Reference
   corrected: the Norris–Sheng coefficients in the earlier text were wrong (from memory). Used instead:
   Silva, Guillemain, Kergomard, Mallaroni & Norris, J. Sound Vib. 322 (2009) 255–263
   (arXiv:0811.3625) Eq. 21, Table 1: |R| = (1 + 0.800 x)/(1 + 1.300 x + 0.266 x² + 0.0263 x³),
   x = (ka)², error < 2 % for ka < 3 vs Levine–Schwinger; l/a → 0.6133. Measured (steady sine,
   48 kHz, a = 21 mm): ka = 0.25, 0.5, 1 within 2 % (the code's one-pole matches 1 − (ka)²/2
   at low ka), ka = 2: 0.447 vs 0.346; end correction 0.419 a vs 0.6133 a. **Fixed 2026-11-08**
   (`fix/open-end-levine-schwinger`): a two-pole end whose low-ka delay is the end correction and
   whose |H|² = 1 − (ka)² at low ka, with no extra 0.98 factor (unsourced since the first
   waveguide commit): |R| within 0.3 % of the fit to ka = 1 and l = 0.6133 a; ka = 2 is 0.392
   (+13 %). No simple stable rational filter met both the delay and the modulus over ka 0–3
   (fits of orders 2–4 searched). Mach-number dependence is covered by the existing (1−M)/(1+M) tests.
6. **Expansion chamber TL with mean flow (❌).** The waveguide pipes do not convect (they apply
   loss only), so the Mach-number shift of chamber resonances and the extra damping are missing,
   and the FV net handles convection implicitly with numerical damping. Benchmark: Munjal ch. 3
   with the convective wave numbers k± = k/(1 ± M), TL = 20 log |cos kL + j ½(m + 1/m) sin kL| with
   the Mach terms of the full 4-pole (Munjal §3.3). API: FV net at M = 0.1–0.2 via
   `exhaust_fv` or `waveguide::Duct::set_mean_flow` for the loss part only.
7. **T-junction / collector pressure losses and wave splitting under flow (🟡).** `exhaust_fv`
   0D junction and `lossy_junction` are tested by identities and one Borda–Carnot case. Benchmarks:
   Winterbone & Pearson, *Theory of Engine Manifold Design* (loss coefficients of three-pipe
   junctions as functions of flow ratio and angle), Miller, *Internal Flow Systems* (Fig. 13.x:
   K for tees and wyes), and the lossless acoustic limit already done. Also the three-branch
   Kelly–Lochbaum at unequal areas under pure acoustic excitation vs a transfer-matrix solution
   (closed form). API: `waveguide::junction` / `lossy_junction`.
8. **Friction factor and viscothermal loss values (❌/🟡).** Pipe loss uses f = 0.025 and α given in
   Np/m, never checked against the Kirchhoff (1868) wall attenuation α = (1/(r c))·√(ωμ/2ρ)·(1 +
   (γ−1)/√Pr) (Kinsler, section unconfirmed; code cites §9.9) nor against Moody/Colebrook f(Re, ε/D) or Blasius 0.316 Re^−¼.
   Benchmark: α at 1 kHz for a 21 mm-radius pipe in 600 K gas (**done**, `waveguide.rs:2347`: 0.0576 Np/m, code within 2 %); the section in Kinsler is not confirmed (the code comment says §9.9, this file said §8.9; the formula is confirmed from Tiikoja et al. SAE 2016-01-1816 Eq. 6); f at Re 1.5e4 (Blasius 0.0285) still open. API:
   `Pipe::new(.., loss_per_m)`, `Pipe::viscothermal`, constants in `exhaust_fv`.
9. **Cylinder heat transfer vs measurement (🟡).** Woschni is checked formula-to-hand only; the
   port wall HTC (200 W/m²K) has no test. Benchmarks: Woschni's own motored-engine heat flux at
   TDC (SAE 670931, Fig. 1) or Heywood Fig. 12-?; the integral of heat loss over a motored cycle
   vs the polytropic exponent n ≈ 1.32–1.35 (Heywood §12.4: compression n ≈ 1.3–1.35) — a direct
   test of the combined heat-transfer plus γ(T) model. API: as gap 1.
10. **Open-pipe / flanged radiation model and directivity (🟡/❌).** The 4.32 ratio checks the
    implementation of a one-pole stand-in. Benchmark: radiation impedance of an unflanged pipe
    (Levine & Schwinger: Re Z = ρc/(πa²)·(ka)²/2 at small ka — the low-ka monopole limit,
    exact), and a flanged-piston radiation resistance R1(2ka) = 1 − 2J1(2ka)/(2ka) (Kinsler §7.5,
    Rayleigh), both closed forms. API: `Listener::radiate` / `radiate_mouth` with swept tones.
11. **Compressor/turbine maps (❌).** Whine and lag depend on the map; there is none (constant
    efficiency). Benchmark when one exists: a published map (e.g. Garrett GT2860RS) read at three
    points; for now the Euler/nozzle laws are verified. Also whine level (25 mPa unsourced).
12. **Cycle-to-cycle and texture noise statistics (🟡).** Heywood Fig. 9-35 spreads are within loose
    bands; benchmark with the COV_imep ≈ 2–5 % at part load, < 2 % at WOT (Heywood §9.4.2), as a
    two-sided test on the ensemble. API: `Engine` runs.
13. **Zwikker–Kosten function F(z) (🟡, real axis done).** Real-axis checks at z = 1, 2 done (`narrow_tube.rs:152`, tables from Schaum's Appendix B Table B-1 / A&S 9.1 at 3 decimals; J1(5) not confirmed so z = 5 dropped). Still open: complex z. Test F(z) = 2J1(z)/(z J0(z)) at z = 1+i, 5+5i, 10 with
    scipy values (e.g. the real-argument check F(1) = 2·0.44005/(1·0.76520) = 1.1501), so the
    Foster-fit test is no longer against an unverified in-crate Bessel. API: `narrow_tube::f_zk`
    (private? check `pub(crate)`).
14. **Air absorption curve and ground effect (🟡).** Full ISO 9613-1 α(f, T, RH) table (Table 1 of
    the standard) rather than one point; ISO 9613-2 ground attenuation is replaced by a rigid
    image — not tested vs. a soft ground (acceptable for asphalt).
15. **DSP corner frequencies (🟡, corners done).** DC blocker −3 dB corner and RBJ low-pass gain at cutoff done (`audio:dsp.rs:761`, `:774`). Open: reverb RT60 within ±10 % (Schroeder
    1962: RT60 = 0.161 V/A, or the comb-feedback formula g = 10^(−3·T_loop/RT60) used to size it),
    leveler/limiter timing constants (design values; exact attack/release −1/e points). API:
    `audio:dsp.rs` public blocks.

### Quick wins (all reachable with the existing public API, no new production code)

Gaps 4 (Toro 2–5, if `Pipe1d` can set the initial states as the `sod()` helper does), 5 (swept
sine through `OpenEnd`), 8 (Kirchhoff α), 13, 15, and the ψ = 1 / small-amplitude reflection
limit of gap 3. Gaps 1 and 9 may need a way to switch off friction/heat loss or to read the
in-cylinder pressure/temperature history (the engine exposes `cylinder(i).pressure_pa`; its
temperature accessor and a heat-loss switch were not checked).

### Notes on what I could not verify

- Toro Table 4.2 values for Tests 2 and 4 were not found in a readable source (the book was not
  reachable); they are reproduced by an independent exact solver and agree with the recalled values.
  Tests 3 and 5 are corroborated by arXiv:2412.03522 Table 1 (mirrored) and the Galilean shift.
- Kinsler's section for Kirchhoff attenuation (the code says §9.9) and the book's own Kirchhoff
  coefficient were not read; the formula was confirmed from Tiikoja et al. (SAE 2016-01-1816).
- Silva et al. coefficients were read from the arXiv HTML (ar5iv) via a summarizing fetch; the
  low-ka limits |R| ≈ 1 − (ka)²/2 and η = 0.6133 are consistent with the classical Levine–Schwinger results.
- I did not read all ~150 scenario tests of `engine.rs`; none of them is a physics benchmark by
  name, but a few (e.g. `engine.rs:7109` `combustion_peaks_after_tdc_and_is_visible`, `:7201`
  VTEC overlap, `:9769` FV vs waveguide torque) may carry an external number I did not see.
- Mutation-score and coverage figures were not consulted: ✅ here means "an external value is
  asserted", not "the assertion would catch a regression" (sabotage each ✅ once before relying on it).
