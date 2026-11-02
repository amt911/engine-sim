# Turbocharger — design

Status: written unattended (roadmap delegated; user 2026-10-31: "sigue el roadmap"; turbo is
after NA diesel and rotary, and the user's 1.9 TDI needs it). Decisions marked *(my call)* are
open to the user's review.

## Why

Most modern engines are turbocharged, and the user wants the VW 1.9 TDI's sound. A turbo
changes the sound three ways: the turbine takes energy out of each exhaust pulse (a turbo engine
is quieter and duller at the tail), the compressor raises the intake's density (more torque, a
different intake note), and it adds its own sounds — the compressor's whine and the blow-off
valve's rush when the pedal lifts under boost.

## Physics (cited)

- **Compressor** (Heywood, *ICE Fundamentals* 2nd ed., §6.8, eq. 6.44): power
  P_c = ṁ_c·c_p·T₀₁·[(p₀₂/p₀₁)^((γ−1)/γ) − 1]/η_c; exit temperature from the same.
- **Its pressure ratio from its speed**, not a maker's map: Euler's turbomachine equation for a
  radial compressor with backswept blades, work = σ·U² (U the tip speed, σ the slip factor,
  Stanitz/Wiesner ≈ 0.85–0.9; Dixon & Hall, *Fluid Mechanics and Thermodynamics of
  Turbomachinery*, ch. 7), so p₀₂/p₀₁ = [1 + η_c·σ·U²/(c_p·T₀₁)]^(γ/(γ−1)), falling with flow
  past the design point (*my call*: a linear backswept work coefficient). Surge is not modelled.
- **Turbine** (Heywood eq. 6.50): power P_t = ṁ_t·c_p·T₀₃·η_t·[1 − (p₄/p₀₃)^((γ−1)/γ)]; its flow
  that of a nozzle of effective area A_T (the "swallowing capacity", Heywood §6.8.2), so the
  exhaust manifold's pressure rises with flow — the turbine is a restriction the pulses meet.
- **Shaft** (Heywood eq. 6.70 in steady state): I·ω·dω/dt = η_m·P_t − P_c — spool-up lag.
- **Wastegate**: an orifice from the turbine inlet to its outlet opened in proportion to boost
  above its set pressure (a spring-loaded actuator). **Intercooler**: effectiveness ε,
  T = T₂ − ε(T₂ − T_amb), a pressure drop ∝ ṁ². **Blow-off valve**: vents the compressor exit to
  the atmosphere when the throttle closes under boost — a jet of that flow (the existing jet
  noise model).
- **Whine** *(my call, second step)*: the compressor's blade-passing tone Z·N at a level from the
  tip Mach number; added after the core works, with a level checked against recordings.

## Shape

- `crate::turbo`: pure functions (compressor ratio and power, turbine flow and power) and a
  `Turbo` state (shaft speed, wastegate, BOV), unit-tested against hand calculations.
- `EngineDef::turbo: Option<TurboDef>` (serde default none; round trip tested).
- Engine: the intake draws from the compressor exit (after the intercooler) instead of the
  atmosphere; each collector's flow passes the turbine (its pressure the exhaust back pressure the
  ports see, its outlet the tail). Off: bit-identical.
- Presets: the **VW 1.9 TDI** (ALH, 66 kW, 210 N·m, VNT turbo; the diesel model) and the user's
  approval decides whether a turbo petrol preset follows.

## Done when

- Unit tests of each function (hand values); spool-up lag over a second or so after a step to full
  load; boost limited by the wastegate; serde round trip; non-turbo presets bit-identical.
- TDI: dyno within 15 % of 66 kW / 210 N·m; idle; clips sent to the user.
