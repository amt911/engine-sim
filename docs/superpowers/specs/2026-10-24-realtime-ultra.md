# Ultra in real time on the V10 — spec and plan (2026-10-24)

Approved direction (user, 2026-10-24): both options — SIMD across the intake runners and a second
simulation thread. Physics unchanged: **every step below must leave the output bit-identical**
(headless WAV compared byte for byte), which is also the test that each one is correct.

## Where the time goes (LFA V10, Ultra, `perf`, cycles of user time per audio second)

`main` at v0.15.0: 7.32 Gcyc/s (inline-4 2.37) — no core runs that in real time.

| Part | Share | Notes |
| --- | --- | --- |
| `fv::Pipe1d::step` (10 ITB runners, WENO5-RK3) | 24 % | fixed CFL rate, not the physics step |
| `waveguide::Duct::send` (+ `ExhaustNet::step` 2.4 %) | 19 % | 1 cm packing segments at Ultra |
| `Engine::step` (cylinders, valves, crank) | 15 % | |
| `exhaust_fv::throttled_ghost` | 5 % | the ITB throttles |
| `gas::port_flow_from` | 4 % | |

## 1. No allocation per step — done

`ExhaustNet::step` allocated four vectors every step. Scratch buffers now: 7.05 Gcyc/s (−3.7 %),
inline-4 2.11 (−11 %), bit-identical.

## 2. The exhaust on a second thread, exactly

Within a step the cylinders read, per primary, the wave arriving at the port (`at_near`) and the
port impedance; the net then takes the valve flows and advances. A primary is at least 2 samples
long, so the wave arriving at step n+1 was sent at step ≤ n−1: **the cylinders of step n+1 do not
depend on the net's step n** — except through the friction gain, which a mean-flow update changes at
the start of the net's step. So the worker, at the start of its step n, applies the outflow Mach and
mean flows, publishes the port waves for step n+1, then advances. The main thread runs step n+1's
cylinders while the worker advances step n. The audio of step n (listener, jets) is assembled when
the worker hands step n's tail signals back: one sample of internal lag, the same samples out.

- Core stays thread-free (`AGENTS.md`): `engine-core` exposes the split (cylinders / exhaust job /
  finish) and keeps the sequential `step` as the reference; `engine-app` owns the thread.
- Used when the machine has the cores (≥ 3 available besides UI/audio); otherwise sequential.
- Sync: two SPSC hand-offs per step (`rtrb`, already a dependency), spinning briefly then yielding.
- Guard: a preset whose primaries are shorter than 2 samples (never a real one) runs sequentially.

Ceiling: the net's ~21 % leaves the main thread (≈ 1.27×).

## 3. Half the intake runners on the worker

With individual throttles the runners are independent within an FV step; the worker takes half of
them each FV step (one barrier per FV step, which comes every few physics steps). Ceiling ≈ another
12 %. With a common plenum the runners are coupled each substep: sequential.

## 4. SIMD across equal runners

The runners of a preset are identical pipes: lay their cells out lane by lane (`wide::f64x4`) and
run WENO5 + HLLC on four runners at once, the same operations per lane (no FMA contraction
differences) so the output stays bit-identical. Ceiling ≈ 2–3× on the runners' share.

## Acceptance

- Each step: LFA, inline-4 and V8 headless WAVs (Ultra, 2 s WOT and idle) byte-identical to `main`.
- Cost measured with `cyc.sh` (cycles of user time per audio second, load-independent) and the
  real-time factor under the CPU lock.
- Target: LFA Ultra with ≥ 1.5× real-time headroom on this machine.

## Status (2026-10-24)

- Steps 1–3 done (PR #23).
- Step 4, first part: WENO5-Z across cells, not across runners. Four cells per vector (`wide::f64x4`, SSE2 baseline, no AVX needed). It applies to every 1D pipe, intake and exhaust. LFA Ultra: 12.9 → 10.9 Gcyc of user time per 2 s of audio (−15 %). The WAVs are byte-identical, and a property test pins it, bit for bit.
- Next candidate: HLLC flux across faces (branches become blends), ~18 % of the time left in `Pipe1d::step`.
