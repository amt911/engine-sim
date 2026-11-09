#!/usr/bin/env python3
"""Catch a muffled build before anyone listens to it: how bright it is against real recordings of
the same engine, at every rpm the recordings cover. No point is summed into a total, so a better
bass cannot hide a lost firing tone.

Why (user, 2026-11-09, on a build realgate had passed): "muy tapado, muchos armónicos tapados".
realgate scores 7500-9000 rpm pulls and sums its distances; the drive scene the user heard had its
500-1000 Hz firing tone 12 dB down. And the reference is the real engine, never one of our builds
(user, 2026-11-09: "no con versiones creadas por nosotros, en todo caso por reales").

Per real recording, each tracked instant (its .trk line, with a firing-line prominence of at least
PROM_DB) gives one NREV-revolution window, resampled to crank angle along the tracked rpm:
- bright: energy 500 Hz-4 kHz over 150-500 Hz, dB (below 150 Hz a recording carries wind);
- centroid: the 150 Hz-8 kHz spectral centroid, log-frequency weighted, Hz;
- harmonics: how many firing orders F..10F (F = cylinders / 2) stand within 20 dB of the strongest;
- treble: energy 2-8 kHz over 150-500 Hz, dB (a muffled top end hides in "bright" behind 1-2 kHz);
- firing: the firing order's own level over the 150 Hz-8 kHz total, dB (the tone the ear follows;
  a build whose firing tone sank 10 dB at 8000-8500 rpm still passed the other four, 2026-11-09).
Single firing harmonics are not judged: each swings +-20 dB from one 500 rpm bin to the next with
the pipes' resonances, in the approved build too (measured 2026-11-09).
- level: the total 150 Hz-8 kHz level re the source's own level at LEVEL_REF_RPM, dB (the real
  car's level stays within ±1.2 dB from 6500 to 9000 rpm; a build whose hiss grew 10 dB there
  sounded "fatal" and "tapado", 2026-11-09). Judged both ways: louder than every real recording
  fails too.
Medians per recording per rpm bin (BIN_RPM wide, at least MIN_WINDOWS windows); the recordings'
lowest and highest medians make the real envelope (they differ in microphone and distance), used
where two recordings cover the bin, or one past the last bin two cover (tolerance doubled). Use full-throttle recordings from outside
the car: in-car films and decelerations are 10-20 dB duller (2026-11-09, LFA). A build is
rendered held at full throttle at each bin's centre and FAILs where a measure falls more than TOL
below the envelope's floor (muffled); above its ceiling is reported as a warning.
Validated on the user's ear verdicts (LFA, 2026-11-09), frozen since: the approved main build
PASSES (margins: level 1.8 dB at 8000 rpm, 2 dB at 9000; centroid only 8 Hz at 9000); the
PR #127 build heard "muy tapado" FAILS (6500-7500 rpm dull, level 3.8-6.7 dB over); the build
whose gears clip "suena fatal" FAILS (level 4-10.6 dB over at 5500-9000). Change it only by
re-running that matrix and showing it still separates them.

Usage:
  scripts/harmgate.py REFS.json LABEL=BIN:ENGINE.yaml [LABEL=BIN:ENGINE.yaml ...]
REFS.json (local, recordings are never committed):
  {"firing_order": 5, "recordings": [{"wav": "a.wav", "trk": "a.trk"}, ...]}
.trk lines: "<t>s <rpm>rpm prom <dB>dB rms <dB>" (the tracker in scripts/realism).
Exit 0 when every build passes.
"""
import importlib.util
import json
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("realgate", os.path.join(HERE, "realgate.py"))
rg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rg)

PROM_DB, BIN_RPM, MIN_WINDOWS = 6.0, 500, 3
TOL = [3.0, 0.0, 1.0, 3.0, 3.0, 3.0]  # beyond the real envelope: dB, Hz (the envelope is wide already), harmonics
LEVEL_REF_RPM = 6500  # the level is judged against each source's own level here
TWO_SIDED = {5}  # level: louder than every real recording fails too
MAX_RPM = 9000  # a held render above it sits on the LFA's limiter
NAMES = ["bright dB", "centroid Hz", "harmonics", "treble dB", "firing dB", "level dB"]


def measure_window(X, rpm, firing):
    """[brightness: 500 Hz-4 kHz over 150-500 Hz, dB; spectral centroid over 150 Hz-8 kHz, Hz
    (log-frequency weighted); harmonics: how many of the firing orders F..10F stand within 20 dB
    of the strongest]. Below 150 Hz a recording carries wind and road rumble."""
    hz = np.arange(len(X)) / rg.NREV * rpm / 60
    k = (hz >= 150) & (hz < 8000)
    lo, hi = X[(hz >= 150) & (hz < 500)].sum(), X[(hz >= 500) & (hz < 4000)].sum()
    cen = 2 ** (np.sum(X[k] * np.log2(hz[k])) / X[k].sum())
    o = np.array([X[m * rg.NREV - 1:m * rg.NREV + 2].sum() for m in range(firing, 10 * firing + 1, firing)])
    rich = float((10 * np.log10(o / o.max() + 1e-30) > -20).sum())
    tre = X[(hz >= 2000) & (hz < 8000)].sum()
    return np.array([10 * np.log10(hi / lo + 1e-30), cen, rich, 10 * np.log10(tre / lo + 1e-30),
                     10 * np.log10(o[0] / X[k].sum() + 1e-30), 10 * np.log10(X[k].sum() + 1e-30)])


def real_bins(rec, firing):
    """{bin centre rpm: median measures} of one recording."""
    fs, x = rg.auth.load(rec["wav"])
    if x.ndim > 1:
        x = x.mean(axis=1)
    rows = [line.split() for line in open(rec["trk"]) if line.strip()]
    T = np.array([float(r[0].rstrip("s")) for r in rows])
    R = np.array([float(r[1].rstrip("rpm")) for r in rows])
    P = np.array([float(r[3].rstrip("dB")) if r[3] != "nandB" else -99.0 for r in rows])
    good = P >= PROM_DB
    got = {}
    for t, rpm in zip(T[good], R[good]):
        span = rg.NREV * 60 / rpm  # s of NREV revolutions
        a, b = t - span / 2, t + span / 2
        k = good & (T >= a - 0.25) & (T <= b + 0.25)
        if a < 0 or int(b * fs) + 1 >= len(x) or k.sum() < 2:
            continue
        seg = x[int(a * fs):int(b * fs) + int(0.05 * fs)]
        y = rg.to_angle(seg, fs, a, lambda tt: np.interp(tt, T[k], R[k]))
        n = rg.NREV * rg.SPR
        if len(y) < n:
            continue
        X = np.abs(np.fft.rfft(y[:n] * np.hanning(n))) ** 2
        c = int(round(rpm / BIN_RPM)) * BIN_RPM
        got.setdefault(c, []).append(measure_window(X, rpm, firing))
    return relative_level({c: np.median(v, axis=0) for c, v in got.items() if len(v) >= MIN_WINDOWS})


def relative_level(bins):
    """The level measure re the source's own level at LEVEL_REF_RPM (NaN when it has none)."""
    ref = bins[LEVEL_REF_RPM][5] if LEVEL_REF_RPM in bins else np.nan
    for v in bins.values():
        v[5] -= ref
    return bins


def build_bins(binary, engine, rpms, firing):
    got = {}
    for rpm in rpms:
        fs, x = rg.render(binary, engine, rpm)
        if x.ndim > 1:
            x = x.mean(axis=1)
        y = rg.to_angle(x, fs, 0, lambda t: np.full_like(t, rpm))
        got[rpm] = np.median([measure_window(X, rpm, firing) for X in rg.windows(y)], axis=0)
    return relative_level(got)


def envelope(refs):
    per = [real_bins(r, refs["firing_order"]) for r in refs["recordings"]]
    bins = sorted({c for p in per for c in p})
    return {c: (np.nanmin([p[c] for p in per if c in p], axis=0),
                np.nanmax([p[c] for p in per if c in p], axis=0),
                sum(c in p for p in per)) for c in bins}


def main(argv):
    refs = json.load(open(argv[0]))
    env = envelope(refs)
    # one recording sets no envelope on its own, except past the last bin two recordings cover:
    # the pulls end there and the top is where a build goes wrong (2026-11-09)
    top2 = max(c for c in env if env[c][2] >= 2)
    rpms = [c for c in sorted(env) if c <= MAX_RPM and (env[c][2] >= 2 or c > top2)]
    print("real envelope (dB re total, floor..ceiling, recordings):")
    print(f"{'rpm':>5} " + " ".join(f"{n:>15}" for n in NAMES))
    for c in rpms:
        lo, hi, n = env[c]
        print(f"{c:5} " + " ".join(f"{a:7.1f}..{b:6.1f}" for a, b in zip(lo, hi)) + f"  ({n})")
    ok = True
    for arg in argv[1:]:
        label, rest = arg.split("=", 1)
        binary, engine = rest.split(":", 1)
        m = build_bins(binary, engine, sorted(set(rpms) | {LEVEL_REF_RPM}), refs["firing_order"])
        fails, warns = [], []
        print(f"\n{label}:")
        for c in rpms:
            lo, hi, n = env[c]
            print(f"{c:5} " + " ".join(f"{v:15.1f}" for v in m[c]))
            for i, name in enumerate(NAMES):
                tol = TOL[i] * (2 if n < 2 else 1)
                if np.isnan(lo[i]):
                    continue
                if m[c][i] < lo[i] - tol:
                    fails.append(f"{c} rpm: {name} {m[c][i]:.1f} under the real floor {lo[i]:.1f}")
                elif m[c][i] > hi[i] + tol and i in TWO_SIDED:
                    fails.append(f"{c} rpm: {name} {m[c][i]:.1f} over the real ceiling {hi[i]:.1f}")
                elif m[c][i] > hi[i] + tol:
                    warns.append(f"{c} rpm: {name} {m[c][i]:.1f} over the real ceiling {hi[i]:.1f}")
        ok &= not fails
        print(f"{label}: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
        for f in fails:
            print("  " + f)
        for w in warns:
            print("  warn (brighter than every real): " + w)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
