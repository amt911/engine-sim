#!/usr/bin/env python3
"""static_gate.py -- impulsive treble ("static", "estatica"): one HF click per exhaust pulse.

Measure: excess kurtosis of the 4-10 kHz band (4th-order Butterworth band-pass), per 100 ms window,
median over the window of a render (one speed bin = one held_*_1.0 WAV; one scene = gears.wav, drive.wav).
Gaussian noise reads 0; a click train (one sharp front per exhaust pulse) reads high.

Bound: real exterior recordings (names ext / passby / pass_by / exhaust; not interior / onboard /
cockpit / intake; sample rate >= 24 kHz; 389 of real_engine_sounds/segmentos, measured 2026-11-15):
median 0.19, p75 0.53, p90 1.29; only straight-pipe bikes and single-cylinder racers read above 5.
BOUND = 2.5: the real p90 (1.29) plus the main build's own worst scene (EJ25, 1.3 at the p90) with
margin, well under the 3.3-7.5 that the reactive-silencer build rsA read and the user heard as
"suena como a estatica" (2026-11-15). Seen failing on rsA, passing on main (m0): see `selftest` and
`gate`.

usage
  static_gate.py gate LABEL [LABEL...]   judge renders in $REALISM_DATA/renders/LABEL/*.wav
  static_gate.py file WAV [WAV...]       the number for any WAV
  static_gate.py selftest                a click train fails, Gaussian noise passes
Needs numpy, scipy, soundfile.
"""
import glob
import os
import sys

import numpy as np
import soundfile as sf
from scipy.signal import butter, sosfilt
from scipy.stats import kurtosis

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rdata  # noqa: E402

BAND = (4000.0, 10000.0)
WINDOW_S = 0.1
BOUND = 2.5
SCENES = ("gears.wav", "drive.wav")
# Presets whose MAIN build (binary engine-sim-m0, 2026-11-15, same score) already reads above BOUND: straight-pipe-like
# or rotary/diesel/single/cross-plane sounds, where real recordings reach 5-22 too. They are judged against main's own
# reading + 25 % (a ratchet: the build must not get more impulsive than main), never against BOUND.
MAIN_READS = {"lfa": 2.7, "mazda-13b-bridgeport": 23.5, "mazda-renesis": 15.5, "single": 9.7,
              "v8-crossplane": 5.4, "vw-tdi": 9.7}


def static_kurtosis(x, sr):
    """Median over 100 ms windows of the 4-10 kHz band's excess kurtosis (nan if the WAV is too short/silent)."""
    y = sosfilt(butter(4, BAND, "band", fs=sr, output="sos"), x)
    n = int(sr * WINDOW_S)
    k = [kurtosis(y[i:i + n]) for i in range(0, len(y) - n + 1, n) if np.std(y[i:i + n]) > 1e-12]
    return float(np.median(k)) if k else float("nan")


def read_mono(path):
    x, sr = sf.read(path)
    return (x if x.ndim == 1 else x.mean(1)), sr


def label_wavs(label):
    d = os.path.join(rdata.DATA, "renders", label)
    held = sorted(glob.glob(os.path.join(d, "held_*_1.0_*.wav")))
    scenes = [os.path.join(d, s) for s in SCENES if os.path.exists(os.path.join(d, s))]
    return scenes + held


def cmd_gate(labels):
    bad = 0
    for l in labels:
        rows = []
        for p in label_wavs(l):
            x, sr = read_mono(p)
            rows.append((os.path.basename(p), static_kurtosis(x, sr)))
        rows = [r for r in rows if np.isfinite(r[1])]
        if not rows:
            print(f"{l} static: NO DATA")
            bad += 1
            continue
        gears = [v for n, v in rows if n == "gears.wav"]
        held = [v for n, v in rows if n.startswith("held_")]
        # the user's scene (gears) and the typical speed bin (median over the WOT bins): a single bin or the
        # drive scene's lift-offs can spike on main too (EJ25 3600 rpm 14.8, K20C1 drive 7.6 on m0)
        score = max(gears + ([float(np.median(held))] if held else []))
        worst = ("gears / median of the WOT bins", score)
        preset = l.rsplit("-", 1)[0]
        bound = max(BOUND, 1.25 * MAIN_READS.get(preset, 0.0))
        ok = score <= bound
        bad += not ok
        print(f"{l} static: {'PASS' if ok else 'FAIL'} {score:.1f} <= {bound:.1f} (gears.wav and the median of {len(held)} WOT bins)"
              + ("" if ok else "  impulsive treble: a click per exhaust pulse, 'suena a estatica'"))
        if "-v" in os.environ.get("STATIC_V", ""):
            for n, v in rows:
                print(f"    {n:28s} {v:6.2f}")
    return 1 if bad else 0


def cmd_selftest():
    sr = 48000
    rng = np.random.default_rng(1)
    noise = rng.standard_normal(sr * 3)
    clicks = noise * 0.05
    clicks[:: int(sr * 0.0048)] += 3.0          # one sharp front every 4.8 ms (6200 rpm, 4 cylinders)
    kn, kc = static_kurtosis(noise, sr), static_kurtosis(clicks, sr)
    print(f"gaussian noise {kn:.2f} (must be <= {BOUND}); click train {kc:.2f} (must be > {BOUND})")
    assert kn <= BOUND < kc
    print("selftest ok")
    return 0


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a or a[0] not in ("gate", "file", "selftest"):
        sys.exit(__doc__)
    if a[0] == "selftest":
        sys.exit(cmd_selftest())
    if a[0] == "file":
        for p in a[1:]:
            x, sr = read_mono(p)
            print(f"{p}: {static_kurtosis(x, sr):.2f}")
        sys.exit(0)
    sys.exit(cmd_gate(a[1:]))
