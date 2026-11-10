#!/usr/bin/env python3
"""Physics realism gate: does a rendered engine behave like a real engine as revs, throttle and
lift-off change, with no recording of that engine needed. Four hard checks; a build FAILS on any,
and each failure says what the listener hears and which physics to look at.

  crescendo  full-load A-weighted level against speed from 0.5 x redline up must build:
             slope >= 3 dB(A) per octave of speed and a mean fall under the running maximum
             <= 2 dB(A). Real fixed-gain pulls: median +10.6 dB(A)/octave, lowest 3.2 (11 sessions);
             a test bed reads ~+9 dB unweighted from 3000 to 6000 rpm (Davies & Holland, JSV 223(3),
             1999, Fig. 1c).
  load       at ~0.75 x redline, full throttle must stand >= 3 dB(A) over a closed throttle (real
             exhaust microphones: decelerating -3.6 to -20.5 dB(A) under the pull).
  lift-off   the drive scene: lifting must be >= 3 dB(A) quieter than the pull at the limiter, and
             no 0.1 s burst louder than the pull by more than 1 dB(A).
  comb       peak-to-peak ripple of the broadband floor (the engine's lines removed) <= 12 dB:
             a coherent fixed reflection combs it (51 real recordings: max 11.6 dB; the rigid ground
             image read 16.0, heard as "enlatado, ecos raros").

Validated on the user's ear verdicts (LFA, 2026-11-10): the approved build and main PASS; the
"muy tapado" build fails crescendo, load and lift-off; the "suena fatal" build fails crescendo
and load; the ground-image build fails comb. Blind to: room reverberation, equal-length pipes,
linear (non-steepening) pipes, and to the spectrum itself (a separate spectral tier is needed).

Usage:
  physgate.py run BINARY ENGINE [--out DIR]   render the scenes and judge (ENGINE: a .yaml file or
                                               a built-in preset name); exit 1 on a failure
  physgate.py judge DIR REDLINE                judge renders already in DIR
  physgate.py selftest                         the metrics on synthetic signals
Needs numpy, scipy and ffmpeg: `uv run --with numpy --with scipy python -I physgate.py ...`.
"""
import glob
import os
import re
import subprocess
import sys

import numpy as np
from scipy import signal
from scipy.ndimage import percentile_filter

FS = 48000
FULL_SCALE_PA = 20.0  # the renderer's calibration: 0 dBFS = 20 Pa (crates/engine-audio dsp.rs)
CRESCENDO_MIN_SLOPE = 3.0
CRESCENDO_MAX_DEFICIT = 2.0
LOAD_MIN_SPAN = 3.0
LIFT_MAX = -3.0
BURST_MAX = 1.0
COMB_MAX = 12.0
LOADS = [0.0, 0.15, 0.4, 0.7, 1.0]


def load_wav(path):
    """Mono float at FS through ffmpeg (any format the renderer writes)."""
    raw = subprocess.run(["ffmpeg", "-loglevel", "error", "-i", path, "-ac", "1", "-ar", str(FS),
                          "-f", "f32le", "-"], capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.float32).astype(float)


def a_weight(x, fs=FS):
    """IEC 61672-1 A-weighting, bilinear transform of its analogue poles."""
    f1, f2, f3, f4 = 20.598997, 107.65265, 737.86223, 12194.217
    num = [(2 * np.pi * f4) ** 2 * (10 ** (1.9997 / 20)), 0, 0, 0, 0]
    den = np.polymul([1, 4 * np.pi * f4, (2 * np.pi * f4) ** 2], [1, 4 * np.pi * f1, (2 * np.pi * f1) ** 2])
    den = np.polymul(np.polymul(den, [1, 2 * np.pi * f3]), [1, 2 * np.pi * f2])
    b, a = signal.bilinear(num, den, fs)
    return signal.lfilter(b, a, x)


def dba(x_pa):
    return float(10 * np.log10(np.mean(a_weight(x_pa) ** 2) / 4e-10 + 1e-30))


def drive_metrics(x_pa):
    """The drive scene (idle 0-2 s; blips; full throttle on the limiter 5-7.5 s; lift-off to 11 s)."""
    xa = a_weight(x_pa)

    def seg(t0, t1):
        s = xa[int(t0 * FS):int(t1 * FS)]
        return 10 * np.log10(np.mean(s ** 2) / 4e-10 + 1e-30)

    wot, lift = seg(6.0, 7.4), seg(7.7, 8.5)
    n = int(0.1 * FS)
    after = [10 * np.log10(np.mean(xa[i:i + n] ** 2) / 4e-10 + 1e-30)
             for i in range(int(7.5 * FS), int(9.5 * FS), n // 2)]
    return lift - wot, max(after) - wot


def comb_floor(x, n=8192, f_lo=400.0, f_hi=9000.0, floor_hz=180.0, q_lo=0.0003, q_hi=0.003):
    """Ripple (dB peak-to-peak) of the broadband floor and the strongest delay (ms): each frame's
    spectrum becomes its running 30th percentile over floor_hz (the engine's lines vanish), the
    loud frames' median is detrended, and its cepstrum is read over 0.3-3 ms."""
    idx = np.arange(0, len(x) - n + 1, n // 2)
    if len(idx) < 6:
        return float("nan"), float("nan")
    w = np.hanning(n)
    f = np.fft.rfftfreq(n, 1 / FS)
    k = (f >= f_lo) & (f <= f_hi)
    size = max(5, int(floor_hz / (f[1] - f[0])) | 1)
    rows, lev = [], []
    for i in idx:
        p = np.abs(np.fft.rfft(x[i:i + n] * w)) ** 2
        lev.append(p[k].sum())
        rows.append(10 * np.log10(percentile_filter(p, 30, size=size)[k] + 1e-20))
    rows, lev = np.array(rows), np.array(lev)
    lm = np.median(rows[lev >= np.percentile(lev, 50)], axis=0)
    lk = lm - signal.savgol_filter(lm, min(len(lm) // 2 * 2 - 1, 601), 2)
    c = np.abs(np.fft.rfft(lk * np.hanning(len(lk)), 8 * len(lk)))
    q = np.fft.rfftfreq(8 * len(lk), d=(f[1] - f[0]))
    sel = (q >= q_lo) & (q <= q_hi)
    return float(np.percentile(lk, 97) - np.percentile(lk, 3)), float(q[sel][int(np.argmax(c[sel]))] * 1000)


def crescendo(levels):
    """(slope dB(A)/octave, mean deficit dB(A), worst rpm) of {rpm: dB(A)}."""
    rp = np.array(sorted(levels))
    la = np.array([levels[r] for r in rp])
    slope = float(np.polyfit(np.log2(rp), la, 1)[0])
    runmax = np.maximum.accumulate(la)
    return slope, float(np.mean(runmax - la)), float(rp[int(np.argmax(runmax - la))])


def judge(d, redline):
    """[(check, measured, ok, what is wrong)] of the renders in d."""
    out = []
    full = {}
    for p in glob.glob(os.path.join(d, "held_*_1.0_*.wav")):
        rpm = float(re.search(r"held_(\d+)_", p).group(1))
        if rpm >= 0.5 * redline:
            full[rpm] = dba(load_wav(p)[FS:] * FULL_SCALE_PA)
    if len(full) >= 5:
        s, dfc, worst = crescendo(full)
        ok = s >= CRESCENDO_MIN_SLOPE and dfc <= CRESCENDO_MAX_DEFICIT
        out.append(("crescendo", f"{s:+.1f} dB(A)/octave, mean deficit {dfc:.2f} dB(A), {len(full)} speeds", ok,
                    f"full-load level does not build with speed (deepest hole at {worst:.0f} rpm): a source "
                    "cancels or a path closes as revs rise — intake and exhaust firing tones cancelling at the "
                    "listener, an exhaust trough on the firing order, a silencer path tuned onto it"))
    ladder = {}
    for p in glob.glob(os.path.join(d, "held_*_*_*.wav")):
        m = re.search(r"held_(\d+)_([0-9.]+)_", p)
        ladder.setdefault(int(m.group(1)), {})[float(m.group(2))] = p
    for rpm, by in sorted(ladder.items()):
        if len(by) < 3 or min(by) > 0.01 or max(by) < 0.99:
            continue
        lv = {l: dba(load_wav(p)[FS:] * FULL_SCALE_PA) for l, p in by.items()}
        span = lv[max(lv)] - lv[min(lv)]
        out.append(("load", f"{rpm} rpm: full over closed throttle {span:+.1f} dB(A)", span >= LOAD_MIN_SPAN,
                    "the sound does not follow the load: overrun sources too loud (reverse flow at the valves, "
                    "pipes ringing with no mean flow to damp them) or the fired cycle too quiet"))
    dr = os.path.join(d, "drive.wav")
    if os.path.exists(dr):
        lift, burst = drive_metrics(load_wav(dr) * FULL_SCALE_PA)
        out.append(("lift-off", f"lift {lift:+.1f} dB(A), burst {burst:+.1f} dB(A) re the pull",
                    lift <= LIFT_MAX and burst <= BURST_MAX,
                    "lifting the throttle is not quieter than the pull: the overrun sources stand over the "
                    "fired cycle"))
    gr = os.path.join(d, "gears.wav")
    if os.path.exists(gr):
        rip, q = comb_floor(load_wav(gr))
        out.append(("comb", f"floor ripple {rip:.1f} dB p-p, delay {q:.2f} ms", rip <= COMB_MAX,
                    "the broadband floor is combed: a coherent fixed reflection (rigid ground image, one "
                    "outlet heard as two sources)"))
    return out


def redline_of(binary, engine, d):
    path = engine if os.path.exists(engine) else os.path.join(d, "engine.yaml")
    if not os.path.exists(path):
        subprocess.run([binary, "--preset", engine, "--save", path], check=True, capture_output=True)
    return float(re.search(r"\n\s*redline_rpm: ([0-9.]+)", open(path).read()).group(1)), path


def run(binary, engine, d):
    os.makedirs(d, exist_ok=True)
    red, path = redline_of(binary, engine, d)
    full = sorted({int(round(red * f / 100.0)) * 100 for f in np.linspace(0.5, 1.0, 7)})
    mid = int(round(0.75 * red / 100.0)) * 100
    scenes = [f"held:{r}:1.0:4" for r in full] + [f"held:{mid}:{l}:4" for l in LOADS if l < 1.0]
    scenes += [] if mid in full else [f"held:{mid}:1.0:4"]
    for sc in scenes + ["drive", "gears"]:
        out = os.path.join(d, sc.replace(":", "_") + ".wav")
        if not os.path.exists(out):
            subprocess.run(["nice", "-n19", binary, "--engine", path, "--render", sc, "--out", out],
                           check=True, capture_output=True)
    return judge(d, red)


def report(rows):
    bad = 0
    for name, measured, ok, why in rows:
        print(f"{'PASS' if ok else 'FAIL'}  {name:10} {measured}")
        if not ok:
            bad += 1
            print(f"      -> {why}")
    print("PASS" if bad == 0 and rows else f"FAIL ({bad})" if rows else "NO DATA")
    return 0 if bad == 0 and rows else 1


def selftest():
    rng = np.random.default_rng(1)
    # A-weighting: 0 dB at 1 kHz, about -19.1 dB at 100 Hz (IEC 61672-1 table)
    t = np.arange(FS) / FS
    g = lambda f: 20 * np.log10(np.std(a_weight(np.sin(2 * np.pi * f * t))[FS // 2:]) / np.std(np.sin(2 * np.pi * f * t)))
    assert abs(g(1000.0)) < 0.2, g(1000.0)
    assert abs(g(100.0) + 19.1) < 0.5, g(100.0)
    # a level rising 6 dB per octave of speed has slope +6 and no deficit; a 4 dB hole is found
    lv = {r: 80 + 6 * np.log2(r / 4000) for r in [4000, 5000, 6000, 7000, 8000]}
    s, dfc, _ = crescendo(lv)
    assert abs(s - 6.0) < 1e-9 and dfc == 0.0, (s, dfc)
    lv[7000] -= 4.0
    assert crescendo(lv)[2] == 7000.0
    # comb: white noise is flat; noise plus its 0.82 ms copy (the ground image's delay) is combed
    x = rng.standard_normal(4 * FS)
    flat, _ = comb_floor(x)
    k = int(0.00082 * FS)
    combed, q = comb_floor(x[k:] + 0.9 * x[:-k])
    assert flat < COMB_MAX < combed, (flat, combed)
    assert abs(q - 0.82) < 0.05, q
    print(f"selftest ok: A(1k) {g(1000.0):+.2f}, A(100) {g(100.0):+.1f} dB; comb flat {flat:.1f}, "
          f"combed {combed:.1f} dB at {q:.2f} ms")
    return 0


def main(argv):
    if argv[:1] == ["selftest"]:
        return selftest()
    if argv[:1] == ["judge"] and len(argv) == 3:
        return report(judge(argv[1], float(argv[2])))
    if argv[:1] == ["run"] and len(argv) >= 3:
        d = argv[argv.index("--out") + 1] if "--out" in argv else os.path.join("target", "physgate",
                                                                               os.path.basename(argv[2]))
        return report(run(argv[1], argv[2], d))
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
