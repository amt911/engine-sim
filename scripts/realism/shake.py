"""shake.py — informative: does the sound tremble after a gear change? (never a gate; exit 0)

After each upshift the drivetrain's shuffle (the clutch and shafts' torsional mode, 2-10 Hz)
rocks the engine speed and load; heard as a level that wobbles for a second or so. This finds the
shifts in a render's gears scene (the level drops > 6 dB in under 0.3 s and comes back), and in the
1.5 s after the level is back measures the A-weighted level's oscillation between 2 and 12 Hz:
its rms (dB) and its strongest frequency. Real recordings with gear changes and a microphone that
rides with the vehicle (onboard, exhaust: no pass-by Doppler or distance swing) give the range.

usage: shake.py LABEL ...     renders/LABEL/gears.wav against the onboard gear-change recordings
"""
import json
import os
import sys

import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "spectral"))
sys.path.insert(0, HERE)
import rdata  # noqa: E402
import sp_core as C  # noqa: E402

FS = C.FS
HOP = 0.01                       # level frames, s
AFTER = 1.5                      # s analysed after each shift
REAL_IDS = ["drz-exh", "scrambler-exh", "pantera-exh", "s1000rr-onb", "rmz-int", "scania-eng"]


def a_level(x):
    """A-weighted level every HOP s over 20 ms (IEC 61672 A curve as a bilinear filter)."""
    f1, f2, f3, f4 = 20.598997, 107.65265, 737.86223, 12194.217
    num = [(2 * np.pi * f4) ** 2 * 10 ** (1.9997 / 20), 0, 0, 0, 0]
    den = np.polymul([1, 4 * np.pi * f4, (2 * np.pi * f4) ** 2], [1, 4 * np.pi * f1, (2 * np.pi * f1) ** 2])
    den = np.polymul(np.polymul(den, [1, 2 * np.pi * f3]), [1, 2 * np.pi * f2])
    b, a = signal.bilinear(num, den, FS)
    y = signal.lfilter(b, a, x)
    h, w = int(HOP * FS), int(0.02 * FS)
    e = np.array([np.mean(y[i:i + w] ** 2) for i in range(0, len(y) - w, h)])
    return 10 * np.log10(e + 1e-20)


def shifts(L):
    """Frame indices where the level is back after a > 6 dB drop that lasted under 0.6 s."""
    out, i, n = [], 0, len(L)
    sm = np.convolve(L, np.ones(5) / 5, mode="same")
    while i < n - 100:
        pre = np.max(sm[max(0, i - 30):i + 1])
        j = i + 1
        if sm[j:j + 30].min() < pre - 6.0:
            k = j + int(np.argmin(sm[j:j + 30]))
            back = np.nonzero(sm[k:k + 60] > pre - 3.0)[0]
            if len(back):
                r = k + int(back[0])
                if r + int(AFTER / HOP) < n:
                    out.append(r)
                i = r + int(AFTER / HOP)
                continue
        i += 1
    return out


def wobble(L, r):
    """rms (dB) and peak frequency (Hz) of the level's 2-12 Hz part over AFTER s from frame r."""
    seg = L[r:r + int(AFTER / HOP)]
    t = np.arange(len(seg))
    seg = seg - np.polyval(np.polyfit(t, seg, 2), t)          # the climb through the gear, out
    b, a = signal.butter(2, [2.0, 12.0], "band", fs=1 / HOP)
    y = signal.filtfilt(b, a, seg)
    f, P = signal.periodogram(seg * np.hanning(len(seg)), 1 / HOP)
    m = (f >= 2) & (f <= 12)
    return float(np.sqrt(np.mean(y ** 2))), float(f[m][np.argmax(P[m])])


def pitch(x):
    """Strongest line between 40 and 700 Hz every HOP s (0.1 s windows, parabolic peak), Hz."""
    b, a = signal.butter(4, 900.0, fs=FS)
    y = signal.lfilter(b, a, x)
    h, w = int(HOP * FS), int(0.1 * FS)
    win = np.hanning(w)
    f = np.fft.rfftfreq(4 * w, 1 / FS)
    m = (f >= 40) & (f <= 700)
    out = []
    for i in range(0, len(y) - w, h):
        S = np.abs(np.fft.rfft(y[i:i + w] * win, 4 * w))
        k = np.flatnonzero(m)[np.argmax(S[m])]
        a0, a1, a2 = np.log(S[k - 1:k + 2] + 1e-20)
        d = 0.5 * (a0 - a2) / (a0 - 2 * a1 + a2 + 1e-20)
        out.append(f[k] + d * (f[1] - f[0]))
    return np.array(out)


def pitch_wobble(F, r):
    """rms (cents) of the pitch's 2-12 Hz part over AFTER s from frame r (the climb taken out)."""
    seg = 1200 * np.log2(np.maximum(F[r:r + int(AFTER / HOP)], 1.0))
    if len(seg) < int(AFTER / HOP):
        return np.nan
    med = np.median(seg)
    seg = np.clip(seg, med - 600, med + 600)                   # octave jumps of the tracker
    t = np.arange(len(seg))
    seg = seg - np.polyval(np.polyfit(t, seg, 2), t)
    b, a = signal.butter(2, [2.0, 12.0], "band", fs=1 / HOP)
    return float(np.sqrt(np.mean(signal.filtfilt(b, a, seg) ** 2)))


def report(name, x):
    L = a_level(x)
    F = pitch(x)
    rs = shifts(L)
    pw = [pitch_wobble(F, r) for r in rs]
    w = [wobble(L, r) for r in rs]
    if not w:
        print(f"  {name:28s} no shift found")
        return None
    rms = [a for a, _ in w]
    print(f"  {name:28s} {len(w)} shifts: wobble {np.median(rms):4.1f} dB rms (max {max(rms):4.1f}) "
          f"at {np.median([f for _, f in w]):4.1f} Hz; pitch wobble {np.nanmedian(pw):5.1f} cents rms")
    return float(np.median(rms)), float(np.nanmedian(pw))


if __name__ == "__main__":
    man = {e["id"]: e for e in json.load(open(os.path.join(rdata.DATA, "spectral", "manifest.json")))}
    print("real recordings with gear changes (microphone riding with the vehicle):")
    real = []
    for i in REAL_IDS:
        e = man.get(i)
        if e and os.path.exists(e["path"]):
            v = report(i, C.decode(e["path"], e.get("channel")))
            if v is not None:
                real.append(v)
    print("renders:")
    for lab in sys.argv[1:]:
        p = os.path.join(rdata.DATA, "renders", lab, "gears.wav")
        v = report(lab, C.A.load(p)) if os.path.exists(p) else None
        if v is not None and real:
            lv, pc = max(r[0] for r in real), max(r[1] for r in real)
            print(f"    -> level {'TREMBLES (over every real one)' if v[0] > lv else 'ok'} (real max {lv:.1f} dB); "
                  f"pitch {'TREMBLES (over every real one)' if v[1] > pc else 'ok'} (real max {pc:.1f} cents)")
