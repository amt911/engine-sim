#!/usr/bin/env python3
"""Spectral balance against engine speed, recordings beside renders of the same engine.

Each 0.2 s window: the firing frequency by harmonic summation (orders 1..8 of f0, f0 in the
engine's firing range), so rpm = 60·f0 / firings_per_rev; its octave levels 125 Hz..8 kHz re the
window's total (dB), so a recording's level (AGC, mic gain) drops out and only its timbre stays.
Windows are grouped in rpm bins and the median per bin printed, one row per file.

usage: tilt.py FIRINGS_PER_REV RPM_MIN RPM_MAX FILE.wav [FILE.wav ...]
       tilt.py --self-test
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modulation as M  # noqa: E402

OCTAVES = [125, 250, 500, 1000, 2000, 4000, 8000]
WIN_S = 0.2
BIN_RPM = 1000


def f0_of(S, fr, lo, hi):
    """The firing frequency: the f0 in [lo, hi] whose orders 1..8 carry the most power, with
    odd sub-multiples penalised by requiring order 1 present (half-f0 errors pick a weak
    fundamental)."""
    df = fr[1] - fr[0]
    cands = np.arange(lo, hi, 0.5)
    score = np.zeros_like(cands)
    for m in range(1, 9):
        idx = np.clip(np.round(m * cands / df).astype(int), 0, len(S) - 1)
        score += np.log(S[idx] + 1e-30) * (1.0 if m > 1 else 2.0)
    f = cands[int(np.argmax(score))]
    # a sub-multiple wins the sum when the engine has half orders (a V10's banks): every order
    # of f/2 lands on content. Climb while f's odd multiples sit 3 dB under its even ones — then
    # the odd ones are the half orders and 2f is the firing frequency.
    def mean_db(ks, f):
        idx = np.clip(np.round(np.array(ks) * f / df).astype(int), 0, len(S) - 1)
        return np.mean(10 * np.log10(S[idx] + 1e-30))
    while 2 * f <= hi and mean_db([1, 3, 5, 7], f) < mean_db([2, 4, 6, 8], f) - 3.0:
        f *= 2
    return f


def analyse(x, fs, per_rev, rpm_lo, rpm_hi):
    n = int(WIN_S * fs)
    N = 4 * n
    fr = np.fft.rfftfreq(N, 1 / fs)
    w = np.hanning(n)
    rows = []
    for k in range(0, len(x) - n, n):
        s = x[k:k + n]
        if np.sqrt(np.mean(s * s)) < 1e-6:
            continue
        S = np.abs(np.fft.rfft(s * w, N)) ** 2
        f0 = f0_of(S, fr, rpm_lo * per_rev / 60, rpm_hi * per_rev / 60)
        tot = S[(fr > 40) & (fr < 11300)].sum()
        octs = [10 * np.log10(S[(fr >= c / 2 ** 0.5) & (fr < c * 2 ** 0.5)].sum() / tot + 1e-30)
                for c in OCTAVES]
        rows.append((60 * f0 / per_rev, octs))
    return rows


def table(rows):
    out = {}
    for rpm, o in rows:
        b = int(rpm // BIN_RPM) * BIN_RPM
        out.setdefault(b, []).append(o)
    return {b: np.median(np.array(v), axis=0) for b, v in sorted(out.items()) if len(v) >= 3}


def self_test():
    fs = 48000
    t = np.arange(fs * 2) / fs
    f0 = 100.0  # 6000 rpm at one firing per rev
    x = sum(np.sin(2 * np.pi * m * f0 * t) / m for m in range(1, 30))
    rows = analyse(x, fs, 1.0, 1000, 9000)
    rpms = [r for r, _ in rows]
    assert all(abs(r - 6000) < 60 for r in rpms), rpms[:5]
    # a 1/m comb: the 1 kHz octave holds orders 8..14, the 125 Hz octave order 1 → by hand
    # 10·lg(Σ_{m=8..14} 1/m²) = 10·lg(0.0642) = −11.9 dB re the fundamental
    o = rows[0][1]
    assert abs((o[3] - o[0]) - (-11.9)) < 0.3, o[3] - o[0]
    # a V10's banks: half-firing orders 16 dB under the firing ones (the LFA's fingerprint,
    # papers/recordings/README.txt) must not halve the speed
    half = sum(np.sin(2 * np.pi * (m + 0.5) * f0 * t) / (m + 0.5) * 10 ** (-16 / 20)
               for m in range(0, 30))
    rows = analyse(x + half, fs, 1.0, 1000, 9000)
    rpms = [r for r, _ in rows]
    assert all(abs(r - 6000) < 60 for r in rpms), rpms[:5]
    # our LFA render at 8600 rpm has its bank orders only 4–5 dB under the firing ones: still
    # the firing frequency, not half of it (f0 120 Hz, 7200 rpm at one firing per rev)
    f1 = 120.0
    y = sum(np.sin(2 * np.pi * m * f1 * t) / m for m in range(1, 60))
    y += sum(np.sin(2 * np.pi * (m + 0.5) * f1 * t) / (m + 1) * 10 ** (-5 / 20)
             for m in range(0, 60))
    rpms = [r for r, _ in analyse(y, fs, 1.0, 800, 9500)]
    assert all(abs(r - 7200) < 60 for r in rpms), rpms[:5]
    print("self-test ok")


def main(argv):
    if argv[:1] == ["--self-test"]:
        return self_test()
    per_rev, lo, hi = float(argv[0]), float(argv[1]), float(argv[2])
    print("file".ljust(34) + "rpm   " + " ".join(f"{c:>6d}" for c in OCTAVES))
    for p in argv[3:]:
        t = table(analyse(M.load(p), M.FS, per_rev, lo, hi))
        for b, o in t.items():
            print(f"{os.path.basename(p)[:33]:34s}{b:5d} " + " ".join(f"{v:6.1f}" for v in o))


if __name__ == "__main__":
    main(sys.argv[1:])
