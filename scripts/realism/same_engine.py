#!/usr/bin/env python3
"""same_engine.py [--fpr N] WAV... — one measurement for real recordings and renders of the same engine.

Tracks the firing order (fpr firing events per revolution: 2 for a four-stroke inline-4) in 0.25 s
windows, masks the half-order lines (+-1.5 bins, widened by k x the speed's change across the window,
at most 35 % of their spacing), and prints per 1000 rpm bin the medians of:
  wind    residual (not on a line) in 2-8 kHz, dB re the window's total
  lowmid  residual in 150 Hz-2 kHz, dB re total
  lines   share of 1.5-4 kHz energy on the lines, % (how periodic, 'synthetic', the treble is)
Only windows where the tracked firing line stands 10 dB over its neighbourhood count.

Why (2026-11-18): the realism gates judge every preset against 26 pass-by recordings of many cars;
their median called the K20A windy. Against stock DC5 / EP3 dyno clips measured this way the K20A was
in range, and the user agreed by ear. Fetch stock clips of the engine (yt-dlp, local only, never
committed) and run this on them and on the preset's render.
`same_engine.py --self-test` checks the measurement on a synthetic engine of known content.
"""
import sys

import numpy as np
import scipy.io.wavfile as wavfile
import scipy.signal as sg


def load(path):
    fs, x = wavfile.read(path)
    x = x.astype(float)
    if x.ndim > 1:
        x = x.mean(1)
    if fs != 48000:
        x = sg.resample_poly(x, 48000, fs)
    return 48000, x


def windows(x, fs, fpr=2.0, lo_rpm=2500.0, hi_rpm=9300.0):
    """[(rpm, wind dB, lowmid dB, firing-line prominence dB, line share 1.5-4 kHz %)] per window."""
    n, hop = int(0.25 * fs), int(0.1 * fs)
    win = np.hanning(n)
    fr = np.fft.rfftfreq(n, 1 / fs)
    cands = np.arange(lo_rpm, hi_rpm, 20.0)
    idx = {k: np.array([np.argmin(np.abs(fr - k * r / 60 * fpr)) for r in cands]) for k in range(1, 9)}
    picked, spectra, prev = [], [], None
    for s in range(0, len(x) - n, hop):
        X = np.abs(np.fft.rfft(x[s:s + n] * win)) ** 2
        if X.sum() <= 0:
            continue
        sc = sum(X[idx[k]] for k in idx)
        if prev is not None:
            sc = sc * np.exp(-((cands - prev) / 800.0) ** 2)
        prev = cands[np.argmax(sc)]
        picked.append(prev)
        spectra.append(X)
    rows, bin_hz = [], fs / n
    for i, (r, X) in enumerate(zip(picked, spectra)):
        # the speed's change across one window (from the neighbours' tracks) smears line k by k times it
        a, b = picked[max(i - 1, 0)], picked[min(i + 1, len(picked) - 1)]
        drift = abs(b - a) / max(min(i + 1, len(picked) - 1) - max(i - 1, 0), 1) * (n / hop) / r
        f0 = r / 60 * fpr
        half = f0 / 2
        lines = np.zeros_like(fr, bool)
        for k in range(1, int(20000 / half)):
            width = min(1.5 * bin_hz + 0.5 * k * half * drift, 0.35 * half)
            lines |= np.abs(fr - k * half) < width
        band = lambda a, b, m: X[(fr >= a) & (fr < b) & m].sum()  # noqa: E731
        tot = X[(fr > 20) & (fr < 20000)].sum()
        near = X[(fr > f0 * 0.6) & (fr < f0 * 1.4)]
        prom = 10 * np.log10(X[np.argmin(np.abs(fr - f0))] / np.median(near) + 1e-30)
        t15 = band(1500, 4000, np.ones_like(lines)) + 1e-30
        rows.append((r, 10 * np.log10(band(2000, 8000, ~lines) / tot + 1e-30),
                     10 * np.log10(band(150, 2000, ~lines) / tot + 1e-30), prom,
                     100 * band(1500, 4000, lines) / t15))
    return np.array(rows)


def summary(rows):
    """{bin start rpm: (n, wind, lowmid, lines %)} over the windows whose firing line stands out."""
    rows = rows[rows[:, 3] > 10] if len(rows) else rows
    out = {}
    for a in range(3000, 9000, 1000):
        m = (rows[:, 0] >= a) & (rows[:, 0] < a + 1000) if len(rows) else np.array([], bool)
        if m.sum() >= 3:
            out[a] = (int(m.sum()), float(np.median(rows[m, 1])), float(np.median(rows[m, 2])),
                      float(np.median(rows[m, 4])))
    return out


def self_test():
    """A synthetic inline-4 at 6000 rpm: firing harmonics (200 Hz, amplitude 1/k) to 8 kHz plus white noise of
    a known level. The residual it reads in 2-8 kHz must be the noise's own share there (by construction:
    the noise's power density x 6 kHz over the total), within 1 dB."""
    fs, secs = 48000, 4.0
    t = np.arange(int(fs * secs)) / fs
    tone = sum(np.sin(2 * np.pi * 200 * k * t) / k for k in range(1, 40))
    rng = np.random.default_rng(1)
    sigma = 0.02
    x = tone + sigma * rng.standard_normal(len(t))
    p_tone = sum(0.5 / k ** 2 for k in range(1, 40))
    p_noise_28 = sigma ** 2 * 6000 / (fs / 2)          # white: share of the band
    # held steady, each line's mask is +-1.5 bins of 4 Hz: 12 Hz of every 100 Hz holds noise counted as line
    want = 10 * np.log10(p_noise_28 * (1 - 12 / 100) / (p_tone + sigma ** 2))
    got = summary(windows(x, fs, lo_rpm=5000, hi_rpm=7000))
    wind = got[6000][1]
    ok = abs(wind - want) < 1.0
    print(f"self-test: wind {wind:.2f} dB, by construction {want:.2f} dB: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def main(argv):
    if argv[:1] == ["--self-test"]:
        return self_test()
    fpr = 2.0
    if argv[:1] == ["--fpr"]:
        fpr, argv = float(argv[1]), argv[2:]
    for p in argv:
        fs, x = load(p)
        print(f"== {p.split('/')[-1]}")
        for a, (n, wind, lowmid, lines) in summary(windows(x, fs, fpr)).items():
            print(f"   {a}-{a + 1000} rpm n={n:3d}  wind {wind:6.1f}  lowmid {lowmid:6.1f}  lines 1.5-4 kHz {lines:5.1f} %")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
