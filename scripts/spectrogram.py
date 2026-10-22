#!/usr/bin/env python3
"""Spectrogram of a drive render (crates/engine-app sim.rs `drive_render`) with the engine-order
lines on top and, below, the share of each frame's energy that lies off every cycle order
(k * rpm/120 Hz, +-2 bins): the aperiodic part a listener hears as "air".
Usage: scripts/spectrogram.py target/sound/lfa-all.wav [more.wav ...] -> <wav>.png"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def load(path):
    # 32-bit float WAV (hound): walk the RIFF chunks for the rate and the data
    b = open(path, "rb").read()
    i, fs, data = 12, None, None
    while i < len(b) - 8:
        cid, size = b[i:i + 4], int.from_bytes(b[i + 4:i + 8], "little")
        if cid == b"fmt ":
            fs = int.from_bytes(b[i + 12:i + 16], "little")
        elif cid == b"data":
            data = b[i + 8:i + 8 + size]
        i += 8 + size + (size & 1)
    return np.frombuffer(data, dtype="<f4").astype(np.float64), fs

for path in sys.argv[1:]:
    x, fs = load(path)
    csv = np.loadtxt(path[:-4] + ".csv", delimiter=",")
    t_c, thr, rpm_c = csv[:, 0], csv[:, 1], csv[:, 2]
    n, hop = 4096, 1024
    win = np.hanning(n)
    frames = range(0, len(x) - n, hop)
    S = np.array([np.abs(np.fft.rfft(x[i:i + n] * win)) ** 2 for i in frames]).T
    f = np.fft.rfftfreq(n, 1 / fs)
    t = np.array([(i + n / 2) / fs for i in frames])
    rpm = np.interp(t, t_c, rpm_c)
    off = []
    for j, r in enumerate(rpm):
        f0 = max(r, 1) / 120.0
        k = np.round(f / f0)
        near = (np.abs(f - k * f0) <= 2.5 * fs / n) & (k >= 1)
        tot = S[1:, j].sum()
        off.append(S[1:, j][~near[1:]].sum() / tot if tot > 0 else 0)
    fig, ax = plt.subplots(3, 1, figsize=(13, 10), sharex=True,
                           gridspec_kw={"height_ratios": [4, 1, 1]})
    db = 10 * np.log10(S + 1e-14)
    ax[0].pcolormesh(t, f, db, shading="auto", vmin=db.max() - 80, vmax=db.max(), cmap="magma")
    ax[0].set_ylim(0, 8000)
    ax[0].set_ylabel("Hz")
    ax[0].set_title(path)
    ax[1].plot(t_c, rpm_c); ax[1].set_ylabel("rpm")
    ax2 = ax[1].twinx(); ax2.plot(t_c, thr, "r", alpha=0.4); ax2.set_ylabel("throttle")
    ax[2].plot(t, 100 * np.array(off)); ax[2].set_ylabel("% off-order"); ax[2].set_ylim(0, 100)
    ax[2].set_xlabel("s")
    lvl = [10 * np.log10(np.mean(x[i:i + n] ** 2) + 1e-14) for i in frames]
    ax3 = ax[2].twinx(); ax3.plot(t, lvl, "g", alpha=0.5); ax3.set_ylabel("dBFS")
    fig.tight_layout()
    fig.savefig(path[:-4] + ".png", dpi=80)
    print(path[:-4] + ".png", "mean off-order %.0f%%" % (100 * np.mean(off)))
