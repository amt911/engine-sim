"""wn.py — the broadband floor against the engine's lines.

held_split()    a render held at a known speed: Welch spectrum split, on the engine-order grid, into
                line energy (the orders) and floor energy (everything between them), per bin, in Pa^2
                (the renderer's calibration: 0 dBFS = 20 Pa). Sum over bins = mean square (Parseval).
frame_split()   any audio, no speed needed: per STFT frame the floor is the running 30th percentile of
                the frame's spectrum over 180 Hz (the estimator of the physics tier's comb check), scaled
                to a mean for noise; what stands over it is "lines". Works on sweeps and recordings.
bands()         energy of a split spectrum in octave bands, optionally A-weighted.
"""
import os
import sys

import numpy as np
from scipy import signal
from scipy.ndimage import percentile_filter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT + "/tools")
import rg_audio as A  # noqa: E402

FS = A.FS
FULL_SCALE_PA = 20.0
P0 = 2e-5
OCT = [63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000]
Q30 = -np.log(0.7)          # 30th percentile of an exponential variable over its mean (one periodogram bin of noise)


def a_gain(f):
    """A-weighting as a power gain (IEC 61672-1 analogue response)."""
    f = np.maximum(np.asarray(f, float), 1e-3)
    f2 = f * f
    ra = (12194.217 ** 2 * f2 * f2) / ((f2 + 20.598997 ** 2) * np.sqrt((f2 + 107.65265 ** 2) * (f2 + 737.86223 ** 2)) * (f2 + 12194.217 ** 2))
    return (ra * 10 ** (1.9997 / 20)) ** 2


def spl(e):
    """dB SPL of a mean-square pressure in Pa^2."""
    return 10 * np.log10(np.maximum(e, 1e-30) / P0 ** 2)


def load_pa(path, t0=None, t1=None):
    """A render in pascals."""
    return A.load(path, t0, t1) * FULL_SCALE_PA


def held_split(x, rpm, sub=2, t_skip=1.0, fmax=20000.0):
    """(f, total, floor, line) per bin, Pa^2 (or the signal's unit squared), of a signal held at rpm.
    Lines sit on multiples of rpm/60/sub Hz (sub = 2: half engine orders, the four-stroke cycle;
    3 for a rotor's three faces). The floor under a line is the median of the bins between it and its
    neighbours (corrected to a mean for the number of averaged frames); a line is what its window holds
    over that floor, kept when it holds at least 3x the floor under it."""
    x = x[int(t_skip * FS):]
    df_line = rpm / 60.0 / sub
    n = 16384
    while n < 65536 and df_line / (FS / n) < 16:
        n *= 2
    n = min(n, 1 << int(np.floor(np.log2(len(x)))))
    f, P = signal.welch(x, FS, window="hann", nperseg=n, noverlap=n // 2, scaling="density", detrend=False)
    df = f[1] - f[0]
    P = P * df
    nav = max(1.0, 0.6 * (2 * (len(x) - n) / n + 1))          # effective independent averages (Hann, 50 % overlap)
    medfac = (1 - 1 / (9 * nav)) ** 3                          # median over mean of a gamma variable (Wilson-Hilferty)
    K = int(min(fmax, f[-1] - df_line) / df_line)
    centres = np.round(np.arange(1, K + 1) * df_line / df).astype(int)
    hw = np.maximum(3, np.ceil(0.0005 * centres).astype(int))
    floor = np.zeros_like(P)
    line = np.zeros_like(P)
    is_line = np.zeros(len(P), bool)
    for c, w in zip(centres, hw):
        is_line[max(c - w, 0):c + w + 1] = True
    # floor per gap, spread over the gap and half of each neighbouring line window
    edges = np.concatenate([[0], centres, [len(P) - 1]])
    gap_floor = np.zeros(len(edges) - 1)
    for i in range(len(edges) - 1):
        a, b = edges[i], edges[i + 1]
        g = P[a:b + 1][~is_line[a:b + 1]]
        gap_floor[i] = np.median(g) / medfac if len(g) else np.nan
    # fill gaps without bins (should not happen with 16 bins per line spacing)
    ok = np.isfinite(gap_floor)
    gap_floor = np.interp(np.arange(len(gap_floor)), np.flatnonzero(ok), gap_floor[ok])
    for i in range(len(edges) - 1):
        floor[edges[i]:edges[i + 1] + 1] = gap_floor[i]
    for j, (c, w) in enumerate(zip(centres, hw)):
        under = 0.5 * (gap_floor[j] + gap_floor[j + 1])
        floor[c - w:c + w + 1] = under
        e = P[c - w:c + w + 1].sum()
        if e > 3.0 * under * (2 * w + 1):
            line[c] = e - under * (2 * w + 1)
    return f, P, floor, line


def frame_split(x, n=8192, hop=None, floor_hz=180.0, q=30):
    """(t, f, P, F): per frame the power spectrum (mean-square per bin) and its broadband floor: the
    running q-th percentile over floor_hz, divided by what that percentile is for noise (so that a
    frame of pure noise has F = its smoothed P)."""
    hop = hop or n // 2
    w = np.hanning(n)
    idx = np.arange(0, len(x) - n + 1, hop)
    f = np.fft.rfftfreq(n, 1 / FS)
    size = max(5, int(floor_hz / (f[1] - f[0])) | 1)
    P = np.empty((len(idx), len(f)))
    F = np.empty_like(P)
    qf = -np.log(1 - q / 100.0)
    for j, i in enumerate(idx):
        p = np.abs(np.fft.rfft(x[i:i + n] * w)) ** 2 * (2.0 / (np.sum(w ** 2) * n))
        P[j] = p
        F[j] = percentile_filter(p, q, size=size, mode="nearest") / qf
    return (idx + n / 2) / FS, f, P, F


def bands(f, e, centres=OCT, weight=None):
    """Energy per octave band (centre/sqrt2 .. centre*sqrt2); weight: None or 'A'."""
    g = a_gain(f) if weight == "A" else 1.0
    ew = e * g
    return np.array([ew[..., (f >= c / np.sqrt(2)) & (f < c * np.sqrt(2))].sum(axis=-1) for c in centres])


def total(f, e, lo=30.0, hi=20000.0, weight=None):
    g = a_gain(f) if weight == "A" else 1.0
    k = (f >= lo) & (f < hi)
    return (e * g)[..., k].sum(axis=-1)
