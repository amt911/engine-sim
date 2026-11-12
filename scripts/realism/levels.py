#!/usr/bin/env python3
"""Absolute level of a render (WAV at 20 Pa full scale, engine-audio FULL_SCALE_PA): dB SPL Lin
and A-weighted (IEC 61672 A curve), and third-octave-ish bands. usage: levels.py WAV..."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modulation as M  # noqa: E402

FULL_SCALE_PA = 20.0
P_REF = 20e-6


def a_weight(f):
    """IEC 61672-1 A-weighting as a power gain."""
    f2 = f * f
    ra = (12194.0**2 * f2 * f2) / ((f2 + 20.6**2) * np.sqrt((f2 + 107.7**2) * (f2 + 737.9**2))
                                  * (f2 + 12194.0**2))
    return (ra * 10 ** (2.0 / 20)) ** 2


def levels(x):
    x = x * FULL_SCALE_PA
    P = np.abs(np.fft.rfft(x)) ** 2 / len(x) ** 2 * 2
    f = np.fft.rfftfreq(len(x), 1 / M.FS)
    lin = 10 * np.log10(P.sum() / P_REF**2)
    a = 10 * np.log10((P * a_weight(np.maximum(f, 1.0))).sum() / P_REF**2)
    bands = [(lo, 10 * np.log10(P[(f >= lo) & (f < 2 * lo)].sum() / P_REF**2 + 1e-30))
             for lo in (31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000)]
    return lin, a, bands


if __name__ == "__main__":
    for p in sys.argv[1:]:
        lin, a, b = levels(M.load(p)[M.FS // 2:])
        print(f"{os.path.basename(p)[:34]:34s} {lin:6.1f} dB  {a:6.1f} dB(A)  octaves " +
              " ".join(f"{v:5.1f}" for _, v in b))
