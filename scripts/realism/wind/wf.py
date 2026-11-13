"""wf.py — features of the speed-free band spectra (wfeat.py) and tables per recording / render.

  rf28      broadband floor energy in 2-8 kHz re the window's total (30 Hz - 20 kHz), dB        "hiss / wind"
  rf416     the same for 4-16 kHz
  flA       A-weighted floor over A-weighted total, dB (how much of what is heard is not the engine's lines)
  tiltF     slope of the floor's density from 500 Hz to 8 kHz, dB per octave (white noise 0, pink -3)
  hfprom    prominence of the highest hump of the spectrum's density above 4.5 kHz over both of its flanks, dB
  hffreq    its frequency, Hz
  hfshare   A-weighted energy within a third of an octave of that hump re the A-weighted total, dB
  hfx       the part of it that stands over the hump's flanks: hfshare + 10 log10(1 - 10^(-hfprom/10)), dB
"""
import glob
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT + "/spectral")
import wn  # noqa: E402
from wfeat import FC  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/realism: rdata
import rdata  # noqa: E402

BW = FC * (2 ** (1 / 12) - 2 ** (-1 / 12))
AG = wn.a_gain(FC)
EDGES = [0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05]
MIN_N = 6
HUMP_LO, HUMP_HI = 4500.0, 14500.0


def db(x):
    return 10 * np.log10(np.maximum(x, 1e-12))


def smooth3(d):
    k = np.array([0.25, 0.5, 0.25])
    return np.convolve(np.pad(d, 1, mode="edge"), k, mode="valid")


def hump(P):
    """(prominence dB, frequency Hz, band index) of the highest hump of the density above HUMP_LO over both flanks:
    the lowest density within 1.3 octaves below it, and within 2/3 octave above it (not beyond 18 kHz)."""
    D = smooth3(db(P / BW))
    best = (0.0, 0.0, -1)
    for b in np.flatnonzero((FC >= HUMP_LO) & (FC <= HUMP_HI)):
        lo = (FC >= FC[b] / 2.5) & (FC < FC[b] / 1.12)
        hi = (FC > FC[b] * 1.12) & (FC <= min(FC[b] * 1.6, 18500.0))
        if not lo.any() or not hi.any():
            continue
        p = D[b] - max(D[lo].min(), D[hi].min())
        if p > best[0]:
            best = (float(p), float(FC[b]), int(b))
    return best


def features(wb):
    P, F = wb["P"].astype(float), np.minimum(wb["F"].astype(float), wb["P"].astype(float) * 1.5)
    o = {}
    band = lambda lo, hi: (FC >= lo) & (FC < hi)   # noqa: E731
    o["rf28"] = float(db(F[band(2000, 8000)].sum()))
    o["rf416"] = float(db(F[band(4000, 16000)].sum()))
    o["rf816"] = float(db(F[band(8000, 16000)].sum()))
    o["p816"] = float(db(P[band(8000, 16000)].sum()))
    o["flA"] = float(db((F * AG).sum()) - db((P * AG).sum()))
    k = band(500, 8000)
    o["tiltF"] = float(np.polyfit(np.log2(FC[k]), db(F[k] / BW[k]), 1)[0])
    pr, fr, b = hump(P)
    o["hfprom"], o["hffreq"] = pr, fr
    if b >= 0:
        near = (FC >= FC[b] / 2 ** (1 / 3)) & (FC <= FC[b] * 2 ** (1 / 3))
        o["hfshare"] = float(db((P * AG)[near].sum()) - db((P * AG).sum()))
    else:
        o["hfshare"] = -60.0
    # how much sound the hump adds: the A-weighted energy it holds over its flanks' level, re the A-weighted total
    o["hfx"] = float(o["hfshare"] + 10 * np.log10(1 - 10 ** (-pr / 10))) if pr > 0.1 else -60.0
    o["hfx"] = max(o["hfx"], -60.0)
    return o


def table(rows, e, min_n=MIN_N):
    acc = {}
    for t, rpm, wb in rows:
        i = int(np.searchsorted(EDGES, rpm / e["redline"], side="right")) - 1
        if 0 <= i < len(EDGES) - 1:
            acc.setdefault(i, []).append(features(wb))
    out = {}
    for i, lst in acc.items():
        if len(lst) >= min_n:
            out[i] = {k: float(np.median([d[k] for d in lst])) for k in lst[0]}
            out[i]["n"] = len(lst)
            # the mean band spectrum of the bin (for plots)
    return out


def load(kind):
    """{id: (entry, table, n windows)}"""
    out = {}
    for p in sorted(glob.glob(f"{rdata.DATA}/wind/out/{kind}/*.pkl")):
        d = pickle.load(open(p, "rb"))
        if not d["rows"]:
            continue
        T = table(d["rows"], d["entry"])
        if T:
            out[os.path.basename(p)[:-4]] = (d["entry"], T, len(d["rows"]))
    return out


def summary(T, f):
    v = [T[b][f] for b in sorted(T)]
    return float(np.median(v)), float(max(v)), float(min(v))


if __name__ == "__main__":
    feats = ["rf28", "rf416", "rf816", "p816", "flA", "tiltF", "hfprom", "hffreq", "hfshare"]
    ref, pub, sim = load("ref"), load("pub"), load("sim")
    print(len(ref), "library recordings,", len(pub), "public,", len(sim), "renders with at least one speed bin")
    def line(name, e, T, n):
        return f"{name[:30]:30s} {e.get('mic','sim'):5s} {e.get('cls','-'):3s} {len(T)}b {n:4d}w " + " ".join(f"{summary(T, f)[0]:7.1f}" + (f"/{summary(T, f)[1]:6.1f}" if f not in ("hffreq", "tiltF") else "       ") for f in feats)
    print(f"{'':48s} " + " ".join(f"{f + ' med/max':>14s}" for f in feats))
    for title, D, flt in (("library, outside microphones", ref, lambda e: e["cls"] == "out"), ("library, engine bay / interior / dyno", ref, lambda e: e["cls"] != "out"), ("public (microphone unknown)", pub, lambda e: True)):
        print("\n" + title)
        for i, (e, T, n) in D.items():
            if flt(e) and n >= 12:
                print(line(i, e, T, n))
    print("\nrenders")
    want = sys.argv[1:] or sorted(sim)
    for i in want:
        if i in sim:
            e, T, n = sim[i]
            print(line(i, e, T, n))
