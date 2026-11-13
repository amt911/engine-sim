"""wideband.py — the noise between the engine's lines, third octave by third octave, 100 Hz to 16 kHz.

The spectral tier judges wind in two bands under 8 kHz (its references include compressed video
audio, cut above). This looks at the whole band with the uncompressed outside recordings only (WAV
from field-recording libraries), so a hiss over 8 kHz or a rush in one third octave shows.

  floor share(b) = energy between the half-order lines / all the energy, in third octave b (dB),
  on the crank-angle grid (32 revolutions a window, so every half order is a line of its own).
  A real exhaust's floor is turbulence; a share over every real recording's at matched speed is wind.

usage: wideband.py refs                cache the real recordings' shares -> $REALISM_DATA/out-wideband-real.pkl
       wideband.py judge LABEL ...     a render against them; informative (exit 0), prints the bands
                                       where the build is windier than the windiest real recording
"""
import glob
import os
import pickle
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "spectral"))
sys.path.insert(0, HERE)
import rdata  # noqa: E402
import sp_core as C  # noqa: E402
import sp_sim as SIM  # noqa: E402

FS = C.FS
LN2 = np.log(2.0)
CENTRES = [100 * 2 ** (i / 3) for i in range(0, 23)]          # 100 Hz .. 16 kHz
REAL = os.path.join(rdata.DATA, "out-wideband-real.pkl")
EDGES = [0.25, 0.45, 0.65, 0.85, 1.05]                         # speed / redline


def shares(seg, rpm_t, rc):
    """Floor share per third octave (dB) of one window, nan where the band is past the analysis or
    empty. The spectral tier's own window statistics (autofocused speed, line vs floor energy per
    half order; sp_feat.window_features' split), its ceiling raised to 16 kHz for WAV sources."""
    C.F_HI = 16500.0
    st = C.window_stats(seg, rpm_t, rc, want_sharp=False)
    if st is None:
        return None
    rpm = rc
    E = np.nan_to_num(st["E"].astype(float))
    F = np.nan_to_num(st["F"].astype(float))
    nb = 2 * st["W"].astype(float) + 1
    k = np.arange(1, len(E) + 1)
    under = nb * F / LN2
    det = (E > 3.0 * under) & (E > 0)
    line = np.where(det, E - under, 0.0)
    cell = E + (C.STEP - nb) * F / LN2
    floor = np.maximum(cell - line, 0.0)
    f = k * rpm / 120.0
    ok = k >= st["k_lo"]
    out = []
    for c in CENTRES:
        m = ok & (f >= c / 2 ** (1 / 6)) & (f < c * 2 ** (1 / 6))
        tot = cell[m].sum()
        out.append(10 * np.log10(floor[m].sum() / tot + 1e-30) if m.sum() >= 2 and tot > 0 else np.nan)
    return np.array(out)


def bin_of(x):
    for i in range(len(EDGES) - 1):
        if EDGES[i] <= x < EDGES[i + 1]:
            return i
    return None


def refs():
    out = {}
    for p in sorted(glob.glob(os.path.join(rdata.DATA, "spectral", "out", "ref", "*.pkl"))):
        d = pickle.load(open(p, "rb"))
        e = d["entry"]
        path = e["path"]
        if e["cls"] != "out" or e["mic"] == "tail" or not path.lower().endswith(".wav") or "/refs/" in path:
            continue
        if not os.path.exists(path):
            continue
        x = C.decode(path, e.get("channel"))
        rows = []
        for tc, rc, st0 in d["rows"]:
            if st0.get("regime") != "pull":
                continue
            a, b, c0, c1, s_, q_ = st0["win"]
            seg, tt, rpm_t = C._one(x, a, b, (c0, c1), s_, q_, False)   # the tier's own window and speed
            sh = shares(seg, rpm_t, rc)
            if sh is not None:
                rows.append((rc / e["redline"], sh))
        if rows:
            out[e["id"]] = rows
            print(e["id"], len(rows), flush=True)
    pickle.dump(out, open(REAL, "wb"))


def per_bin(rows):
    """{bin: median shares over its windows}"""
    out = {}
    for b in range(len(EDGES) - 1):
        s = [r for x, r in rows if bin_of(x) == b]
        if len(s) >= 3:
            out[b] = np.nanmedian(np.array(s), axis=0)
    return out


def judge(labels):
    real = pickle.load(open(REAL, "rb"))
    R = {i: per_bin(v) for i, v in real.items()}
    for lab in labels:
        e = SIM.spec(lab)
        rows = []
        for f in sorted(glob.glob(os.path.join(rdata.DATA, "renders", lab, "held_*_1.0_*.wav"))):
            rpm = float(re.search(r"held_(\d+)_", f).group(1))
            x = C.A.load(f)[FS:]
            n = int(32 * 60.0 / rpm * FS) + 64
            for i in range(0, len(x) - n, n):
                s = shares(x[i:i + n], np.full(n, rpm), rpm)
                if s is not None:
                    rows.append((rpm / e["redline"], s))
        mine = per_bin(rows)
        print(f"\n{lab}: floor share between the lines, dB (build | real median, real max; * over the max)")
        print("  band Hz " + " ".join(f"{c:>6.0f}" for c in CENTRES))
        for b, v in sorted(mine.items()):
            pool = np.array([r[b] for r in R.values() if b in r])
            if len(pool) < 3:
                continue
            med, mx = np.nanmedian(pool, axis=0), np.nanmax(pool, axis=0)
            print(f"  {EDGES[b]:.2f}-{EDGES[b + 1]:.2f} " + " ".join(
                f"{v[k]:5.1f}{'*' if v[k] > mx[k] else ' '}" if np.isfinite(v[k]) else "     . " for k in range(len(CENTRES))))
            print(f"   real med " + " ".join(f"{m:6.1f}" if np.isfinite(m) else "     ." for m in med))
            print(f"   real max " + " ".join(f"{m:6.1f}" if np.isfinite(m) else "     ." for m in mx) + f"   (n={len(pool)})")


if __name__ == "__main__":
    if sys.argv[1] == "refs":
        refs()
    else:
        judge(sys.argv[2:])
