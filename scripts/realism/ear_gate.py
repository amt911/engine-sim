#!/usr/bin/env python3
"""ear_gate.py — the four things the ear named on 2026-11-13, judged against real exterior recordings.

  M1 bank  firing series over the bank orders, 1-2 kHz (>= 0.5 redline, full load)   'sintético, faltan armónicos'
  M3 wind  non-repeating 150-500 Hz re the firing line (0.7-0.9 redline)            'más viento'
  M2 lift  treble change minus 0.5-1 kHz change at each throttle lift               'soplido al soltar'
  M4 peak  sample peak over the loud half's median loudness                          'callado'

Every bound is one-sided, from real recordings only (ear_feat.py says how each is measured): M1 and M3
against recordings of the same engine family (all evenly firing engines when the family has fewer than
two), M2 and M4 against every recording. Each bound is widened by how far the most extreme real
recording falls past the others (leave one out), so no real recording can fail it.

usage
  ear_gate.py refs                 measure the real exterior recordings (cached in $REALISM_DATA/ear)
  ear_gate.py gate LABEL [...]     judge renders (renders/LABEL: held_*_1.0_*.wav, gears.wav)
  ear_gate.py validate             every real recording judged by the others + the ear's verdicts
Why: the spectral tier pooled every engine and microphone, so a defect the ear hears (bank orders
15-20 dB too strong on the LFA) sat inside the pool; see docs/agents/debugging.md (sound realism).
"""
import glob
import json
import os
import pickle
import re
import sys
import warnings
from concurrent.futures import ProcessPoolExecutor

import numpy as np

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "ear"))
sys.path.insert(0, os.path.join(HERE, "spectral"))
import ear_feat as F  # noqa: E402
import rdata  # noqa: E402
from sp_sim import SPEC  # noqa: E402

DATA = rdata.DATA
CACHE = os.path.join(DATA, "ear")
EXT_MICS = ("ext", "mix")
METRICS = {  # name: (side the defect is on, words)
    "bank": ("lo", "bank orders too strong against the firing series: 'sintético, faltan armónicos'"),
    "wind": ("hi", "noise between the orders at 150-500 Hz: 'viento'"),
    "lift": ("hi", "the treble stays when the tone drops at a lift: 'soplido al soltar'"),
    "peak": ("hi", "a transient far above the engine sets the clip's peak: 'callado'"),
}
# The ear's verdicts this gate must reproduce (label, verdict). Builds judged before 2026-11-13 and
# approved then share the bank-order defect; the ear of 2026-11-13 heard it ('sintético') and supersedes.
REJECTED = [
    ("lfa-pf", "2026-11-13 'muy callado, sintético, le faltan armónicos'"),
    ("lfa-subin", "2026-11-13 'peor en todo: más sintético, más viento, soplido al soltar'"),
    ("lfa-tapado", "'muy tapado, muchos armónicos tapados'"),
    ("lfa-fatal", "'suena fatal'"),
    ("lfa-x-pulses", "'sintético'"),
    ("lfa-x-equal", "'demasiado limpio, mosquito'"),
    ("lfa-x-linear", "'suena muy lejos'"),
    ("lfa-x-ground", "'enlatado, ecos raros'"),
    ("lfa-x-reverb", "'como mucho eco'"),
]


def summarise(rows, lifts, peak, redline):
    def med(key, lo, hi, n):
        v = [r[key] for r in rows if lo * redline <= r["rpm"] <= hi * redline and r["lockcv"] < 0.3 and np.isfinite(r[key])]
        return float(np.median(v)) if len(v) >= n else np.nan
    return dict(bank=med("bank", 0.5, 1.0, 4), wind=med("wind", 0.7, 0.9, 3),
                lift=float(np.median([d for _, d in lifts])) if lifts else np.nan, peak=peak, nlift=len(lifts))


def one_ref(entry):
    pk = os.path.join(DATA, "spectral", "out", "ref", entry["id"] + ".pkl")
    rows = F.ref_rows(entry, pickle.load(open(pk, "rb"))) if os.path.exists(pk) else []
    x, fs = F.load(entry["path"])
    return entry["id"], dict(entry=entry, **summarise(rows, F.lifts(x, fs), F.peak_over_loudness(x, fs), entry["redline"]))


def cmd_refs():
    man = [e for e in json.load(open(os.path.join(DATA, "spectral", "manifest.json"))) if e.get("mic") in EXT_MICS]
    out = {}
    with ProcessPoolExecutor(4) as ex:
        for i, r in ex.map(one_ref, man):
            out[i] = r
            print(f"{i:22s} bank {r['bank']:6.1f} wind {r['wind']:6.1f} lift {r['lift']:6.1f} ({r['nlift']}) peak {r['peak']:5.1f}", flush=True)
    os.makedirs(CACHE, exist_ok=True)
    pickle.dump(out, open(os.path.join(CACHE, "refs.pkl"), "wb"))


def spec_of(label):
    return next(v for k, v in sorted(SPEC.items(), key=lambda kv: -len(kv[0])) if label.startswith(k))


def sim(label):
    """The label's metrics, cached by the newest render's time."""
    d = os.path.join(DATA, "renders", label)
    held = sorted(glob.glob(os.path.join(d, "held_*_1.0_*.wav")))
    clip = next((p for p in (os.path.join(d, "gears.wav"), os.path.join(d, "drive.wav")) if os.path.exists(p)), None)
    stamp = max([os.path.getmtime(p) for p in held + ([clip] if clip else [])], default=0)
    cp = os.path.join(CACHE, "sim", label + ".pkl")
    if os.path.exists(cp):
        c = pickle.load(open(cp, "rb"))
        if c.get("stamp") == stamp:
            return c
    sp = spec_of(label)
    fpc = int(round(2 * sp["fpr"]))
    rows = []
    for p in held:
        rpm = float(re.search(r"held_(\d+)_", os.path.basename(p)).group(1))
        rows += F.sim_rows(p, rpm, fpc) if sp.get("even") and fpc >= 2 else []
    lifts, peak = [], np.nan
    if clip:
        x, fs = F.load(clip)
        lifts, peak = F.lifts(x, fs), F.peak_over_loudness(x, fs)
    c = dict(stamp=stamp, family=sp["family"], **summarise(rows, lifts, peak, sp["redline"]))
    os.makedirs(os.path.dirname(cp), exist_ok=True)
    pickle.dump(c, open(cp, "wb"))
    return c


# Microphones each metric is judged against: the bank orders hold at any exterior position; noise,
# lifts and peaks depend on the distance and the recordist (a mixed onboard/exterior clip of the LFA read
# its 150-500 Hz noise 22 dB over the exterior ones), so those take exterior microphones only.
MICS = {"bank": ("ext", "mix"), "wind": ("ext",), "lift": ("ext",), "peak": ("ext",)}
VALID = {"bank": (-60.0, 60.0), "wind": (-60.0, 40.0), "lift": (-40.0, 40.0), "peak": (0.0, 60.0)}


def usable(r, k):
    """A real measurement that can bound metric k (its microphone, and not an artefact: -120 dB bank
    orders where the tracker found no half lines, 180 dB peaks over a file of digital silence)."""
    lo, hi = VALID[k]
    return r["entry"].get("mic") in MICS[k] and np.isfinite(r[k]) and lo < r[k] < hi


def pool(refs, metric, family, exclude=None):
    """Same-family real values of the metric; [] when the family has fewer than two (not judged)."""
    v = [r[metric] for i, r in refs.items() if i != exclude and usable(r, metric) and r["entry"]["family"] == family]
    return v if len(v) >= 2 else []


def bound(refs, metric, family, exclude=None):
    v = pool(refs, metric, family, exclude)
    if not v:
        return np.nan
    return min(v) if METRICS[metric][0] == "lo" else max(v)


def margins(refs, family):
    """{metric: dB} for one family: the most any of its real recordings falls past the bound of the
    others (leave one out, >= 0), so no real recording of the family fails."""
    m = {k: 0.0 for k in METRICS}
    for i, r in refs.items():
        if r["entry"]["family"] != family:
            continue
        for k, (side, _) in METRICS.items():
            b = bound(refs, k, family, exclude=i)
            if usable(r, k) and np.isfinite(b):
                m[k] = max(m[k], (b - r[k]) if side == "lo" else (r[k] - b))
    return m


def judge(refs, item, family, exclude=None):
    M = margins(refs, family)
    out = []
    for k, (side, words) in METRICS.items():
        b = bound(refs, k, family, exclude)
        v = item[k]
        if not (np.isfinite(v) and np.isfinite(b)):
            out.append((k, v, np.nan, "n/a", words))
            continue
        lim = b - M[k] if side == "lo" else b + M[k]
        bad = v < lim if side == "lo" else v > lim
        out.append((k, v, lim, "FAIL" if bad else "ok", words))
    return out


def load_refs():
    p = os.path.join(CACHE, "refs.pkl")
    if not os.path.exists(p):
        sys.exit("no real measurements: run `ear_gate.py refs` first")
    return pickle.load(open(p, "rb"))


def line(name, res):
    v = "FAIL" if any(s == "FAIL" for _, _, _, s, _ in res) else "PASS"
    cells = "  ".join(f"{k} {x:+6.1f}{'<' if METRICS[k][0] == 'lo' else '>'}{lim:+.1f} {s}" if np.isfinite(lim) else f"{k} n/a"
                      for k, x, lim, s, _ in res)
    return v, f"{name:26s} {v}  {cells}"


def cmd_gate(labels):
    refs = load_refs()
    rc = 0
    for lab in labels:
        it = sim(lab)
        v, s = line(lab, judge(refs, it, it["family"]))
        print(s)
        for k, x, lim, st, words in judge(refs, it, it["family"]):
            if st == "FAIL":
                print(f"    -> {words}")
        rc |= v == "FAIL"
    return rc


def cmd_validate():
    refs = load_refs()
    for fam in sorted({r["entry"]["family"] for r in refs.values()}):
        M = margins(refs, fam)
        n = {k: len(pool(refs, k, fam)) for k in METRICS}
        if any(n.values()):
            print(f"{fam:9s} " + "  ".join(f"{k}: n={n[k]} bound {bound(refs, k, fam):+.1f} margin {M[k]:.1f}" for k in METRICS if n[k]))
    bad = 0
    print("\nreal exterior recordings, each against the others:")
    for i, r in sorted(refs.items()):
        res = [(k, x, lim, st if usable(r, k) else "n/a", w) for k, x, lim, st, w in judge(refs, r, r["entry"]["family"], exclude=i)]
        v, s = line(f"REAL {i}", res)
        print(s)
        bad += v == "FAIL"
    print("\nbuilds the ear rejected (must FAIL):")
    for lab, why in REJECTED:
        if not os.path.isdir(os.path.join(DATA, "renders", lab)):
            print(f"{lab:26s} (no renders)")
            continue
        it = sim(lab)
        v, s = line(lab, judge(refs, it, it["family"]))
        print(s + f"   ear: {why}")
        bad += v != "FAIL"
    print(f"\n{'VALID' if bad == 0 else 'NOT VALID'}: {bad} disagreement(s)")
    return int(bad > 0)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[:1] == ["refs"]:
        cmd_refs()
    elif a[:1] == ["gate"] and len(a) > 1:
        sys.exit(cmd_gate(a[1:]))
    elif a[:1] == ["validate"]:
        sys.exit(cmd_validate())
    else:
        sys.exit(__doc__)
