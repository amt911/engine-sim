"""sp_sim.py [LABEL ...] | --all [PAR] | --force LABEL...  — raw window statistics of renders/LABEL/held_*.wav
(speed known: no tracking, no autofocus). Cached in spectral/out/sim/LABEL.pkl:
  rows  [(t, rpm, stats)] of the full-load held renders
  part  {load: rows} of the part-load ones
"""
import glob
import os
import pickle
import re
import sys
import warnings
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/realism: rdata
import rdata  # noqa: E402

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
OUT = rdata.DATA + "/spectral/out/sim"
RENDERS = [rdata.DATA + "/renders", rdata.DATA + "/spectral/renders", rdata.DATA + "/wind/renders"]

# label prefix -> engine facts (firing events per revolution, redline, order family, evenly spaced at the outlet)
SPEC = {
    "lfa": dict(fpr=5, redline=9000, family="v10", even=True, engine="lfa"),
    "honda-k20a-tuned": dict(fpr=2, redline=9200, family="i4", even=True),
    "honda-k20a": dict(fpr=2, redline=8400, family="i4", even=True),
    "honda-k20c1": dict(fpr=2, redline=7000, family="i4", even=True, asp="turbo"),
    "inline4": dict(fpr=2, redline=7000, family="i4", even=True),
    "single": dict(fpr=0.5, redline=7500, family="single", even=True),
    "subaru-ej25": dict(fpr=2, redline=6200, family="boxer4-uel", even=False),
    "v8-crossplane": dict(fpr=4, redline=6600, family="v8-cross", even=False),
    # one pipe per bank (merge: null): each bank is a four firing at order 2; heard from the side the two
    # do not cancel. The real Ferrari 458 clip (public) has that bank order as strong as the firing order,
    # so it is legitimate: uneven at the outlets, order structure not judged.
    "v8-flatplane": dict(fpr=4, redline=9000, family="v8-flat", even=False),
    "vw-sdi": dict(fpr=2, redline=4800, family="i4", even=True, fuel="diesel"),
    "vw-tdi": dict(fpr=2, redline=4800, family="i4", even=True, fuel="diesel", asp="turbo"),
    "mazda-renesis": dict(fpr=2, redline=9000, family="rotary2", even=True),
    "mazda-13b-bridgeport": dict(fpr=2, redline=9500, family="rotary2", even=True),
}


def _extra():
    """Entries derived from an engine's own definition (expect.py register): {label: entry}."""
    import json
    p = rdata.DATA + "/spectral/extra_spec.json"
    return json.load(open(p)) if os.path.exists(p) else {}


def spec(label):
    ex = _extra()
    if label in ex:
        d = dict(fuel="petrol", asp="na", engine=label)
        d.update(ex[label])
        d["comb"] = d["fpr"]
        return d
    for k in sorted(SPEC, key=len, reverse=True):
        if label.startswith(k):
            d = dict(fuel="petrol", asp="na", engine=k)
            d.update(SPEC[k])
            d["comb"] = d["fpr"] if d["even"] else (d["fpr"] if d["family"] in ("v8-cross", "v8-flat", "boxer4-uel") else 0.5)
            return d
    raise KeyError(label)


def render_files(label, pattern):
    """Files of a label in the render folders (the first prototype's and this tier's)."""
    out = {}
    for r in RENDERS:
        for p in glob.glob(f"{r}/{label}/{pattern}"):
            out.setdefault(os.path.basename(p), p)
    return [out[k] for k in sorted(out)]


def run(label, force=False):
    import sp_core as C
    try:
        spec(label)
    except KeyError:
        return f"skip {label}: not a known preset (add it to SPEC)"
    os.makedirs(OUT, exist_ok=True)
    out = f"{OUT}/{label}.pkl"
    files = render_files(label, "held_*_*_*.wav")
    if not files and os.path.exists(out):
        return f"have {label} (renders deleted, statistics kept)"
    key = tuple(os.path.basename(p) for p in files)
    if os.path.exists(out) and not force and pickle.load(open(out, "rb")).get("key") == key:
        return f"have {label}"
    rows, part = [], {}
    for p in files:
        m = re.search(r"held_(\d+)_([0-9.]+)_", p)
        rpm, load = float(m.group(1)), float(m.group(2))
        w = C.held_windows(C.A.load(p), rpm)
        if load >= 0.999:
            rows += w
        else:
            part.setdefault(load, []).extend(w)
    pickle.dump(dict(rows=rows, part=part, label=label, entry=spec(label), key=key), open(out, "wb"))
    return f"ok {label}: {len(rows)} full-load windows at {len(set(r[1] for r in rows))} speeds, part loads {sorted(part)}"


def run_scene(label, scene="sweep", force=False):
    """A scene with changing speed (sweep, gears) through the REAL-recording pipeline: tracker, pull
    windows, autofocus. For checking that held renders and tracked pulls measure the same thing."""
    import numpy as np
    import sp_core as C
    out = f"{OUT}/{label}-{scene}.pkl"
    if os.path.exists(out) and not force:
        return f"have {label} {scene}"
    e = spec(label)
    x = C.A.load(render_files(label, scene + ".wav")[0])
    lo, hi = max(0.25 * e["redline"], 60.0 * 18.0 / e["comb"]), 1.03 * e["redline"]
    t, rpm, sal, mar = C.A.track(x, e["comb"], lo, hi, return_margin=True)
    rows = C.tracked_windows(x, t, rpm, sal, mar, e["comb"], (lo, hi))
    pickle.dump(dict(rows=rows, t=t, rpm=rpm, sal=sal, mar=mar, label=label, entry=e), open(out, "wb"))
    return f"ok {label} {scene}: {sum(1 for r in rows if r[2]['regime'] == 'pull')} pull windows"


if __name__ == "__main__":
    if sys.argv[1] == "--all":
        from concurrent.futures import ProcessPoolExecutor
        labels = sorted({os.path.basename(d.rstrip("/")) for r in RENDERS for d in glob.glob(f"{r}/*/")})
        with ProcessPoolExecutor(int(sys.argv[2]) if len(sys.argv) > 2 else 3) as ex:
            for m in ex.map(run, labels):
                print(m, flush=True)
    else:
        force = sys.argv[1] == "--force"
        for lab in sys.argv[1 + force:]:
            print(run(lab, force=force), flush=True)

