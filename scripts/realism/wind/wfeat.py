"""wfeat.py — speed-free spectra of analysis windows, for the wind / electric checks.

For one window of sound (a pull window of a recording, a held render): the spectrum and its broadband
floor (wn.frame_split: running 30th percentile over 180 Hz) in 1/6-octave bands from 50 Hz to 20 kHz,
as energies relative to the window's total (30 Hz - 20 kHz). Everything is a ratio inside the window.

  wfeat.py refs [PAR]            every library recording with analysed pull windows (spectral/out/ref/*.pkl)
                                 -> wind/out/ref/ID.pkl   {entry, rows: [(t, rpm, bands)]}
  wfeat.py pub [PAR]             the public recordings (spectral/out/pub) -> wind/out/pub/ID.pkl
  wfeat.py sim LABEL... | --all  held renders of a label (renders/LABEL, wind/renders/LABEL) -> wind/out/sim/LABEL.pkl
"""
import glob
import os
import pickle
import re
import sys
import warnings

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT + "/spectral")
import numpy as np  # noqa: E402
import wn  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/realism: rdata
import rdata  # noqa: E402

FS = wn.FS
FC = 50.0 * 2 ** (np.arange(0, 53) / 6.0)          # 50 Hz .. 20.3 kHz
FC = FC[FC < 20500]
EDGE_LO, EDGE_HI = FC / 2 ** (1 / 12), FC * 2 ** (1 / 12)
RENDERS = [rdata.DATA + "/renders", rdata.DATA + "/wind/renders"]


def window_bands(seg):
    """dict(P, F: energies per 1/6-octave band re the window's total; tot: its mean square) or None."""
    n = 8192 if len(seg) >= 8192 + 2048 else 4096
    if len(seg) < n:
        return None
    t, f, P, F = wn.frame_split(seg, n=n, hop=n // 4)
    Pm, Fm = P.mean(axis=0), F.mean(axis=0)
    k = (f >= 30.0) & (f <= 20000.0)
    tot = Pm[k].sum() + 1e-30
    cs_p, cs_f = np.concatenate([[0], np.cumsum(Pm)]), np.concatenate([[0], np.cumsum(Fm)])
    df = f[1] - f[0]

    def band(cs):
        # energy between band edges, bins split linearly (a 1/6-octave band at 50 Hz is one bin wide)
        lo, hi = EDGE_LO / df + 0.5, EDGE_HI / df + 0.5
        return np.interp(hi, np.arange(len(cs)), cs) - np.interp(lo, np.arange(len(cs)), cs)

    return dict(P=(band(cs_p) / tot).astype(np.float32), F=(band(cs_f) / tot).astype(np.float32), tot=float(tot))


def run_ref(args):
    id_, kind = args
    import sp_core as C
    import sp_feat as Fe
    src = f"{rdata.DATA}/spectral/out/{kind}/{id_}.pkl"
    out = f"{rdata.DATA}/wind/out/{kind}/{id_}.pkl"
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if os.path.exists(out):
        return f"have {id_}"
    d = pickle.load(open(src, "rb"))
    e = d["entry"]
    rows = [r for r in d["rows"] if "win" in r[2] and Fe.gate(r[2], e, r[1], Fe.window_features(r[2], r[1], e))]
    if len(rows) < 6:
        pickle.dump(dict(entry=e, rows=[]), open(out, "wb"))
        return f"thin {id_}: {len(rows)} accepted pull windows"
    try:
        x = C.decode(e["path"], e.get("channel"))
    except Exception as ex:
        return f"FAILED {id_}: {str(ex)[:100]}"
    res = []
    for tc, rc, st in rows:
        a, b = st["win"][0], st["win"][1]
        seg = x[int(a * FS):int(b * FS)]
        wb = window_bands(seg)
        if wb is not None:
            res.append((tc, rc, wb))
    pickle.dump(dict(entry=e, rows=res), open(out, "wb"))
    return f"ok {id_}: {len(res)} windows"


def render_files(label, pattern):
    out = {}
    for r in RENDERS:
        for p in glob.glob(f"{r}/{label}/{pattern}"):
            out.setdefault(os.path.basename(p), p)
    return [out[k] for k in sorted(out)]


def run_sim(label, force=False):
    import sp_sim
    try:
        e = sp_sim.spec(label)
    except KeyError:
        return f"skip {label}"
    out = f"{rdata.DATA}/wind/out/sim/{label}.pkl"
    os.makedirs(os.path.dirname(out), exist_ok=True)
    files = render_files(label, "held_*_*_*.wav")
    key = tuple(os.path.basename(p) for p in files)
    if os.path.exists(out) and not force and pickle.load(open(out, "rb")).get("key") == key:
        return f"have {label}"
    if not files:
        return f"no renders {label}"
    rows, part = [], {}
    for p in files:
        m = re.search(r"held_(\d+)_([0-9.]+)_", p)
        rpm, load = float(m.group(1)), float(m.group(2))
        x = wn.A.load(p)[FS:]
        n = int((32 * 60.0 / rpm + 0.07) * FS)
        n = max(n, 8192 + 2048)
        w = []
        for i in range(0, len(x) - n, max(n // 2, 1)):
            wb = window_bands(x[i:i + n])
            if wb is not None:
                w.append((i / FS + 1.0, rpm, wb))
        if load >= 0.999:
            rows += w
        else:
            part.setdefault(load, []).extend(w)
    pickle.dump(dict(entry=e, rows=rows, part=part, key=key, label=label), open(out, "wb"))
    return f"ok {label}: {len(rows)} full-load windows"


if __name__ == "__main__":
    from concurrent.futures import ProcessPoolExecutor
    cmd = sys.argv[1]
    if cmd in ("refs", "pub"):
        kind = "ref" if cmd == "refs" else "pub"
        ids = sorted(os.path.basename(p)[:-4] for p in glob.glob(f"{rdata.DATA}/spectral/out/{kind}/*.pkl"))
        par = int(sys.argv[2]) if len(sys.argv) > 2 else 4
        with ProcessPoolExecutor(par) as ex:
            for m in ex.map(run_ref, [(i, kind) for i in ids]):
                print(m, flush=True)
    elif cmd == "sim":
        labs = sys.argv[2:]
        force = "--force" in labs
        labs = [l for l in labs if l != "--force"]
        if labs and labs[0] == "--all":
            labs = sorted({os.path.basename(d.rstrip("/")) for r in RENDERS for d in glob.glob(f"{r}/*/")})
        with ProcessPoolExecutor(4) as ex:
            for m in ex.map(run_sim, labs, [force] * len(labs)):
                if not m.startswith("skip"):
                    print(m, flush=True)
