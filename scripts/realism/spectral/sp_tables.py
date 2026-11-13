"""sp_tables.py — the feature tables of every recording and render: spectral/out/tables.pkl
{'ref': {id: (entry, {bin: {feature: median}}, extra)}, 'pub': the same for the public recordings, 'sim': {label: ...}}. Rebuilt when the
window statistics or the feature definitions are newer than the cache."""
import glob
import os
import pickle
import sys
import warnings

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sp_feat as Fe  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/realism: rdata
import rdata  # noqa: E402

CACHE = rdata.DATA + "/spectral/out/tables.pkl"


WIND = os.path.join(rdata.DATA, "wind", "out")


def _bands(kind, name):
    """{(round(t, 3), round(rpm, 1)): band spectra} of the windows of a recording / render (wind/wfeat.py), or None."""
    p = f"{WIND}/{kind}/{name}.pkl"
    if not os.path.exists(p):
        return None
    return {(round(float(t), 3), round(float(rpm), 1)): wb for t, rpm, wb in pickle.load(open(p, "rb")).get("rows", [])}


def _one(p):
    d = pickle.load(open(p, "rb"))
    e = d["entry"]
    kind = "ref" if "/ref/" in p else "pub" if "/pub/" in p else "sim"
    name = os.path.basename(p)[:-4]
    wb = _bands(kind, name)
    if kind in ("ref", "pub"):
        T = Fe.table(d["rows"], e, real=True, extra=wb)
        extra = dict(seconds=d.get("seconds"), n_pull=sum(1 for r in d["rows"] if r[2]["regime"] == "pull"))
    else:
        T = Fe.table(d["rows"], e, real="sal" in d, extra=wb)
        extra = {}
    return kind, name, e, T, extra


def build(par=5, force=False):
    files = sorted(glob.glob(rdata.DATA + "/spectral/out/ref/*.pkl") + glob.glob(rdata.DATA + "/spectral/out/sim/*.pkl") + glob.glob(rdata.DATA + "/spectral/out/pub/*.pkl"))
    wind = glob.glob(WIND + "/ref/*.pkl") + glob.glob(WIND + "/sim/*.pkl") + glob.glob(WIND + "/pub/*.pkl")
    newest = max([os.path.getmtime(p) for p in files + wind] + [os.path.getmtime(Fe.__file__), os.path.getmtime(Fe.WF.__file__)])
    if os.path.exists(CACHE) and not force and os.path.getmtime(CACHE) > newest:
        return pickle.load(open(CACHE, "rb"))
    from concurrent.futures import ProcessPoolExecutor
    out = {"ref": {}, "sim": {}, "pub": {}}
    with ProcessPoolExecutor(par) as ex:
        for kind, name, e, T, extra in ex.map(_one, files):
            out[kind][name] = (e, T, extra)
    pickle.dump(out, open(CACHE, "wb"))
    return out


if __name__ == "__main__":
    t = build(force="--force" in sys.argv)
    print(len(t["ref"]), "recordings,", len(t["pub"]), "public recordings,", len(t["sim"]), "renders")
