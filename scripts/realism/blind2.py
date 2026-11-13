"""blind2.py — cycle-to-cycle periodicity and discrete-echo features on the spectral tier's own windows
(same speed tracking and bins as spectral_gate), for the builds the spectral tier lets through.

  hnr   harmonics-to-noise ratio of the 1-8 kHz envelope between consecutive engine cycles (720 deg),
        on the crank-angle grid: r = mean correlation of cycle k with cycle k+1, HNR = 10 log10 r/(1-r)
        (Boersma 1993, Proc. IFA 17: 97-110, eq. 4). A real engine never repeats a cycle exactly.
  echo  real-cepstrum peak over its median at quefrencies 0.6-8 ms, the rahmonics of the firing
        period excluded (Bogert, Healy & Tukey 1963; Childers et al. 1977, Proc. IEEE 65(10)):
        a discrete reflection (ground, wall) shows as a cepstral peak at its delay.

usage: blind2.py gate LABEL ...       ratchet: hnr and echo per speed bin at most STEP dB over the ear-approved
                                     build of the same engine (APPROVED); exit 1 on a failure
       blind2.py selftest             sabotage a real recording (cycles copied; a 0.8 ms echo) -> both must rise
       blind2.py refs                 cache features of every outside recording -> out-blind2-real.pkl
       blind2.py judge LABEL ...      a render (renders/LABEL/held_*.wav) against the real pool per speed bin
"""
import glob
import os
import pickle
import re
import sys

import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "spectral"))
import sp_core as C  # noqa: E402
import sp_sim as SIM  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # rdata
import rdata  # noqa: E402

FS = C.FS
EDGES = [0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05]
SOS_HF = signal.butter(4, [1000, 7900], "band", fs=FS, output="sos")
SPR = 256
REAL = os.path.join(rdata.DATA, "out-blind2-real.pkl")


def feats(seg, rpm_t, fpr):
    env = np.abs(signal.hilbert(signal.sosfilt(SOS_HF, seg)))
    env = signal.sosfilt(signal.butter(2, 3000, fs=FS, output="sos"), env)
    a = C.to_angle(env, rpm_t, SPR)
    n = 2 * SPR
    k = len(a) // n
    if k < 3:
        return None
    L = int(round(12 / 360 * SPR))        # +-12 deg: a speed track's drift over one cycle
    rs = []
    for i in range(k - 1):
        c0 = a[i * n + L:(i + 1) * n - L]
        c0 = c0 - c0.mean()
        best = -1.0
        for s in range(-L, L + 1, 2):
            c1 = a[(i + 1) * n + L + s:(i + 2) * n - L + s] if (i + 2) * n - L + s <= len(a) else None
            if c1 is None or len(c1) != len(c0):
                continue
            c1 = c1 - c1.mean()
            v = float((c0 * c1).sum() / (np.sqrt((c0 ** 2).sum() * (c1 ** 2).sum()) + 1e-30))
            best = max(best, v)
        rs.append(best)
    r = float(np.clip(np.mean(rs), 1e-4, 0.9999))
    hnr = 10 * np.log10(r / (1 - r))
    w = seg * np.hanning(len(seg))
    c = np.fft.irfft(np.log(np.abs(np.fft.rfft(w)) + 1e-12))
    q = np.arange(len(c)) / FS
    m = (q > 0.0006) & (q < 0.008)   # under 0.6 ms: the spectral envelope, not a delay
    rpm = float(np.mean(rpm_t))
    P = 60.0 / (rpm * fpr) if fpr >= 1 else 120.0 / rpm   # firing period
    for per in (P, P / 2):
        kk = np.round(q / per)
        m &= ~((kk >= 1) & (np.abs(q - kk * per) < min(0.0003, 0.15 * per)))
    cq = np.abs(c[m])
    if len(cq) < 20:
        return None
    echo = 20 * np.log10(cq.max() / (np.median(cq) + 1e-12))
    return hnr, echo


def bin_of(f):
    for i in range(len(EDGES) - 1):
        if EDGES[i] <= f < EDGES[i + 1]:
            return i
    return None


def refs():
    out = {}
    for p in sorted(glob.glob(os.path.join(rdata.DATA, "spectral", "out", "ref", "*.pkl"))):
        d = pickle.load(open(p, "rb"))
        e = d["entry"]
        if e["cls"] != "out" or not os.path.exists(e["path"]):
            continue
        x = C.decode(e["path"], e.get("channel"))
        fpr = e["fpr"]
        res = []
        for t, rpm, st in d["rows"]:
            reg = st.get("regime")
            if reg not in ("pull", "steady"):
                continue
            n = int(C.NREV * 60.0 / rpm * FS)
            a = int(t * FS)
            seg = x[a:a + n]
            if len(seg) < n:
                continue
            rpm_t = np.interp(t + np.arange(n) / FS, d["t"], d["rpm"])
            f = feats(seg, rpm_t, fpr)
            if f:
                res.append((rpm / e["redline"],) + f + (1.0 if reg == "steady" else 0.0,))
        if res:
            out[e["id"]] = dict(entry=e, rows=np.array(res))
            print(e["id"], len(res), flush=True)
    pickle.dump(out, open(REAL, "wb"))


def sim_rows(label):
    e = SIM.spec(label)
    res = []
    for f in sorted(glob.glob(os.path.join(rdata.DATA, "renders", label, "held_*.wav"))):
        rpm = float(re.search(r"held_(\d+)_", f).group(1))
        x = C.A.load(f)[FS:]
        n = int(C.NREV * 60.0 / rpm * FS)
        for i in range(0, len(x) - n, n // 2):
            r = feats(x[i:i + n], np.full(n, rpm), e["fpr"])
            if r:
                res.append((rpm / e["redline"],) + r)
    return e, np.array(res)


def pool(real, feat, reg=0.0):
    """Per speed bin: the per-recording medians of the feature (outside recordings, pulls)."""
    j = {"hnr": 1, "echo": 2}[feat]
    out = {b: [] for b in range(len(EDGES) - 1)}
    for v in real.values():
        R = v["rows"]
        R = R[R[:, 3] == reg]
        for b in out:
            s = R[(R[:, 0] >= EDGES[b]) & (R[:, 0] < EDGES[b + 1]), j]
            if len(s) >= 3:
                out[b].append(float(np.median(s)))
    return out


def judge(labels):
    real = pickle.load(open(REAL, "rb"))
    P = {f: pool(real, f) for f in ("hnr", "echo")}
    for lab in labels:
        e, R = sim_rows(lab)
        line = [f"{lab:24s}"]
        worst = {}
        for f, j in (("hnr", 1), ("echo", 2)):
            for b in range(len(EDGES) - 1):
                s = R[(R[:, 0] >= EDGES[b]) & (R[:, 0] < EDGES[b + 1]), j] if len(R) else []
                if len(s) < 3 or len(P[f][b]) < 5:
                    continue
                v = float(np.median(s))
                hi = float(np.max(P[f][b]))
                over = v - hi
                if f not in worst or over > worst[f][0]:
                    worst[f] = (over, v, hi, b)
        for f in ("hnr", "echo"):
            if f in worst:
                o, v, hi, b = worst[f]
                line.append(f"{f} {v:6.2f} (real max {hi:6.2f} @ bin {b}) {'FAIL' if o > 0 else ' ok '}")
        print("  ".join(line))


APPROVED = {"lfa": ["lfa-new", "lfa-approved"], "v8-flatplane": ["v8-flatplane-new"]}   # every build approved by ear; bound = their worst
STEP = {"hnr": 1.0, "echo": 2.0}   # dB over the approved build


def per_bin(R, j):
    out = {}
    for b in range(len(EDGES) - 1):
        s = R[(R[:, 0] >= EDGES[b]) & (R[:, 0] < EDGES[b + 1]), j] if len(R) else []
        if len(s) >= 3:
            out[b] = float(np.median(s))
    return out


def approved_of(label):
    for k in sorted(APPROVED, key=len, reverse=True):
        if label.startswith(k):
            return APPROVED[k]
    return None


def gate(labels):
    real = pickle.load(open(REAL, "rb"))
    P = {f: pool(real, f) for f in ("hnr", "echo")}
    rc = 0
    for lab in labels:
        _, R = sim_rows(lab)
        ap = approved_of(lab)
        As = [sim_rows(a)[1] for a in ap] if ap else []
        if not len(R):
            print(f"{lab}: FAIL (nothing rendered)")
            rc = 1
            continue
        fails, info = [], []
        for f, j in (("hnr", 1), ("echo", 2)):
            v = per_bin(R, j)
            a = {}
            for A in As:
                for b, x in per_bin(A, j).items():
                    a[b] = max(a.get(b, -1e9), x)
            for b in sorted(v):
                if b in a and v[b] > a[b] + STEP[f]:
                    fails.append(f"{f} {v[b]:.1f} > approved {a[b]:.1f} + {STEP[f]:.0f} @ bin {b}")
            hi = [v[b] - max(P[f][b]) for b in v if len(P[f][b]) >= 5]
            if hi:
                info.append(f"{f} median {np.median(list(v.values())):.1f}, worst {max(hi):+.1f} dB over the real maximum")
        rc |= bool(fails)
        print(f"{lab}: {'FAIL' if fails else 'PASS'}" + ("" if ap else " (no ear-approved build: ratchet not judged)"))
        for x in fails:
            print("  FAIL", x)
        for x in info:
            print("  info", x)
    return rc


def selftest():
    """Both features must rise on a real recording when the defect they name is put in."""
    real = pickle.load(open(REAL, "rb"))
    v = real["fabia-exh"] if "fabia-exh" in real else next(iter(real.values()))
    e = v["entry"]
    x = C.decode(e["path"], e.get("channel"))
    i = int(np.argmax(v["rows"][:, 0]))
    rpm = 4000.0
    n = int(C.NREV * 60.0 / rpm * FS)
    seg = x[len(x) // 2:len(x) // 2 + n]
    rpm_t = np.full(n, rpm)
    cyc = int(round(120.0 / rpm * FS))
    per = np.tile(seg[:cyc], n // cyc + 1)[:n]          # every cycle the first one
    d = int(0.0008 * FS)
    echo = seg + 0.9 * np.concatenate([np.zeros(d), seg[:-d]])
    h0, e0 = feats(seg, rpm_t, e["fpr"])
    h1, _ = feats(per, rpm_t, e["fpr"])
    _, e1 = feats(echo, rpm_t, e["fpr"])
    ok = h1 > h0 + 6 and e1 > e0 + 3
    print(f"selftest on {e['id']}: hnr {h0:.1f} -> copied cycles {h1:.1f}; echo {e0:.1f} -> 0.8 ms echo {e1:.1f}: {'PASS' if ok else 'FAIL'}")
    _ = i
    return 0 if ok else 1


def bias():
    real = pickle.load(open(REAL, "rb"))
    for f in ("hnr", "echo"):
        p, q = pool(real, f, 0.0), pool(real, f, 1.0)
        print(f, "pull median/max vs steady median/max per bin:")
        for b in p:
            if p[b] and q[b]:
                print(f"  bin {b}: pull {np.median(p[b]):6.2f}/{np.max(p[b]):6.2f} (n {len(p[b])})  steady {np.median(q[b]):6.2f}/{np.max(q[b]):6.2f} (n {len(q[b])})")


if __name__ == "__main__":
    if sys.argv[1] == "gate":
        sys.exit(gate(sys.argv[2:]))
    elif sys.argv[1] == "selftest":
        sys.exit(selftest())
    elif sys.argv[1] == "bias":
        bias()
    elif sys.argv[1] == "refs":
        refs()
    else:
        judge(sys.argv[2:])
