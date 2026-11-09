#!/usr/bin/env python3
"""Score engine builds against real recordings, so a sound change shows whether it got closer to
the real engine before anyone listens to it.

Four distances, each in dB (lower is closer):
- orders: engine-order levels 1..30 (each relative to the sum of the 30) at full load, real WOT
  pulls order-tracked against a linear rpm fit per pull vs held renders at the same rpm;
- bands:  octave-band levels 125 Hz..8 kHz relative to the total, same windows;
- treble: the 2-16 kHz octaves' mean signed error, as a magnitude: the bands' average lets a
  better bass hide a treble deficit the ear hears at once (user, 2026-11-08: "le faltan agudos");
- random: share of the energy that is not repeated from one engine cycle to the next, per band
  (scripts/authenticity.py), median over 0.6 s windows of the real WOT pulls against the build's
  own full-throttle pulls through its gears (`--render gears`): the crank runs free in both. A
  held render pins the crank and drops the random walk each cycle's work spread gives it (about
  10 dB of the 250 Hz-1 kHz randomness, docs/FACTS.md), so it is not compared. Too low reads as a
  synthetic, canned sound: every cycle a copy of the last.

The renders are what the build plays (its own listener, ground reflection included).

Usage:
  scripts/realgate.py REFS.json LABEL=BIN:ENGINE.yaml [LABEL=BIN:ENGINE.yaml ...]
The first build is the baseline (the last sound the user approved); each other build PASSes when
its total is lower than the baseline's and no distance is worse by more than 0.5 dB.

REFS.json (local, the recordings are never committed):
  {"pulls": {"wav": "a.wav", "trk": "a.trk", "spans": [[3.25, 4.75]], "rpms": [7500, 8000]},
   "firing_order": 5}
  firing_order: the engine order of the firing (cylinders / 2), to follow a gears render's rpm.
  .trk lines: "<t>s <rpm>rpm ..." (the tracker in scripts/realism); WAVs at 48 kHz mono or stereo.
"""
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("authenticity", os.path.join(HERE, "authenticity.py"))
auth = importlib.util.module_from_spec(spec)
spec.loader.exec_module(auth)

SPR, NREV = 256, 16  # samples per crank revolution, revolutions per window (bin k = order k/16)
OCT = [125, 250, 500, 1000, 2000, 4000, 8000]
CACHE = os.environ.get("REALGATE_CACHE", os.path.join(HERE, "..", "target", "realgate"))


def to_angle(x, fs, t0, rpm_of_t):
    """Resample onto uniform crank angle (SPR samples per revolution)."""
    t = np.arange(len(x)) / fs + t0
    ph = np.cumsum(rpm_of_t(t) / 60 / fs)
    ph -= ph[0]
    return np.interp(np.arange(0, ph[-1], 1 / SPR), ph, x)


def windows(y):
    n = NREV * SPR
    return [np.abs(np.fft.rfft(y[i:i + n] * np.hanning(n))) ** 2 for i in range(0, len(y) - n, n // 2)]


def features(X, rpm):
    """(orders 1..30 dB re their sum, octave bands dB re total)."""
    o = np.array([X[k * NREV - 1:k * NREV + 2].sum() for k in range(1, 31)])
    hz = np.arange(len(X)) / NREV * rpm / 60
    b = np.array([X[(hz >= lo) & (hz < 2 * lo)].sum() for lo in OCT])
    return 10 * np.log10(o / o.sum() + 1e-30), 10 * np.log10(b / b.sum() + 1e-30)


def real_pulls(ref):
    fs, x = auth.load(ref["wav"])
    if x.ndim > 1:
        x = x.mean(axis=1)
    rows = [line.split() for line in open(ref["trk"]) if line.strip()]
    T = np.array([float(r[0].rstrip("s")) for r in rows])
    R = np.array([float(r[1].rstrip("rpm")) for r in rows])
    got = {}
    for a, b in ref["spans"]:
        k = (T >= a) & (T <= b)
        c = np.polyfit(T[k] + 0.125, R[k], 1)
        i0, i1 = int(a * fs), int((b + 0.25) * fs)
        y = to_angle(x[i0:i1], fs, a, lambda t: np.polyval(c, t))
        ph = np.cumsum(np.polyval(c, np.arange(i0, i1) / fs) / 60 / fs)
        for j, X in enumerate(windows(y)):
            tc = np.searchsorted(ph, j * NREV // 2 + NREV / 2) / fs + a
            rpm = np.polyval(c, tc)
            near = min(ref["rpms"], key=lambda r: abs(r - rpm))
            if abs(near - rpm) <= 250:
                got.setdefault(near, []).append(features(X, rpm))
    return {r: (np.median([f[0] for f in v], axis=0), np.median([f[1] for f in v], axis=0))
            for r, v in got.items() if len(v) >= 2}


def render(binary, engine, rpm, seconds=3):
    os.makedirs(CACHE, exist_ok=True)
    h = hashlib.sha1()
    h.update(open(binary, "rb").read(1 << 20) + str(os.path.getmtime(binary)).encode())
    h.update(open(engine, "rb").read() + f"{rpm}:{seconds}".encode())
    out = os.path.join(CACHE, h.hexdigest()[:16] + ".wav")
    if not os.path.exists(out):
        subprocess.run(["nice", "-n19", binary, "--engine", engine, "--render",
                        f"held:{rpm}:1.0:{seconds}", "--out", out], check=True, capture_output=True)
    fs, x = auth.load(out)
    return fs, x[int(0.8 * fs):]


WIN = 0.6  # s, the randomness windows


def median_shares(x, fs, spans):
    """Median per-band random share over WIN windows every WIN/2 of `spans`: (t0, t1, rpm at t0,
    rpm at t1) with the rpm linear in between."""
    got = []
    for t0, t1, r0, r1 in spans:
        for t in np.arange(t0, t1 - WIN, WIN / 2):
            rpm = r0 + (r1 - r0) * (t + WIN / 2 - t0) / (t1 - t0)
            try:
                got.append(auth.measure(x[int(t * fs):int((t + WIN) * fs)], fs, rpm, width=0.06)["shares"])
            except Exception:  # a window the tracker cannot follow is left out
                pass
    return np.median(got, axis=0) if got else None


def real_pull_spans(ref):
    rows = [line.split() for line in open(ref["trk"]) if line.strip()]
    T = np.array([float(r[0].rstrip("s")) for r in rows])
    R = np.array([float(r[1].rstrip("rpm")) for r in rows])
    spans = []
    for a, b in ref["spans"]:
        k = (T >= a) & (T <= b)
        c = np.polyfit(T[k] + 0.125, R[k], 1)
        spans.append((a, b + 0.25, np.polyval(c, a), np.polyval(c, b + 0.25)))
    return spans


def gear_pull_spans(x, fs, order, lo, hi):
    """Stretches of a gears render rising through lo..hi rpm: its firing line followed every
    0.1 s (the strongest peak within 40 % of the last), cut at each shift."""
    hop, n = 0.1, int(0.25 * fs)
    t, rpm = [], []
    for i in range(0, len(x) - n, int(hop * fs)):
        X = np.abs(np.fft.rfft(x[i:i + n] * np.hanning(n)))
        f = np.fft.rfftfreq(n, 1 / fs)
        k = (f > lo / 60 * order * 0.6) & (f < hi / 60 * order * 1.3)
        t.append(i / fs + 0.125)
        rpm.append(f[k][np.argmax(X[k])] * 60 / order)
    spans, start = [], None
    for j in range(1, len(t)):
        rising = lo <= rpm[j] <= hi and 0 <= rpm[j] - rpm[j - 1] < 300
        if rising and start is None:
            start = j - 1
        if (not rising or j == len(t) - 1) and start is not None:
            if t[j - 1] - t[start] >= WIN:
                spans.append((t[start], t[j - 1], rpm[start], rpm[j - 1]))
            start = None
    return spans


def render_gears(binary, engine):
    os.makedirs(CACHE, exist_ok=True)
    h = hashlib.sha1()
    h.update(open(binary, "rb").read(1 << 20) + str(os.path.getmtime(binary)).encode())
    h.update(open(engine, "rb").read() + b"gears")
    out = os.path.join(CACHE, h.hexdigest()[:16] + ".wav")
    if not os.path.exists(out):
        subprocess.run(["nice", "-n19", binary, "--engine", engine, "--render", "gears",
                        "--out", out], check=True, capture_output=True)
    return auth.load(out)


def score(binary, engine, pulls, steady):
    orders, bands, treble, rand = [], [], [], []
    for rpm, (ro, rb) in sorted(pulls.items()):
        fs, x = render(binary, engine, rpm)
        f = [features(X, rpm) for X in windows(to_angle(x, fs, 0, lambda t: np.full_like(t, rpm)))]
        mo, mb = np.median([v[0] for v in f], axis=0), np.median([v[1] for v in f], axis=0)
        orders.append(np.mean(np.clip(np.abs(mo - ro), 0, 20)))
        bands.append(np.mean(np.abs(mb - rb)))
        treble.append(abs(np.mean((mb - rb)[OCT.index(2000):])))
    if steady is not None:
        fs, x = render_gears(binary, engine)
        lo, hi = min(pulls) - 250, max(pulls) + 250
        m = median_shares(x, fs, gear_pull_spans(x, fs, steady["order"], lo, hi))
        if m is not None:
            rand.append(np.mean(np.abs(m - steady["shares"])))
    return dict(orders=float(np.mean(orders)), bands=float(np.mean(bands)),
                treble=float(np.mean(treble)),
                random=float(np.mean(rand)) if rand else 0.0)


def main(argv):
    refs = json.load(open(argv[0]))
    pulls = real_pulls(refs["pulls"]) if "pulls" in refs else {}
    steady = None
    if "firing_order" in refs and "pulls" in refs:
        fs, x = auth.load(refs["pulls"]["wav"])
        if x.ndim > 1:
            x = x.mean(axis=1)
        steady = dict(order=refs["firing_order"],
                      shares=median_shares(x, fs, real_pull_spans(refs["pulls"])))
    print(f"refs: pulls at {sorted(pulls)} rpm; randomness "
          + ("from the pulls" if steady is not None else "not scored"))
    rows = []
    for arg in argv[1:]:
        label, rest = arg.split("=", 1)
        binary, engine = rest.split(":", 1)
        rows.append((label, score(binary, engine, pulls, steady)))
    base = rows[0][1]
    ok = True
    print(f"{'build':12s} " + " ".join(f"{k:>7s}" for k in rows[0][1]) + f" {'total':>7s}")
    for i, (label, s) in enumerate(rows):
        tot = sum(s.values())
        verdict = "baseline"
        if i:
            worse = [k for k in s if s[k] > base[k] + 0.5]
            good = tot < sum(base.values()) and not worse
            verdict = "PASS" if good else "FAIL" + (f" (worse: {', '.join(worse)})" if worse else "")
            ok &= good
        print(f"{label:12s} " + " ".join(f"{v:7.2f}" for v in s.values()) + f" {tot:7.2f}  {verdict}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
