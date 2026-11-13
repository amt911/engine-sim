"""Features of what the ear named (2026-11-13), measured the same way on real recordings and renders.

  M1 bank   firing series over the bank (half-firing) orders, 1-2 kHz, dB    'sintético, faltan armónicos'
  M3 wind   non-repeating sound at 150-500 Hz re the firing line, dB          'más viento'
  M2 lift   treble (2-8 kHz) change minus 0.5-1 kHz change at each lift, dB   'soplido al soltar'
  M4 peak   sample peak over the median loudness of the loud half, dB         'callado' (a transient sets the
            peak a clip is normalised to)

M1 and M3 come from phase-locked order tracking (ear_track): the signal is resampled on the engine
cycle, blocks of 8 cycles are averaged, and the averaged (repeating) part is split into firing lines,
bank lines and other lines; what does not repeat is the noise.
"""
import numpy as np
import soundfile as sf
from scipy import signal

import ear_track as T

S = 2400                     # samples per engine cycle (divisible by 2, 3, 4, 5, 6, 10, 12 firings)
B = 8                        # cycles per averaged block


def load(path, t0=None, t1=None):
    x, fs = sf.read(path, dtype="float64", always_2d=True)
    x = x[:, 0]
    if t0 is not None:
        x = x[int(t0 * fs):int(t1 * fs)]
    return x - x.mean(), fs


def win_feats(x, fs, tr, rr, fpc, a, b):
    """M1/M3 features of one window [a, b] s of x, the firing speed tracked by (tr, rr); fpc firings per
    cycle. None when the lock is poor or the window too short."""
    seg = x[int(a * fs):int(b * fs)]
    if len(seg) < fs * 0.1:
        return None
    ph, mag = T.fine_phase(seg, fs, tr - a, rr, fpc)
    m = mag[int(0.02 * fs):-int(0.02 * fs)]
    if len(m) < 10 or m.mean() <= 0:
        return None
    lockcv = float(m.std() / m.mean())
    y, rpm_c = T.angle(seg, fs, ph, fpc, S)
    n = len(y) // S
    if n < B:
        return None
    rpm = float(np.median(rpm_c))
    hzo = rpm / 120.0
    pw, tot, ap = None, {}, {}
    k = None
    for i in range(0, n - B + 1, B):
        blk = y[i * S:(i + B) * S].reshape(B, S)
        X = np.fft.rfft(blk, axis=1)
        k = np.arange(X.shape[1])
        mc = X.mean(0)
        noise = (np.abs(X - mc) ** 2).sum(0) / (B - 1) / B          # noise power left in the mean
        p = np.maximum(np.abs(mc) ** 2 - noise, 0)
        pw = p if pw is None else pw + p
        for lo, hi in ((150, 500),):
            sel = (k * hzo >= lo) & (k * hzo < hi)
            t = (np.abs(X[:, sel]) ** 2).sum() / B
            tot[lo] = tot.get(lo, 0.0) + t
            ap[lo] = ap.get(lo, 0.0) + max(t - (np.abs(mc[sel]) ** 2).sum(), 0) * B / (B - 1)
    h1 = pw[fpc] + 1e-30
    half = fpc // 2 if fpc % 2 == 0 else None
    out = dict(rpm=rpm, lockcv=lockcv)
    sel = (k * hzo >= 1000) & (k * hzo < 2000) & (k > 0)
    fi = pw[sel & (k % fpc == 0)].sum()
    hb = pw[sel & (k % fpc == half)].sum() if half else np.nan
    out["bank"] = float(10 * np.log10(fi / (hb + 1e-30) + 1e-12)) if half else np.nan
    out["wind"] = float(10 * np.log10(ap[150] / h1 + 1e-12))
    return out


def ref_rows(entry, cache):
    """M1/M3 rows of a real recording: the spectral tier's pull windows (cache = its ref pickle)."""
    if not entry.get("even") or entry["fpr"] < 1:
        return []
    fpc = int(round(2 * entry["fpr"]))
    x, fs = load(entry["path"])
    rows = []
    for _, _, st in cache["rows"]:
        if st.get("regime") != "pull":
            continue
        try:
            f = win_feats(x, fs, np.asarray(cache["t"]), np.asarray(cache["rpm"]), fpc, st["win"][0], st["win"][1])
        except Exception:  # noqa: BLE001 - a window the tracker cannot follow is skipped, not fatal
            f = None
        if f:
            rows.append(f)
    return rows


def sim_rows(path, rpm, fpc, t0=1.0, t1=4.0, wcyc=16):
    """M1/M3 rows of a held render at a known speed."""
    x, fs = load(path)
    tr = np.arange(0, len(x) / fs, 0.05)
    rr = np.full(len(tr), float(rpm))
    span = wcyc * 120.0 / rpm + 0.06
    rows, a = [], t0
    while a + span <= min(t1, len(x) / fs):
        f = win_feats(x, fs, tr, rr, fpc, a, a + span)
        if f:
            rows.append(f)
        a += span / 2
    return rows


def _env(x, fs, lo, hi, hop):
    sos = signal.butter(4, [lo, min(hi, 0.45 * fs)], btype="band", fs=fs, output="sos")
    y = signal.sosfiltfilt(sos, x) ** 2
    n = int(hop * fs)
    return 10 * np.log10(np.array([y[i:i + n].mean() for i in range(0, len(y) - n, n)]) + 1e-20)


LIFT_BANDS = [(150, 500), (500, 1000), (1000, 2000), (2000, 4000), (4000, 8000)]


def lifts(x, fs, hop=0.02):
    """M2 at every throttle lift: the level (0.5-8 kHz) falls 8 dB within 0.15 s; the change of each band
    over the 0.3 s after it against the 0.5 s of pull before. [(t, treble minus 0.5-1 kHz change)]."""
    E = {b: _env(x, fs, *b, hop) for b in LIFT_BANDS}
    tot = 10 * np.log10(sum(10 ** (E[b] / 10) for b in LIFT_BANDS[1:]))
    out, i = [], int(0.7 / hop)
    while i < len(tot) - int(0.4 / hop):
        pre = np.median(tot[i - int(0.6 / hop):i - int(0.1 / hop)])
        if tot[i + int(0.15 / hop)] < pre - 8 and tot[i] > pre - 3:
            a0, a1, b0, b1 = i - int(0.6 / hop), i - int(0.1 / hop), i + int(0.05 / hop), i + int(0.35 / hop)
            d = [np.mean(E[b][b0:b1]) - np.median(E[b][a0:a1]) for b in LIFT_BANDS]
            out.append((i * hop, float(0.5 * (d[3] + d[4]) - d[1])))
            i += int(1.2 / hop)
        else:
            i += 1
    return out


def peak_over_loudness(x, fs):
    """M4: sample peak over the median short-term loudness (K-weighted, 0.4 s) of the loud half, dB."""
    if fs != 48000:
        x = signal.resample_poly(x, 48000, fs)
        fs = 48000
    b1, a1 = [1.53512485958697, -2.69169618940638, 1.19839281085285], [1, -1.69065929318241, 0.73248077421585]
    b2, a2 = [1.0, -2.0, 1.0], [1, -1.99004745483398, 0.99007225036621]
    y = signal.lfilter(b2, a2, signal.lfilter(b1, a1, x))
    n, h = int(0.4 * fs), int(0.1 * fs)
    L = np.array([-0.691 + 10 * np.log10(np.mean(y[i:i + n] ** 2) + 1e-20) for i in range(0, len(x) - n, h)])
    if not len(L):
        return np.nan
    loud = L >= np.percentile(L, 50)
    return float(20 * np.log10(np.abs(x).max() + 1e-20) - np.median(L[loud]))
