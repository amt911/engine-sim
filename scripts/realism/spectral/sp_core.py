"""Spectral tier — signal layer.

decode()        any audio/video file -> mono float64 at 48 kHz (ffmpeg pipe, nothing written to disk)
window_stats()  one analysis window (32 crank revolutions resampled to crank angle) -> the raw
                statistics every feature of the tier is derived from:
                  E, F, P   per half engine order k = 1..K (up to 8 kHz): energy in the line window,
                            floor per bin between the orders, peak bin (all re the window's total)
                  toct      third-octave levels (dB re total) from the time-domain spectrum
                  env       per treble band: envelope spectrum at the half orders (amplitude / mean)
                            and its random floor, crest factor
                  sharp     DIN 45692 sharpness of the window at 80 dB SPL
autofocus()     refines the speed of a tracked window (scale and slope) by maximising the
                concentration of the comb lines, so that a line at order 40 of a real pull is not
                smeared by the tracker's 0.4 % grid: the real side gets the best lines it can have.

Everything is a ratio inside one window: microphone gain, automatic gain and distance drop out.
"""
import os
import subprocess
import sys

import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import rg_audio as A  # noqa: E402  (the previous prototype's reader and comb tracker)

FS = A.FS
NREV = 32                 # revolutions per window: order resolution 1/32
STEP = NREV // 2          # bins between two half orders
F_HI = 8000.0             # codec cut-offs start above
DELTA = 0.002             # residual speed error after autofocus the line window allows for (+-0.2 %)
TOCT = [25 * 2 ** (i / 3.0) for i in range(27)]           # third-octave centres 25 Hz .. 10 kHz
EBANDS = [(700, 1400), (1400, 2800), (2800, 5600), (5600, 10000)]   # envelope bands (octaves)
KENV = 48                 # half orders of the envelope spectrum kept
_sos = {}


def decode(path, channel=None):
    """Mono float64 at 48 kHz. channel=None: the file's first channel (a stereo pair averaged combs
    a moving source; one microphone is one microphone)."""
    ch = 0 if channel is None else channel
    cmd = ["ffmpeg", "-v", "error", "-nostdin", "-i", path, "-af", f"pan=mono|c0=c{ch}", "-ar", str(FS), "-f", "f32le", "-"]
    raw = subprocess.run(cmd, capture_output=True, check=True).stdout
    x = np.frombuffer(raw, "<f4").astype(np.float64)
    return x - x.mean()


def sos(kind, *f):
    key = (kind,) + f
    if key not in _sos:
        if kind == "lp":
            _sos[key] = signal.butter(8, f[0], btype="low", fs=FS, output="sos")
        else:
            _sos[key] = signal.butter(6, [f[0], min(f[1], 0.45 * FS)], btype="band", fs=FS, output="sos")
    return _sos[key]


def spr_for(rpm):
    """Samples per revolution: the angle-domain sampling rate stays above 30 kHz."""
    for spr in (512, 1024, 2048, 4096, 8192):
        if spr * rpm / 60.0 >= 30000.0:
            return spr
    return 8192


def to_angle(seg, rpm_t, spr):
    """seg (already low-passed) on a uniform crank-angle grid; rpm_t: speed at every sample."""
    ph = np.concatenate([[0.0], np.cumsum(rpm_t[:-1] / 60.0 / FS)])
    grid = np.arange(0, ph[-1], 1.0 / spr)
    return np.interp(grid, ph, seg)


def order_spectrum(seg_lp, rpm_t, spr):
    y = to_angle(seg_lp, rpm_t, spr)
    n = NREV * spr
    if len(y) < n:
        return None
    return np.abs(np.fft.rfft(y[:n] * np.hanning(n))) ** 2


def comb_metric(Y, comb, kmax):
    """Concentration of the comb lines: sum of the square roots of the peak bins at multiples of the
    comb order (square root: the middle orders, which smear first, count)."""
    c = np.arange(1, kmax + 1) * int(round(comb * NREV))
    c = c[c + 2 < len(Y)]
    pk = np.maximum(np.maximum(Y[c - 1], Y[c]), Y[c + 1])
    return float(np.sqrt(pk).sum())


def autofocus(seg_lp, rpm_t, comb, rpm_c):
    """(scale, slope, gain): rpm_t * (1 + scale) * (1 + slope * (t - T/2)) maximising comb_metric.
    gain = metric / metric of the untouched track."""
    spr = 256 if 256 * rpm_c / 60.0 >= 9000 else 512 if 512 * rpm_c / 60.0 >= 9000 else spr_for(rpm_c)
    f1 = comb * rpm_c / 60.0
    kmax = int(max(3, min(4000.0 / f1, 0.45 * spr / comb, 24)))
    t = np.arange(len(seg_lp)) / FS
    tc = t - t[-1] / 2

    def met(s, q):
        Y = order_spectrum(seg_lp, rpm_t * (1 + s) * (1 + q * tc), spr)
        return -1.0 if Y is None else comb_metric(Y, comb, kmax)

    m0 = met(0.0, 0.0)
    if m0 <= 0:
        return 0.0, 0.0, 1.0
    best = (m0, 0.0, 0.0)
    for s in np.arange(-0.006, 0.0061, 0.001):
        m = met(s, 0.0)
        if m > best[0]:
            best = (m, s, 0.0)
    for q in (-0.012, -0.006, 0.006, 0.012):          # relative speed change per second on top of the track's
        m = met(best[1], q)
        if m > best[0]:
            best = (m, best[1], q)
    for s in (best[1] - 0.0005, best[1] + 0.0005):
        m = met(s, best[2])
        if m > best[0]:
            best = (m, s, best[2])
    return best[1], best[2], best[0] / m0


def sharpness_din(P, f):
    from mosqito.sq_metrics import loudness_zwst_freq, sharpness_din_from_loudness
    N, N_spec, _ = loudness_zwst_freq(np.sqrt(P), f)
    return float(sharpness_din_from_loudness(N, N_spec, weighting="din")), float(N)


def window_stats(seg, rpm_t, rpm_c, want_sharp=True):
    """Raw statistics of one window. seg: samples covering >= NREV revolutions; rpm_t: speed per sample."""
    out = {}
    f, P = signal.welch(seg, FS, window="hann", nperseg=min(8192, len(seg)), noverlap=min(8192, len(seg)) // 2, scaling="spectrum")
    band = (f >= 30.0) & (f < F_HI)
    tot = P[band].sum() + 1e-30
    out["toct"] = np.array([10 * np.log10(P[(f >= c / 2 ** (1 / 6)) & (f < c * 2 ** (1 / 6))].sum() / tot + 1e-30) for c in TOCT], dtype=np.float32)
    out["level"] = float(10 * np.log10(tot))
    if want_sharp:
        try:
            Pn = P * (0.2 ** 2) / (P[(f >= 20) & (f < 16000)].sum() + 1e-30)     # the window played at 80 dB SPL
            out["sharp"], out["sone"] = sharpness_din(Pn, f)
        except Exception:
            out["sharp"], out["sone"] = np.nan, np.nan
    # ---- order spectrum
    spr = spr_for(rpm_c)
    seg_lp = signal.sosfiltfilt(sos("lp", 12000.0), seg)
    Y = order_spectrum(seg_lp, rpm_t, spr)
    if Y is None:
        return None
    hz_per_bin = rpm_c / 60.0 / NREV
    K = int(min(F_HI / hz_per_bin / STEP, (len(Y) - 2 * STEP) // STEP))
    k_lo = max(1, int(np.ceil(30.0 / hz_per_bin / STEP)))      # nothing under 30 Hz
    bins = (np.arange(len(Y)) * hz_per_bin >= 30.0) & (np.arange(len(Y)) * hz_per_bin < F_HI)
    tot_o = Y[bins].sum() + 1e-30
    E = np.full(K, np.nan, dtype=np.float32)
    F = np.full(K, np.nan, dtype=np.float32)
    Pk = np.full(K, np.nan, dtype=np.float32)
    W = np.zeros(K, dtype=np.int8)
    off = 0.0
    for k in range(k_lo, K + 1):
        c = k * STEP
        w = int(min(6, max(2, np.ceil(DELTA * c))))
        lo_gap = Y[c - STEP + w + 1:c - w]         # bins between this line window and the previous one
        hi_gap = Y[c + w + 1:c + STEP - w]
        fl = np.median(hi_gap if k == k_lo else np.concatenate([lo_gap, hi_gap]))
        E[k - 1] = Y[c - w:c + w + 1].sum() / tot_o
        Pk[k - 1] = Y[c - w:c + w + 1].max() / tot_o
        F[k - 1] = fl / tot_o
        W[k - 1] = w
        # narrow peaks between the orders (10 dB over the mean floor there): lines that are not engine
        # orders at this speed -> another machine, or the speed is wrong (an octave high puts the real
        # half orders on quarter orders)
        m = fl / np.log(2.0)
        pk = hi_gap[hi_gap > 10.0 * m]
        off += float((pk - m).sum())
    out["E"], out["F"], out["P"], out["W"] = E, F, Pk, W
    out["offgrid"] = off / tot_o
    out["k_lo"] = k_lo
    # ---- treble envelopes at the half orders (angle domain), amplitude over the envelope's mean
    env = {}
    n = NREV * 512
    for lo, hi in EBANDS:
        xb = signal.sosfiltfilt(sos("bp", lo, hi), seg)
        e = to_angle(np.abs(signal.hilbert(xb)), rpm_t, 512)
        if len(e) < n:
            continue
        e = e[:n]
        w = np.hanning(n)
        S = 2 * np.abs(np.fft.rfft(e * w)) / w.sum() / (e.mean() + 1e-30)
        lines = np.array([S[max(1, k * STEP - 1):k * STEP + 2].max() for k in range(1, KENV + 1)], dtype=np.float32)
        gaps = np.concatenate([S[k * STEP + 4:(k + 1) * STEP - 3] for k in range(1, KENV)])
        env[lo] = dict(lines=lines, floor=float(np.median(gaps)),
                       cv=float(e.std() / (e.mean() + 1e-30)),
                       crest=float(20 * np.log10(np.abs(xb).max() / (np.sqrt(np.mean(xb ** 2)) + 1e-30))),
                       share=float(10 * np.log10(np.mean(xb ** 2) / (np.mean(seg ** 2) + 1e-30) + 1e-30)))
    out["env"] = env
    return out


def held_windows(x, rpm, t_skip=1.0, want_sharp=True):
    """Windows of a render held at a known speed: [(t, rpm, stats)]."""
    x = x[int(t_skip * FS):]
    n = int((NREV * 60.0 / rpm + 0.07) * FS)
    rows = []
    rpm_t = np.full(n, float(rpm))
    for i in range(0, len(x) - n, max(n // 2, 1)):
        st = window_stats(x[i:i + n], rpm_t, float(rpm), want_sharp=want_sharp)
        if st is not None:
            st.update(slew=0.0, sal=np.nan, mar=np.nan, lev_rel=0.0, regime="pull", focus=(0.0, 0.0, 1.0))
            rows.append((i / FS + t_skip, float(rpm), st))
    return rows


def _one(x, a, b, c, s, q, want_sharp):
    seg = x[int(a * FS):int(b * FS)]
    tt = np.arange(len(seg)) / FS
    rpm_t = np.polyval(c, tt) * (1 + s) * (1 + q * (tt - tt[-1] / 2))
    return seg, tt, rpm_t


def tracked_windows(x, t, rpm, sal, mar, comb, rpm_range, hop_s=0.15, sal_min=6.0, min_slew=0.02, max_jump=0.25,
                    level_floor_db=20.0, want_sharp=True, focus=True, pull_cap=500, steady_cap=120, decel_cap=80):
    """Windows along a tracked speed: [(t, rpm, stats)]. A window is a candidate when the comb is
    salient, the passage is not level_floor_db under the clip's loud passages and the speed does not
    jump inside it (gear change). Analysed: the pulls (speed rising >= min_slew per second, at most
    pull_cap, evenly thinned), and for regime contrasts at most steady_cap steady windows (|slew| < 1 %/s)
    and decel_cap falling ones (slew < -3 %/s). stats['regime'] says which."""
    nlev = int(0.1 * FS)
    lev = 10 * np.log10(np.array([np.mean(x[i:i + nlev] ** 2) for i in range(0, len(x) - nlev, nlev)]) + 1e-20)
    lev_t = (np.arange(len(lev)) + 0.5) * nlev / FS
    lev_ref = np.percentile(lev, 95)
    rs = signal.medfilt(rpm, 5)
    cand = {"pull": [], "steady": [], "decel": []}
    tc = t[0] + 0.3
    while tc < t[-1] - 0.3:
        rc = float(np.interp(tc, t, rs))
        span = NREV * 60.0 / rc
        a, b = tc - span / 2 - 0.02, tc + span / 2 + 0.05
        nxt = tc + max(hop_s, span / 2)
        if a >= max(t[0], 0) and b <= t[-1] and int(b * FS) < len(x) and rpm_range[0] <= rc <= rpm_range[1]:
            k = (t >= a) & (t <= b)
            lv = float(np.interp(tc, lev_t, lev) - lev_ref)
            if k.sum() >= 3 and np.mean(sal[k]) >= sal_min and lv >= -level_floor_db:
                rk = rs[k]
                if rk.max() / rk.min() - 1 <= max_jump:
                    c = np.polyfit(t[k] - a, rk, 1)
                    slew = float(c[0] / rc)
                    reg = "pull" if slew >= min_slew else "steady" if abs(slew) < 0.01 else "decel" if slew < -0.03 else None
                    if reg:
                        cand[reg].append(dict(tc=tc, rc=rc, a=a, b=b, c=c, slew=slew, sal=float(np.mean(sal[k])),
                                              mar=float(np.mean(mar[k])) if mar is not None else np.nan, lev_rel=lv, regime=reg))
        tc = nxt
    chosen = []
    for reg, cap in (("pull", pull_cap), ("steady", steady_cap), ("decel", decel_cap)):
        lst = cand[reg]
        if cap is not None and len(lst) > cap:
            lst = [lst[i] for i in np.unique(np.linspace(0, len(lst) - 1, cap).astype(int))]
        chosen += lst
    rows = []
    for w in sorted(chosen, key=lambda w: w["tc"]):
        seg, tt, rpm_t = _one(x, w["a"], w["b"], w["c"], 0.0, 0.0, want_sharp)
        s = q = 0.0
        g = 1.0
        if focus:
            s, q, g = autofocus(signal.sosfiltfilt(sos("lp", 12000.0), seg), rpm_t, comb, w["rc"])
            rpm_t = rpm_t * (1 + s) * (1 + q * (tt - tt[-1] / 2))
        st = window_stats(seg, rpm_t, w["rc"] * (1 + s), want_sharp=want_sharp)
        if st is not None:
            st.update(slew=w["slew"], sal=w["sal"], mar=w["mar"], lev_rel=w["lev_rel"], regime=w["regime"], focus=(s, q, g),
                      win=(w["a"], w["b"], float(w["c"][0]), float(w["c"][1]), s, q))
            rows.append((w["tc"], w["rc"] * (1 + s), st))
    return rows


def rerun(x, rows, want_sharp=True):
    """The same windows with the same speeds on another signal (a defect injected into the recording)."""
    out = []
    for tc, rc, st0 in rows:
        a, b, c0, c1, s, q = st0["win"]
        seg, tt, rpm_t = _one(x, a, b, (c0, c1), s, q, want_sharp)
        st = window_stats(seg, rpm_t, rc, want_sharp=want_sharp)
        if st is not None:
            for key in ("slew", "sal", "mar", "lev_rel", "regime", "focus", "win"):
                st[key] = st0[key]
            out.append((tc, rc, st))
    return out
