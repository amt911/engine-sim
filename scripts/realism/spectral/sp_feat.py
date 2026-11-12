"""Spectral tier — features of one window from its raw statistics (sp_core.window_stats), and the
tables per recording (speed bins of the pull).

Three kinds, by what a microphone can and cannot change:
  order structure   which engine orders carry the tone (ratios between lines a few hundred Hz apart,
                    or summed over the pull): changed little by the microphone
  texture           how periodic the sound is band by band (lines against the floor between them at
                    the same frequency), how the treble envelope rides the engine cycle: unchanged by
                    any linear filter (microphone, distance, equaliser), changed by added noise
  timbre            band levels and sharpness: the microphone's response is in them; only recordings
                    of one microphone class, or of the engine itself, can judge them
"""
import numpy as np

import os
import sys

import sp_core as C

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "wind"))
import wf as WF  # noqa: E402  (speed-free features: hump of the top octaves, floor shares)

WF_NAMES = ["rf28", "rf416", "rf816", "p816", "flA", "tiltF", "hfprom", "hffreq", "hfshare", "hfx"]

LN2 = np.log(2.0)
T_DET = 3.0          # a line: its window holds >= 3x the floor energy under it (false alarms ~1 % of the cells)
STEP = C.STEP
EDGES = [0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05]     # rpm / redline bins
ABANDS = [(250, 500), (500, 1000), (1000, 2000), (2000, 4000), (4000, 8000)]
EB = [700, 1400, 2800, 5600]
TOCT = np.array(C.TOCT)


def db(p):
    return 10 * np.log10(np.maximum(p, 1e-9))


def window_features(st, rpm, e):
    """{feature: value} of one window; e: engine facts (fpr, even, family)."""
    fpr = e["fpr"]
    kf = int(round(2 * fpr))                     # the firing order in half orders
    E = np.nan_to_num(st["E"].astype(float))
    F = np.nan_to_num(st["F"].astype(float))
    P = np.nan_to_num(st["P"].astype(float))
    nb = 2 * st["W"].astype(float) + 1
    K = len(E)
    k = np.arange(1, K + 1)
    fk = k * rpm / 120.0
    under = nb * F / LN2                         # floor energy under the line window
    det = (E > T_DET * under) & (E > 0)
    Ln = np.where(det, E - under, 0.0)           # line energy
    Nn = STEP * F / LN2                          # floor energy of the cell (one half order wide)
    Ct = E + (STEP - nb) * F / LN2               # all the energy of the cell
    valid = k >= st["k_lo"]
    is_f = (k % kf == 0)
    is_h = ((k % (kf // 2) == 0) & ~is_f) if kf % 2 == 0 else np.zeros(K, bool)
    is_o = ~(is_f | is_h)
    f1 = fpr * rpm / 60.0
    o = {}
    # ------------------------------------------------------------------ order structure
    o["tone"] = max(db(Ln[kf - 1]), -40.0) if kf <= K else np.nan         # firing line re total (floor -40: not detected)
    o["fser"] = db(Ln[is_f].sum())                                       # firing series re total
    if kf > 1 and kf <= K:
        nf = ~is_f & valid
        strongest_f = Ln[is_f].max() if is_f.any() else 0.0
        # the firing series over the strongest other LINE (a detected one: a noisy recording must not read
        # as an engine whose orders compete; with no other line at all the dominance is capped at 40 dB)
        o["dom"] = float(np.clip(db(strongest_f) - db(Ln[nf].max()), -40, 40)) if nf.any() else np.nan
        # the same with a cell that holds no detected line counted as its detection threshold: a lower
        # bound limited by the recording's own floor (how far the firing series stands over line OR noise)
        cand = np.where(det, Ln, (T_DET - 1) * under)
        o["dom3"] = float(np.clip(db(strongest_f) - db(cand[nf].max()), -40, 40)) if nf.any() else np.nan
        o["sub"] = float(np.clip(db(Ln[:kf - 1][valid[:kf - 1]].sum()) - db(max(Ln[kf - 1], under[kf - 1])), -60, 40))
        o["fund"] = float(db(Ln[kf - 1]) - db(strongest_f))              # 0 when the fundamental is the strongest harmonic
        o["nonf"] = float(np.clip(db(Ln[nf].sum()) - db(Ln[is_f].sum()), -60, 40))
        o["halfs"] = float(np.clip(db(Ln[is_h & valid].sum()) - db(Ln[is_f].sum()), -60, 40)) if is_h.any() else np.nan
        o["others"] = float(np.clip(db(Ln[is_o & valid].sum()) - db(Ln[is_f].sum()), -60, 40)) if (is_o & valid).any() else np.nan
        # the octave under the pitch: the line at half the firing order re the firing line
        o["suboct"] = float(np.clip(db(Ln[kf // 2 - 1]) - db(max(Ln[kf - 1], under[kf - 1])), -60, 40)) if kf % 2 == 0 and st["k_lo"] <= kf // 2 else np.nan
    else:
        for n in ("dom", "dom3", "sub", "fund", "nonf", "halfs", "others", "suboct"):
            o[n] = np.nan
    # firing-harmonic profile up to 4 kHz
    mf = k[is_f & (fk <= 4000.0) & valid]
    if len(mf) >= 3:
        y = db(np.maximum(Ln[mf - 1], under[mf - 1]))
        m = mf / kf
        o["hslope"] = float(np.polyfit(np.log2(m), y, 1)[0])            # dB per octave of harmonic number
        o["nharm"] = float(((y > y.max() - 20) & det[mf - 1]).sum())
        dfir = det[mf - 1]
        o["hext"] = float(np.log2(fk[mf - 1][dfir].max() / f1)) if dfir.any() else 0.0    # octaves above f1 the series reaches
    else:
        o["hslope"] = o["nharm"] = o["hext"] = np.nan
    # ------------------------------------------------------------------ texture: lines against the floor, band by band
    for lo, hi in ABANDS:
        b = (fk >= lo) & (fk < hi) & valid
        nm = f"{lo}"
        if b.sum() >= 4:
            o["per" + nm] = float(np.clip(db(Ln[b].sum() + 1e-9) - db(Nn[b].sum() + 1e-9), -30, 40))     # periodic over random
            o["lin" + nm] = float(db(Ln[b].sum()))
            o["flo" + nm] = float(db(Nn[b].sum()))
            bn = b & ~is_f
            if kf > 1 and bn.sum() >= 4:
                o["for" + nm] = float(100.0 * det[bn].mean())            # % of the non-firing half orders that hold a line
                o["pnf" + nm] = float(np.clip(db(Ln[bn].sum() + 1e-9) - db(Nn[b].sum() + 1e-9), -30, 40))
            else:
                o["for" + nm] = o["pnf" + nm] = np.nan
            d = det & b & is_f
            o["conc" + nm] = float(np.median(P[d] / E[d])) if d.sum() >= 1 else np.nan      # how sharp the firing lines are
        else:
            for p in ("per", "lin", "flo", "for", "pnf", "conc"):
                o[p + nm] = np.nan
    # ------------------------------------------------------------------ timbre
    # octave bands relative to the firing frequency, re the firing band
    def oband(j):
        b = (k / kf >= 2 ** (j - 0.5)) & (k / kf < 2 ** (j + 0.5)) & valid
        return Ct[b].sum() if b.any() and f1 * 2 ** (j + 0.5) <= 1.15 * C.F_HI else np.nan     # the band must lie under 8 kHz
    b0 = oband(0)
    for j in (1, 2, 3, 4):
        v = oband(j)
        o[f"up{j}"] = float(db(v) - db(b0)) if np.isfinite(v) and np.isfinite(b0) else np.nan
    low = (k / kf < 2 ** -0.5) & valid
    o["low"] = float(db(Ct[low].sum()) - db(b0)) if low.any() and np.isfinite(b0) else np.nan
    t = 10 ** (st["toct"].astype(float) / 10)
    def tb(a, b_):
        return t[(TOCT >= a * 0.95) & (TOCT <= b_ * 1.05)].sum()
    o["b_lo"] = float(db(tb(31, 200)))            # shares of the total (30 Hz - 8 kHz)
    o["b_mid"] = float(db(tb(250, 800)))
    o["b_1k"] = float(db(tb(1000, 1600)))
    o["b_2k"] = float(db(tb(2000, 3200)))
    o["b_4k"] = float(db(tb(4000, 6400)))
    o["tr_mid"] = float(db(tb(2000, 6400)) - db(tb(250, 1600)))          # treble re mids
    o["air"] = float(db(tb(4000, 6400)) - db(tb(1000, 3200)))
    o["sharp"] = float(st.get("sharp", np.nan))
    # ------------------------------------------------------------------ texture: the treble envelope and the engine cycle
    for lo in EB:
        ev = st["env"].get(lo)
        nm = f"{lo}"
        if ev is None:
            for p in ("mF", "mD", "mR", "mX", "crest", "esh"):
                o[p + nm] = np.nan
            continue
        ln = ev["lines"].astype(float)
        o["mF" + nm] = float(100 * ln[kf - 1]) if kf <= len(ln) else np.nan          # % at the firing order
        top = min(len(ln), max(3 * kf, 6))
        dl = ln[:top][ln[:top] > 3.0 * ev["floor"]]
        o["mD" + nm] = float(100 * np.sqrt((dl ** 2).sum()))                          # % deterministic (all engine orders up to 3x firing)
        o["mR" + nm] = float(100 * ev["floor"])                                       # random fluctuation per bin, %
        o["mX" + nm] = float(100 * ln[:top].max())
        o["crest" + nm] = float(ev["crest"])
        o["esh" + nm] = float(ev["share"])
    # ------------------------------------------------------------------ noise: the broadband floor's share (wind, hiss)
    # floor energy between the engine orders in 2-8 kHz (and 1-8 kHz) re the window's total: what is heard
    # up there that is not the engine's own lines
    for nm, lo, hi in (("nz28", 2000.0, 8000.0), ("nz18", 1000.0, 8000.0), ("nzlo", 150.0, 2000.0)):
        b = (fk >= lo) & (fk < hi) & valid
        o[nm] = float(np.clip(db(Nn[b].sum() + 1e-9), -60, 0)) if b.sum() >= 4 else np.nan
    # speed-free spectra of the window up to 20 kHz (wind/wfeat.py): the hump in the top octaves, etc.
    wb = st.get("wb")
    if wb is not None:
        o.update(WF.features(wb))
    else:
        for nm in WF_NAMES:
            o[nm] = np.nan
    o["og"] = float(db(st.get("offgrid", 0.0) + 1e-9) - db(Ln.sum() + 1e-9))        # off-grid lines re the engine's lines
    o["nlines"] = float(det.sum())
    # ------------------------------------------------------------------ is the speed an octave low? (even-firing engines)
    # At half the true speed the odd "firing harmonics" are the engine's half-firing orders (weak) and the
    # even ones its firing harmonics (strong); and no line sits on the believed half-firing positions.
    o["alt"] = o["hp"] = o["evod"] = np.nan
    if kf > 1:
        lv = db(np.where(det, Ln, 2 * under))
        m = np.arange(1, 9)
        idx = m * kf - 1
        ok = (idx < K) & (m * f1 <= 5000.0) & (idx + 1 >= st["k_lo"])
        odd, ev = lv[idx[ok & (m % 2 == 1)]], lv[idx[ok & (m % 2 == 0)]]
        n = min(len(odd), len(ev))
        if n >= 2:
            o["alt"] = float(np.mean(ev[:n] - odd[:n]))
            pw = np.where(det, Ln, 0.0)
            o["evod"] = float(db(pw[idx[ok & (m % 2 == 0)]].sum() + 1e-9) - db(pw[idx[ok & (m % 2 == 1)]].sum() + 1e-9))   # power of the even over the odd firing harmonics
        if kf % 2 == 0:
            h = np.arange(1, 16, 2) * (kf // 2) - 1
            h = h[(h < K) & (h + 1 >= st["k_lo"])]
            if len(h):
                o["hp"] = float(det[h].mean())
    return o


OG_MAX = -6.0        # off-grid lines at most a quarter of the engine's line energy
SUB_MAX = 3.0        # dB: the orders under the firing order, summed, re the firing line
EVOD_MAX = 3.0       # dB: even over odd firing harmonics, in the lower octave of the speed range
LOW_OCT = 0.56       # under this share of the redline a window could be the upper octave tracked an octave low
                     # (half of 1.03 x redline, plus 8 % for a redline that is higher than the manifest says)


def gate(st, e, rpm=None, f=None):
    """Real-recording window gate: a pull, the comb salient, a loud passage; with the window's features
    f also: the lines on the order grid, and (even-firing engines, lower octave of the speed range) the
    harmonics not alternating the way an octave-low speed makes them."""
    if st["regime"] != "pull" or st["sal"] < 8.0 or st["lev_rel"] < -15.0:
        return False
    if f is None:
        return True
    if f["og"] > OG_MAX:
        return False
    if e["even"] and e["fpr"] >= 1:
        # an order under the firing order louder than the firing line: the pitch is not the firing
        # order -> the speed is a multiple of the true one (a truck idling under the tracker's range,
        # the upper octave of a clean engine). This selects on `sub`: real windows cannot exceed
        # SUB_MAX by construction, and a build that does fails the same rule (PRIOR in sp_stat).
        if np.isfinite(f["sub"]) and f["sub"] > SUB_MAX:
            return False
        # lower octave of the speed range and the even firing harmonics carrying the power (fundamental,
        # third, fifth all weak): a pulse train at twice the rate, i.e. the upper octave tracked an octave
        # low (an engine held on its limiter does this, with its weak half orders filling the gaps). A real
        # engine at this speed with such harmonics cannot be told apart, so the window is not a reference.
        if rpm is not None and rpm < LOW_OCT * e["redline"]:
            if np.isfinite(f["evod"]) and f["evod"] > EVOD_MAX:
                return False
    return True


MIN_N = 6            # windows a speed bin needs (medians of 3-5 windows were where the tracker's glitches lived)


def table(rows, e, real=False, min_n=MIN_N, extra=None):
    """{bin: {feature: median over the windows}} + {'n': windows}. real=True applies the window gate.
    extra: {(round(t, 3), round(rpm, 1)): band spectra of that window} from wind/wfeat.py (the speed-free features)."""
    acc = {}
    for t, rpm, st in rows:
        if real and not gate(st, e):
            continue
        if extra is not None and "wb" not in st:
            wb = extra.get((round(float(t), 3), round(float(rpm), 1)))
            if wb is not None:
                st = dict(st, wb=wb)
        fr = rpm / e["redline"]
        i = int(np.searchsorted(EDGES, fr, side="right")) - 1
        if 0 <= i < len(EDGES) - 1:
            f = window_features(st, rpm, e)
            if real and not gate(st, e, rpm, f):
                continue
            acc.setdefault(i, []).append(f)
    out = {}
    for i, lst in acc.items():
        if len(lst) >= min_n:
            with np.errstate(all="ignore"):
                out[i] = {kk: float(np.nanmedian([d[kk] for d in lst])) if np.isfinite([d[kk] for d in lst]).any() else np.nan for kk in lst[0]}
            out[i]["n"] = len(lst)
    return out
