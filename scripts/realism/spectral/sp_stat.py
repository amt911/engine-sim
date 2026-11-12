"""Spectral tier — the checks and how a build (or a recording) is judged against real recordings.

A check = one feature, one summary over the pull (its median over the speed bins, or its worst bin),
the side on which leaving the real population is a defect, and a pool of real recordings.

How far outside is "outside": a p-value read on the pool itself.
  inside the pool      p = (number of pool values at least as extreme + 1) / (n + 1)   (rank; no shape assumed)
  beyond the k-th most extreme pool value u (k = a fifth of the pool, at least 4)
                       p = (k + 1) / (n + 1) * exp(-(distance beyond u) / b), b = mean excess of the pool
                       values beyond u (peaks over threshold, exponential tail; b never under half a
                       just-noticeable step). The pool's own tail sets the scale: a feature whose real
                       values stop dead at a bound (the firing series always dominates) rejects a build
                       10 dB beyond it; a feature with a long real tail (treble modulation of singles) does not.
  worst-bin checks     the pool is every speed bin of every real recording; the build's p is its worst
                       bin's, times its number of bins (Bonferroni).
The score of a build is S = max over the checks of -log10 p. The threshold S* is not chosen: it is the
(1 - ALPHA) conformal quantile of the scores real recordings get when each is judged by the others
(leave-one-recording-out). About one real recording in ten is rejected by construction; `loo` prints it.

Signature: when the engine has outside recordings of its own, every feature is also compared with
them speed bin by speed bin; the unit is how much two recordings of ONE real engine differ (pairs of
recordings of the same engine in the library), and the threshold is what real pairs reach.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sp_feat as Fe  # noqa: E402
import sp_tables as TB  # noqa: E402

EXCLUDE = {
    "f1-ext": "trackside: several cars at once",
}
MIN_WINDOWS = 12          # accepted pull windows a recording needs to enter a pool
ALPHA = 0.10              # share of real recordings the gate may reject, split over its tiers
TIER_ALPHA = {"order": 0.05, "texture": 0.02}     # the gate (sum well under ALPHA); "watch", "trend" and "timbre" are printed with their own p, never failed
EDGE_P = 0.03             # a check rarer than this among real recordings is printed as EDGE even when it does not fail

# id, feature, summary, side, pool, classes it applies to, tier, what a listener / a mechanic would say
CHECKS = [
    ("pitch: firing series dominates", "dom", "min", "lo", "even", {"even"}, "order",
     "another engine order is as loud as the firing series at some speed: the pitch wavers or is lost ('tapado', 'suena fatal')"),
    ("pitch: firing series dominates (pull)", "dom", "med", "lo", "even", {"even"}, "order",
     "over the whole pull the firing series barely stands over the other orders: rough, no clear pitch"),
    ("orders: half-firing series", "halfs", "max", "hi", "even", {"even"}, "order",
     "odd multiples of half the firing order as strong as the firing series at some speed: bank / pair imbalance, the octave under the pitch and its odd harmonics"),
    ("orders: half-firing series (pull)", "halfs", "med", "hi", "even", {"even"}, "order",
     "half-firing orders too strong over the whole pull: sounds like half the cylinders at twice the speed"),
    ("pitch: orders under the firing order", "sub", "max", "hi", "even", {"even"}, "order",
     "orders under the firing order nearly as loud as the firing line: the pitch drops, mud"),
    ("orders: cylinder-to-cylinder", "others", "max", "hi", "even", {"even"}, "order",
     "orders that are neither firing nor half-firing multiples too strong at some speed: uneven cylinders"),
    ("pitch: fundamental", "fund", "min", "lo", "even", {"even"}, "order",
     "the firing line itself much weaker than its own harmonics at some speed: the tone thins / jumps an octave"),
    ("texture: periodic 1-2 kHz", "per1000", "med", "hi", "all", {"even", "single", "uneven"}, "texture",
     "1-2 kHz is engine-order lines where real engines are mostly noise: clean, 'synthetic', whistle"),
    ("texture: periodic 2-4 kHz", "per2000", "med", "hi", "all", {"even", "single", "uneven"}, "texture",
     "2-4 kHz is engine-order lines where real engines are noise: 'synthetic', every cycle a copy"),
    ("texture: periodic 4-8 kHz", "per4000", "med", "hi", "all", {"even", "single", "uneven"}, "texture",
     "4-8 kHz is engine-order lines where real engines are noise: 'synthetic', buzz instead of hiss"),
    ("watch: line forest 2-4 kHz", "for2000", "med", "hi", "multi", {"even", "uneven"}, "watch",
     "a line on most half orders in 2-4 kHz: cylinder-to-cylinder differences repeated identically every cycle"),
    ("watch: line forest 4-8 kHz", "for4000", "med", "hi", "multi", {"even", "uneven"}, "watch",
     "a line on most half orders in 4-8 kHz: no cycle-to-cycle variation up there"),
    ("texture: treble at the firing rate 1.4-2.8 kHz", "mF1400", "med", "hi", "class", {"even", "single", "uneven"}, "texture",
     "the 1.4-2.8 kHz envelope pulses with every firing: 'machine gun'"),
    ("texture: treble at the firing rate 2.8-5.6 kHz", "mF2800", "med", "hi", "class", {"even", "single", "uneven"}, "texture",
     "the 2.8-5.6 kHz envelope pulses with every firing: 'machine gun', crackle on every pulse"),
    ("texture: treble at the firing rate 5.6-10 kHz", "mF5600", "med", "hi", "class", {"even", "single", "uneven"}, "texture",
     "the top octave pulses with every firing: 'machine gun'"),
    ("texture: treble randomness", "mR2800", "med", "lo", "all", {"even", "single", "uneven"}, "texture",
     "the 2.8-5.6 kHz envelope has too little random fluctuation: deterministic treble, 'synthetic'"),
    ("texture: treble crest", "crest2800", "med", "hi", "class", {"even", "single", "uneven"}, "texture",
     "2.8-5.6 kHz crest factor over real engines: crackle (shock fronts radiated raw)"),
]
# trends with speed inside one recording: the microphone's fixed response cancels
TREND = [
    ("trend: firing tone against speed", "tone", "slope", "lo", "all", {"even", "single", "uneven"}, "trend",
     "the firing tone loses its share of the sound as the engine revs up; in real engines it holds or grows: the pitch thins out towards the top"),
    ("trend: harmonic slope against speed", "hslope", "slope", "both", "all", {"even", "single", "uneven"}, "trend",
     "the firing harmonics fall away faster as the speed rises (real engines keep their slope): the top end sounds far / dull, no steepening | they flatten with speed: buzzier towards the top"),
    ("trend: treble against speed", "tr_mid", "slope", "both", "all", {"even", "single", "uneven"}, "trend",
     "the treble falls behind as the engine revs up: dull top end | the treble grows faster with speed than in any real engine: thin, hissy top end"),
]
CHECKS += TREND
# timbre: the microphone is in it -> judged against the class pool for information and, with weight, in the signature
TIMBRE = [
    ("timbre: octave above the firing band", "up1", "med", "both", "class", "weak: dull pulse | strong: the second firing harmonic / the orders around it dominate (hollow, nasal)"),
    ("timbre: two octaves above", "up2", "med", "both", "class", "weak: muffled ('tapado') | strong: harsh"),
    ("timbre: three octaves above", "up3", "med", "both", "class", "weak: muffled, 'far' | strong: hiss / crackle"),
    ("timbre: treble re mids", "tr_mid", "med", "both", "class", "weak: muffled | strong: hissy, thin"),
    ("timbre: sharpness", "sharp", "med", "both", "class", "low: dull | high: shrill beyond real engines"),
    ("timbre: harmonic slope", "hslope", "med", "both", "class", "steep: only the fundamental (pure, 'mosquito') | flat: buzzy"),
]
LOGF = {"mF1400", "mF2800", "mF5600"}        # judged as 20 log10(index)
JND = {"for2000": 6.0, "for4000": 6.0, "mR2800": 0.2, "sharp": 0.1, "crest2800": 1.0, "hslope": 1.5}      # one audible step (dB features: 2 dB)
EXT_MICS = ("ext", "mix")     # an ear some metres from the car (not a microphone at the tail pipe, in the jet's near field)
UNIT = {"for2000": "%", "for4000": "%", "mR2800": "%", "sharp": "acum", "hslope": "dB/oct"}
ALL_CHECKS = [c for c in CHECKS] + [(c[0], c[1], c[2], c[3], c[4], {"even", "single", "uneven"}, "timbre", c[5]) for c in TIMBRE]
SIG_FEATS = ["dom", "halfs", "others", "sub", "tone", "per1000", "per2000", "per4000", "for2000", "for4000", "mF1400", "mF2800",
             "mF5600", "mR2800", "up1", "up2", "up3", "tr_mid", "sharp", "hslope"]
SIG_WORDS = {
    "dom": ("the firing series stands less over the other orders than in the engine's own recordings", "cleaner than the engine's own recordings: other orders missing"),
    "halfs": ("half-firing orders weaker than the real engine's: too clean", "half-firing orders stronger than in any recording of this engine: bank / pair imbalance the real one does not have"),
    "others": ("cylinder-to-cylinder orders missing: too clean", "cylinder-to-cylinder orders stronger than the real engine's"),
    "sub": ("", "orders under the firing order stronger than the real engine's: pitch drops"),
    "tone": ("the firing tone is weaker than in the engine's own recordings ('tapado')", "only the firing tone: thin"),
    "per1000": ("1-2 kHz less periodic than the real engine: harmonics buried in noise (hiss, 'tapado')", "1-2 kHz more periodic than the real engine: synthetic"),
    "per2000": ("2-4 kHz less periodic than the real engine: harmonics buried in noise (hiss)", "2-4 kHz more periodic than the real engine: synthetic"),
    "per4000": ("4-8 kHz less periodic than the real engine: hiss over the harmonics", "4-8 kHz more periodic than the real engine: synthetic"),
    "for2000": ("fewer orders in 2-4 kHz than the real engine: too clean", "more lines on the half orders in 2-4 kHz than the real engine"),
    "for4000": ("fewer orders in 4-8 kHz than the real engine", "more lines on the half orders in 4-8 kHz than the real engine"),
    "mF1400": ("", "1.4-2.8 kHz pulses with the firing more than the real engine: machine gun"),
    "mF2800": ("", "2.8-5.6 kHz pulses with the firing more than the real engine: machine gun"),
    "mF5600": ("", "5.6-10 kHz pulses with the firing more than the real engine: machine gun"),
    "mR2800": ("treble envelope more deterministic than the real engine's", ""),
    "up1": ("the octave above the firing band is weaker than in the engine's own recordings", "the octave above the firing band is stronger than in any recording of this engine"),
    "up2": ("two octaves above the firing band weaker than the real engine: muffled ('tapado')", "two octaves above the firing band stronger than the real engine: harsh"),
    "up3": ("three octaves above the firing band weaker than the real engine: muffled, 'far'", "three octaves above the firing band stronger than the real engine: hiss"),
    "tr_mid": ("treble weaker than in the engine's own recordings: muffled", "treble stronger than in the engine's own recordings: hissy / shrill"),
    "sharp": ("duller than the engine's own recordings", "sharper than the engine's own recordings"),
    "hslope": ("harmonics fall faster than the real engine's: pure, thin", "harmonics fall slower than the real engine's: buzzy"),
}
SIG_SIDES = {"sub": "hi", "mF1400": "hi", "mF2800": "hi", "mF5600": "hi", "mR2800": "lo"}


def klass(e):
    if e["family"].startswith("single"):
        return "single"
    return "even" if e.get("even") else "uneven"


def tr(feat, v):
    """Modulation depths are judged in dB (20 log10 of the index): their real tail is log-normal."""
    if feat in LOGF:
        return 20 * np.log10(max(v, 0.3) / 100.0) if np.isfinite(v) else np.nan
    return v


class Item:
    def __init__(self, name, entry, table, real):
        self.name, self.e, self.T, self.real = name, entry, table, real
        self.k = klass(entry)
        self.n = sum(table[b]["n"] for b in table)

    def bins(self, feat):
        return [(tr(feat, self.T[b][feat]), b) for b in sorted(self.T) if np.isfinite(self.T[b].get(feat, np.nan))]

    def med(self, feat):
        v = [a for a, b in self.bins(feat)]
        return float(np.median(v)) if v else np.nan

    def slope(self, feat):
        """Change of the feature per octave of engine speed inside this recording (least squares over
        its speed bins; needs three bins spanning half an octave). A fixed microphone response cancels."""
        v = self.bins(feat)
        if len(v) < 3:
            return np.nan
        x = np.log2([0.5 * (Fe.EDGES[b] + Fe.EDGES[b + 1]) for a, b in v])
        if x.max() - x.min() < 0.5:
            return np.nan
        return float(np.polyfit(x, [a for a, b in v], 1)[0])


def load(force=False):
    """(real outside recordings of the library {id: Item}, renders {label: Item}, every recording {id: Item})."""
    t = TB.build(force=force)
    everything = {i: Item(i, e, T, True) for i, (e, T, x) in t["ref"].items()}
    real = {i: it for i, it in everything.items() if it.e["cls"] == "out" and i not in EXCLUDE and it.n >= MIN_WINDOWS}
    sim = {s: Item(s, e, T, False) for s, (e, T, x) in t["sim"].items()}
    return real, sim, everything


def load_public():
    """The public recordings (Wikimedia Commons, CC BY) with enough accepted pull windows: {id: Item}."""
    t = TB.build()
    pub = {i: Item(i, e, T, True) for i, (e, T, x) in t.get("pub", {}).items()}
    return {i: it for i, it in pub.items() if it.n >= MIN_WINDOWS}


def pool_ids(real, pool, item, exclude=()):
    out = []
    for i, r in real.items():
        if i in exclude or i == item.name:
            continue
        if pool == "even" and r.k != "even":
            continue
        if pool == "multi" and r.k == "single":
            continue
        if pool == "class" and r.k != item.k:
            continue
        if pool == "ext" and r.e.get("mic") not in EXT_MICS:
            continue
        out.append(i)
    if pool == "class" and len(out) < 10:        # a thin class borrows every outside recording
        out = [i for i in real if i not in exclude and i != item.name]
    return out


def tail_p(v, pv, side, feat):
    """p-value of v against the pool on one side: rank inside the pool, exponential tail beyond its
    k-th most extreme value. Returns (p, percentile of v in the pool, pool median, pool extreme)."""
    pv = np.asarray(pv, float)
    pv = pv[np.isfinite(pv)]
    n = len(pv)
    if n < 8 or not np.isfinite(v):
        return np.nan, np.nan, np.nan, np.nan
    sgn = 1.0 if side == "hi" else -1.0
    x = np.sort(sgn * pv)[::-1]                    # most extreme first
    w = sgn * v
    k = max(4, int(np.ceil(n / 5)))
    u = x[k - 1]
    pct = 100.0 * (np.sum(pv < v) + 0.5 * np.sum(pv == v)) / n
    if w <= u:
        p = (np.sum(x >= w) + 1.0) / (n + 1.0)
    else:
        b = max(float(np.mean(x[:k - 1] - u)), 0.5 * JND.get(feat, 2.0))
        p = (k + 1.0) / (n + 1.0) * np.exp(-(w - u) / b)
    return float(min(p, 1.0)), float(pct), float(np.median(pv)), float(sgn * x[0])


def judge(item, real, exclude=(), tiers=("order", "texture", "watch", "trend", "timbre")):
    """[{check, tier, feat, kind, side, value, bin, p, score, pct, n, med, ext, msg}] for every check
    that applies to the item."""
    out = []
    for name, feat, kind, side, pool, classes, tier, msg in ALL_CHECKS:
        if item.k not in classes or tier not in tiers:
            continue
        ids = pool_ids(real, pool, item, exclude)
        sides = ("hi", "lo") if side == "both" else (side,)
        best = None
        for sd in sides:
            if kind in ("med", "slope"):
                v = item.med(feat) if kind == "med" else item.slope(feat)
                if not np.isfinite(v):
                    continue
                p, pct, med, ext = tail_p(v, [real[i].med(feat) if kind == "med" else real[i].slope(feat) for i in ids], sd, feat)
                b = None
            else:
                mine = item.bins(feat)
                if not mine:
                    continue
                pv = [a for i in ids for a, _ in real[i].bins(feat)]
                v, b = (max(mine) if sd == "hi" else min(mine))
                p, pct, med, ext = tail_p(v, pv, sd, feat)
                p = min(1.0, p * len(mine)) if np.isfinite(p) else p
            if not np.isfinite(p):
                continue
            r = dict(check=name, tier=tier, feat=feat, kind=kind, side=sd, value=float(v), bin=b, p=p, score=float(-np.log10(max(p, 1e-12))),
                     pct=pct, n=len(ids), med=med, ext=ext, msg=msg, pool=pool)
            if best is None or r["score"] > best["score"]:
                best = r
        if best:
            out.append(best)
    return out


# ---------------------------------------------------------------------------------------------- noise tier
# Sounds that are not the engine's lines: steady broadband treble ("wind", hiss) and a band of treble
# standing out of the spectrum at a fixed frequency ("sizzle", "electric"). Each is one feature with a
# bound that is not chosen: the most extreme real recording of its pool plus half an audible step. The hump
# needs two things at once: to stand out (prominence over both flanks) and to carry sound (the A-weighted
# energy it adds over its flanks, re the total) - each beyond every real recording; or to stand out like a
# tone (TONE_PROM), which is heard at any level. A new
# real recording exceeds the most extreme of n others with probability 1 / (n + 1) (exchangeability: a
# conformal bound with the feature itself as the score; no tail shape assumed).
#   name, feature, summary, pool, gates?, what a listener would say
NOISE = [
    ("noise: broadband treble (wind, hiss)", "nz28", "med", "ext", True,
     "2-8 kHz holds more sound that is not engine-order lines than any real exterior recording: steady hiss over the engine, 'suena a viento'"),
    # not gated: real exterior recordings carry real wind and road noise under 2 kHz (pool extreme -1.2 dB);
    # watched against the ear-approved build instead (lfa-new -20.4, the windy lfa-ex -11.7, 2026-11-11)
    ("noise: broadband low-mid (wind, rush)", "nzlo", "med", "ext", True,
     "150 Hz-2 kHz holds more sound that is not engine-order lines than any real exterior recording: a rush of air under the "
     "engine's tone, 'mucho viento' (the user on an LFA whose exit flow noise radiated straight from the tips, 2026-11-11)"),
    ("noise: a hump in the top octaves (sizzle, 'electric')", "hfprom", "med", "all", True,
     "a band of treble above 4.5 kHz stands out of the spectrum on both sides, at a frequency that does not follow the engine, and holds "
     "more sound than any real recording's: sizzle / 'ruido electrico' (real spectra fall towards the top)"),
    ("noise: broadband treble at its worst speed", "nz28", "max", "ext", False,
     "at one speed 2-8 kHz is mostly not the engine's lines"),
    ("noise: hump in the top octaves at its worst speed", "hfprom", "max", "all", False,
     "at one speed a band of treble stands out of the spectrum"),
    ("noise: share of the top octave (8-16 kHz)", "p816", "med", "all", False,
     "8-16 kHz holds more of the sound than in any real outside recording: the pulses' shock fronts (and flow noise) coming through where the "
     "silencers stop absorbing; not gated: the ear did not name it on the flat-plane V8, which has it too, and 12-16 kHz is at the edge of hearing"),
]
NOISE_STEP = {"nz28": 2.0, "nzlo": 2.0, "hfprom": 2.0, "p816": 2.0}
# A band standing 9 dB out of its neighbours is heard as a tone whatever its share of the sound: the prominence-ratio
# criterion for discrete tones above 1 kHz (ECMA-74 Annex D / ECMA-418-1; from memory, not reopened). The hump's
# prominence here is taken over the lowest point of each flank, which reads a little higher than that ratio.
TONE_PROM = 9.0        # one audible step, dB; the bound sits half of it beyond the real extreme


def noise(item, real, exclude=(), as_build=None):
    """[{check, tier 'noise', feat, kind, value, bin, bound, ext (most extreme real), med, n, margin (value - bound),
    fail, gates, pct, msg, rule, hz}] for the noise checks that apply. A real recording is judged by an 'ext' pool
    only if its own microphone is an exterior one; a build (as_build) always: its ear is 3 m behind the car."""
    build = (not item.real) if as_build is None else as_build
    out = []
    for name, feat, kind, pool, gates, msg in NOISE:
        if pool == "ext" and not build and item.e.get("mic") not in EXT_MICS:
            continue
        ids = pool_ids(real, pool, item, exclude)
        if kind == "med":
            v, b = item.med(feat), None
            pv = np.array([real[i].med(feat) for i in ids], float)
        else:
            mine = item.bins(feat)
            if not mine:
                continue
            v, b = max(mine)
            pv = np.array([a for i in ids for a, _ in real[i].bins(feat)], float)
        pv = pv[np.isfinite(pv)]
        if len(pv) < 8 or not np.isfinite(v):
            continue
        ext = float(pv.max())
        bound = ext + 0.5 * NOISE_STEP[feat]
        if feat in ("nz28", "nzlo"):
            # the most windy exterior recordings carry the camera's own wind: the 75th percentile of
            # the pool plus 1 dB. The user heard 'viento' on builds at -11.4 (2-8 kHz) and -6.2
            # (150 Hz-2 kHz) that the extreme + 1 dB passed (2026-11-11), and still on an inline-4 at
            # -17.9 that the 95th percentile + 2 dB (-12.4) passed (2026-11-12: "sigue sonando a
            # viento"); the builds approved by ear sit at -22.0 / -22.8 (LFA approved / main)
            bound = float(np.percentile(pv, 75)) + 1.0
        # a hump must stand out AND carry sound: its excess energy (hfx) beyond every real recording's as well
        also = None
        if feat == "hfprom":
            if kind == "med":
                v2 = item.med("hfx")
                pv2 = np.array([real[i].med("hfx") for i in ids], float)
            else:
                v2 = float(item.T[b].get("hfx", np.nan))
                pv2 = np.array([a for i in ids for a, _ in real[i].bins("hfx")], float)
            pv2 = pv2[np.isfinite(pv2)]
            if len(pv2) >= 8 and np.isfinite(v2):
                also = dict(value=float(v2), ext=float(pv2.max()), bound=float(pv2.max() + 1.0))
        hz = np.nan
        if feat == "hfprom":
            if b is None:
                fr = [item.T[bb].get("hffreq", np.nan) for bb in item.T if item.T[bb].get("hfprom", 0) >= 0.5 * max(v, 1e-9)]
                hz = float(np.median(fr)) if fr else np.nan
            else:
                hz = float(item.T[b].get("hffreq", np.nan))
        bad = v > bound and (also is None or also["value"] > also["bound"] or v >= TONE_PROM)
        out.append(dict(check=name, tier="noise", feat=feat, kind=kind, side="hi", value=float(v), bin=b, bound=bound, ext=ext,
                        med=float(np.median(pv)), n=len(pv), margin=float(v - bound), fail=bool(gates and bad), gates=gates, over=bool(bad),
                        pct=float(100.0 * np.mean(pv < v)), p=float((np.sum(pv >= v) + 1.0) / (len(pv) + 1.0)),
                        score=99.0 if (gates and bad) else 0.0, pool=pool, msg=msg, hz=hz, also=also,
                        rule=f"<= {bound:.1f} dB: the most extreme of {len(pv)} real " + ("exterior " if pool == "ext" else "outside ")
                             + ("recordings" if kind == "med" else "speed bins") + f" ({ext:.1f} dB) + {0.5 * NOISE_STEP[feat]:.0f} dB"))
    return out


def noise_loo(real):
    """{id: [failed noise results]}: each real recording judged by the others on the noise checks."""
    return {i: [r for r in noise(it, real, exclude=(i,)) if r["fail"]] for i, it in real.items()}


DOM_MIN = 0.0        # dB: no engine order louder than the firing series, in any speed bin


def prior(item):
    """Rules that follow from the firing pattern alone (even-firing engines; no pool, no statistics):
    the firing series is the pitch. (1) the orders under the firing order, summed, are not louder than
    the firing line by more than 3 dB - the rule the real windows are selected by (sp_feat.gate);
    (2) no other engine order is louder than the firing series in any speed bin. Real recordings: the
    lowest of 44 bins has the firing series 4.1 dB over every other line."""
    out = []
    if item.k == "even":
        mine = item.bins("sub")
        if mine:
            v, b = max(mine)
            bad = v > Fe.SUB_MAX
            out.append(dict(check="prior: the pitch is the firing order", tier="prior", feat="sub", kind="max", side="hi", value=float(v), bin=b,
                            p=0.0 if bad else 1.0, score=99.0 if bad else 0.0, pct=np.nan, n=0, med=np.nan, ext=Fe.SUB_MAX, pool="rule", rule="<= 3 dB",
                            msg="the orders under the firing order are louder than the firing line: the engine is heard an octave (or more) "
                                "lower than it turns. Real windows like this cannot be told from a tracking error and are not in the pool: the rule stands on its own"))
        mine = item.bins("dom")
        if mine:
            v, b = min(mine)
            bad = v < DOM_MIN
            out.append(dict(check="prior: no order louder than the firing series", tier="prior", feat="dom", kind="min", side="lo", value=float(v), bin=b,
                            p=0.0 if bad else 1.0, score=99.0 if bad else 0.0, pct=np.nan, n=0, med=np.nan, ext=DOM_MIN, pool="rule", rule=">= 0 dB",
                            msg="another engine order is louder than the whole firing series at this speed: the pitch is no longer the firing order "
                                "('tapado', 'suena fatal', the tone jumping at the top). In real even-firing engines the firing series leads by at least 4 dB in every bin"))
    return out


def loo(real, tiers=("order", "texture"), by_engine=False):
    """{id: (score S, the check that gives it, every result)}: each real recording judged by the others
    (by_engine: by the recordings of OTHER engines only - what a new engine would meet)."""
    out = {}
    for i, it in real.items():
        ex = tuple(j for j, r in real.items() if r.e["engine"] == it.e["engine"]) if by_engine else (i,)
        res = judge(it, real, exclude=ex, tiers=tiers)
        if res:
            top = max(res, key=lambda r: r["score"])
            out[i] = (top["score"], top, res)
    return out


def conformal(scores, alpha=ALPHA):
    """The (1 - alpha) conformal quantile of the real leave-one-out scores: a new real recording
    exceeds it with probability <= alpha. With fewer than 1/alpha - 1 recordings it is their maximum
    (then the guaranteed rate is 1 / (n + 1))."""
    z = np.sort(np.asarray(scores, float))
    if len(z) < 5:
        return np.nan
    k = int(np.ceil((1 - alpha) * (len(z) + 1))) - 1
    return float(z[min(k, len(z) - 1)])


_cache = {}


def sstar(real, tiers=("order", "texture"), alpha=ALPHA):
    """(S*, leave-one-out results)."""
    key = (tuple(sorted(real)), tiers, alpha)
    if key not in _cache:
        L = loo(real, tiers)
        _cache[key] = (conformal([v[0] for v in L.values()], alpha), L)
    return _cache[key]


# ---------------------------------------------------------------------------------------------- signature
def bin_diffs(a, b, feat):
    """Differences a - b of a feature over the speed bins both have (their pull medians when none)."""
    A, B = dict((bb, v) for v, bb in a.bins(feat)), dict((bb, v) for v, bb in b.bins(feat))
    d = [A[i] - B[i] for i in A if i in B]
    if d:
        return d
    va, vb = a.med(feat), b.med(feat)
    return [va - vb] if np.isfinite(va) and np.isfinite(vb) else []


def pair_scales(real):
    """{feature: robust sd of the difference between two recordings of ONE engine}, and the pairs."""
    by = {}
    for i, it in real.items():
        by.setdefault(it.e["engine"], []).append(it)
    pairs = [(a, b) for lst in by.values() for x, a in enumerate(lst) for b in lst[x + 1:]]
    sc = {}
    for f in SIG_FEATS:
        d = [float(np.median(bin_diffs(a, b, f))) for a, b in pairs if bin_diffs(a, b, f)]
        sc[f] = max(1.4826 * float(np.median(np.abs(d))), 0.5 * JND.get(f, 2.0)) if len(d) >= 6 else np.nan
    return sc, pairs


def own(item, pool, exclude=()):
    return [r for i, r in pool.items() if r.e["engine"] == item.e.get("engine") and i != item.name and i not in exclude]


def signature(item, pool, scales, exclude=(), min_rec=2):
    """Per feature, the build against the range of the engine's own recordings at the same speed bins:
    [{feat, worst (signed distance outside that range at the worst bin), bin, value, lo, hi, d (median
    over the bins), e (= |worst| / pair scale)}]; [] when the engine has fewer than min_rec recordings."""
    recs = own(item, pool, exclude)
    if len(recs) < min_rec:
        return []
    out = []
    for f in SIG_FEATS:
        if not np.isfinite(scales.get(f, np.nan)):
            continue
        tabs = [dict((b, v) for v, b in r.bins(f)) for r in recs]
        ds = []
        for v, b in item.bins(f):
            ov = [t[b] for t in tabs if b in t]
            if len(ov) < min(2, len(recs)):
                ov = [t[j] for t in tabs for j in (b - 1, b, b + 1) if j in t]
            if len(ov) < min(2, len(recs)):
                continue
            lo, hi = min(ov), max(ov)
            ds.append((v - hi if v > hi else v - lo if v < lo else 0.0, b, v, lo, hi))
        if not ds:
            continue
        side = SIG_SIDES.get(f, "both")
        key = (lambda t: t[0]) if side == "hi" else (lambda t: -t[0]) if side == "lo" else (lambda t: abs(t[0]))
        w = max(ds, key=key)
        d_w = w[0] if side == "both" else max(w[0], 0.0) if side == "hi" else min(w[0], 0.0)
        out.append(dict(feat=f, worst=d_w, bin=w[1], value=w[2], lo=w[3], hi=w[4], d=float(np.median([t[0] for t in ds])),
                        e=abs(d_w) / scales[f], scale=scales[f], nbins=len(ds), nrec=len(recs)))
    return out


def sig_star(real, scales, alpha=ALPHA):
    """Threshold of the signature: what real recordings reach against the other recordings of their
    own engine, the maximum over the features; conformal (1 - alpha) quantile. (E*, {id: (e, top)})."""
    E = {}
    for i, it in real.items():
        s = signature(it, real, scales, exclude=(i,), min_rec=1)
        if s:
            top = max(s, key=lambda r: r["e"])
            E[i] = (top["e"], top)
    return conformal([v[0] for v in E.values()], alpha), E
