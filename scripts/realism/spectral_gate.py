#!/usr/bin/env python3
"""Spectral / harmonic tier of the realism test: are the engine orders, the texture and the timbre of a
render those of a real engine of its kind, and if not, what would a listener call it.

It complements the physics tier (realism_gate.py: crescendo, load, lift-off, comb). Everything it
knows about real engines comes from the recordings in spectral/manifest.json (the classified library
real_engine_sounds/ and the LFA clips); nothing is tuned on the simulator's own builds.

  ORDER     which engine orders carry the tone (even-firing engines): the firing series over every
            other line, half-firing orders, orders under the firing order, cylinder-to-cylinder orders
  TEXTURE   how periodic the sound is band by band (lines against the floor at the same frequency),
            how the treble envelope rides the firing rhythm; unchanged by any microphone response
  TIMBRE    octave bands above the firing band, sharpness, harmonic slope: the microphone is in them;
            printed against the class pool, judged only against the engine's own recordings
  SIGNATURE every feature against the engine's own outside recordings, speed bin by speed bin, in units
            of how much two recordings of one real engine differ

usage
  spectral_gate.py refs                    the real pools, every check's real range, the thresholds
  spectral_gate.py loo                     every real recording judged by the others (false rejections)
  spectral_gate.py gate LABEL [LABEL...]   judge renders (spectral/out/sim/LABEL.pkl), with words
  spectral_gate.py matrix                  the ear-verdict validation table
  spectral_gate.py inject [ID...]          one defect at a time put into real pulls: what catches it
  spectral_gate.py presets                 every preset of the simulator + the fleet against real engines
  spectral_gate.py physics BASE ABL...     which simulator source moves which feature (ablation renders)
  spectral_gate.py public                  openly licensed recordings: out-of-sample check, families the library lacks
  spectral_gate.py limits                  where each check starts to fail for a build, beside the real pool
  spectral_gate.py seen                    every check seen failing: by sabotage, and on audio (builds, injected defects)
  spectral_gate.py today                   the ear verdicts of 2026-11-10 ('viento', 'electrico') on the noise tier

  NOISE     (added 2026-11-10 evening) what is heard that is not the engine's lines: the broadband floor's share of
            2-8 kHz ('wind', hiss) against real exterior recordings; a hump of the spectrum above 4.5 kHz that
            stands out on both sides ('sizzle', 'electric') against every outside recording

Prepared by spectral/sp_refs.py (recordings), sp_sim.py (renders), sp_inject.py (defects). Local only.
"""
import os
import pickle
import sys
import textwrap
import warnings

warnings.filterwarnings("ignore")
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "spectral"))
import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402
import sp_feat as Fe  # noqa: E402
import sp_stat as S  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # rdata
import rdata  # noqa: E402

GATE_TIERS = ("order", "texture")
WATCH_TIERS = ("watch", "trend")
# Periodicity beyond EVERY real recording fails on its own, whatever the engine's own recordings say
# (2026-11-13). Engine lines where every real engine has noise is the 'synthetic' the user hears
# ("suena sintético", LFA pf build); before, the texture tier excused it whenever the engine had
# recordings of its own and the line forest was only watched, so every LFA build passed with 1-2 kHz
# 13 dB more periodic than any real recording. 'Beyond' is past the pool's extreme by more than the
# most extreme real recording reaches past the others (Tables.periodic_margin), so no real one fails.
PERIODIC_TIERS = ("texture", "watch")
PERIODIC_FEATS = ("per1000", "per2000", "per4000", "for2000", "for4000")


def binname(b, e=None):
    if b is None:
        return "the pull"
    lo, hi = Fe.EDGES[b], Fe.EDGES[b + 1]
    return f"{lo * e['redline']:.0f}-{hi * e['redline']:.0f} rpm" if e else f"{lo:.2f}-{hi:.2f} x redline"


def show(feat, v):
    """A feature value in the unit a person reads (modulation depths are stored in dB of the index)."""
    if feat in S.LOGF:
        return f"{100 * 10 ** (v / 20):.0f} %"
    return f"{v:.1f} {S.UNIT.get(feat, 'dB')}"


class Tier:
    """The pools and thresholds, computed once."""

    def __init__(self):
        self.real, self.sim, self.all = S.load()
        self.Sstar, self.L = {}, {}
        for t in GATE_TIERS:                      # each tier is calibrated on what real recordings reach on it
            self.Sstar[t], self.L[t] = S.sstar(self.real, (t,), S.TIER_ALPHA[t])
        self.scales, self.pairs = S.pair_scales(self.real)
        self.Estar, self.E = S.sig_star(self.real, self.scales)
        self._pmargin = None

    def periodic_margin(self):
        """{feat: dB} how far the most extreme real recording reaches past all the others (leave one out),
        at least 0: a build beyond the real pool's extreme by more than this is beyond what any real
        recording does to the others, so no real recording can fail the periodicity rule."""
        if self._pmargin is None:
            m = {f: 0.0 for f in PERIODIC_FEATS}
            for i, it in self.real.items():
                for r in S.judge(it, self.real, exclude=(i,), tiers=PERIODIC_TIERS):
                    if r["feat"] in m and r["side"] == "hi":
                        m[r["feat"]] = max(m[r["feat"]], r["value"] - r["ext"])
            self._pmargin = m
        return self._pmargin

    def state(self, r, sig=False):
        """' ok ', 'EDGE' (rarer than 97 % of real recordings), 'OVER' (beyond what real recordings reach
        on this tier, but the tier does not decide here) or 'FAIL'. sig: the engine has recordings of its
        own, so its texture is judged against them (signature), not against other engines."""
        if (r["tier"] in PERIODIC_TIERS and r["feat"] in PERIODIC_FEATS and r["side"] == "hi"
                and r["value"] > r["ext"] + self.periodic_margin()[r["feat"]]):
            return "FAIL"
        if r["tier"] in GATE_TIERS and r["score"] > self.Sstar[r["tier"]]:
            return "OVER" if (sig and r["tier"] == "texture") else "FAIL"
        return "EDGE" if r["p"] < S.EDGE_P else " ok "

    def judge(self, item, exclude=(), as_build=None):
        """dict(failed [(tier, result)], edge, sfail, res, prior, sig, S {tier: score}, E, by_sig).
        FAIL when: a rule of the firing pattern is broken (prior); or the order tier is beyond what real
        recordings reach; or the texture is - against the engine's own recordings when it has at least two
        (signature, E > E*), else against all real engines (texture tier, S > S*)."""
        res = S.judge(item, self.real, exclude=exclude)
        pr = S.prior(item)
        nz = S.noise(item, self.real, exclude=exclude, as_build=as_build)
        sig = S.signature(item, self.real, self.scales, exclude=exclude)
        by_sig = bool(sig)
        failed = [("prior", r) for r in pr if r["score"] > 50] + [(r["tier"], r) for r in res if self.state(r, by_sig) == "FAIL"]
        failed += [("noise", r) for r in nz if r["fail"]]
        if not len(item.T):  # nothing rendered or nothing accepted: an empty build must not pass
            failed.append(("data", {"check": "no speed bins: nothing judged"}))
        edge = [(r["tier"], r) for r in res if self.state(r, by_sig) in ("EDGE", "OVER") and r["tier"] != "timbre"]
        sfail = [r for r in sig if np.isfinite(self.Estar) and r["e"] > self.Estar]
        return dict(failed=failed, edge=edge, sfail=sfail, res=res, prior=pr, sig=sig, by_sig=by_sig, noise=nz,
                    S={t: max([r["score"] for r in res if r["tier"] == t], default=np.nan) for t in GATE_TIERS},
                    E=max([r["e"] for r in sig], default=np.nan))

    def sline(self, J):
        out = [f"{t} {J['S'][t]:.2f}/{self.Sstar[t]:.2f}" for t in GATE_TIERS if np.isfinite(J["S"][t])]
        if J["by_sig"]:
            out.append(f"signature {J['E']:.2f}/{self.Estar:.2f}")
        for r in J.get("noise", []):
            if r["gates"]:
                out.append(f"{'wind' if r['feat'] == 'nz28' else 'hump'} {r['value']:.1f}/{r['bound']:.1f} dB"
                           + (f" & {r['also']['value']:.1f}/{r['also']['bound']:.1f} dB" if r.get("also") else ""))
        return ", ".join(out)


def cmd_refs(T, args):
    real = T.real
    eng = {r.e["engine"] for r in real.values()}
    print(f"pool: {len(real)} outside recordings (tail / exterior microphones) of {len(eng)} engines, pull windows only")
    for k in ("even", "single", "uneven"):
        ids = sorted(i for i, r in real.items() if r.k == k)
        items = [f"{i} ({len(real[i].T)})" for i in ids]
        print(f"  {k:7s} {len(ids):2d}: " + ", ".join(items[:7]))
        for a in range(7, len(items), 7):
            print(f"  {'':11s} " + ", ".join(items[a:a + 7]))
    print(f"  (number of speed bins in brackets; a bin needs {Fe.MIN_N} accepted windows, a recording {S.MIN_WINDOWS})")
    print("  left out: " + "; ".join(f"{i}: {why}" for i, why in S.EXCLUDE.items()))
    probe = {k: next(r for r in real.values() if r.k == k) for k in ("even", "single", "uneven")}
    print(f"\n{'check':50s} {'tier':8s} {'summary':10s} {'defect':6s} {'pool':6s} {'n':>4s} {'p10':>8s} {'median':>8s} {'p90':>8s} {'extreme':>8s}")
    for name, feat, kind, side, pool, classes, tier, msg in S.ALL_CHECKS:
        k = "even" if "even" in classes else sorted(classes)[0]
        ids = S.pool_ids(real, pool, probe[k], exclude=("__none__",)) + ([probe[k].name] if (pool != "even" or probe[k].k == "even") else [])
        ids = sorted(set(ids))
        pv = np.array([real[i].med(feat) for i in ids] if kind == "med" else [real[i].slope(feat) for i in ids] if kind == "slope" else [a for i in ids for a, _ in real[i].bins(feat)], float)
        pv = pv[np.isfinite(pv)]
        if len(pv) < 8:
            continue
        ext = pv.max() if side in ("hi", "both") else pv.min()
        sm = {"med": "pull median", "min": "worst bin", "max": "worst bin", "slope": "per octave"}[kind]
        print(f"{name:50s} {tier:8s} {sm:10s} {side:6s} {pool:6s} {len(pv):4d} {show(feat, np.percentile(pv, 10)):>8s} {show(feat, np.median(pv)):>8s} "
              f"{show(feat, np.percentile(pv, 90)):>8s} {show(feat, ext):>8s}")
    print("\nthresholds (S = -log10 p of a tier's worst check; S* = what real recordings reach when each is judged by the others):")
    for t in GATE_TIERS:
        rej = sum(1 for v in T.L[t].values() if v[0] > T.Sstar[t])
        print(f"  {t:8s} S* = {T.Sstar[t]:.2f} (p = {10 ** -T.Sstar[t]:.2g}); allowed share of real recordings {S.TIER_ALPHA[t]:.2f}; leave-one-out rejects {rej} of {len(T.L[t])}")
    print(f"  a check with p < {S.EDGE_P} that does not fail is printed as EDGE; tiers {', '.join(WATCH_TIERS)} and timbre are printed, never failed")
    print("\nnoise tier (bound = the most extreme real recording of the pool + half an audible step; a new real recording exceeds the extreme")
    print("of n others with probability 1 / (n + 1)):")
    probe_b = S.Item("__build__", dict(family="i4", even=True, engine="__build__"), {}, False)
    for name, feat, kind, pool, gates, msg in S.NOISE:
        ids = S.pool_ids(real, pool, probe_b)
        pv = np.array([real[i].med(feat) for i in ids] if kind == "med" else [a for i in ids for a, _ in real[i].bins(feat)], float)
        pv = pv[np.isfinite(pv)]
        top = sorted(((real[i].med(feat) if kind == "med" else max(a for a, _ in real[i].bins(feat))), i) for i in ids)[-3:][::-1]
        print(f"  {name:55s} {'pull median' if kind == 'med' else 'worst bin':11s} pool {pool:3s} n={len(pv):3d}  p10 {np.percentile(pv, 10):6.1f}  median {np.median(pv):6.1f}  p90 {np.percentile(pv, 90):6.1f}"
              f"  extreme {pv.max():6.1f} dB  bound {pv.max() + 0.5 * S.NOISE_STEP[feat]:6.1f} dB  {'GATE ' if gates else 'watch'}  top: " + ", ".join(f"{i} {v:.1f}" for v, i in top))
        if feat == "hfprom" and kind == "med":
            p2 = np.array([real[i].med("hfx") for i in ids], float)
            p2 = p2[np.isfinite(p2)]
            t2 = sorted((real[i].med("hfx"), i) for i in ids)[-3:][::-1]
            print(f"  {'   and the sound the hump adds (re the A-weighted whole)':55s} {'pull median':11s} pool {pool:3s} n={len(p2):3d}  p10 {np.percentile(p2, 10):6.1f}  median {np.median(p2):6.1f}  p90 {np.percentile(p2, 90):6.1f}"
                  f"  extreme {p2.max():6.1f} dB  bound {p2.max() + 1.0:6.1f} dB  GATE   top: " + ", ".join(f"{i} {v:.1f}" for v, i in t2))
    print(f"\nsignature: {len(T.pairs)} pairs of recordings of one engine ({', '.join(sorted({a.e['engine'] for a, b in T.pairs}))})")
    print("  robust sd of the difference between two recordings of ONE engine (the unit of the signature):")
    print(textwrap.fill(", ".join(f"{f} {show(f, v) if f not in S.LOGF else f'{v:.1f} dB'}" for f, v in T.scales.items() if np.isfinite(v)),
                        width=150, initial_indent="  ", subsequent_indent="  "))
    print(f"  threshold E* = {T.Estar:.2f} of those units (what real recordings reach against the others of their engine, {len(T.E)} recordings)")


def cmd_loo(T, args):
    print("leave-one-recording-out: every real outside recording judged by the others")
    tot = set()
    for t in GATE_TIERS:
        L = T.L[t]
        print(f"\n{t} tier: {len(L)} recordings, S* = {T.Sstar[t]:.2f}")
        print(f"  {'recording':18s} {'class':7s} {'mic':5s} bins     S           the check that gives S")
        for i, (sc, top, res) in sorted(L.items(), key=lambda kv: -kv[1][0]):
            r = T.real[i]
            bad = sc > T.Sstar[t]
            if bad:
                tot.add(i)
            if bad or sc > 0.75 * T.Sstar[t] or "-v" in args:
                print(f"  {i:18s} {r.k:7s} {r.e['mic']:5s} {len(r.T):3d}  {sc:5.2f}  {'REJECT' if bad else '  ok  '}   {top['check'].split(': ')[1]} = {show(top['feat'], top['value'])}"
                      + (f" at {binname(top['bin'])}" if top["bin"] is not None else "") + f" (others: median {show(top['feat'], top['med'])}, extreme {show(top['feat'], top['ext'])})")
        rest = sum(1 for v in L.values() if v[0] <= 0.75 * T.Sstar[t])
        if "-v" not in args:
            print(f"  ... and {rest} recordings under three quarters of S*")
    NL = S.noise_loo(T.real)
    print("\nnoise tier: each real recording against the most extreme of the others (+ half an audible step); by construction only a")
    print("recording that is itself the extreme of its pool by more than that can fail (a new one: probability 1 / (n + 1) per check)")
    for name, feat, kind, pool, gates, msg in S.NOISE:
        if not gates:
            continue
        judged = [i for i, it in T.real.items() if any(r["check"] == name for r in S.noise(it, T.real, exclude=(i,)))]
        bad = [(i, r) for i in judged for r in NL[i] if r["check"] == name]
        print(f"  {name:55s} judged {len(judged):2d} ({'exterior microphones' if pool == 'ext' else 'every outside recording'}), rejected {len(bad)}: "
              + ("; ".join(f"{i} {r['value']:.1f} dB (the others reach {r['ext']:.1f})" for i, r in bad) or "none"))
        if pool == "ext":
            tails = [(i, it) for i, it in T.real.items() if it.e.get("mic") not in S.EXT_MICS]
            over = []
            for i, it in tails:
                for r in S.noise(it, T.real, exclude=(i,), as_build=True):
                    if r["check"] == name and r["fail"]:
                        over.append(f"{i} {r['value']:.1f} dB")
            print(f"  {'':55s} not judged: {len(tails)} tail-pipe microphones (in the jet's near field); held to the exterior bound, {len(over)} would fail: " + ("; ".join(over) or "none"))
    for i, lst in NL.items():
        if lst:
            tot.add(i)
    n = len(T.real)
    print(f"\nrejected by the gate (any tier): {len(tot)} of {n} real recordings ({100 * len(tot) / n:.0f} %): {', '.join(sorted(tot)) or 'none'}")
    by = {k: (sum(1 for i in tot if T.real[i].k == k), sum(1 for r in T.real.values() if r.k == k)) for k in ("even", "single", "uneven")}
    print("  by class: " + ", ".join(f"{k} {a}/{b}" for k, (a, b) in by.items()))
    print("\nleave-one-ENGINE-out (each recording judged by recordings of other engines only: what a new engine meets), same thresholds:")
    for t in GATE_TIERS:
        Le = S.loo(T.real, (t,), by_engine=True)
        bad = sorted(((v[0], i, v[1]) for i, v in Le.items() if v[0] > T.Sstar[t]), reverse=True)
        print(f"  {t:8s} {len(bad)} of {len(Le)} beyond S*: " + ("; ".join(f"{i} ({sc:.2f}: {top['check'].split(': ')[1]} {show(top['feat'], top['value'])})" for sc, i, top in bad) or "none"))
    print(f"\nsignature, each recording against the others of its own engine ({len(T.E)} recordings of engines with at least two); E* = {T.Estar:.2f}")
    for i, (e, top) in sorted(T.E.items(), key=lambda kv: -kv[1][0])[:8]:
        print(f"  {i:18s} E {e:5.2f} {'REJECT' if e > T.Estar else '  ok  '} {top['feat']:8s} {top['worst']:+6.1f} outside its engine's other recordings at {binname(top['bin'])}")


def report(T, item, J, verbose=True):
    lines = []
    e = item.e
    for r in J["prior"]:
        st = "FAIL" if r["score"] > 50 else " ok "
        lines.append(f"  {st} {r['check']:50s} {show(r['feat'], r['value']):>9s} at {binname(r['bin'], e)}   (rule: {r['rule']})")
        if st == "FAIL":
            lines.append(f"         -> {r['msg']}")
    for r in J.get("noise", []):
        st = "FAIL" if r["fail"] else ("over" if r.get("over") else "EDGE" if r["value"] > r["ext"] else " ok ")
        if not verbose and st == " ok ":
            continue
        where = "pull median" if r["bin"] is None else binname(r["bin"], e)
        hz = f" at {r['hz'] / 1000:.1f} kHz" if np.isfinite(r.get("hz", np.nan)) and r["value"] > 1.0 else ""
        lines.append(f"  {st} {r['check']:50s} {r['value']:6.1f} dB{hz} {where:19s} real: median {r['med']:.1f}, most extreme {r['ext']:.1f} dB (n={r['n']});"
                     f"  bound {r['bound']:.1f} dB, margin {r['margin']:+.1f}"
                     + (f"; the sound it adds {r['also']['value']:.1f} dB re the whole (real: most extreme {r['also']['ext']:.1f}, bound {r['also']['bound']:.1f})" if r.get("also") else "")
                     + ("" if r["gates"] else "   [watch, not part of the verdict]"))
        if st != " ok ":
            lines.append(f"         -> {r['msg']}")
    for tier in GATE_TIERS + WATCH_TIERS:
        rs = sorted([x for x in J["res"] if x["tier"] == tier], key=lambda x: -x["score"])
        if rs and tier in WATCH_TIERS and tier == WATCH_TIERS[0]:
            lines.append("  watch (printed with its p against real recordings; not part of the verdict)")
        for r in rs:
            st = T.state(r, J["by_sig"])
            if not verbose and st == " ok ":
                continue
            where = binname(r["bin"], e) if r["bin"] is not None else "per octave of speed" if r["kind"] == "slope" else "pull median"
            lines.append(f"  {st} {r['check']:50s} {show(r['feat'], r['value']):>9s} {where:19s} real: median {show(r['feat'], r['med'])}, extreme {show(r['feat'], r['ext'])}"
                         f" (n={r['n']});  percentile {r['pct']:.0f}, p = {r['p']:.2g}")
            if st != " ok ":
                m = r["msg"]
                if " | " in m:
                    m = m.split(" | ")[0 if r["side"] == "lo" else 1]
                lines.append(f"         -> {m}" + ("  [rarer than 97 % of real recordings, not beyond what they reach]" if st == "EDGE" else
                                               "  [beyond every other real engine; this engine has recordings of its own, the signature decides]" if st == "OVER" else ""))
    if verbose:
        lines.append("  timbre against the class pool (the microphone is in these: information, not a verdict)")
        for r in [x for x in J["res"] if x["tier"] == "timbre"]:
            lines.append(f"    -  {r['check']:50s} {show(r['feat'], r['value']):>9s} percentile {r['pct']:3.0f} of {r['n']} recordings (median {show(r['feat'], r['med'])})")
    if J["sig"]:
        nrec = J["sig"][0]["nrec"]
        lines.append(f"  signature: against {nrec} outside recordings of this engine ({', '.join(sorted(r.name for r in S.own(item, T.real)))}), speed bin by speed bin;")
        lines.append(f"             unit = robust spread of two recordings of one real engine; E* = {T.Estar:.2f} (what real recordings reach against their engine's others)")
        for r in sorted(J["sig"], key=lambda x: -x["e"]):
            if r["worst"] == 0 or (r["e"] < 1.0 and not verbose):
                continue
            st = "OFF " if r["e"] > T.Estar else " ~  "
            lines.append(f"  {st} {r['feat']:8s} {show(r['feat'], r['value']):>9s} at {binname(r['bin'], e):14s} own recordings {show(r['feat'], r['lo'])} .. {show(r['feat'], r['hi'])}"
                         f"  -> {r['worst']:+.1f} outside ({r['e']:.1f} units; median over the bins {r['d']:+.1f})")
            w = S.SIG_WORDS[r["feat"]][0 if r["worst"] < 0 else 1]
            if w and r["e"] >= 1.0:
                lines.append(f"         -> {w}")
    return lines


def verdict(J):
    return "FAIL" if (J["failed"] or J["sfail"]) else "PASS"


def fails(J, e=None):
    out = [short(r, e) + (f" (rule: {r['rule']})" if t in ("prior", "noise") else f" (p={r['p']:.2g})") for t, r in J["failed"]]
    out += [f"signature: {r['feat']} {r['worst']:+.1f} outside its own recordings @ {binname(r['bin'], e)} ({r['e']:.1f} units)" for r in sorted(J["sfail"], key=lambda r: -r["e"])]
    return out


def short(r, e=None):
    return f"{r['check']} {show(r['feat'], r['value'])}" + (f" @ {binname(r['bin'], e)}" if r["bin"] is not None else "")


# The last build of each engine the user approved by ear (label prefix -> render label). Real exterior recordings
# carry real wind and road noise under 2 kHz, so that band is judged against the ear instead: a build may not
# hold more broadband sound between its orders in 150 Hz-2 kHz than the approved one plus one audible step
# (2 dB). The LFA whose exit flow noise was radiated from the tips read -11.7 dB against the approved -20.4:
# "mucho viento" (2026-11-11).
APPROVED = {"lfa": "lfa-new", "v8-flatplane": "v8-flatplane-new"}
LOW_WIND_STEP = 2.0


def low_wind(T, lab, it):
    """(value, bound, baseline label) of the low-mid wind guard, or None when the engine has no approved build."""
    base = next((b for p, b in sorted(APPROVED.items(), key=lambda x: -len(x[0])) if lab.startswith(p)), None)
    if base is None or base == lab or base not in T.sim:
        return None
    v, b = it.med("nzlo"), T.sim[base].med("nzlo")
    if not (np.isfinite(v) and np.isfinite(b)):
        return None
    return v, b + LOW_WIND_STEP, base


def cmd_gate(T, args):
    rc = 0
    verbose = "-q" not in args
    for lab in [a for a in args if not a.startswith("-")]:
        it = T.sim[lab]
        J = T.judge(it)
        lw = low_wind(T, lab, it)
        if lw and lw[0] > lw[1]:
            J["failed"].append(("noise", {"check": "noise: low-mid wind over the ear-approved build"}))
        v = verdict(J)
        names = ", ".join(sorted({r["check"] for t, r in J["failed"]} | {"signature: " + r["feat"] for r in J["sfail"]}))
        print(f"\n{lab} ({it.e['family']}, {len(it.T)} speed bins): {v}" + (f" ({names})" if names else "") + (f"; {len(J['edge'])} at the edge" if J["edge"] else "")
              + f"   score / threshold: {T.sline(J)}" + ("" if J["sig"] else "; no outside recordings of this engine: texture judged against all real engines"))
        if lw:
            print(f"  {'FAIL' if lw[0] > lw[1] else ' ok '} noise: low-mid wind 150 Hz-2 kHz {lw[0]:6.1f} dB   bound {lw[1]:.1f} dB "
                  f"(the ear-approved {lw[2]} + {LOW_WIND_STEP:.0f} dB)")
            if lw[0] > lw[1]:
                print("         -> more air rush under the engine's tone than the build the user approved: 'mucho viento'")
        else:
            print("  note: no ear-approved build of this engine: low-mid wind not judged")
        print("\n".join(report(T, it, J, verbose)))
        rc |= v == "FAIL"
    return rc


VERDICTS = [  # (label, the ear's verdict, expected result)
    ("lfa-approved", "approved by ear (~98 %)", "PASS"),
    ("lfa-main", "main: the approved sound without the ground comb", "PASS"),
    ("lfa-tapado", "'muy tapado, muchos armonicos tapados'", "FAIL"),
    ("lfa-fatal", "'suena fatal', level jumping at the top, muffled", "FAIL"),
    ("lfa-x-pulses", "rebuilt: every texture source off -> 'sintetico'", "FAIL"),
    ("lfa-x-equal", "rebuilt: equal pipes -> 'demasiado limpio, mosquito'", "FAIL"),
    ("lfa-x-linear", "rebuilt: linear pipes -> 'suena muy lejos'", "FAIL"),
    ("lfa-x-ground", "rebuilt: rigid ground image -> 'enlatado, ecos raros'", "FAIL"),
    ("lfa-x-reverb", "rebuilt: room reverb 0.15 -> 'como mucho eco'", "FAIL"),
]


TODAY = [  # (label, the ear 2026-11-10 on the gears clip, expected, the check that names what the ear said)
    ("honda-k20a-new", "'el honda suena muchisimo a viento'", "FAIL", "noise: broadband treble (wind, hiss)"),
    ("honda-k20a-tuned-new", "the tuned K20A, same engine ('el honda')", "FAIL", "noise: broadband treble (wind, hiss)"),
    ("inline4-new", "'asi como el inline 4' (viento)", "FAIL", "noise: broadband treble (wind, hiss)"),
    ("v8-crossplane-new", "'el crossplane tiene como un ruido electrico'", "FAIL", "noise: a hump in the top octaves (sizzle, 'electric')"),
    ("lfa-new", "'suena bien' (not criticised)", "PASS", None),
    ("v8-flatplane-new", "not criticised", "PASS", None),
    ("honda-k20c1-new", "not named - but 'el honda' may mean it", "PASS?", None),
]


def cmd_today(T, args):
    """The ear verdicts of 2026-11-10 (gears clips of the build with the body screen): does the gate fail the three
    'wind' builds and the 'electric' one for the reason the ear gave, and pass the ones not criticised."""
    print("ear verdicts of 2026-11-10 on the new build (held full-load renders of the same binary). 'reason': the check that names")
    print("what the ear said is among the failures. Other failures are listed: they are the gate's, not the ear's.")
    print(f"{'build':22s} {'ear':48s} {'gate':5s} {'want':5s} {'reason':7s} failed checks")
    ok = n = right = nr = 0
    for lab, ear, exp, chk in TODAY:
        if lab not in T.sim:
            continue
        it = T.sim[lab]
        J = T.judge(it)
        v = verdict(J)
        names = [r["check"] for t, r in J["failed"]] + ["signature: " + r["feat"] for r in J["sfail"]]
        hit = "-" if chk is None else ("yes" if chk in names else "NO")
        if not exp.endswith("?"):
            n += 1
            ok += v == exp
        if chk is not None:
            nr += 1
            right += chk in names
        nz = {r["feat"]: r for r in J["noise"] if r["gates"]}
        print(f"{lab:22s} {ear:48s} {v:5s} {exp:5s} {hit:7s} " + ("; ".join(names) or "-"))
        print(f"{'':22s}   wind {nz['nz28']['value']:6.1f} dB (bound {nz['nz28']['bound']:.1f}, margin {nz['nz28']['margin']:+.1f});  hump {nz['hfprom']['value']:5.1f} dB"
              + (f" at {nz['hfprom']['hz'] / 1000:.1f} kHz" if np.isfinite(nz['hfprom']['hz']) and nz['hfprom']['value'] > 1 else "")
              + f" (bound {nz['hfprom']['bound']:.1f}, margin {nz['hfprom']['margin']:+.1f}) holding {nz['hfprom']['also']['value']:.1f} dB of the sound (bound {nz['hfprom']['also']['bound']:.1f})")
    print(f"\nagreement with the ear: {ok} / {n} (the K20C1 not counted: the verdict does not name it); failed for the ear's reason: {right} / {nr}")
    labs = ["lfa-approved", "lfa-main", "lfa-tapado", "lfa-fatal", "lfa-x-pulses", "lfa-x-equal", "lfa-x-linear", "lfa-x-ground", "lfa-x-reverb"]
    print("\nthe earlier verdict builds on the noise checks (none may fail them: no one called them windy or electric):")
    for lab in labs:
        if lab in T.sim:
            nz = {r["feat"]: r for r in T.judge(T.sim[lab])["noise"] if r["gates"]}
            print(f"  {lab:14s} wind {nz['nz28']['value']:6.1f} dB ({'FAIL' if nz['nz28']['fail'] else 'ok'}, margin {nz['nz28']['margin']:+.1f})   hump {nz['hfprom']['value']:4.1f} dB ({'FAIL' if nz['hfprom']['fail'] else 'ok'}, margin {nz['hfprom']['margin']:+.1f})")
    return 0


def cmd_matrix(T, args):
    print("thresholds from real recordings only (leave-one-out): " + ", ".join(f"{t} S* = {T.Sstar[t]:.2f}" for t in GATE_TIERS))
    print(f"{'build':13s} {'ear verdict':52s} {'gate':5s} {'want':5s} score / threshold per tier")
    agree = n = heard = heard_ok = 0
    rows = []
    for lab, ear, exp in VERDICTS:
        if lab not in T.sim:
            continue
        it = T.sim[lab]
        J = T.judge(it)
        v = verdict(J)
        n += 1
        agree += v == exp
        if not ear.startswith("rebuilt"):
            heard += 1
            heard_ok += v == exp
        print(f"{lab:13s} {ear:52s} {v:5s} {exp:5s} {T.sline(J)}")
        for f in fails(J, it.e)[:4]:
            print(f"{'':14s}FAIL  {f}")
        rows.append((lab, it, J))
    print(f"\nagreement with the ear: {heard_ok} / {heard} on the builds the user heard, {agree} / {n} counting the rebuilt defects")
    print("\nnot failed but printed for the four heard builds: EDGE (rarer than 97 % of real recordings), OVER (texture beyond every other real")
    print("engine; the LFA's own recordings decide), watch lines")
    for lab, it, J in rows[:4]:
        print(f"  {lab}")
        for t, r in J["edge"]:
            print(f"      {T.state(r, J['by_sig'])} {short(r, it.e)} (p={r['p']:.2g}; real median {show(r['feat'], r['med'])}, extreme {show(r['feat'], r['ext'])})")
    print(f"\nsignature: distance outside the LFA's own outside recordings at the worst speed bin (units of the spread of two recordings of one engine); E* = {T.Estar:.2f}")
    feats = ["dom", "halfs", "others", "tone", "up1", "per1000", "per2000", "mF2800"]
    print(f"  {'':13s} {'E':>5s}  " + " ".join(f"{f:>15s}" for f in feats))
    for lab, it, J in rows:
        by = {r["feat"]: r for r in J["sig"]}
        print(f"  {lab:13s} {J['E']:5.2f}  " + " ".join(f"{by[f]['worst']:+7.1f} ({by[f]['e']:3.1f})  " if f in by else f"{'.':>15s}" for f in feats))
    # the deciding feature, bin by bin, beside the lowest real value of each bin
    bins = sorted({b for lab, it, J in rows for b in it.T})
    e = rows[0][1].e
    print("\nfiring series over the strongest other order (dB), speed bin by speed bin; rule: >= 0 dB in every bin")
    print(f"  {'':13s} " + " ".join(f"{binname(b, e).replace(' rpm', ''):>10s}" for b in bins) + "    worst")
    for lab, it, J in rows:
        d = dict((b, a) for a, b in it.bins("dom"))
        print(f"  {lab:13s} " + " ".join(f"{d[b]:10.1f}" if b in d else f"{'.':>10s}" for b in bins) + f"   {min(d.values()):6.1f}")
    ev = [r for r in T.real.values() if r.k == "even"]
    low, cnt = [], []
    for b in bins:
        v = [a for r in ev for a, bb in r.bins("dom") if bb == b]
        low.append(f"{min(v):10.1f}" if v else f"{'.':>10s}")
        cnt.append(f"{len(v):10d}")
    allv = [a for r in ev for a, bb in r.bins("dom")]
    print(f"  {'real, lowest':13s} " + " ".join(low) + f"   {min(allv):6.1f}   (16 even-firing outside recordings, same fractions of their own redline)")
    print(f"  {'real, n':13s} " + " ".join(cnt))
    return 0


INJECT_SET = ["lfa-accel", "lfa-flight", "lfa-tips", "s2000-ext8", "s2000-ext1", "escort-ext", "cruze-ext", "alfa4c-ext", "octavia-exh", "fabia-exh",
              "r33-exh", "911sc-exh", "drz-exh", "kodiak-exh", "drz-ext1", "softail-exh", "nighttrain-ext10", "falco-exh"]
COMB_MAX = 12.0     # floor ripple, dB (the physics tier's comb check)
INJECT = rdata.DATA + "/spectral/out/inject2" if os.path.isdir(rdata.DATA + "/spectral/out/inject2") else rdata.DATA + "/spectral/out/inject"


def inject_rows(T, ids):
    """Per recording: (id, entry, cached data, untouched Item, its judgment, [(defect, judgment, new failures of this
    tier's checks, caught by the physics tier's floor ripple, new signature failures)]). None fields when not prepared."""
    for i in ids:
        p = f"{INJECT}/{i}.pkl"
        if not os.path.exists(p):
            yield i, None, None, None, None, None
            continue
        d = pickle.load(open(p, "rb"))
        e = T.all[i].e
        base = S.Item(i, e, d["original"]["table"], True)
        # as a build: the noise checks apply whatever the recording's microphone (the question is whether the
        # check hears the defect; a tail-pipe recording is not itself judged by the exterior pool, see loo)
        J0 = T.judge(base, exclude=(i,), as_build=True)
        f0 = {r["check"] for t, r in J0["failed"]}
        s0 = {r["feat"] for r in J0["sfail"]}
        sc0 = {r["check"]: r["score"] for r in J0["res"] + J0["prior"] + J0["noise"]}
        out = []
        for name in [k for k in d if k != "original"]:
            J = T.judge(S.Item(i, e, d[name]["table"], True), exclude=(i,), as_build=True)
            # a new failure: the check did not fail before, and its p fell by at least a factor two (a recording
            # that sits exactly on a threshold must not count every nudge as a catch)
            new = [r for t, r in sorted(J["failed"], key=lambda tr: -tr[1]["score"]) if r["check"] not in f0 and r["score"] - sc0.get(r["check"], 0.0) > 0.3]
            comb = d[name]["ripple"] > COMB_MAX and d["original"]["ripple"] <= COMB_MAX
            snew = [r for r in sorted(J["sfail"], key=lambda r: -r["e"]) if r["feat"] not in s0]
            out.append((name, J, new, comb, snew))
        yield i, e, d, base, J0, out


def cmd_inject(T, args):
    ids = args or [i for i in INJECT_SET if os.path.exists(f"{INJECT}/{i}.pkl")]
    names = None
    tally = {}
    print("one defect put into real pulls (same windows, same speeds), judged by the other real recordings;")
    print("S* " + ", ".join(f"{t} {T.Sstar[t]:.2f}" for t in GATE_TIERS) + f"; signature E* = {T.Estar:.2f}; floor ripple <= {COMB_MAX:.0f} dB (the physics tier's comb check).")
    print("CAUGHT = a check fails that did not fail on the untouched recording.")
    for i, e, d, base, J0, rows in inject_rows(T, ids):
        if e is None:
            print(f"{i}: run spectral/sp_inject.py {i} first")
            continue
        f0 = {r["check"] for t, r in J0["failed"]}
        names = names or [r[0] for r in rows]
        print(f"\n{i} ({e['family']}, {e['mic']}; {base.n} windows): untouched S / S*: {T.sline(J0)}" + (f" (fails: {', '.join(sorted(f0))})" if f0 else "")
              + ("" if J0["sig"] else ", no other recordings of this engine") + f", ripple {d['original']['ripple']:.1f} dB")
        kk = "even" if base.k == "even" else "other"
        for name, J, new, comb, snew in rows:
            hits = [f"{r['check']} ({show(r['feat'], r['value'])}" + (", rule)" if r["tier"] == "prior" else f", p={r['p']:.1g})") for r in new]
            if comb:
                hits.append(f"comb (ripple {d[name]['ripple']:.1f} dB)")
            sig = [f"{r['feat']} {r['worst']:+.1f} ({r['e']:.1f})" for r in snew][:3]
            own = bool(new) or bool(snew)              # by this tier's checks, without the physics tier's ripple
            caught = own or comb
            for key, hit in ((name, caught), ((name, "own"), own), ((name, kk), caught)):
                tally.setdefault(key, [0, 0])
                tally[key][0] += hit
                tally[key][1] += 1
            print(f"   {name:9s} {'CAUGHT' if caught else 'missed'}  " + ("; ".join(hits[:3]) or "-")
                  + (f"   | signature: {'; '.join(sig) or '-'}" if J["sig"] else ""))
    print("\ndefect     caught     without the comb check     even-firing engines     singles and uneven-firing engines (no order checks apply)")
    for name in names or []:
        a, b = tally[name]
        o, _ = tally[(name, "own")]
        c, dn = tally.get((name, "even"), [0, 0])
        c2, d2 = tally.get((name, "other"), [0, 0])
        print(f"  {name:9s} {a:2d} / {b:<2d}    {o:2d} / {b:<2d}                    {c:2d} / {dn:<2d}                 {c2:2d} / {d2:<2d}")
    print("  ('without the comb check': by this tier's own checks; the floor-ripple check belongs to the physics tier and is run here on the same windows)")
    return 0


PRESETS = ["lfa-main", "honda-k20a", "honda-k20a-tuned", "honda-k20c1", "inline4", "v8-flatplane", "v8-crossplane", "subaru-ej25",
           "vw-sdi", "vw-tdi", "mazda-renesis", "mazda-13b-bridgeport", "single"]
FLEET = ["dom", "halfs", "others", "per1000", "per2000", "per4000", "mF1400", "mF2800", "mF5600", "mR2800", "crest2800", "up1", "up2", "up3",
         "tr_mid", "sharp", "hslope"]


def cmd_presets(T, args):
    print("every preset of the current main build; S* " + ", ".join(f"{t} {T.Sstar[t]:.2f}" for t in GATE_TIERS))
    print(f"{'preset':22s} {'class':7s} {'gate':5s} score / threshold per tier")
    for lab in PRESETS:
        if lab not in T.sim:
            continue
        it = T.sim[lab]
        J = T.judge(it)
        note = ""
        if it.k != "even":
            note = "  [order structure not judged: " + ("single: every half order is a firing harmonic" if it.k == "single" else "uneven firing: no pool of its family") + "]"
        print(f"{lab:22s} {it.k:7s} {verdict(J):5s} {T.sline(J)}{note}")
        for f in fails(J, it.e)[:5]:
            print(f"{'':23s}FAIL  {f}")
        for t, r in J["edge"][:6]:
            print(f"{'':23s}{T.state(r, J['by_sig'])}  {short(r, it.e)} (p={r['p']:.2g})")
    print("\nthe fleet against real engines (pull medians; Mann-Whitney U, two-sided). A feature on which the presets sit to one side of")
    print("real engines is a bias of the physics they share. Even-firing presets against even-firing recordings; then every preset")
    print("against every recording for the features that do not depend on the firing pattern.")
    for title, cls, feats in (("even-firing", ("even",), FLEET), ("all classes", ("even", "single", "uneven"), ["per1000", "per2000", "per4000", "mR2800", "crest2800", "tr_mid", "sharp"])):
        sims = [T.sim[lab] for lab in PRESETS if lab in T.sim and T.sim[lab].k in cls]
        reals = [r for r in T.real.values() if r.k in cls]
        print(f"\n  {title}: {len(sims)} presets against {len(reals)} real outside recordings")
        print(f"  {'feature':10s} {'real median':>12s} {'real p25..p75':>22s} {'presets median':>15s} {'presets p25..p75':>22s} {'shift':>8s} {'presets above real median':>26s} {'p':>8s}")
        for f in feats:
            rv = np.array([r.med(f) for r in reals], float)
            sv = np.array([x.med(f) for x in sims], float)
            rv, sv = rv[np.isfinite(rv)], sv[np.isfinite(sv)]
            if len(rv) < 8 or len(sv) < 4:
                continue
            u = stats.mannwhitneyu(sv, rv, alternative="two-sided")
            q = lambda v: f"{show(f, np.percentile(v, 25)).split()[0]} .. {show(f, np.percentile(v, 75))}"   # noqa: E731
            shift = f"x{10 ** ((np.median(sv) - np.median(rv)) / 20):.1f}" if f in S.LOGF else f"{np.median(sv) - np.median(rv):+.1f}"
            print(f"  {f:10s} {show(f, np.median(rv)):>12s} {q(rv):>22s} {show(f, np.median(sv)):>15s} {q(sv):>22s} {shift:>8s} {int((sv > np.median(rv)).sum()):12d} / {len(sv):<11d} {u.pvalue:8.4f}"
                  + ("  <-- bias" if u.pvalue < 0.01 else ""))
    print("\nper preset (pull medians):")
    print(f"{'preset':22s} " + " ".join(f"{f:>8s}" for f in FLEET))
    for lab in PRESETS:
        if lab in T.sim:
            print(f"{lab:22s} " + " ".join(f"{show(f, T.sim[lab].med(f)).split()[0]:>8s}" if np.isfinite(T.sim[lab].med(f)) else f"{'.':>8s}" for f in FLEET))
    return 0


def cmd_physics(T, args):
    """physics BASE ABL...: change of each feature when one source is switched off, at the SAME held speeds."""
    feats = ["tone", "fund", "dom", "halfs", "others", "sub", "per1000", "per2000", "per4000", "mF1400", "mF2800", "mF5600", "mR2800", "up1", "up2", "up3", "tr_mid", "sharp"]
    for a in args:
        if a.startswith("--cols="):
            feats = a[7:].split(",")
    args = [a for a in args if not a.startswith("--")]
    load = lambda lab: pickle.load(open(f"{rdata.DATA}/spectral/out/sim/{lab}.pkl", "rb"))   # noqa: E731
    base = load(args[0])
    e = base["entry"]
    print(f"{args[0]}: median over the held speeds both renders have, then the change when a source is switched off")
    print("(dB; modulation depths in dB of the index; treble randomness in %; sharpness in acum)")

    def med(rows, speeds):
        by = {}
        for t, rpm, st in rows:
            if rpm in speeds:
                by.setdefault(rpm, []).append(Fe.window_features(st, rpm, e))
        per_speed = [{f: np.nanmedian([S.tr(f, w[f]) for w in ws]) for f in feats} for ws in by.values()]
        return {f: float(np.nanmedian([d[f] for d in per_speed])) for f in feats}

    print(f"  {'':16s} {'speeds (rpm)':22s} " + " ".join(f"{f:>8s}" for f in feats))
    for lab in args[1:]:
        ab = load(lab)
        speeds = sorted({r[1] for r in base["rows"]} & {r[1] for r in ab["rows"]})
        if not speeds:
            continue
        b0, a0 = med(base["rows"], speeds), med(ab["rows"], speeds)
        sp = ",".join(f"{v:.0f}" for v in speeds)
        name = lab.replace(args[0] + "-", "").replace(args[0].split("-")[0] + "-", "")
        print(f"  {'base at':16s} {sp[:22]:22s} " + " ".join(f"{show(f, b0[f]).split()[0]:>8s}" for f in feats))
        print(f"  {name[-16:]:16s} {'change':22s} " + " ".join(f"{a0[f] - b0[f]:+8.1f}" for f in feats))
    return 0


# ------------------------------------------------------------------------- what it takes to fail; self-test
PHYS_MAX = 20 * np.log10(2.0)     # a firing-rate envelope line cannot exceed twice the envelope's mean (a train of narrow pulses): 200 %
CLASS_ENTRY = {"even": dict(family="i4", even=True, engine="__build__"), "single": dict(family="single", even=True, engine="__build__"),
               "uneven": dict(family="vtwin", even=False, engine="__build__")}


def pool_values(T, chk, k, exclude=()):
    name, feat, kind, side, pool, classes, tier, msg = chk
    it = S.Item("__build__", CLASS_ENTRY[k], {}, False)
    ids = S.pool_ids(T.real, pool, it, exclude)
    pv = [T.real[i].med(feat) for i in ids] if kind == "med" else [T.real[i].slope(feat) for i in ids] if kind == "slope" else [a for i in ids for a, _ in T.real[i].bins(feat)]
    pv = np.array(pv, float)
    return pv[np.isfinite(pv)]


def fail_point(T, chk, k, nb=8, exclude=()):
    """The value at which a check starts to fail for a build of class k with nb speed bins:
    (value, pool extreme, pool median, n) or None. Worst-bin checks carry the Bonferroni factor nb."""
    name, feat, kind, side, pool, classes, tier, msg = chk
    pv = pool_values(T, chk, k, exclude)
    if len(pv) < 8 or side == "both":
        return None
    pstar = 10 ** -T.Sstar[tier]
    mult = nb if kind in ("min", "max") else 1
    sgn = 1.0 if side == "hi" else -1.0
    ext = pv.max() if side == "hi" else pv.min()
    f = lambda d: S.tail_p(ext + sgn * d, pv, side, feat)[0] * mult      # noqa: E731   d: distance beyond the real extreme
    lo, hi = -abs(ext - np.median(pv)), 400.0
    if f(hi) >= pstar or f(lo) < pstar:
        return None
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if f(mid) < pstar:
            hi = mid
        else:
            lo = mid
    return ext + sgn * hi, ext, float(np.median(pv)), len(pv)


def cmd_limits(T, args):
    """Where each gate check starts to fail for a build (8 speed bins), beside what real recordings reach."""
    nb = 8
    print(f"where a check starts to fail for a build with {nb} speed bins (p of the check = p* of its tier; worst-bin checks x {nb} bins),")
    print("beside the real pool. 'margin' = distance from the most extreme real value to the failing value.")
    print(f"{'check':48s} {'class':7s} {'n':>3s} {'real median':>12s} {'real extreme':>13s} {'fails beyond':>13s} {'margin':>8s}")
    for chk in S.CHECKS:
        name, feat, kind, side, pool, classes, tier, msg = chk
        if tier not in GATE_TIERS:
            continue
        done = set()
        for k in ("even", "single", "uneven"):
            if k not in classes:
                continue
            fp = fail_point(T, chk, k, nb)
            if fp is None:
                print(f"{name:48s} {k:7s}   . no failing value")
                continue
            v, ext, med, n = fp
            key = (round(v, 3), n)
            if key in done:                     # same pool for every class: one line
                continue
            done.add(key)
            same = pool in ("all", "even") or all(len(pool_values(T, chk, kk)) == n and kk in classes for kk in ("even", "single", "uneven"))
            note = ""
            if feat in S.LOGF and v > PHYS_MAX:
                note = "   <-- cannot fail: an envelope line cannot exceed 200 % of the mean"
            if feat == "mR2800" and v <= 0:
                note = "   <-- cannot fail"
            print(f"{name:48s} {'every' if same and pool != 'even' else k:7s} {n:3d} {show(feat, med):>12s} {show(feat, ext):>13s} {show(feat, v):>13s} "
                  f"{abs(v - ext):6.1f} {'dB' if feat not in S.UNIT else S.UNIT[feat]}{note}")
    print(f"prior: the pitch is the firing order                 even        rule: orders under the firing order <= {Fe.SUB_MAX:.0f} dB re the firing line, every bin")
    print(f"prior: no order louder than the firing series        even        rule: firing dominance >= {S.DOM_MIN:.0f} dB, every bin")
    print(f"signature (engines with at least two outside recordings): a feature more than E* = {T.Estar:.2f} units outside its own recordings' range;")
    print(textwrap.fill("in the feature's own unit (dB unless marked): " + ", ".join(
        f"{f} {T.Estar * v:.0f}" + ("" if f in S.LOGF else {"%": " %", "acum": " acum", "dB/oct": " dB/oct"}.get(S.UNIT.get(f, ""), ""))
        for f, v in T.scales.items() if np.isfinite(v)), width=150, initial_indent="  ", subsequent_indent="  "))
    return 0


def _sabotage(table, feat, value):
    """A copy of a feature table with one feature set to value (judged units) in every speed bin."""
    raw = 100.0 * 10 ** (value / 20.0) if feat in S.LOGF else value
    return {b: dict(row, **{feat: raw}) for b, row in table.items()}


def cmd_seen(T, args):
    """Every check seen failing: (1) sabotage - one feature of an untouched real recording pushed past the
    failing value, the gate must fail with that check; (2) on audio - the builds and injected defects that
    made it fail."""
    print("1. sabotage: one feature of a real recording that passes untouched is pushed 3 audible steps past the failing value")
    print("   (spectral_gate.py limits) in every speed bin; the gate must FAIL and name that check. Recordings of engines without a signature.")
    print(f"   {'check':48s} {'class':7s} {'pushed to':>10s}   gate fails with this check")
    never = []
    for chk in S.CHECKS:
        name, feat, kind, side, pool, classes, tier, msg = chk
        if tier not in GATE_TIERS:
            continue
        for k in ("even", "single", "uneven"):
            if k not in classes:
                continue
            fp = fail_point(T, chk, k)
            if fp is None:
                continue
            sgn = 1.0 if side == "hi" else -1.0
            v = fp[0] + sgn * 3 * S.JND.get(feat, 2.0)
            capped = ""
            if feat in S.LOGF and v > PHYS_MAX:
                v, capped = PHYS_MAX, " (the most a pulse train gives)"
            if feat == "mR2800":
                v = max(v, 0.0)
            ok = n = 0
            for i, it in T.real.items():
                if it.k != k:
                    continue
                J0 = T.judge(it, exclude=(i,))
                if J0["by_sig"] or verdict(J0) == "FAIL":
                    continue
                J = T.judge(S.Item(i, it.e, _sabotage(it.T, feat, v), True), exclude=(i,))
                n += 1
                ok += any(r["check"] == name for t, r in J["failed"])
            if ok < n:
                never.append((name, k))
            print(f"   {name:48s} {k:7s} {show(feat, v):>10s}   {ok:2d} / {n:<2d}{capped}" + ("" if ok == n else "   <-- does not fail"))
    for name, feat, v in (("prior: the pitch is the firing order", "sub", Fe.SUB_MAX + 1.0), ("prior: no order louder than the firing series", "dom", S.DOM_MIN - 1.0)):
        ok = n = 0
        for i, it in T.real.items():
            if it.k != "even" or verdict(T.judge(it, exclude=(i,))) == "FAIL":
                continue
            J = T.judge(S.Item(i, it.e, _sabotage(it.T, feat, v), True), exclude=(i,))
            n += 1
            ok += any(r["check"] == name for t, r in J["failed"])
        print(f"   {name:48s} {'even':7s} {show(feat, v):>10s}   {ok:2d} / {n:<2d}")
    for name, feat, kind, pool, gates, msg in S.NOISE:
        if not gates:
            continue
        ok = n = 0
        v = None
        for i, it in T.real.items():
            J0 = T.judge(it, exclude=(i,), as_build=True)
            if verdict(J0) == "FAIL":
                continue
            r0 = next(r for r in J0["noise"] if r["check"] == name)
            v = r0["bound"] + 3.0
            tb = _sabotage(it.T, feat, v)
            if r0.get("also"):
                tb = _sabotage(tb, "hfx", r0["also"]["bound"] + 3.0)
            J = T.judge(S.Item(i, it.e, tb, True), exclude=(i,), as_build=True)
            n += 1
            ok += any(r["check"] == name for t, r in J["failed"])
        print(f"   {name:48s} {'every':7s} {v:7.1f} dB   {ok:2d} / {n:<2d}" + ("" if ok == n else "   <-- does not fail"))
    sig_ids = [i for i, it in T.real.items() if T.judge(it, exclude=(i,))["by_sig"]]
    hit = {}
    for i in sig_ids:
        it = T.real[i]
        for r in S.signature(it, T.real, T.scales, exclude=(i,)):
            f = r["feat"]
            sd = S.SIG_SIDES.get(f, "hi")
            sgn = 1.0 if sd in ("hi", "both") else -1.0
            v = (r["hi"] if sgn > 0 else r["lo"]) + sgn * (T.Estar + 1.0) * T.scales[f]
            tb = {b: dict(row, **{f: (100.0 * 10 ** (v / 20.0) if f in S.LOGF else v)}) for b, row in it.T.items()}
            J = T.judge(S.Item(i, it.e, tb, True), exclude=(i,))
            hit.setdefault(f, [0, 0])
            hit[f][1] += 1
            hit[f][0] += any(x["feat"] == f for x in J["sfail"])
    bad = [f for f, (a, b) in hit.items() if a < b]
    print(f"   signature: each of its {len(hit)} features pushed E* + 1 units outside the engine's other recordings, on {len(sig_ids)} recordings ({', '.join(sig_ids)}): "
          + ("every feature fails the signature in every recording" if not bad else "NOT failing: " + ", ".join(f"{f} {hit[f][0]}/{hit[f][1]}" for f in bad)))

    print("\n2. on audio: where each check has failed. Builds = the heard and rebuilt LFA builds and the presets; injected = one defect put into")
    print("   a real pull (number of recordings in which it made this check fail, of 18; order checks apply to the 12 even-firing ones).")
    seen = {}
    labs = [v[0] for v in VERDICTS] + [p for p in PRESETS if p not in [v[0] for v in VERDICTS]] + [v[0] for v in TODAY] + [
        x for x in ("subaru-ej25-new", "vw-sdi-new", "vw-tdi-new", "mazda-renesis-new", "mazda-13b-bridgeport-new", "single-new") if x in T.sim]
    for lab in labs:
        if lab not in T.sim:
            continue
        J = T.judge(T.sim[lab])
        for t, r in J["failed"]:
            seen.setdefault(r["check"], ([], {}))[0].append(lab)
        for r in J["sfail"]:
            seen.setdefault("signature: " + r["feat"], ([], {}))[0].append(lab)
    ids = [i for i in INJECT_SET if os.path.exists(f"{INJECT}/{i}.pkl")]
    for i, e, d, base, J0, rows in inject_rows(T, ids):
        for name, J, new, comb, snew in rows:
            for r in new:
                dd = seen.setdefault(r["check"], ([], {}))[1]
                dd[name] = dd.get(name, 0) + 1
            for r in snew:
                dd = seen.setdefault("signature: " + r["feat"], ([], {}))[1]
                dd[name] = dd.get(name, 0) + 1
    order = [c[0] for c in S.CHECKS if c[6] in GATE_TIERS] + ["prior: the pitch is the firing order", "prior: no order louder than the firing series"]
    order += [c[0] for c in S.NOISE if c[4]]
    order += sorted(k for k in seen if k.startswith("signature"))
    silent = []
    for name in order:
        b, dd = seen.get(name, ([], {}))
        if not b and not dd:
            silent.append(name)
        print(f"   {name:48s} builds: {', '.join(b) or '-'}")
        print(f"   {'':48s} injected: {', '.join(f'{k} {v}' for k, v in sorted(dd.items(), key=lambda kv: -kv[1])) or '-'}")
    print("\n   never failed on audio (sabotage only): " + ("; ".join(silent) or "none"))
    return 0


FAMILY_PRESETS = {"rotary2": ["mazda-renesis", "mazda-13b-bridgeport"], "boxer4": ["subaru-ej25"], "v8-flat": ["v8-flatplane"],
                  "diesel": ["vw-tdi", "vw-sdi"], "v8-cross": ["v8-crossplane"], "v10-odd": [], "v10": ["lfa-main"]}
PUB_FEATS = ["dom", "sub", "fund", "halfs", "others", "per1000", "per2000", "per4000", "mF1400", "mF2800", "mF5600", "crest2800", "up1", "up2", "sharp"]


def cmd_public(T, args):
    """The public recordings (Wikimedia Commons, CC BY; microphone position not controlled, engine data
    from the car's name): (1) judged out of sample by the library's pools, (2) families the library lacks."""
    pub = S.load_public()
    if not pub:
        print("no public recordings processed (spectral/sp_public.py fetch; sp_refs.py --public --all)")
        return 0
    import sp_tables as TB
    thin = {i: S.Item(i, e, tb, True) for i, (e, tb, x) in TB.build().get("pub", {}).items() if tb}      # at least one speed bin
    print(f"{len(pub)} public recordings with at least {S.MIN_WINDOWS} accepted pull windows, judged by the library's pools (thresholds unchanged)")
    tot = {}
    rows = []
    for i, it in sorted(pub.items()):
        res = S.judge(it, T.real)
        pr = S.prior(it)
        bad = {t: [r for r in res if r["tier"] == t and r["score"] > T.Sstar[t]] for t in GATE_TIERS}
        if any(r["score"] > 50 for r in pr):
            bad["order"] = bad["order"] + [r for r in pr if r["score"] > 50]
        for t in GATE_TIERS:
            applies = any(r["tier"] == t for r in res)
            if applies:
                k = (it.k, t)
                tot.setdefault(k, [0, 0])
                tot[k][0] += bool(bad[t])
                tot[k][1] += 1
        rows.append((i, it, bad))
    for k in ("even", "single", "uneven"):
        line = [f"{t} {tot[(k, t)][0]} of {tot[(k, t)][1]} beyond S*" for t in GATE_TIERS if (k, t) in tot]
        if line:
            print(f"  {k:7s} " + ", ".join(line))
    print("  the order-tier rejections, most extreme first (the microphone position of these clips is not stated; inside the car or at the")
    print("  engine the cycle orders of single cylinders are heard; the library's pool is tail and exterior microphones only):")
    worst = sorted(((max(r["score"] for r in bad["order"]), i, it, bad["order"]) for i, it, bad in rows if bad["order"]), key=lambda x: -x[0])
    for sc, i, it, b in worst[:12]:
        r = max(b, key=lambda r: r["score"])
        print(f"    {i:34s} {it.e['family']:8s} {short(r)} (p={r['p']:.2g})")
    print("\nfamilies without (or nearly without) studio recordings: public recordings (pull medians) beside the presets")
    fam = lambda it: "diesel" if it.e.get("fuel") == "diesel" else it.e["family"]   # noqa: E731
    for f, presets in FAMILY_PRESETS.items():
        recs = [(i, it) for i, it in sorted(thin.items()) if fam(it) == f]
        if not recs:
            continue
        print(f"\n  {f}: {len(recs)} public recordings with at least one speed bin of {Fe.MIN_N} accepted windows")
        print(f"    {'':34s} bins      n " + " ".join(f"{x:>8s}" for x in PUB_FEATS))
        for i, it in recs:
            print(f"    {i:34s} {''.join(str(b) if b in it.T else '.' for b in range(8))} {it.n:4d} "
                  + " ".join(f"{show(x, it.med(x)).split()[0]:>8s}" if np.isfinite(it.med(x)) else f"{'.':>8s}" for x in PUB_FEATS))
        for lab in presets:
            if lab in T.sim:
                it = T.sim[lab]
                print(f"    {'PRESET ' + lab:34s} {''.join(str(b) if b in it.T else '.' for b in range(8))} {it.n:4d} "
                      + " ".join(f"{show(x, it.med(x)).split()[0]:>8s}" if np.isfinite(it.med(x)) else f"{'.':>8s}" for x in PUB_FEATS))
    return 0


if __name__ == "__main__":
    cmds = {"refs": cmd_refs, "loo": cmd_loo, "gate": cmd_gate, "matrix": cmd_matrix, "inject": cmd_inject, "presets": cmd_presets, "physics": cmd_physics,
            "public": cmd_public, "limits": cmd_limits, "seen": cmd_seen, "today": cmd_today}
    if len(sys.argv) < 2 or sys.argv[1] not in cmds:
        print(__doc__)
        sys.exit(2)
    sys.exit(cmds[sys.argv[1]](Tier(), sys.argv[2:]) or 0)
