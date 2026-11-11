#!/usr/bin/env python3
"""expect.py — what an engine should sound like, from its definition, and whether its renders do.

The other tiers judge a render against real recordings. This one starts from the engine file itself
(any engine the editor can build: diesel, rotary, odd cylinder counts, any exhaust grouping) and derives
what physics says its sound must be, then judges the render against that and against whatever real
recordings of its kind exist. For an engine with no recording it says what can and cannot be judged.

  FIRING PATTERN (exact kinematics)
    chambers n, revolutions per cycle R (2: four-stroke piston; 3: a Wankel rotor's face), firings per
    revolution n / R, the firing instants (even, in the firing order: the simulator's rule), which
    chambers share an outlet (collectors, merge), pipe-length differences, tails apart seen off the axis.
    Structure factor of cycle order k at a speed:  S_k = | sum_i exp(-j 2 pi f_k t_i) |^2 / n^2,
    t_i = firing instant + its pipe's and its tail's extra delay.  S_k = 1: every chamber's pulse adds
    (the firing order and its multiples); S_k = 0: they cancel exactly (an order the pattern cannot make).
      expected orders   S_k >= -6 dB      forbidden orders   S_k <= -25 dB
    Judged on held renders (speed known):
      K1  energy on forbidden orders re the expected ones            rule: <= what real even-firing engines
          (cylinder-to-cylinder scatter is all that can put it there)       carry on their non-firing orders
      K2  the strongest low order is an expected one                 rule: yes (else the pitch is not the
                                                                            engine's)
      K3  lines between the cycle's orders (off the 1/R grid)        rule: none over -20 dB (another period
                                                                            in the model)
  COMBUSTION SYSTEM (Anderton, SAE 790270: combustion index n and cylinder-pressure level at the
    reference frequency, 26 engines): petrol n = 6.3, 130 dB; NA direct-injection diesel 3.4, 156 dB;
    turbocharged diesel 5.1, 144 dB.  Combustion noise ~ N^n: 3.01 n dB per octave of speed.
    Printed with what the render does (level against speed of 1-3 kHz, load span); not a gate: the ear
    behind the car hears the exhaust and the bay's loss with it, and no exterior pool of diesels exists.
  THE OTHER TIERS, with an entry derived from the definition (no table of presets needed):
    physics (crescendo, load, lift-off, comb), order / texture / noise (spectral_gate.py).

usage
  expect.py describe ENGINE.yaml|json                 the expectations, no audio
  expect.py judge ENGINE.yaml|json LABEL              + the renders of LABEL (renders/LABEL or wind/renders/LABEL)
  expect.py presets                                   every preset's new-build renders (renders/<preset>-new)
"""
import glob
import json
import os
import re
import subprocess
import sys
import warnings

warnings.filterwarnings("ignore")
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/spectral")
sys.path.insert(0, ROOT + "/wind")
sys.path.insert(0, ROOT + "/tools")
import numpy as np  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # rdata
import rdata  # noqa: E402

BIN = os.environ.get("ENGINE_SIM_BIN", os.path.join(rdata.DATA, "..", "verdicts", "engine-sim"))
PHYSGATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "physgate.py")       # read only
C_PRIMARY, C_COLLECTOR, C_AIR = 600.0, 560.0, 343.0       # the simulator's own (exhaust.rs), m/s
EXPECT_DB, FORBID_DB = -6.0, -25.0
OFFGRID_MAX = -20.0
# Anderton 1979, conclusions (c): combustion index n, cylinder-pressure level at the reference frequency CPR(1.0), dB
ANDERTON = {"petrol": (6.3, 130.0), "diesel-na-di": (3.4, 156.0), "diesel-turbo": (5.1, 144.0), "diesel-na-idi": (4.45, 145.0)}
PRESETS = ["single", "inline4", "honda-k20a", "honda-k20a-tuned", "honda-k20c1", "subaru-ej25", "vw-sdi", "vw-tdi", "mazda-renesis",
           "mazda-13b-bridgeport", "lfa-v10", "v8-flatplane", "v8-crossplane"]
RENDER_LABEL = {"lfa-v10": "lfa-new"}


def load_def(path):
    """The engine definition as a dict (a yaml goes through the simulator's own --save to json)."""
    if path.endswith(".json"):
        return json.load(open(path))["engine"]
    out = os.path.join(rdata.DATA, "wind", "defs", "_" + os.path.basename(path).rsplit(".", 1)[0] + ".json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(path):
        subprocess.run([BIN, "--engine", path, "--save", out], check=True, capture_output=True)
    return json.load(open(out))["engine"]


def derive(e):
    """The firing pattern and the facts the tiers need, from the definition."""
    n = int(e["cylinders"])
    rotor = e["geometry"].get("rotor")
    R = 3 if rotor else 2
    order = list(e["firing_order"])[:n] or list(range(1, n + 1))
    slot = {c: j for j, c in enumerate(order)}
    ex = e["exhaust"]
    groups = [list(g) for g in ex.get("collectors") or []] or [list(range(1, n + 1))]
    listed = {c for g in groups for c in g}
    rest = [c for c in range(1, n + 1) if c not in listed]
    if rest:
        groups.append(rest)
    merged = ex.get("merge") is not None or len(groups) == 1
    dev = list(ex.get("primary_length_deviation_m") or []) + [0.0] * n
    extra = list(ex.get("collector_extra_length_m") or []) + [0.0] * len(groups)
    spacing = float(ex.get("tail_spacing_m") or 0.0)
    angle = np.radians(float(e.get("sound", {}).get("listener_angle_deg", 30.0)))
    G = len(groups)
    tail_delay = [0.0] * G if merged else [-(g - (G - 1) / 2.0) * spacing * np.sin(angle) / C_AIR for g in range(G)]
    cyc = np.zeros(n)          # firing instant as a fraction of the cycle
    fixed = np.zeros(n)        # extra delay, s
    for g, grp in enumerate(groups):
        for c in grp:
            cyc[c - 1] = slot.get(c, 0) / n
            fixed[c - 1] = dev[c - 1] / C_PRIMARY + extra[g] / C_COLLECTOR + tail_delay[g]
    comb = e.get("combustion") or {}
    diesel = comb.get("kind") == "diesel"
    turbo = bool(e.get("turbo"))
    g = e["geometry"]
    if rotor:
        swept = 3 * np.sqrt(3) * rotor["eccentricity_m"] * rotor["generating_radius_m"] * rotor["width_m"]      # per chamber (Yamamoto)
        disp = swept * (n / 3) * 1.0          # one chamber's swept volume per rotor per shaft revolution: the usual rating
    else:
        swept = np.pi / 4 * g["bore_m"] ** 2 * g["stroke_m"]
        disp = swept * n
    d = dict(n=n, R=R, rotor=bool(rotor), fpr=n / R, order=order, groups=groups, merged=merged, cyc=cyc, fixed=fixed, diesel=diesel, turbo=turbo,
             redline=float(e["redline_rpm"]), idle=float(e.get("idle_rpm", 800.0)), bore=g["bore_m"], cr=g["compression_ratio"],
             swept=swept, disp=disp, name=e.get("name", "?"), spacing=spacing, angle=float(np.degrees(angle)), dev=dev[:n], extra=extra[:G],
             tail_d=float(ex.get("tail_diameter_m", 0.05)))
    d["system"] = ("diesel-turbo" if turbo else "diesel-na-di") if diesel else "petrol"
    # even at the outlets? no cycle order under 4x the firing order, other than its multiples, within 20 dB of full strength
    S = structure(d, 0.75 * d["redline"], 4 * n)
    non = [S[k - 1] for k in range(1, 4 * n + 1) if k % n]
    d["even"] = bool(n == 1 or (max(non) if non else 0.0) < 10 ** (-20 / 10))
    d["klass"] = "single" if n == 1 else ("even" if d["even"] else "uneven")
    fam = ("single" if n == 1 else f"rotary{n // 3}" if rotor else f"{'v' if len(groups) > 1 else 'i'}{n}") + ("-diesel" if diesel else "")
    d["family"] = fam
    return d


def structure(d, rpm, kmax):
    """S_k for cycle orders k = 1..kmax at rpm (1: all the chambers' pulses add; 0: they cancel)."""
    T = d["R"] * 60.0 / rpm
    k = np.arange(1, kmax + 1)[:, None]
    ph = 2 * np.pi * k * (d["cyc"][None, :] + d["fixed"][None, :] / T)
    return np.abs(np.exp(-1j * ph).sum(axis=1)) ** 2 / d["n"] ** 2


def order_name(k, R):
    from fractions import Fraction
    fr = Fraction(k, R)
    return str(fr.numerator) if fr.denominator == 1 else f"{fr.numerator}/{fr.denominator}"


def describe(d):
    L = []
    R, n = d["R"], d["n"]
    L.append(f"{d['name']}")
    kind = f"{n // 3}-rotor Wankel ({n} chambers, each a four-stroke cycle in three shaft revolutions)" if d["rotor"] else f"{n}-cylinder four-stroke"
    L.append(f"  definition: {kind}; {'diesel' if d['diesel'] else 'spark ignition'}{', turbocharged' if d['turbo'] else ', naturally aspirated'}; "
             f"compression {d['cr']:.1f}; swept {d['disp'] * 1e3:.2f} L; redline {d['redline']:.0f} rpm")
    out = "one outlet" if d["merged"] else f"{len(d['groups'])} tails {d['spacing']:.2f} m apart, heard {d['angle']:.0f} deg off the axis"
    L.append(f"  exhaust: {len(d['groups'])} collector group(s) {d['groups']}, {out}; primary length differences "
             + (", ".join(f"{1000 * v:+.0f}" for v in d["dev"]) + " mm" if any(d["dev"]) else "none")
             + (f"; collector extra lengths {', '.join(f'{1000 * v:.0f}' for v in d['extra'])} mm" if any(d["extra"]) else ""))
    L.append(f"  firing: {d['fpr']:g} per revolution, evenly spaced in the order {d['order']} (the simulator's rule): the firing order is engine order {d['fpr']:g};")
    L.append(f"          pitch {d['fpr'] * d['idle'] / 60:.0f} Hz at idle, {d['fpr'] * d['redline'] / 60:.0f} Hz at the redline")
    for frac in (0.5, 1.0):
        rpm = frac * d["redline"]
        S = structure(d, rpm, 3 * n)
        exp = [f"{order_name(k, R)} ({10 * np.log10(S[k - 1] + 1e-12):+.0f})" for k in range(1, 3 * n + 1) if 10 * np.log10(S[k - 1] + 1e-12) >= EXPECT_DB]
        mid = [f"{order_name(k, R)} ({10 * np.log10(S[k - 1] + 1e-12):+.0f})" for k in range(1, 3 * n + 1) if FORBID_DB < 10 * np.log10(S[k - 1] + 1e-12) < EXPECT_DB]
        nforb = sum(1 for k in range(1, 3 * n + 1) if 10 * np.log10(S[k - 1] + 1e-12) <= FORBID_DB)
        L.append(f"  at {rpm:.0f} rpm the pattern makes engine orders (dB re all chambers adding): " + ", ".join(exp[:8]))
        L.append(f"          weaker but allowed: " + (", ".join(mid[:10]) or "none") + f"; forbidden (cancel): {nforb} of the {3 * n} lowest cycle orders")
    if d["klass"] == "even":
        L.append("  -> even at the outlets: the firing order is the pitch; every other order is cylinder-to-cylinder scatter (order tier applies)")
    elif d["klass"] == "uneven":
        L.append("  -> uneven at the outlets (banks or pipes of unequal length heard apart): orders under the firing order are part of this engine's sound"
                 " (rumble / burble); the order tier's even-firing pool does not apply, the kinematic checks do")
    else:
        L.append("  -> one chamber: every cycle order is a harmonic of its own firing; nothing is forbidden")
    idx, cpr = ANDERTON[d["system"]]
    L.append(f"  combustion system ({d['system']}; Anderton, SAE 790270): combustion index {idx}, cylinder-pressure level at the reference frequency {cpr:.0f} dB"
             f" -> combustion noise rises {3.01 * idx:.1f} dB per octave of speed;")
    if d["diesel"]:
        L.append("          a diesel lights its charge at once: 26 dB (NA, direct injection) / 14 dB (turbocharged) more excitation than a petrol engine's at the"
                 " reference frequency, a 1-3 kHz knock on every firing, most audible at idle and light load; " + ("the turbine takes the blowdown: a softer tail"
                 if d["turbo"] else "no throttle: the intake is not a load cue"))
    if d["rotor"]:
        L.append("          ports, no valves: no seat ticks; a peripheral or bridge port opens in a few degrees (a sharp blowdown, harmonics that fall slowly: 'buzz');"
                 " lines on half-integer shaft orders have no source (the cycle is three revolutions: thirds)")
    return L


def kinematic(d, files):
    """K1-K3 on full-load held renders, over the cycle orders up to four times the firing order (where a held render's
    lines are sharp and the pattern, not the pipes' fine structure, decides):
    [(rpm, K1 dB, strongest low cycle order, its S dB, K3 dB)]."""
    import wn
    R, n = d["R"], d["n"]
    rows = []
    for p in files:
        rpm = float(re.search(r"held_(\d+)_", p).group(1))
        f_half = rpm / 60.0 / (2 * R)                       # half a cycle order
        kmax = 4 * n
        f, P, F, L = wn.held_split(wn.load_pa(p), rpm, sub=2 * R, fmax=(2 * kmax + 2) * f_half)
        df = f[1] - f[0]
        c = np.round(np.arange(1, 2 * kmax + 1) * f_half / df).astype(int)
        e = np.array([L[i - 1:i + 2].sum() for i in c])
        on = e[1::2]                       # the cycle's orders k = 1 .. kmax
        off = e[0::2]                      # half-way between them: no engine cycle puts a line there
        SdB = 10 * np.log10(structure(d, rpm, kmax) + 1e-12)
        forb = SdB <= FORBID_DB
        e_allow = on[~forb].sum() + 1e-30
        k1 = float(np.clip(10 * np.log10(on[forb].sum() / e_allow + 1e-9), -60, 40)) if forb.any() else np.nan
        low = np.arange(1, min(2 * n, kmax) + 1)
        ks = int(low[np.argmax(on[low - 1])])
        k3 = float(np.clip(10 * np.log10(off.sum() / e_allow + 1e-9), -60, 40))
        rows.append((rpm, k1, ks, float(SdB[ks - 1]), k3))
    return rows


def band_levels(files, lo, hi):
    """{rpm or load: (dB SPL of lo..hi Hz, dB(A) overall)} of held renders."""
    import wn
    out = {}
    for p in files:
        m = re.search(r"held_(\d+)_([0-9.]+)_", p)
        x = wn.load_pa(p)[wn.FS:]
        from scipy import signal
        f, P = signal.welch(x, wn.FS, "hann", 8192, 4096, scaling="density")
        P = P * (f[1] - f[0])
        out[(float(m.group(1)), float(m.group(2)))] = (float(wn.spl(P[(f >= lo) & (f < hi)].sum())), float(wn.spl(wn.total(f, P, weight="A"))))
    return out


def render_dir(label):
    for r in (rdata.DATA + "/renders", rdata.DATA + "/wind/renders"):
        if os.path.isdir(f"{r}/{label}"):
            return f"{r}/{label}"
    return None


def register(label, d):
    """Make the label known to the spectral tier (spectral/extra_spec.json), with the entry the definition gives."""
    p = rdata.DATA + "/spectral/extra_spec.json"
    reg = json.load(open(p)) if os.path.exists(p) else {}
    entry = dict(fpr=d["fpr"], redline=d["redline"], family=d["family"], even=bool(d["klass"] != "uneven"), fuel="diesel" if d["diesel"] else "petrol",
                 asp="turbo" if d["turbo"] else "na", engine=label)
    if reg.get(label) != entry:
        reg[label] = entry
        json.dump(reg, open(p, "w"), indent=1)
    return entry


_T = None


def tier():
    global _T
    if _T is None:
        import spectral_gate as G
        _T = (G, G.Tier())
    return _T


def judge(path, label, quiet=False):
    e = load_def(path)
    d = derive(e)
    out = describe(d)
    rd = render_dir(label)
    if rd is None:
        return out + [f"  no renders for {label}"], None
    full = sorted(glob.glob(f"{rd}/held_*_1.0_*.wav"), key=lambda p: float(re.search(r"held_(\d+)_", p).group(1)))
    out.append(f"  renders: {rd.replace(rdata.DATA + '/', '')} ({len(full)} full-load speeds)")
    res = {}
    # ---- kinematics
    K = kinematic(d, full)
    import sp_stat as S
    G, T = tier()
    ev = [r for r in T.real.values() if r.k == "even"]
    nonf = np.array([r.med("nonf") for r in ev])
    nonf = nonf[np.isfinite(nonf)]
    bound = float(nonf.max() + 1.0)
    k1 = [r[1] for r in K if np.isfinite(r[1])]
    k1m = float(np.median(k1)) if k1 else np.nan
    k3m = float(np.max([r[4] for r in K])) if K else np.nan
    out.append(f"  kinematics (held full-load renders, cycle orders up to engine order {4 * d['fpr']:g}, four times the firing order):")
    if k1:
        ok = k1m <= bound
        res["K1"] = ok
        worst = max(K, key=lambda r: r[1] if np.isfinite(r[1]) else -99)
        out.append(f"    {'ok  ' if ok else 'FAIL'} K1 orders the pattern cannot make, re the ones it makes: median {k1m:.1f} dB over {len(k1)} speeds (worst {worst[1]:.1f} dB at {worst[0]:.0f} rpm);"
                   f" real even-firing engines carry {np.median(nonf):.1f} dB (median) to {nonf.max():.1f} dB on their non-firing orders (n={len(nonf)}); rule <= {bound:.1f} dB")
        if not ok:
            out.append("         -> orders this firing pattern and pipe layout cannot make are as strong as in no real engine: a source that does not follow the"
                       " definition (a bank's side of the block heard apart, a period in the model that is not the cycle's)")
    else:
        out.append("    -    K1 nothing is forbidden for this pattern (one chamber, or every order allowed)")
    bad = [r for r in K if r[3] <= FORBID_DB]
    weak = [r for r in K if FORBID_DB < r[3] < EXPECT_DB]
    res["K2"] = not bad
    if bad:
        out.append("    FAIL K2 the strongest low order is one the pattern cannot make: " + ", ".join(f"order {order_name(r[2], d['R'])} at {r[0]:.0f} rpm" for r in bad)
                   + ": the pitch heard is not this engine's")
    else:
        strong = sorted({order_name(r[2], d["R"]) for r in K}, key=lambda t: eval(t))
        out.append(f"    ok   K2 the strongest low order is one the pattern makes at every speed (engine order {', '.join(strong)})"
                   + (f"; at {', '.join(f'{r[0]:.0f}' for r in weak)} rpm it is one the pattern makes only weakly ("
                      + ", ".join(sorted({f"order {order_name(r[2], d['R'])}: S {r[3]:.0f} dB" for r in weak})) + "): the pipes carry it over the firing order" if weak else ""))
    res["K3"] = bool(k3m <= OFFGRID_MAX)
    out.append(f"    {'ok  ' if res['K3'] else 'FAIL'} K3 lines between the cycle's orders: worst {k3m:.1f} dB re the orders the pattern makes (rule <= {OFFGRID_MAX:.0f} dB)"
               + ("" if res["K3"] else "  -> something in the model repeats with another period than the engine's cycle"))
    # ---- combustion system: what the render does, beside Anderton
    idx, cpr = ANDERTON[d["system"]]
    lv = band_levels(full, 1000.0, 3150.0)
    rp = np.array(sorted(k[0] for k in lv))
    if len(rp) >= 4:
        b = np.array([lv[(r, 1.0)][0] for r in rp])
        a = np.array([lv[(r, 1.0)][1] for r in rp])
        s_band = float(np.polyfit(np.log2(rp), b, 1)[0])
        s_all = float(np.polyfit(np.log2(rp), a, 1)[0])
        out.append(f"  combustion system: 1-3 kHz rises {s_band:+.1f} dB per octave of speed at full load, the whole sound {s_all:+.1f} dB(A) per octave"
                   f" (Anderton's index for {d['system']}: {3.01 * idx:.1f} dB per octave for the combustion noise alone; real fixed-gain pulls: median +10.6 dB(A))")
    part = sorted(glob.glob(f"{rd}/held_*_*_*.wav"))
    pl = {}
    for p in part:
        m = re.search(r"held_(\d+)_([0-9.]+)_", p)
        pl.setdefault(float(m.group(1)), []).append((float(m.group(2)), p))
    for rpm, lst in pl.items():
        if len(lst) >= 3:
            lev = band_levels([p for l, p in lst], 1000.0, 3150.0)
            lo, hi = min(l for l, p in lst), max(l for l, p in lst)
            out.append(f"          load at {rpm:.0f} rpm, full over {'no fuel' if d['diesel'] else 'closed throttle'}: 1-3 kHz {lev[(rpm, hi)][0] - lev[(rpm, lo)][0]:+.1f} dB, whole sound"
                       f" {lev[(rpm, hi)][1] - lev[(rpm, lo)][1]:+.1f} dB(A)" + ("  (a diesel's knock is set by its ignition delay, not by its load: Anderton's NA diesels are combustion-noise"
                       " engines at every load; the tail and the bay are what the ear behind the car adds)" if d["diesel"] else ""))
    # ---- the other tiers
    pg = rd + ".physgate.txt"
    if not os.path.exists(pg):
        r = subprocess.run([sys.executable, "-I", PHYSGATE, "judge", rd, str(int(d["redline"]))], capture_output=True, text=True)
        open(pg, "w").write(r.stdout + r.stderr)
    ptxt = open(pg).read().strip().split("\n")
    res["physics"] = not any(l.startswith("FAIL") for l in ptxt)
    out.append("  physics tier: " + ("PASS" if res["physics"] else "FAIL") + "  [" + "; ".join(re.sub(r"\s+", " ", l) for l in ptxt if l[:4] in ("PASS", "FAIL") and len(l) > 5) + "]")
    if d["diesel"] and any(l.startswith("FAIL") and "load" in l for l in ptxt):
        out.append("         note: the load bound (>= 3 dB(A)) comes from petrol engines, whose closed throttle starves the cylinders; a diesel has no throttle and pumps the same"
                   " air with no fuel. No real diesel session with a fixed gain is in the library to set a diesel bound: read this FAIL as 'smaller than any petrol', not as proven wrong")
    if label not in T.sim:
        register(label, d)
        import importlib
        import sp_sim
        import wfeat
        importlib.reload(sp_sim)
        print(sp_sim.run(label), "|", wfeat.run_sim(label), file=sys.stderr)
        import sp_tables as TB
        TB.build(force=True)
        global _T
        _T = None
        G, T = tier()
    it = T.sim[label]
    if it.k != d["klass"]:
        out.append(f"  note: the tier's table of presets lists this engine as '{it.k}'; its definition makes it '{d['klass']}' at the listener. Judged below as its"
                   f" definition says" + (" (as an even-firing engine with one outlet it would fail the order tier for orders its two tails make by geometry)" if it.k == "even" else ""))
        it = S.Item(label, dict(it.e, even=bool(d["klass"] != "uneven"), family=("single" if d["klass"] == "single" else it.e["family"])), it.T, False)
    J = T.judge(it)
    v = G.verdict(J)
    res["spectral"] = v == "PASS"
    out.append(f"  spectral tiers (judged as {it.k}): {v}   {T.sline(J)}")
    for f in G.fails(J, it.e)[:6]:
        out.append(f"      FAIL {f}")
    # ---- recordings of its kind (any microphone, any number of windows: a look, not a pool)
    def kin(e2):
        if d["diesel"]:
            return e2.get("fuel") == "diesel"
        if d["rotor"]:
            return "rotary" in e2["family"]
        return False
    import sp_tables as TB
    tb = TB.build()
    feats = ["dom", "sub", "halfs", "others", "per1000", "mF1400", "nz28", "hfprom"]
    rows = []
    for kind_ in ("ref", "pub"):
        for i, (e2, tab, x) in tb[kind_].items():
            if kin(e2) and tab:
                itx = S.Item(i, e2, tab, True)
                rows.append((i, e2.get("mic", "?") if kind_ == "ref" else "public", itx))
    if d["diesel"] or d["rotor"]:
        out.append(f"  recordings of its kind with at least one speed bin of {6} accepted pull windows ({'diesels' if d['diesel'] else 'rotaries'}; any microphone - a look, not a pool):")
        out.append(f"      {'':34s} {'mic':7s} {'bins':>4s} " + " ".join(f"{f:>8s}" for f in feats))
        for i, mic, itx in rows:
            out.append(f"      {i[:34]:34s} {mic:7s} {len(itx.T):4d} " + " ".join(f"{G.show(f, itx.med(f)).split()[0]:>8s}" if np.isfinite(itx.med(f)) else f"{'.':>8s}" for f in feats))
        if not rows:
            out.append("      none")
        out.append(f"      {'THIS RENDER':34s} {'3 m':7s} {len(it.T):4d} " + " ".join(f"{G.show(f, it.med(f)).split()[0]:>8s}" if np.isfinite(it.med(f)) else f"{'.':>8s}" for f in feats))
    else:
        out.append("  recordings of its kind: judged against the library's pools (above); no family table printed")
    res["verdict"] = all(res.get(k, True) for k in ("K1", "K2", "K3", "physics", "spectral"))
    out.append(f"  => {'PASS' if res['verdict'] else 'FAIL'}: " + ", ".join(f"{k} {'ok' if res[k] else 'FAIL'}" for k in ("K1", "K2", "K3", "physics", "spectral") if k in res))
    return out, res


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "describe":
        print("\n".join(describe(derive(load_def(sys.argv[2])))))
    elif cmd == "judge":
        lines, res = judge(sys.argv[2], sys.argv[3])
        print("\n".join(lines))
        sys.exit(0 if res and res["verdict"] else 1)
    elif cmd == "presets":
        rows = []
        for p in PRESETS:
            lab = RENDER_LABEL.get(p, p + "-new")
            lines, res = judge(f"{rdata.DATA}/wind/defs/{p}.json", lab)
            print("\n".join(lines) + "\n")
            rows.append((p, res))
        print(f"{'preset':22s} " + " ".join(f"{k:>9s}" for k in ("K1", "K2", "K3", "physics", "spectral", "verdict")))
        for p, res in rows:
            if res:
                print(f"{p:22s} " + " ".join(f"{('ok' if res[k] else 'FAIL') if k in res else '-':>9s}" for k in ("K1", "K2", "K3", "physics", "spectral", "verdict")))
    else:
        print(__doc__)
        sys.exit(2)
