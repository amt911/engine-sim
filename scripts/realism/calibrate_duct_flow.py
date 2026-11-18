#!/usr/bin/env python3
"""Calibrate the one duct-flow constant K (crates/engine-core/src/jet_noise.rs, DUCT_FLOW_K)
for every engine at once.

Every preset is rendered held at three points with `sound.duct_flow` = 0 and = 1; the duct
noise adds linearly (it rides the linear waves), so the render at mix g is x0 + g·(x1 − x0)
(checked against a real render at one point by --check). For each g of a sweep the treble's
firing-rhythm modulation (scripts/realism/modulation.py) is measured on every render; the
chosen g* puts the presets' median at the real steady recordings' median. K = K_now · g*²
(the mix scales pressure, K power).

usage: calibrate_duct_flow.py BINARY OUTDIR [--real-median 9.6] [--jobs 6]
       calibrate_duct_flow.py BINARY OUTDIR --check PRESET RPM LOAD G
"""
import glob, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modulation as M  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
POINTS = [(1500, 0.1), (3000, 0.3), (6000, 1.0)]
SECONDS = 3
GAINS = [0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 5.0, 7.0, 10.0, 20.0, 30.0]


def with_mix(yaml_text, g):
    """The preset with `sound.duct_flow: g` (added under `sound:` or replaced)."""
    lines = yaml_text.splitlines()
    out, done = [], False
    for ln in lines:
        if ln.strip().startswith("duct_flow:"):
            ln = ln[: len(ln) - len(ln.lstrip())] + f"duct_flow: {g}"
            done = True
        out.append(ln)
        if not done and ln.rstrip() == "  sound:":
            out.append(f"    duct_flow: {g}")
            done = True
    if not done:
        raise ValueError("no sound: section")
    return "\n".join(out) + "\n"


MODEL = None


def with_model(yaml_text):
    """The exhaust's own model line (4-space indent under `exhaust:`) set to MODEL."""
    if MODEL is None:
        return yaml_text
    return yaml_text.replace("\n    model: waveguide\n", f"\n    model: {MODEL}\n")


def render(binary, preset, rpm, load, g, outdir):
    name = f"{os.path.basename(preset)[:-5]}_{rpm}_{load}_{g}"
    wav = os.path.join(outdir, name + ".wav")
    if os.path.exists(wav):
        return wav
    yml = os.path.join(outdir, name + ".yaml")
    with open(preset) as f:
        text = with_model(with_mix(f.read(), g))
    with open(yml, "w") as f:
        f.write(text)
    subprocess.run([binary, "--engine", yml, "--render", f"held:{rpm}:{load}:{SECONDS}",
                    "--out", wav], check=True, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    return wav


def main(argv):
    global MODEL
    binary, outdir = argv[0], argv[1]
    MODEL = argv[argv.index("--model") + 1] if "--model" in argv else None
    os.makedirs(outdir, exist_ok=True)
    jobs = int(argv[argv.index("--jobs") + 1]) if "--jobs" in argv else 6
    presets = sorted(glob.glob(os.path.join(ROOT, "presets", "*.yaml")))
    if "--check" in argv:
        i = argv.index("--check")
        p = os.path.join(ROOT, "presets", argv[i + 1] + ".yaml")
        rpm, load, g = int(argv[i + 2]), float(argv[i + 3]), float(argv[i + 4])
        x0, x1, xg = (M.load(render(binary, p, rpm, load, v, outdir)) for v in (0.0, 1.0, g))
        y = x0 + g * (x1 - x0)
        n = min(len(y), len(xg))
        err = np.sqrt(np.mean((y[:n] - xg[:n]) ** 2) / np.mean(xg[:n] ** 2))
        print(f"superposition at g={g}: rms error {100 * err:.2f} % of the render")
        print("superposed", M.treble_mod(M.measure(y[M.FS:])[0]),
              "rendered", M.treble_mod(M.measure(xg[M.FS:])[0]))
        return
    real = float(argv[argv.index("--real-median") + 1]) if "--real-median" in argv else 9.6
    todo = [(p, r, l, g) for p in presets for r, l in POINTS for g in (0.0, 1.0)]
    with ThreadPoolExecutor(jobs) as ex:
        list(ex.map(lambda a: render(binary, *a, outdir), todo))
    table = {}
    for p in presets:
        for r, l in POINTS:
            x0 = M.load(render(binary, p, r, l, 0.0, outdir))[M.FS:]
            x1 = M.load(render(binary, p, r, l, 1.0, outdir))[M.FS:]
            n = min(len(x0), len(x1))
            table[(os.path.basename(p)[:-5], r, l)] = [
                M.measure(x0[:n] + g * (x1[:n] - x0[:n])) for g in GAINS
            ]
    medians = [np.median([M.treble_mod(v[k][0]) for v in table.values()]) for k in range(len(GAINS))]
    print("g      " + " ".join(f"{g:6.2f}" for g in GAINS))
    print("median " + " ".join(f"{m:6.1f}" for m in medians))
    treble = [np.median([v[k][1][2] for v in table.values()]) for k in range(len(GAINS))]
    print("3-8kHz " + " ".join(f"{m:6.1f}" for m in treble) + "   (real median -26.0 dB)")
    # the median falls as g grows: interpolate where it crosses the real median
    k = next((i for i in range(1, len(GAINS)) if medians[i] <= real), None)
    if k is None:
        print(f"no g in the sweep brings the median to {real} %")
        return
    g0, g1, m0, m1 = GAINS[k - 1], GAINS[k], medians[k - 1], medians[k]
    g_star = g0 + (g1 - g0) * (m0 - real) / (m0 - m1)
    print(f"g* = {g_star:.3f} (median {real} %)")
    kk = min(range(len(GAINS)), key=lambda i: abs(GAINS[i] - g_star))
    print(f"\nper render at g = {GAINS[kk]} (treble %, levels re 50–300 Hz dB):")
    for key, v in sorted(table.items()):
        mod, lev = v[kk]
        mod0, _ = v[0]
        print(f"{key[0]:22s} {key[1]:5d} {key[2]:4.1f}  {M.treble_mod(mod0):6.1f} -> "
              f"{M.treble_mod(mod):6.1f}   " + " ".join(f"{u:6.1f}" for u in lev))


if __name__ == "__main__":
    main(sys.argv[1:])
