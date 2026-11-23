"""regress.py SUFFIX BASE — a build may not get audibly worse than the main it replaces.

The realism gates judge each build against absolute bounds from real recordings. When main already
fails a bound, a change that makes it worse still reads FAIL -> FAIL and slipped through: PR #147
raised the K20A's broadband wind from -15.5 to -10.6 dB (EJ25 -12.6 -> -8.0), the gate summary kept
the same FAIL list, and the user heard it at once (2026-11-15: "ahora suena viento de nuevo").

This check compares the numbers, not the verdicts, of gates-SUFFIX.txt against gates-BASE.txt for
every preset both judged. A step counts as audible at 1 dB, the just-noticeable level difference for
broadband noise (Zwicker & Fastl, Psychoacoustics, 3rd ed., sec. 7.1: ~0.5-1 dB). Static (4-10 kHz
kurtosis, dimensionless) may not rise by more than 25 % plus 0.5.

Exit 1 when any preset regresses; prints one line per preset.
"""
import os
import re
import sys

import rdata

STEP_DB = 1.0
# The lift is the median over a render's throttle lifts; a median of fewer than three is one or two
# events at moments the other build need not share (2026-11-23: main's K20A lift was a single event).
MIN_LIFTS = 3
LIFTS = re.compile(r"^(\S+)\s+(?:PASS|FAIL)\s+bank .*\blifts (\d+)")
METRICS = {  # name: (regex on the summary, higher-is-worse)
    "wind": (re.compile(r"^(\S+) \(.*?wind (-?[\d.]+)/"), True),
    "lowmid": (re.compile(r"^(\S+) \(.*?lowmid (-?[\d.]+)/"), True),
    "hump": (re.compile(r"^(\S+) \(.*? hump (-?[\d.]+)/"), True),
    "lift": (re.compile(r"^(\S+)\s+(?:PASS|FAIL)\s+bank .*?lift\s+([-+][\d.]+)>"), True),
    "static": (re.compile(r"^(\S+) static: (?:PASS|FAIL) (-?[\d.]+) <="), True),
}


def read(sfx):
    path = os.path.join(rdata.DATA, "renders", f"gates-{sfx}.txt")
    out = {}
    with open(path) as f:
        for line in f:
            for name, (rx, _) in METRICS.items():
                m = rx.match(line)
                if m and m.group(1).endswith("-" + sfx):
                    preset = m.group(1)[: -len(sfx) - 1]
                    out.setdefault(preset, {})[name] = float(m.group(2))
            m = LIFTS.match(line)
            if m and m.group(1).endswith("-" + sfx):
                out.setdefault(m.group(1)[: -len(sfx) - 1], {})["lifts"] = int(m.group(2))
    return out


def worse(name, new, base):
    if name == "static":
        return new > base * 1.25 + 0.5
    return new > base + STEP_DB


def main(sfx, base):
    a, b = read(sfx), read(base)
    for r in (a, b):
        for p in r.values():   # an old summary without the count keeps being judged
            if p.get("lifts", MIN_LIFTS) < MIN_LIFTS:
                p.pop("lift", None)
    rc = 0
    for preset in sorted(set(a) & set(b)):
        bad = [f"{n} {b[preset][n]:+.1f} -> {a[preset][n]:+.1f}"
               for n in METRICS if n in a[preset] and n in b[preset] and worse(n, a[preset][n], b[preset][n])]
        print(f"{preset}-{sfx} regress vs {base}: {'FAIL ' + '; '.join(bad) if bad else 'PASS'}")
        rc |= bool(bad)
    if not set(a) & set(b):
        print(f"regress: no preset judged by both {sfx} and {base}")
        return 1
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
