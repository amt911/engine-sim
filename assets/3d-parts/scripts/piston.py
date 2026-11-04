"""Piston, rings and wrist pin (reference image refs/1.png).

Origin: the wrist-pin centre. Cylinder axis +Z (glTF +Y), pin axis X. Reference bore 86 mm;
the scene scales the three nodes uniformly by bore / 0.086. Proportions follow engine-sim's
view3d: crown 0.35 bore over the pin (piston 0.65 bore tall), wrist pin radius 0.11 bore.
Run: blender -b -P piston.py [-- --render 1]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

A = args()
B = 0.086
R = 0.4985 * B  # piston radius (a little under the bore)
CROWN = 0.35 * B  # crown edge over the pin
H = 0.65 * B  # a longer skirt than the scene's 0.6 bore primitive, as in refs/1.png
SKIRT = CROWN - H  # skirt bottom
PIN_R = 0.11 * B
mm = MM


def round_skirt_edges(p, xc, top):
    """Round the skirt's cut ends (refs/1.png: the skirt curls into the pin-side panel): a
    4 mm radius on the outer vertical edges, 1 mm on the inner ones."""
    import bmesh as _bm

    bm = _bm.new()
    bm.from_mesh(p.data)
    on = lambda v: abs(abs(v.co.x) - xc) < 1e-6 and v.co.z < top + 1e-6
    rad = lambda v: math.hypot(v.co.x, v.co.y)
    for r_edge, width, segs in ((R, 4.0 * mm, 8), (R - 3.0 * mm, 1.0 * mm, 3)):
        edges = [
            e
            for e in bm.edges
            if all(on(v) and abs(rad(v) - r_edge) < 0.2 * mm for v in e.verts)
        ]
        if edges:
            _bm.ops.bevel(bm, geom=edges, offset=width, segments=segs, profile=0.5, affect="EDGES", clamp_overlap=True)
        print(f"[piston] rounded {len(edges)} skirt edges at r={r_edge*1e3:.1f} mm")
    bm.to_mesh(p.data)
    bm.free()


def body():
    g1, g2, g3 = CROWN - 5 * mm, CROWN - 9.2 * mm, CROWN - 13.4 * mm  # groove tops
    gd = 3.6 * mm
    prof = [
        (0.0, CROWN + 0.9 * mm),
        (R - 7 * mm, CROWN + 0.35 * mm),
        (R - 0.7 * mm, CROWN),
        (R, CROWN - 0.7 * mm),
        (R, g1),
        (R - gd, g1),
        (R - gd, g1 - 1.25 * mm),
        (R, g1 - 1.25 * mm),
        (R, g2),
        (R - gd, g2),
        (R - gd, g2 - 1.25 * mm),
        (R, g2 - 1.25 * mm),
        (R, g3),
        (R - gd, g3),
        (R - gd, g3 - 2.6 * mm),
        (R, g3 - 2.6 * mm),
        (R, SKIRT + 0.8 * mm),
        (R - 0.6 * mm, SKIRT),
        (R - 3.0 * mm, SKIRT),
        (R - 3.0 * mm, g3 - 6 * mm),
        (R - 6.5 * mm, g3 - 2 * mm),
        (R - 6.5 * mm, CROWN - 8 * mm),
        (R - 13 * mm, CROWN - 9.5 * mm),
        (0.0, CROWN - 9.0 * mm),
    ]
    p = lathe("piston", prof, segs=56)
    # slipper skirt: the skirt is cut away on the pin sides below the ring belt
    top = g3 - 2.6 * mm - 2.2 * mm
    xc = 0.62 * R  # the slipper cut plane
    for sgn in (1, -1):
        cut(p, box("c", (min(sgn * xc, sgn * 1.3 * R), -1.3 * R, SKIRT - 0.01), (max(sgn * xc, sgn * 1.3 * R), 1.3 * R, top)))
    round_skirt_edges(p, xc, top)
    # the pin-side panel (refs/1.png): a recessed wall joining the two thrust skirts, set back
    # from the skirt's circle so nothing stands proud of it, its lower edge a concave arc from
    # one skirt tip to the other that rises to just under the pin boss
    boss_r = PIN_R + 6.5 * mm
    xp, t = 0.62 * R - 1.2 * mm, 3.5 * mm  # set back from the cut: the rounded skirt ends stand out
    yw = math.sqrt((R - 0.8 * mm) ** 2 - xp**2)  # inside the skirt wall: merges, never protrudes
    z_top = CROWN - 8.5 * mm  # up into the crown's underside
    z_mid = -boss_r - 2.0 * mm  # arc apex: a little material under the boss
    n = 48
    arc = [(yw * math.cos(math.pi * i / n), SKIRT + (z_mid - SKIRT) * math.sin(math.pi * i / n)) for i in range(n + 1)]
    panel_pts = [(-yw, z_top)] + [(-yw, SKIRT)] + list(reversed(arc))[1:-1] + [(yw, SKIRT), (yw, z_top)]
    for sgn in (1, -1):
        x0, x1 = sorted((sgn * (xp - t), sgn * xp))
        union(p, prism("panel", panel_pts, x0, x1, "X"))
        # two shallow pockets in the panel, one each side of the boss
        for sy in (1, -1):
            y0, y1 = sorted((sy * (boss_r + 2.5 * mm), sy * (yw - 3.5 * mm)))
            px0, px1 = sorted((sgn * (xp - 1.2 * mm), sgn * (xp + 1.0 * mm)))
            cut(p, box("pocket", (px0, y0, z_mid + 4.0 * mm), (px1, y1, top - 3.0 * mm)))
        boss = cylinder("boss", boss_r, sgn * 0.24 * R, sgn * 0.8 * R, "X", segs=48)
        union(p, boss)
    # the pin bore, right through
    cut(p, cylinder("bore", PIN_R + 0.02 * mm, -1.2 * R, 1.2 * R, "X", segs=48))
    # snap-ring grooves near each end of the bore
    for sgn in (1, -1):
        cut(p, cylinder("clip", PIN_R + 1.0 * mm, sgn * 0.755 * R, sgn * 0.775 * R, "X", segs=48))
    # oil drain holes in the oil-ring groove (pin sides)
    for sgn in (1, -1):
        for dy in (-7 * mm, 7 * mm):
            cut(p, cylinder("drain", 1.4 * mm, sgn * (R - gd - 3 * mm), sgn * (R + 1 * mm), "X", segs=16, at=(dy, g3 - 1.3 * mm)))
    # valve reliefs (intake one side, exhaust the other, at ±0.25 bore like the scene's valves)
    for sgn, rr in ((1, 0.185 * B), (-1, 0.165 * B)):
        cut(p, cylinder("relief", rr, CROWN - 0.9 * mm, CROWN + 3 * mm, "Z", segs=64, at=(0.0, sgn * 0.25 * B)))
    return finish(paint(p, ALU), bevel=0.35 * mm, segments=2, angle=30)


def rings():
    g1, g2, g3 = CROWN - 5 * mm, CROWN - 9.2 * mm, CROWN - 13.4 * mm
    out = []

    def ring(name, z_top, h, rin, mat):
        c = 0.03 * mm
        prof = [(rin, z_top - c), (R - 0.02 * mm, z_top - c), (R - 0.02 * mm, z_top - h + c), (rin, z_top - h + c)]
        r = lathe(name, prof, segs=56, closed=True)
        return finish(paint(r, mat), bevel=0.12 * mm, segments=1, angle=60)

    out.append(ring("ring_top", g1, 1.25 * mm, R - 3.3 * mm, DARK))
    out.append(ring("ring_second", g2, 1.25 * mm, R - 3.3 * mm, DARK))
    # oil control ring: two rails over an expander
    out.append(ring("ring_oil_rail_top", g3, 0.5 * mm, R - 3.0 * mm, POLISHED))
    out.append(ring("ring_oil_rail_bottom", g3 - 2.1 * mm, 0.5 * mm, R - 3.0 * mm, POLISHED))
    exp = lathe("ring_oil_expander", [(R - 3.4 * mm, g3 - 0.55 * mm), (R - 1.2 * mm, g3 - 0.55 * mm), (R - 1.2 * mm, g3 - 2.05 * mm), (R - 3.4 * mm, g3 - 2.05 * mm)], segs=56, closed=True)
    out.append(finish(paint(exp, DARK), 0, smooth=60))
    return out


def wrist_pin():
    L = 0.79 * B
    prof = [(PIN_R - 0.5 * mm, -L / 2), (PIN_R, -L / 2 + 0.5 * mm), (PIN_R, L / 2 - 0.5 * mm), (PIN_R - 0.5 * mm, L / 2), (0.55 * PIN_R, L / 2), (0.55 * PIN_R, -L / 2)]
    pin = lathe("wrist_pin", prof, segs=48, closed=True)
    pin.data.transform(Matrix.Rotation(math.pi / 2, 4, "Y"))
    return finish(paint(pin, POLISHED), 0.15 * mm, 1, angle=50)


reset()
p = body()
rs = rings()
pin = wrist_pin()
common = dict(
    ref={"bore_m": B},
    origin="wrist-pin centre; cylinder axis +Y, pin axis +X",
    rule="uniform scale s = bore_m / 0.086 about the origin",
)
for o in [p, *rs, pin]:
    tag(o, part=o.name, **common)
export("piston.glb", [p, *rs, pin])
if A.get("render"):
    render("piston.png", [p, *rs, pin], direction=(1.0, -1.6, 0.55))
    render("piston_pinside.png", [p, *rs, pin], direction=(1.0, -0.12, 0.05), lens=85)
    render("piston_under.png", [p, *rs, pin], direction=(0.9, -1.1, -0.8), floor=False)
