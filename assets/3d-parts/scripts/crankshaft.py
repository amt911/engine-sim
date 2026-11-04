"""Crankshaft (reference image refs/3.png): one throw module repeated per cylinder, plus a nose
(front: timing sprocket, snout, keyway) and a flange (rear: oil-seal land, flywheel flange).

Frame: crank axis +X, the throw's pin at +Z at crank angle 0 (glTF: +X axis, pin at +Y, as the
scene's `pin = (x, r cos th, r sin th)`). Origin on the axis at the cylinder's centre plane.
Reference: bore 86 mm, stroke 86 mm (throw r 43 mm), cylinder pitch 1.2 bore (scene).

crank_throw.glb: x in [-p/2, p/2]: half main journal, web, crankpin, web, half main journal.
  Rule: p' = s*p + w*(r - s*0.043)*Y, s = bore_m/0.086, r = stroke_m/2, w = WEIGHTS_0 of joint
  `stretch` (pin 1, main journals 0, webs w = clamp(z / r_ref): the arm stretches, the
  counterweight stays). Place throw i at x = i*pitch, turned about X by its crank angle.
crank_nose.glb / crank_flange.glb: rigid, uniform scale s; nose spans x <= 0 (put its x = 0 on
  the first throw's -p/2), flange spans x >= 0 (on the last throw's +p/2).
Run: blender -b -P crankshaft.py [-- --render 1]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

A = args()
mm = MM
B = 0.086
R_T = 0.043  # throw radius (stroke / 2)
P = 1.2 * B  # cylinder pitch
PIN_R = 0.25 * B  # = the conrod's big-end bore
MAIN_R = 0.30 * B
TW = 0.21 * B  # web thickness
LP = 0.30 * B  # crankpin length
LM = P - 2 * TW - LP  # main journal length (half in each neighbouring module)
RB = PIN_R + 5.5 * mm  # pin boss radius
RH = MAIN_R + 6.5 * mm  # hub radius
RC = 0.74 * B  # counterweight radius (clears the piston skirt at BDC with the ref rod)
CW_HALF = math.radians(62)  # counterweight half angle


def journal_x(name, r, x0, x1, at=(0.0, 0.0), segs=56, ch=0.8 * mm):
    """A journal along X from x0 to x1, chamfered ends, axis through (y, z) = at."""
    prof = [(0.0, x0), (r - ch, x0), (r, x0 + ch), (r, x1 - ch), (r - ch, x1), (0.0, x1)]
    o = lathe(name, prof, segs=segs)
    o.data.transform(Matrix.Translation((0, at[0], at[1])) @ Matrix.Rotation(math.pi / 2, 4, "Y"))  # lathe Z -> +X
    return o


def web_outline(n=360):
    """Polar outline about the main axis (phi from +Z toward +Y): a fan, the convex hull of the
    pin boss, the hub and the counterweight sector (its two tips rounded), as the forged webs
    in refs/3.png: wide over the weight, straight flanks tapering to the pin."""
    pts = []

    def circle(cy, cz, rr, k=160):
        for i in range(k):
            a = 2 * math.pi * i / k
            pts.append((cy + rr * math.sin(a), cz + rr * math.cos(a)))

    circle(0.0, R_T, RB)
    circle(0.0, 0.0, RH)
    tip = 10 * mm
    for sgn in (1, -1):
        a = math.pi + sgn * CW_HALF
        circle((RC - tip) * math.sin(a), (RC - tip) * math.cos(a), tip, 48)
    for i in range(121):
        a = math.pi - CW_HALF + 2 * CW_HALF * i / 120
        pts.append((RC * math.sin(a), RC * math.cos(a)))
    pts = sorted(set(pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p_ in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p_) <= 0:
            lower.pop()
        lower.append(p_)
    for p_ in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p_) <= 0:
            upper.pop()
        upper.append(p_)
    hull = np.array(lower[:-1] + upper[:-1])
    a, b = hull, np.roll(hull, -1, axis=0)
    e = b - a
    nrm = np.stack([e[:, 1], -e[:, 0]], axis=1)
    c = (nrm * a).sum(axis=1)
    nrm[c < 0] *= -1
    c = np.abs(c)
    phi = np.linspace(0, 2 * math.pi, n, endpoint=False)
    d = np.stack([np.sin(phi), np.cos(phi)], axis=1)  # (y, z)
    nd = d @ nrm.T
    t = np.where(nd > 1e-12, c[None, :] / np.where(nd > 1e-12, nd, 1.0), np.inf)
    rho = t.min(axis=1)
    return [(r * math.sin(f), r * math.cos(f)) for r, f in zip(rho, phi)]


def throw():
    outline = web_outline()
    webs = []
    for x0, x1 in ((-LP / 2 - TW, -LP / 2), (LP / 2, LP / 2 + TW)):
        w = prism("web", outline, x0, x1, "X")
        webs.append(finish(paint(w, FORGED), bevel=3.2 * mm, segments=4, angle=30))
    web_o = webs[0]
    web_o.name = "crank_webs"
    union(web_o, webs[1])
    web_o.data.name = "crank_webs"
    finish(web_o)
    pin = journal_x("crank_pin", PIN_R, -LP / 2 - 0.5 * mm, LP / 2 + 0.5 * mm, at=(0.0, R_T))
    # oil feed hole on the pin
    cut(pin, cylinder("oh", 2.6 * mm, R_T + PIN_R - 9 * mm, R_T + PIN_R + 1 * mm, "Z", segs=16, at=(0.0, 0.0)))
    finish(paint(pin, POLISHED), 0, smooth=40)
    mains = []
    for x0, x1 in ((-P / 2, -LP / 2 - TW + 0.5 * mm), (LP / 2 + TW - 0.5 * mm, P / 2)):
        mains.append(journal_x("m", MAIN_R, x0, x1))
    main_o = mains[0]
    union(main_o, mains[1])
    main_o.name = main_o.data.name = "crank_main_journals"
    for sgn in (1, -1):
        xc = sgn * (P / 2 - LM / 4)
        cut(main_o, cylinder("oh", 2.6 * mm, MAIN_R - 9 * mm, MAIN_R + 1 * mm, "Y", segs=16, at=(xc, 0.0)))
    finish(paint(main_o, POLISHED), 0, smooth=40)
    return web_o, pin, main_o


def gear_outline(teeth, r_root, r_tip):
    pts = []
    for i in range(teeth):
        a0 = 2 * math.pi * i / teeth
        da = 2 * math.pi / teeth
        for frac, rr in ((0.0, r_root), (0.18, r_root), (0.32, r_tip), (0.58, r_tip), (0.72, r_root)):
            a = a0 + frac * da
            pts.append((rr * math.sin(a), rr * math.cos(a)))
    return pts


def nose():
    x = 0.0
    parts = [journal_x("n_main", MAIN_R, -LM / 2, x)]
    x = -LM / 2
    collar = journal_x("n_collar", 0.25 * B, x - 3 * mm, x + 0.5 * mm)
    x -= 3 * mm
    gw = 0.13 * B
    gear = prism("n_gear", gear_outline(26, 0.30 * B, 0.335 * B), x - gw, x, "X")
    cut(gear, cylinder("gb", 0.18 * B, x - gw - 1e-3, x + 1e-3, "X"))
    finish(paint(gear, DARK), 0.4 * mm, 1, angle=40)
    x -= gw
    snout = journal_x("n_snout", 0.19 * B, x - 0.32 * B, x + gw + 0.5 * mm)
    # keyway along the snout, and the pulley bolt's thread hole
    cut(snout, box("key", (x - 0.32 * B - 1e-3, -2.5 * mm, 0.19 * B - 2.5 * mm), (x + 1e-3, 2.5 * mm, 0.19 * B + 1 * mm)))
    cut(snout, cylinder("th", 0.06 * B, x - 0.32 * B - 1e-3, x - 0.32 * B + 18 * mm, "X"))
    body = parts[0]
    union(body, collar, snout)
    body.name = body.data.name = "crank_nose"
    finish(paint(body, POLISHED), 0, smooth=40)
    return [body, gear]


def flange():
    m = journal_x("f_main", MAIN_R, 0.0, LM / 2)
    seal = journal_x("f_seal", 0.36 * B, LM / 2 - 0.5 * mm, LM / 2 + 9 * mm)
    union(m, seal)
    m.name = m.data.name = "crank_flange_journal"
    finish(paint(m, POLISHED), 0, smooth=40)
    x0 = LM / 2 + 9 * mm
    fl = journal_x("crank_flange", 0.5 * B, x0, x0 + 12 * mm, segs=72, ch=1.2 * mm)
    for i in range(6):
        a = 2 * math.pi * i / 6 + math.pi / 6
        cut(fl, cylinder("bh", 4.5 * mm, x0 - 1e-3, x0 + 12 * mm + 1e-3, "X", segs=20, at=(0.33 * B * math.sin(a), 0.33 * B * math.cos(a))))
    cut(fl, cylinder("pilot", 0.12 * B, x0 + 6 * mm, x0 + 12 * mm + 1e-3, "X", segs=40))
    finish(paint(fl, FORGED), 0.5 * mm, 2, angle=40)
    return [m, fl]


def duplicate(ob, name, mat):
    c = ob.copy()
    c.data = ob.data.copy()
    c.name = name
    c.data.transform(mat)
    link(c)
    return c


reset()
web_o, pin_o, main_o = throw()
nose_o = nose()
flange_o = flange()

# assembled inline-4 (crank angles 0, 180, 180, 0) for the preview only, made before the skin
assembly = []
if A.get("render"):
    n_cyl = 4
    angles = [0, 180, 180, 0]
    for i, a in enumerate(angles):
        mat = Matrix.Translation((i * P, 0, 0)) @ Matrix.Rotation(math.radians(a), 4, "X")
        assembly += [duplicate(o, f"{o.name}_{i}", mat) for o in (web_o, pin_o, main_o)]
    assembly += [duplicate(o, f"{o.name}_a", Matrix.Translation((-P / 2, 0, 0))) for o in nose_o]
    assembly += [duplicate(o, f"{o.name}_a", Matrix.Translation(((n_cyl - 1) * P + P / 2, 0, 0))) for o in flange_o]

ref = {"bore_m": B, "stroke_m": 2 * R_T, "pitch_m": P, "crankpin_r_m": PIN_R, "main_r_m": MAIN_R, "counterweight_r_m": RC}
for o in (web_o, pin_o, main_o):
    tag(o, part=o.name, ref=ref, origin="axis at the cylinder plane; crank +X, pin +Y at angle 0",
        rule="p' = s*p + w*(stroke_m/2 - s*0.043)*Y, s = bore_m/0.086")
for o in nose_o + flange_o:
    tag(o, part=o.name, ref=ref, origin="x = 0 on the end throw's module face", rule="uniform scale s = bore_m/0.086")


def w_throw(co):
    return co.z / R_T


arm = skin("crank_rig", [web_o], (0, 0, R_T), w_throw)
for o, w in ((pin_o, 1.0), (main_o, 0.0)):
    g0, g1 = o.vertex_groups.new(name="root"), o.vertex_groups.new(name="stretch")
    idx = [v.index for v in o.data.vertices]
    g0.add(idx, 1.0 - w, "REPLACE")
    g1.add(idx, w, "REPLACE")
    mod = o.modifiers.new("armature", "ARMATURE")
    mod.object = arm
    o.parent = arm
export("crank_throw.glb", [web_o, pin_o, main_o, arm])
export("crank_nose.glb", nose_o)
export("crank_flange.glb", flange_o)
if A.get("render"):
    render("crank_i4.png", assembly, direction=(0.75, -1.0, 0.35), lens=60)
    render("crank_throw.png", [web_o, pin_o, main_o], direction=(1.0, -0.9, 0.5), lens=70)
    arm.pose.bones["stretch"].location = (0.0, 0.015, 0.0)  # stroke 86 -> 116 mm
    bpy.context.view_layer.update()
    render("crank_throw_long_stroke.png", [web_o, pin_o, main_o], direction=(1.0, -0.9, 0.5), lens=70)
