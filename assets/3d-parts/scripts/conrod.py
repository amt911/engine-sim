"""Connecting rod: forged I-beam body, split cap, bolts, big-end shells and small-end bush
(reference image refs/1.png).

Origin: the big-end (crankpin) centre. Rod axis +Z (glTF +Y) to the small end at L = 143 mm,
pin axes along X, the rod swings in the Y-Z plane (glTF: crank along +X).
Reference: bore 86 mm, rod 143 mm, crankpin radius 0.25 bore, wrist-pin radius 0.11 bore.
Stretch: a two-joint skin, joint `stretch` at the small-end centre. Every vertex has weight w
(0 on the big end, 1 on the small end, linear along the beam). Scene rule:
    p' = s * p + w * (rod_m - s * 0.143) * Y,   s = bore_m / 0.086
Run: blender -b -P conrod.py [-- --render 1]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

A = args()
B = 0.086
L = 0.143
mm = MM
PIN_R = 0.25 * B  # crankpin
WP_R = 0.11 * B  # wrist pin
SHELL = 1.6 * mm
BUSH = 1.4 * mm
BIG_R = PIN_R + SHELL + 7.0 * mm  # big-end eye outer radius
SMALL_R = WP_R + BUSH + 5.0 * mm
BOLT_Y = PIN_R + SHELL + 5.6 * mm
BOSS_W = BOLT_Y + 6.0 * mm  # half-width over the bolt bosses
BOSS_TOP = 15 * mm
CAP_BOSS = -17 * mm
HALF_T = 9.5 * mm  # half the rod's width along X
FLANGE = 3.2 * mm
WEB = 3.2 * mm  # half the web thickness


def softmax(vals, k=1.6 * mm):
    vals = np.array(vals)
    m = vals.max(axis=0)
    return m + k * np.log(np.exp((vals - m) / k).sum(axis=0))


def circle_hw(z, cz, r):
    d = r * r - (z - cz) ** 2
    return np.where(d > 0, np.sqrt(np.clip(d, 0, None)), -1.0)


def beam_hw(z):
    z0, z1 = BIG_R, L - SMALL_R
    t = np.clip((z - z0) / (z1 - z0), 0, 1)
    v = (10.5 * mm) * (1 - t) + (7.0 * mm) * t
    return np.where((z > 0) & (z < L), v, -1.0)


def boss_hw(z, z_lo, z_hi, rr=4 * mm):
    """Half-width BOSS_W over [z_lo, z_hi], corners rounded with radius rr."""
    out = np.full_like(z, -1.0)
    inside = (z >= z_lo) & (z <= z_hi)
    out[inside] = BOSS_W
    for zc, sgn in ((z_hi - rr, 1), (z_lo + rr, -1)):
        band = inside & ((z - zc) * sgn > 0)
        dz = np.abs(z[band] - zc)
        out[band] = BOSS_W - rr + np.sqrt(np.clip(rr * rr - dz * dz, 0, None))
    return out


def outline(z, terms):
    hw = softmax([t(z) for t in terms])
    right = list(zip(hw, z))
    left = [(-y, zz) for y, zz in reversed(right)]
    return right + left


def body():
    z = np.concatenate([np.linspace(0.0, L + SMALL_R, 420)])
    z[-1] = L + SMALL_R - 1e-6
    terms = [
        beam_hw,
        lambda z: circle_hw(z, L, SMALL_R),
        lambda z: circle_hw(z, 0.0, BIG_R),
        lambda z: boss_hw(z, -1.0, BOSS_TOP),
    ]
    pts = outline(z, terms)
    # the outline is (half-width, z): drop near-zero widths at the very top (closes on itself)
    pts = [(y, zz) for y, zz in pts if abs(y) > 0.05 * mm or zz < L]
    rod = prism("rod_body", pts, -HALF_T, HALF_T, "X")
    # I-beam: a pocket each side of the web
    zp = np.linspace(BIG_R + 4 * mm, L - SMALL_R - 3 * mm, 160)
    hw = beam_hw(zp) - FLANGE
    # round the pocket's ends
    span = zp[-1] - zp[0]
    for i, zz in enumerate(zp):
        for end in (zp[0], zp[-1]):
            d = abs(zz - end)
            if d < hw[i]:
                hw[i] = min(hw[i], math.sqrt(max(hw[i] ** 2 - (hw[i] - d) ** 2, 0)))
    pk = [(y, zz) for y, zz in zip(hw, zp)] + [(-y, zz) for y, zz in reversed(list(zip(hw, zp)))]
    pk = [p for p in pk if abs(p[0]) > 1e-5] or pk
    for sgn in (1, -1):
        lo, hi = sorted((sgn * WEB, sgn * (HALF_T + 1 * mm)))
        cut(rod, prism("pocket", pk, lo, hi, "X"))
    # big end and small end bores (shells and bush are separate parts)
    cut(rod, cylinder("bore", PIN_R + SHELL, -HALF_T - 1e-3, HALF_T + 1e-3, "X", segs=64))
    cut(rod, cylinder("sbore", WP_R + BUSH, -HALF_T - 1e-3, HALF_T + 1e-3, "X", segs=48, at=(0.0, L)))
    # oil hole on top of the small end
    cut(rod, cylinder("oil", 1.6 * mm, L, L + SMALL_R + 1 * mm, "Z", segs=16))
    # bolt holes (the bolts are their own part)
    for sgn in (1, -1):
        cut(rod, cylinder("bh", 4.2 * mm, 0.0 - 1e-4, BOSS_TOP + 1e-3, "Z", segs=24, at=(0.0, sgn * BOLT_Y)))
    return finish(paint(rod, FORGED), bevel=0.8 * mm, segments=2, angle=35)


def cap():
    z = np.linspace(-0.25 * mm, -BIG_R + 1e-6, 260)
    terms = [lambda z: circle_hw(z, 0.0, BIG_R), lambda z: boss_hw(z, CAP_BOSS, 1.0)]
    hw = softmax([t(z) for t in terms])
    right = list(zip(hw, z))
    pts = right + [(-y, zz) for y, zz in reversed(right)]
    pts = [(y, zz) for y, zz in pts if abs(y) > 0.05 * mm]
    c = prism("rod_cap", pts, -HALF_T, HALF_T, "X")
    cut(c, cylinder("bore", PIN_R + SHELL, -HALF_T - 1e-3, HALF_T + 1e-3, "X", segs=64))
    for sgn in (1, -1):
        cut(c, cylinder("bh", 4.2 * mm, CAP_BOSS - 1e-3, 0.0, "Z", segs=24, at=(0.0, sgn * BOLT_Y)))
    return finish(paint(c, FORGED), bevel=0.8 * mm, segments=2, angle=35)


def bolts():
    parts = []
    for sgn in (1, -1):
        y = sgn * BOLT_Y
        head = cylinder("h", 6.6 * mm, CAP_BOSS, CAP_BOSS - 5.5 * mm, "Z", segs=6, at=(0.0, y))
        washer = cylinder("w", 6.2 * mm, CAP_BOSS + 0.2 * mm, CAP_BOSS - 0.8 * mm, "Z", segs=32, at=(0.0, y))
        shank = cylinder("s", 4.0 * mm, CAP_BOSS, BOSS_TOP + 3.2 * mm, "Z", segs=24, at=(0.0, y))
        union(head, washer, shank)
        parts.append(head)
    b = parts[0]
    union(b, parts[1])
    b.name = b.data.name = "rod_bolts"
    return finish(paint(b, DARK), bevel=0.3 * mm, segments=1, angle=40)


def ring_x(name, r_in, r_out, half, at_z, mat, segs=64):
    prof = [(r_in, -half), (r_out, -half), (r_out, half), (r_in, half)]
    o = lathe(name, prof, segs=segs, closed=True)
    o.data.transform(Matrix.Translation((0, 0, at_z)) @ Matrix.Rotation(math.pi / 2, 4, "Y"))
    return finish(paint(o, mat), 0.15 * mm, 1, angle=50)


reset()
body_o = body()
cap_o = cap()
bolt_o = bolts()
shells = ring_x("rod_bearing_shells", PIN_R, PIN_R + SHELL, HALF_T - 0.6 * mm, 0.0, BRONZE)
bush = ring_x("rod_small_end_bush", WP_R, WP_R + BUSH, HALF_T - 0.3 * mm, L, BRONZE, segs=48)
obs = [body_o, cap_o, bolt_o, shells, bush]

Z0, Z1 = BIG_R + 2 * mm, L - SMALL_R - 1 * mm


def w_of(co):
    return (co.z - Z0) / (Z1 - Z0)


common = dict(
    ref={"bore_m": B, "rod_m": L, "crankpin_r_m": PIN_R, "wrist_pin_r_m": WP_R},
    origin="big-end centre; rod axis +Y to the small end; pin axes +X",
    rule="p' = s*p + w*(rod_m - s*0.143)*Y, s = bore_m/0.086, w = WEIGHTS_0 of joint 'stretch'",
    stretch_joint_ref_m=[0.0, L, 0.0],
)
for o in obs:
    tag(o, part=o.name, **common)
arm = skin("conrod_rig", obs, (0, 0, L), w_of)
export("conrod.glb", obs + [arm])
if A.get("render"):
    render("conrod.png", obs, direction=(1.0, -0.35, 0.25), lens=85)
    # stretch check: the same rod 30 mm longer, posed through its own skin
    pb = arm.pose.bones["stretch"]
    pb.location = (0.0, 0.03, 0.0)  # bone-local Y is the bone's axis (+Z in Blender here)
    bpy.context.view_layer.update()
    render("conrod_stretched.png", obs, direction=(1.0, -0.35, 0.25), lens=85)
