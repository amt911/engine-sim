"""Wankel rotor, rotor housing ("caja"), side plate with the stationary gear, eccentric shaft and
spark plugs. Defaults: Mazda 13B / RENESIS presets (e 15 mm, R 105 mm, width 80 mm).

Frame = engine-sim's `view3d::mesh::rotor` / `housing`: the rotor's plane is glTF x-z, with the
2D view's (x, y) = glTF (x, z); the shaft is glTF +Y; rotor apex 0 on +x. Unlike the unit meshes
these are real size and centred in y (y in [-w/2, w/2]), so the scene's frame drops its
`- 0.45 w` offset and scales by (R/R_ref, w/w_ref, R/R_ref). The housing's waist (the minor axis,
2D +y) carries the two plugs at x = ±0.18 R, as the scene does.

The rotor flank is the true inner envelope of the epitrochoid x = e cos 3a + R cos a,
y = e sin 3a + R sin a (rotor turning theta about its centre e (cos 3theta, sin 3theta), apexes
on the housing: Yamamoto, *Rotary Engine*, 1981, ch. 2), computed numerically, so the shape
depends on e/R: regenerate for another e/R (`-- --e 0.015 --R 0.105 --w 0.08`); for the same
e/R a uniform in-plane scale is exact.
Run: blender -b -P wankel.py [-- --render 1]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

A = args()
mm = MM
E = float(A.get("e", 0.015))
R = float(A.get("R", 0.105))
W = float(A.get("w", 0.08))
CLEAR = 0.25 * mm  # flank to housing
BEAR_R = 0.35 * R  # eccentric lobe radius (the scene's 0.35 r)
SHAFT_R = 0.205 * R
IRON = ("rotor_iron", (0.2, 0.2, 0.21), 1.0, 0.5)
CHROME = ("trochoid_chrome", (0.85, 0.86, 0.88), 1.0, 0.08)
CERAMIC = ("plug_ceramic", (0.9, 0.9, 0.87), 0.0, 0.3)


def b(pts):
    """2D view (x, y) -> Blender (x, -y): the outline in Blender's XY plane, extruded along Z
    (glTF +Y)."""
    return [(u, -v) for u, v in pts]


def epitrochoid(n=720, off=0.0):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    x = E * np.cos(3 * a) + R * np.cos(a)
    y = E * np.sin(3 * a) + R * np.sin(a)
    if off:
        r = np.hypot(x, y)
        x, y = x * (1 + off / r), y * (1 + off / r)
    return list(zip(x, y))


def rotor_rho(nphi=1440):
    """Polar radius of the rotor flank (rotor frame, apex 0 at phi = 0)."""
    al = np.linspace(0, 2 * math.pi, 4096, endpoint=False)
    px = E * np.cos(3 * al) + R * np.cos(al)
    py = E * np.sin(3 * al) + R * np.sin(al)
    best = np.full(nphi, np.inf)
    for t in np.linspace(0, 2 * math.pi, 1080, endpoint=False):
        dx, dy = px - E * math.cos(3 * t), py - E * math.sin(3 * t)
        c, s = math.cos(-t), math.sin(-t)
        qx, qy = c * dx - s * dy, s * dx + c * dy
        phi = np.arctan2(qy, qx) % (2 * math.pi)
        idx = (phi / (2 * math.pi) * nphi).astype(int) % nphi
        np.minimum.at(best, idx, np.hypot(qx, qy))
    third = nphi // 3
    best = np.minimum(best, np.minimum(np.roll(best, third), np.roll(best, 2 * third)))
    return best


RHO = rotor_rho()
NPHI = len(RHO)
PHI = np.linspace(0, 2 * math.pi, NPHI, endpoint=False)
print(f"[wankel] rotor flank: apex {RHO[0]*1e3:.2f} mm (R {R*1e3:.1f}), mid-flank {RHO[NPHI // 6]*1e3:.2f} mm")


def rho_at(phi):
    return float(np.interp(phi % (2 * math.pi), np.append(PHI, 2 * math.pi), np.append(RHO, RHO[0])))


def polar(phi, r):
    return (r * math.cos(phi), r * math.sin(phi))


def rect(phi, r0, r1, t):
    """A radial rectangle at angle phi, radius r0..r1, half-width t (tangential)."""
    c, s = math.cos(phi), math.sin(phi)
    return [(r * c - tt * s, r * s + tt * c) for r, tt in ((r0, -t), (r1, -t), (r1, t), (r0, t))]


def gear2d(teeth, r_a, r_b, frac=(0.0, 0.2, 0.33, 0.62, 0.75)):
    """Teeth alternating between radius r_a (tooth) and r_b (gap)."""
    pts = []
    for i in range(teeth):
        a0, da = 2 * math.pi * i / teeth, 2 * math.pi / teeth
        for f, rr in zip(frac, (r_b, r_b, r_a, r_a, r_b)):
            pts.append(polar(a0 + f * da, rr))
    return pts


def rotor():
    hw = W / 2 - 0.15 * mm
    outline = [polar(f, r - CLEAR) for f, r in zip(PHI[::4], RHO[::4])]
    ro = prism("rotor", b(outline), -hw, hw, "Z")
    # combustion recess on each flank, biased to the leading side (the rotor turns +phi)
    depth, half, off = 3.6 * mm, math.radians(22), math.radians(6)
    for k in range(3):
        fc = math.radians(60 + 120 * k) + off
        fs = np.linspace(fc - half, fc + half, 60)
        ramp = math.radians(7)
        inner = []
        for f in fs:
            d = depth * min(1.0, (f - fs[0]) / ramp, (fs[-1] - f) / ramp)
            inner.append(polar(f, rho_at(f) - CLEAR - max(d, 0.4 * mm)))
        outer = [polar(f, rho_at(f) + 4 * mm) for f in fs[::-1]]
        cut(ro, prism("recess", b(inner + outer), -0.29 * W, 0.29 * W, "Z"))
    # apex seal slots
    for k in range(3):
        f = math.radians(120 * k)
        cut(ro, prism("slot", b(rect(f, R - 9.5 * mm, R + 2 * mm, 1.3 * mm)), -hw - 1e-3, hw + 1e-3, "Z"))
    # side faces: side-seal grooves along each flank between the corner seals, oil-seal grooves
    for z0, z1 in ((hw - 1.0 * mm, hw + 1e-3), (-hw - 1e-3, -hw + 1.0 * mm)):
        for k in range(3):
            fs = np.linspace(math.radians(120 * k + 6.5), math.radians(120 * k + 113.5), 40)
            band = [polar(f, rho_at(f) - 3.0 * mm) for f in fs] + [polar(f, rho_at(f) - 3.8 * mm) for f in fs[::-1]]
            cut(ro, prism("sseal", b(band), z0, z1, "Z"))
        for rr in (0.56 * R, 0.59 * R):
            g = cylinder("og", rr + 0.4 * mm, z0, z1, "Z", segs=96)
            cut(g, cylinder("ogi", rr - 0.4 * mm, z0 - 1e-3, z1 + 1e-3, "Z", segs=96))
            cut(ro, g)
        # corner seal pockets, 2 mm deep
        p0, p1 = (hw - 2.0 * mm, hw + 1e-3) if z0 > 0 else (-hw - 1e-3, -hw + 2.0 * mm)
        for k in range(3):
            u, v = polar(math.radians(120 * k), R - 7.5 * mm)
            cut(ro, cylinder("cs", 5.5 * mm, p0, p1, "Z", segs=24, at=(u, -v)))
    # internal gear on the -y face (the scene's side plate side), the bearing bore through
    pitch = 3 * E
    gear_cut = prism("gear", b(gear2d(51, pitch - 1.4 * mm, pitch + 2.2 * mm)), -hw - 1e-3, -hw + 14 * mm, "Z")
    cut(ro, gear_cut)
    cut(ro, cylinder("bore", BEAR_R + 2.2 * mm, -hw - 1e-3, hw + 1e-3, "Z", segs=96))
    finish(paint(ro, IRON), bevel=0.5 * mm, segments=2, angle=35)

    seals = []
    for k in range(3):
        f = math.radians(120 * k)
        s = prism("apex", b(rect(f, R - 9.0 * mm, R - 0.02 * mm, 1.15 * mm)), -hw + 0.3 * mm, hw - 0.3 * mm, "Z")
        seals.append(s)
        for z0, z1 in ((hw - 2.0 * mm, hw - 0.05 * mm), (-hw + 0.05 * mm, -hw + 2.0 * mm)):
            u, v = polar(f, R - 7.5 * mm)
            seals.append(cylinder("corner", 5.3 * mm, z0, z1, "Z", segs=24, at=(u, -v)))
    so = seals[0]
    union(so, *seals[1:])
    so.name = so.data.name = "rotor_seals"
    finish(paint(so, DARK), 0.15 * mm, 1, angle=40)

    bear = cylinder("rotor_bearing", BEAR_R + 2.2 * mm, -hw + 14.5 * mm, hw - 0.5 * mm, "Z", segs=96)
    cut(bear, cylinder("bi", BEAR_R + 0.05 * mm, -hw, hw, "Z", segs=96))
    bear.name = bear.data.name = "rotor_bearing"
    finish(paint(bear, BRONZE), 0.2 * mm, 1, angle=40)
    return [ro, so, bear]


A_OUT, B_OUT, N_SE = R + E + 0.30 * R, R - E + 0.36 * R, 3.5  # housing outline (superellipse)


def outer_rho(phi):
    c, s = abs(math.cos(phi)), abs(math.sin(phi))
    return 1.0 / ((c / A_OUT) ** N_SE + (s / B_OUT) ** N_SE) ** (1 / N_SE)


def housing():
    hw = W / 2
    fs = np.linspace(0, 2 * math.pi, 240, endpoint=False)
    outer = [polar(f, outer_rho(f)) for f in fs]
    h = prism("rotor_housing", b(outer), -hw, hw, "Z")
    cut(h, prism("bore", b(epitrochoid(360, 0.8 * mm)), -hw - 1e-3, hw + 1e-3, "Z"))
    # coolant passages and tension bolts through the housing, between bore and outside
    for i in range(24):
        f = 2 * math.pi * (i + 0.5) / 24
        if abs(math.degrees(f) - 90) < 22:  # leave the plug bosses solid
            continue
        x_in, y_in = E * math.cos(3 * f) + R * math.cos(f), E * math.sin(3 * f) + R * math.sin(f)
        r_in, a_in = math.hypot(x_in, y_in), math.atan2(y_in, x_in)
        gap = outer_rho(a_in) - r_in
        bolt = i % 3 == 1
        rr = r_in + (0.68 if bolt else 0.32) * gap
        u, v = polar(a_in, rr)
        cut(h, cylinder("hole", (4.6 if bolt else 3.6) * mm, -hw - 1e-3, hw + 1e-3, "Z", segs=20, at=(u, -v)))
    # plug bores at the waist (2D +y = Blender -Y), leading and trailing
    for u in (-0.18 * R, 0.18 * R):
        cut(h, cylinder("plug", 7.0 * mm, -(R - E - 2 * mm), -(B_OUT + 5 * mm), "Y", segs=24, at=(u, 0.0)))
    finish(paint(h, ("housing_aluminium", (0.42, 0.43, 0.44), 1.0, 0.5)), bevel=1.2 * mm, segments=2, angle=35)

    liner = prism("trochoid_liner", b(epitrochoid(360, 0.8 * mm)), -hw + 0.01 * mm, hw - 0.01 * mm, "Z")
    cut(liner, prism("in", b(epitrochoid(360)), -hw - 1e-3, hw + 1e-3, "Z"))
    for u in (-0.18 * R, 0.18 * R):
        cut(liner, cylinder("plug", 7.0 * mm, -(R - E - 2 * mm), -(B_OUT + 5 * mm), "Y", segs=24, at=(u, 0.0)))
    finish(paint(liner, CHROME), 0, smooth=50)
    return [h, liner]


def plugs():
    out = []
    for u in (-0.18 * R, 0.18 * R):
        top = B_OUT * (1 - abs(u / A_OUT) ** N_SE) ** (1 / N_SE)
        # the waist is 2D +y = Blender -Y: build the plugs pointing to -Y
        parts = [
            cylinder("thread", 6.8 * mm, -(R - E + 1.5 * mm), -top, "Y", segs=24, at=(u, 0.0)),
            cylinder("hex", 9.0 * mm, -top, -(top + 9 * mm), "Y", segs=6, at=(u, 0.0)),
        ]
        ins = cylinder("ins", 5.6 * mm, -(top + 9 * mm), -(top + 30 * mm), "Y", segs=24, at=(u, 0.0), r2=4.6 * mm)
        term = cylinder("term", 2.2 * mm, -(top + 30 * mm), -(top + 37 * mm), "Y", segs=12, at=(u, 0.0))
        m = parts[0]
        union(m, parts[1], term)
        out.append(finish(paint(m, POLISHED), 0.3 * mm, 1, angle=40))
        out.append(finish(paint(ins, CERAMIC), 0.4 * mm, 2, angle=40))
    metal = out[0]
    union(metal, out[2])
    metal.name = metal.data.name = "spark_plugs_metal"
    cer = out[1]
    union(cer, out[3])
    cer.name = cer.data.name = "spark_plugs_insulator"
    return [finish(metal), finish(cer)]


def side_plate():
    t = 0.15 * W
    z0 = -W / 2 - t
    fs = np.linspace(0, 2 * math.pi, 240, endpoint=False)
    p = prism("side_plate", b([polar(f, outer_rho(f)) for f in fs]), z0, -W / 2, "Z")
    cut(p, cylinder("shaft", SHAFT_R + 1.0 * mm, z0 - 1e-3, -W / 2 + 1e-3, "Z", segs=72))
    finish(paint(p, ("side_plate_cast", (0.5, 0.5, 0.52), 1.0, 0.5)), bevel=1.2 * mm, segments=2, angle=35)
    # the stationary gear (34 teeth on 2e), standing out of the plate into the rotor's gear
    g = prism("stationary_gear", b(gear2d(34, 2 * E + 1.3 * mm, 2 * E - 2.0 * mm, (0.0, 0.22, 0.36, 0.64, 0.78))), -W / 2 - 1e-4, -W / 2 + 13.5 * mm, "Z")
    cut(g, cylinder("gb", SHAFT_R + 1.2 * mm, -W, W, "Z", segs=72))
    finish(paint(g, DARK), 0.3 * mm, 1, angle=40)
    return [p, g]


def eccentric():
    sh = cylinder("eccentric_shaft", SHAFT_R, -W / 2 - 0.25 * W, W / 2 + 0.1 * W, "Z", segs=64)
    finish(paint(sh, POLISHED), 0.5 * mm, 2, angle=40)
    lobe = cylinder("eccentric_lobe", BEAR_R, -W / 2 + 15 * mm, W / 2 - 0.6 * mm, "Z", segs=96, at=(E, 0.0))
    cut(lobe, cylinder("oil", 3 * mm, -W, W, "Z", segs=16, at=(E + 0.6 * BEAR_R, 0.0)))
    finish(paint(lobe, POLISHED), 0.8 * mm, 2, angle=40)
    return [sh, lobe]


reset()
rotor_o = rotor()
housing_o = housing()
plug_o = plugs()
plate_o = side_plate()
ecc_o = eccentric()

ref = {"e_m": E, "generating_radius_m": R, "width_m": W}
for o in rotor_o:
    tag(o, part=o.name, ref=ref, origin="rotor centre; apex 0 on +x; shaft +y",
        rule="pose like view3d (centre e(cos3t, sin3t), turn t); same e/R: scale (R/R_ref, w/w_ref, R/R_ref); other e/R: regenerate")
for o in housing_o + plug_o + plate_o:
    tag(o, part=o.name, ref=ref, origin="shaft axis; waist (plugs) on +z, i.e. the 2D view's +y",
        rule="still; same e/R: scale (R/R_ref, w/w_ref, R/R_ref)")
for o in ecc_o:
    tag(o, part=o.name, ref=ref, origin="shaft axis; lobe centre at +x e (shaft angle 0)",
        rule="turns with the shaft (3t); lobe offset e")
export("wankel_rotor.glb", rotor_o)
export("wankel_housing.glb", housing_o + plug_o)
export("wankel_side_plate.glb", plate_o)
export("wankel_eccentric_shaft.glb", ecc_o)

if A.get("render"):
    # assembled at chamber angle psi = 0 (view::rotor_pose: theta = 30°), the shaft laid
    # horizontal (Blender Z -> Y) so the waist and plugs point up
    th = math.radians(30.0)
    cx, cy = E * math.cos(3 * th), E * math.sin(3 * th)
    lay = Matrix.Rotation(-math.pi / 2, 4, "X")

    def dup(o, mat):
        c = o.copy()
        c.data = o.data.copy()
        c.data.transform(mat)
        return link(c)

    rot = Matrix.Translation((cx, -cy, 0)) @ Matrix.Rotation(-th, 4, "Z")
    asm = [dup(o, lay @ rot) for o in rotor_o]
    asm += [dup(o, lay @ Matrix.Rotation(-3 * th, 4, "Z")) for o in ecc_o]
    asm += [dup(o, lay) for o in housing_o + plug_o + plate_o]
    render("wankel_assembly.png", asm, direction=(0.35, 1.0, 0.45), lens=60)
    render("wankel_front.png", asm, direction=(0.0, 1.0, 0.0), lens=85, floor=False)
    for o in asm:
        bpy.data.objects.remove(o)
    rl = [dup(o, lay) for o in rotor_o]
    render("wankel_rotor.png", rl, direction=(0.5, -1.0, 0.6), lens=70)
    render("wankel_rotor_gear_side.png", rl, direction=(-0.4, 1.0, 0.5), lens=70)
