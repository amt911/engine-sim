"""Valve train (reference image refs/2.png): valve + keepers + retainer, spring, cam lobe,
rocker arm.

Frames (glTF; Blender Z up = glTF +Y):
- valve.glb: origin at the valve face centre with the valve closed, stem up +Y. The scene puts
  it at (x, deck - lift, z) as its disc now (head radius 0.17 bore, as view3d).
  Rule: p' = s*p + w*(stem_m - s*L_REF)*Y, s = bore_m/0.086, w = WEIGHTS_0 of joint `stretch`
  (0 head, 1 tip/keepers/retainer, linear along the stem).
- valve_spring.glb: origin at the spring's seat in the head, axis +Y, free of the valve.
  Installed height H_REF closed; with lift: scale y by (s*H_REF - lift)/(s*H_REF) about its
  origin (a real spring shortens the same way, coils closing up). Seat it at
  y_seat = SEAT_REF*s (+ stem stretch) in the valve's closed frame.
- cam_lobe.glb: camshaft along +X, lobe nose at +Y (full lift at cam angle 0), with a stub of
  shaft. A flat-follower (bucket) cam. Lift: morph target 0, weight = lift/(s*LIFT_REF) - 1.
- rocker_arm.glb: rocker shaft along +X through the origin, valve end at +Z (glTF -Z is the
  Blender +Y side: see README), rigid, uniform scale s.
Run: blender -b -P valvetrain.py [-- --render 1]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

A = args()
mm = MM
B = 0.086
HEAD_R = 0.17 * B  # = view3d's valve disc
STEM_R = 0.032 * B
L_REF = 1.15 * B  # face to tip
H_REF = 0.44 * B  # spring installed height
COIL_R = 0.135 * B  # spring mean coil radius
WIRE_R = 0.022 * B
FLANGE_LO = L_REF - 4.5 * mm  # retainer's spring seat (underside of its flange)
SEAT_REF = FLANGE_LO - H_REF  # spring seat in the head, valve closed
R_BASE = 0.22 * B
LIFT_REF = 0.11 * B
HALF_DUR = math.radians(80)  # cam half duration, base circle to nose (cam degrees)
T_FLANK = math.radians(20)  # positive-acceleration flank, then constant deceleration to the nose
TI = ("valve_steel", (0.55, 0.56, 0.58), 1.0, 0.25)


def valve():
    r_h, r_s, L = HEAD_R, STEM_R, L_REF
    prof = [(0.0, 0.25 * mm), (r_h - 0.5 * mm, 0.0), (r_h, 0.5 * mm), (r_h, 1.4 * mm), (r_h - 2.2 * mm, 3.6 * mm)]
    # tulip: a quarter ellipse from the seat's top to the stem (concave, tangent to the stem)
    a, bz = (r_h - 2.2 * mm) - r_s, 0.23 * B - 3.6 * mm
    for i in range(1, 25):
        t = math.pi / 2 * i / 24
        prof.append(((r_h - 2.2 * mm) - a * math.sin(t), 0.23 * B - bz * math.cos(t)))
    # keeper beads near the tip
    z = L - 9.0 * mm
    prof.append((r_s, z))
    for k in range(3):
        g = L - 8.4 * mm + k * 2.0 * mm
        prof += [(r_s, g), (r_s - 0.35 * mm, g + 0.25 * mm), (r_s - 0.35 * mm, g + 0.55 * mm), (r_s, g + 0.8 * mm)]
    prof += [(r_s, L - 0.5 * mm), (r_s - 0.5 * mm, L), (0.0, L)]
    v = lathe("valve", prof, segs=48)
    finish(paint(v, TI), 0, smooth=35)

    keep = lathe("keepers", [(r_s - 0.05 * mm, L - 9.5 * mm), (3.9 * mm, L - 9.5 * mm), (5.3 * mm, L - 2.0 * mm), (r_s - 0.05 * mm, L - 2.0 * mm)], segs=32, closed=True)
    cut(keep, box("gap", (-0.3 * mm, -10 * mm, L - 12 * mm), (0.3 * mm, 10 * mm, L)))
    finish(paint(keep, DARK), 0.1 * mm, 1, angle=40)

    ret = lathe(
        "retainer",
        [
            (3.95 * mm, L - 9.5 * mm), (7.6 * mm, L - 9.5 * mm), (8.2 * mm, L - 8.9 * mm),
            (8.2 * mm, FLANGE_LO), (COIL_R + WIRE_R + 1.2 * mm, FLANGE_LO),
            (COIL_R + WIRE_R + 1.6 * mm, FLANGE_LO + 0.4 * mm), (COIL_R + WIRE_R + 1.6 * mm, L - 2.4 * mm),
            (COIL_R + WIRE_R + 1.0 * mm, L - 1.8 * mm), (5.35 * mm, L - 1.8 * mm),
        ],
        segs=56,
        closed=True,
    )
    finish(paint(ret, POLISHED), 0.15 * mm, 1, angle=40)
    return [v, keep, ret]


def spring():
    """Helix with closed ends (pitch = wire diameter over the last 0.85 turn each end), ground
    flat to [0, H]."""
    n_total, segs_turn, ring = 6.5, 40, 10
    p_closed = 2 * WIRE_R
    ends = 0.85
    p_active = (H_REF - 2 * ends * p_closed) / (n_total - 2 * ends)

    def smooth(x):
        x = min(1.0, max(0.0, x))
        return x * x * (3 - 2 * x)

    n = int(n_total * segs_turn)
    zs, z = [], -0.35 * WIRE_R
    for i in range(n + 1):
        turns = i / segs_turn
        up = smooth((turns - ends + 0.25) / 0.5) * smooth((n_total - ends + 0.25 - turns) / 0.5)
        zs.append(z)
        z += (p_closed + (p_active - p_closed) * up) / segs_turn
    zs = np.array(zs)
    zs = (zs - zs[0]) * (H_REF + 0.7 * WIRE_R) / (zs[-1] - zs[0]) - 0.35 * WIRE_R
    bm = bmesh.new()
    rings = []
    for i in range(n + 1):
        th = 2 * math.pi * i / segs_turn
        c = Vector((COIL_R * math.cos(th), COIL_R * math.sin(th), zs[i]))
        radial = Vector((math.cos(th), math.sin(th), 0.0))
        tangent = Vector((-math.sin(th) * COIL_R * 2 * math.pi, math.cos(th) * COIL_R * 2 * math.pi, (zs[min(i + 1, n)] - zs[max(i - 1, 0)]) / (2 / segs_turn))).normalized()
        binorm = tangent.cross(radial).normalized()
        nrm = binorm.cross(tangent).normalized()
        rings.append([bm.verts.new(c + WIRE_R * (math.cos(2 * math.pi * k / ring) * nrm + math.sin(2 * math.pi * k / ring) * binorm)) for k in range(ring)])
    for a_, b_ in zip(rings, rings[1:]):
        for k in range(ring):
            bm.faces.new((a_[k], a_[(k + 1) % ring], b_[(k + 1) % ring], b_[k]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    s = from_bmesh("valve_spring", bm)
    # ground ends: keep only 0 <= z <= H
    keep = box("grind", (-3 * COIL_R, -3 * COIL_R, 0.0), (3 * COIL_R, 3 * COIL_R, H_REF))
    boolean(s, keep, "INTERSECT")
    return finish(paint(s, SPRING), 0, smooth=50)


def lift_shape(theta):
    """Unit lift law g(theta) (g = lift / lift_max), symmetric about the nose at theta = 0:
    tau = HALF_DUR - |theta| from the start of lift; a half-sine acceleration pulse over
    T_FLANK, then constant deceleration to zero velocity at the nose. Linear in the lift, so the
    cam profile is too."""
    h, t1 = HALF_DUR, T_FLANK
    t2 = h - t1
    v1 = 2.0 / h  # unit lift: g = v1 * h / 2
    a1 = math.pi * v1 / (2 * t1)
    a2 = v1 / t2
    tau = h - np.abs(theta)
    g = np.zeros_like(theta)
    m1 = (tau > 0) & (tau <= t1)
    g[m1] = a1 * t1 / math.pi * (tau[m1] - t1 / math.pi * np.sin(math.pi * tau[m1] / t1))
    m2 = tau > t1
    sv = tau[m2] - t1
    g[m2] = v1 * t1 / 2 + v1 * sv - a2 * sv**2 / 2
    return g


def cam_outline(lift, n=720):
    """Flat-faced follower cam (a bucket tappet): the envelope of the follower's face,
    P = (r_b + y) e_r + y' e_t (Norton, *Cam Design and Manufacturing Handbook*, 2nd ed., §7.6),
    convex iff r_b + y + y'' > 0. Nose (theta 0) on +Z; outline in (y, z)."""
    th = np.linspace(-math.pi, math.pi, 7200, endpoint=False)
    y = lift * lift_shape(th)
    d1 = np.gradient(y, th)
    d2 = np.gradient(d1, th)
    curv = (R_BASE + y + d2).min()
    pts_y = (R_BASE + y) * np.sin(th) + d1 * np.cos(th)
    pts_z = (R_BASE + y) * np.cos(th) - d1 * np.sin(th)
    k = len(th) // n
    return list(zip(pts_y[::k], pts_z[::k])), curv


def cam():
    pts, curv = cam_outline(LIFT_REF)
    print(f"[valvetrain] cam: min radius of curvature {curv * 1e3:.2f} mm (must be > 0)")
    assert curv > 0, "cam lobe is not convex: a flat follower cannot ride it"
    w = 0.17 * B
    lobe = prism("cam_lobe", pts, -w / 2, w / 2, "X")
    finish(paint(lobe, POLISHED), 0.5 * mm, 2, angle=30, smooth=30)
    # morph target: the same construction at 1.25x lift; the delta scaled by 4 is the exact
    # change for +LIFT_REF (the profile is linear in the lift), so weight = lift/lift_ref - 1
    pts2, _ = cam_outline(1.25 * LIFT_REF)
    l2 = prism("cam_lobe_2", pts2, -w / 2, w / 2, "X")
    finish(l2, 0.5 * mm, 2, angle=30, smooth=30)
    assert len(l2.data.vertices) == len(lobe.data.vertices), "lift morph: topology differs"
    lobe.shape_key_add(name="Basis")
    sk = lobe.shape_key_add(name="lift_plus_ref")
    for i, v in enumerate(lobe.data.vertices):
        sk.data[i].co = v.co + 4.0 * (l2.data.vertices[i].co - v.co)
    bpy.data.objects.remove(l2)
    sh = cylinder("camshaft", 0.15 * B, -0.3 * B, 0.3 * B, "X", segs=40)
    finish(paint(sh, FORGED), 0.4 * mm, 1, angle=40)
    return [lobe, sh]


def band(c1, r1, c2, r2):
    """The quadrilateral joining two circles (c, r) in the (y, z) plane along their outer
    tangents, so a union of circles and bands reads as one forged arm."""
    (y1, z1), (y2, z2) = c1, c2
    d = math.hypot(y2 - y1, z2 - z1)
    ny, nz = -(z2 - z1) / d, (y2 - y1) / d
    return [(y1 + r1 * ny, z1 + r1 * nz), (y2 + r2 * ny, z2 + r2 * nz), (y2 - r2 * ny, z2 - r2 * nz), (y1 - r1 * ny, z1 - r1 * nz)]


def smooth_arm(p0, r0, pm, p1, r1, rm, n=48):
    """One outline for the whole arm: a quadratic Bezier centreline from p0 to p1 pulled
    toward pm, its half-width easing from r0 through rm to r1, offset to both sides. A single
    piece, no crease where two straight bands would meet."""
    c = [np.array(p0), 2 * np.array(pm) - 0.5 * (np.array(p0) + np.array(p1)), np.array(p1)]
    left, right = [], []
    for i in range(n + 1):
        t = i / n
        pt = (1 - t) ** 2 * c[0] + 2 * (1 - t) * t * c[1] + t**2 * c[2]
        tg = 2 * (1 - t) * (c[1] - c[0]) + 2 * t * (c[2] - c[1])
        nv = np.array([-tg[1], tg[0]]) / np.linalg.norm(tg)
        # quadratic through (0, r0), (0.5, rm), (1, r1)
        hw = r0 * (1 - t) * (1 - 2 * t) + 4 * rm * t * (1 - t) + r1 * t * (2 * t - 1)
        left.append(tuple(pt + hw * nv))
        right.append(tuple(pt - hw * nv))
    return left + right[::-1]


def disc(c, r, w, segs=48):
    return cylinder("d", r, -w, w, "X", segs=segs, at=c)


def rocker():
    """refs/2.png: a pivot boss on the rocker shaft, a slim arm curving down to the valve end
    (adjusting screw + lock nut), a slipper pad under the arm riding the cam."""
    piv, piv_r = (0.0, 0.0), 8.0 * mm
    mid, mid_r = (24 * mm, -3.0 * mm), 4.2 * mm
    end, end_r = (42 * mm, -7 * mm), 5.8 * mm
    pad, pad_r = (13 * mm, -10.5 * mm), 5.2 * mm
    arm = disc(piv, piv_r, 6.5 * mm)
    for part in (
        prism("arm", smooth_arm(piv, 6.0 * mm, mid, end, 4.6 * mm, mid_r), -4.0 * mm, 4.0 * mm, "X"),
        disc(end, end_r, 5.0 * mm),
        prism("p1", band(piv, 5.5 * mm, pad, pad_r), -4.5 * mm, 4.5 * mm, "X"),
        disc(pad, pad_r, 4.5 * mm),
    ):
        union(arm, part)
    arm.name = arm.data.name = "rocker_arm"
    cut(arm, cylinder("shaft", 5.2 * mm, -9 * mm, 9 * mm, "X", segs=40))
    cut(arm, cylinder("adj", 3.6 * mm, end[1] - 9 * mm, end[1] + 9 * mm, "Z", segs=24, at=(0.0, end[0])))
    finish(paint(arm, FORGED), 0.7 * mm, 2, angle=35)
    screw = cylinder("rocker_adjuster", 3.5 * mm, end[1] - 14 * mm, end[1] + 10 * mm, "Z", segs=24, at=(0.0, end[0]))
    tip = cylinder("tip", 3.5 * mm, end[1] - 14 * mm, end[1] - 16 * mm, "Z", segs=24, at=(0.0, end[0]), r2=2.6 * mm)
    nut = cylinder("nut", 5.4 * mm, end[1] + 6.4 * mm, end[1] + 10.0 * mm, "Z", segs=6, at=(0.0, end[0]))
    union(screw, tip, nut)
    screw.name = screw.data.name = "rocker_adjuster"
    finish(paint(screw, DARK), 0.3 * mm, 1, angle=40)
    return [arm, screw]


reset()
valve_o = valve()
spring_o = spring()
cam_o = cam()
rocker_o = rocker()

ref = {"bore_m": B, "stem_m": L_REF, "head_r_m": HEAD_R, "spring_installed_m": H_REF, "spring_seat_m": SEAT_REF,
       "cam_base_r_m": R_BASE, "cam_lift_m": LIFT_REF, "cam_half_duration_deg": math.degrees(HALF_DUR)}
for o in valve_o:
    tag(o, part=o.name, ref=ref, origin="valve face centre, valve closed; stem +Y",
        rule="p' = s*p + w*(stem_m - s*stem_ref)*Y; the whole valve moves -lift")
tag(spring_o, part="valve_spring", ref=ref, origin="spring seat in the head; axis +Y",
    rule="scale y by (s*H - lift)/(s*H) about the origin; place at y = s*spring_seat_m in the closed valve frame")
for o in cam_o:
    tag(o, part=o.name, ref=ref, origin="camshaft axis (+X); nose +Y at cam angle 0",
        rule="lift: morph target 0 weight = lift/(s*cam_lift_m) - 1 (exact); turns at half crank speed")
for o in rocker_o:
    tag(o, part=o.name, ref=ref, origin="rocker shaft axis (+X); valve end along Blender +Y", rule="rigid, uniform scale s")

# preview copies first (no skin): an intake/exhaust pair under their cam lobes, as refs/2.png
asm = []


def dup(o, mat):
    c = o.copy()
    c.data = o.data.copy()
    link(c)
    if c.data.shape_keys:
        c.shape_key_clear()  # mesh.transform() does not move shape keys
    c.data.transform(mat)
    return c


if A.get("render"):

    for y, scale, lift, cam_a in ((-0.25 * B, 1.0, 0.0, 0.0), (0.25 * B, 0.88, LIFT_REF, 180.0)):
        tilt = Matrix.Rotation(math.radians(14 if y < 0 else -14), 4, "X")  # stems lean outward (pent roof)
        base = Matrix.Translation((0, y, 0)) @ tilt
        mv = base @ Matrix.Translation((0, 0, -lift)) @ Matrix.Scale(scale, 4)
        asm += [dup(o, mv) for o in valve_o]
        sp = base @ Matrix.Translation((0, 0, SEAT_REF)) @ Matrix.Diagonal((1, 1, (H_REF - lift) / H_REF, 1))
        asm.append(dup(spring_o, sp))
        # the camshaft is fixed: base circle on the closed valve's tip; nose down at full lift
        cm = base @ Matrix.Translation((0, 0, L_REF + R_BASE + 0.4 * mm)) @ Matrix.Rotation(math.radians(cam_a), 4, "X")
        asm += [dup(o, cm) for o in cam_o]

w_stem0, w_stem1 = 0.23 * B + 2 * mm, L_REF - 12 * mm
arm_v = skin("valve_rig", valve_o, (0, 0, L_REF), lambda co: (co.z - w_stem0) / (w_stem1 - w_stem0))


export("valve.glb", valve_o + [arm_v])
export("valve_spring.glb", [spring_o])
export("cam_lobe.glb", cam_o)
export("rocker_arm.glb", rocker_o)
if A.get("render"):
    render("valvetrain.png", asm, direction=(1.0, -0.25, 0.3), lens=55)
    seated = dup(spring_o, Matrix.Translation((0, 0, SEAT_REF)))
    render("valve_parts.png", valve_o + [seated], direction=(1.0, -0.6, 0.35), lens=70, margin=1.3)
    render("cam_lobe.png", cam_o, direction=(1.0, -0.8, 0.5), lens=70)
    render("rocker_arm.png", rocker_o, direction=(1.0, -0.35, 0.5), lens=70)
