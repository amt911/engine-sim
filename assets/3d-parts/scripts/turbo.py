"""Turbocharger: compressor housing, centre (bearing) housing, turbine housing with its wastegate
port; the compressor wheel (main blades + splitters, backswept), the radial turbine wheel and the
wastegate flap.

Frames (glTF; Blender Z up = glTF +Y), all in metres at the reference size, every part scaled
uniformly by s = compressor_tip_diameter_m / D_REF:
- turbo_housing.glb: origin on the shaft axis, midway between the wheels; shaft along +X, the
  turbine at +X, the compressor at -X. Node extras carry the ports (in this frame, glTF axes):
  `inlet` (the turbine's inlet flange centre, opening up +Y), `outlet` (the turbine's outlet face
  centre, opening +X), `charge` (the compressor's outlet centre, opening up +Y), `eye` (the
  compressor's inlet face centre, opening -X).
- turbo_compressor_wheel.glb / turbo_turbine_wheel.glb: origin on the axis at the wheel's back
  face, which sits at x = -X_WHEEL / +X_WHEEL in the housing frame; they turn about +X.
- turbo_wastegate_flap.glb: origin on its hinge axis (along +Z in glTF), placed at the extras'
  `hinge` point of the housing; it opens by turning about +Z by the flap angle (positive swings
  it out of the outlet face, towards +X).
Proportions are ESTIMATES from photographs of typical automotive turbos (a K20C1-class unit:
compressor wheel 56 mm), as fractions of the compressor wheel's tip diameter D.
Run: blender -b -P turbo.py [-- --render 1]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

A = args()
D = 0.056  # D_REF: the reference compressor wheel's tip diameter
R = D / 2
X_WHEEL = 0.75 * D  # each wheel's back face from the middle of the shaft
CAST = ("cast_iron", (0.25, 0.24, 0.23), 1.0, 0.55)
INCONEL = ("inconel", (0.45, 0.43, 0.40), 1.0, 0.35)
MAIN_BLADES = 6  # the K20C1 preset's `blades`; the splitters between them
TURBINE_BLADES = 11  # radial turbines carry 9-12 (ESTIMATE)

TO_COMP = Matrix.Translation((-X_WHEEL, 0, 0)) @ Matrix.Rotation(-math.pi / 2, 4, "Y")
TO_TURB = Matrix.Translation((X_WHEEL, 0, 0)) @ Matrix.Rotation(math.pi / 2, 4, "Y")


def merge(name, *obs):
    """The meshes of `obs` as one object, without booleans (they may overlap: drawn, not cut)."""
    bm = bmesh.new()
    for o in obs:
        bm.from_mesh(o.data)
        bpy.data.objects.remove(o)
    return from_bmesh(name, bm)


def blades(name, n, hub, shroud, theta, thick, s0=0.0, nu=24, nv=6):
    """`n` blades round local Z between the meridional curves `hub(s)` and `shroud(s)` -> (r, z),
    s in [s0, 1]; the camber line at angle theta(s) (+ k 2pi/n); `thick` m thick."""
    bm = bmesh.new()
    for k in range(n):
        a0 = k * 2 * math.pi / n
        grid = []
        for side in (-1, 1):
            rows = []
            for i in range(nu + 1):
                s = s0 + (1 - s0) * i / nu
                (rh, zh), (rs, zs) = hub(s), shroud(s)
                row = []
                for j in range(nv + 1):
                    t = j / nv
                    r = rh + (rs - rh) * t
                    z = zh + (zs - zh) * t
                    a = a0 + theta(s) + side * 0.5 * thick / max(r, 1e-4)
                    row.append(bm.verts.new((r * math.cos(a), r * math.sin(a), z)))
                rows.append(row)
            grid.append(rows)
        lo, hi = grid
        for i in range(nu):
            for j in range(nv):
                bm.faces.new((lo[i][j], lo[i][j + 1], lo[i + 1][j + 1], lo[i + 1][j]))
                bm.faces.new((hi[i][j], hi[i + 1][j], hi[i + 1][j + 1], hi[i][j + 1]))
        for i in range(nu):  # hub and tip edges
            bm.faces.new((lo[i][0], lo[i + 1][0], hi[i + 1][0], hi[i][0]))
            bm.faces.new((lo[i][nv], hi[i][nv], hi[i + 1][nv], lo[i + 1][nv]))
        for j in range(nv):  # leading and trailing edges
            bm.faces.new((lo[0][j], hi[0][j], hi[0][j + 1], lo[0][j + 1]))
            bm.faces.new((lo[nu][j], lo[nu][j + 1], hi[nu][j + 1], hi[nu][j]))
    return from_bmesh(name, bm)


def compressor_wheel():
    """Local frame: axis Z, back face at z = 0, the inducer facing +Z."""
    L, b = 0.42 * D, 0.07 * D  # axial length, exit (tip) width
    rh0, rs0 = 0.13 * D, 0.37 * D  # hub and shroud radii at the inducer
    hub = lambda s: (rh0 + (R - rh0) * s**2.2, L * (1 - s) ** 1.6)
    shroud = lambda s: (rs0 + (R - rs0) * s**3, b + (L - b) * (1 - s**1.5))
    # the inducer's lean against the flow, then 35 deg of backsweep at the exit
    theta = lambda s: 0.55 * (1 - s) ** 2 - 0.45 * s**2
    main = blades("cw_main", MAIN_BLADES, hub, shroud, theta, 0.012 * D)
    split = blades(
        "cw_split", MAIN_BLADES, hub, shroud, lambda s: theta(s) + math.pi / MAIN_BLADES, 0.010 * D, s0=0.35
    )
    prof = [(0.0, L + 0.08 * D), (0.06 * D, L + 0.07 * D), (rh0, L)]
    prof += [hub(i / 20) for i in range(1, 21)]
    prof += [(R, -0.03 * D), (0.30 * D, -0.06 * D), (0.12 * D, -0.06 * D), (0.0, -0.06 * D)]
    hub_o = lathe("cw_hub", [(r, z) for r, z in prof], segs=64)
    ob = union(hub_o, main, split)
    finish(ob, smooth=50)
    ob.data.transform(Matrix.Rotation(-math.pi / 2, 4, "Y"))  # inducer to -X
    ob.name = "compressor_wheel"
    return paint(tag(ob, part="turbo_compressor_wheel", ref=D, origin="axis at the back face, inducer -X", rule="uniform s; turns about X"), ALU)


def turbine_wheel():
    """Local frame: axis Z, back face at z = 0, the exducer facing +Z."""
    Rt = 0.95 * R
    Lt = 0.40 * D
    bt = 0.10 * D
    rh1, rs1 = 0.17 * D, 0.40 * D  # hub and tip radii at the exducer
    hub = lambda s: (Rt + (rh1 - Rt) * (1 - (1 - s) ** 2), Lt * s**1.8)
    shroud = lambda s: (Rt + (rs1 - Rt) * s**2, bt + (Lt - bt) * s**1.2)
    theta = lambda s: 0.65 * s**2.5  # radial at the rim, curved at the exducer
    bl = blades("tw_blades", TURBINE_BLADES, hub, shroud, theta, 0.014 * D)
    prof = [(0.0, -0.05 * D), (Rt * 0.9, -0.05 * D), (Rt, -0.01 * D)]
    prof += [hub(i / 20) for i in range(1, 21)]
    prof += [(0.08 * D, Lt + 0.02 * D), (0.0, Lt + 0.02 * D)]
    ob = union(lathe("tw_hub", prof, segs=64), bl)
    finish(ob, smooth=50)
    ob.data.transform(Matrix.Rotation(math.pi / 2, 4, "Y"))  # exducer to +X
    ob.name = "turbine_wheel"
    return paint(tag(ob, part="turbo_turbine_wheel", ref=D, origin="axis at the back face, exducer +X", rule="uniform s; turns about X"), INCONEL)


def volute(name, r0, rho0, rho1, z, phi_end, tail, segs=72, ring=24, reverse=False):
    """A scroll round local Z in the plane z: its centre line at r0 + rho, its section a circle
    of radius rho growing from rho0 to rho1 over the turn ending at angle phi_end, then a straight
    tube of `tail` m along the tangent there. Returns the object and the tube's end (x, y, z)."""
    bm = bmesh.new()
    rings = []

    def ring_at(c, er, rho):
        return [
            bm.verts.new(c + er * (rho * math.cos(u)) + Vector((0, 0, rho * math.sin(u))))
            for u in (2 * math.pi * k / ring for k in range(ring))
        ]

    for i in range(segs + 1):
        f = i / segs
        phi = phi_end - 2 * math.pi * (1 - f)
        rho = rho0 + (rho1 - rho0) * f
        er = Vector((math.cos(phi), math.sin(phi), 0))
        rings.append(ring_at(Vector((0, 0, z)) + er * (r0 + rho), er, rho))
    er = Vector((math.cos(phi_end), math.sin(phi_end), 0))
    tan = Vector((-math.sin(phi_end), math.cos(phi_end), 0))
    end_c = Vector((0, 0, z)) + er * (r0 + rho1) + tan * tail
    rings.append(ring_at(end_c, er, rho1))
    for a, b in zip(rings, rings[1:]):
        for k in range(ring):
            q = (a[k], a[(k + 1) % ring], b[(k + 1) % ring], b[k])
            bm.faces.new(q[::-1] if reverse else q)
    bm.faces.new(rings[0])  # the tongue's closed end; the port's end stays open (no boolean
    # touches the scroll: its last turn overlaps the tongue, which the exact solver rejects)
    return from_bmesh(name, bm), end_c


def housings():
    # compressor (local: wheel back face z = 0, inducer +Z; outlet tangent along local +X = up)
    comp_scroll, c_end = volute("c_scroll", 0.58 * D, 0.14 * D, 0.32 * D, 0.04 * D, -math.pi / 2, 0.50 * D)
    bead = cylinder("c_bead", 0.36 * D, c_end.x - 0.10 * D, c_end.x - 0.04 * D, axis="X", at=(c_end.y, c_end.z))
    comp_shroud = lathe(
        "c_shroud",
        [(0.40 * D, 1.00 * D), (0.40 * D, 0.50 * D), (0.385 * D, 0.43 * D), (0.45 * D, 0.25 * D),
         (R + 0.02 * D, 0.09 * D), (0.64 * D, 0.07 * D), (0.66 * D, 0.10 * D), (0.70 * D, 0.24 * D),
         (0.55 * D, 0.52 * D), (0.55 * D, 0.86 * D), (0.60 * D, 0.88 * D), (0.60 * D, 0.94 * D),
         (0.55 * D, 0.96 * D), (0.55 * D, 1.00 * D)],
        segs=64,
        closed=True,
    )
    comp_back = cylinder("c_back", 0.74 * D, -0.07 * D, 0.0, axis="Z", segs=64)
    out_bore = lambda: cylinder("c_out", 0.26 * D, c_end.x - 0.45 * D, c_end.x + 0.01 * D, axis="X", at=(c_end.y, c_end.z))
    cut(bead, out_bore())
    comp = merge("compressor_housing", comp_shroud, comp_scroll, bead, comp_back)
    finish(comp, bevel=0.004 * D, smooth=45)
    comp.data.transform(TO_COMP)
    comp.name = "compressor_housing"
    paint(comp, ALU)
    tag(comp, part="turbo_housing", ref=D, origin="shaft axis between the wheels", rule="uniform s")
    # centre housing: bearing body between the back plates, oil feed on top, drain below
    body = cylinder("centre", 0.36 * D, -X_WHEEL - 0.07 * D, X_WHEEL - 0.08 * D, axis="X", segs=48)
    rib = cylinder("centre_rib", 0.46 * D, -0.18 * D, 0.18 * D, axis="X", segs=48)
    oil = cylinder("oil_in", 0.07 * D, 0.30 * D, 0.58 * D, axis="Z")
    drain = box("drain", (-0.16 * D, -0.13 * D, -0.62 * D), (0.16 * D, 0.13 * D, -0.30 * D))
    shield = cylinder("heat_shield", 0.70 * D, X_WHEEL - 0.10 * D, X_WHEEL - 0.06 * D, axis="X", segs=48)
    ctr = merge("centre_housing", body, rib, oil, drain, shield)
    finish(ctr, bevel=0.004 * D, smooth=45)
    ctr.name = "centre_housing"
    paint(ctr, DARK)
    tag(ctr, part="turbo_housing", ref=D, origin="shaft axis between the wheels", rule="uniform s")
    # turbine (local: wheel back face z = 0, exducer +Z; inlet tangent along local -X = up)
    turb_scroll, t_end = volute("t_scroll", 0.56 * D, 0.10 * D, 0.30 * D, 0.06 * D, math.pi / 2, 0.40 * D)
    flange = box("t_flange", (t_end.x - 0.05 * D, t_end.y - 0.42 * D, t_end.z - 0.42 * D),
                 (t_end.x + 0.03 * D, t_end.y + 0.42 * D, t_end.z + 0.42 * D))
    outlet_z = 0.85 * D
    turb_body = lathe(
        "t_body",
        [(0.42 * D, 0.40 * D), (0.44 * D, 0.10 * D), (0.62 * D, 0.06 * D), (0.66 * D, 0.24 * D),
         (0.55 * D, 0.50 * D), (0.55 * D, outlet_z - 0.07 * D), (0.72 * D, outlet_z - 0.07 * D),
         (0.72 * D, outlet_z), (0.45 * D, outlet_z), (0.45 * D, 0.45 * D)],
        segs=64,
        closed=True,
    )
    # the wastegate port: a bore from the scroll beside the wheel's, out through a boss on the
    # outlet face, where the flap seats
    port_r, port_y = 0.13 * D, 0.62 * D
    port_boss = cylinder("wg_boss", port_r + 0.06 * D, 0.20 * D, outlet_z, axis="Z", at=(port_y, 0.0))
    in_bore = lambda: cylinder("t_in", 0.26 * D, t_end.x + 0.04 * D, t_end.x - 0.35 * D, axis="X", at=(t_end.y, t_end.z))
    cut(flange, in_bore())
    cut(port_boss, cylinder("wg_bore", port_r, 0.15 * D, outlet_z + 0.01 * D, axis="Z", at=(port_y, 0.0)))
    turb = merge("turbine_housing", turb_body, turb_scroll, flange, port_boss)
    finish(turb, bevel=0.004 * D, smooth=45)
    turb.data.transform(TO_TURB)
    turb.name = "turbine_housing"
    paint(turb, CAST)
    # the ports, carried to the housing frame (Blender) and then to glTF (x, z, -y)
    gl = lambda v: [round(v.x, 6), round(v.z, 6), round(-v.y, 6)]
    inlet = TO_TURB @ t_end
    outlet = TO_TURB @ Vector((0, 0, outlet_z))
    charge = TO_COMP @ c_end
    eye = TO_COMP @ Vector((0, 0, 1.00 * D))
    # the flap's hinge: on the outlet face at the port's edge away from the wheel
    port_c = TO_TURB @ Vector((port_y, 0.0, outlet_z))
    hinge = port_c + Vector((0, 0, port_r + 0.03 * D))
    tag(turb, part="turbo_housing", ref=D, origin="shaft axis between the wheels, turbine +X",
        rule="uniform s", inlet=gl(inlet), outlet=gl(outlet), charge=gl(charge), eye=gl(eye),
        hinge=gl(hinge), wheel_x=round(X_WHEEL, 6))
    return [comp, ctr, turb], port_c, hinge, port_r


def flap(port_c, hinge, port_r):
    """The wastegate's flap on its seat over the port, its arm to the hinge; origin at the hinge."""
    disc = cylinder("flap_disc", port_r + 0.04 * D, 0.0, 0.035 * D, axis="X", at=(port_c.y, port_c.z))
    disc.data.transform(Matrix.Translation((port_c.x, 0, 0)))
    arm = box("flap_arm", (port_c.x + 0.035 * D, -0.06 * D, port_c.z), (port_c.x + 0.075 * D, 0.06 * D, hinge.z))
    pin = cylinder("flap_pin", 0.05 * D, -0.12 * D, 0.12 * D, axis="Y", at=(port_c.x + 0.055 * D, hinge.z))
    ob = union(disc, arm, pin)
    finish(ob, bevel=0.003 * D, smooth=45)
    ob.data.transform(Matrix.Translation(-Vector((port_c.x + 0.055 * D, 0.0, hinge.z))))
    ob.name = "wastegate_flap"
    return paint(tag(ob, part="turbo_wastegate_flap", ref=D, origin="hinge axis (glTF +Z)",
                     rule="uniform s; opens by turning about glTF +Z"), FORGED)


reset()
cw = compressor_wheel()
tw = turbine_wheel()
hs, port_c, hinge, port_r = housings()
fl = flap(port_c, hinge, port_r)
export("turbo_housing.glb", hs)
export("turbo_compressor_wheel.glb", [cw])
export("turbo_turbine_wheel.glb", [tw])
export("turbo_wastegate_flap.glb", [fl])
if A.get("render"):

    def placed(o, m):
        c = o.copy()
        c.data = o.data.copy()
        link(c)
        c.data.transform(m)
        return c

    asm = hs + [
        placed(cw, Matrix.Translation((-X_WHEEL, 0, 0))),
        placed(tw, Matrix.Translation((X_WHEEL, 0, 0))),
        placed(fl, Matrix.Translation((port_c.x + 0.055 * D, 0, hinge.z)) @ Matrix.Rotation(math.radians(-25), 4, "Y")),
    ]
    render("turbo.png", asm, direction=(0.6, -1.0, 0.55), lens=60)
    render("turbo_eye.png", asm, direction=(-1.0, -0.7, 0.5), lens=60)
    render("turbo_wheels.png", [cw, placed(tw, Matrix.Translation((1.2 * D, 0, 0)))], direction=(-0.2, -1.0, 0.35), lens=70)
