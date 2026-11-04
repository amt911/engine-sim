"""Shared helpers for the parametric part scripts (run inside Blender: `blender -b -P <part>.py`).

Frame (Blender, Z up) -> glTF (Y up) on export: Blender (x, y, z) -> glTF (x, z, -y).
Every engine part is modelled with the crank / camshaft / eccentric-shaft axis on Blender X and
the cylinder (or valve) axis on Blender Z, so in the exported glTF the crank is +X and the
cylinder is +Y, as in engine-sim's `view3d` world frame. Units: metres.
"""

import json
import math
import os
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "out")
RENDERS = os.path.join(ROOT, "renders")
MM = 1e-3


def args():
    """Arguments after `--` on the Blender command line, as a dict of floats (`--bore 0.086`)."""
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    out = {}
    for k, v in zip(argv[::2], argv[1::2]):
        out[k.lstrip("-").replace("-", "_")] = v
    return out


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    s = bpy.context.scene
    s.unit_settings.system = "METRIC"
    s.unit_settings.scale_length = 1.0


def link(ob):
    bpy.context.scene.collection.objects.link(ob)
    return ob


def from_bmesh(name, bm):
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-8)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    return link(bpy.data.objects.new(name, me))


def lathe(name, prof, segs=64, closed=False):
    """Revolve the (r, z) polyline `prof` about Blender Z. An open profile must start and end
    on the axis (r = 0); a closed one (a ring section) is joined back to its first point."""
    bm = bmesh.new()
    vs = [bm.verts.new((r, 0.0, z)) for r, z in prof]
    pairs = list(zip(vs, vs[1:]))
    if closed:
        pairs.append((vs[-1], vs[0]))
    for a, b in pairs:
        bm.edges.new((a, b))
    bmesh.ops.spin(
        bm,
        geom=bm.verts[:] + bm.edges[:],
        cent=(0, 0, 0),
        axis=(0, 0, 1),
        angle=2 * math.pi,
        steps=segs,
        use_merge=True,
    )
    return from_bmesh(name, bm)


def prism(name, pts, h0, h1, axis="X"):
    """The closed 2D outline `pts` extruded along `axis` from h0 to h1.
    axis X: pts are (y, z); axis Z: pts are (x, y); axis Y: pts are (x, z)."""
    bm = bmesh.new()

    def p3(u, v, h):
        return {"X": (h, u, v), "Z": (u, v, h), "Y": (u, h, v)}[axis]

    vs = [bm.verts.new(p3(u, v, h0)) for u, v in pts]
    f = bm.faces.new(vs)
    ret = bmesh.ops.extrude_face_region(bm, geom=[f])
    nv = [g for g in ret["geom"] if isinstance(g, bmesh.types.BMVert)]
    d = {"X": (h1 - h0, 0, 0), "Z": (0, 0, h1 - h0), "Y": (0, h1 - h0, 0)}[axis]
    bmesh.ops.translate(bm, verts=nv, vec=d)
    return from_bmesh(name, bm)


def cylinder(name, r, a, b, axis="X", segs=48, at=(0.0, 0.0), r2=None):
    """A capped cylinder (cone if r2) along `axis` from a to b, its axis through `at`
    (the two other coordinates, in x-y-z order)."""
    bm = bmesh.new()
    bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        cap_tris=False,
        segments=segs,
        radius1=r,
        radius2=r if r2 is None else r2,
        depth=abs(b - a),
    )
    ob = from_bmesh(name, bm)
    mid = 0.5 * (a + b)
    flip = b < a
    if axis == "X":
        m = Matrix.Rotation(math.pi / 2 * (-1 if flip else 1), 4, "Y")
        loc = (mid, at[0], at[1])
    elif axis == "Y":
        m = Matrix.Rotation(-math.pi / 2 * (-1 if flip else 1), 4, "X")
        loc = (at[0], mid, at[1])
    else:
        m = Matrix.Rotation(math.pi if flip else 0.0, 4, "X")
        loc = (at[0], at[1], mid)
    ob.data.transform(Matrix.Translation(loc) @ m)
    return ob


def box(name, lo, hi):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    ob = from_bmesh(name, bm)
    lo, hi = Vector(lo), Vector(hi)
    ob.data.transform(
        Matrix.Translation((lo + hi) / 2) @ Matrix.Diagonal((*(hi - lo), 1.0))
    )
    return ob


def apply_mods(ob):
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data
    ob.modifiers.clear()
    ob.data = me
    bpy.data.meshes.remove(old)
    me.name = ob.name


def boolean(target, cutter, op="DIFFERENCE"):
    m = target.modifiers.new("bool", "BOOLEAN")
    m.operation = op
    m.object = cutter
    m.solver = "EXACT"
    apply_mods(target)
    bpy.data.objects.remove(cutter)
    return target


def union(target, *others):
    for o in others:
        boolean(target, o, "UNION")
    return target


def cut(target, *cutters):
    for c in cutters:
        boolean(target, c, "DIFFERENCE")
    return target


def finish(ob, bevel=0.0, segments=2, angle=35.0, smooth=40.0):
    """Edge bevel (angle-limited), then smooth shading with sharp edges past `smooth` degrees."""
    if bevel > 0:
        m = ob.modifiers.new("bevel", "BEVEL")
        m.width = bevel
        m.segments = segments
        m.limit_method = "ANGLE"
        m.angle_limit = math.radians(angle)
        m.use_clamp_overlap = True
        apply_mods(ob)
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-8)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=1e-8)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(ob.data)
    bm.free()
    ob.data.shade_smooth()
    ob.data.set_sharp_from_angle(angle=math.radians(smooth))
    # face-area weighted normals: flat faces stay flat after booleans and bevels (no dark
    # smudges across big n-gons), exported to glTF as the vertex normals
    wn = ob.modifiers.new("wn", "WEIGHTED_NORMAL")
    wn.keep_sharp = True
    wn.weight = 100
    apply_mods(ob)
    return ob


_MATS = {}


def material(name, rgb, metallic=1.0, rough=0.4):
    if name in _MATS:
        return _MATS[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = rough
    m.diffuse_color = (*rgb, 1.0)
    _MATS[name] = m
    return m


# Shared palette (linear RGB), also listed in README.md so the renderer can reuse it.
ALU = ("cast_aluminium", (0.62, 0.63, 0.64), 1.0, 0.42)
FORGED = ("forged_steel", (0.30, 0.30, 0.31), 1.0, 0.55)
POLISHED = ("polished_steel", (0.78, 0.78, 0.80), 1.0, 0.12)
DARK = ("dark_steel", (0.18, 0.18, 0.19), 1.0, 0.35)
BRONZE = ("bearing_bronze", (0.62, 0.45, 0.25), 1.0, 0.30)
VALVE = ("valve_steel", (0.55, 0.56, 0.58), 1.0, 0.25)
SPRING = ("spring_steel", (0.12, 0.12, 0.13), 1.0, 0.30)


def paint(ob, mat):
    ob.data.materials.clear()
    ob.data.materials.append(material(*mat))
    return ob


def skin(name, obs, stretch_point, weight):
    """A two-joint skin: `root` at the origin and `stretch` at `stretch_point` (Blender coords).
    `weight(co) -> w in [0, 1]` is the stretch joint's weight per vertex; the root gets 1 - w.
    Moving the `stretch` joint by d moves every vertex by w * d."""
    ad = bpy.data.armatures.new(name)
    arm = link(bpy.data.objects.new(name, ad))
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    root = ad.edit_bones.new("root")
    root.head, root.tail = (0, 0, 0), (0, 0, 0.01)
    st = ad.edit_bones.new("stretch")
    p = Vector(stretch_point)
    st.head, st.tail = p, p + Vector((0, 0, 0.01))
    st.parent = root
    bpy.ops.object.mode_set(mode="OBJECT")
    for ob in obs:
        g0 = ob.vertex_groups.new(name="root")
        g1 = ob.vertex_groups.new(name="stretch")
        for v in ob.data.vertices:
            w = float(min(1.0, max(0.0, weight(v.co))))
            g0.add([v.index], 1.0 - w, "REPLACE")
            g1.add([v.index], w, "REPLACE")
        mod = ob.modifiers.new("armature", "ARMATURE")
        mod.object = arm
        ob.parent = arm
    return arm


def tag(ob, **extras):
    """glTF node extras (exported with export_extras)."""
    ob["engine_sim"] = json.dumps(extras)
    return ob


def export(filename, obs):
    bpy.ops.object.select_all(action="DESELECT")
    for o in obs:
        o.select_set(True)
    path = os.path.join(OUT, filename)
    bpy.ops.export_scene.gltf(
        filepath=path,
        export_format="GLB",
        use_selection=True,
        export_apply=True,
        export_extras=True,
        export_yup=True,
        export_skins=True,
        export_def_bones=False,
        export_animations=False,
        export_normals=True,
        export_texcoords=False,
    )
    tris = 0
    for o in obs:
        if o.type == "MESH":
            o.data.calc_loop_triangles()
            tris += len(o.data.loop_triangles)
    print(f"[export] {path}: {tris} triangles")
    return path


# ---- preview render ----------------------------------------------------------------------


def _look_at(ob, target):
    d = Vector(target) - ob.location
    ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def render(filename, obs, direction=(1.0, -1.4, 0.8), lens=70, res=(1200, 800), margin=1.15, floor=True):
    """Studio render (white backdrop, soft key/fill/rim) framing `obs`, Cycles on the CPU."""
    s = bpy.context.scene
    hidden = [o for o in s.objects if o.type == "MESH" and o not in obs and not o.hide_render]
    for o in hidden:
        o.hide_render = True
    pts = [o.matrix_world @ Vector(c) for o in obs if o.type == "MESH" for c in o.bound_box]
    lo = Vector([min(p[i] for p in pts) for i in range(3)])
    hi = Vector([max(p[i] for p in pts) for i in range(3)])
    c, size = (lo + hi) / 2, (hi - lo).length
    d = Vector(direction).normalized()
    cam = link(bpy.data.objects.new("cam", bpy.data.cameras.new("cam")))
    cam.data.lens = lens
    fov = 2 * math.atan(18.0 / lens)
    dist = margin * size / 2 / math.tan(fov / 2) * max(1.0, res[1] / res[0] * 1.2)
    cam.location = c + d * dist
    _look_at(cam, c)
    cam.data.clip_start, cam.data.clip_end = dist / 100, dist * 10
    s.camera = cam
    for nm, off, energy, sz in [
        ("key", (1.0, -1.0, 1.4), 300, 0.45),
        ("fill", (-1.4, -0.6, 0.6), 90, 0.8),
        ("rim", (-0.4, 1.4, 1.0), 220, 0.35),
    ]:
        L = link(bpy.data.objects.new(nm, bpy.data.lights.new(nm, "AREA")))
        L.data.energy = energy * size**2
        L.data.size = sz * size
        L.location = c + Vector(off).normalized() * size * 2.2
        _look_at(L, c)
    # shadow-catching floor under the part
    fl = []
    if floor:
        fl = [box("floor", (c.x - 20 * size, c.y - 20 * size, lo.z - 0.002), (c.x + 20 * size, c.y + 20 * size, lo.z - 0.001))]
        paint(fl[0], ("floor", (0.9, 0.9, 0.9), 0.0, 0.8))
        fl[0].is_shadow_catcher = True
        fl[0].visible_glossy = False
        fl[0].visible_diffuse = False
    # camera rays see a light studio backdrop; reflections see a dark-to-light gradient
    # (a studio's black flags and soft boxes), so polished metal reads as metal, not white
    w = bpy.data.worlds.new("w")
    w.use_nodes = True
    nt = w.node_tree
    bg = nt.nodes["Background"]
    bg.inputs["Color"].default_value = (0.8, 0.81, 0.83, 1)
    bg.inputs["Strength"].default_value = 0.35
    refl = nt.nodes.new("ShaderNodeBackground")
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.02, 0.02, 0.022, 1)
    ramp.color_ramp.elements[1].color = (0.9, 0.9, 0.92, 1)
    ramp.color_ramp.elements[0].position = 0.45
    ramp.color_ramp.elements[1].position = 0.75
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    coord = nt.nodes.new("ShaderNodeTexCoord")
    nt.links.new(coord.outputs["Generated"], sep.inputs[0])
    nt.links.new(sep.outputs["Z"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], refl.inputs["Color"])
    refl.inputs["Strength"].default_value = 0.5
    lp = nt.nodes.new("ShaderNodeLightPath")
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
    nt.links.new(refl.outputs[0], mix.inputs[1])
    nt.links.new(bg.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], nt.nodes["World Output"].inputs["Surface"])
    s.world = w
    s.render.engine = "CYCLES"
    s.cycles.device = "CPU"
    s.cycles.samples = 64
    s.cycles.use_denoising = True
    s.render.film_transparent = False
    s.render.threads_mode = "FIXED"
    s.render.threads = 8
    s.render.resolution_x, s.render.resolution_y = res
    s.view_settings.view_transform = "AgX"
    s.render.filepath = os.path.join(RENDERS, filename)
    bpy.ops.render.render(write_still=True)
    print(f"[render] {s.render.filepath}")
    for o in hidden:
        o.hide_render = False
    for o in [cam] + fl + [o for o in s.objects if o.type == "LIGHT"]:
        bpy.data.objects.remove(o)
