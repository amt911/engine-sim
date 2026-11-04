"""Check the exported .glb files the way the app will read them (plain Python + numpy, no
Blender): finite positions, unit normals, skins where expected, key dimensions, and that the
stretch rule lands the stretched point where it should.
Run: python3 scripts/verify.py
"""

import json
import os
import struct
import sys

import numpy as np

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out")
COMP = {5126: np.float32, 5123: np.uint16, 5121: np.uint8, 5125: np.uint32}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
fails = []


def load(name):
    raw = open(os.path.join(OUT, name), "rb").read()
    magic, _, _ = struct.unpack_from("<4sII", raw, 0)
    assert magic == b"glTF"
    jl, _ = struct.unpack_from("<I4s", raw, 12)
    doc = json.loads(raw[20 : 20 + jl])
    bl = struct.unpack_from("<I", raw, 20 + jl)[0]
    binbuf = raw[28 + jl : 28 + jl + bl]

    def view(bv_i, dt, count, ncomp, extra_off=0):
        bv = doc["bufferViews"][bv_i]
        off = bv.get("byteOffset", 0) + extra_off
        return np.frombuffer(binbuf, dt, count * ncomp, off).reshape(count, ncomp)

    def acc(i):
        a = doc["accessors"][i]
        if "bufferView" not in a:  # sparse accessor (morph targets): zeros + listed values
            n = NCOMP[a["type"]]
            out = np.zeros((a["count"], n), COMP[a["componentType"]])
            sp = a.get("sparse")
            if sp:
                ix = view(sp["indices"]["bufferView"], COMP[sp["indices"]["componentType"]], sp["count"], 1, sp["indices"].get("byteOffset", 0))[:, 0]
                vals = view(sp["values"]["bufferView"], COMP[a["componentType"]], sp["count"], n, sp["values"].get("byteOffset", 0))
                out[ix] = vals
            return out
        bv = doc["bufferViews"][a["bufferView"]]
        dt = COMP[a["componentType"]]
        n = NCOMP[a["type"]]
        off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        stride = bv.get("byteStride", 0)
        item = np.dtype(dt).itemsize * n
        if stride and stride != item:
            rows = [np.frombuffer(binbuf, dt, n, off + k * stride) for k in range(a["count"])]
            return np.array(rows)
        return np.frombuffer(binbuf, dt, a["count"] * n, off).reshape(a["count"], n)

    meshes = {}
    for node in doc["nodes"]:
        if "mesh" not in node:
            continue
        for prim in doc["meshes"][node["mesh"]]["primitives"]:
            at = prim["attributes"]
            d = {"pos": acc(at["POSITION"]).astype(float), "nrm": acc(at["NORMAL"]).astype(float)}
            if "WEIGHTS_0" in at:
                d["joints"] = acc(at["JOINTS_0"]).astype(int)
                d["weights"] = acc(at["WEIGHTS_0"]).astype(float)
            if prim.get("targets"):
                d["morph"] = [acc(t["POSITION"]).astype(float) for t in prim["targets"]]
            d["extras"] = node.get("extras", {})
            d["skin"] = node.get("skin")
            meshes.setdefault(node["name"], []).append(d)
    return doc, meshes


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def stretch_w(doc, d):
    """Weight of the joint named 'stretch' per vertex."""
    skin = doc["skins"][d["skin"]]
    names = [doc["nodes"][j]["name"] for j in skin["joints"]]
    k = names.index("stretch")
    return (d["weights"] * (d["joints"] == k)).sum(axis=1)


def bbox(meshes, names=None):
    ps = np.concatenate([d["pos"] for n, ds in meshes.items() if names is None or n in names for d in ds])
    return ps.min(axis=0), ps.max(axis=0)


for f in sorted(os.listdir(OUT)):
    if not f.endswith(".glb"):
        continue
    print(f)
    doc, meshes = load(f)
    for n, ds in meshes.items():
        for d in ds:
            check(np.isfinite(d["pos"]).all(), f"{n}: finite positions ({len(d['pos'])} verts)")
            ln = np.linalg.norm(d["nrm"], axis=1)
            check(np.abs(ln - 1).max() < 1e-3, f"{n}: unit normals")
            check("engine_sim" in d["extras"], f"{n}: extras.engine_sim present")

    if f == "piston.glb":
        lo, hi = bbox(meshes, ["piston"])
        check(abs((hi[0] - lo[0]) - 0.086) < 0.6e-3, f"piston diameter {1e3*(hi[0]-lo[0]):.2f} mm ~ bore 86")
        check(abs(hi[1] - 0.35 * 0.086 - 0.9e-3) < 0.3e-3, f"crown {1e3*hi[1]:.2f} mm over the pin (0.35 bore + dome)")
    if f == "conrod.glb":
        doc_, ms = doc, meshes
        d = ms["rod_body"][0]
        w = stretch_w(doc, d)
        top = d["pos"][:, 1].max()
        check(abs(top - (0.143 + 0.0095 + 0.0014 + 0.005)) < 0.5e-3, f"small eye top {1e3*top:.2f} mm")
        # stretch to rod 0.160 at bore 0.086: the small-end top moves by exactly +17 mm
        p2 = d["pos"] + (w * (0.160 - 0.143))[:, None] * np.array([0, 1, 0])
        check(abs(p2[:, 1].max() - top - 0.017) < 1e-6, "rod stretch moves the small end by rod - rod_ref")
        check(np.allclose(w[d["pos"][:, 1] < 0.02], 0), "big end has weight 0")
    if f == "crank_throw.glb":
        d = meshes["crank_pin"][0]
        check(np.allclose(stretch_w(doc, d), 1), "crankpin weight 1")
        cz = 0.5 * (d["pos"][:, 1].max() + d["pos"][:, 1].min())  # bbox centre (the oil hole skews a mean)
        check(abs(cz - 0.043) < 1e-3, f"crankpin centre at throw radius ({1e3*cz:.2f} mm)")
        m = meshes["crank_main_journals"][0]
        check(np.allclose(stretch_w(doc, m), 0), "main journals weight 0")
        lo, hi = bbox(meshes)
        check(abs((hi[0] - lo[0]) - 1.2 * 0.086) < 1e-4, f"module length {1e3*(hi[0]-lo[0]):.2f} mm = pitch 103.2")
    if f == "wankel_rotor.glb":
        p = meshes["rotor"][0]["pos"]
        r = np.hypot(p[:, 0], p[:, 2]).max()
        check(r < 0.105, f"rotor body inside R ({1e3*r:.2f} mm; the apex is cut by the seal slot)")
        sp = meshes["rotor_seals"][0]["pos"]
        rs = np.hypot(sp[:, 0], sp[:, 2]).max()
        check(abs(rs - 0.105) < 0.1e-3, f"apex seals reach R ({1e3*rs:.2f} mm)")
        ang = np.degrees(np.arctan2(p[:, 2], p[:, 0]))
        a0 = ang[np.argmax(np.hypot(p[:, 0], p[:, 2]))]
        check(min(abs(a0 - x) for x in (0, 120, -120)) < 3, f"an apex on 0/±120° ({a0:.1f}°)")
        check(abs(p[:, 1].max() - 0.04) < 0.5e-3 and abs(p[:, 1].min() + 0.04) < 0.5e-3, "rotor centred in y, width 80")
    if f == "wankel_housing.glb":
        h = meshes["trochoid_liner"][0]["pos"]
        rr = np.hypot(h[:, 0], h[:, 2])
        check(abs(rr.min() - 0.090) < 0.5e-3, f"trochoid minor radius {1e3*rr.min():.2f} mm = R - e")
        check(abs(rr.max() - 0.1208) < 0.6e-3, f"trochoid major radius {1e3*rr.max():.2f} mm = R + e (+0.8 liner)")
        pl = meshes["spark_plugs_metal"][0]["pos"]
        check(pl[:, 2].min() > 0.08, "plugs on the waist at +z (the 2D view's +y)")

    if f == "valve.glb":
        B = 0.086
        d = meshes["valve"][0]
        lo, hi = d["pos"].min(axis=0), d["pos"].max(axis=0)
        check(abs(hi[0] - 0.17 * B) < 0.1e-3, f"valve head radius {1e3*hi[0]:.2f} mm = 0.17 bore (the scene's disc)")
        check(abs(lo[1]) < 1e-5 and abs(hi[1] - 1.15 * B) < 1e-5, f"face at 0, tip at {1e3*hi[1]:.2f} mm")
        w = stretch_w(doc, d)
        check(np.allclose(w[d["pos"][:, 1] < 0.015], 0), "valve head weight 0")
        rt = meshes["retainer"][0]
        check(np.allclose(stretch_w(doc, rt), 1), "retainer moves with the tip (weight 1)")
    if f == "valve_spring.glb":
        p = meshes["valve_spring"][0]["pos"]
        check(abs(p[:, 1].min()) < 1e-5 and abs(p[:, 1].max() - 0.44 * 0.086) < 1e-5, f"spring ground flat to 0..{1e3*p[:,1].max():.2f} mm")
    if f == "cam_lobe.glb":
        d = meshes["cam_lobe"][0]
        rb, L = 0.22 * 0.086, 0.11 * 0.086
        nose = d["pos"][:, 1].max()
        check(abs(nose - (rb + L)) < 0.15e-3, f"nose {1e3*nose:.2f} mm = base {1e3*rb:.2f} + lift {1e3*L:.2f}")
        check("morph" in d, "lift morph target present")
        if "morph" in d:
            for wgt in (-0.3, 0.2):
                p2 = d["pos"] + wgt * d["morph"][0]
                n2 = p2[:, 1].max()
                check(abs(n2 - (rb + L * (1 + wgt))) < 0.15e-3, f"morph weight {wgt:+.1f} -> nose {1e3*n2:.2f} mm (lift x{1+wgt:.1f})")
            base = d["pos"][:, 1].min()
            check(abs(-base - rb) < 0.15e-3, f"base circle untouched by the morph ({1e3*-base:.2f} mm)")

print("\nFAILURES:" if fails else "\nall checks passed", *fails, sep="\n  ")
sys.exit(1 if fails else 0)
