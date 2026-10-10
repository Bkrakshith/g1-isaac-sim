"""Author the Isaac physics layer and the robot root stage (pure USD, no Isaac needed):

    uv run --no-project --with usd-core python g1-isaac-sim/make_stage.py

Writes
  g1-isaac-sim/usd/physics_colliders.usd  PhysicsScene (500 Hz) + static box colliders for the wing:
                                       floor slab, walls from hospital-sim-env/isaac/scene/layout.py (doorways left open),
                                       and oriented boxes around furniture that has no collider of its own.
  g1-isaac-sim/usd/hospital_g1_robot.usd  root stage = physics_colliders.usd + ../../hospital-sim-env/isaac/usd/hospital_g1.usd
                                       (sublayers, untouched) + /World/G1 (reference to g1_29dof_brainco.usd)
                                       at the spawn pose (g1_config.spawn_world()).
  g1-isaac-sim/usd/physics_colliders.json summary (counts, skipped items).

Nothing under hospital-sim-env/isaac/ (scene/, usd/, assets/) is modified: everything is new layers that sublayer/reference them.
"""

from __future__ import annotations

import json
import math
import os
import sys

from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics

try:
    from pxr import PhysxSchema
except ImportError:  # usd-core: raw attributes are authored instead
    PhysxSchema = None

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "hospital-sim-env", "isaac", "scene"))
import g1_config  # noqa: E402
import layout  # noqa: E402

USD_DIR = os.path.join(HERE, "usd")
HOSPITAL = os.path.join(REPO, "hospital-sim-env", "isaac", "usd", "hospital_g1.usd")
PHYS = os.path.join(USD_DIR, "physics_colliders.usd")
ROOT = os.path.join(USD_DIR, "hospital_g1_robot.usd")
ROBOT = os.path.join(USD_DIR, "g1_29dof_brainco.usd")

WALL_H = 3.0
WALL_T = 0.20  # collider thickness centred on the grid line (kit walls are 0.1 m inside the cells)
PHYSICS_HZ = 500
HOSPITAL_FOOTPRINT = (-49.7, -4.7, 27.1, 37.2)  # world x0, y0, x1, y1 of /World/Hospital (usd-core bbox)

OX, OY, _ = layout.WING_ORIGIN


def wing_to_world(x, y):
    return x + OX, y + OY


def add_box(stage, path, center, size, rot=None):
    """Invisible static box collider (UsdGeom.Cube, purpose=guide)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
    cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*center))
    if rot is not None:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(rot)
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def subtract_openings(a, b, openings):
    """Segment [a, b] (scalars along one axis) minus a list of (lo, hi) intervals."""
    segs = [(a, b)]
    for lo, hi in openings:
        out = []
        for s0, s1 in segs:
            if hi <= s0 or lo >= s1:
                out.append((s0, s1))
                continue
            if lo > s0:
                out.append((s0, lo))
            if hi < s1:
                out.append((hi, s1))
        segs = out
    return [(s0, s1) for s0, s1 in segs if s1 - s0 > 0.05]


def wall_segments():
    """Unique wall segments (wing-local) from room rectangles, minus doorways/openings."""
    rooms = dict(layout.KIT_ROOMS)
    rooms.update(layout.SUITE_ROOMS)
    edges = set()
    for x0, y0, x1, y1 in rooms.values():
        edges |= {("h", y0, x0, x1), ("h", y1, x0, x1), ("v", x0, y0, y1), ("v", x1, y0, y1)}
    openings = list(layout.DOORWAYS.values()) + list(layout.OPENINGS)
    segs = []
    # merge colinear, overlapping edges first
    for orient in ("h", "v"):
        by_line = {}
        for o, c, a, b in edges:
            if o == orient:
                by_line.setdefault(c, []).append((min(a, b), max(a, b)))
        for c, ivs in by_line.items():
            ivs.sort()
            merged = []
            for a, b in ivs:
                if merged and a <= merged[-1][1] + 1e-6:
                    merged[-1] = (merged[-1][0], max(merged[-1][1], b))
                else:
                    merged.append((a, b))
            ops = []
            for x0, y0, x1, y1 in openings:
                if orient == "h" and abs(y0 - c) < 1e-6 and abs(y1 - c) < 1e-6:
                    ops.append((min(x0, x1), max(x0, x1)))
                if orient == "v" and abs(x0 - c) < 1e-6 and abs(x1 - c) < 1e-6:
                    ops.append((min(y0, y1), max(y0, y1)))
            for a, b in merged:
                for s0, s1 in subtract_openings(a, b, ops):
                    segs.append((orient, c, s0, s1))
    return segs


def has_collider(prim):
    for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            return True
    return False


def furniture_boxes(stage):
    """Oriented bounding boxes for furniture roots without their own colliders."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    roots = []
    for room in stage.GetPrimAtPath("/World/Wing/Dressing").GetChildren():
        roots += list(room.GetChildren())
    roots += list(stage.GetPrimAtPath("/World/Wing/ORSuite/Props").GetChildren())
    boxes, skipped = [], {}
    for p in roots:
        if not p.IsActive() or UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        if has_collider(p):
            skipped[str(p.GetPath())] = "own collider"
            continue
        bb = cache.ComputeWorldBound(p)
        rng = bb.GetRange()
        if rng.IsEmpty():
            continue
        m = bb.GetMatrix()
        # world-space corners of the oriented box
        corners = [m.Transform(rng.GetCorner(i)) for i in range(8)]
        zmin = min(c[2] for c in corners)
        zmax = max(c[2] for c in corners)
        sx = (m.TransformDir(Gf.Vec3d(rng.GetSize()[0], 0, 0))).GetLength()
        sy = (m.TransformDir(Gf.Vec3d(0, rng.GetSize()[1], 0))).GetLength()
        sz = (m.TransformDir(Gf.Vec3d(0, 0, rng.GetSize()[2]))).GetLength()
        name = p.GetName()
        if zmin > 1.4:
            skipped[str(p.GetPath())] = f"overhead (z>{zmin:.2f})"
            continue
        if zmin > 1.0 and zmax > 2.4:
            skipped[str(p.GetPath())] = "ceiling-hung"
            continue
        dims = sorted([sx, sy, sz])
        if dims[-1] < 0.3 or (dims[-1] < 0.5 and dims[-2] < 0.2):
            skipped[str(p.GetPath())] = "small"
            continue
        center = m.Transform(rng.GetMidpoint())
        # orientation: rotation part of m (strip scale)
        rot = m.ExtractRotationQuat() if hasattr(m, "ExtractRotationQuat") else Gf.Matrix4d(m).ExtractRotation().GetQuat()
        rot = Gf.Quatd(rot.GetReal(), *rot.GetImaginary())
        # keep boxes upright: only yaw is kept (furniture stands on the floor)
        w, (x, y, z) = rot.GetReal(), rot.GetImaginary()
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        if abs(roll) > 0.05 or abs(pitch) > 0.05:
            # tilted asset frame (e.g. authored Y-up): fall back to the world-aligned box
            ab = bb.ComputeAlignedRange()
            center = ab.GetMidpoint()
            sx, sy, sz = ab.GetSize()
            yaw = 0.0
        q = Gf.Quatd(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        boxes.append((name, str(p.GetPath()), tuple(center), (sx, sy, sz), q))
    return boxes, skipped


def build_physics_layer(hospital_stage):
    layer = Sdf.Layer.CreateNew(PHYS) if not os.path.exists(PHYS) else Sdf.Layer.FindOrOpen(PHYS)
    layer.Clear()
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.OverridePrim("/World")

    scene = UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    sp = scene.GetPrim()
    if PhysxSchema is not None:
        ps = PhysxSchema.PhysxSceneAPI.Apply(sp)
        ps.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
        ps.CreateEnableGPUDynamicsAttr(False)
        ps.CreateBroadphaseTypeAttr("MBP")
        ps.CreateSolverTypeAttr("TGS")
        ps.CreateEnableStabilizationAttr(False)
    else:  # usd-core has no PhysxSchema: author the raw attributes (same names)
        sp.AddAppliedSchema("PhysxSceneAPI")
        sp.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(PHYSICS_HZ)
        sp.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(False)
        sp.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set("MBP")
        sp.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("TGS")
        sp.CreateAttribute("physxScene:enableStabilization", Sdf.ValueTypeNames.Bool).Set(False)

    root = "/World/HG1_Colliders"
    UsdGeom.Scope.Define(st, root)
    summary = {"physics_hz": PHYSICS_HZ, "floors": [], "walls": 0, "furniture": [], "skipped": {}}

    # Floor slabs (top at z = 0): the whole wing (wing-local x -12..14, y -20..0) and the Isaac hospital's
    # footprint -- the hospital's own floor meshes (z = 0.001) carry no colliders, only its walls/furniture do.
    (wx0, wy0), (wx1, wy1) = wing_to_world(-12.2, -20.2), wing_to_world(14.2, 0.0)
    slabs = {"WingFloor": (wx0, wy0, wx1, wy1), "HospitalFloor": HOSPITAL_FOOTPRINT}
    for name, (x0, y0, x1, y1) in slabs.items():
        add_box(st, f"{root}/{name}", ((x0 + x1) / 2, (y0 + y1) / 2, -0.05), (x1 - x0, y1 - y0, 0.1))
        summary["floors"].append(name)

    UsdGeom.Scope.Define(st, f"{root}/Walls")
    for i, (o, c, a, b) in enumerate(wall_segments()):
        if o == "h":
            (cx, cy), size = wing_to_world((a + b) / 2, c), (b - a + WALL_T, WALL_T, WALL_H)
        else:
            (cx, cy), size = wing_to_world(c, (a + b) / 2), (WALL_T, b - a + WALL_T, WALL_H)
        add_box(st, f"{root}/Walls/W{i:03d}", (cx, cy, WALL_H / 2), size)
        summary["walls"] += 1

    UsdGeom.Scope.Define(st, f"{root}/Furniture")
    boxes, skipped = furniture_boxes(hospital_stage)
    used = set()
    for name, src, c, s, q in boxes:
        n = Tf.MakeValidIdentifier(name)
        while n in used:
            n += "_"
        used.add(n)
        add_box(st, f"{root}/Furniture/{n}", c, s, q)
        summary["furniture"].append({"name": n, "source": src, "center": [round(v, 3) for v in c],
                                     "size": [round(v, 3) for v in s]})
    summary["skipped"] = skipped
    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    layer.Save()
    with open(os.path.join(USD_DIR, "physics_colliders.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(f"physics layer: {len(summary['floors'])} floors, {summary['walls']} walls, "
          f"{len(summary['furniture'])} furniture boxes, {len(skipped)} furniture roots skipped")


def build_root_stage():
    layer = Sdf.Layer.CreateNew(ROOT) if not os.path.exists(ROOT) else Sdf.Layer.FindOrOpen(ROOT)
    layer.Clear()
    layer.subLayerPaths.append("./physics_colliders.usd")
    layer.subLayerPaths.append(os.path.relpath(HOSPITAL, USD_DIR))
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    layer.documentation = ("Hospital_G1 + physics colliders + Unitree G1 29-DoF (BrainCo Revo2). "
                           "Generated by g1-isaac-sim/make_stage.py; open with g1-isaac-sim/run.sh.")
    (x, y, z), yaw = g1_config.spawn_world()
    g1 = UsdGeom.Xform.Define(st, "/World/G1")
    g1.GetPrim().GetReferences().AddReference(os.path.relpath(ROBOT, USD_DIR))
    xf = UsdGeom.Xformable(g1)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    layer.Save()
    print(f"root stage: {ROOT}  (G1 at world ({x:.2f}, {y:.2f}, {z:.3f}), yaw {math.degrees(yaw):.0f} deg)")


def build_flat_stage():
    """Flat-ground test stage: physics scene + 40x40 m ground box + dome/distant light + G1 at the origin."""
    path = os.path.join(USD_DIR, "flat_g1_robot.usd")
    layer = Sdf.Layer.CreateNew(path) if not os.path.exists(path) else Sdf.Layer.FindOrOpen(path)
    layer.Clear()
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.Xform.Define(st, "/World")
    scene = UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    sp = scene.GetPrim()
    sp.AddAppliedSchema("PhysxSceneAPI")
    sp.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(PHYSICS_HZ)
    sp.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(False)
    sp.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set("MBP")
    sp.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("TGS")
    g = UsdGeom.Cube.Define(st, "/World/Ground")
    g.CreateSizeAttr(1.0)
    g.CreateDisplayColorAttr([(0.45, 0.47, 0.5)])
    xf = UsdGeom.Xformable(g)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    xf.AddScaleOp().Set(Gf.Vec3d(40, 40, 0.1))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    from pxr import UsdLux
    UsdLux.DomeLight.Define(st, "/World/Dome").CreateIntensityAttr(800)
    UsdLux.DistantLight.Define(st, "/World/Sun").CreateIntensityAttr(2000)
    g1 = UsdGeom.Xform.Define(st, "/World/G1")
    g1.GetPrim().GetReferences().AddReference(os.path.relpath(ROBOT, USD_DIR))
    UsdGeom.Xformable(g1).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, g1_config.PELVIS_HEIGHT))
    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    layer.Save()
    print(f"flat stage: {path}")


if __name__ == "__main__":
    hs = Usd.Stage.Open(HOSPITAL)
    build_physics_layer(hs)
    build_root_stage()
    build_flat_stage()
