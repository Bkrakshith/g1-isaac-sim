"""Isaac twin of hospital-sim-env/objects/check_objects.py's settle test: the instruments standing in the tool rack (no robot).

    ./g1-isaac-sim/run.sh --python g1-isaac-sim/tools/rack_settle_test.py [--duration 3]

Reports tilt (deg), xy drift and z change (mm) per instrument at a few times; PASS if every instrument stays within
12 deg / 15 mm like the MuJoCo check. Writes g1-isaac-sim/tools/results/rack_settle.json.
"""
import argparse
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.dirname(HERE)
REPO = os.path.dirname(ISAAC)
ap = argparse.ArgumentParser()
ap.add_argument("--duration", type=float, default=3.0)
ap.add_argument("--no-ccd", action="store_true")
ap.add_argument("--scan", default=None, help="instrument whose actual extent is mapped with a ray grid (diagnostic)")
ap.add_argument("--probe", default=None, help="t=0: horizontal rays through this instrument at several heights")
ap.add_argument("--scan-rack", action="store_true", help="map the guide-plate holes with rays, no instruments")
ap.add_argument("--no-paxes", action="store_true", help="identity principal axes (diagnostic)")
ap.add_argument("--iters", type=int, default=0, help="override instrument solver position iterations")
ap.add_argument("--offset", type=float, default=0.0, help="override contact offset of every theatre collider")
ap.add_argument("--render", default=None, help="save a side-view PNG of the rack at the end to this path")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "disable_viewport_updates": not args.render})
import carb  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(ISAAC, "usd", "flat_theatre_robot.usd"))
while ctx.get_stage_loading_status()[2] > 0 or not ctx.get_stage():
    app.update()
stage = ctx.get_stage()
with Usd.EditContext(stage, stage.GetSessionLayer()):
    stage.GetPrimAtPath("/World/G1").SetActive(False)
    if args.scan_rack:
        for n_ in json.load(open(os.path.join(REPO, "hospital-sim-env/objects/catalog.json")))["labels"]:
            stage.GetPrimAtPath(f"/World/Theatre/{n_}").SetActive(False)
    for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Theatre")):
        if args.no_paxes and p.HasAttribute("physics:principalAxes"):
            from pxr import Gf
            p.GetAttribute("physics:principalAxes").Set(Gf.Quatf(1, 0, 0, 0))
        if args.no_ccd and p.HasAttribute("physxRigidBody:enableCCD"):
            p.GetAttribute("physxRigidBody:enableCCD").Set(False)
        if args.iters and p.HasAttribute("physxRigidBody:solverPositionIterationCount"):
            p.GetAttribute("physxRigidBody:solverPositionIterationCount").Set(args.iters)
        if args.offset and p.HasAttribute("physxCollision:contactOffset"):
            p.GetAttribute("physxCollision:contactOffset").Set(args.offset)
cat = json.load(open(os.path.join(REPO, "hospital-sim-env/objects/catalog.json")))
names = cat["labels"]
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

DT = 0.002
SimulationManager.set_physics_sim_device("cpu")
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Scene):
        SimulationManager.set_default_physics_scene(str(p.GetPath()))
        break
SimulationManager.set_physics_dt(DT)
carb.settings.get_settings().set_bool("/physics/updateToUsd", False)
omni.timeline.get_timeline_interface().play()
app.update()
carb.settings.get_settings().set_bool("/app/player/playSimulations", False)
SimulationManager.initialize_physics()
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.core.simulation_manager", False)
import omni.physics.core  # noqa: E402
import omni.physics.tensors  # noqa: E402

phys = omni.physics.core.get_physics_simulation_interface()
view = omni.physics.tensors.create_simulation_view("numpy", stage_id=ctx.get_stage_id())
view.set_subspace_roots("/")
if args.scan_rack:
    import omni.physx

    sq = omni.physx.get_physx_scene_query_interface()
    rk = cat["world_poses"]["tool_rack"]
    yaw = 2 * math.atan2(rk["quat"][3], rk["quat"][0])
    for n in names:
        sl = cat["rack"]["slots"][n]
        row = []
        for axis in (0, 1):
            xs = []
            for u in np.arange(-0.06, 0.0601, 0.001):
                x, y = (u, sl["y"]) if axis == 0 else (0.0, sl["y"] + u)
                wx = rk["pos"][0] + math.cos(yaw) * x - math.sin(yaw) * y
                wy = rk["pos"][1] + math.sin(yaw) * x + math.cos(yaw) * y
                h = sq.raycast_closest((wx, wy, 1.5), (0, 0, -1), 2.0)
                z = 1.5 - h["distance"] if h["hit"] else 0
                if z < 0.83:
                    xs.append(u)
            row.append((round(min(xs) * 1000, 1), round(max(xs) * 1000, 1)) if xs else None)
        print(f"[rack] hole {n:16s} rack-x open {row[0]} mm (expect +-{sl['hole_half'][0]*1000:.1f}), "
              f"rack-y open {row[1]} mm around slot (expect +-{sl['hole_half'][1]*1000:.1f})", flush=True)
    app.close()
    raise SystemExit
rb = view.create_rigid_body_view([f"/World/Theatre/{n}" for n in names])


def tilt(q):  # x y z w
    x, y = q[0], q[1]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


X0 = np.asarray(rb.get_transforms()).reshape(len(names), 7).copy()
if args.probe:
    import omni.physx

    sq0 = omni.physx.get_physx_scene_query_interface()
    o = X0[names.index(args.probe), :3]
    L = cat["instruments"][args.probe]["size_xyz_m"][2]
    for zz in np.arange(0.005, L, 0.01):
        hits = []
        for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)):
            start = (float(o[0] - 0.2 * d[0]), float(o[1] - 0.2 * d[1]), float(o[2] + zz))
            h = sq0.raycast_closest(start, d, 0.4)
            hits.append(h["collision"].rsplit("/", 1)[-1] + f"@{(h['distance'] - 0.2) * 1000:.0f}" if h["hit"] and args.probe in h["collision"] else "-")
        print(f"[rack] probe z={zz * 1000:5.0f} mm: " + "  ".join(hits), flush=True)
# raycast probe of the rack colliders (rack frame -> world): guide plate between slots, a guide hole, tip plate
import omni.physx  # noqa: E402

sq = omni.physx.get_physx_scene_query_interface()
rk = cat["world_poses"]["tool_rack"]
rq = rk["quat"]
yaw = 2 * math.atan2(rq[3], rq[0])
slots = cat["rack"]["slots"]


def to_world(x, y):
    return (rk["pos"][0] + math.cos(yaw) * x - math.sin(yaw) * y, rk["pos"][1] + math.sin(yaw) * x + math.cos(yaw) * y)


for label, (x, y) in {"guide between slot 2/3": (0.0, (slots["angled_tweezers"]["y"] + slots["scissors"]["y"]) / 2),
                      "front bar at scissors": (-0.045, slots["scissors"]["y"]),
                      "scissors hole centre": (0.0, slots["scissors"]["y"] + 0.03 * 0),
                      "rack end (no plate)": (0.0, 0.40)}.items():
    wx, wy = to_world(x, y)
    hit = sq.raycast_closest((wx, wy, 1.5), (0, 0, -1), 2.0)
    print(f"[rack] ray {label:24s} world ({wx:.3f},{wy:.3f}) -> hit={hit['hit']} z={1.5 - hit['distance'] if hit['hit'] else None} "
          f"{hit.get('collision', '') if hit['hit'] else ''}", flush=True)
log = {}
t = 0.0
for k in range(int(args.duration / DT) + 1):
    if k in (0, 5, 25, 50, 250, int(args.duration / DT)):
        X = np.asarray(rb.get_transforms()).reshape(len(names), 7)
        log[round(t, 3)] = {n: dict(tilt_deg=round(tilt(X[i, 3:7]), 2),
                                    xy_drift_mm=round(float(np.linalg.norm(X[i, :2] - X0[i, :2])) * 1000, 2),
                                    dz_mm=round(float(X[i, 2] - X0[i, 2]) * 1000, 2)) for i, n in enumerate(names)}
    phys.simulate(DT, t)
    t += DT
if args.render:
    import omni.replicator.core as rep
    from pxr import Gf, UsdGeom

    cam = UsdGeom.Camera.Define(stage, "/World/RackCam")
    cam.CreateFocalLengthAttr(24.0)
    rk = np.asarray(cat["world_poses"]["tool_rack"]["pos"])
    eye = Gf.Vec3d(rk[0] + 0.05, rk[1] + 0.45, rk[2] + 0.12)
    tgt = Gf.Vec3d(rk[0], rk[1], rk[2] + 0.07)
    f = np.asarray(tgt - eye)
    f /= np.linalg.norm(f)
    xa = np.cross(f, [0, 0, 1.0])
    xa /= np.linalg.norm(xa)
    ya = np.cross(xa, f)
    m = Gf.Matrix4d(*xa, 0, *ya, 0, *(-f), 0, *eye, 1)  # rows: camera x, y, z (= -forward), position
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    # copy the simulated poses to USD for rendering
    X = np.asarray(rb.get_transforms()).reshape(len(names), 7)
    for i, n in enumerate(names):
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Theatre/{n}"))
        ops = xf.GetOrderedXformOps()
        ops[0].Set(Gf.Vec3d(*[float(v) for v in X[i, :3]]))
        ops[1].Set(Gf.Quatd(float(X[i, 6]), float(X[i, 3]), float(X[i, 4]), float(X[i, 5])))
    rp = rep.create.render_product("/World/RackCam", (1280, 720))
    ann = rep.AnnotatorRegistry.get_annotator("rgb")
    ann.attach([rp])
    for _ in range(30):
        app.update()
    from PIL import Image
    Image.fromarray(np.asarray(ann.get_data())[..., :3]).save(args.render)
    print("[rack] rendered", args.render, flush=True)
X = np.asarray(rb.get_transforms()).reshape(len(names), 7)
for i, n in enumerate(names):
    qx, qy, qz, qw = X[i, 3:7]
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                  [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    top = X[i, :3] + R @ np.array([0, 0, cat["instruments"][n]["size_xyz_m"][2] - 0.004])
    hit = sq.raycast_closest((float(top[0]), float(top[1]), 1.5), (0, 0, -1), 2.0)
    print(f"[rack] {n:16s} origin {np.round(X[i, :3], 4)} predicted top {np.round(top, 4)} ray hit z "
          f"{round(1.5 - hit['distance'], 4) if hit['hit'] else None} {hit.get('collision', '')}", flush=True)
if args.scan:
    i = names.index(args.scan)
    o = X[i, :3]
    best = None
    for dx in np.arange(-0.09, 0.0901, 0.004):
        for dy in np.arange(-0.09, 0.0901, 0.004):
            h = sq.raycast_closest((float(o[0] + dx), float(o[1] + dy), 1.5), (0, 0, -1), 2.0)
            if h["hit"] and args.scan in h["collision"]:
                z = 1.5 - h["distance"]
                if best is None or z > best[0]:
                    best = (z, dx, dy)
    print(f"[rack] scan {args.scan}: highest point z={best[0]:.4f} at dx={best[1]*1000:.0f} dy={best[2]*1000:.0f} mm "
          f"from the reported origin (upright top would be z={o[2] + cat['instruments'][args.scan]['size_xyz_m'][2]:.4f})"
          if best else f"[rack] scan {args.scan}: no hit", flush=True)
final = log[max(log)]
ok = all(v["tilt_deg"] < 12 and v["xy_drift_mm"] < 15 for v in final.values())
for tt, d in log.items():
    print(f"[rack] t={tt:5.2f} " + "  ".join(f"{n[:8]} {v['tilt_deg']:4.1f}deg" for n, v in d.items()), flush=True)
print("[rack]", "PASS" if ok else "FAIL", json.dumps(final), flush=True)
os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
json.dump({"log": log, "pass": ok}, open(os.path.join(HERE, "results", "rack_settle.json"), "w"), indent=1)
omni.timeline.get_timeline_interface().stop()
app.close()
