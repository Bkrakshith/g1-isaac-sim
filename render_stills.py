"""Render stills of the G1 standing in the Isaac hospital (RTX, headless) to g1-isaac-sim/renders/.

    ./g1-isaac-sim/run.sh --stills [--res 1280x720] [--spp 64]

The robot is settled on the floor at the default pose (stiff implicit joint drives, 1 s of physics), then
rendered from a few cameras at the corridor_west and link spawns. Uses the same stage as run_sim.py.
Needs roughly 5 GB of free VRAM (the hospital is uploaded to the RTX renderer); close other Isaac instances.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g1_config as C  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--res", default="1280x720")
ap.add_argument("--frames", type=int, default=24, help="render updates per still (RTX accumulation)")
ap.add_argument("--out", default=os.path.join(HERE, "renders"))
ap.add_argument("--shots", default="corridor,link", help="comma list: corridor, link")
ap.add_argument("--deactivate", nargs="*", default=[],
                help="prim paths hidden for this render (e.g. /World/Hospital to fit next to another GPU user)")
args, _ = ap.parse_known_args()
W, H = (int(v) for v in args.res.split("x"))

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "renderer": "RaytracedLighting", "anti_aliasing": 3})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(HERE, "usd", "hospital_g1_robot.usd"))
while ctx.get_stage_loading_status()[2] > 0 or not ctx.get_stage():
    app.update()
stage = ctx.get_stage()
with Usd.EditContext(stage, stage.GetSessionLayer()):
    for _p in args.deactivate:
        stage.GetPrimAtPath(_p).SetActive(False)
    for _p in Usd.PrimRange(stage.GetPrimAtPath("/World/G1")):  # finger colliders off, as in run_sim.py
        if _p.HasAPI(UsdPhysics.CollisionAPI) and any(
                k in str(_p.GetPath()) for k in ("_thumb_", "_index_", "_middle_", "_ring_", "_pinky_")):
            UsdPhysics.CollisionAPI(_p).CreateCollisionEnabledAttr(False)
root_path = next(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/G1"))
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI))

from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

settings = carb.settings.get_settings()
SimulationManager.set_physics_sim_device("cpu")
SimulationManager.set_default_physics_scene("/World/PhysicsScene")
SimulationManager.set_physics_dt(0.002)
settings.set_bool("/physics/updateToUsd", False)
SimulationManager.enable_fabric(True)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()
settings.set_bool("/app/player/playSimulations", False)
SimulationManager.initialize_physics()
view = SimulationManager.get_physics_sim_view()
art = view.create_articulation_view(root_path)
names = list(art.shared_metatype.dof_names)
N = art.max_dofs
idx = np.arange(1, dtype=np.int32)
q0 = np.zeros((1, N), np.float32)
default_q, _src = C.load_default_qpos()
for n, v in zip(C.BODY_JOINTS, default_q):
    q0[0, names.index(n)] = v
art.set_dof_stiffnesses(np.full((1, N), 2000.0, np.float32), idx)
art.set_dof_dampings(np.full((1, N), 40.0, np.float32), idx)
fabric = __import__("omni.physxfabric", fromlist=["x"]).get_physx_fabric_interface()


def place(spawn):
    (x, y, z), yaw = C.spawn_world(spawn)
    art.set_root_transforms(np.array([[x, y, z, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]], np.float32), idx)
    art.set_root_velocities(np.zeros((1, 6), np.float32), idx)
    art.set_dof_positions(q0, idx)
    art.set_dof_velocities(np.zeros((1, N), np.float32), idx)
    art.set_dof_position_targets(q0, idx)
    t = 0.0
    for _ in range(500):  # 1 s settle
        SimulationManager._physics_sim_interface.simulate(0.002, t)
        t += 0.002
    fabric.update(t, 0.002)
    return (x, y), yaw, float(np.asarray(art.get_root_transforms())[0, 2])


def look_at(name, eye, target, focal=14.0):
    path = f"/World/StillCams/{name}"
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955 * H / W)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    return path


os.makedirs(args.out, exist_ok=True)
rp = None
annot = rep.AnnotatorRegistry.get_annotator("rgb")


def shoot_new(cam_path, fname):
    """one render product per still (simplest reliable camera switch)."""
    global rp
    if rp is not None:
        annot.detach([rp])
        rp.destroy()
    rp = rep.create.render_product(cam_path, (W, H))
    annot.attach([rp])
    for _ in range(args.frames):
        app.update()
    img = np.asarray(annot.get_data())[..., :3]
    from PIL import Image

    Image.fromarray(img).save(os.path.join(args.out, fname))
    print(f"[stills] wrote {fname} ({img.shape[1]}x{img.shape[0]}, mean {img.mean():.1f})", flush=True)


shots = args.shots.split(",")
# 1-2: corridor_west spawn (facing east down the surgical corridor)
if "corridor" in shots:
    (x, y), yaw, z = place("corridor_west")
    print(f"[stills] corridor_west: pelvis z after settle {z:.3f} m", flush=True)
    f = (math.cos(yaw), math.sin(yaw))
    l = (-f[1], f[0])
    cam = look_at("Front", (x + 2.3 * f[0] + 0.9 * l[0], y + 2.3 * f[1] + 0.9 * l[1], 1.35), (x, y, 0.75), 16.0)
    shoot_new(cam, "g1_corridor_front.png")
    cam = look_at("Behind", (x - 2.2 * f[0] - 1.2 * l[0], y - 2.2 * f[1] - 1.2 * l[1], 1.9),
                  (x + 6 * f[0], y + 6 * f[1], 0.6), 12.0)
    shoot_new(cam, "g1_corridor_behind.png")
# 3: link spawn (just inside the SE door, facing south into the wing), seen from the wing side
if "link" in shots:
    (x, y), yaw, z = place("link")
    print(f"[stills] link: pelvis z after settle {z:.3f} m", flush=True)
    f = (math.cos(yaw), math.sin(yaw))
    cam = look_at("Link", (x + 2.6 * f[0] + 0.35, y + 2.6 * f[1], 1.45), (x, y + 1.0, 0.9), 12.0)
    shoot_new(cam, "g1_link_door.png")

timeline.stop()
app.close()
