"""Re-render the G1 head camera from a RECORDED walk (no new physics): for every sample the robot is put at the
recorded pelvis pose (/sim/ground_truth) with the recorded joint angles (/joint_states), and the head camera
(same "video view" settings as run_sim.py --camera-pitch/--camera-vfov) is rendered with RTX.

    ./g1-isaac-sim/run.sh --python g1-isaac-sim/tools/replay_camera.py --traj traj.npz --out frames_dir \
        [--res 960x540] [--pitch 12] [--vfov 50] [--start 0] [--every 1]
traj.npz (make with tools/extract_traj.py from the run's rosbag): t [N] sim s, root [N,7] x y z qx qy qz qw (world),
names [J], q [N,J]. Frames: <out>/<sim_ns>.jpg + index.csv (sim_s, wall_s, file) -- same layout as
ros2_ws/src/hospital-sim-ros2/g1_navigation/scripts/camera_frame_saver.py. Resumable (existing frames are skipped).
Needs ~5 GB free VRAM; never run it next to another Isaac instance.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.dirname(HERE)
sys.path.insert(0, ISAAC)
import g1_config as C  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--traj", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--res", default="960x540")
ap.add_argument("--pitch", type=float, default=12.0)
ap.add_argument("--vfov", type=float, default=50.0)
ap.add_argument("--start", type=int, default=0)
ap.add_argument("--every", type=int, default=1)
ap.add_argument("--updates", type=int, default=2, help="app.update() per frame (DLSS history)")
args, _ = ap.parse_known_args()
W, H = (int(v) for v in args.res.split("x"))

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "renderer": "RaytracedLighting", "anti_aliasing": 3,
                     "disable_viewport_updates": True})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(ISAAC, "usd", "hospital_g1_robot.usd"))
while ctx.get_stage_loading_status()[2] > 0 or not ctx.get_stage():
    app.update()
stage = ctx.get_stage()
G1 = "/World/G1"
root_path = next(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(G1))
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI))
d435 = next((p for p in Usd.PrimRange(stage.GetPrimAtPath(G1)) if p.GetName() == "d435_link"), None)
with Usd.EditContext(stage, stage.GetSessionLayer()):
    for _p in Usd.PrimRange(stage.GetPrimAtPath(G1)):  # no robot self-contacts needed for a kinematic replay
        if _p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(_p).CreateCollisionEnabledAttr(False)
    cam_path = str(d435.GetPath()) + "/replay_camera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    vf = math.radians(args.vfov)
    fy = H / (2 * math.tan(vf / 2))
    focal = 1.93
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(focal * W / fy)
    cam.CreateVerticalApertureAttr(focal * H / fy)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
    cx = UsdGeom.Xformable(cam)
    cx.AddRotateYOp(opSuffix="videoPitch").Set(args.pitch - math.degrees(0.8307767239493009))
    cx.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, -90))

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
fabric = __import__("omni.physxfabric", fromlist=["x"]).get_physx_fabric_interface()
art.set_dof_stiffnesses(np.full((1, N), 0.0, np.float32), idx)
art.set_dof_dampings(np.full((1, N), 0.0, np.float32), idx)

tr = np.load(args.traj, allow_pickle=True)
T, ROOT, JN, Q = tr["t"], tr["root"], list(tr["names"]), tr["q"]
col = [(names.index(n), k) for k, n in enumerate(JN) if n in names]
print(f"[replay] {len(T)} samples, {len(col)} joints mapped, articulation {root_path}", flush=True)
os.makedirs(args.out, exist_ok=True)
idxf = open(os.path.join(args.out, "index.csv"), "a")
rp = rep.create.render_product(cam_path, (W, H))
annot = rep.AnnotatorRegistry.get_annotator("rgb")
annot.attach([rp])
q0 = np.zeros((1, N), np.float32)
t_sim = 0.0
w0 = time.time()
done = 0
for i in range(args.start, len(T), args.every):
    ns = int(round(T[i] * 1e9))
    fname = f"{ns // 1_000_000_000:06d}{ns % 1_000_000_000:09d}.jpg"
    if os.path.exists(os.path.join(args.out, fname)):
        continue
    q = q0.copy()
    for di, k in col:
        q[0, di] = Q[i, k]
    art.set_root_transforms(ROOT[i:i + 1].astype(np.float32), idx)
    art.set_root_velocities(np.zeros((1, 6), np.float32), idx)
    art.set_dof_positions(q, idx)
    art.set_dof_velocities(np.zeros((1, N), np.float32), idx)
    SimulationManager._physics_sim_interface.simulate(1e-5, t_sim)
    t_sim += 1e-5
    art.set_root_transforms(ROOT[i:i + 1].astype(np.float32), idx)
    art.set_dof_positions(q, idx)
    fabric.update(t_sim, 1e-5)
    if done == 0:
        for _ in range(60):  # renderer warm-up (shaders, texture streaming)
            app.update()
    for _ in range(args.updates):
        app.update()
    img = np.asarray(annot.get_data())
    k = 0
    while img.size == 0 and k < 100:
        app.update()
        img = np.asarray(annot.get_data())
        k += 1
    img = img[..., :3]
    Image.fromarray(img).save(os.path.join(args.out, fname), quality=92)
    idxf.write(f"{T[i]:.3f},{time.time():.3f},{fname}\n")
    done += 1
    if done % 100 == 0:
        idxf.flush()
        el = time.time() - w0
        print(f"[replay] {i}/{len(T)} sim {T[i]:.1f}s  {done / el:.2f} frames/s", flush=True)
idxf.close()
timeline.stop()
app.close()
