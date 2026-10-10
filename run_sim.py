"""Isaac Sim 6.0 backend for the Hospital_G1 teleop stack (alternative to g1_mujoco_sim).

Launch through the wrapper (it sets up the ROS 2 / DDS environment):
    ./g1-isaac-sim/run.sh --headless            # hospital, no window
    ./g1-isaac-sim/run.sh --gui                 # hospital, Isaac window
    ./g1-isaac-sim/run.sh --headless --flat     # flat ground (quick tests)

Interfaces (docs/interfaces.md):
  low level  rt/lowcmd  -> PD torques, rt/lowstate <- state, 500 Hz physics (2 ms). Spoken directly on DDS
             with unitree_sdk2_python + CycloneDDS 0.10.5 (unitree_hg::msg::dds_::LowCmd_/LowState_), the
             same way Unitree's unitree_sim_isaaclab does. ROS 2 nodes using rmw_cyclonedds see them as
             /lowcmd and /lowstate (unitree_hg/msg/LowCmd, LowState). CRC is filled on /lowstate and checked
             (warn-only unless --strict-crc) on /lowcmd.
  ROS 2      /clock, /joint_states (29 body + hand joints), /odom + /tf odom->pelvis (ground truth),
             /livox/lidar (PointCloud2, MID-360-like pattern), /scan (LaserScan), both in frame mid360_link,
             /g1/hand/{left,right}/cmd in (0..1) -> Revo2 finger targets, /g1/hand/{left,right}/state out,
             optional head RGB-D /g1/head_camera/{image_raw,depth,camera_info} (--camera), /sim/objects (--theatre).
             Published with Isaac's bundled rclpy (Humble).
Lidar: CPU/GPU-cheap Warp ray casting against a snapshot of the static collision geometry (walls, floors,
furniture boxes, building colliders); dynamic props and the robot itself are not seen.
"""

from __future__ import annotations

import argparse
import array
import json
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "pydeps"))
import g1_config as C  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--headless", action="store_true", help="no window (default when --gui is not given)")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--flat", action="store_true", help="flat-ground stage instead of the hospital")
ap.add_argument("--stage", default=None, help="USD stage to open (overrides --flat)")
ap.add_argument("--spawn", default=None, help="named spawn from g1-mujoco-sim/worlds/spawns.json, a robot_stances name "
                "from hospital-sim-env/objects/placements.yaml (e.g. theatre_instrument_table), or 'x,y,yaw' (default: spawns.json default)")
ap.add_argument("--pd", choices=["explicit", "implicit"], default="explicit",
                help="explicit: tau = kp(q*-q)+kd(dq*-dq)+tau_ff computed per 2 ms step (like the motor driver and "
                     "the MuJoCo backend); implicit: PhysX joint drives with the /lowcmd gains")
ap.add_argument("--idle", choices=["hold", "damp", "zero"], default="zero",
                help="motors before the first /lowcmd: zero torque (like the MuJoCo sim; the harness holds the robot), "
                     "damp (kd only), or hold the default pose with --idle-gains")
ap.add_argument("--harness", choices=["on", "off"], default="on",
                help="virtual harness on the pelvis (same law/gains as g1_mujoco_sim): holds the spawn pose while "
                     "/g1/mode is DAMP, released 2.5 s after STAND with a 1 s fade; /sim/harness toggles it")
ap.add_argument("--effort-limits", choices=["model", "ultra"], default="ultra",
                help="motor torque limits: model = robot_spec effort_limit (35 Nm ankles/waist roll+pitch), ultra = "
                     "effort_limit_ultra (50 Nm) -- g1_bringup's auto choice for the placeholder policy")
ap.add_argument("--idle-gains", choices=["stand", "ultra"], default="stand",
                help="gains of the idle hold: stiff test-stand PD (default) or the ULTRA policy gains")
ap.add_argument("--hand-collisions", choices=["on", "off"], default="on",
                help="Revo2 finger/palm colliders for grasping, with the hand_physics.py mass/inertia floors, contact "
                     "offsets and 16/2 solver iterations (off = the old behaviour: finger colliders disabled)")
ap.add_argument("--theatre", action="store_true",
                help="add the scrub-nurse theatre objects (usd/theatre_instruments.usd from hospital-sim-env/objects/placements.yaml): "
                     "hospital_g1_theatre.usd, or flat_theatre_robot.usd with --flat; publishes /sim/objects")
ap.add_argument("--fixed-base", action="store_true",
                help="weld the pelvis at the spawn pose (arm-skill development without a balance policy)")
ap.add_argument("--objects-hz", type=float, default=20.0, help="/sim/objects rate (with theatre objects)")
ap.add_argument("--lidar", choices=["warp", "none"], default="warp")
ap.add_argument("--lidar-hz", type=float, default=10.0)
ap.add_argument("--scan-level", action="store_true",
                help="cast /scan in a gravity-levelled plane at the MID-360 (pelvis yaw, no roll/pitch) and publish it "
                     "in frame laser_level with a dynamic TF pelvis->laser_level (like slicing the MID-360 cloud in an "
                     "IMU-levelled frame). Default: the scan plane tilts with the torso (laser_link), so a forward-"
                     "leaning walk puts floor hits ~8-10 m ahead into the scan")
ap.add_argument("--base-footprint", action="store_true",
                help="also publish TF pelvis->base_footprint (the pelvis projected to the floor: x, y, yaw only), the "
                     "planar base frame 2-D SLAM expects (slam_toolbox base_frame:=base_footprint)")
ap.add_argument("--topic-prefix", default="",
                help="publish /clock /tf /tf_static /odom /joint_states /scan /livox/lidar under this prefix (e.g. "
                     "/isaac) for a system-ROS relay (g1-isaac-sim/tools/isaac_relay.py): Nav2's costmaps stalled when fed "
                     "directly by this process's DDS writers")
ap.add_argument("--lidar-device", default="cuda:0", help="warp device for lidar ray casting (cuda:0 or cpu)")
ap.add_argument("--camera", action="store_true",
                help="head RGB-D camera (D435 colour intrinsics, vertical FOV 42 deg): /g1/head_camera/{image_raw,depth,"
                     "camera_info} (costs VRAM + render time)")
ap.add_argument("--camera-res", default="640x360", help="head camera resolution WxH (16:9 keeps the D435 FOV)")
ap.add_argument("--camera-hz", type=float, default=10.0)
ap.add_argument("--camera-depth", choices=["on", "off"], default="on",
                help="also publish /g1/head_camera/depth (off saves render-readback time on long camera runs)")
ap.add_argument("--camera-vfov", type=float, default=42.0,
                help="head camera vertical FOV in degrees (42 = D435 colour; wider for walkthrough videos)")
ap.add_argument("--camera-pitch", type=float, default=None,
                help="VIDEO VIEW: point the head camera this many degrees below the torso's x axis instead of the "
                     "D435 mount (48 deg down). Not the real sensor; for walkthrough videos only")
ap.add_argument("--anti-aliasing", type=int, default=0,
                help="RTX anti-aliasing/denoise (0 off, 1 TAA, 2 FXAA, 3 DLSS, 4 DLAA); 3 removes the sampled-light "
                     "speckle in the hospital camera images")
ap.add_argument("--no-viewport", action="store_true",
                help="headless + camera: keep the default viewport's render updates off, render only the head camera")
ap.add_argument("--no-realtime", action="store_true", help="run as fast as possible instead of wall-clock paced")
ap.add_argument("--duration", type=float, default=0.0, help="stop after this many sim seconds (0 = run forever)")
ap.add_argument("--render-hz", type=float, default=None, help="viewport/render rate (default 30 with --gui, 0 headless)")
ap.add_argument("--strict-crc", action="store_true", help="drop /lowcmd messages with a bad CRC")
ap.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
ap.add_argument("--net-if", default=os.environ.get("HG1_DDS_IF", ""), help="network interface for the Unitree DDS "
                "channel (default: CycloneDDS default / CYCLONEDDS_URI)")
ap.add_argument("--stats", type=float, default=5.0, help="print loop statistics every N wall seconds (0 = off)")
ap.add_argument("--dynamic-props", action="store_true",
                help="keep the Lightwheel props' rigid bodies dynamic (default: frozen, i.e. static colliders)")
ap.add_argument("--broadphase", default=None, help="override physxScene:broadphaseType (MBP, SAP, GPU)")
ap.add_argument("--deactivate", nargs="*", default=[], help="(debug) prim paths to deactivate before physics start")
ap.add_argument("--no-fabric", action="store_true", help="(debug) do not enable PhysX Fabric updates")
ap.add_argument("--keep-simmgr", action="store_true", help="(debug) keep isaacsim.core.simulation_manager enabled")
ap.add_argument("--profile", action="store_true", help="print per-section loop timings with the stats")
ap.add_argument("--log", default=None, help="write a CSV of pelvis height / roll / pitch at 50 Hz to this path")
args, _unknown = ap.parse_known_args()
HEADLESS = not args.gui

# ------------------------------------------------------------------------------------------------ app
from isaacsim import SimulationApp  # noqa: E402

# headless without a camera: viewport updates off, so the hospital is never uploaded to the RTX renderer (VRAM).
app = SimulationApp({"headless": HEADLESS, "width": 1280, "height": 720, "renderer": "RaytracedLighting",
                     "anti_aliasing": args.anti_aliasing, "disable_viewport_updates": HEADLESS and (args.no_viewport or not args.camera)})

import carb  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

ext = omni.kit.app.get_app().get_extension_manager()
# ROS 2: Isaac's bundled Humble rclpy is imported directly (sys.path below; libs via LD_LIBRARY_PATH from run.sh).
# The isaacsim.ros2.bridge extension itself is NOT enabled: with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (what the
# rest of the stack uses) Isaac 6.0 segfaults in omni.graph on the first app.update() after it loads.
_rclpy_dir = os.path.join(os.environ.get("ISAAC_DIR", os.path.expanduser("~/isaacsim")), "exts/isaacsim.ros2.core",
                          os.environ.get("ROS_DISTRO", "humble"), "rclpy")
sys.path.insert(0, _rclpy_dir)

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

if args.theatre:
    _default_stage = "flat_theatre_robot.usd" if args.flat else "hospital_g1_theatre.usd"
else:
    _default_stage = "flat_g1_robot.usd" if args.flat else "hospital_g1_robot.usd"
stage_path = args.stage or os.path.join(HERE, "usd", _default_stage)
print(f"[run_sim] opening {stage_path}", flush=True)
ctx = omni.usd.get_context()
ctx.open_stage(stage_path)
while ctx.get_stage_loading_status()[2] > 0 or not ctx.get_stage():
    app.update()
for _ in range(10):
    app.update()
stage = ctx.get_stage()

# ---- robot prim / articulation root ----------------------------------------------------------------
G1 = "/World/G1"
root_path = None
for p in Usd.PrimRange(stage.GetPrimAtPath(G1)):
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        root_path = str(p.GetPath())
        break
if root_path is None:
    raise SystemExit("[run_sim] no ArticulationRootAPI under /World/G1")

def _spawn(name):
    if name and "," in name:
        x_, y_, yaw_ = (float(v) for v in name.split(","))
        return (x_, y_, C.PELVIS_HEIGHT), yaw_
    if name:
        try:
            import yaml

            st_ = yaml.safe_load(open(os.path.join(REPO, "hospital-sim-env/objects/placements.yaml"))).get("robot_stances", {})
            if name in st_:
                x_, y_, yaw_ = st_[name]
                return (float(x_), float(y_), C.PELVIS_HEIGHT), float(yaw_)
        except Exception as e:  # noqa: BLE001
            print(f"[run_sim] placements.yaml not read: {e}", flush=True)
    return C.spawn_world(name)


if args.flat and not (args.theatre or args.spawn):
    (sx, sy, sz), syaw = (0.0, 0.0, C.PELVIS_HEIGHT), 0.0
elif args.flat and args.theatre and not args.spawn:
    (sx, sy, sz), syaw = _spawn("theatre_instrument_table")
else:
    (sx, sy, sz), syaw = _spawn(args.spawn)
g1x = UsdGeom.Xformable(stage.GetPrimAtPath(G1))
g1x.ClearXformOpOrder()
g1x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(sx, sy, sz))
g1x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(math.cos(syaw / 2), 0, 0, math.sin(syaw / 2)))
print(f"[run_sim] G1 spawn ({sx:.2f}, {sy:.2f}, {sz:.2f}) yaw {math.degrees(syaw):.0f} deg; articulation {root_path}",
      flush=True)

# prim paths of sensor frames (for lidar / camera poses)
def find_prim(name):
    for p in Usd.PrimRange(stage.GetPrimAtPath(G1)):
        if p.GetName() == name:
            return p
    return None


import hand_physics  # noqa: E402

with Usd.EditContext(stage, stage.GetSessionLayer()):
    if args.hand_collisions == "on":
        hand_physics.apply(stage, G1, collisions=True, log=lambda m: print("[run_sim] " + m, flush=True))
    if args.fixed_base:
        _fj = UsdPhysics.FixedJoint.Define(stage, "/World/G1_base_weld")
        _fj.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
        _fj.CreateLocalPos0Attr(Gf.Vec3f(sx, sy, C.HOME_PELVIS_HEIGHT))
        _fj.CreateLocalRot0Attr(Gf.Quatf(math.cos(syaw / 2), 0, 0, math.sin(syaw / 2)))
        g1x.GetOrderedXformOps()[0].Set(Gf.Vec3d(sx, sy, C.HOME_PELVIS_HEIGHT))
        print(f"[run_sim] fixed base: pelvis welded at ({sx:.2f}, {sy:.2f}, {C.HOME_PELVIS_HEIGHT:.3f})", flush=True)
    if args.hand_collisions == "off":
        _n = 0
        for _p in Usd.PrimRange(stage.GetPrimAtPath(G1)):
            if _p.HasAPI(UsdPhysics.CollisionAPI) and any(
                    k in str(_p.GetPath()) for k in ("_thumb_", "_index_", "_middle_", "_ring_", "_pinky_")):
                UsdPhysics.CollisionAPI(_p).CreateCollisionEnabledAttr(False)
                _n += 1
        print(f"[run_sim] finger colliders disabled: {_n}", flush=True)
    for _p in args.deactivate:
        stage.GetPrimAtPath(_p).SetActive(False)

# ---- scene props: freeze third-party rigid bodies (Lightwheel carts/trays) unless asked otherwise --------------
if not args.dynamic_props:
    # The props are USD instances (proxies are read-only), so the instance root is made non-instanceable in the
    # session layer first; then each rigid body is switched to a static collider (rigidBodyEnabled = false).
    roots = set()
    it = iter(Usd.PrimRange(stage.GetPrimAtPath("/World"), Usd.TraverseInstanceProxies()))
    for p in it:
        if str(p.GetPath()).startswith(G1) or str(p.GetPath()).startswith("/World/Theatre"):
            it.PruneChildren()  # the robot and the scrub-nurse objects stay dynamic
            continue
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            q = p
            while q.IsInstanceProxy():
                q = q.GetParent()
            roots.add(q.GetPath())
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for r in roots:
            if stage.GetPrimAtPath(r).IsInstance():
                stage.GetPrimAtPath(r).SetInstanceable(False)
        n_frozen = 0
        for r in roots:
            for p in Usd.PrimRange(stage.GetPrimAtPath(r)):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
                    n_frozen += 1
                if p.IsA(UsdPhysics.Joint):  # joints between now-static bodies would only log errors
                    UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
    print(f"[run_sim] froze {n_frozen} scene rigid bodies under {len(roots)} props (--dynamic-props keeps them dynamic)",
          flush=True)

# ---- physics setup -----------------------------------------------------------------------------------
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

DT = 0.002
SimulationManager.set_physics_sim_device("cpu")
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Scene):
        SimulationManager.set_default_physics_scene(str(p.GetPath()))
        if args.broadphase:
            p.GetAttribute("physxScene:broadphaseType").Set(args.broadphase)
        break
SimulationManager.set_physics_dt(DT)
# Do not write physics results back to USD every 2 ms step (slow for ~70 links); state is read through the tensor
# API and rendering reads transforms from Fabric (updated only on render frames).
settings = carb.settings.get_settings()
settings.set_bool("/physics/updateToUsd", False)
settings.set_bool("/physics/updateVelocitiesToUsd", False)
settings.set_bool("/physics/updateForceSensorsToUsd", False)
settings.set_bool("/physics/outputVelocitiesLocalSpace", False)
try:
    if args.no_fabric:
        raise RuntimeError("disabled by --no-fabric")
    if not (args.gui or args.camera or (args.render_hz or 0) > 0):
        raise RuntimeError("nothing is rendered (headless, no camera): Fabric sync not needed, saves ~0.6 ms/step")
    SimulationManager.enable_fabric(True)
    FABRIC = True
except Exception as e:  # noqa: BLE001
    print(f"[run_sim] PhysX->Fabric sync off: {e}", flush=True)
    FABRIC = False

timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()
settings.set_bool("/app/player/playSimulations", False)  # app.update() only renders; physics stepped below
SimulationManager.initialize_physics()
phys_sim = SimulationManager._physics_sim_interface
fabric_if = None
if FABRIC:
    import omni.physxfabric

    fabric_if = omni.physxfabric.get_physx_fabric_interface()
# isaacsim.core.simulation_manager's native plugin subscribes to every physics step and, on each one, calls an
# omni.physics query that traverses the whole USD stage: ~8-10 ms per 2 ms step with the hospital loaded. Once
# physics is initialised we no longer need it, so it is disabled (with its dependents) and the tensor view is
# created directly.
import omni.physics.core  # noqa: E402
import omni.physics.tensors  # noqa: E402

if not args.keep_simmgr:
    ext.set_extension_enabled_immediate("isaacsim.core.simulation_manager", False)
    print(f"[run_sim] isaacsim.core.simulation_manager disabled after physics init "
          f"(enabled={ext.is_extension_enabled('isaacsim.core.simulation_manager')})", flush=True)
phys_sim = omni.physics.core.get_physics_simulation_interface()
view = omni.physics.tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
view.set_subspace_roots("/")
if view is None:
    raise SystemExit("[run_sim] physics simulation view not available")
art = view.create_articulation_view(root_path)
dof_names = list(art.shared_metatype.dof_names)
link_names = list(art.shared_metatype.link_names)
NDOF = art.max_dofs
print(f"[run_sim] articulation: {NDOF} dofs, {len(link_names)} links", flush=True)
body_idx = np.array([dof_names.index(n) for n in C.BODY_JOINTS])  # contract order -> articulation dof index
hands = {s: C.hand_map(dof_names, s) for s in ("left", "right")}
# actuated hand joints (12, slot order) -> /joint_states and /g1/hand/*/state; all 22 (incl. coupled) are driven
hand_act = [hands[s][d][0][0] for s in ("left", "right") for d in C.HAND_DOFS if d in hands[s]]
hand_all = [(n, r, s, d) for s in ("left", "right") for d in C.HAND_DOFS for n, r in hands[s].get(d, [])]
print(f"[run_sim] hand joints: {len(hand_act)} actuated, {len(hand_all)} driven (incl. coupled/mimic)", flush=True)
lims = np.asarray(art.get_dof_limits()).reshape(NDOF, 2)
all_idx = np.arange(1, dtype=np.int32)

default_q, default_src = C.load_default_qpos()
DEFAULT_Q = np.array(default_q, dtype=np.float32)
KP0 = np.array(C.STAND_KP if args.idle_gains == "stand" else C.DEFAULT_KP, dtype=np.float32)
KD0 = np.array(C.STAND_KD if args.idle_gains == "stand" else C.DEFAULT_KD, dtype=np.float32)
TAU_MAX = np.array(C.TORQUE_LIMIT if args.effort_limits == "model" or not C.SPEC
                   else C.SPEC.get("effort_limit_ultra", C.TORQUE_LIMIT), dtype=np.float32)
print(f"[run_sim] effort limits: {args.effort_limits} (ankle {TAU_MAX[4]:.0f} Nm, waist roll {TAU_MAX[13]:.0f} Nm)", flush=True)
print(f"[run_sim] default pose from {default_src}", flush=True)

# initial state: default pose, zero velocity
q_init = np.zeros((1, NDOF), dtype=np.float32)
q_init[0, body_idx] = DEFAULT_Q
art.set_dof_positions(q_init, all_idx)
art.set_dof_velocities(np.zeros((1, NDOF), dtype=np.float32), all_idx)
art.set_dof_position_targets(q_init, all_idx)

# hands: implicit PhysX position drives on every finger joint (coupled distal joints follow ratio x proximal;
# PhysX ignores the importer's NewtonMimicAPI). Gains: robot_spec.json per-finger kp, light damping.
stiff = np.zeros((1, NDOF), dtype=np.float32)
damp = np.zeros((1, NDOF), dtype=np.float32)
hgain = {**C.hand_gains("left"), **C.hand_gains("right")}
act_of = {}
for s_ in ("left", "right"):
    for d in C.HAND_DOFS:
        for n, r in hands[s_].get(d, []):
            act_of[n] = hands[s_][d][0][0]
hand_dof_idx = [dof_names.index(n) for n, _r, _s, _d in hand_all]
hand_act_idx = [dof_names.index(n) for n in hand_act]
for n, _r, _s, _d in hand_all:
    kp = hgain.get(act_of[n], (2.0, 1.0))[0]
    stiff[0, dof_names.index(n)] = kp
    damp[0, dof_names.index(n)] = 0.02 * kp
art.set_dof_stiffnesses(stiff, all_idx)
art.set_dof_dampings(damp, all_idx)
hand_target = {"left": np.zeros(6), "right": np.zeros(6)}


def hand_joint_targets(side, v01):
    """0 (open) .. 1 (closed) per slot -> {dof index: target}; spec normalisation q = u * range_upper."""
    out = {}
    for k, dof in enumerate(C.HAND_DOFS):
        entries = hands[side].get(dof, [])
        if not entries:
            continue
        i0 = dof_names.index(entries[0][0])
        q0 = float(np.clip(v01[k], 0, 1)) * lims[i0][1]
        for n, r in entries:
            i = dof_names.index(n)
            out[i] = float(np.clip(r * q0, lims[i][0], lims[i][1]))
    return out


# ---- DDS low-level channel ---------------------------------------------------------------------------
# unitree_sdk2py's ChannelFactoryInitialize() creates the CycloneDDS domain with its own XML config, which fails
# when the ROS 2 rmw (same libddsc, same process) already created the domain. So we attach plain CycloneDDS
# entities to the existing domain instead (same topics, types and default QoS as unitree ChannelPublisher/
# ChannelSubscriber). Network interface selection: CYCLONEDDS_URI (shared by rclpy and this channel).
import rclpy  # noqa: E402  (internal Humble rclpy; must init before the Unitree participant, see above)

if args.net_if:
    os.environ["CYCLONEDDS_URI"] = ("<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"%s\"/>"
                                    "</Interfaces></General></Domain></CycloneDDS>" % args.net_if)
rclpy.init(args=None, domain_id=args.domain_id)
node = rclpy.create_node("g1_isaac_sim")  # creates the CycloneDDS domain (rmw config); must precede our participant
from cyclonedds.core import Listener  # noqa: E402
from cyclonedds.domain import DomainParticipant  # noqa: E402
from cyclonedds.internal import InvalidSample  # noqa: E402
from cyclonedds.pub import DataWriter  # noqa: E402
from cyclonedds.sub import DataReader  # noqa: E402
from cyclonedds.topic import Topic  # noqa: E402
from cyclonedds.util import duration  # noqa: E402
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

dds_participant = DomainParticipant(args.domain_id)
crc = CRC()
low_state = unitree_hg_msg_dds__LowState_()
low_state.mode_machine = 5  # G1 29-DoF
state_writer = DataWriter(dds_participant, Topic(dds_participant, "rt/lowstate", LowState_))


class _StatePub:
    def Write(self, m):
        state_writer.write(m)


state_pub = _StatePub()
for _k in range(29):
    low_state.motor_state[_k].mode = 1
fast = None
try:
    import copy

    from fast_lowstate import FastLowState

    fast = FastLowState(state_writer, copy.deepcopy(low_state), crc, os.path.join(HERE, "pydeps/unitree_sdk2py"))
    if not fast.ok:
        print("[run_sim] fast lowstate self-test FAILED; using the cyclonedds serializer", flush=True)
        fast = None
    else:
        fast.m["mode"][:29] = 1
        fast.buf["mode_machine"] = 5
        fast.buf["tick"] = 0
        fast.m["q"][:] = 0
        fast.m["q"][3] = 0.0
        print("[run_sim] fast lowstate writer: self-test OK (byte-identical CDR + CRC)", flush=True)
except Exception as e:  # noqa: BLE001
    print(f"[run_sim] fast lowstate writer unavailable: {e}", flush=True)
    fast = None

cmd_lock = threading.Lock()
cmd = {"q": DEFAULT_Q.copy(), "dq": np.zeros(29, np.float32), "tau": np.zeros(29, np.float32),
       "kp": KP0.copy(), "kd": KD0.copy(), "n": 0, "bad_crc": 0, "t_wall": 0.0, "mode_pr": 0}


def on_lowcmd(msg: LowCmd_):
    ok = True
    try:
        ok = crc.Crc(msg) == msg.crc
    except Exception:
        ok = False
    if not ok:
        cmd["bad_crc"] += 1
        if args.strict_crc:
            return
    mc = msg.motor_cmd
    q = np.fromiter((mc[i].q for i in range(29)), np.float32, 29)
    dq = np.fromiter((mc[i].dq for i in range(29)), np.float32, 29)
    tau = np.fromiter((mc[i].tau for i in range(29)), np.float32, 29)
    kp = np.fromiter((mc[i].kp for i in range(29)), np.float32, 29)
    kd = np.fromiter((mc[i].kd for i in range(29)), np.float32, 29)
    en = np.fromiter((mc[i].mode for i in range(29)), np.float32, 29)
    kp *= en > 0  # mode 0 = motor disabled (no torque)
    kd *= en > 0
    tau *= en > 0
    with cmd_lock:
        cmd.update(q=q, dq=dq, tau=tau, kp=kp, kd=kd, t_wall=time.monotonic(), mode_pr=int(msg.mode_pr))
        cmd["n"] += 1


def _on_data(reader):
    try:
        for smp in reader.take(10):
            if not isinstance(smp, InvalidSample):
                on_lowcmd(smp)
    except Exception as e:  # noqa: BLE001
        print(f"[run_sim] lowcmd read error: {e}", flush=True)


cmd_reader = DataReader(dds_participant, Topic(dds_participant, "rt/lowcmd", LowCmd_), listener=Listener(on_data_available=_on_data))

# /g1/mode (g1_interfaces/Mode, transient-local) read straight off DDS: g1_interfaces is not built for Isaac's
# Python 3.12, but its wire type is a plain struct {uint8 mode; string source; string detail}.
from cyclonedds.qos import Policy, Qos  # noqa: E402
from dds_types import Mode_  # noqa: E402

mode_state = {"mode": None, "stand_since": None}


def _on_mode(reader):
    try:
        for smp in reader.take(10):
            if isinstance(smp, InvalidSample):
                continue
            if smp.mode == 1 and mode_state["mode"] != 1:
                # sim time, not wall time: with --camera the sim runs at RTF ~0.3-0.5 and a wall-clock 2.5 s
                # released the harness ~1 s (sim) into the controller's 2 s stand-up -> fall
                mode_state["stand_since"] = globals().get("sim_t", 0.0)
            if smp.mode == 0:
                mode_state["stand_since"] = None
            mode_state["mode"] = int(smp.mode)
    except Exception as e:  # noqa: BLE001
        print(f"[run_sim] /g1/mode read error: {e}", flush=True)


mode_reader = DataReader(dds_participant, Topic(dds_participant, "rt/g1/mode", Mode_),
                         qos=Qos(Policy.Reliability.Reliable(duration(seconds=1)), Policy.Durability.TransientLocal,
                                 Policy.History.KeepLast(1)),
                         listener=Listener(on_data_available=_on_mode))
print("[run_sim] DDS: publishing rt/lowstate, subscribed rt/lowcmd (domain %d)" % args.domain_id, flush=True)

# ---- ROS 2 (bundled rclpy) ------------------------------------------------------------------------------
from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2, PointField  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
TP = args.topic_prefix.rstrip("/")
pub_clock = node.create_publisher(Clock, TP + "/clock", 10)
pub_js = node.create_publisher(JointState, TP + "/joint_states", 10)
pub_odom = node.create_publisher(Odometry, TP + "/odom", 10)
pub_tf = node.create_publisher(TFMessage, TP + "/tf", 50)
pub_cloud = node.create_publisher(PointCloud2, TP + "/livox/lidar", 5)
pub_scan = node.create_publisher(LaserScan, TP + "/scan", 5)
pub_hand_state = {s: node.create_publisher(JointState, f"/g1/hand/{s}/state", 10) for s in ("left", "right")}
pub_gt = node.create_publisher(Odometry, "/sim/ground_truth", 10)
pub_tf_static = node.create_publisher(TFMessage, TP + "/tf_static", QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
_st = TransformStamped()
_st.header.frame_id = "mid360_link"
_st.child_frame_id = "laser_link"
_st.transform.rotation.x = 1.0  # rotation by pi about x (same static TF as g1_mujoco_sim)
_st.transform.rotation.w = 0.0
pub_tf_static.publish(TFMessage(transforms=[_st]))
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402

pub_status = node.create_publisher(String, "/sim/status", 10)
_h_on = args.harness == "on" and not args.fixed_base  # the weld replaces the harness
harness = {"on": _h_on, "gain": 1.0 if _h_on else 0.0, "anchor": None, "quat": None, "reset": False}


def _srv_harness(req, res):
    harness["on"] = bool(req.data)
    if harness["on"]:
        harness["gain"] = 1.0
        harness["anchor"] = None  # re-anchored at the current x, y and the spawn height
    res.success = True
    res.message = f"harness {'on' if harness['on'] else 'releasing'}"
    return res


def _srv_reset(req, res):
    harness["reset"] = True
    res.success = True
    res.message = "respawning with harness"
    return res


node.create_service(SetBool, "/sim/harness", _srv_harness)
node.create_service(Trigger, "/sim/reset", _srv_reset)
from sensor_msgs.msg import CameraInfo  # noqa: E402

pub_img = node.create_publisher(Image, "/g1/head_camera/image_raw", sensor_qos) if args.camera else None
pub_depth = node.create_publisher(Image, "/g1/head_camera/depth", sensor_qos) if args.camera else None
pub_cinfo = node.create_publisher(CameraInfo, "/g1/head_camera/camera_info", sensor_qos) if args.camera else None
pub_objects = node.create_publisher(String, "/sim/objects", 10)


def make_hand_cb(side):
    def cb(msg):
        v = np.asarray(msg.position[:6], dtype=float)
        if v.size == 6:
            hand_target[side] = v
    return cb


for s in ("left", "right"):
    node.create_subscription(JointState, f"/g1/hand/{s}/cmd", make_hand_cb(s), 10)


def stamp(t):
    m = TimeMsg()
    m.sec = int(t)
    m.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
    return m


# ---- geometry helpers --------------------------------------------------------------------------------
def quat_xyzw_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_from_xyzw(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rel_transform(parent, child):
    """child pose in parent frame (4x4, row-vector USD convention converted to column-vector numpy)."""
    cache = UsdGeom.XformCache()
    mp = cache.GetLocalToWorldTransform(parent)
    mc = cache.GetLocalToWorldTransform(child)
    rel = mc * mp.GetInverse()
    return np.array(rel).T  # column-vector convention


torso_i = link_names.index("torso_link")
mid360 = find_prim("mid360_link")
torso_prim = find_prim("torso_link")
T_torso_mid = rel_transform(torso_prim, mid360) if (mid360 and torso_prim) else np.array(
    [[1, 0, 0, 0.0003], [0, 1, 0, 0], [0, 0, 1, 0.416], [0, 0, 0, 1]], dtype=float)
T_torso_mid[:3, :3] /= np.linalg.norm(T_torso_mid[:3, :3], axis=0)

# livox_ros_driver2 PointCloud2 point layout (26 bytes), identical to the MuJoCo backend
LIVOX_FIELDS = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
                PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
                PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1)]
LIVOX_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("tag", "u1"),
                        ("line", "u1"), ("timestamp", "<f8")])
FLIP_X = np.diag([1.0, -1.0, -1.0, 1.0])  # laser_link = mid360_link rotated by pi about x (z up)
SCAN_RANGE_MAX = 30.0

# ---- lidar (Warp ray casting on a static collision snapshot) -------------------------------------------------
lidar = None
if args.lidar == "warp":
    try:
        from lidar_warp import WarpLidar

        lidar = WarpLidar(stage, exclude_roots=[G1], device=args.lidar_device)
        print(f"[run_sim] lidar: {lidar.describe()}", flush=True)
    except Exception as e:  # pragma: no cover
        print(f"[run_sim] lidar disabled: {e}", flush=True)
        lidar = None

# ---- camera (optional) ------------------------------------------------------------------------------------
cam_annot = None
depth_annot = None
CAM_W, CAM_H = (int(v) for v in args.camera_res.lower().split("x"))
CAM_VFOV = math.radians(args.camera_vfov)  # 42 = D435 colour stream (69.4 x 42.5 deg at 16:9); same as the MJCF camera head_d435
CAM_FY = CAM_H / (2 * math.tan(CAM_VFOV / 2))
CAM_FX = CAM_FY  # square pixels
cam_info_msg = None
if args.camera:
    import omni.replicator.core as rep

    d435 = find_prim("d435_link")
    cam_path = str(d435.GetPath()) + "/head_camera" if d435 else G1 + "/head_camera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    _focal = 1.93  # mm; apertures chosen so fx = fy = CAM_FY for the given resolution
    cam.CreateFocalLengthAttr(_focal)
    cam.CreateHorizontalApertureAttr(_focal * CAM_W / CAM_FX)
    cam.CreateVerticalApertureAttr(_focal * CAM_H / CAM_FY)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
    # USD camera looks down -Z with +Y up; d435_link is x-forward, z-up
    cx = UsdGeom.Xformable(cam)
    if args.camera_pitch is not None:
        # video view: undo the D435 mount pitch (0.8308 rad about d435_link y, URDF d435_joint) and pitch down by
        # --camera-pitch instead (rotation about the link y axis, applied in the link frame)
        _mount = math.degrees(0.8307767239493009)
        cx.AddRotateYOp(opSuffix="videoPitch").Set(args.camera_pitch - _mount)
        print(f"[run_sim] head camera VIDEO VIEW: {args.camera_pitch:.1f} deg below the torso x axis "
              f"(D435 mount is {_mount:.1f} deg)", flush=True)
    cx.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, -90))  # camera -Z -> link +x, camera +Y -> link +z
    rp = rep.create.render_product(cam_path, (CAM_W, CAM_H))
    cam_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    cam_annot.attach([rp])
    if args.camera_depth == "on":
        depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        depth_annot.attach([rp])
    cam_info_msg = CameraInfo()
    cam_info_msg.header.frame_id = "d435_link"
    cam_info_msg.width, cam_info_msg.height = CAM_W, CAM_H
    cam_info_msg.distortion_model = "plumb_bob"
    cam_info_msg.d = [0.0] * 5
    _cx, _cy = CAM_W / 2.0, CAM_H / 2.0
    cam_info_msg.k = [CAM_FX, 0.0, _cx, 0.0, CAM_FY, _cy, 0.0, 0.0, 1.0]
    cam_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info_msg.p = [CAM_FX, 0.0, _cx, 0.0, 0.0, CAM_FY, _cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    print(f"[run_sim] head camera {CAM_W}x{CAM_H} fx=fy={CAM_FX:.1f} (optical convention: x right, y down, z forward "
          f"= d435_link rotated; see perception/README.md)", flush=True)

# ---- theatre objects: /sim/objects ground truth ---------------------------------------------------------------
obj_dynamic, obj_static = [], {}
_theatre = stage.GetPrimAtPath("/World/Theatre")
if _theatre and _theatre.IsValid():
    _xc = UsdGeom.XformCache()
    for _p in Usd.PrimRange(_theatre):
        _cd = _p.GetCustomDataByKey("hg1:type")
        if not _cd:
            continue
        _nm = _p.GetCustomDataByKey("hg1:name") or _p.GetName()
        if _p.HasAPI(UsdPhysics.RigidBodyAPI):
            obj_dynamic.append((_nm, _cd, str(_p.GetPath())))
        else:
            _m = _xc.GetLocalToWorldTransform(_p)
            _t = _m.ExtractTranslation()
            _q = _m.ExtractRotationQuat()
            obj_static[_nm] = {"pose": [round(v, 5) for v in (_t[0], _t[1], _t[2], _q.GetReal(), *_q.GetImaginary())],
                               "type": _cd}
    print(f"[run_sim] /sim/objects: {len(obj_dynamic)} dynamic + {len(obj_static)} static theatre objects", flush=True)
obj_view = None
# ---- main loop --------------------------------------------------------------------------------------------
if obj_dynamic:
    obj_view = view.create_rigid_body_view([p_ for _n, _c, p_ in obj_dynamic])
objects_every = max(1, int(round(1.0 / (DT * args.objects_hz))))
render_hz = args.render_hz if args.render_hz is not None else (30.0 if args.gui else (args.camera_hz if args.camera else 0.0))
render_every = int(round(1.0 / (DT * render_hz))) if render_hz > 0 else 0
lidar_every = max(1, int(round(1.0 / (DT * args.lidar_hz))))
js_every = 5  # 100 Hz joint_states / odom / tf
cam_every = max(1, int(round(1.0 / (DT * args.camera_hz))))

odom_origin = None
step = 0
sim_t = 0.0
prev_lin_vel = None
wall0 = time.monotonic()
stat_t, stat_steps, stat_busy = wall0, 0, 0.0
logf = open(args.log, "w") if args.log else None
if logf:
    logf.write("t,pelvis_z,roll,pitch,yaw,x,y,n_cmd\n")
tau_applied = np.zeros(29, np.float32)
forces = np.zeros((1, NDOF), dtype=np.float32)
targets = q_init.copy()
last_hand = None
first_cmd_logged = False
running = True
pelvis_i = link_names.index("pelvis")
root = np.asarray(art.get_root_transforms()).reshape(7).copy()
vel = np.zeros(6)
harness["quat"] = root[3:7].copy()  # xyzw
SPAWN = (sx, sy, sz, syaw)
h_force = np.zeros((1, len(link_names), 3), dtype=np.float32)
h_torque = np.zeros((1, len(link_names), 3), dtype=np.float32)
status_every = 500
st_wall, st_sim, st_cmd, st_busy = time.monotonic(), 0.0, 0, 0.0


def quat_mul_xyzw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def apply_harness():
    """Spring-damper on the pelvis toward the spawn pose (g1_mujoco_sim law and gains), faded out on release."""
    if harness["on"] and mode_state["stand_since"] is not None and sim_t - mode_state["stand_since"] > 2.5:
        harness["on"] = False
    if not harness["on"] and harness["gain"] > 0:
        harness["gain"] = max(0.0, harness["gain"] - DT / 1.0)
    if harness["gain"] <= 0:
        return
    if harness["anchor"] is None:
        harness["anchor"] = np.array([root[0], root[1], SPAWN[2]])
    g = harness["gain"]
    f = np.array([1500.0, 1500.0, 4000.0]) * (harness["anchor"] - root[:3]) - 300.0 * vel[:3]
    q = root[3:7]
    qe = quat_mul_xyzw(harness["quat"], np.array([-q[0], -q[1], -q[2], q[3]]))
    if qe[3] < 0:
        qe = -qe
    tq = 300.0 * 2.0 * qe[:3] - 30.0 * vel[3:6]
    h_force[0, pelvis_i] = g * f
    h_torque[0, pelvis_i] = g * tq
    art.apply_forces_and_torques_at_position(h_force, h_torque, None, all_idx, True)


def respawn():
    x, y, z, yaw = SPAWN
    art.set_root_transforms(np.array([[x, y, z, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]], np.float32), all_idx)
    art.set_root_velocities(np.zeros((1, 6), np.float32), all_idx)
    art.set_dof_positions(q_init, all_idx)
    art.set_dof_velocities(np.zeros((1, NDOF), np.float32), all_idx)
    harness.update(on=args.harness == "on", gain=1.0 if args.harness == "on" else 0.0, anchor=None, reset=False)
    harness["quat"] = np.array([0, 0, math.sin(yaw / 2), math.cos(yaw / 2)])
    with cmd_lock:
        cmd["n"] = 0  # motors follow --idle again until the next /lowcmd
    print("[run_sim] respawned (/sim/reset)", flush=True)


prof = {}
_pt = [0.0]


def _P(name):
    now = time.perf_counter()
    if name is not None and args.profile:
        prof[name] = prof.get(name, 0.0) + now - _pt[0]
    _pt[0] = now


print("[run_sim] running (Ctrl-C to stop)", flush=True)
try:
    while running and app.is_running():
        t_loop = time.monotonic()
        _P(None)
        q_all = np.asarray(art.get_dof_positions()).reshape(NDOF)
        dq_all = np.asarray(art.get_dof_velocities()).reshape(NDOF)
        q = q_all[body_idx]
        dq = dq_all[body_idx]

        with cmd_lock:
            n_cmd = cmd["n"]
            c_q, c_dq, c_tau, c_kp, c_kd = cmd["q"], cmd["dq"], cmd["tau"], cmd["kp"], cmd["kd"]
        if n_cmd == 0:
            if args.idle == "hold":
                c_q, c_dq, c_tau, c_kp, c_kd = DEFAULT_Q, np.zeros(29), np.zeros(29), KP0, KD0
            elif args.idle == "damp":
                c_q, c_dq, c_tau, c_kp, c_kd = q, np.zeros(29), np.zeros(29), np.zeros(29), KD0
            else:
                c_q, c_dq, c_tau, c_kp, c_kd = q, np.zeros(29), np.zeros(29), np.zeros(29), np.zeros(29)
        elif not first_cmd_logged:
            first_cmd_logged = True
            print(f"[run_sim] first /lowcmd received at sim t={sim_t:.3f}s", flush=True)

        if args.pd == "explicit":
            tau = c_kp * (c_q - q) + c_kd * (c_dq - dq) + c_tau
            tau = np.clip(tau, -TAU_MAX, TAU_MAX)
            forces[0, body_idx] = tau
            tau_applied = tau
        else:
            stiff[0, body_idx] = c_kp
            damp[0, body_idx] = c_kd
            art.set_dof_stiffnesses(stiff, all_idx)
            art.set_dof_dampings(damp, all_idx)
            targets[0, body_idx] = c_q
            vt = np.zeros((1, NDOF), dtype=np.float32)
            vt[0, body_idx] = c_dq
            art.set_dof_velocity_targets(vt, all_idx)
            forces[0, body_idx] = np.clip(c_tau, -TAU_MAX, TAU_MAX)
            tau_applied = np.clip(c_kp * (c_q - q) + c_kd * (c_dq - dq) + c_tau, -TAU_MAX, TAU_MAX)
        # hands
        ht = (hand_target["left"].tobytes(), hand_target["right"].tobytes())
        if hand_dof_idx and ht != last_hand:
            last_hand = ht
            for s in ("left", "right"):
                for i, v in hand_joint_targets(s, hand_target[s]).items():
                    targets[0, i] = v
        if hand_dof_idx or args.pd == "implicit":
            art.set_dof_position_targets(targets, all_idx)
        art.set_dof_actuation_forces(forces, all_idx)
        if harness["reset"]:
            respawn()
        apply_harness()

        _P('ctrl')
        # simulate directly: SimulationManager.step() calls get_simulation_time(), whose native implementation
        # traverses the whole USD stage every call (~9 ms/step in the hospital).
        phys_sim.simulate(DT, sim_t)
        if FABRIC and render_every and (step + 1) % render_every == 0:
            fabric_if.update(sim_t, DT)
        _P('physics')
        step += 1
        sim_t = step * DT

        # ---- state out -------------------------------------------------------------------------------
        root = np.asarray(art.get_root_transforms()).reshape(7)  # x y z qx qy qz qw
        vel = np.asarray(art.get_root_velocities()).reshape(6)  # world lin, world ang
        R = quat_xyzw_to_mat(root[3:7])
        lin_acc = (vel[:3] - prev_lin_vel) / DT if prev_lin_vel is not None else np.zeros(3)
        prev_lin_vel = vel[:3].copy()
        gyro_b = R.T @ vel[3:6]
        acc_b = R.T @ (lin_acc + np.array([0.0, 0.0, 9.81]))
        rpy = rpy_from_xyzw(root[3:7])

        if fast is not None:
            fm = fast.m
            fm["q"][:29] = q_all[body_idx]
            fm["dq"][:29] = dq_all[body_idx]
            fm["tau_est"][:29] = tau_applied
            fi = fast.imu
            fi["quaternion"] = (root[6], root[3], root[4], root[5])  # w x y z
            fi["gyroscope"] = gyro_b
            fi["accelerometer"] = acc_b
            fi["rpy"] = rpy
            fast.buf["tick"] = (step * 2) & 0xFFFFFFFF  # ms
            _P('state_fill')
            fast.write()
            _P('dds_write')
        else:
            ms = low_state.motor_state
            for k in range(29):
                m = ms[k]
                m.q = float(q_all[body_idx[k]])
                m.dq = float(dq_all[body_idx[k]])
                m.tau_est = float(tau_applied[k])
                m.mode = 1
            imu = low_state.imu_state
            imu.quaternion = [float(root[6]), float(root[3]), float(root[4]), float(root[5])]  # w x y z
            imu.gyroscope = [float(v) for v in gyro_b]
            imu.accelerometer = [float(v) for v in acc_b]
            imu.rpy = [float(v) for v in rpy]
            low_state.tick = (step * 2) & 0xFFFFFFFF  # ms
            _P('state_fill')
            low_state.crc = crc.Crc(low_state)
            state_pub.Write(low_state)
        _P('dds_write')

        now = stamp(sim_t)
        cm = Clock()
        cm.clock = now
        pub_clock.publish(cm)

        _P('clock')
        if step % js_every == 0:
            if odom_origin is None:
                odom_origin = (root[0], root[1], rpy[2])
            ox, oy, oyaw = odom_origin
            co, so = math.cos(-oyaw), math.sin(-oyaw)
            dx, dy = root[0] - ox, root[1] - oy
            px, py = co * dx - so * dy, so * dx + co * dy
            # orientation relative to odom frame: q_odom^-1 * q
            hq = (0.0, 0.0, math.sin(-oyaw / 2), math.cos(-oyaw / 2))
            x1, y1, z1, w1 = hq
            x2, y2, z2, w2 = root[3:7]
            qo = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
            od = Odometry()
            od.header.stamp = now
            od.header.frame_id = "odom"
            od.child_frame_id = "pelvis"
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = float(px), float(py), float(root[2])
            (od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z,
             od.pose.pose.orientation.w) = (float(v) for v in qo)
            vb = R.T @ vel[:3]
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = (float(v) for v in vb)
            od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = (float(v) for v in gyro_b)
            pub_odom.publish(od)
            gt = Odometry()
            gt.header.stamp = now
            gt.header.frame_id = "world"
            gt.child_frame_id = "pelvis"
            gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z = (float(v) for v in root[:3])
            (gt.pose.pose.orientation.x, gt.pose.pose.orientation.y, gt.pose.pose.orientation.z,
             gt.pose.pose.orientation.w) = (float(v) for v in root[3:7])
            gt.twist.twist = od.twist.twist
            pub_gt.publish(gt)
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = "pelvis"
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = float(px), float(py), float(root[2])
            tf.transform.rotation = od.pose.pose.orientation
            _tfs = [tf]
            if args.base_footprint:
                _Tp = np.eye(4)
                _Tp[:3, :3] = quat_xyzw_to_mat(root[3:7])
                _Tp[:3, 3] = root[:3]
                _yw = math.atan2(_Tp[1, 0], _Tp[0, 0])
                _F = np.eye(4)
                _F[:3, :3] = [[math.cos(_yw), -math.sin(_yw), 0.0], [math.sin(_yw), math.cos(_yw), 0.0], [0.0, 0.0, 1.0]]
                _F[0, 3], _F[1, 3] = root[0], root[1]
                _T = np.linalg.inv(_Tp) @ _F
                _R = _T[:3, :3]
                _w = math.sqrt(max(1e-12, 1.0 + _R[0, 0] + _R[1, 1] + _R[2, 2])) / 2.0
                bf = TransformStamped()
                bf.header.stamp = now
                bf.header.frame_id = "pelvis"
                bf.child_frame_id = "base_footprint"
                bf.transform.translation.x, bf.transform.translation.y, bf.transform.translation.z = (
                    float(v) for v in _T[:3, 3])
                bf.transform.rotation.w = _w
                bf.transform.rotation.x = (_R[2, 1] - _R[1, 2]) / (4 * _w)
                bf.transform.rotation.y = (_R[0, 2] - _R[2, 0]) / (4 * _w)
                bf.transform.rotation.z = (_R[1, 0] - _R[0, 1]) / (4 * _w)
                _tfs.append(bf)
            pub_tf.publish(TFMessage(transforms=_tfs))
            js = JointState()
            js.header.stamp = now
            js.name = list(C.BODY_JOINTS) + hand_act
            idx = list(body_idx) + hand_act_idx
            js.position = [float(q_all[i]) for i in idx]
            js.velocity = [float(dq_all[i]) for i in idx]
            js.effort = [float(v) for v in tau_applied] + [0.0] * len(hand_act_idx)
            pub_js.publish(js)
            if hand_dof_idx and step % (js_every * 4) == 0:
                for s in ("left", "right"):
                    hs = JointState()
                    hs.header.stamp = now
                    hs.name = list(C.HAND_DOFS)
                    pos = []
                    for d in C.HAND_DOFS:
                        if d in hands[s]:
                            i = dof_names.index(hands[s][d][0][0])
                            pos.append(float(np.clip(q_all[i] / (lims[i][1] + 1e-9), 0, 1)))
                        else:
                            pos.append(0.0)
                    hs.position = pos
                    pub_hand_state[s].publish(hs)
            rclpy.spin_once(node, timeout_sec=0)
            if logf and step % 10 == 0:
                logf.write(f"{sim_t:.3f},{root[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f},{root[0]:.3f},{root[1]:.3f},{n_cmd}\n")

        if (obj_dynamic or obj_static) and step % objects_every == 0:
            objs = dict(obj_static)
            if obj_view is not None:
                tr_ = np.asarray(obj_view.get_transforms()).reshape(-1, 7)  # x y z qx qy qz qw
                for (nm_, cd_, _p), t_ in zip(obj_dynamic, tr_):
                    objs[nm_] = {"pose": [round(float(v), 5) for v in (t_[0], t_[1], t_[2], t_[6], t_[3], t_[4], t_[5])],
                                 "type": cd_}
            pub_objects.publish(String(data=json.dumps(objs)))
        _P('ros_100hz')
        if lidar is not None and step % lidar_every == 0:
            links = np.asarray(art.get_link_transforms()).reshape(-1, 7)
            tt = links[torso_i]
            Tw = np.eye(4)
            Tw[:3, :3] = quat_xyzw_to_mat(tt[3:7])
            Tw[:3, 3] = tt[:3]
            Tm = Tw @ T_torso_mid
            pts, ring = lidar.cloud(Tm)
            cl = np.zeros(pts.shape[0], dtype=LIVOX_DTYPE)
            cl["x"], cl["y"], cl["z"], cl["intensity"] = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
            cl["tag"] = 0x10
            cl["line"] = ring % 4
            cl["timestamp"] = sim_t * 1e9
            pc = PointCloud2()
            pc.header.stamp = now
            pc.header.frame_id = "mid360_link"
            pc.height = 1
            pc.width = cl.shape[0]
            pc.fields = LIVOX_FIELDS
            pc.is_bigendian = False
            pc.point_step = LIVOX_DTYPE.itemsize
            pc.row_step = LIVOX_DTYPE.itemsize * cl.shape[0]
            pc.is_dense = True
            pc.data = array.array("B", cl.tobytes())
            pub_cloud.publish(pc)
            if args.scan_level:
                Tp = np.eye(4)
                Tp[:3, :3] = quat_xyzw_to_mat(root[3:7])
                Tp[:3, 3] = root[:3]
                _yaw = math.atan2(Tp[1, 0], Tp[0, 0])  # pelvis heading
                Tl = np.eye(4)
                Tl[:3, :3] = np.array([[math.cos(_yaw), -math.sin(_yaw), 0.0], [math.sin(_yaw), math.cos(_yaw), 0.0],
                                       [0.0, 0.0, 1.0]])
                Tl[:3, 3] = Tm[:3, 3]
                scan = lidar.scan2d(Tl, range_max=SCAN_RANGE_MAX)
                Tpl = np.linalg.inv(Tp) @ Tl
                _R = Tpl[:3, :3]
                _w = math.sqrt(max(1e-12, 1.0 + _R[0, 0] + _R[1, 1] + _R[2, 2])) / 2.0
                tl = TransformStamped()
                tl.header.stamp = now
                tl.header.frame_id = "pelvis"
                tl.child_frame_id = "laser_level"
                tl.transform.translation.x, tl.transform.translation.y, tl.transform.translation.z = (
                    float(v) for v in Tpl[:3, 3])
                tl.transform.rotation.w = _w
                tl.transform.rotation.x = (_R[2, 1] - _R[1, 2]) / (4 * _w)
                tl.transform.rotation.y = (_R[0, 2] - _R[2, 0]) / (4 * _w)
                tl.transform.rotation.z = (_R[1, 0] - _R[0, 1]) / (4 * _w)
                pub_tf.publish(TFMessage(transforms=[tl]))
            else:
                scan = lidar.scan2d(Tm @ FLIP_X, range_max=SCAN_RANGE_MAX)
            ls = LaserScan()
            ls.header.stamp = now
            ls.header.frame_id = "laser_level" if args.scan_level else "laser_link"
            ls.angle_min = -math.pi
            ls.angle_increment = 2 * math.pi / scan.size
            ls.angle_max = ls.angle_min + ls.angle_increment * (scan.size - 1)
            ls.scan_time = 1.0 / args.lidar_hz
            ls.time_increment = 0.0
            ls.range_min, ls.range_max = lidar.range_min, SCAN_RANGE_MAX
            ls.ranges = array.array("f", scan.tobytes())
            pub_scan.publish(ls)

        if render_every and step % render_every == 0:
            app.update()
            if cam_annot is not None and step % cam_every == 0:
                data = cam_annot.get_data()
                if data is not None and getattr(data, "size", 0):
                    img = np.asarray(data)[..., :3]
                    im = Image()
                    im.header.stamp = now
                    im.header.frame_id = "d435_link"
                    im.height, im.width = img.shape[:2]
                    im.encoding = "rgb8"
                    im.step = img.shape[1] * 3
                    im.data = np.ascontiguousarray(img).tobytes()
                    pub_img.publish(im)
                    dep = depth_annot.get_data() if depth_annot is not None else None
                    if dep is not None and getattr(dep, "size", 0):
                        dm = np.asarray(dep, dtype=np.float32).reshape(img.shape[0], img.shape[1])
                        dm = np.where(np.isfinite(dm) & (dm > 0) & (dm < 50.0), dm, np.float32("nan"))
                        di = Image()
                        di.header = im.header
                        di.height, di.width = dm.shape
                        di.encoding = "32FC1"  # metres along the optical axis (REP 118); NaN = no return
                        di.step = dm.shape[1] * 4
                        di.data = np.ascontiguousarray(dm).tobytes()
                        pub_depth.publish(di)
                    cam_info_msg.header = im.header
                    pub_cinfo.publish(cam_info_msg)

        _P('lidar_render')
        busy = time.monotonic() - t_loop
        stat_busy += busy
        stat_steps += 1
        if not args.no_realtime:
            target = wall0 + sim_t
            slack = target - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.5:
                wall0 -= slack  # fell behind: do not try to catch up in a burst
        st_busy += busy
        if step % status_every == 0:
            _now = time.monotonic()
            pub_status.publish(String(data=json.dumps({
                "backend": "isaac", "sim_time": round(sim_t, 3), "rtf": round((sim_t - st_sim) / max(1e-9, _now - st_wall), 3),
                "step_ms_mean": round(1e3 * st_busy / status_every, 3),
                "lowcmd_hz": round((n_cmd - st_cmd) / max(1e-9, _now - st_wall), 1), "lowcmd_bad_crc": cmd["bad_crc"],
                "harness": round(harness["gain"], 2), "world": "flat" if args.flat else "hospital",
                "hands_modelled": bool(hand_act), "pelvis_xyz": [round(float(v), 3) for v in root[:3]]})))
            st_wall, st_sim, st_cmd, st_busy = _now, sim_t, n_cmd, 0.0
        if args.stats and time.monotonic() - stat_t >= args.stats:
            el = time.monotonic() - stat_t
            _j = int(np.argmax(np.abs(dq_all)))
            print(f"[run_sim] t={sim_t:7.2f}s  max|dq| {abs(dq_all[_j]):.1f} ({dof_names[_j]})  rate {stat_steps / el:6.1f} Hz (target 500)  busy {1e3 * stat_busy / max(1, stat_steps):.2f} ms/step  "
                  f"pelvis z {root[2]:.3f} rpy ({math.degrees(rpy[0]):.1f},{math.degrees(rpy[1]):.1f})  lowcmd rx {n_cmd} "
                  f"(bad crc {cmd['bad_crc']})", flush=True)
            if args.profile:
                print("[run_sim] profile ms/step: " + ", ".join(f"{k} {1e3 * v / max(1, stat_steps):.3f}" for k, v in prof.items()), flush=True)
                prof.clear()
            stat_t, stat_steps, stat_busy = time.monotonic(), 0, 0.0
        if args.duration and sim_t >= args.duration:
            running = False
except KeyboardInterrupt:
    pass
except Exception:  # noqa: BLE001  (app.close() would otherwise swallow the traceback)
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
finally:
    print(f"[run_sim] stopping at sim t={sim_t:.2f}s", flush=True)
    if logf:
        logf.close()
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    timeline.stop()
    app.close()
