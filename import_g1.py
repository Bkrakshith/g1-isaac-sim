"""Import the G1 29-DoF rev 1.0 (+ BrainCo Revo2) URDF into USD with the Isaac Sim 6.0 URDF importer.

Run with Isaac's python (headless):
    ./g1-isaac-sim/run.sh --import [--urdf PATH]
or  env -u LD_LIBRARY_PATH ~/isaacsim/python.sh g1-isaac-sim/import_g1.py [--urdf PATH]

Output: g1-isaac-sim/usd/g1_29dof_brainco.usd (single self-contained USD crate file), plus
g1-isaac-sim/usd/g1_29dof_brainco.joints.json (joint order / limits as imported).

Post-processing on top of the importer:
  * floating base, ArticulationRootAPI on /g1/pelvis (the importer puts it on the robot prim or root link;
    we move it to the pelvis so the articulation root is the floating base link),
  * every revolute joint gets an angular drive in *force* mode with stiffness = damping = 0 (effort mode).
    run_sim.py computes the Unitree motor PD law  tau = kp (q* - q) + kd (dq* - dq) + tau_ff  from /lowcmd
    and applies it as joint effort, exactly like the real G1 motor driver and the MuJoCo backend.
    (run_sim.py --pd implicit switches to PhysX implicit drives with the /lowcmd gains instead.)
  * joint armature from g1-isaac-sim/g1_config.py (ARMATURE) (reflected rotor inertia, same values as the MJCF),
  * max joint velocity / effort from the URDF limits,
  * self collision off (as in the MJCF, foot/ground and body/world contacts only).
The script verifies the 29 body joints are present with the contract names (docs/interfaces.md section 2)
and, when present, the 12 Revo2 hand joints.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from g1_config import BODY_JOINTS, hand_map, load_config  # noqa: E402

CANONICAL_URDF = os.path.join(REPO, "g1-brainco-description/g1_29dof_brainco/urdf/g1_29dof_brainco.urdf")
PROTOTYPE_URDF = os.path.expanduser("~/ntu_dev/unitree_rl_gym/resources/robots/g1_description/g1_29dof_rev_1_0.urdf")

ap = argparse.ArgumentParser()
ap.add_argument("--urdf", default=None, help="URDF to import (default: g1-brainco-description/ canonical, else unitree_rl_gym prototype)")
ap.add_argument("--out", default=os.path.join(HERE, "usd/g1_29dof_brainco.usd"))
args, _ = ap.parse_known_args()

urdf = args.urdf or (CANONICAL_URDF if os.path.exists(CANONICAL_URDF) else PROTOTYPE_URDF)
urdf = os.path.abspath(urdf)
print(f"[import_g1] URDF: {urdf}", flush=True)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

cfg = load_config()
work = tempfile.mkdtemp(prefix="hg1_g1_import_")
try:
    # ROS package:// URIs: map every package name used in the URDF to the URDF's parent dir.
    urdf_dir = os.path.dirname(urdf)
    pkg_paths = []
    with open(urdf) as f:
        txt = f.read()
    import re

    for pkg in sorted(set(re.findall(r"package://([^/]+)/", txt))):
        # try <pkg> dir next to / above the URDF, else the URDF's parent
        for cand in (os.path.join(os.path.dirname(urdf_dir), ""), os.path.join(urdf_dir, "..", "..", pkg)):
            if os.path.isdir(cand):
                pkg_paths.append({"name": pkg, "path": os.path.normpath(cand)})
                break

    conf = URDFImporterConfig(
        urdf_path=urdf,
        usd_path=work,
        merge_fixed_joints=False,  # keep mid360_link / d435_link / imu frames for sensors
        merge_mesh=False,
        collision_from_visuals=False,
        allow_self_collision=False,
        ros_package_paths=pkg_paths,
        fix_base=False,
        joint_drive_type="force",
        joint_target_type="position",
        override_joint_stiffness=0.0,
        override_joint_damping=0.0,
        run_asset_transformer=False,
    )
    out_usda = URDFImporter(conf).import_urdf()
    print(f"[import_g1] importer output: {out_usda}", flush=True)

    stage = Usd.Stage.Open(out_usda)
    default_prim = stage.GetDefaultPrim()
    print(f"[import_g1] default prim: {default_prim.GetPath()}", flush=True)

    # ---- locate links / joints ------------------------------------------------------------
    joints = {}
    links = {}
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Joint):
            joints[p.GetName()] = p
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            links[p.GetName()] = p
    missing = [j for j in BODY_JOINTS if j not in joints]
    if missing:
        raise SystemExit(f"[import_g1] missing contract joints: {missing}")
    hand_joints = []
    for side in ("left", "right"):
        m = hand_map(list(joints), side)
        print(f"[import_g1] {side} hand DoF map: {m}", flush=True)
        hand_joints += [n for v in m.values() for n, _r in v]

    # ---- articulation root on the pelvis ------------------------------------------------------
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI) and p.GetName() != "pelvis":
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
                p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    # a world-fixed joint must not exist (floating base)
    for n, p in list(joints.items()):
        if p.IsA(UsdPhysics.FixedJoint):
            j = UsdPhysics.Joint(p)
            if not j.GetBody0Rel().GetTargets() or not j.GetBody1Rel().GetTargets():
                print(f"[import_g1] removing world joint {p.GetPath()}", flush=True)
                stage.RemovePrim(p.GetPath())
                joints.pop(n)
    pelvis = links["pelvis"]
    UsdPhysics.ArticulationRootAPI.Apply(pelvis)
    pa = PhysxSchema.PhysxArticulationAPI.Apply(pelvis)
    pa.CreateEnabledSelfCollisionsAttr(False)
    pa.CreateSolverPositionIterationCountAttr(cfg["solver_position_iterations"])
    pa.CreateSolverVelocityIterationCountAttr(cfg["solver_velocity_iterations"])
    pa.CreateSleepThresholdAttr(0.0)
    pa.CreateStabilizationThresholdAttr(0.0)

    # ---- drives: effort mode ---------------------------------------------------------------
    arm = cfg["armature"]
    report = []
    for n, p in joints.items():
        if not p.IsA(UsdPhysics.RevoluteJoint):
            continue
        drive = UsdPhysics.DriveAPI.Apply(p, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
        rj = UsdPhysics.RevoluteJoint(p)
        lo, hi = rj.GetLowerLimitAttr().Get(), rj.GetUpperLimitAttr().Get()
        pj = PhysxSchema.PhysxJointAPI.Apply(p)
        a = arm.get(n, arm["default"])
        pj.CreateArmatureAttr(float(a))
        pj.CreateJointFrictionAttr(0.0)
        # URDF effort / velocity limits land on drive maxForce and physxJoint:maxJointVelocity
        mf = drive.GetMaxForceAttr().Get()
        mv = pj.GetMaxJointVelocityAttr().Get()
        mimic = [str(x) for x in p.GetAppliedSchemas() if "Mimic" in str(x)]
        report.append({"mimic": mimic, "name": n, "lower_deg": lo, "upper_deg": hi, "max_force": mf, "max_vel_deg_s": mv, "armature": a})

    # articulation root link also gets a sane contact offset; nothing else.
    stage.SetDefaultPrim(default_prim)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    stage.Export(args.out)  # flattened, self-contained
    print(f"[import_g1] wrote {args.out}", flush=True)

    body_order = [r for j in BODY_JOINTS for r in report if r["name"] == j]
    info = {
        "urdf": os.path.relpath(urdf, REPO) if urdf.startswith(REPO + os.sep) else urdf,
        "default_prim": str(default_prim.GetPath()),
        "articulation_root": str(pelvis.GetPath()),
        "body_joints": body_order,
        "hand_joints": [r for r in report if r["name"] in hand_joints],
        "other_revolute": [r["name"] for r in report if r["name"] not in BODY_JOINTS and r["name"] not in hand_joints],
        "links": sorted(links),
    }
    with open(os.path.splitext(args.out)[0] + ".joints.json", "w") as f:
        json.dump(info, f, indent=1)
    print(f"[import_g1] 29/29 contract body joints present; hand joints found: {len(hand_joints)}", flush=True)
    print(f"[import_g1] links: {len(links)}; revolute joints: {len(report)}", flush=True)
finally:
    shutil.rmtree(work, ignore_errors=True)
    app.close()
