"""Isaac twin of hospital-sim-env/objects/tools/hand_grasp_test_mujoco.py, plus a fall test (grasp physics of the Revo2 hands).

    ./g1-isaac-sim/run.sh --python g1-isaac-sim/tools/hand_grasp_test.py --mode grasp --hand-physics fixed
    ./g1-isaac-sim/run.sh --python g1-isaac-sim/tools/hand_grasp_test.py --mode fall  --hand-physics raw

grasp: fixed-base G1 on the flat stage, right arm at the pose from hospital-sim-env/objects/docs/hand_grasp_spec.json (computed by
       the MuJoCo test), each instrument (hospital-sim-env/objects/usd/*.usd) is placed in the open hand at the same wrist-frame
       pose, pinned while the hand closes (0.6 s), 0.3 s gravity-free, then gravity on: hold 1.5 s, shake (wrist
       pitch +-0.35 rad, shoulder pitch +-0.25 rad, 1 Hz, 3 s), hold 1 s. Same metrics as MuJoCo.
fall:  free G1, motors off, hands half closed, pushed forward so it falls onto its hands; reports max link speed
       and whether anything became non-finite / exploded (the reason finger collisions had been disabled).
--hand-physics: fixed = hand_physics.apply (default), raw = finger colliders on without the fixes,
                off = finger colliders off (the old run_sim default).
Writes g1-isaac-sim/tools/results/hand_<mode>_<hand-physics>.json.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.dirname(HERE)
REPO = os.path.dirname(ISAAC)
sys.path.insert(0, ISAAC)
import g1_config as C  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["grasp", "fall"], default="grasp")
ap.add_argument("--hand-physics", choices=["fixed", "raw", "off"], default="fixed")
ap.add_argument("--instruments", nargs="*", default=None)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "disable_viewport_updates": True})
import carb  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import hand_physics  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(ISAAC, "usd", "flat_g1_robot.usd"))
while ctx.get_stage_loading_status()[2] > 0 or not ctx.get_stage():
    app.update()
stage = ctx.get_stage()
G1 = "/World/G1"
spec = json.load(open(os.path.join(REPO, "hospital-sim-env/objects/docs/hand_grasp_spec.json")))
cat = json.load(open(os.path.join(REPO, "hospital-sim-env/objects/catalog.json")))
names = args.instruments or cat["labels"]

root_path = next(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(G1)) if p.HasAPI(UsdPhysics.ArticulationRootAPI))
PZ = C.HOME_PELVIS_HEIGHT if args.mode == "grasp" else C.PELVIS_HEIGHT
with Usd.EditContext(stage, stage.GetSessionLayer()):
    x = UsdGeom.Xformable(stage.GetPrimAtPath(G1))
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, PZ))
    if args.hand_physics == "fixed":
        hand_physics.apply(stage, G1, collisions=True)
    else:
        n = 0
        for p in Usd.PrimRange(stage.GetPrimAtPath(G1)):
            if p.HasAPI(UsdPhysics.CollisionAPI) and hand_physics.is_hand(str(p.GetPath())):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(args.hand_physics == "raw")
                n += 1
        print(f"[test] {n} hand colliders {'ON (no fixes)' if args.hand_physics == 'raw' else 'off'}", flush=True)
    if args.mode == "grasp":
        fj = UsdPhysics.FixedJoint.Define(stage, "/World/G1_base_weld")  # fixed base (pelvis welded)
        fj.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
        fj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, PZ))
        fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        for i, n in enumerate(names):  # instruments parked on the floor, 0.5 m apart, behind the robot
            pr = stage.DefinePrim(f"/World/Instr/{n}", "Xform")
            pr.GetReferences().AddReference(os.path.join(REPO, "hospital-sim-env/objects/usd", f"{n}.usd"))
            xf = UsdGeom.Xformable(pr)
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(-1.5, -1.5 + 0.5 * i, 0.02))
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(0.7071068, 0.7071068, 0, 0))

from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

DT = 0.002
SimulationManager.set_physics_sim_device("cpu")
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Scene):
        SimulationManager.set_default_physics_scene(str(p.GetPath()))
        break
SimulationManager.set_physics_dt(DT)
settings = carb.settings.get_settings()
settings.set_bool("/physics/updateToUsd", False)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()
settings.set_bool("/app/player/playSimulations", False)
SimulationManager.initialize_physics()
ext = omni.kit.app.get_app().get_extension_manager()
ext.set_extension_enabled_immediate("isaacsim.core.simulation_manager", False)
import omni.physics.core  # noqa: E402
import omni.physics.tensors  # noqa: E402

phys = omni.physics.core.get_physics_simulation_interface()
view = omni.physics.tensors.create_simulation_view("numpy", stage_id=ctx.get_stage_id())
view.set_subspace_roots("/")
art = view.create_articulation_view(root_path)
dof = list(art.shared_metatype.dof_names)
links = list(art.shared_metatype.link_names)
ND = art.max_dofs
idx0 = np.arange(1, dtype=np.int32)
lims = np.asarray(art.get_dof_limits()).reshape(ND, 2)
hands = {s: C.hand_map(dof, s) for s in ("left", "right")}
body_idx = np.array([dof.index(n) for n in C.BODY_JOINTS])
q_def = np.zeros(ND, np.float32)
q_def[body_idx] = np.array(C.load_default_qpos()[0], np.float32)
for n, v in zip(spec["joints"], spec["q_arm"]):
    q_def[dof.index(n + "_joint")] = v
kp = np.zeros((1, ND), np.float32)
kd = np.zeros((1, ND), np.float32)
kp[0, body_idx], kd[0, body_idx] = (150.0, 5.0) if args.mode == "grasp" else (0.0, 0.0)
hg = {**C.hand_gains("left"), **C.hand_gains("right")}
for s in ("left", "right"):
    for d_, ents in hands[s].items():
        for n, _r in ents:
            k = hg.get(ents[0][0], (2.0, 1.0))[0]
            kp[0, dof.index(n)], kd[0, dof.index(n)] = k, 0.02 * k
art.set_dof_stiffnesses(kp, idx0)
art.set_dof_dampings(kd, idx0)
t_sim = [0.0]


def step(n=1):
    for _ in range(n):
        phys.simulate(DT, t_sim[0])
        t_sim[0] += DT


def hand_targets(q, side, u):
    for k, d_ in enumerate(C.HAND_DOFS):
        ents = hands[side].get(d_, [])
        if not ents:
            continue
        q0 = float(np.clip(u[k], 0, 1)) * lims[dof.index(ents[0][0])][1]
        for n, r in ents:
            i = dof.index(n)
            q[i] = np.clip(r * q0, lims[i][0], lims[i][1])
    return q


def reset_robot(u_right):
    q = hand_targets(q_def.copy(), "right", u_right)
    art.set_dof_positions(q[None], idx0)
    art.set_dof_velocities(np.zeros((1, ND), np.float32), idx0)
    art.set_dof_position_targets(q[None], idx0)
    return q


def quat_wxyz_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_xyzw(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def wrist_pose():
    lt = np.asarray(art.get_link_transforms()).reshape(-1, 7)[links.index("right_wrist_yaw_link")]
    return lt[:3].astype(float), quat_wxyz_to_R(lt[6], lt[3], lt[4], lt[5])


results = {"mode": args.mode, "hand_physics": args.hand_physics, "trials": []}
wall0 = time.time()
if args.mode == "grasp":
    rb = view.create_rigid_body_view([f"/World/Instr/{n}" for n in names])
    park = np.asarray(rb.get_transforms()).reshape(len(names), 7).copy()
    allidx = np.arange(len(names), dtype=np.int32)
    ph = spec["phases"]
    T_CLOSE, T_SOAK, T_HOLD, T_SHAKE, T_END = ph["close"], 0.3, ph["hold"], ph["shake"], ph["end"]
    rel_pos, rel_R = np.asarray(spec["rel_pos"]), np.asarray(spec["rel_R"])
    i_wp, i_sp = dof.index("right_wrist_pitch_joint"), dof.index("right_shoulder_pitch_joint")
    for ti, n in enumerate(names):
        c = cat["instruments"][n]
        gz = float(np.clip(c["com"][2], *c["grasp"]["z_range"]))
        tr = park.copy()
        rb.set_transforms(tr, allidx)
        rb.set_velocities(np.zeros((len(names), 6), np.float32), allidx)
        q = reset_robot(spec["hand_open"])
        step(250)  # settle the arm at its pose (instrument parked)
        pw, Rw = wrist_pose()
        Ro = Rw @ rel_R
        po = pw + Rw @ rel_pos - Ro @ np.array([0, 0, gz])
        hold = np.r_[po, R_to_xyzw(Ro)].astype(np.float32)
        max_slip = max_rot = shake_slip = shake_rot = max_fv = max_v = 0.0
        rel0 = rel1 = None
        finite = True
        hand_dofs = [dof.index(ents[0][0]) for ents in hands["right"].values()]
        total = T_CLOSE + T_SOAK + T_HOLD + T_SHAKE + T_END
        for k in range(int(total / DT)):
            t = k * DT
            tgt = q.copy()
            s = min(1.0, t / (0.8 * T_CLOSE))
            tgt = hand_targets(tgt, "right", (1 - s) * np.asarray(spec["hand_open"]) + s * np.asarray(spec["hand_closed"]))
            if T_CLOSE + T_SOAK + T_HOLD <= t < T_CLOSE + T_SOAK + T_HOLD + T_SHAKE:
                w = 2 * math.pi * (t - T_CLOSE - T_SOAK - T_HOLD)
                tgt[i_wp] += 0.35 * math.sin(w)
                tgt[i_sp] += 0.25 * math.sin(w)
            art.set_dof_position_targets(tgt[None], idx0)
            if t < T_CLOSE:
                tr[ti] = hold
                rb.set_transforms(tr, allidx)
                rb.set_velocities(np.zeros((len(names), 6), np.float32), allidx)
            dg = np.zeros(len(names), np.uint8)
            dg[ti] = 1 if t < T_CLOSE + T_SOAK else 0
            rb.set_disable_gravities(dg, allidx)
            step()
            X = np.asarray(rb.get_transforms()).reshape(len(names), 7)[ti].astype(float)
            V = np.asarray(rb.get_velocities()).reshape(len(names), 6)[ti].astype(float)
            dq = np.asarray(art.get_dof_velocities()).reshape(ND)
            if not (np.all(np.isfinite(X)) and np.all(np.isfinite(dq))):
                finite = False
                break
            max_fv = max(max_fv, float(np.abs(dq[hand_dofs]).max()))
            pw, Rw = wrist_pose()
            rel = Rw.T @ (X[:3] - pw)
            relR = Rw.T @ quat_wxyz_to_R(X[6], X[3], X[4], X[5])
            if t >= T_CLOSE:
                max_v = max(max_v, float(np.linalg.norm(V[:3])))
                if rel0 is None:
                    rel0, relR0 = rel.copy(), relR.copy()
                max_slip = max(max_slip, float(np.linalg.norm(rel - rel0)))
                max_rot = max(max_rot, math.degrees(math.acos(np.clip((np.trace(relR0.T @ relR) - 1) / 2, -1, 1))))
            if t >= T_CLOSE + T_SOAK + T_HOLD:
                if rel1 is None:
                    rel1, relR1 = rel.copy(), relR.copy()
                shake_slip = max(shake_slip, float(np.linalg.norm(rel - rel1)))
                shake_rot = max(shake_rot, math.degrees(math.acos(np.clip((np.trace(relR1.T @ relR) - 1) / 2, -1, 1))))
        r = dict(instrument=n, held=bool(finite and max_slip < 0.08), firm=bool(finite and shake_slip < 0.01), finite=finite,
                 settle_slip_mm=round(max_slip * 1000, 1), settle_rot_deg=round(max_rot, 1),
                 shake_slip_mm=round(shake_slip * 1000, 1), shake_rot_deg=round(shake_rot, 1),
                 max_finger_speed=round(max_fv, 2), max_instrument_speed=round(max_v, 3))
        print("[test]", json.dumps(r), flush=True)
        results["trials"].append(r)
else:
    # fall: motors off, hands half closed, a forward push at t=0 -> the robot falls onto its hands
    q = reset_robot([0.5, 0.8, 0.5, 0.5, 0.5, 0.5])
    q = hand_targets(q, "left", [0.5, 0.8, 0.5, 0.5, 0.5, 0.5])
    art.set_dof_positions(q[None], idx0)
    art.set_dof_position_targets(q[None], idx0)
    art.set_root_velocities(np.array([[0.8, 0, 0, 0, 1.5, 0]], np.float32), idx0)
    max_link_v, max_dq, finite, t_bad = 0.0, 0.0, True, None
    for k in range(int(4.0 / DT)):
        step()
        lv = np.asarray(art.get_link_velocities()).reshape(-1, 6)
        dq = np.asarray(art.get_dof_velocities()).reshape(ND)
        if not (np.all(np.isfinite(lv)) and np.all(np.isfinite(dq))):
            finite, t_bad = False, k * DT
            break
        max_link_v = max(max_link_v, float(np.linalg.norm(lv[:, :3], axis=1).max()))
        max_dq = max(max_dq, float(np.abs(dq).max()))
    root = np.asarray(art.get_root_transforms()).reshape(7)
    r = dict(finite=finite, t_nonfinite=t_bad, max_link_speed=round(max_link_v, 2), max_joint_speed=round(max_dq, 1),
             exploded=bool((not finite) or max_link_v > 20.0), final_pelvis_z=round(float(root[2]), 3))
    print("[test]", json.dumps(r), flush=True)
    results["trials"].append(r)
results["wall_s"] = round(time.time() - wall0, 1)
os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
out = os.path.join(HERE, "results", f"hand_{args.mode}_{args.hand_physics}.json")
json.dump(results, open(out, "w"), indent=1)
print(f"[test] wrote {out}", flush=True)
timeline.stop()
app.close()
