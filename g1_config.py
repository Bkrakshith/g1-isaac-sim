"""Shared constants for the Isaac G1 backend (no Isaac imports: usable from Isaac python and .venv).

Joint order = docs/interfaces.md section 2 (= Unitree hg motor index in LowCmd/LowState).
"""

from __future__ import annotations

import glob
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

_BODY = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]
BODY_JOINTS = [n + "_joint" for n in _BODY]
NUM_MOTORS = 29
HAND_DOFS = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]

ROBOT_SPEC = os.path.join(REPO, "g1-brainco-description/g1_29dof_brainco/robot_spec.json")
try:
    with open(ROBOT_SPEC) as _f:
        SPEC = json.load(_f)
except Exception:  # g1-brainco-description/ not built yet: prototype values (unitree_rl_gym / ULTRA)
    SPEC = None

if SPEC:
    assert SPEC["joint_names"] == BODY_JOINTS, "robot_spec.json joint order differs from the contract"
    DEFAULT_QPOS = list(SPEC["default_qpos"])       # ULTRA default_dof_pos (contract default pose)
    DEFAULT_KP = list(SPEC["kp"])                   # ULTRA released-policy gains
    DEFAULT_KD = list(SPEC["kd"])
    TORQUE_LIMIT = list(SPEC["effort_limit"])       # unitree_ros URDF <limit effort>
    ARMATURE_LIST = list(SPEC["armature"])
    HOME_PELVIS_HEIGHT = float(SPEC["home_pelvis_height"])
else:
    DEFAULT_QPOS = [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0] * 2 + [0.0] * 3 + [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0] * 2
    DEFAULT_KP = [40.18, 99.10, 40.18, 99.10, 28.50, 28.50] * 2 + [40.18, 28.50, 28.50] + (
        [14.25, 14.25, 14.25, 14.25, 14.25, 16.78, 16.78] * 2)
    DEFAULT_KD = [2.558, 6.309, 2.558, 6.309, 1.814, 1.814] * 2 + [2.558, 1.814, 1.814] + (
        [0.907, 0.907, 0.907, 0.907, 0.907, 1.068, 1.068] * 2)
    TORQUE_LIMIT = [88, 139, 88, 139, 35, 35] * 2 + [88, 35, 35] + [25, 25, 25, 25, 25, 5, 5] * 2
    ARMATURE_LIST = [0.010178, 0.025102, 0.010178, 0.025102, 0.007219, 0.007219] * 2 + [0.010178, 0.007219, 0.007219] + (
        [0.00361] * 5 + [0.00425] * 2) * 2
    HOME_PELVIS_HEIGHT = 0.7842

# Stiff "test stand" gains used only by the idle hold (--idle hold) and tools/lowcmd_hold_test.py: the ULTRA gains
# alone cannot hold the robot up without a policy (it falls in ~0.7-0.8 s). Hips/knees 500/10 as verified by the
# robot agent in MuJoCo; ankles 400/3 because an explicit PD at 2 ms is only stable for kd < 2 * I / dt, and the
# ankle's reflected inertia is ~0.008 kg m^2 (kd 10 chatters); waist 300/6; arms keep the ULTRA gains.
STAND_KP = [500.0, 500.0, 500.0, 500.0, 400.0, 400.0] * 2 + [300.0] * 3 + DEFAULT_KP[15:]
STAND_KD = [10.0, 10.0, 10.0, 10.0, 3.0, 3.0] * 2 + [6.0] * 3 + DEFAULT_KD[15:]

# Armature by joint name (used by import_g1.py); hand joints get ARMATURE["default"].
ARMATURE = {n: a for n, a in zip(BODY_JOINTS, ARMATURE_LIST)}
ARMATURE["default"] = 0.001

# Spawn poses (world frame of hospital-sim-env/isaac/usd/hospital_g1.usd = MuJoCo hospital world frame). Shared with the MuJoCo
# backend: read from g1-mujoco-sim/worlds/spawns.json when it exists. A name that the file does not have comes from
# _FALLBACK_SPAWNS (the file wins for the same name). WING_ORIGIN = layout.WING_ORIGIN: the west edge of the 4 m link
# (link wing x 0..4 = world x 17.93..21.93; hospital opening world x 18.03..20.76; FD60S doorset at world y -4.75).
WING_ORIGIN = (17.93, -4.45, 0.0)
SPAWNS_JSON = os.path.join(REPO, "g1-mujoco-sim/worlds/spawns.json")
_FALLBACK_SPAWNS = {
    "default": "corridor_west",
    "corridor_west": {"x": 8.93, "y": -12.45, "yaw": 0.0},  # west end of the surgical corridor, facing east
    "link": {"x": 18.93, "y": -5.65, "yaw": -1.570796},     # in the 4 m link (wing x 1.0), facing south into the wing
    "lobby": {"x": 18.93, "y": -1.95, "yaw": -1.570796},    # hospital SE lobby, facing the link opening
    "ward_entrance": {"x": 17.6, "y": 8.5, "yaw": 1.570796},  # ward entrance, north of the nurse base, facing the ward
    "ae_handover": {"x": -29.0, "y": 9.0, "yaw": 0.0},      # A&E hall north of the ambulance handover bay, facing east
}
PELVIS_HEIGHT = HOME_PELVIS_HEIGHT + 0.01  # spawn 1 cm above the home pelvis height (feet flat, then settle)


def load_spawns():
    try:
        with open(SPAWNS_JSON) as f:
            d = json.load(f)["hospital"]
        if d.get("default") in d:
            return {**_FALLBACK_SPAWNS, **d}
    except Exception:
        pass
    return dict(_FALLBACK_SPAWNS)


def spawn_world(name=None):
    """((x, y, z), yaw) of a named spawn (default: the MuJoCo world's default spawn)."""
    sp = load_spawns()
    name = name or sp["default"]
    p = sp[name]
    return (float(p["x"]), float(p["y"]), PELVIS_HEIGHT), float(p["yaw"])


def hand_map(all_joint_names, side):
    """{dof slot: [(joint, ratio), ...]}: first entry = actuated joint (ratio 1), then coupled (mimic) joints.
    From robot_spec.json; falls back to name matching when the spec is missing."""
    out = {}
    if SPEC:
        for _k, h in SPEC["hands"][side].items():
            if h["joint"] in all_joint_names:
                out[h["slot"]] = [(h["joint"], 1.0)] + [(n, float(r)) for n, r in h["coupled"] if n in all_joint_names]
        return out
    for slot in HAND_DOFS:
        key = {"thumb": "thumb_proximal", "thumb_aux": "thumb_metacarpal"}.get(slot, slot + "_proximal")
        hits = [n for n in all_joint_names if n.startswith(side) and key in n]
        if hits:
            out[slot] = [(hits[0], 1.0)] + [(hits[0].replace("proximal", "distal"), 1.0)] * (
                hits[0].replace("proximal", "distal") in all_joint_names and "proximal" in hits[0])
    return out


def hand_gains(side):
    """{actuated joint: (kp, effort)} from the spec."""
    if not SPEC:
        return {}
    return {h["joint"]: (float(h["kp"]), float(h["effort"])) for h in SPEC["hands"][side].values()}


def load_config():
    cfg = {
        "armature": ARMATURE,
        "solver_position_iterations": 4,  # as Unitree's Isaac Lab G1 configs (4 / 1)
        "solver_velocity_iterations": 1,
        "hand_joint_names": {"left": [], "right": []},
    }
    return cfg


def load_default_qpos():
    """default_qpos from the newest g1-whole-body-rl/exports/*/policy.json (non-placeholder preferred), else DEFAULT_QPOS."""
    best = None
    for p in sorted(glob.glob(os.path.join(REPO, "g1-whole-body-rl/exports/*/policy.json")), key=os.path.getmtime):
        try:
            with open(p) as f:
                d = json.load(f)
            if len(d.get("default_qpos", [])) == NUM_MOTORS and d.get("joint_names", BODY_JOINTS) == BODY_JOINTS:
                if best is None or not d.get("placeholder", False) or best[1].get("placeholder", False):
                    best = (p, d)
        except Exception:
            pass
    if best:
        return list(best[1]["default_qpos"]), best[0]
    return list(DEFAULT_QPOS), "g1-isaac-sim/g1_config.py DEFAULT_QPOS"
