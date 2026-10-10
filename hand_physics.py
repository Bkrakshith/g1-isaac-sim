"""Revo2 hand physics for grasping in PhysX (used by run_sim.py and tools/hand_grasp_test.py).

The imported Revo2 links are very light: 9-50 g finger segments and 2 g fingertip "touch" pads (separate links on
fixed joints) with principal inertias down to 3e-9 kg m^2. With the articulation's 4/1 solver iterations, finger
contacts made the solver blow up when the robot fell on its hands, so run_sim disabled finger collisions. `apply`
makes them usable, authored in the session layer (the robot USD is untouched):

  * mass floor 5 g and principal-inertia floor 1e-6 kg m^2 on every hand link (touch pads 2 g / 3e-9 -> 5 g / 1e-6;
    finger segments ~1e-7 -> 1e-6). Adds < 30 g per hand; finger dynamics are dominated by the drives anyway.
  * finger / palm colliders on, contact offset 2 mm, rest offset 0, max depenetration velocity 0.5 m/s on hand links
    (a deep penetration is pushed out gently instead of launching the link).
  * finger joints: max joint velocity 10 rad/s (the real Revo2 closes in ~0.5 s, i.e. < 4 rad/s).
  * articulation solver iterations 4/1 -> 16/2 (position/velocity).

Measured effect: g1-isaac-sim/README.md ("Grasp physics"), tools/hand_grasp_test.py.
"""
from __future__ import annotations

FINGER_KEYS = ("_thumb_", "_index_", "_middle_", "_ring_", "_pinky_")
HAND_KEYS = FINGER_KEYS + ("_base_link", "_base2_link")

DEFAULTS = dict(mass_floor=0.005, inertia_floor=1e-6, contact_offset=0.002, rest_offset=0.0,
                max_depenetration=0.5, finger_max_vel_deg=573.0, solver_pos=16, solver_vel=2)


def is_hand(path: str) -> bool:
    return any(k in path for k in HAND_KEYS)


def apply(stage, robot_root: str, cfg: dict | None = None, collisions: bool = True, log=print):
    """Author the hand fixes under `robot_root` in the current edit target (use the session layer)."""
    from pxr import Gf, Sdf, Usd, UsdPhysics

    c = dict(DEFAULTS, **(cfg or {}))
    n_body = n_col = n_joint = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        path = str(p.GetPath())
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(c["solver_pos"])
            p.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(c["solver_vel"])
        name = p.GetName()
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and is_hand(name):
            ma = UsdPhysics.MassAPI(p)
            m = ma.GetMassAttr().Get() or 0.0
            if m < c["mass_floor"]:
                ma.CreateMassAttr(c["mass_floor"])
            I = ma.GetDiagonalInertiaAttr().Get()
            if I is not None:
                ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*[max(float(v), c["inertia_floor"]) for v in I]))
            if "PhysxRigidBodyAPI" not in p.GetAppliedSchemas():
                p.AddAppliedSchema("PhysxRigidBodyAPI")
            p.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(c["max_depenetration"])
            n_body += 1
        if p.HasAPI(UsdPhysics.CollisionAPI) and is_hand(path):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(bool(collisions))
            if "PhysxCollisionAPI" not in p.GetAppliedSchemas():
                p.AddAppliedSchema("PhysxCollisionAPI")
            p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(c["contact_offset"])
            p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(c["rest_offset"])
            n_col += 1
        if p.IsA(UsdPhysics.RevoluteJoint) and any(k in name for k in FINGER_KEYS):
            p.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(c["finger_max_vel_deg"])
            n_joint += 1
    log(f"[hand_physics] {n_body} hand links (mass >= {c['mass_floor']} kg, I >= {c['inertia_floor']:g}), "
        f"{n_col} hand colliders {'ON' if collisions else 'off'} (contact offset {c['contact_offset']} m), "
        f"{n_joint} finger joints, articulation iterations {c['solver_pos']}/{c['solver_vel']}")
    return dict(links=n_body, colliders=n_col, joints=n_joint, cfg=c)
