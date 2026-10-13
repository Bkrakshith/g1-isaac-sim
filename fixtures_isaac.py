"""Hospital fixtures (fire doors, bed lifts, lift buttons) in the Isaac backend, for the whole hospital.

The articulations come from hospital-sim-env/isaac/usd/fixtures.usd: one tensor-view group per spec group,
/World/Fixtures/{Doors,Doors1,Lifts,Calls}/* (Doors: 2-leaf doorsets, Doors1: 1-leaf doorsets, Lifts: 2 panels +
Open + Close, Calls: landing call buttons). Every articulation in a group has the same links and joints, so each
group is one homogeneous view.
The behaviour is hospital-sim-env/fixtures/hospital_fixtures.py: one FixtureController, the same code as the
MuJoCo backend. Every physics step, Fixtures.step() reads the fixture joint positions and velocities through
omni.physics.tensors, runs the controller and writes its efforts as joint forces (N*m for hinges, N for slides).
The lift light curtains (every lift of every bank) are PhysX scene-query rays (curtain_rays() of the spec).

Fixture tier (hospital_fixtures.TIERS: wing, tier1, tier2 = default, full). The stage records the tier it was built
with (customData "hg1:tier" on /World/Fixtures). The run time uses that tier. When $HG1_FIXTURES names another tier,
a warning tells you to rebuild the stages with the same HG1_FIXTURES.

The pure functions at the top (load_spec, moving_boxes) need no Isaac; tools/check_routes.py uses them too.
"""

from __future__ import annotations

import importlib.util
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
FIXTURES_PY = os.path.join(REPO, "hospital-sim-env", "fixtures", "hospital_fixtures.py")
LAYOUT_PY = os.path.join(REPO, "hospital-sim-env", "isaac", "scene", "layout.py")
ROOT = "/World/Fixtures"
GROUPS = ("Doors", "Doors1", "Lifts", "Calls")
CURTAIN_EVERY = 5  # steps between light-curtain ray casts (100 Hz at 500 Hz physics)
BTN_PLATE, BTN_CAP = 0.004, 0.012  # button faceplate and cap depth (m), as hospital-sim-env/isaac/scene/fixtures_usd.py


def _module():
    s = importlib.util.spec_from_file_location("hospital_fixtures", FIXTURES_PY)
    F = importlib.util.module_from_spec(s)
    s.loader.exec_module(F)
    return F


def load_spec(tier=None):
    """(hospital_fixtures module, spec) from the layout. tier None: $HG1_FIXTURES, else the default (tier2)."""
    F = _module()
    return F, F.build_spec(F.load_layout(LAYOUT_PY), tier)


def stage_tier(stage):
    """Fixture tier the stage was built with (customData "hg1:tier" on /World/Fixtures), or None."""
    p = stage.GetPrimAtPath(ROOT)
    return (p.GetCustomDataByKey("hg1:tier") or None) if p else None


def moving_boxes(F, spec, q, buttons=False):
    """World boxes (centres n x 3, yaws n, sizes n x 3) of every door leaf (Doors and Doors1) and lift door panel
    at positions q; with buttons=True also every lift car button and landing call button cap."""
    ox, oy, oz = spec["wing_origin"]
    C, Y, S = [], [], []
    for d in spec["doors"]:
        for lf in d["leaves"]:
            c, yaw, size = F.leaf_box(lf, q.get(lf["joint"], 0.0))
            C.append((c[0] + ox, c[1] + oy, c[2] + oz))
            Y.append(yaw)
            S.append(size)
    for lift in spec["lifts"]:
        for p in lift["panels"]:
            s = q.get(p["joint"], 0.0)
            c = [cc + a * s for cc, a in zip(p["closed_center"], p["axis"])]
            C.append((c[0] + ox, c[1] + oy, c[2] + oz))
            Y.append(0.0)
            S.append(tuple(p["size"]))
    if buttons:
        size = spec["button"]["size"] * 0.9
        for b in [b for lift in spec["lifts"] for b in lift["buttons"]] + list(spec["calls"]):
            s, ax = q.get(b["joint"], 0.0), b["axis"]
            c = [p - a * (BTN_PLATE + BTN_CAP / 2) + a * s for p, a in zip(b["pos"], ax)]
            C.append((c[0] + ox, c[1] + oy, c[2] + oz))
            Y.append(0.0)
            S.append(tuple(BTN_CAP if abs(a) > 0.5 else size for a in ax))
    return np.asarray(C, float), np.asarray(Y, float), np.asarray(S, float)


class Fixtures:
    """Tensor views + controller. Create after physics is initialised (needs the simulation view)."""

    def __init__(self, stage, sim_view, log=print, tier=None):
        from pxr import UsdPhysics

        built = stage_tier(stage)
        env = os.environ.get("HG1_FIXTURES")
        if tier is None:
            tier = built or env
        if built and tier != built:
            log(f"fixtures: WARNING the stage has tier {built!r}, asked for {tier!r}: using {built!r}. Rebuild the stages "
                f"with HG1_FIXTURES={tier} (hospital-sim-env/isaac build_hospital.py, then ./g1-isaac-sim/run.sh --stage)")
            tier = built
        elif env and built and env != built:
            log(f"fixtures: WARNING HG1_FIXTURES={env} but the stage was built with {built!r}: using {built!r}")
        self.F, self.spec = load_spec(tier)
        self.tier = self.spec.get("tier", "wing")
        self.ctl = self.F.FixtureController(self.spec)
        self.names = self.F.joint_names(self.spec)
        self.log = log
        # [view, dof names per (articulation, dof) as a 2-D list, index array, flat names (row-major)]
        self.views = []
        self.groups = {}
        for g in GROUPS:
            gp = stage.GetPrimAtPath(f"{ROOT}/{g}")
            if not gp:
                continue
            paths = [str(p.GetPath()) for p in gp.GetChildren() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
            if not paths:
                continue
            v = sim_view.create_articulation_view(f"{ROOT}/{g}/*")
            if v is not None and v.count == len(paths) and v.is_homogeneous:
                self._add(stage, v)
                self.groups[g] = (v.count, v.max_dofs)
            else:  # fall back to one view per articulation (slower; the stage is not built as the spec groups)
                log(f"fixtures: {g}/* view not homogeneous, one view per articulation")
                for p in paths:
                    self._add(stage, sim_view.create_articulation_view(p))
                self.groups[g] = (len(paths), None)
        found = {n for _v, names, _i, _f in self.views for row in names for n in row}
        missing = [n for n in self.names if n not in found]
        if missing:
            raise RuntimeError(f"fixture joints missing in the stage (tier {self.tier}): {missing}")
        self.q = self.ctl.initial_q()
        self.qd = {n: 0.0 for n in self.names}
        self.blocked = {}
        self._k = 0
        self._sq = None
        self.reset()

    def _add(self, stage, v):
        """DOF (articulation i, dof k) -> spec joint name, from the joint prim's customData "hg1:joint"."""
        names = []
        for row in v.dof_paths:
            names.append([stage.GetPrimAtPath(str(p)).GetCustomDataByKey("hg1:joint") or str(p).rsplit("/", 1)[-1]
                          for p in row])
        flat = [n for row in names for n in row]
        self.views.append((v, names, np.arange(v.count, dtype=np.int32), flat))

    def describe(self):
        return f"tier {self.tier}; " + "; ".join(
            f"{g} {n} x {d} DOF" for g, (n, d) in self.groups.items()) + f"; {len(self.names)} joints"

    # -- inputs (ROS callbacks)
    def set_alarm(self, on):
        self.ctl.set_alarm(on)

    def press(self, name):
        self.ctl.press(name)

    def buttons(self):
        """Short names of every button that press() accepts (lift car buttons and landing calls)."""
        return [b["joint"][len(self.F.PREFIX):] for b in self.ctl.buttons]

    def reset(self):
        """Joint positions to initial_q (held doors open, the rest closed), zero velocity, controller reset."""
        self.ctl.reset()
        q0 = self.ctl.initial_q()
        for v, names, idx, flat in self.views:
            pos = np.asarray([q0.get(n, 0.0) for n in flat], np.float32).reshape(v.count, v.max_dofs)
            v.set_dof_positions(pos, idx)
            v.set_dof_velocities(np.zeros_like(pos), idx)
            v.set_dof_actuation_forces(np.zeros_like(pos), idx)
        self.q = dict(q0)
        self.qd = {n: 0.0 for n in self.names}
        self.blocked = {}

    # -- light curtain
    def curtain(self, q):
        """lift -> True when a curtain ray between the panels hits anything (PhysX scene query), for every lift."""
        if self._sq is None:
            import carb
            from omni.physx import get_physx_scene_query_interface

            self._sq, self._f3 = get_physx_scene_query_interface(), carb.Float3
        ox, oy, oz = self.spec["wing_origin"]
        out = {}
        for lift in self.spec["lifts"]:
            name = lift["name"]
            if self.ctl.lifts[name].state == "closed":
                out[name] = False
                continue
            hit = False
            for (x, y, z), (dx, dy, dz), length in self.F.curtain_rays(self.spec, name, q):
                if self._sq.raycast_any(self._f3(x + ox, y + oy, z + oz), self._f3(dx, dy, dz), float(length)):
                    hit = True
                    break
            out[name] = hit
        return out

    # -- the step
    def step(self, dt):
        q, qd = self.q, self.qd
        for v, _names, _idx, flat in self.views:
            q.update(zip(flat, np.asarray(v.get_dof_positions()).ravel().tolist()))
            qd.update(zip(flat, np.asarray(v.get_dof_velocities()).ravel().tolist()))
        if self._k % CURTAIN_EVERY == 0:
            self.blocked = self.curtain(q)
        self._k += 1
        tau = self.ctl.step(dt, q, qd, self.blocked)
        for v, _names, idx, flat in self.views:
            f = np.fromiter((tau.get(n, 0.0) for n in flat), np.float32, len(flat)).reshape(v.count, v.max_dofs)
            v.set_dof_actuation_forces(f, idx)

    # -- outputs
    def state_json(self):
        return self.ctl.state_json(self.q)

    def boxes(self, buttons=True):
        return moving_boxes(self.F, self.spec, self.q, buttons=buttons)

    def summary(self):
        return json.dumps({k: v for k, v in self.ctl.state(self.q).items() if k != "events"})
