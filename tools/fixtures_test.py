"""Physics test of the hospital fixtures in Isaac, whole hospital (headless, no robot, no ROS):

    flock /tmp/isaac_gpu.lock ./g1-isaac-sim/run.sh --fixtures TIER --python g1-isaac-sim/tools/fixtures_test.py [options]

Opens usd/hospital_g1_robot.usd with /World/G1 deactivated, steps PhysX at 500 Hz with the FixtureController
(g1-isaac-sim/fixtures_isaac.py) at the tier the stage was built with (HG1_FIXTURES at build time) and checks:
  1. the views: one per group, Doors n x 2, Doors1 n x 1, Lifts n x 4, Calls n x 1, as spec["groups"];
     held-open doors start and stay at +-90 deg on their magnets; every other door starts shut;
  2. an 80 N push opens every kept-shut leaf (wing stores and exit, stair doors, hazard rooms, D07/D34 exits) past
     30 deg in its swing direction, and the closer shuts it again;
  3. the fire alarm releases every magnet and the closers shut every door (time to < 2 deg, about 10 s), the lifts
     of both banks go on fire recall; the doors stay shut after the alarm reset;
  4. the double-swing building leaves (W/E_Compartment) open the other way too and self-close;
  5. snag test: the net contact force on every door leaf stays ~0 N over all the sweeps above (the leaves are
     filtered only against the Static collision group: building jambs, screens and walls must not touch them);
  6. the landing call buttons of both banks (LobbyCall, MainCall) pressed by a 6 N force on the button body open a
     car of their own bank (~3 s), it dwells (~7 s), then closes; the MainLift1 car "Open" button opens MainLift1;
  7. an obstacle in the Lift2 and MainLift2 doorways holds the doors open through the light curtain; removed, they
     close;
  8. the lidar (Warp, static mesh + moving fixture boxes) sees the closed E_Compartment door (open on its magnets
     before the alarm), the closed CrossCorridor door and the closed D34_Exit; lidar cost per scan is measured;
  9. reset puts every door and lift back to the initial state.
Prints one PASS/FAIL line per check, timings (fixture step and physics step, mean and p99 ms) and a JSON summary
(also written to --out); exits 1 on any failure.
Options: --no-building-filter removes the asset frames/thresholds from the Static collision group (snag experiment).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, SIM)

ap = argparse.ArgumentParser()
ap.add_argument("--stage", default=os.path.join(SIM, "usd", "hospital_g1_robot.usd"))
ap.add_argument("--no-building-filter", action="store_true",
                help="Static collision group = fixtures Static + HG1_Colliders only (no asset frames/thresholds)")
ap.add_argument("--contact-limit", type=float, default=1.0, help="max net contact force on a leaf (N) for PASS")
ap.add_argument("--out", default=None, help="write the JSON summary here")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "disable_viewport_updates": True})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import fixtures_isaac as FI  # noqa: E402

DT = 0.002
ctx = omni.usd.get_context()
ctx.open_stage(args.stage)
while ctx.get_stage_loading_status()[2] > 0 or not ctx.get_stage():
    app.update()
stage = ctx.get_stage()
TIER = FI.stage_tier(stage)
F_, SPEC_ = FI.load_spec(TIER)
ox, oy, oz = SPEC_["wing_origin"]
OBST = {"Lift2": "/World/TestObstacle_Lift2", "MainLift2": "/World/TestObstacle_MainLift2"}
with Usd.EditContext(stage, stage.GetSessionLayer()):
    stage.GetPrimAtPath("/World/G1").SetActive(False)
    # freeze third-party rigid bodies as run_sim.py does (fixtures stay dynamic)
    roots = set()
    it = iter(Usd.PrimRange(stage.GetPrimAtPath("/World"), Usd.TraverseInstanceProxies()))
    for p in it:
        if str(p.GetPath()).startswith("/World/Fixtures"):
            it.PruneChildren()
            continue
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            q = p
            while q.IsInstanceProxy():
                q = q.GetParent()
            roots.add(q.GetPath())
    for r in roots:
        if stage.GetPrimAtPath(r).IsInstance():
            stage.GetPrimAtPath(r).SetInstanceable(False)
    for r in roots:
        for p in Usd.PrimRange(stage.GetPrimAtPath(r)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            if p.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
    # obstacles for the light-curtain check: static boxes in the lift doorways (lobby side, z 0.3..1.2)
    for lname, path in OBST.items():
        lf = next((l for l in SPEC_["lifts"] if l["name"] == lname), None)
        if lf is None:
            continue
        obs = UsdGeom.Cube.Define(stage, path)
        obs.CreateSizeAttr(1.0)
        obs.AddTranslateOp().Set(Gf.Vec3d(lf["front_x"] + 0.06 + ox, lf["center_y"] + oy, 0.75))
        obs.AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.9))
        UsdPhysics.CollisionAPI.Apply(obs.GetPrim())
        obs.GetPrim().SetActive(False)  # switched on for check 7
    # contact reports on every door leaf (snag test)
    leaf_paths = []
    for d in SPEC_["doors"]:
        for i, _lf in enumerate(d["leaves"]):
            lp = f"/World/Fixtures/{d['group']}/{d['name']}/Leaf{i}"
            cr = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(lp))
            cr.CreateThresholdAttr(0.0)
            leaf_paths.append((d["name"], i, lp))
    grp = stage.GetPrimAtPath("/World/Fixtures/CollisionGroups/Static")
    inc = UsdPhysics.CollisionGroup(grp).GetCollidersCollectionAPI().GetIncludesRel()
    BUILDING_FILTER = [str(t) for t in inc.GetTargets() if str(t).startswith("/World/Hospital")]
    if args.no_building_filter:
        inc.SetTargets([t for t in inc.GetTargets() if not str(t).startswith("/World/Hospital")])
        print(f"[fixtures_test] Static group without the asset frames/thresholds: {inc.GetTargets()}", flush=True)

from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

SimulationManager.set_physics_sim_device("cpu")
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Scene):
        SimulationManager.set_default_physics_scene(str(p.GetPath()))
        break
SimulationManager.set_physics_dt(DT)
settings = carb.settings.get_settings()
settings.set_bool("/physics/updateToUsd", False)
settings.set_bool("/physics/updateVelocitiesToUsd", False)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()
settings.set_bool("/app/player/playSimulations", False)
SimulationManager.initialize_physics()
import omni.physics.core  # noqa: E402
import omni.physics.tensors  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.core.simulation_manager", False)
phys = omni.physics.core.get_physics_simulation_interface()
view = omni.physics.tensors.create_simulation_view("numpy", stage_id=ctx.get_stage_id())
view.set_subspace_roots("/")
fx = FI.Fixtures(stage, view)
print("[fixtures_test] views:", fx.describe(), flush=True)
contact = view.create_rigid_contact_view([p for _n, _i, p in leaf_paths])
LEAF_ROW = {(n, i): k for k, (n, i, _p) in enumerate(leaf_paths)}
contact_max = np.zeros(len(leaf_paths))

t = [0.0]
results, summary = [], {"tier": fx.tier, "building_filter": [] if args.no_building_filter else BUILDING_FILTER}
step_ms, phys_ms = [], []


def run(seconds, cb=None):
    n = int(round(seconds / DT))
    for _k in range(n):
        t0 = time.perf_counter()
        fx.step(DT)
        t1 = time.perf_counter()
        step_ms.append((t1 - t0) * 1e3)
        if cb is not None:
            cb()
        t2 = time.perf_counter()
        phys.simulate(DT, t[0])
        phys_ms.append((time.perf_counter() - t2) * 1e3)
        t[0] += DT
        f = np.asarray(contact.get_net_contact_forces(DT)).reshape(-1, 3)
        np.maximum(contact_max, np.linalg.norm(f, axis=1), out=contact_max)


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"[fixtures_test] {'PASS' if ok else 'FAIL'} {name} {info}", flush=True)


def deg(j):
    return math.degrees(fx.q[j])


def wait_until(pred, timeout, cb=None):
    t0 = t[0]
    while t[0] - t0 < timeout:
        run(0.02, cb)
        if pred():
            return round(t[0] - t0, 2)
    return None


DOORS = {d["name"]: d for d in fx.spec["doors"]}
HELD = [d["name"] for d in fx.spec["doors"] if any(lf["hold"] is not None for lf in d["leaves"])]
SHUT = [d["name"] for d in fx.spec["doors"] if d["name"] not in HELD]
BUILDING = [d["name"] for d in fx.spec["doors"] if d.get("source_frame") == "world"]


def leaf_closed(lf):
    return abs(fx.q[lf["joint"]]) < math.radians(2.0)


def doors_closed(names=None):
    return all(leaf_closed(lf) for n in (names or DOORS) for lf in DOORS[n]["leaves"])


# 1. views and initial state -----------------------------------------------------------------------------------
groups = fx.spec["groups"]
want_views = {g: (len(groups[g]), fx.spec["group_dof"][g]) for g in groups if groups[g]}
check("1 one homogeneous view per group", fx.groups == want_views, f"views {fx.groups}, spec {want_views}")
summary["views"] = {g: list(v) for g, v in fx.groups.items()}
run(0.5)
want = {k: math.degrees(v) for k, v in fx.ctl.initial_q().items() if "leaf" in k}
held = {k: round(deg(k), 1) for k in want}
check("1 initial door angles (held at +-90, the rest shut)", all(abs(held[k] - want[k]) < 3.0 for k in held),
      {k: v for k, v in held.items() if abs(v - want[k]) >= 3.0})
run(2.0)
check("1 held doors stay on their magnets for 2 s", all(abs(deg(k) - want[k]) < 3.0 for k in want),
      {k: round(deg(k), 1) for k in want if abs(deg(k) - want[k]) >= 3.0})
st0 = fx.ctl.state(fx.q)["doors"]
check(f"1 {len(st0)} doors in the state; {len(HELD)} held open, {len(SHUT)} shut",
      sorted(st0) == sorted(DOORS) and all(st0[n]["held_open"] for n in HELD) and all(st0[n]["closed"] for n in SHUT),
      {"held": HELD, "shut": SHUT})
summary["doors"] = {"held": HELD, "shut": SHUT}


# 2. push opens every kept-shut leaf and the closer shuts it again ----------------------------------------------
def leaf_target(door, i):
    """(view, row, link index) of a door leaf in its group view."""
    for v, names, _idx, _flat in fx.views:
        for r, row in enumerate(names):
            if f"fx_{door}_leaf{i}" in row:
                return v, r, list(v.shared_metatype.link_names).index(f"Leaf{i}")
    raise KeyError(door)


def pusher(leaves, newton=80.0):
    """Callback: a force at each leaf's centre of mass, normal to the leaf, towards `sign` (80 N x ~0.5 m)."""
    per_view = {}
    for door, i, sign in leaves:
        v, r, li = leaf_target(door, i)
        per_view.setdefault(id(v), (v, []))[1].append((r, li, DOORS[door]["leaves"][i], sign))
    amax = {(d, i): 0.0 for d, i, _s in leaves}

    def cb():
        for v, items in per_view.values():
            f = np.zeros((v.count, len(v.shared_metatype.link_names), 3), np.float32)
            for r, li, lf, sign in items:
                a = fx.q[lf["joint"]]
                dx, dy = F_._rot(lf["closed_dir"], a)
                f[r, li] = (-dy * sign * newton, dx * sign * newton, 0.0)
                key = (lf["joint"].split("_leaf")[0][3:], int(lf["joint"][-1]))
                amax[key] = max(amax[key], abs(a))
            v.apply_forces_and_torques_at_position(f, np.zeros_like(f), None, np.arange(v.count, dtype=np.int32), True)
    return cb, amax


def push_and_release(leaves, label, seconds=2.5):
    cb, amax = pusher(leaves)
    run(seconds, cb)
    end = {(d, i): fx.q[DOORS[d]["leaves"][i]["joint"]] for d, i, _s in leaves}
    t0 = t[0]
    close_t = {}
    while t[0] - t0 < 15.0 and len(close_t) < len(leaves):
        run(0.02)
        for d, i, _s in leaves:
            if (d, i) not in close_t and leaf_closed(DOORS[d]["leaves"][i]):
                close_t[(d, i)] = round(t[0] - t0, 2)
    rows = {}
    for d, i, s in leaves:
        lf = DOORS[d]["leaves"][i]
        lo, hi = lf["range"]
        ok = (amax[(d, i)] > math.radians(30) and lo - 0.02 <= end[(d, i)] <= hi + 0.02
              and s * end[(d, i)] > 0 and (d, i) in close_t)
        rows[f"{d}.leaf{i}"] = {"max_deg": round(math.degrees(amax[(d, i)]), 1),
                                "end_deg": round(math.degrees(end[(d, i)]), 1), "close_s": close_t.get((d, i)),
                                "ok": ok}
    bad = {k: v for k, v in rows.items() if not v["ok"]}
    check(f"{label} ({len(leaves)} leaves)", not bad, bad or
          f"max {min(r['max_deg'] for r in rows.values())}..{max(r['max_deg'] for r in rows.values())} deg, "
          f"closed in <= {max(r['close_s'] for r in rows.values())} s")
    return rows


def open_sign(lf):
    lo, hi = lf["range"]
    return 1.0 if hi > 0 else -1.0


shut_leaves = [(d, i, open_sign(lf)) for d in SHUT for i, lf in enumerate(DOORS[d]["leaves"]) if not lf["bolted"]]
summary["push_kept_shut"] = push_and_release(shut_leaves, "2 push opens every kept-shut leaf and it self-closes")

# 8a. lidar (moving fixture boxes): rays at known closed-door planes ---------------------------------------------
from lidar_warp import WarpLidar, _cast2  # noqa: E402
import warp as wp  # noqa: E402

t_l = time.perf_counter()
lid = WarpLidar(stage, exclude_roots=["/World/G1"] + list(OBST.values()), device="cuda:0")
summary["lidar_build_s"] = round(time.perf_counter() - t_l, 1)
print(f"[fixtures_test] lidar: {lid.describe()} (built in {summary['lidar_build_s']} s)", flush=True)
EYE = np.eye(3, dtype=np.float32).ravel()


def ray(o_world, d):
    lid.set_dynamic_boxes(*fx.boxes())
    dirs = wp.array(np.array([d], np.float32), dtype=wp.vec3, device=lid.device)
    out = wp.zeros(1, dtype=float, device=lid.device)
    wp.launch(_cast2, dim=1, inputs=[lid.mesh.id, lid.dyn.id, wp.vec3(*o_world), wp.mat33(*EYE), dirs, 40.0, out],
              device=lid.device)
    return float(out.numpy()[0])


def door_plane_dist(door, origin, d):
    """Distance from origin along d (unit, axis-aligned) to the near face of the closed door's leaves."""
    lf = DOORS[door]["leaves"][0]
    hx, hy = lf["hinge"][0] + ox, lf["hinge"][1] + oy
    t2 = lf["thickness"] / 2
    return (hx - origin[0]) * d[0] - t2 if abs(d[0]) > 0.5 else (hy - origin[1]) * d[1] - t2


RAYS = {}
if "E_Compartment" in DOORS:  # from the ward entrance (C3), westwards through the C2|C3 doorway
    RAYS["E_Compartment"] = ((11.21 + 1.30, 7.255, 1.0), (-1.0, 0.0, 0.0))
if "CrossCorridor" in DOORS:  # wing corridor centre, eastwards
    RAYS["CrossCorridor"] = ((1.5 + ox, -8.0 + oy, 1.0), (1.0, 0.0, 0.0))
if "D34_Exit" in DOORS:  # NE stair hall, eastwards to the final exit
    RAYS["D34_Exit"] = ((20.81 - 1.20, 30.787, 1.0), (1.0, 0.0, 0.0))
r_open = {n: ray(o, d) for n, (o, d) in RAYS.items()}

# 3. fire alarm -------------------------------------------------------------------------------------------------
fx.set_alarm(True)
t0 = t[0]
close_t = {}
while t[0] - t0 < 20.0 and len(close_t) < len(DOORS):
    run(0.02)
    for n in DOORS:
        if n not in close_t and doors_closed([n]):
            close_t[n] = round(t[0] - t0, 2)
tc = max(close_t.values()) if len(close_t) == len(DOORS) else None
summary["alarm_close_s"] = {"all": tc, "per_door": close_t}
check("3 alarm closes every fire door (< 2 deg) in ~10 s", tc is not None and tc < 12.0,
      f"all in {tc} s; slowest {max(close_t, key=close_t.get) if close_t else None}")
check("3 fire recall opens the lifts of both banks", all(l.state in ("opening", "open") for l in fx.ctl.lifts.values()),
      {n: l.state for n, l in fx.ctl.lifts.items()})
r_closed = {n: ray(o, d) for n, (o, d) in RAYS.items()}
summary["lidar_rays"] = {}
for n, (o, d) in RAYS.items():
    want_d = door_plane_dist(n, o, d)
    ok = r_closed[n] > 0 and abs(r_closed[n] - want_d) < 0.05
    if n in HELD:
        ok = ok and (r_open[n] < 0 or r_open[n] > want_d + 1.0)
    summary["lidar_rays"][n] = {"open_m": round(r_open[n], 3), "closed_m": round(r_closed[n], 3), "door_m": round(want_d, 3)}
    check(f"8 lidar sees the closed {n} door", ok,
          f"before alarm {r_open[n]:.2f} m, closed {r_closed[n]:.2f} m (door face {want_d:.2f} m)")

# 4. double-swing building leaves open the other way and self-close (alarm on: no magnets) ----------------------
dbl = [(d, i, -1.0) for d in BUILDING for i, lf in enumerate(DOORS[d]["leaves"]) if lf["range"][0] < 0 < lf["range"][1]]
if dbl:
    summary["push_reverse"] = push_and_release(dbl, "4 double-swing building leaves open the other way and self-close")
fx.set_alarm(False)
run(1.0)
check("3 doors stay shut after the alarm reset", doors_closed())
t_rec = wait_until(lambda: all(l.state == "closed" for l in fx.ctl.lifts.values()), 20.0)
check("3 lifts close after the recall", t_rec is not None, f"in {t_rec} s")

# 5. snag test: net contact force on every leaf over all the sweeps ----------------------------------------------
cmax = {f"{n}.leaf{i}": round(float(contact_max[k]), 2) for (n, i), k in LEAF_ROW.items()}
summary["leaf_contact_max_N"] = cmax
bad = {k: v for k, v in cmax.items() if v > args.contact_limit}
check(f"5 no leaf touches the building over the sweeps (net contact < {args.contact_limit} N)", not bad,
      bad or f"max {max(cmax.values()):.2f} N over {len(cmax)} leaves")


# 6. landing calls of both banks by a force on the button body; car Open button -----------------------------------
def lift(name):
    return fx.ctl.lifts[name].state


def button_target(joint):
    for v, names, _idx, _flat in fx.views:
        for r, row in enumerate(names):
            if joint in row:
                return v, r
    raise KeyError(joint)


def press_body(joints, newton=6.0, seconds=0.3):
    """Push button bodies along their press axis; returns the max travel (m) per joint."""
    items = []
    for j in joints:
        v, r = button_target(j)
        b = next(b for b in fx.ctl.buttons if b["joint"] == j)
        link = {"call": "Btn", "open": "BtnOpen", "close": "BtnClose"}[b["kind"]]
        items.append((v, r, list(v.shared_metatype.link_names).index(link), b["axis"], j))
    smax = {j: 0.0 for j in joints}

    def cb():
        fs = {}
        for v, r, li, ax, j in items:
            f = fs.setdefault(id(v), (v, np.zeros((v.count, len(v.shared_metatype.link_names), 3), np.float32)))[1]
            f[r, li] = [a * newton for a in ax]
            smax[j] = max(smax[j], fx.q[j])
        for v, f in fs.values():
            v.apply_forces_and_torques_at_position(f, np.zeros_like(f), None, np.arange(v.count, dtype=np.int32), True)
    run(seconds, cb)
    run(0.3)
    return smax


calls = [c["joint"] for c in fx.spec["calls"]]
smax = press_body(calls)
check("6 6 N pushes move every call button past 2.5 mm", all(v > 0.0025 for v in smax.values()),
      {j: round(v * 1000, 2) for j, v in smax.items()})
summary["call_buttons"] = {}
for c in fx.spec["calls"]:
    bank = [n for n in c["lifts"]]
    called = next((n for n in bank if lift(n) in ("opening", "open")), None)
    summary["call_buttons"][c["joint"]] = {"lift": called}
    check(f"6 {c['joint'][3:]} opens a car of its own bank", called is not None, {n: lift(n) for n in bank})
called = {c["joint"]: summary["call_buttons"][c["joint"]]["lift"] for c in fx.spec["calls"]}
t0 = t[0] - 0.6
ev = {j: {} for j in called}


def track():
    for j, n in called.items():
        if n is None:
            continue
        s = lift(n)
        if s == "open" and "open_s" not in ev[j]:
            ev[j]["open_s"] = round(t[0] - t0, 2)
        if s == "closing" and "open_s" in ev[j] and "dwell_s" not in ev[j]:
            ev[j]["dwell_s"] = round(t[0] - t0 - ev[j]["open_s"], 2)
        if s == "closed" and "dwell_s" in ev[j] and "close_s" not in ev[j]:
            ev[j]["close_s"] = round(t[0] - t0 - ev[j]["open_s"] - ev[j]["dwell_s"], 2)


wait_until(lambda: (track(), all("close_s" in e for j, e in ev.items() if called[j]))[1], 25.0)
for j, e in ev.items():
    summary["call_buttons"][j].update(e)
    ok = (2.5 < e.get("open_s", 0) < 4.5 and 6.5 < e.get("dwell_s", 0) < 8.0 and 0 < e.get("close_s", 99) < 5.0)
    check(f"6 {j[3:]}: car opens (~3 s), dwells (~7 s), closes", ok, f"{called[j]} {e}")
if "MainLift1" in fx.ctl.lifts:
    wait_until(lambda: lift("MainLift1") == "closed", 10.0)
    sm = press_body(["fx_MainLift1_Open"])
    check("6 6 N push on the MainLift1 car Open button opens MainLift1",
          sm["fx_MainLift1_Open"] > 0.0025 and lift("MainLift1") in ("opening", "open"),
          f"{sm['fx_MainLift1_Open'] * 1000:.2f} mm, {lift('MainLift1')}")
    wait_until(lambda: lift("MainLift1") == "closed", 20.0)

# 7. light curtain on both banks ---------------------------------------------------------------------------------
cur = [n for n in OBST if n in fx.ctl.lifts]
for n in cur:
    fx.press(f"{n}_Open")
wait_until(lambda: all(lift(n) == "open" for n in cur), 6.0)
with Usd.EditContext(stage, stage.GetSessionLayer()):
    for n in cur:
        stage.GetPrimAtPath(OBST[n]).SetActive(True)  # obstacle appears in the open doorway
for _ in range(5):
    app.update()
still = wait_until(lambda: any(lift(n) != "open" for n in cur), 12.0)
check(f"7 light curtain holds {', '.join(cur)} open with an obstacle (12 s)", still is None,
      {n: lift(n) for n in cur})
with Usd.EditContext(stage, stage.GetSessionLayer()):
    for n in cur:
        stage.GetPrimAtPath(OBST[n]).SetActive(False)
for _ in range(5):
    app.update()
t2 = wait_until(lambda: all(lift(n) == "closed" for n in cur), 12.0)
check(f"7 {', '.join(cur)} close once the doorways are clear", t2 is not None, f"in {t2} s")

# 8b. lidar cost (full MID-360 pattern + 720-ray scan, warp on cuda:0) ----------------------------------------------
T = np.eye(4)
T[:3, 3] = (8.0, 2.0, 1.2)  # main lift lobby, 1.2 m: the lift doors and the lobby door are in view


def timed(fn, n=60):
    ts = []
    for _ in range(n):
        t0_ = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0_) * 1e3)
    return round(float(np.mean(ts[5:])), 3), round(float(np.percentile(ts[5:], 99)), 3)


dyn = lid.dyn
lid.dyn = None
static_scan = timed(lambda: (lid.cloud(T), lid.scan2d(T)))
lid.dyn = dyn
boxes_t = timed(lambda: lid.set_dynamic_boxes(*fx.boxes()))
J0 = DOORS[next(iter(DOORS))]["leaves"][0]["joint"]


def _moved():
    fx.q[J0] += 1e-5  # a leaf moves: the boxes change, so the moving mesh is rewritten and refitted
    lid.set_dynamic_boxes(*fx.boxes())


move_t = timed(_moved)
dyn_scan = timed(lambda: (lid.cloud(T), lid.scan2d(T)))
summary["lidar_ms"] = {"moving_boxes": lid.n_dyn, "static_only_cloud+scan": static_scan,
                       "with_moving_cloud+scan": dyn_scan, "set_dynamic_boxes_unchanged": boxes_t,
                       "set_dynamic_boxes_moved": move_t}
print(f"[fixtures_test] lidar ms (mean, p99): {summary['lidar_ms']}", flush=True)

# 9. reset --------------------------------------------------------------------------------------------------------
fx.set_alarm(True)
run(0.5)
fx.reset()
run(0.5)
st1 = fx.ctl.state(fx.q)
check("9 reset: alarm off, held doors open, the rest shut, lifts closed",
      not st1["fire_alarm"] and all(st1["doors"][n]["held_open"] for n in HELD)
      and all(st1["doors"][n]["closed"] for n in SHUT) and all(l["state"] == "closed" for l in st1["lifts"].values()),
      {n: (v["held_open"], v["closed"]) for n, v in st1["doors"].items()})

summary["events"] = fx.ctl.events[-12:]
summary["fixtures_step_ms"] = {"mean": round(float(np.mean(step_ms)), 4), "p99": round(float(np.percentile(step_ms, 99)), 4),
                               "max": round(float(np.max(step_ms)), 3)}
summary["physics_step_ms"] = {"mean": round(float(np.mean(phys_ms)), 4), "p99": round(float(np.percentile(phys_ms, 99)), 4),
                              "max": round(float(np.max(phys_ms)), 3)}
summary["sim_s"] = round(t[0], 1)
print(f"[fixtures_test] fixtures step ms {summary['fixtures_step_ms']}, physics step ms {summary['physics_step_ms']} "
      f"(no robot), {summary['sim_s']} s simulated", flush=True)
print("[fixtures_test] summary " + json.dumps(summary), flush=True)
if args.out:
    with open(args.out, "w") as f:
        json.dump(summary, f, indent=1)
ok = all(results)
print(f"[fixtures_test] {'ALL PASS' if ok else 'FAILURES'} ({sum(results)}/{len(results)})", flush=True)
timeline.stop()
app.close()
sys.exit(0 if ok else 1)
