"""Check the robot's walkable routes against the static collision geometry of the Isaac stage (no Isaac needed):

    HG1_FIXTURES=tier2 uv run --no-project --with usd-core --with warp-lang python g1-isaac-sim/tools/check_routes.py

Run ./g1-isaac-sim/run.sh --stage [--fixtures TIER] first, with the same tier. The script reads the composed robot
stage (usd/hospital_g1_robot.usd): the building and wing meshes that have colliders, the HG1_Colliders boxes of
make_stage.py (walls, static partitions, link junction boxes, exterior) and the static fixture boxes.

Casts rays at knee / hip / head height (0.3, 0.8, 1.3 m) along each route segment, on the centre line and 0.25 m
to each side, and down from 1.5 m every 0.5 m to find the floor. A route is clear when no ray hits within the
segment, every sample point has a floor hit near z = 0, and a 0.5 m footprint at the last waypoint is free (12
headings, END_CLEAR). Each spawn in SPAWNS must have a floor and 0.3 m of
free space around it.

Routes:
  ROUTES        wing-local waypoints (layout.py wing frame), the surgical wing and the 4 m link.
  BUILDING      layout.BUILDING_ROUTES (world frame): the 17 routes of the final design (ward round, ambulance bay ->
                courtyard -> D14 -> resus, resus -> X-ray, walk-in, triage, main lift cars, link, stairs, exits).
  WORLD_ROUTES  world-frame routes added here: nurse-base deliveries, pharmacy -> ward clean utility,
                resus -> surgical corridor -> theatre through the link.
A route is skipped when it passes a door that the current HG1_FIXTURES tier leaves out (NEEDS: the asset's own
closed leaves stay in that doorway) or the main-entrance glass below tier2 (NEEDS_GLASS: the glazing stays).

The moving fixtures are checked at their "route pose": every door with hold-open magnets held open at 90 deg as at
reset (the link Compartment doorset, CrossCorridor, LiftLobby, Theatre, W/E_Compartment, Main_LiftLobby), every
kept-shut or self-closing door opened to 90 deg by the robot (Store, ScrubStore, FinalExit, the stair doors, the
exits D07 and D34, the FD30S hazard-room doors), and every lift car at the landing with its doors open. The narrow
Store leaf is flush-bolted, so the store route uses the 1.2 m leaf only (x = -8.4). The final-exit route ends on the
paved exit path outside (floor = the ExitPathFloor slab of make_stage.py).
"""

import math
import os
import sys

import numpy as np
from pxr import Usd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "hospital-sim-env", "isaac", "scene"))  # layout.py
import layout  # noqa: E402
from fixtures_isaac import load_spec, moving_boxes  # noqa: E402
from lidar_warp import WarpLidar  # noqa: E402
from make_stage import GLASS_TIERS  # noqa: E402

import g1_config as C  # noqa: E402

OX, OY = C.WING_ORIGIN[0], C.WING_ORIGIN[1]


def W(x, y):
    return (x + OX, y + OY)


# Wing-local waypoints (layout.py wing frame). The link is 4 m on the grid (wing x 0..4); its hospital opening is
# wing x 0.10..2.83 and the FD60S doorset leaves are held open against the link walls, so x 1.3 is the centre line.
ROUTES = {
    "lobby -> SE door -> link (4 m) -> corridor": [(1.3, 2.5), (1.3, 0.0), (1.3, -3.0), (1.3, -6.0), (1.3, -8.0)],
    "link west side (x 0.6) -> corridor": [(0.6, 2.5), (0.6, 0.0), (0.6, -3.0), (0.6, -6.0), (0.6, -8.0)],
    "link east side (x 2.2) -> corridor": [(2.2, 2.5), (2.2, 0.0), (2.2, -3.0), (2.2, -6.0), (2.2, -8.0)],
    "corridor west -> east (CrossCorridor fire door)": [(-9.0, -8.0), (13.0, -8.0)],
    "corridor -> pre-op": [(-4.0, -8.0), (-4.0, -6.0), (-4.0, -4.5)],
    "corridor -> PACU": [(8.0, -8.0), (8.0, -6.0), (8.0, -4.5)],
    "corridor -> sterile store (store fire door opened)": [(-8.4, -8.0), (-8.4, -10.0), (-8.4, -12.0)],
    "corridor -> scrub": [(-2.0, -8.0), (-2.0, -10.0), (-2.0, -12.0)],
    "corridor -> theatre": [(8.0, -8.0), (8.0, -10.0), (8.0, -12.0)],
    "scrub -> theatre": [(-2.0, -14.0), (0.0, -14.0), (1.5, -14.0)],
    "scrub -> store (ScrubStore fire door opened)": [(-2.0, -14.0), (-4.0, -14.0), (-5.5, -14.0)],
    "corridor -> final exit -> outside (FinalExit opened)": [(9.0, -8.0), (12.0, -8.0), (12.0, -10.0), (12.0, -13.0),
                                                             (12.0, -20.0)],
    "corridor -> lift lobby -> Lift1 car": [(11.0, -8.0), (14.0, -8.0), (16.5, -8.0), (16.5, -10.0), (18.5, -10.0),
                                            (21.5, -10.0)],
    "corridor -> lift lobby -> Lift2 car": [(11.0, -8.0), (14.0, -8.0), (16.5, -8.0), (16.5, -6.0), (18.5, -6.0),
                                            (21.5, -6.0)],
}

# World-frame routes added to layout.BUILDING_ROUTES. Ward points: corridor centre x 19.0, ward-side doors at
# x 20.83 (east row) and 17.19 (west row); nurse base counter at (17.35, 6.55), staff side to the south.
_RESUS_TO_WARD_ENTRANCE = [(-26.45, 15.0), (-26.45, 12.5), (-23.0, 10.6), (3.0, 10.6), (6.0, 7.25), (11.21, 7.25),
                           (14.0, 7.25)]
WORLD_ROUTES = {
    "nurse base -> B3 bed side": [(17.35, 7.6), (19.0, 8.0), (19.0, 15.8), (20.83, 15.8), (22.4, 15.8)],
    "nurse base -> B4 bed side": [(17.35, 7.6), (19.0, 8.0), (19.0, 8.4), (20.83, 8.4), (22.4, 8.4)],
    "nurse base -> B5 bed side": [(17.35, 7.6), (19.5, 7.6), (19.5, 4.68), (20.83, 4.68), (22.4, 4.68)],
    "nurse base -> ward clean utility": [(17.35, 7.6), (19.0, 8.0), (19.0, 19.5), (17.19, 19.5), (15.6, 19.5)],
    "pharmacy 1 -> main corridor -> E compartment door -> ward clean utility": [
        (-9.1, 7.2), (-9.1, 8.77), (-9.1, 10.6), (3.0, 10.6), (6.0, 7.25), (11.21, 7.25), (14.0, 7.25),
        (19.0, 9.0), (19.0, 19.5), (17.19, 19.5), (15.6, 19.5)],
    "resus -> E compartment door -> SE lobby -> link (4 m) -> theatre": _RESUS_TO_WARD_ENTRANCE + [
        (19.0, 9.0), (19.0, -2.0), (19.23, -4.45), (19.23, -7.5), (19.23, -12.45), (25.93, -12.45),
        (25.93, -14.45), (25.93, -16.45)],
}

# Corrections to layout.BUILDING_ROUTES, found by this check (the design's raster pre-check had no vehicle and no
# door frames). name -> (waypoints, reason). Remove an entry when layout.py has the same change.
BUILDING_ROUTE_FIXES = {}   # the earlier fixes (ambulance start y 0.1, walk-in x 1.46) are now in layout.py

# Doors (spec names) that a route passes and that are not in every tier. When the current tier leaves one of them
# out, the asset's own leaves stay in that doorway, so the route is skipped.
NEEDS = {
    "pharmacy 1 -> main corridor -> E compartment door -> ward clean utility": ["Pharmacy1", "Ward_CleanLinen"],
    "nurse base -> ward clean utility": ["Ward_CleanLinen"],
    "D07 final exit": ["D07_Exit"],
    "D34 final exit": ["D34_Exit"],
}

# Routes through the main-entrance glass units: make_stage.py removes the glazing only at GLASS_TIERS.
NEEDS_GLASS = {"walk-in: main entrance -> A&E waiting"}

# Spawns (g1_config.load_spawns()) that must stand clear. Other spawns are checked by their own tools.
SPAWNS = ("corridor_west", "link", "lobby", "ward_entrance", "ae_handover")
SPAWN_CLEAR = 0.30


def route_pose(F, spec):
    """Fixture joint positions for the route check (see the module docstring)."""
    q = F.FixtureController(spec).initial_q()
    for d in spec["doors"]:  # kept-shut doors (no magnets): the robot opens every leaf that is not bolted
        for lf in d["leaves"]:
            if lf["hold"] is None and not lf["bolted"]:
                q[lf["joint"]] = math.copysign(math.pi / 2, lf["range"][0] + lf["range"][1])
    for lift in spec["lifts"]:
        for p in lift["panels"]:
            q[p["joint"]] = p["travel"]
    return q


HEIGHTS = (0.3, 0.8, 1.3)
BODY_HALF_WIDTH = 0.25  # G1 shoulder half-width ~0.23 m
END_CLEAR = BODY_HALF_WIDTH  # free radius around the last waypoint (a 0.5 m footprint)


def main():
    stage = Usd.Stage.Open(os.path.join(os.path.dirname(HERE), "usd", "hospital_g1_robot.usd"))
    lid = WarpLidar(stage, exclude_roots=["/World/G1"], device="cpu", rings=2, azimuths=4, scan_rays=4)
    F, spec = load_spec()  # tier from $HG1_FIXTURES, as make_stage.py
    if stage.GetPrimAtPath("/World/Fixtures"):
        lid.set_dynamic_boxes(*moving_boxes(F, spec, route_pose(F, spec)))
    print(lid.describe())
    import warp as wp

    def cast(o, d, maxd):
        dirs = wp.array(np.asarray([d], np.float32), dtype=wp.vec3, device="cpu")
        out = wp.zeros(1, dtype=float, device="cpu")
        lid._launch(1, wp.vec3(*o), wp.mat33(*np.eye(3, dtype=np.float32).ravel()), dirs, float(maxd), out)
        return float(out.numpy()[0])

    excluded = set(spec["excluded"]["doors"]) if spec else set()
    routes = [(n, [W(*p) for p in pts]) for n, pts in ROUTES.items()]
    routes += [(n, list(BUILDING_ROUTE_FIXES.get(n, (pts, None))[0]))
               for n, pts in getattr(layout, "BUILDING_ROUTES", {}).items()]
    routes += [(n, list(pts)) for n, pts in WORLD_ROUTES.items()]
    ok_all, counts = True, {"OK": 0, "FAIL": 0, "SKIP": 0}
    for name, wpts in routes:
        if name in BUILDING_ROUTE_FIXES:
            print(f"     ({name}: waypoints adjusted: {BUILDING_ROUTE_FIXES[name][1]})")
        missing = [d for d in NEEDS.get(name, []) if d in excluded]
        if name in NEEDS_GLASS and spec["tier"] not in GLASS_TIERS:
            missing.append("the main-entrance glazing removal")
        if missing:
            counts["SKIP"] += 1
            print(f"SKIP {name}  (tier {spec['tier']} leaves out {', '.join(missing)})")
            continue
        issues = []
        for (x0, y0), (x1, y1) in zip(wpts, wpts[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 1e-6:
                continue
            d = ((x1 - x0) / L, (y1 - y0) / L, 0.0)
            n = (-d[1], d[0])
            for h in HEIGHTS:
                for off in (-BODY_HALF_WIDTH, 0.0, BODY_HALF_WIDTH):
                    o = (x0 + n[0] * off, y0 + n[1] * off, h)
                    t = cast(o, d, L)
                    if t > 0:
                        issues.append(f"blocked at h={h} off={off:+.2f}: hit {t:.2f} m into {L:.2f} m segment "
                                      f"from ({x0:.2f},{y0:.2f})")
            for k in range(int(L / 0.5) + 1):
                px, py = x0 + d[0] * 0.5 * k, y0 + d[1] * 0.5 * k
                t = cast((px, py, 1.5), (0.0, 0.0, -1.0), 3.0)
                if t < 0 or abs(1.5 - t) > 0.05:
                    issues.append(f"no floor at ({px:.2f},{py:.2f}) (hit {t:.2f})")
        ex, ey = wpts[-1]          # the robot stands at the route end: its footprint must be free
        for h in HEIGHTS:
            for k in range(12):
                a = 2 * math.pi * k / 12
                t = cast((ex, ey, h), (math.cos(a), math.sin(a), 0.0), END_CLEAR)
                if t > 0:
                    issues.append(f"route end ({ex:.2f},{ey:.2f}): obstacle {t:.2f} m away at h={h}, "
                                  f"heading {math.degrees(a):.0f} deg")
        ok_all &= not issues
        counts["FAIL" if issues else "OK"] += 1
        print(("OK   " if not issues else "FAIL ") + name)
        for i in issues[:6]:
            print("      " + i)

    spawns = C.load_spawns()
    for sname in SPAWNS:
        if sname not in spawns:
            print(f"SKIP spawn {sname} (not defined)")
            continue
        x, y = float(spawns[sname]["x"]), float(spawns[sname]["y"])
        issues = []
        t = cast((x, y, 1.5), (0.0, 0.0, -1.0), 3.0)
        if t < 0 or abs(1.5 - t) > 0.05:
            issues.append(f"no floor (hit {t:.2f})")
        for h in HEIGHTS:
            for k in range(12):
                a = 2 * math.pi * k / 12
                t = cast((x, y, h), (math.cos(a), math.sin(a), 0.0), SPAWN_CLEAR)
                if t > 0:
                    issues.append(f"obstacle {t:.2f} m away at h={h}, heading {math.degrees(a):.0f} deg")
        ok_all &= not issues
        counts["FAIL" if issues else "OK"] += 1
        print(("OK   " if not issues else "FAIL ") + f"spawn {sname} ({x:.2f}, {y:.2f})")
        for i in issues[:6]:
            print("      " + i)
    print(f"tier {spec['tier'] if spec else 'none'}: {counts['OK']} clear, {counts['FAIL']} blocked, "
          f"{counts['SKIP']} skipped")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
