"""Check the robot's walkable routes against the static collision geometry of the Isaac stage (no Isaac needed):

    uv run --no-project --with usd-core --with warp-lang python g1-isaac-sim/tools/check_routes.py

Casts rays at knee / hip / head height (0.3, 0.8, 1.3 m) along each route segment (spawn -> corridor -> every
doorway -> room centre) and down from 1.5 m to find the floor. A route is clear when no ray hits within the
segment, and every sample point has a floor hit near z = 0.
"""

import math
import os
import sys

import numpy as np
from pxr import Usd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "hospital-sim-env", "isaac", "scene"))  # layout.py
from lidar_warp import WarpLidar  # noqa: E402

import g1_config as C  # noqa: E402

OX, OY = C.WING_ORIGIN[0], C.WING_ORIGIN[1]


def W(x, y):
    return (x + OX, y + OY)


# wing-local waypoints (layout.py), world spawns
ROUTES = {
    "lobby -> SE door -> link -> corridor": [(1.0, 2.5), (1.0, 0.0), (1.0, -3.0), (1.0, -6.0), (1.0, -8.0)],
    "corridor west -> east": [(-9.0, -8.0), (11.5, -8.0)],  # a plant stands at the east end
    "corridor -> pre-op": [(-4.0, -8.0), (-4.0, -6.0), (-4.0, -4.5)],
    "corridor -> PACU": [(8.0, -8.0), (8.0, -6.0), (8.0, -4.5)],
    "corridor -> sterile store": [(-8.0, -8.0), (-8.0, -10.0), (-8.0, -12.0)],
    "corridor -> scrub": [(-2.0, -8.0), (-2.0, -10.0), (-2.0, -12.0)],
    "corridor -> theatre": [(8.0, -8.0), (8.0, -10.0), (8.0, -12.0)],
    "scrub -> theatre": [(-2.0, -14.0), (0.0, -14.0), (1.5, -14.0)],
    "scrub -> store": [(-2.0, -14.0), (-4.0, -14.0), (-5.5, -14.0)],
}
HEIGHTS = (0.3, 0.8, 1.3)
BODY_HALF_WIDTH = 0.25  # G1 shoulder half-width ~0.23 m


def main():
    stage = Usd.Stage.Open(os.path.join(os.path.dirname(HERE), "usd", "hospital_g1_robot.usd"))
    lid = WarpLidar(stage, exclude_roots=["/World/G1"], device="cpu", rings=2, azimuths=4, scan_rays=4)
    print(lid.describe())
    import warp as wp

    from lidar_warp import _cast

    def cast(o, d, maxd):
        dirs = wp.array(np.asarray([d], np.float32), dtype=wp.vec3, device="cpu")
        out = wp.zeros(1, dtype=float, device="cpu")
        wp.launch(_cast, dim=1, inputs=[lid.mesh.id, wp.vec3(*o), wp.mat33(*np.eye(3, dtype=np.float32).ravel()),
                                        dirs, float(maxd), out], device="cpu")
        return float(out.numpy()[0])

    ok_all = True
    for name, pts in ROUTES.items():
        issues = []
        wpts = [W(*p) for p in pts]
        for (x0, y0), (x1, y1) in zip(wpts, wpts[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
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
        ok_all &= not issues
        print(("OK   " if not issues else "FAIL ") + name)
        for i in issues[:6]:
            print("      " + i)
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
