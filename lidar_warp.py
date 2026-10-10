"""MID-360-like lidar + 2-D scan by Warp ray casting against a snapshot of the static collision geometry.

Why not RTX lidar: an RTX lidar render product adds GPU memory and render time on an 8 GB card that the
hospital already fills to ~7 GB when rendering. This caster runs on the CPU (Warp BVH), costs no VRAM, and sees
exactly what the robot can collide with: the physics layer's walls / floors / furniture boxes, the Isaac
hospital's static colliders and every other static collider. Dynamic rigid bodies and the robot are ignored.

Pattern (Livox MID-360: 360 deg x -7..52 deg in its own frame, 10 Hz, non-repetitive): 25 elevation rings x 400
azimuths with per-scan jitter (8 pre-generated patterns cycled) = up to 10k points / scan, like the MuJoCo backend.
/scan: 720 rays in the x-y plane of laser_link (= mid360_link rolled back by pi, z up).
"""

from __future__ import annotations

import math

import numpy as np
import warp as wp
from pxr import Gf, Usd, UsdGeom, UsdPhysics

wp.config.quiet = True
wp.init()


@wp.kernel
def _cast(mesh: wp.uint64, origin: wp.vec3, rot: wp.mat33, dirs: wp.array(dtype=wp.vec3), max_d: float,
          out: wp.array(dtype=float)):
    tid = wp.tid()
    d = rot * dirs[tid]
    q = wp.mesh_query_ray(mesh, origin, d, max_d)
    if q.result:
        out[tid] = q.t
    else:
        out[tid] = -1.0


_BOX_TRIS = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]], dtype=np.int32)


def _box(mn, mx, m):
    c = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])], float)
    pts = np.array([m.Transform(Gf.Vec3d(*p)) for p in c], float)
    return pts, _BOX_TRIS.copy()


def _triangulate(counts, idx):
    tris = []
    o = 0
    for n in counts:
        for k in range(1, n - 1):
            tris.append((idx[o], idx[o + k], idx[o + k + 1]))
        o += n
    return np.asarray(tris, dtype=np.int32).reshape(-1, 3)


class WarpLidar:
    range_min = 0.1
    range_max = 40.0

    def __init__(self, stage, exclude_roots=(), device="cpu", rings=25, azimuths=400, scan_rays=720,
                 elev_deg=(-7.0, 52.0)):
        self.device = device
        pts_all, tris_all = [], []
        nverts = 0
        self.n_prims = 0
        cache = UsdGeom.XformCache()
        ex = [str(r) for r in exclude_roots]
        it = iter(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
        for p in it:
            path = str(p.GetPath())
            if any(path == r or path.startswith(r + "/") for r in ex):
                it.PruneChildren()
                continue
            if p.HasAPI(UsdPhysics.RigidBodyAPI):  # dynamic body: skip whole subtree
                it.PruneChildren()
                continue
            if not p.HasAPI(UsdPhysics.CollisionAPI) or not p.IsA(UsdGeom.Gprim):
                continue
            if UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible and not p.IsA(UsdGeom.Cube):
                continue
            m = cache.GetLocalToWorldTransform(p)
            if p.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(p)
                v = mesh.GetPointsAttr().Get()
                c = mesh.GetFaceVertexCountsAttr().Get()
                i = mesh.GetFaceVertexIndicesAttr().Get()
                if not v or not c or not i:
                    continue
                v = np.asarray(v, dtype=float)
                v = np.c_[v, np.ones(len(v))] @ np.asarray(m)  # row-vector convention
                tri = _triangulate(np.asarray(c), np.asarray(i))
                pts, tr = v[:, :3], tri
            elif p.IsA(UsdGeom.Cube):
                s = UsdGeom.Cube(p).GetSizeAttr().Get() / 2.0
                pts, tr = _box((-s, -s, -s), (s, s, s), m)
            else:  # sphere / capsule / cylinder / cone: local bound box
                ext = UsdGeom.Boundable.ComputeExtentFromPlugins(UsdGeom.Boundable(p), Usd.TimeCode.Default())
                if not ext:
                    continue
                pts, tr = _box(ext[0], ext[1], m)
            pts_all.append(pts)
            tris_all.append(tr + nverts)
            nverts += len(pts)
            self.n_prims += 1
        if not pts_all:
            raise RuntimeError("no static collision geometry found")
        P = np.concatenate(pts_all).astype(np.float32)
        T = np.concatenate(tris_all).astype(np.int32).reshape(-1)
        self.n_tris = len(T) // 3
        self.points = wp.array(P, dtype=wp.vec3, device=device)
        self.indices = wp.array(T, dtype=wp.int32, device=device)
        self.mesh = wp.Mesh(points=self.points, indices=self.indices)
        self.rings, self.azimuths = rings, azimuths
        self.elev = np.radians(np.linspace(elev_deg[0], elev_deg[1], rings))
        self.rng = np.random.default_rng(0)
        a = np.linspace(-math.pi, math.pi, scan_rays, endpoint=False)
        self.scan_dirs = wp.array(np.c_[np.cos(a), np.sin(a), np.zeros_like(a)].astype(np.float32), dtype=wp.vec3,
                                  device=device)
        self.scan_out = wp.zeros(scan_rays, dtype=float, device=device)
        self.cloud_out = wp.zeros(rings * azimuths, dtype=float, device=device)
        # pre-generated jittered patterns (cycled per scan: MID-360-like non-repetitive coverage, no per-scan upload)
        self._pat_np = [self._dirs() for _ in range(8)]
        self._pat = [wp.array(d, dtype=wp.vec3, device=device) for d in self._pat_np]
        self._k = 0

    def describe(self):
        return f"warp/{self.device}: {self.n_prims} static collider prims, {self.n_tris} triangles"

    def _dirs(self):
        az = np.linspace(-math.pi, math.pi, self.azimuths, endpoint=False)
        az = az[None, :] + self.rng.uniform(0, 2 * math.pi / self.azimuths, size=(self.rings, 1))
        el = self.elev[:, None] + self.rng.uniform(-0.5, 0.5, size=(self.rings, 1)) * np.radians(59.0 / self.rings)
        ce = np.cos(el)
        d = np.stack([ce * np.cos(az), ce * np.sin(az), np.sin(el) * np.ones_like(az)], -1).reshape(-1, 3)
        return d.astype(np.float32)

    @staticmethod
    def _pose(T):
        R = T[:3, :3].astype(np.float32)
        return wp.vec3(*T[:3, 3].astype(np.float32)), wp.mat33(*R.reshape(-1))

    def cloud(self, T_world_sensor):
        """MID-360 frame (4x4 world pose, column-vector) -> (N x 4 [x y z intensity] in sensor frame, ring index)."""
        o, rot = self._pose(T_world_sensor)
        self._k = (self._k + 1) % len(self._pat)
        dirs_np, dirs = self._pat_np[self._k], self._pat[self._k]
        wp.launch(_cast, dim=len(dirs_np), inputs=[self.mesh.id, o, rot, dirs, self.range_max, self.cloud_out],
                  device=self.device)
        r = self.cloud_out.numpy()
        ok = r > self.range_min
        cloud = np.empty((int(ok.sum()), 4), dtype=np.float32)
        cloud[:, :3] = dirs_np[ok] * r[ok, None]
        cloud[:, 3] = np.clip(255.0 * (1.0 - r[ok] / self.range_max), 5, 255)
        ring = np.repeat(np.arange(self.rings), self.azimuths)[ok]
        return cloud, ring

    def scan2d(self, T_world_laser, range_max=30.0):
        """Planar scan in the laser frame's x-y plane: ranges[scan_rays] from -pi, inf = no return."""
        o, rot = self._pose(T_world_laser)
        wp.launch(_cast, dim=self.scan_dirs.shape[0], inputs=[self.mesh.id, o, rot, self.scan_dirs, range_max,
                                                              self.scan_out], device=self.device)
        s = self.scan_out.numpy()
        return np.where(s > self.range_min, s, np.inf).astype(np.float32)
