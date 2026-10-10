"""Usage (no Isaac needed):
    uv run --no-project --with usd-core --with warp-lang --with numpy python g1-isaac-sim/tools/rasterize_colliders.py [out.npz]

Rasterise the Isaac stage's static collision geometry (exactly what the Warp lidar ray-casts) to occupancy.

A cell is occupied at height h when a short horizontal ray from the cell centre (8 directions, length ~ cell
half-diagonal) hits a surface: i.e. there is a lidar-visible surface inside the cell. 'scan' = heights 1.0-1.4 m
(the /scan plane band while walking), 'band' = 0.2-1.8 m (anything a body could hit), 'floor' = floor present.
"""
import os, sys, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from pxr import Usd
import warp as wp
from lidar_warp import WarpLidar

@wp.kernel
def surf(mesh: wp.uint64, xs: wp.array(dtype=wp.vec3), z: float, L: float, hit: wp.array(dtype=int)):
    i = wp.tid()
    o = wp.vec3(xs[i][0], xs[i][1], z)
    h = int(0)
    for k in range(8):
        a = float(k) * 0.7853981
        q = wp.mesh_query_ray(mesh, o, wp.vec3(wp.cos(a), wp.sin(a), 0.0), L)
        if q.result:
            h = 1
    if h == 1:
        hit[i] = 1

@wp.kernel
def down(mesh: wp.uint64, xs: wp.array(dtype=wp.vec3), ztop: float, length: float, hit: wp.array(dtype=int)):
    i = wp.tid()
    q = wp.mesh_query_ray(mesh, wp.vec3(xs[i][0], xs[i][1], ztop), wp.vec3(0.0, 0.0, -1.0), length)
    if q.result:
        hit[i] = 1

stage = Usd.Stage.Open(os.path.join(os.path.dirname(HERE), 'usd', 'hospital_g1_robot.usd'))
lid = WarpLidar(stage, exclude_roots=['/World/G1'], device='cpu', rings=2, azimuths=4, scan_rays=4)
print(lid.describe())
res = 0.05
x0, y0, x1, y1 = -50.5, -26.0, 33.5, 38.5
W = int(round((x1 - x0) / res)); H = int(round((y1 - y0) / res))
xs = x0 + (np.arange(W) + 0.5) * res
ys = y0 + (np.arange(H) + 0.5) * res
X, Y = np.meshgrid(xs, ys)
P = wp.array(np.c_[X.ravel(), Y.ravel(), np.zeros(X.size)].astype(np.float32), dtype=wp.vec3, device='cpu')
out = {}
for name, hs in {'scan': np.arange(1.0, 1.41, 0.1), 'band': np.arange(0.2, 1.81, 0.1)}.items():
    hit = wp.zeros(X.size, dtype=int, device='cpu')
    for h in hs:
        wp.launch(surf, dim=X.size, inputs=[lid.mesh.id, P, float(h), res * 0.75, hit], device='cpu')
    if name == 'band':
        wp.launch(down, dim=X.size, inputs=[lid.mesh.id, P, 1.8, 1.6, hit], device='cpu')
    out[name] = hit.numpy().reshape(H, W).astype(bool)
    print(name, out[name].mean())
hit = wp.zeros(X.size, dtype=int, device='cpu')
wp.launch(down, dim=X.size, inputs=[lid.mesh.id, P, 0.05, 0.25, hit], device='cpu')
out['floor'] = hit.numpy().reshape(H, W).astype(bool)
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'results', 'collider_raster.npz')
os.makedirs(os.path.dirname(OUT), exist_ok=True)
np.savez_compressed(OUT,
                    origin=np.array([x0, y0]), res=res, **out)
