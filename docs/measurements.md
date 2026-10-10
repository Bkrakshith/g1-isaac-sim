# g1-isaac-sim measurements

This file records what the simulator did on this machine (RTX 3070 8 GB, ROS_DOMAIN_ID 64 unless stated). The
scripts `tools/vram_watch.sh` and `tools/vram_guard.sh` took the samples. Nothing here is a target; it is what was
observed.

## Rates and real-time factor

- Headless hospital: the loop takes 1.3-2.0 ms per 2 ms step, 500 Hz, RTF 0.99-1.00. A ROS 2 node receives
  `/lowstate` at 495-500 Hz. The other rates are `/clock` 500 Hz; `/joint_states`, `/odom` and
  `/sim/ground_truth` 100 Hz; `/livox/lidar` and `/scan` 10 Hz; `/sim/status` 1 Hz. The process VRAM peak is
  1.6 GB, because the RTX renderer never loads the hospital when nothing is rendered. The flat world peaks at
  1.3 GB.
- PD hold at the default pose, driven by `tools/lowcmd_hold_test.py` from `.venv`. The test releases the harness,
  settles for 2 s, then sends 10 s of 500 Hz `/lowcmd` (CRC OK, 0 bad). With the stiff test-stand gains
  (hips/knees kp 500 / kd 10, ankles 400 / 3, waist 300 / 6, arms ULTRA), the robot stays upright in the hospital.
  The max |roll|/|pitch| is 3.0 deg and the pelvis is at 0.784 m. With the ULTRA policy gains alone, it falls after 0.72 s
  (MuJoCo: about 0.83 s), as expected without a policy. After the fall the simulator stays stable; the robot lies
  at z 0.1 m.
- Harness: it holds the pelvis at 0.73 m with the motors off. It releases about 3.5 s after a `/g1/mode` STAND
  message.
- The script `tools/check_routes.py` reports all 9 routes clear.
- Flat ground with `--camera`: RTF about 0.77, because the render every 50 steps blocks the loop. VRAM +0.3 GB.
- Flat theatre, fixed base, hand physics, no lidar (ROS_DOMAIN_ID 72): 500 Hz, 1.2 ms/step. With `--camera`
  (640x360 RGB-D at 10 Hz sim time): 8 ms/step, RTF about 0.24, that is about 2.4 camera frames per wall second.
  The process VRAM is 1.6 GB (another Isaac GUI held 4.2 GB). `/sim/objects` runs at 20 Hz sim time. The depth on
  the table stance is 0.57-1.9 m.
- `/g1/head_camera` at 960x540, 4-10 Hz: about 250 ms per frame. The render is CPU-bound; the GPU is about 1 %
  busy and the CPU governor is powersave. RTF 0.27 at 10 Hz, about 0.38 at 4 Hz, about 0.45 at 640x360 10 Hz
  without viewport and depth.
- Hospital headless with lidar and the hand-physics fixes: 1.7-1.8 ms/step (1.3-2.0 before the fixes).
- Stills (`renders/`, RTX RaytracedLighting 1280x720, process VRAM peak 3.0-3.3 GB): `g1_corridor_front.png`,
  `g1_corridor_behind.png` (corridor_west spawn), `g1_link_door.png` (link spawn, hospital lobby through the SE
  door). Another Isaac instance held 3.3 GB during the renders. The renders therefore hid the rooms not in view
  with `--deactivate`. The corridor shots hid the building; the link shot hid the OR suite and the other rooms'
  dressing.

## VRAM notes

The lite asset mirror (`hospital-sim-env/isaac/scene/select_assets.py lite`) is required on 8 GB. Headless without
a camera needs little VRAM: 1.3-1.6 GB. Anything that renders the hospital (`--gui`, `--camera`, `--stills`,
`tools/replay_camera.py`) needs about 5 GB on top of the desktop. Close other Isaac instances first. A second Isaac
GUI with the hospital open plus this backend's rendering does not fit in 8 GB. The full scene peaks near 7 GB when
rendered, with about 1.9 GB idle.

Keep `XLA_PYTHON_CLIENT_MEM_FRACTION=0.35` for JAX jobs on the same card. Stale `kit` processes hold VRAM after a
crash. Check `pgrep -x kit` and `pgrep -f run_sim.py`.

## Not measured

- The option `--gui` on this machine. The card was shared during the build.
- The lidar timing on `--lidar-device cpu` versus `cuda:0`.
- Any run on a real robot. None exists for this backend.
