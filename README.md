# Isaac Sim backend for the G1

The Isaac Sim 6.0 backend of the G1 stack. The G1 29-DoF rev 1.0 with BrainCo Revo2 hands stands in the Isaac
hospital and speaks the same low-level interface as the real robot and as the MuJoCo backend in
`g1-mujoco-sim`, so the ROS 2 stack in `hospital-sim-ros2` (controller, manager, keyboard, UI, Nav2) runs on
top without changes. It reads from three siblings: `g1-brainco-description` for the URDF and
`robot_spec.json`, `hospital-sim-env` for the scene and the instruments, and `g1-mujoco-sim` for the shared
spawn poses in `worlds/spawns.json`. The master repo `hospital-sim` checks all of them out in that
arrangement, and `env.sh` lives there too.

Paths below are relative to this folder unless they start with a sibling repo name (`hospital-sim-env/...`,
`g1-brainco-description/...`, `g1-mujoco-sim/...`) or `docs/`, which are relative to the master repo root. Commands
are written to be run from the master repo root (`./g1-isaac-sim/run.sh ...`).

Nothing in `hospital-sim-env/isaac/` (`scene/`, `usd/`, `assets/`) or `hospital-sim-env/objects/` is modified: this
folder adds layers that sublayer/reference them.

## Launch

```bash
./g1-isaac-sim/run.sh --headless              # hospital, no window (RTX never loads the scene: ~1.6 GB VRAM)
./g1-isaac-sim/run.sh --gui                   # hospital with the Isaac window (needs ~5 GB free VRAM; not yet
                                           # exercised here, the card was shared during this build)
./g1-isaac-sim/run.sh --headless --flat       # flat ground, quick tests
./g1-isaac-sim/run.sh --headless --spawn link # other spawns: corridor_west (default), link, lobby
./g1-isaac-sim/run.sh --headless --theatre --camera --spawn theatre_instrument_table   # scrub-nurse theatre + RGB-D
./g1-isaac-sim/run.sh --headless --flat --theatre --fixed-base --camera  # theatre objects on flat ground, pelvis welded
```

Then, in another terminal, the usual stack (`source env.sh`, controller + manager + robot_state_publisher with
`use_sim_time:=true`). `g1_bringup/sim.launch.py` always starts `g1_mujoco_sim`; it needs a switch (e.g.
`backend:=isaac`) that skips that node. Use a private `ROS_DOMAIN_ID` when other sims run on the machine. Useful options of `run_sim.py` (all passed through `run.sh`):

| Option | Default | What |
|---|---|---|
| `--pd explicit\|implicit` | explicit | explicit: `tau = kp(q*-q) + kd(dq*-dq) + tau_ff` from `/lowcmd`, clipped to the motor limit, every 2 ms step (motor-driver semantics, same as MuJoCo). implicit: PhysX joint drives with the `/lowcmd` gains |
| `--harness on\|off` | on | virtual harness on the pelvis, same law and gains as `g1_mujoco_sim`: holds the spawn pose while `/g1/mode` is DAMP, released 2.5 s after STAND (1 s fade); `/sim/harness` (SetBool) toggles it, `/sim/reset` (Trigger) respawns with it |
| `--idle zero\|damp\|hold` | zero | motors before the first `/lowcmd` (after `/sim/reset` too): zero torque like MuJoCo, kd only, or hold the default pose with `--idle-gains stand\|ultra` |
| `--effort-limits ultra\|model` | ultra | motor torque clip: `robot_spec.json` `effort_limit_ultra` (50 Nm ankles / waist roll+pitch; what `g1_bringup` picks for the placeholder policy) or `effort_limit` (35 Nm) |
| `--lidar warp\|none`, `--lidar-hz`, `--lidar-device` | warp, 10, cuda:0 | lidar ray casting (below) |
| `--camera`, `--camera-res WxH`, `--camera-hz` | off, 640x360, 10 | head RGB-D camera: `/g1/head_camera/{image_raw,depth,camera_info}` (renders the scene: VRAM + RTF cost) |
| `--hand-collisions on\|off` | **on** | Revo2 finger/palm colliders with the `hand_physics.py` fixes (grasping); off = old behaviour |
| `--theatre` | off | scrub-nurse objects (`usd/theatre_instruments.usd`): stage `hospital_g1_theatre.usd` (or `flat_theatre_robot.usd` with `--flat`); publishes `/sim/objects` |
| `--fixed-base` | off | weld the pelvis at the spawn (arm-skill development, contract 6); the harness is disabled |
| `--spawn NAME\|x,y,yaw` | spawns.json default | also `hospital-sim-env/objects/placements.yaml` robot stances (`theatre_instrument_table`, `theatre_surgeon_side`) or explicit `x,y,yaw` |
| `--objects-hz` | 20 | `/sim/objects` rate |
| `--dynamic-props` | off | keep the Lightwheel carts/trays dynamic (default: frozen static colliders) |
| `--no-realtime`, `--duration S`, `--stats S`, `--profile`, `--log CSV` | | pacing, run length, statistics |
| `--scan-level` | off | cast `/scan` in a gravity-levelled plane at the MID-360 (pelvis heading) and publish it in frame `laser_level` with a dynamic TF `pelvis->laser_level`. Without it the scan plane tilts with the torso and a walking G1 (torso pitched ~4-10 deg) puts floor hits 8-10 m ahead into the scan, which slam_toolbox paints into the map as phantom walls |
| `--base-footprint` | off | also publish TF `pelvis->base_footprint` (pelvis projected to the floor, x/y/yaw only). slam_toolbox needs a planar base frame (`slam.launch.py base_frame:=base_footprint`): with `pelvis` its map->odom took the torso tilt (up to 10 deg, z -2 m) |
| `--topic-prefix /isaac` | "" | publish /clock /tf /tf_static /odom /joint_states /scan /livox/lidar under the prefix; `tools/isaac_relay.py` (system ROS) republishes them. Nav2's costmaps stopped receiving TF/clock within minutes when fed straight from this process (root cause not found; the relay fixed the local costmap) |
| `--camera-depth on\|off`, `--no-viewport`, `--anti-aliasing N`, `--camera-pitch DEG`, `--camera-vfov DEG` | on, -, 0, mount, 42 | long camera runs: skip the depth readback, render only the head camera (not the default viewport: 2x faster), DLSS (3) removes sampled-light speckle; `--camera-pitch` is a VIDEO VIEW that points the head camera DEG below the torso x axis instead of the real D435 mount (48 deg down, it only sees the floor), with `--camera-vfov` |
| `ROS_DOMAIN_ID` env | 0 | used by both the ROS 2 node and the Unitree DDS channel |

Other entry points: `run.sh --import` (re-import the URDF), `run.sh --stage` (regenerate the physics layer and
root stages), `run.sh --stills` (render stills), `run.sh --python script.py`. One-time DDS deps: `setup_deps.sh`
(run automatically by `run.sh`).

## Interface (what the sim speaks)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/lowstate` (DDS `rt/lowstate`) | `unitree_hg/LowState` | 500 Hz | motors 0..28 in contract order (q, dq, tau_est = applied torque), IMU = pelvis (quat wxyz, gyro, accel, rpy), `tick` = sim ms, `mode_machine` 5, CRC filled |
| `/lowcmd` (DDS `rt/lowcmd`) | `unitree_hg/LowCmd` | any | q, dq, tau, kp, kd per motor; `mode` 0 = motor off. CRC checked (count reported; `--strict-crc` drops bad ones) |
| `/clock` | rosgraph_msgs/Clock | 500 Hz | sim time |
| `/joint_states` | JointState | 100 Hz | 29 body + 12 actuated Revo2 joints (same names as MuJoCo) |
| `/odom` + `/tf` `odom->pelvis` | Odometry | 100 Hz | ground truth; `odom` = spawn pose |
| `/sim/ground_truth` | Odometry | 100 Hz | pelvis pose in the world frame |
| `/livox/lidar` | PointCloud2 | 10 Hz | frame `mid360_link` (real mount: rolled by pi, z down), livox_ros_driver2 layout x y z intensity tag line timestamp (26 B), <= 10 000 pts |
| `/scan` | LaserScan | 10 Hz | frame `laser_link` (= mid360_link rolled back by pi, z up), 720 beams, -pi..pi, 30 m |
| `/tf_static` | | latched | `mid360_link -> laser_link` (rotation pi about x), as `g1_mujoco_sim` |
| `/g1/hand/{left,right}/cmd` | JointState | in | 6 values 0 (open) .. 1 (closed), order thumb, thumb_aux, index, middle, ring, pinky -> `q = u * upper` (robot_spec.json); distal joints follow with the URDF mimic ratios |
| `/g1/hand/{left,right}/state` | JointState | 25 Hz | same normalisation |
| `/g1/head_camera/image_raw` | Image rgb8 | `--camera-hz` | only with `--camera`; 640x360, frame `d435_link`, pixels in the optical convention (x right, y down, z forward) |
| `/g1/head_camera/depth` | Image 32FC1 | with image_raw | metres along the optical axis (distance to image plane), NaN = no return; pixel-aligned with image_raw |
| `/g1/head_camera/camera_info` | CameraInfo | with image_raw | pinhole, no distortion: fx = fy = 468.9, cx = 320, cy = 180 at 640x360 (vertical FOV 42 deg = D435 colour, same as the MJCF `head_d435`) |
| `/sim/objects` | String (JSON) | 20 Hz | with `--theatre`: `{name: {"pose": [x,y,z,qw,qx,qy,qz], "type"}}`, world (= map) frame, contract 6; the 7 instruments live from PhysX, table / rack / surgeon / `surgeon_hand` (type `site`) static |
| `/g1/mode` (in) | g1_interfaces/Mode | | read straight off DDS (`rt/g1/mode`, `dds_types.py`) for the harness release |
| `/sim/status` | String (JSON) | 1 Hz | backend, sim_time, rtf, step_ms_mean, lowcmd_hz, lowcmd_bad_crc, harness, pelvis_xyz |
| `/sim/harness`, `/sim/reset` | std_srvs SetBool / Trigger | | as `g1_mujoco_sim` |

### Low-level transport choice

`/lowcmd` and `/lowstate` are **not** published through the Isaac ROS 2 bridge: `unitree_hg` messages would need
to be built for Isaac's Python 3.12. Instead, as in Unitree's `unitree_sim_isaaclab`, the sim talks DDS directly
with CycloneDDS + the `unitree_sdk2_python` IDL types (`unitree_hg::msg::dds_::LowState_` / `LowCmd_` on
`rt/lowstate` / `rt/lowcmd`). ROS 2 nodes with `rmw_cyclonedds_cpp` (what `env.sh` sets) see these as
`/lowstate` and `/lowcmd` with type `unitree_hg/msg/...`, exactly like on the real robot.

Details that matter if you touch it:
- `setup_deps.sh` builds the **cyclonedds 0.10.5** Python binding against Isaac's bundled Humble `libddsc`
  0.10.5 (identical to `/opt/ros/humble`), into `pydeps/` (Isaac's install is not modified). The pip build bakes
  its temporary build directory into `pydeps/cyclonedds/__library__.py`; `setup_deps.sh` rewrites `library_path`
  to `/opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so` afterwards. The rmw
  and the Unitree channel therefore share one libddsc and one DDS domain in the process.
- The ROS 2 node must be created **before** the Unitree participant, and `unitree_sdk2py`'s
  `ChannelFactoryInitialize()` cannot be used (it creates the domain with its own XML config, which then fails).
  `run_sim.py` attaches plain CycloneDDS writers/readers to the existing domain instead. Pick a network interface
  with `--net-if` / `CYCLONEDDS_URI` (shared by both).
- `/lowstate` is serialized with numpy (`fast_lowstate.py`), self-tested byte-for-byte against the cyclonedds
  serializer at start-up: 0.05 ms instead of ~1 ms per message.
- Standard messages use Isaac's bundled Humble **rclpy** directly. The `isaacsim.ros2.bridge` extension itself
  loads with Humble, but with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` Isaac 6.0 segfaults in OmniGraph on the
  first `app.update()` (with FastDDS it works). No OmniGraph ROS nodes are used.

## Files

| Path | What |
|---|---|
| `import_g1.py` | Isaac 6.0 URDF importer: `g1-brainco-description/g1_29dof_brainco/urdf/g1_29dof_brainco.urdf` -> `usd/g1_29dof_brainco.usd` (floating base, articulation root on `pelvis`, drives in force mode with stiffness 0 (effort mode), armature and effort limits from `robot_spec.json`, self-collision off, 4/1 solver iterations). Checks the 29 contract joints and 12 + 10 Revo2 joints; writes `usd/g1_29dof_brainco.joints.json` |
| `make_stage.py` | pure-USD generator of `usd/physics_colliders.usd`, `usd/hospital_g1_robot.usd` (root: colliders + `hospital-sim-env/isaac/usd/hospital_g1.usd` + `/World/G1` at the default spawn) and `usd/flat_g1_robot.usd` |
| `run_sim.py` | the sim + bridge (standalone Isaac Python app) |
| `lidar_warp.py` | MID-360 / 2-D scan by Warp ray casting |
| `fast_lowstate.py` | numpy CDR writer for `rt/lowstate` |
| `g1_config.py` | joint order, default pose, gains, spawns (reads `g1-brainco-description/g1_29dof_brainco/robot_spec.json`, `g1-mujoco-sim/worlds/spawns.json`) |
| `render_stills.py` | RTX stills -> `renders/` |
| `make_theatre_layer.py` | pure-USD: `usd/theatre_instruments.usd` (`hospital-sim-env/objects/usd` references at the `hospital-sim-env/objects/placements.yaml` poses, `/World/Theatre/*` tagged `hg1:type`; deactivates the two OR-suite stools and their collider boxes), `usd/hospital_g1_theatre.usd`, `usd/flat_theatre_robot.usd`. Run by `run.sh --stage` |
| `hand_physics.py` | Revo2 grasp physics (session-layer overrides, below); used by `run_sim.py` and the tests |
| `tools/hand_grasp_test.py` | in-hand grasp test (twin of `hospital-sim-env/objects/tools/hand_grasp_test_mujoco.py`) and fall test -> `tools/results/` |
| `tools/rack_settle_test.py` | instruments standing in the rack for 3 s (+ `--probe`, `--scan-rack` ray diagnostics) |
| `tools/theatre_topics_check.py` | ROS 2 client: rates of `/sim/objects` + head RGB-D, saves a frame / depth / objects |
| `tools/lowcmd_hold_test.py` | ROS 2 test client (`.venv`): 500 Hz `/lowcmd` PD hold at the default pose, checks `/lowstate` |
| `tools/check_routes.py` | ray-casts the walkable routes (spawns, corridor, every doorway) against the static colliders |
| `tools/vram_watch.sh`, `tools/vram_guard.sh` | VRAM sampling / kill-switch used for the measurements below |
| `tools/estop_stub.py` | hardware e-stop heartbeat stub (`/g1/estop/heartbeat`) for this backend (g1_mujoco_sim has it built in); motion modes stay locked without it |
| `tools/isaac_relay.py` | system-ROS relay for `--topic-prefix` (see above) |
| `tools/rasterize_colliders.py` | occupancy rasters of the static colliders the lidar ray-casts (0.2-1.8 m band, 1.0-1.4 m scan band, floor) -> `tools/results/collider_raster.npz`; ground truth for the SLAM map check and the survey route planning |
| `tools/extract_traj.py`, `tools/replay_camera.py` | re-render the head camera from a recorded walk (pelvis pose + joint angles from the rosbag), RTX, no new physics |

### Physics layer (`usd/physics_colliders.usd`)

- `/World/PhysicsScene`: 500 Hz (`physxScene:timeStepsPerSecond`, dt 2 ms), TGS, CPU dynamics, MBP.
- Floors: one slab under the wing, one under the whole Isaac hospital footprint (the hospital's floor meshes
  have no colliders; its walls and furniture do, as exact triangle meshes).
- 22 wing walls from `hospital-sim-env/isaac/scene/layout.py` room rectangles with every `DOORWAYS`/`OPENINGS` gap cut out, 3 m high.
- 96 upright oriented boxes around dressing / OR-suite furniture that has no collider of its own (items smaller
  than ~0.3 m, above 1.4 m, or hanging from the ceiling are skipped; see `physics_colliders.json`).
- All boxes are invisible `Cube` guides with `PhysicsCollisionAPI`.
- `tools/check_routes.py`: lobby -> SE door -> link -> corridor, the whole corridor, and all 7 room doorways
  are clear at 0.3 / 0.8 / 1.3 m (+-0.25 m body width) with a floor under every point.

### Spawn pose

Same world frame and spawns as the MuJoCo world (`g1-mujoco-sim/worlds/spawns.json`): default **corridor_west**
= world (8.93, -12.45), yaw 0 (west end of the surgical corridor, facing east); `link` (18.93, -5.65, -pi/2);
`lobby` (18.93, -1.95, -pi/2). Pelvis starts 1 cm above the home height 0.784 m.

### Grasp physics (`hand_physics.py`, default on)

The imported Revo2 links are very light: 9–50 g finger segments, and 2 g fingertip "touch" pads on fixed joints with
principal inertias down to 3e-9 kg m². With finger colliders on and the stock 4/1 articulation iterations, a fall
onto the hands blew the solver up, which is why finger collisions used to be off. The session-layer fix:
- mass floor 5 g and inertia floor 1e-6 kg m² on every hand link (adds < 30 g per hand);
- finger and palm colliders on, contact offset 2 mm, rest offset 0;
- max depenetration velocity 0.5 m/s on hand links;
- finger joint velocity limit 10 rad/s;
- articulation solver iterations 16/2.

Measured (`tools/hand_grasp_test.py`, results in `tools/results/`):

| Test | fingers off (old) | finger colliders, no fix | **fixed** |
|---|---|---|---|
| fall onto the hands (motors off, pushed, 4 s) | max link speed 4.75 m/s | **NaN at t = 2.09 s, link speeds 6e11 m/s** | max link speed 5.0 m/s, finite |
| in-hand pinch, 7 instruments, 1.5 s hold + 3 s shake | – | 6/7 retained, 5/7 firm | 5–7/7 retained, 5–6/7 firm (≤ 7.7 mm slip in the shake), all finite |

Cost: still real time. Hospital headless with lidar: **500 Hz, 1.7-1.8 ms/step** (before the change: 1.3-2.0).
Theatre (flat, fixed base, no camera, no lidar): 500 Hz, 1.2 ms/step.

### Scrub-nurse theatre (`--theatre`)

`usd/theatre_instruments.usd` puts the `hospital-sim-env/objects/` table, rack, surgeon and 7 instruments at the
`hospital-sim-env/objects/placements.yaml` poses (the same poses the MuJoCo theatre uses). The instruments are dynamic, and
`run_sim.py` keeps `/World/Theatre` out of the prop freezing. In PhysX the rack settle gives tilt ≤ 5.0°
(`tools/rack_settle_test.py`). The instruments' colliders are box stacks: with CoACD hulls, PhysX pushed the
scissors' blades through the rack guide (hospital-sim-env/objects/README.md).

Head camera fix: the old camera op `rotateXYZ(90, -90, 0)` looked along d435_link **+y** (sideways). It is now
`(90, 0, -90)`, i.e. camera −Z = link +x and camera +Y = link +z. Verified on the rack image.

## Isaac vs MuJoCo backend

| | MuJoCo (`g1_mujoco_sim`) | Isaac (`g1-isaac-sim`) |
|---|---|---|
| Low-level topics, rates, CRC, IMU, tick | yes | same |
| `/joint_states`, `/odom`, `/tf`, `/clock`, `/sim/ground_truth` | yes | same |
| `/livox/lidar`, `/scan` frames and formats | yes | same (Warp ray casting; sees colliders) |
| Hands | position actuators | implicit PhysX drives; distal joints driven as mimic (PhysX ignores the importer's NewtonMimicAPI) |
| Hospital geometry | AABB boxes (wing exact) | full visual scene; building = its own triangle-mesh colliders, wing walls exact, furniture boxes |
| Collisions | soles only | every link incl. fingers (convex hulls; `hand_physics.py`) |
| Harness (`/g1/mode` release), `/sim/harness`, `/sim/reset`, `/sim/status` | yes | same (no `record:=` replay) |
| Head camera | MJCF camera | `--camera` (RTX): RGB + depth + camera_info, 640x360, vertical FOV 42 deg |
| Theatre objects, `/sim/objects` | mission's MuJoCo theatre (`hospital-sim-env/objects/mjcf`) | `--theatre` (`hospital-sim-env/objects/usd`) |
| RTF (hospital, full ROS I/O) | 0.99-1.00 | 1.00 headless (1.3-1.5 ms/step) |
| VRAM | none | 1.6 GB headless, ~5 GB with rendering (`--gui`, `--camera`, stills) |

## Measured (RTX 3070 8 GB, this machine, ROS_DOMAIN_ID 64)

- Headless hospital: loop 1.3-2.0 ms per 2 ms step, **500 Hz, RTF 0.99-1.00**; `/lowstate` received at 495-500 Hz
  by a ROS 2 node, `/clock` 500 Hz, `/joint_states` `/odom` `/sim/ground_truth` 100 Hz, `/livox/lidar` `/scan`
  10 Hz, `/sim/status` 1 Hz. Process VRAM peak **1.6 GB** (the RTX renderer never loads the hospital when
  nothing is rendered); flat world 1.3 GB.
- PD hold at the default pose driven by `tools/lowcmd_hold_test.py` from `.venv` (releases the harness, 2 s
  settle, then 10 s of 500 Hz `/lowcmd`, CRC OK, 0 bad): with the stiff **test-stand gains** (hips/knees kp 500 /
  kd 10, ankles 400 / 3, waist 300 / 6, arms ULTRA) the robot stays upright in the hospital, max |roll|/|pitch|
  3.0 deg, pelvis 0.784 m. With the ULTRA policy gains alone it falls after 0.72 s (MuJoCo: ~0.83 s), as expected
  without a policy; after the fall the sim stays sane (robot lies at z 0.1 m).
- Harness: holds the pelvis (0.73 m with motors off), released ~3.5 s after a `/g1/mode` STAND message.
- `tools/check_routes.py`: all 9 routes clear.
- Flat ground with `--camera`: RTF ~0.77 (rendering every 50 steps blocks the loop), VRAM +0.3 GB.
- Flat theatre, fixed base, hand physics, no lidar (ROS_DOMAIN_ID 72): 500 Hz, 1.2 ms/step. With `--camera`
  (640x360 RGB-D at 10 Hz sim time): 8 ms/step, **RTF ~0.24**, i.e. about 2.4 camera frames per wall second; process
  VRAM 1.6 GB (another Isaac GUI held 4.2 GB). `/sim/objects` at 20 Hz sim time. Depth on the table stance: 0.57–1.9 m.
- Stills (`renders/`, RTX RaytracedLighting 1280x720, process VRAM peak 3.0-3.3 GB): `g1_corridor_front.png`,
  `g1_corridor_behind.png` (corridor_west spawn), `g1_link_door.png` (link spawn, hospital lobby through the SE
  door). Another Isaac instance held 3.3 GB during the renders, so they were made with the rooms not in view
  hidden (`--deactivate`: building for the corridor shots, OR suite + other rooms' dressing for the link shot).

VRAM notes: the lite asset mirror is required. Headless without camera is cheap; anything that renders the
hospital needs ~5 GB on top of the desktop, so close other Isaac instances first (a second Isaac GUI with the
hospital open plus this backend's rendering does not fit in 8 GB). JAX jobs should keep
`XLA_PYTHON_CLIENT_MEM_FRACTION=0.35`.

## Known limits / gotchas

- **Do not step physics through `SimulationManager.step()`** and keep `isaacsim.core.simulation_manager`
  disabled after physics init (run_sim does this): in Isaac 6.0 its native per-step hook traverses the whole
  USD stage, ~9 ms per step with the hospital (RTF 0.2). Also `/physics/updateToUsd` is off and PhysX->Fabric sync
  is only enabled when something is rendered (it costs 0.6 ms/step).
- Finger collisions are on (grasping) with the `hand_physics.py` mass/inertia floors. `--hand-collisions off`
  restores the old behaviour. At the table stance the default-pose hands are inside the 0.78 m tray: raise the arms
  before stepping in, or the fingers chatter against the tray and knock the instruments.
- The camera render is synchronous: with `--camera` the sim runs at RTF ~0.24 (lower `--camera-hz` or the
  resolution to trade).
- The Lightwheel carts/trays are frozen by default (they are USD instances; `run_sim.py` de-instances the 9
  roots in the session layer to do it).
- The lidar sees the collision geometry (furniture boxes, not the furniture meshes); intensity is range-based,
  no noise model.
- The camera render is synchronous and costs real-time factor.
- `mode_pr` is ignored (ankles are always pitch/roll joints).
- The harness release used wall time (2.5 s after STAND); with the camera on (RTF 0.3-0.4) that released it ~1 s
  (sim) into the controller's 2 s stand-up. It now uses sim time (2026-10-09).
- `/g1/head_camera` at 960x540, 4-10 Hz costs ~250 ms per frame here (CPU-bound; GPU ~1 % busy, CPU governor
  powersave): RTF 0.27 at 10 Hz, ~0.38 at 4 Hz, ~0.45 at 640x360 10 Hz without viewport/depth. With the camera on,
  Nav2's costmaps stalled every few minutes even behind the relay, so the hospital survey ran without the camera
  (RTF 1) and rendered the camera afterwards with `tools/replay_camera.py`.
- A stopped Isaac (`kill`) can leave the `kit/python` child alive: check `pgrep -f run_sim.py` and use SIGKILL, or
  two sims end up on one domain (double /lowstate -> the robot falls).
