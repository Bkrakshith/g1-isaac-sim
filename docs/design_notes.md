# g1-isaac-sim design notes

This file holds the details behind the README section "How it works". Paths are relative to `g1-isaac-sim/`
unless they name a sibling repository.

## Low-level transport choice

The simulator does not publish `/lowcmd` and `/lowstate` through the Isaac ROS 2 bridge. That bridge would need
the `unitree_hg` messages built for Isaac's Python 3.12. Instead, the simulator talks DDS directly, as Unitree's
`unitree_sim_isaaclab` does. It uses CycloneDDS and the `unitree_sdk2_python` IDL types
(`unitree_hg::msg::dds_::LowState_` / `LowCmd_` on `rt/lowstate` / `rt/lowcmd`). ROS 2 nodes with
`rmw_cyclonedds_cpp` (what `env.sh` sets) see these as `/lowstate` and `/lowcmd` with the type
`unitree_hg/msg/...`, exactly as on the real robot.

These details matter if you change the transport:

- The script `setup_deps.sh` builds the cyclonedds 0.10.5 Python binding against Humble's `libddsc` 0.10.5 into
  `pydeps/`. Isaac bundles the same version in `exts/isaacsim.ros2.core/humble/lib`; the Isaac install is not
  modified. The pip build bakes its temporary build folder into `pydeps/cyclonedds/__library__.py`. The script
  `setup_deps.sh` then rewrites `library_path` to `/opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so`. The rmw and
  the Unitree channel therefore share one libddsc and one DDS domain in the process.
- The ROS 2 node must exist before the Unitree participant. The script `run_sim.py` does not call
  `ChannelFactoryInitialize()` of `unitree_sdk2py`, because that function creates the domain with its own XML
  config, which then fails. Instead, `run_sim.py` attaches plain CycloneDDS writers and readers to the existing
  domain. Pick a network interface with `--net-if` or `CYCLONEDDS_URI`; both channels share it.
- The writer `fast_lowstate.py` serialises `/lowstate` with numpy. At start-up it tests itself byte-for-byte
  against the cyclonedds serialiser. It costs 0.05 ms instead of about 1 ms per message. The numpy buffer mirrors
  Unitree's C struct (2092 bytes, over which the CRC is computed). The XCDR1 wire payload is 2088 bytes, because
  CDR has no struct-end padding.
- The standard messages use the Humble rclpy bundled with Isaac (`exts/isaacsim.ros2.core/humble/rclpy` on
  `sys.path`, libs through `LD_LIBRARY_PATH` from `run.sh`). The extension `isaacsim.ros2.bridge` itself loads
  with Humble. With `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, however, Isaac 6.0 segfaults in OmniGraph on the
  first `app.update()`; with FastDDS it works. The simulator uses no OmniGraph ROS nodes.
- The simulator reads `/g1/mode` (`g1_interfaces/Mode`, transient-local) straight off DDS with the IDL struct in
  `dds_types.py`, because `g1_interfaces` is not built for Isaac's Python.

## Physics layer (`usd/physics_colliders.usd`)

The layer `usd/physics_colliders.usd` holds these items:

- The scene `/World/PhysicsScene` runs at 500 Hz (`physxScene:timeStepsPerSecond`, dt 2 ms) with TGS, CPU
  dynamics and the MBP broadphase.
- Two floors: one slab under the wing and one under the whole Isaac hospital footprint. The hospital's floor
  meshes have no colliders; its walls and furniture have exact triangle-mesh colliders.
- 22 wing walls, 3 m high, from the room rectangles in `hospital-sim-env/isaac/scene/layout.py`, with every
  `DOORWAYS` and `OPENINGS` gap cut out.
- 96 upright oriented boxes around the dressing and OR-suite furniture that has no collider of its own. The
  generator skips items smaller than about 0.3 m, above 1.4 m, or hanging from the ceiling (see
  `usd/physics_colliders.json`).
- All boxes are invisible `Cube` guides with `PhysicsCollisionAPI`.
- The script `tools/check_routes.py` checks that the walkable routes are clear. These are lobby -> SE door ->
  link -> corridor, the whole corridor, and all 7 room doorways. It casts rays at 0.3 / 0.8 / 1.3 m (+-0.25 m
  body width) and checks for a floor under every point.

The session layer in `run_sim.py` adds more at runtime. The script de-instances the Lightwheel carts and trays (USD
instances, 9 roots) and freezes their rigid bodies into static colliders, unless `--dynamic-props` is set. It keeps
`/World/Theatre` out of the freezing. The option `--deactivate` hides prims before physics starts.

## Spawn pose

The simulator uses the same world frame and spawns as the MuJoCo world (`g1-mujoco-sim/worlds/spawns.json`). The
default spawn `corridor_west` is world (8.93, -12.45), yaw 0, at the west end of the surgical corridor, facing
east. The spawn `link` is (18.93, -5.65, -pi/2) and `lobby` is (18.93, -1.95, -pi/2). The wing origin is world
(17.93, -4.45, 0). The pelvis starts 1 cm above the home height 0.784 m (`robot_spec.json` `home_pelvis_height`).
The option `--spawn` also accepts a `robot_stances` name from `hospital-sim-env/objects/placements.yaml` or
`x,y,yaw`.

## Grasp physics (`hand_physics.py`, default on)

The imported Revo2 links are very light. The finger segments weigh 9-50 g. The fingertip "touch" pads weigh 2 g,
sit on fixed joints and have principal inertias down to 3e-9 kg m^2. With the finger colliders on and the stock
4/1 articulation iterations, a fall onto the hands made the solver diverge. That is why the finger collisions used
to be off. The session layer fixes this as follows:

- a mass floor of 5 g and an inertia floor of 1e-6 kg m^2 on every hand link (adds < 30 g per hand);
- finger and palm colliders on, contact offset 2 mm, rest offset 0;
- a max depenetration velocity of 0.5 m/s on the hand links;
- a finger joint velocity limit of 10 rad/s (the real Revo2 closes in about 0.5 s, that is < 4 rad/s);
- articulation solver iterations 16/2.

The table below shows the measurements from `tools/hand_grasp_test.py` (results in `tools/results/`).

| Test | fingers off (old) | finger colliders, no fix | fixed |
|---|---|---|---|
| fall onto the hands (motors off, pushed, 4 s) | max link speed 4.75 m/s | NaN at t = 2.09 s, link speeds 6e11 m/s | max link speed 5.0 m/s, finite |
| in-hand pinch, 7 instruments, 1.5 s hold + 3 s shake | - | 6/7 retained, 5/7 firm | 5-7/7 retained, 5-6/7 firm (<= 7.7 mm slip in the shake), all finite |

The cost is small, and the simulator stays real time. The hospital headless with lidar runs at 500 Hz with
1.7-1.8 ms/step (before the change: 1.3-2.0). The theatre (flat, fixed base, no camera, no lidar) runs at 500 Hz
with 1.2 ms/step.

## Scrub-nurse theatre (`--theatre`)

The layer `usd/theatre_instruments.usd` puts the table, the rack, the surgeon and the 7 instruments from
`hospital-sim-env/objects/` at the poses in `hospital-sim-env/objects/placements.yaml`. These are the same poses
the MuJoCo theatre uses. Every prim under `/World/Theatre` carries the customData `hg1:type` (instrument |
furniture | person | site), which `run_sim.py` uses for `/sim/objects`. The layer also deactivates the two OR-suite
stools and their collider boxes (`remove_scene_props`). The instruments are dynamic, and `run_sim.py` keeps
`/World/Theatre` out of the prop freezing. In PhysX the rack settle gives a tilt <= 5.0 deg
(`tools/rack_settle_test.py`).

The instruments' colliders are box stacks. With CoACD hulls, PhysX pushed the scissors' blades through the rack
guide (`hospital-sim-env/objects/README.md`).

The head camera orientation is fixed. The old camera op `rotateXYZ(90, -90, 0)` looked along d435_link +y, that is
sideways. It is now `(90, 0, -90)`, that is camera -Z = link +x and camera +Y = link +z. The rack image confirms
the fix.

## Lidar (`lidar_warp.py`)

The simulator does not use an RTX lidar. An RTX lidar render product adds GPU memory and render time on an 8 GB
card. The hospital already fills about 7 GB of that card when rendered. The Warp caster builds a BVH from a snapshot
of the static collision geometry at start-up. This snapshot holds the physics-layer walls, floors and furniture
boxes, the static colliders of the Isaac hospital and every other static collider. The caster costs no VRAM when it
runs on `cpu`, and it sees exactly what the robot can collide with.

The pattern is like a Livox MID-360: 360 deg x -7..52 deg, 25 elevation rings x 400 azimuths, with per-scan jitter
(8 pre-generated patterns cycled). Each scan has up to 10 000 points at 10 Hz, like the MuJoCo backend. The topic
`/scan` has 720 rays in the x-y plane of `laser_link` (or `laser_level` with `--scan-level`).
