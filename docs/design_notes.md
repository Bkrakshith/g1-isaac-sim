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
- Four floors: one slab under the wing, one under the lift lobby (to the lift front at x = 20), one under the
  whole Isaac hospital footprint (with the courtyard and the A&E apron north of the facade) and one under the exit
  path. The exterior `DriveFloor` covers the drive, the A&E lane and the apron. The hospital's floor meshes have
  no colliders; its walls and furniture have exact triangle-mesh colliders. The lift car floors and sills are
  fixture colliders (`fixtures.usd`).
- 27 wing walls, 3 m high, from the room rectangles and `PARTITIONS` in `hospital-sim-env/isaac/scene/layout.py`,
  with every `DOORWAYS` and `OPENINGS` gap cut out. The link is 4 m (wing x 0..4) and the PACU is 10 m (wing
  x 4..14), so the old link/PACU wall at x = 2 is gone. The lobby's east side is open: the lift fronts of the
  fixtures layer close it.
- 87 static partitions of the main building from `hospital_fixtures.static_partitions()` (world frame): 48 screen
  pieces, 30 rebuilt-wall pieces, 4 vision-panel glass boxes and 5 link junction boxes. The junction boxes rebuild
  the hospital opening of the link to 2.73 m clear (world x 18.03..20.76).
- 19 exterior boxes from `layout.EXTERIOR_COLLIDERS`: the drive floor, the footway and its ramp, 10 kerbs, the 2
  canopy posts, the 2 sign posts and the ambulance (body and cab).
- Overrides that deactivate the building prims of `hospital_fixtures.building_hide()` for the tier (248 at tier2,
  256 at full, 244 at tier1 and wing). A deactivated prim drops its triangle-mesh colliders too. At tier2 and full,
  the glazing of the main-entrance units `GlassDoor_55` and `GlassDoor2_5` is removed, so the walk-in route is
  open. At a tier that leaves a building door out, the asset's own leaves stay in that doorway with their
  colliders, at the asset's pose.
- An override that adds `/World/HG1_Colliders` to the collision group `/World/Fixtures/CollisionGroups/Static`,
  so the door leaves and lift panels ignore the walls (see "Fire doors and bed lifts").
- 106 upright oriented boxes (tier2) around the dressing and OR-suite furniture that has no collider of its own.
  The generator skips items smaller than about 0.3 m, above 1.4 m, or hanging from the ceiling (see
  `usd/physics_colliders.json`).
- All boxes are invisible `Cube` guides with `PhysicsCollisionAPI`.
- The script `tools/check_routes.py` checks that the walkable routes are clear. In the wing these are lobby ->
  SE door -> 4 m link -> corridor (at x 0.6, 1.3 and 2.2), the whole corridor through the CrossCorridor fire door,
  all room doorways, the final exit to the paved path outside and both lift cars. In the main building these are
  the 17 `BUILDING_ROUTES` (ward round, ambulance bay -> D14 -> resus, walk-in, main lift cars, stairs, exits)
  and 6 more world routes, such as resus -> link -> theatre. It also checks the 5 spawns. The fixtures are at
  their route pose: held doors open, the kept-shut doors (Store, ScrubStore, FinalExit, stair doors, D07, D34,
  hazard-room doors) opened to 90 deg, lift doors open. It casts rays at 0.3 / 0.8 / 1.3 m (+-0.25 m body width)
  and checks for a floor under every point. A route through a door that the tier leaves out is skipped.

The session layer in `run_sim.py` adds more at runtime. The script de-instances the Lightwheel carts and trays (USD
instances, 9 roots) and freezes their rigid bodies into static colliders, unless `--dynamic-props` is set. It keeps
`/World/Theatre` and `/World/Fixtures` out of the freezing. The option `--deactivate` hides prims before physics starts.

## Spawn pose

The simulator uses the same world frame and spawns as the MuJoCo world (`g1-mujoco-sim/worlds/spawns.json`). The
default spawn `corridor_west` is world (8.93, -12.45), yaw 0, at the west end of the surgical corridor, facing
east. The spawn `link` is (18.93, -5.65, -pi/2), inside the 4 m link. The spawn `lobby` is (18.93, -1.95, -pi/2),
in the hospital SE lobby. In the main building, `ward_entrance` is (17.6, 8.5, +pi/2) at the ward nurse base,
facing north up the ward corridor. `ae_handover` is (-29.0, 9.0, 0) in the A&E hall north of the handover bay,
facing east. Both simulators have the same spawns (`g1_config._FALLBACK_SPAWNS` has them if `spawns.json` does
not). The wing origin is world (17.93, -4.45, 0). The pelvis starts 1 cm above the home height 0.784 m (`robot_spec.json` `home_pelvis_height`).
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
runs on `cpu`, and it sees exactly what the robot can collide with. Rigid bodies are not in the snapshot. The
moving fixtures of every group (door leaves, lift panels, button caps) are in a second small mesh. Before a scan,
the caster writes and refits that mesh only when a box moved.

The pattern is like a Livox MID-360: 360 deg x -7..52 deg, 25 elevation rings x 400 azimuths, with per-scan jitter
(8 pre-generated patterns cycled). Each scan has up to 10 000 points at 10 Hz, like the MuJoCo backend. The topic
`/scan` has 720 rays in the x-y plane of `laser_link` (or `laser_level` with `--scan-level`).

## Fire doors and bed lifts (fixtures)

The fixtures layer `hospital-sim-env/isaac/usd/fixtures.usd` is the only hospital-sim-env layer with physics.
The script `hospital-sim-env/isaac/scene/fixtures_usd.py` builds it from `build_spec(layout, tier)` of
`hospital-sim-env/fixtures/hospital_fixtures.py`, for the whole hospital (wing and main building). These choices
matter if you change it:

- Each doorset, lift and call button is one articulation with a fixed base. The `Base` body is fixed to the world
  by a `FixedJoint` without `body0`. The `ArticulationRootAPI` is on the parent Xform, so the patterns
  `/World/Fixtures/Doors/*`, `Doors1/*`, `Lifts/*` and `Calls/*` match the roots.
- One tensor view per spec group. PhysX puts articulations in one view only when their link and joint names are
  the same. The joint prims therefore have generic names, and `fixtures_isaac.py` maps each DOF path to the spec
  name through the customData `hg1:joint`. Single-leaf doors are a separate group (`Doors1`, 1 DOF), so the
  2-leaf group stays homogeneous. If a view is still not homogeneous, `fixtures_isaac.py` falls back to one view
  per articulation.
- The tier is in the customData `hg1:tier` of `/World/Fixtures`. The run time uses it, so the articulations and
  the controller always agree. A different `HG1_FIXTURES` gives a warning.
- The joints have no drives. `set_dof_actuation_forces` adds the controller efforts as joint forces (rad, N m;
  m, N). Self-collision is off, because the leaves of a pair overlap 10 mm at the meeting stiles. The sleep and
  stabilisation thresholds are 0, so held doors never sleep. The step reads and writes the views with `zip` and
  `np.fromiter` over the flat DOF name lists (no per-DOF Python loop).
- The spec puts the hinges on the wall line and the lift panels against the front. The moving parts are in the
  collision group `Moving`, which filters the group `Static`. `Static` holds the fixed fixture parts, the frames
  and thresholds of the asset doors that hold fixture leaves (`BUILDING_FRAMES` in `fixtures_usd.py`), and
  `/World/HG1_Colliders` (`make_stage.py`: walls, floors, the 87 static partitions with the screens and the
  junction boxes). The robot collides with all of them.
- Snag test at the building jambs. The other triangle colliders of the asset are not filtered. The leaves of the
  main building keep 30 mm from the jambs and swing at most 92 deg in asset frames. `tools/fixtures_test.py`
  measures the net contact force on every leaf over a full sweep (push open, alarm close, reverse swing of the
  double-swing leaves). Result: 0.00 N on all 26 (tier2) and 34 (full) leaves. Without the frame filter
  (`--no-building-filter`), the 4 doors on a 25 mm threshold (Central_Store, Ward_Cleaners, Ward_CleanLinen,
  Ward_DirtyUtility) do not move at all: the threshold (top at z 0.0254) is above the 10 mm leaf undercut. The
  frames without a threshold and the D07/D34 wall holes give 0 N. Thus the thresholds must stay in the filter.
- Leaf visuals and colliders. The collider is the spec box (z 0.01 to 2.11), as in MuJoCo and the lidar. In asset
  frames (hole head 2.21 m for the single frames, 2.32 m for D07 and D34), the leaf visual goes up to 4 mm under
  the head, and 6 mm above a threshold. Screen openings (head 2.12 m) need nothing more: the screens of the
  static partitions have a head piece, so `fixtures_usd.py` builds no transom there (the spec has
  `transom: True` for these doors). The wing Compartment door keeps its transom to 3.6 m.
- Signs: blue mandatory signs on both faces of each leaf (except the exits), textures drawn by
  `python scene/fixtures_usd.py --signs`: "AUTOMATIC FIRE DOOR KEEP CLEAR" on held doors, "FIRE DOOR KEEP LOCKED"
  on locked stores, "FIRE DOOR KEEP SHUT" on the others (stairs, linen, pharmacies, wing stores). The exits have
  push bars on the inside face and a green FIRE EXIT sign over the hole on the inside.
- Lifts: per bank (`spec["lift_banks"]`), the front boxes, cars and sills are static colliders. A bulkhead from
  3.0 m to the ceiling is built only where `ceiling_z` > 3.0 (the wing, 3.6 m). The main bank sits under the
  building roof (2.99 m), so it has none.
- The bodies are authored at `initial_q()` (held doors at +-90 deg), with a matching
  `PhysicsJointStateAPI` position. `fixtures_isaac.py` still sets `initial_q()` through the tensor API at start
  and on `/sim/reset`.
- The leaf mass and inertia come from the box of the leaf (about the centre of mass). About the hinge, this
  gives the `inertia_z` of the spec within 1 %.
- The kit door module of the wing doors has a 1.83 m x 2.20 m hole, but the spec leaves span 2.0 m x 2.11 m. The
  outer 85 mm of each closed leaf therefore sits inside the kit frame jamb, and a head stop closes the 90 mm above
  the leaves. The physics walls (`make_stage.py`) reach 0.1 m into each doorway, so the robot sees about 1.8 m
  clear, which matches the visible frame.
- The light curtain uses `raycast_any` of the PhysX scene query at 100 Hz, for every lift of both banks that is not
  closed. The rays run between the leading edges of the panels at their mid-plane, so they cannot hit the panels.
- The lidar does not see rigid bodies in its static snapshot. `run_sim.py` passes the leaf, panel and button-cap
  boxes (`moving_boxes(..., buttons=True)`, from the joint positions) to `WarpLidar.set_dynamic_boxes()` before
  each scan: 44 boxes at tier2, 52 at full (12 triangles each).

Measured on the RTX 3070 (2026-10-13), headless, `tools/fixtures_test.py` (no robot) and `run_sim.py --profile`
(robot standing, real time, 60 s). The CPU governor was `powersave` during these runs.

| Check | tier2 (44 joints) | full (52 joints) |
|---|---|---|
| Views | Doors 12 x 2, Doors1 2 x 1, Lifts 4 x 4, Calls 2 x 1 | Doors 12 x 2, Doors1 10 x 1, Lifts 4 x 4, Calls 2 x 1 |
| 80 N push on every kept-shut leaf, then release | 11 leaves open to the 92/100 deg stop, close in <= 6.8 s | 19 leaves, the same |
| Fire alarm to all leaves under 2 deg | 6.4 s (sim) | 6.4 s |
| Net contact force on the leaves over all sweeps | 0.00 N (26 leaves) | 0.00 N (34 leaves) |
| LobbyCall / MainCall (6 N on the button body): open / dwell / close | 3.0 s / 7.0 s / 3.6 s, both banks | the same |
| Obstacle in the Lift2 and MainLift2 doorways | doors stay open for 12 s; close 5.1 s after it goes | the same |
| Lidar ray to the closed E_Compartment / CrossCorridor / D34_Exit | 1.27 / 2.48 / 1.17 m (door faces 1.27 / 2.48 / 1.17 m) | the same |
| Fixture step (test, no robot) | 0.32 ms mean, 0.58 ms p99 | 0.32 ms mean, 0.56 ms p99 |
| Physics step (test, no robot) | 0.73 ms mean, 1.03 ms p99 | 0.70 ms mean, 0.96 ms p99 |
| Loop with the robot (`--profile`) | 500 Hz real time; busy 1.67-1.76 ms mean, p50 1.40-1.52 ms, p99 4.8-5.2 ms; fixtures 0.19-0.20 ms, physics 0.91-0.96 ms | 500 Hz real time; busy 1.69-1.86 ms mean, p50 1.60 ms, p99 4.7-7.3 ms; fixtures 0.19-0.21 ms, physics 0.96-1.02 ms |
| Lidar cloud + scan (warp, cuda:0), static only / with moving boxes | 1.74 / 1.81 ms mean | 1.64 / 1.70 ms mean |
| `set_dynamic_boxes`, no box moved / a box moved | 0.12 / 0.18 ms | 0.14 / 0.19 ms |


ROS 2 check at tier2 (`tools/fixtures_ros_check.py`, robot at wing (1.5, -8) facing the CrossCorridor door): ALL PASS
(22/22). The alarm closes the 14 doors in 6.4 s (sim). `/scan` ahead goes from 17.47 m (open) to 2.45 m (closed).
`/sim/lift/call` and `/sim/lift/call_main` answer `success=False` during the alarm. After it, they open Lift1 and
MainLift1 in 3.1 s, which dwell 7.0 s and close. `/sim/fixtures/press` opens each of the 4 lifts and `MainCall` opens
a main-bank car. An unknown button name is ignored.
