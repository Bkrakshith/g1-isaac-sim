#!/usr/bin/env bash
# Hospital_G1 Isaac Sim backend launcher.
#   ./g1-isaac-sim/run.sh --headless [run_sim.py options]   hospital, no window
#   ./g1-isaac-sim/run.sh --gui      [run_sim.py options]   hospital, Isaac window
#   ./g1-isaac-sim/run.sh --import   [--urdf PATH]          (re)import the G1 URDF -> usd/g1_29dof_brainco.usd
#   ./g1-isaac-sim/run.sh --stage                           (re)generate physics layer + root stages + theatre layer (uv)
#   ./g1-isaac-sim/run.sh --stills                          render stills of the G1 in the hospital to renders/
#   ./g1-isaac-sim/run.sh --python script.py [args]         any script with this environment
#   ./g1-isaac-sim/run.sh --help                            this text
# Option for every mode (put it anywhere on the command line):
#   --fixtures TIER   building fixture tier: wing | tier1 | tier2 | full (default: $HG1_FIXTURES, else tier2).
#                     It sets HG1_FIXTURES for make_stage.py (--stage) and for run_sim.py, so that the colliders,
#                     the building hides and the fixture articulations use the same tier. Use the same tier for
#                     the scene build (hospital-sim-env/isaac/scene/build_hospital.py) and for MuJoCo.
#                     wing: surgical wing only (7 doors, 23 joints); tier1: + compartment and stair doors, main
#                     lift bank (38 joints); tier2: + main lift-lobby door, D07 and D34 exits (44 joints);
#                     full: + 8 FD30S hazard-room doors (52 joints).
#                     After a tier change, run --stage again before you run the simulation.
# Spawns for --headless/--gui (run_sim.py --spawn NAME, or x,y,yaw in the world frame): corridor_west (default),
#   link (in the 4 m link), lobby (hospital SE lobby), ward_entrance (ward nurse base), ae_handover (A&E hall).
# Environment: Isaac's bundled ROS 2 Humble libs (no system ROS sourcing; a sourced /opt/ros env is scrubbed),
# RMW = rmw_cyclonedds_cpp (same as env.sh), unitree_sdk2py + CycloneDDS 0.10.5 from g1-isaac-sim/pydeps.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim}"
ROS_LIB="$ISAAC_DIR/exts/isaacsim.ros2.core/humble/lib"

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }

# --fixtures TIER / --fixtures=TIER: remove it from the arguments and export HG1_FIXTURES.
TIER="${HG1_FIXTURES:-tier2}"
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --fixtures)   [ $# -ge 2 ] || { echo "[run.sh] --fixtures needs a tier (wing|tier1|tier2|full)" >&2; exit 2; }
                  TIER="$2"; shift 2 ;;
    --fixtures=*) TIER="${1#--fixtures=}"; shift ;;
    -h|--help)    if [ ${#ARGS[@]} -eq 0 ]; then usage; exit 0; fi; ARGS+=("$1"); shift ;;
    *)            ARGS+=("$1"); shift ;;
  esac
done
case "$TIER" in
  wing|tier1|tier2|full) ;;
  *) echo "[run.sh] unknown fixture tier '$TIER' (use wing, tier1, tier2 or full)" >&2; exit 2 ;;
esac
export HG1_FIXTURES="$TIER"
set -- "${ARGS[@]}"

if [ ! -d "$HERE/pydeps/unitree_sdk2py" ] || [ ! -d "$HERE/pydeps/cyclonedds" ]; then
  echo "[run.sh] installing DDS deps into g1-isaac-sim/pydeps (one time)"; "$HERE/setup_deps.sh"
fi

CLEAN_ENV=(env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u PYTHONHOME
           -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u VIRTUAL_ENV -u ROS_PYTHON_VERSION
           ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
           LD_LIBRARY_PATH="$ROS_LIB" PYTHONPATH="$HERE/pydeps" HG1_FIXTURES="$HG1_FIXTURES"
           OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y)
PY="$ISAAC_DIR/python.sh"
cd "$REPO"
echo "[run.sh] fixture tier: $HG1_FIXTURES" >&2
case "$1" in
  --import) shift; exec "${CLEAN_ENV[@]}" "$PY" "$HERE/import_g1.py" "$@" ;;
  --stage)  shift; env -u LD_LIBRARY_PATH uv run --no-project --with usd-core python "$HERE/make_stage.py" "$@"
            exec env -u LD_LIBRARY_PATH uv run --no-project --with usd-core --with pyyaml python "$HERE/make_theatre_layer.py" ;;
  --stills) shift; exec "${CLEAN_ENV[@]}" "$PY" "$HERE/render_stills.py" "$@" ;;
  --python) shift; exec "${CLEAN_ENV[@]}" "$PY" "$@" ;;
  --gui)    shift; exec "${CLEAN_ENV[@]}" "$PY" "$HERE/run_sim.py" --gui "$@" ;;
  --headless) shift; exec "${CLEAN_ENV[@]}" "$PY" "$HERE/run_sim.py" --headless "$@" ;;
  *) exec "${CLEAN_ENV[@]}" "$PY" "$HERE/run_sim.py" --headless "$@" ;;
esac
