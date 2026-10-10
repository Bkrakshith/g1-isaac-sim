#!/usr/bin/env bash
# Hospital_G1 Isaac Sim backend launcher.
#   ./g1-isaac-sim/run.sh --headless [run_sim.py options]   hospital, no window
#   ./g1-isaac-sim/run.sh --gui      [run_sim.py options]   hospital, Isaac window
#   ./g1-isaac-sim/run.sh --import   [--urdf PATH]          (re)import the G1 URDF -> usd/g1_29dof_brainco.usd
#   ./g1-isaac-sim/run.sh --stage                           (re)generate physics layer + root stages + theatre layer (uv)
#   ./g1-isaac-sim/run.sh --stills                          render stills of the G1 in the hospital to renders/
#   ./g1-isaac-sim/run.sh --python script.py [args]         any script with this environment
# Environment: Isaac's bundled ROS 2 Humble libs (no system ROS sourcing; a sourced /opt/ros env is scrubbed),
# RMW = rmw_cyclonedds_cpp (same as env.sh), unitree_sdk2py + CycloneDDS 0.10.5 from g1-isaac-sim/pydeps.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim}"
ROS_LIB="$ISAAC_DIR/exts/isaacsim.ros2.core/humble/lib"

if [ ! -d "$HERE/pydeps/unitree_sdk2py" ] || [ ! -d "$HERE/pydeps/cyclonedds" ]; then
  echo "[run.sh] installing DDS deps into g1-isaac-sim/pydeps (one time)"; "$HERE/setup_deps.sh"
fi

CLEAN_ENV=(env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u PYTHONHOME
           -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u VIRTUAL_ENV -u ROS_PYTHON_VERSION
           ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
           LD_LIBRARY_PATH="$ROS_LIB" PYTHONPATH="$HERE/pydeps"
           OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y)
PY="$ISAAC_DIR/python.sh"
cd "$REPO"
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
