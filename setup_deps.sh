#!/usr/bin/env bash
# One-time: install the DDS dependencies for g1-isaac-sim/run_sim.py into g1-isaac-sim/pydeps (Isaac's
# python 3.12), without touching the Isaac install itself.
#   - cyclonedds 0.10.5 python binding, built against ROS 2 Humble's own libddsc 0.10.5
#     (/opt/ros/humble), so Isaac's ROS 2 bridge (rmw_cyclonedds) and the Unitree DDS channel share
#     one libddsc in-process and speak the exact same CycloneDDS version as the ROS 2 side.
#   - unitree_sdk2_python (IDL types unitree_hg LowCmd_/LowState_, CRC, ChannelPublisher/Subscriber).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim}"
BUILD="${TMPDIR:-/tmp}/hg1_isaac_deps"
mkdir -p "$BUILD/cdh"
ln -sfn /opt/ros/humble/include "$BUILD/cdh/include"
ln -sfn /opt/ros/humble/lib/x86_64-linux-gnu "$BUILD/cdh/lib"
ln -sfn /opt/ros/humble/bin "$BUILD/cdh/bin"
PY=(env -u LD_LIBRARY_PATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u PYTHONPATH "$ISAAC_DIR/python.sh")
CYCLONEDDS_HOME="$BUILD/cdh" "${PY[@]}" -m pip install --no-deps --target "$HERE/pydeps" cyclonedds==0.10.5
# the pip build bakes the temporary build directory into library_path; point it at the real Humble libddsc
printf "in_wheel = False\nlibrary_path = '/opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so'\n" > "$HERE/pydeps/cyclonedds/__library__.py"
if [ ! -d "$BUILD/unitree_sdk2_python" ]; then
  git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git "$BUILD/unitree_sdk2_python"
fi
rm -rf "$HERE/pydeps/unitree_sdk2py"
cp -r "$BUILD/unitree_sdk2_python/unitree_sdk2py" "$HERE/pydeps/"
(cd "$BUILD/unitree_sdk2_python" && git log -1 --format='unitree_sdk2_python %H' > "$HERE/pydeps/UNITREE_SDK2PY_VERSION")
echo "deps installed in $HERE/pydeps"
