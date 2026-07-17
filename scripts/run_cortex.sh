#!/usr/bin/env bash
# Launch the full cortex node graph on the workstation PC.
# Assumes the workspace is built and env.sh has configured ROS + DDS.
set -euo pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )/.." && pwd )"
cd "${_here}"

# shellcheck disable=SC1091
source env.sh

if [[ ! -f install/setup.bash ]]; then
  echo "[run_cortex] workspace not built — run: colcon build --symlink-install" >&2
  exit 1
fi
source install/setup.bash

exec ros2 launch cortex_bringup cortex.launch.py "$@"
