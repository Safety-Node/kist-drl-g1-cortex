#!/usr/bin/env bash
# Source this file to activate the cortex ROS 2 workspace with the same
# CycloneDDS config / bridge domain used by the onboard NX stack.
#
#   source env.sh
#
# After sourcing, ros2 CLI commands share the DDS participant settings of the
# running nodes and can discover the NX onboard nodes on the bridge domain.

_env_sh_dir="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
_ros_distro="${ROS_DISTRO:-humble}"
_ros_setup="/opt/ros/${_ros_distro}/setup.bash"
_cyclonedds_xml="${_env_sh_dir}/config/cyclonedds.xml"
_ws_setup="${_env_sh_dir}/install/setup.bash"

if [[ ! -f "${_ros_setup}" ]]; then
  echo "[env.sh] ROS 2 ${_ros_distro} setup not found at ${_ros_setup}" >&2
  unset _env_sh_dir _ros_distro _ros_setup _cyclonedds_xml _ws_setup
  return 1 2>/dev/null || exit 1
fi
if [[ ! -f "${_cyclonedds_xml}" ]]; then
  echo "[env.sh] CycloneDDS config not found at ${_cyclonedds_xml}" >&2
  unset _env_sh_dir _ros_distro _ros_setup _cyclonedds_xml _ws_setup
  return 1 2>/dev/null || exit 1
fi

# Keep the shared interface submodule initialised.
git -C "${_env_sh_dir}" submodule update --init src/g1_onboard_msgs 2>/dev/null \
  || echo "[env.sh] WARN: could not init g1_onboard_msgs submodule — run: git submodule update --init --recursive" >&2

source "${_ros_setup}"

# Source the local colcon workspace if built.
if [[ -f "${_ws_setup}" ]]; then
  source "${_ws_setup}"
fi

export CYCLONEDDS_URI="file://${_cyclonedds_xml}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# Bridge domain shared with onboard (onboard cyclonedds.xml pins Domain Id=1).
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# Peer = the NX onboard IP (this PC is 192.168.123.222; NX is 192.168.123.164).
export DDS_PEER_IP=${DDS_PEER_IP:-192.168.123.164}

echo "[env.sh] Activated ROS ${ROS_DISTRO:-unknown} with ${RMW_IMPLEMENTATION}"
echo "[env.sh]   ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  DDS_PEER_IP=${DDS_PEER_IP}"
echo "[env.sh]   CYCLONEDDS_URI=${CYCLONEDDS_URI}"

unset _env_sh_dir _ros_distro _ros_setup _cyclonedds_xml _ws_setup
