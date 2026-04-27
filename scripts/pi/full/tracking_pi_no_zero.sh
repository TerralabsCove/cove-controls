#!/usr/bin/env bash
# DEBUG ONLY: bring up the robot stack WITHOUT capturing zero offsets at activation.
# ROS will report raw motor encoder positions, so the URDF reflects the arm's
# actual physical pose. Do NOT execute MoveIt trajectories from this state —
# the planner will see arbitrary start joint angles and could swing the arm.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$REPO_ROOT/config/pi/simple_assembly_cyclonedds_pi.xml"

export DAMIAO_SKIP_ZERO_CAPTURE=1

ros2 launch simple_assembly_tracking_moveit_config robot.launch.py enable_tracker:=false "$@"
