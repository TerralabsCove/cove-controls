#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$REPO_ROOT/config/pi/simple_assembly_cyclonedds_pi.xml"

echo "Press ENTER to execute each waypoint (Ctrl+C to stop)"
while true; do
    read -r -p "→ Press ENTER to execute next waypoint… "
    ros2 topic pub --once /waypoints/execute std_msgs/msg/Empty '{}'
done
