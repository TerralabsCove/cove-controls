#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$REPO_ROOT/config/pi/simple_assembly_cyclonedds_pi.xml}"

CAN_INTERFACE="${CAN_INTERFACE:-can1}"
LOOP_RATE="${LOOP_RATE:-50.0}"
MOTOR_SPEC="${MOTOR_SPEC:-revolute_1_0:DM4340:0x01:0x11:-1,revolute_2_0:DM10010:0x02:0x12:1,revolute_3_0:DM4340:0x03:0x13:-1,revolute_4_0:DM10010:0x04:0x14:1,revolute_5_0:DM4340:0x05:0x15:-1,revolute_6_0:DM10010:0x06:0x16:1,revolute_7_0:DM4340:0x07:0x17:-1}"
EEF_FRAME="${EEF_FRAME:-camera_optical_frame}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${OUTPUT:-$REPO_ROOT/recorded_waypoints/canhat_joint_waypoints_${STAMP}.jsonl}"
LATEST_LINK="$REPO_ROOT/recorded_waypoints/canhat_joint_waypoints_latest.jsonl"

cleanup() {
  kill "${driver_pid:-}" "${rsp_pid:-}" 2>/dev/null || true
  wait "${driver_pid:-}" "${rsp_pid:-}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo "Starting read-only CAN HAT status driver on $CAN_INTERFACE"
echo "This does not enable motors, switch modes, or send motion commands."
ros2 launch damiao_socketcan_driver damiao_socketcan_launch.py \
  can_interface:="$CAN_INTERFACE" \
  motors:="$MOTOR_SPEC" \
  loop_rate:="$LOOP_RATE" \
  control_mode:=status \
  auto_enable:=false \
  switch_mode_on_start:=false \
  disable_on_shutdown:=false &
driver_pid=$!

echo "Starting CAN HAT robot_state_publisher"
ros2 launch simple_assembly_tracking_moveit_config rsp_canhat.launch.py &
rsp_pid=$!

sleep 3

if ! kill -0 "$driver_pid" 2>/dev/null; then
  wait "$driver_pid"
fi
if ! kill -0 "$rsp_pid" 2>/dev/null; then
  wait "$rsp_pid"
fi

mkdir -p "$(dirname "$OUTPUT")"
ln -sfn "$OUTPUT" "$LATEST_LINK"
echo "Recording to $OUTPUT"
echo "Latest link: $LATEST_LINK"

python3 "$REPO_ROOT/scripts/common/record_waypoints.py" \
  --output "$OUTPUT" \
  --target-frame "$EEF_FRAME" \
  "$@"
