#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

REMOTE_USER="${REMOTE_USER:?Set REMOTE_USER to the Ubuntu username}"
REMOTE_HOST="${REMOTE_HOST:?Set REMOTE_HOST to the Ubuntu host or IP}"
REMOTE_PASS="${REMOTE_PASS:?Set REMOTE_PASS to the Ubuntu SSH password}"
REMOTE_REPO="${REMOTE_REPO:?Set REMOTE_REPO to the repo path on Ubuntu}"
PI_IP="${PI_IP:?Set PI_IP to the Pi Tailscale IP}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
REMOTE_DISPLAY="${REMOTE_DISPLAY:-:0}"

cleanup() {
  if [[ -n "${ROBOT_PID:-}" ]]; then
    kill "$ROBOT_PID" 2>/dev/null || true
    wait "$ROBOT_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$REPO_ROOT/install/setup.bash"
set -u

export ROS_DOMAIN_ID
export CYCLONEDDS_URI="file://$REPO_ROOT/config/pi/cyclonedds_network.xml"

ros2 launch moveitturrettest robot.launch.py &
ROBOT_PID=$!

sleep 8

sshpass -p "$REMOTE_PASS" ssh -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}" \
  "cat > /tmp/cyclonedds_remote.xml <<'XMLEOF'
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name=\"tailscale0\" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer Address=\"$PI_IP\" />
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
XMLEOF"

sshpass -p "$REMOTE_PASS" ssh -t -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}" \
  "export XDG_RUNTIME_DIR=/run/user/\$(id -u) \
   && export DISPLAY=$REMOTE_DISPLAY \
   && export XAUTHORITY=\$(ls /run/user/\$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1) \
   && export CYCLONEDDS_URI=file:///tmp/cyclonedds_remote.xml \
   && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
   && source /opt/ros/$ROS_DISTRO/setup.bash \
   && source $REMOTE_REPO/install/setup.bash \
   && ros2 launch moveitturrettest moveit_rviz.launch.py"
