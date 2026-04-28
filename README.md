# Cove ROS Setup

This repository is the reproducible ROS 2 workspace for the full setup:

- Pi side: real hardware bringup for the 7-DOF arm and the turret
- Ubuntu side: RViz / MoveIt client launchers and DDS configs

The source tree here is staged from the known-good backup snapshot, including the working `damiao_driver` hardware fixes.

## Layout

- `src/`: ROS packages for arm, turret, MoveIt, and hardware driver
- `scripts/pi/` and `scripts/ubuntu/`: launch helpers, grouped by which URDF they bring up:
  - `simple/`: the original `simple_assembly` arm URDF (no camera mount)
  - `full/`: the `full_assembly` arm URDF (camera mount included; used by all `plan_to_tag` and `waypoint_runner` scripts via `simple_assembly_tracking`)
  - `turret/`: the `simple_turret` / `moveitturrettest` URDF
  - `util/`: URDF-agnostic helpers (camera viewers, topic listers, `tf2_echo` wrappers)
- `config/pi/`: Pi CycloneDDS configs
- `config/ubuntu/`: Ubuntu CycloneDDS configs

## Prerequisites

- ROS 2 Jazzy installed on both machines
- `colcon`, `rosdep`, and `python3-colcon-common-extensions` installed
- ROS 2 control development packages installed on both machines:

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

- `sshpass` installed if you want to use `scripts/pi/turret/dual_launch.sh`
- Pi user added to `dialout` for motor serial access:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership.

## Clone And Build

Clone this repo on both machines, then build it in place:

```bash
git clone <your-remote-url> ~/cove_ros_setup
cd ~/cove_ros_setup
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Helper scripts are also included:

```bash
./scripts/install_rosdeps.sh
./scripts/build_workspace.sh
```

If `rosdep` has never been initialized on the machine:

```bash
sudo rosdep init
rosdep update
```

If the build fails with `hardware_interfaceConfig.cmake` not found, the ROS 2 control packages above are missing.

## DDS Configuration

The XML files in `config/` contain the working snapshot IPs from the backed-up system. Before first launch on new hardware, update them to match the current Tailscale or network addresses.

Arm pair:

- `config/pi/simple_assembly_cyclonedds_pi.xml`
- `config/ubuntu/simple_assembly_cyclonedds_ubuntu.xml`

Turret pair:

- `config/pi/cyclonedds_network.xml`
- `config/ubuntu/cyclonedds_pi.xml`

At minimum, set:

- the local interface address or interface name for the machine running that file
- the peer address for the opposite machine

## Tailscale SSH And Routing

Known-good Pi targets from the current setup:

- LAN SSH fallback: `terralabscove@10.66.10.30`
- Pi Tailscale IP: `100.88.175.52`
- Ubuntu VM Tailscale IP: `100.86.104.75`

If Tailscale SSH or normal SSH to the Pi Tailscale IP times out, first verify whether Linux is routing the Pi Tailscale IP through `tailscale0`:

```bash
ip -brief addr show tailscale0
ip route get 100.88.175.52
```

Healthy output should show `tailscale0` with the Ubuntu VM's `100.x` address and the route to `100.88.175.52` using `tailscale0`. If the route instead goes through the normal network interface, such as `enp0s5` via `10.211.55.1`, the local Tailscale kernel route state is stale. In that broken state, Tailscale control-plane pings may still work, but TCP/SSH to `100.88.175.52:22` can time out.

Immediate recovery on the Ubuntu VM:

```bash
sudo systemctl restart tailscaled
sudo tailscale up --timeout 5s
ip -brief addr show tailscale0
ip route get 100.88.175.52
```

Or run the automated helper from the Ubuntu VM:

```bash
./scripts/ubuntu/util/fix_ssh.sh
```

The helper restarts local Tailscale, disables Tailscale SSH on the Pi through the LAN fallback, installs the NetworkManager dispatcher hook described below, and verifies SSH over the Pi Tailscale IP. Override defaults with `PI_USER`, `PI_PASS`, `PI_LAN_HOST`, `PI_TS_IP`, or `NM_IFACE` if the setup changes.

The LAN path can be used while Tailscale routing is broken:

```bash
sshpass -p terralabscove ssh -o StrictHostKeyChecking=no terralabscove@10.66.10.30
```

The Pi should use normal OpenSSH for this setup. Keep Tailscale SSH disabled on the Pi so `ssh terralabscove@100.88.175.52` goes directly to `sshd` and does not trigger the Tailscale browser re-auth check:

```bash
sshpass -p terralabscove ssh -o StrictHostKeyChecking=no terralabscove@10.66.10.30 \
  "echo 'terralabscove' | sudo -S tailscale set --ssh=false"
```

If Tailscale SSH is intentionally enabled later, the browser prompt is controlled by the tailnet SSH access policy. Use an `accept` rule instead of a `check` rule in the Tailscale admin console to avoid the extra web authentication step.

For a more permanent fix, make sure the local Tailscale daemon is enabled at boot and let the current user operate Tailscale without sudo:

```bash
sudo systemctl enable --now tailscaled
sudo tailscale set --operator=$USER
```

If the VM still loses the `tailscale0` IPv4 address or the `100.64.0.0/10` route after Wi-Fi changes, VM suspend/resume, or campus network transitions, add a small NetworkManager dispatcher hook on the Ubuntu VM to restart Tailscale whenever the primary interface comes back up:

```bash
sudo tee /etc/NetworkManager/dispatcher.d/90-restart-tailscale >/dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

IFACE="$1"
STATE="$2"

case "$STATE" in
  up|connectivity-change)
    if [[ "$IFACE" == "enp0s5" ]]; then
      systemctl restart tailscaled
    fi
    ;;
esac
EOF
sudo chmod +x /etc/NetworkManager/dispatcher.d/90-restart-tailscale
```

After installing the hook, unplug/reconnect the network or run `sudo systemctl restart NetworkManager`, then re-check:

```bash
ip -brief addr show tailscale0
ip route get 100.88.175.52
sshpass -p terralabscove ssh -o StrictHostKeyChecking=no terralabscove@100.88.175.52 true
```

## Launch

### 7-DOF Arm

Pi:

```bash
cd ~/cove_ros_setup
./scripts/pi/simple/simple_assembly_pi.sh
```

Ubuntu:

```bash
cd ~/cove_ros_setup
./scripts/ubuntu/simple/simple_assembly_rviz.sh
```

### Turret

Pi:

```bash
cd ~/cove_ros_setup
./scripts/pi/turret/robot_launch.sh
```

Ubuntu:

```bash
cd ~/cove_ros_setup
./scripts/ubuntu/turret/rviz_launch.sh
```

### Turret Visual Only

Ubuntu:

```bash
cd ~/cove_ros_setup
./scripts/ubuntu/turret/show_robot.sh
```

Pi:

```bash
cd ~/cove_ros_setup
./scripts/pi/turret/show_robot.sh
```

## Dual Launch Script

`scripts/pi/turret/dual_launch.sh` is intentionally sanitized. It does not store credentials in git. Export the required variables first:

```bash
export REMOTE_USER=parallels
export REMOTE_HOST=<ubuntu-ip-or-hostname>
export REMOTE_PASS=<ubuntu-ssh-password>
export REMOTE_REPO=/home/parallels/cove_ros_setup
export PI_IP=<pi-tailscale-ip>
./scripts/pi/turret/dual_launch.sh
```

---

## AprilTag Plan-to-Tag

Moves the arm's camera to a detected AprilTag using MoveIt.

### Architecture

- The Pi runs the full robot stack (MoveIt `move_group`, ros2_control, camera, AprilTag detector) plus the `plan_to_tag` node.
- Ubuntu runs RViz for visualisation and the MotionPlanning panel.
- The `plan_to_tag` node solves goals using a MoveIt `PositionConstraint` on `camera_optical_frame` — no IK pre-solve, no orientation fallback. The MoveIt planner picks any valid joint configuration that places the camera at the target XYZ.
- When `orient_to_tag:=true` (default), an `OrientationConstraint` is added that aligns the camera Z-axis anti-parallel to the tag Z-axis (arm approaches perpendicular to the tag plane). Roll around the approach axis is free.

### Workflow

1. **Capture** — freezes the current tag TF position.
2. **Plan Captured** — sends a MoveGroup goal. In execute mode the arm moves; in preview mode RViz shows the ghost robot.

Capture and Plan buttons appear as interactive markers in RViz. Alternatively publish to the topics directly:

```bash
ros2 topic pub --once /apriltag/capture_tag std_msgs/msg/Empty '{}'
ros2 topic pub --once /apriltag/plan_captured_tag std_msgs/msg/Empty '{}'
```

### Scripts

| Script | Mode | Orientation |
|---|---|---|
| `scripts/pi/full/plan_to_tag_pi.sh` | preview (no execute) | normal to tag |
| `scripts/pi/full/plan_to_tag_pi_execute.sh` | execute | normal to tag |
| `scripts/pi/full/plan_to_tag_pi_position_only.sh` | preview | position only |
| `scripts/pi/full/plan_to_tag_pi_execute_position_only.sh` | execute | position only |

Ubuntu (RViz):

```bash
./scripts/ubuntu/full/plan_to_tag_rviz.sh
```

### Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `execute` | `false` | If true, arm moves; if false, plan preview only |
| `approach_distance` | `0.0` | Standoff from tag face in metres (0 = go to tag) |
| `orient_to_tag` | `true` | Constrain camera to face the tag perpendicularly |
| `orientation_tolerance` | `0.2` | Approach axis tolerance in radians (~11°) |
| `goal_tolerance` | `0.04` | Position goal sphere radius in metres |
| `velocity_scale` | `0.3` | MoveIt velocity scaling (0–1) |

### RViz Setup

The MotionPlanning panel must have **External Comm.** enabled (already set in `plan_to_tag.rviz`). The node pushes the planned goal state to RViz after each successful plan so the ghost robot preview is always visible.

---

## Waypoint Runner

Steps the arm through a predefined list of Cartesian positions, one at a time, requiring manual approval for each motion.

### Defining Waypoints

Edit the `WAYPOINTS` list near the top of `src/simple_assembly_tracking/nodes/waypoint_runner.py`:

```python
WAYPOINTS = [
    ("home",    0.30,  0.00, 0.45),   # (name, x, y, z) in metres, root frame
    ("point_a", 0.35,  0.10, 0.35),
]
```

Waypoints are also stored in `locations.txt` at the repo root (one per line, `tf2_echo` Translation format). When you record new positions with `show_eef_pose.sh`, copy the lines into `locations.txt` then update `WAYPOINTS` in the node.

### Z Limit

The node refuses to execute any waypoint whose Z is below `min_z` (default `0.0565 m` — the height of `revolute_1_0`, the first base joint, in the root frame). Override at launch:

```bash
./scripts/pi/full/waypoint_runner_pi.sh min_z:=0.10
```

### Scripts

Pi (robot + waypoint node):

```bash
./scripts/pi/full/waypoint_runner_pi.sh
```

Ubuntu (RViz):

```bash
./scripts/ubuntu/full/plan_to_tag_rviz.sh
```

### Approving Each Motion

**Option A — RViz button:** click the green **EXECUTE** button in the 3D view. The button label shows the next waypoint name and index.

**Option B — Terminal on the Pi:** run this in a second terminal alongside `waypoint_runner_pi.sh`:

```bash
./scripts/pi/full/waypoint_approve.sh
```

Press **Enter** to execute each waypoint.

**Option C — Topic:**

```bash
ros2 topic pub --once /waypoints/execute std_msgs/msg/Empty '{}'
```

### RViz Visualisation

All waypoints appear as labelled spheres. The current target is bright green; completed/future waypoints are grey. After each successful motion the index advances automatically and loops back to the start.

---

## Utilities

### Live End-Effector Position

Stream the Cartesian position of the camera frame in the terminal (with correct DDS environment):

```bash
./scripts/ubuntu/util/show_eef_pose.sh                        # root → camera_optical_frame
./scripts/ubuntu/util/show_eef_pose.sh root wrist_link        # root → wrist_link
```

Output is the `Translation: [x, y, z]` line from `tf2_echo`, filtered for readability.

---

## Notes

- The repo is meant to be built directly as the workspace root.
- Do not commit `build/`, `install/`, or `log/`.
- The working hardware driver state comes from the Pi backup snapshot, not the stale local copy that lacked the crash fix.
- DOUBLE CHECK IF ANY CAN CONNECTORS ARE FLIPPED. SOMETIMES THEY CAME FLIPPED
