# Cove ROS Setup

This repository is the reproducible ROS 2 workspace for the full setup:

- Pi side: real hardware bringup for the 7-DOF arm and the turret
- Ubuntu side: RViz / MoveIt client launchers and DDS configs

The source tree here is staged from the known-good backup snapshot, including the working `damiao_driver` hardware fixes.

## Layout

- `src/`: ROS packages for arm, turret, MoveIt, and hardware driver
- `scripts/pi/`: Pi launch helpers
- `scripts/ubuntu/`: Ubuntu launch helpers
- `config/pi/`: Pi CycloneDDS configs
- `config/ubuntu/`: Ubuntu CycloneDDS configs

## Prerequisites

- ROS 2 Jazzy installed on both machines
- `colcon`, `rosdep`, and `python3-colcon-common-extensions` installed
- ROS 2 control development packages installed on both machines:

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

- `sshpass` installed if you want to use `scripts/pi/dual_launch.sh`
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
./scripts/pi/simple_assembly_pi.sh
```

Ubuntu:

```bash
cd ~/cove_ros_setup
./scripts/ubuntu/simple_assembly_rviz.sh
```

### Turret

Pi:

```bash
cd ~/cove_ros_setup
./scripts/pi/robot_launch.sh
```

Ubuntu:

```bash
cd ~/cove_ros_setup
./scripts/ubuntu/rviz_launch.sh
```

### Turret Visual Only

Ubuntu:

```bash
cd ~/cove_ros_setup
./scripts/ubuntu/show_robot.sh
```

Pi:

```bash
cd ~/cove_ros_setup
./scripts/pi/show_robot.sh
```

## Dual Launch Script

`scripts/pi/dual_launch.sh` is intentionally sanitized. It does not store credentials in git. Export the required variables first:

```bash
export REMOTE_USER=parallels
export REMOTE_HOST=<ubuntu-ip-or-hostname>
export REMOTE_PASS=<ubuntu-ssh-password>
export REMOTE_REPO=/home/parallels/cove_ros_setup
export PI_IP=<pi-tailscale-ip>
./scripts/pi/dual_launch.sh
```

## Notes

- The repo is meant to be built directly as the workspace root.
- Do not commit `build/`, `install/`, or `log/`.
- The working hardware driver state comes from the Pi backup snapshot, not the stale local copy that lacked the crash fix.
