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
