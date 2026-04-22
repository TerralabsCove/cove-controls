# COVE Kiosk Agent Handoff

## Purpose

This document is for another coding agent that needs to continue the COVE kiosk integration and connect it cleanly to the actual robot.

The goal is fast navigation:

- where the frontend lives
- where ROS state and HTTP behavior live
- where simulated motion lives
- where real hardware integration lives
- which files to edit for specific tasks

This is not an operator doc. It is a codebase map for implementation work.

## Current System Model

Current control path:

```text
Browser UI
  -> HTTP API
  -> cove_kiosk_bridge
  -> FollowJointTrajectory action client
  -> /arm_controller/follow_joint_trajectory
  -> ros2_control
  -> fake hardware or real hardware
  -> joint_states
  -> robot_state_publisher / RViz / MoveIt consumers
```

There are now two major modes:

- simulation mode via `mock_components/GenericSystem`
- real hardware mode via `damiao_driver/DamiaoHardwareInterface`

## High-Level Package Map

### `src/cove_kiosk_bridge`

Owns kiosk backend behavior.

Use this package when you need to change:

- HTTP endpoints
- queueing
- order lifecycle
- slot assignment
- robot execution sequence
- backend/frontend served page

Most important files:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/kiosk_bridge_node.py`
- `src/cove_kiosk_bridge/cove_kiosk_bridge/motion_controller.py`
- `src/cove_kiosk_bridge/web/index.html`
- `src/cove_kiosk_bridge/launch/cove_kiosk.launch.py`

### `src/simple_assembly_moveit_config`

Owns MoveIt, ros2_control, simulation launch, RViz launch, and robot config for the assembly arm.

Use this package when you need to change:

- fake vs real hardware selection
- MoveIt launch behavior
- controller configuration
- RViz simulation launch
- planning group configuration

Most important files:

- `src/simple_assembly_moveit_config/config/simple_assembly.ros2_control.xacro`
- `src/simple_assembly_moveit_config/config/simple_assembly.urdf.xacro`
- `src/simple_assembly_moveit_config/config/ros2_controllers.yaml`
- `src/simple_assembly_moveit_config/config/moveit_controllers.yaml`
- `src/simple_assembly_moveit_config/config/simple_assembly.srdf`
- `src/simple_assembly_moveit_config/launch/sim_kiosk.launch.py`
- `src/simple_assembly_moveit_config/launch/robot.launch.py`

### `src/simple_assembly`

Owns the base robot description and older motion scripts.

Use this package when you need to change:

- robot meshes / URDF base assets
- legacy motion-script logic
- reference positions and geometry

Most important areas:

- `src/simple_assembly/urdf`
- `src/simple_assembly/motion_scripts`

### `src/damiao_driver`

Owns real actuator / motor hardware integration.

Use this package when you need to change:

- hardware interface behavior
- low-level motor command flow
- real robot driver integration

Most important files:

- `src/damiao_driver/src/damiao_node.cpp`
- `src/damiao_driver/include/...`
- driver launch/config files under `src/damiao_driver/launch` and `src/damiao_driver/config`

## What Owns What

## Frontend UI

Primary source:

- `Cove Matcha Kiosk - standalone.html`

Served package copy:

- `src/cove_kiosk_bridge/web/index.html`

Sync script:

- `scripts/update_kiosk_bundle.py`

Important detail:

- the root HTML file is the editable bundle source
- the package copy is what the ROS kiosk bridge serves
- if the frontend changes, re-run `scripts/update_kiosk_bundle.py`

## HTTP API and Queue Logic

Main file:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/kiosk_bridge_node.py`

This file owns:

- `GET /api/state`
- `POST /api/orders`
- `POST /api/operator/error`
- `POST /api/operator/reset`
- queue worker thread
- order status transitions
- slot assignment
- snapshot payloads consumed by the frontend

If the task is:

- change API payloads
- add websocket support
- add telemetry to the operator panel
- change order lifecycle behavior

edit this file first.

## Motion Sequencing

Main file:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/motion_controller.py`

This file owns:

- joint list used by the kiosk bridge
- waypoint definitions
- pouring / moving sequences
- direct `FollowJointTrajectory` action client behavior
- simulation fallback timing

If the task is:

- change waypoint positions
- change sequence order
- add gripper handling
- move from fixed poses to perception-driven targets
- switch from trajectory controller to another action/service

edit this file first.

## Fake vs Real Hardware Selection

Primary files:

- `src/simple_assembly_moveit_config/config/simple_assembly.ros2_control.xacro`
- `src/simple_assembly_moveit_config/config/simple_assembly.urdf.xacro`

This is where:

- `mock_components/GenericSystem` is selected for sim
- `damiao_driver/DamiaoHardwareInterface` is selected for real hardware
- the `use_fake_hardware` argument is threaded into the robot description

If the task is:

- switch sim vs real behavior
- add hardware parameters
- change serial/device config injection

edit these files first.

## ros2_control Controller Configuration

Primary file:

- `src/simple_assembly_moveit_config/config/ros2_controllers.yaml`

This file owns:

- `arm_controller`
- `joint_state_broadcaster`
- joint list for the controller
- controller update behavior

If the task is:

- change controller type
- adjust joint list
- add a separate gripper controller
- tune ros2_control side behavior

edit this file first.

## MoveIt Controller Mapping

Primary file:

- `src/simple_assembly_moveit_config/config/moveit_controllers.yaml`

This file tells MoveIt which trajectory controller it can use.

If MoveIt plans but cannot execute, check this file early.

## MoveIt Arm Group Definition

Primary file:

- `src/simple_assembly_moveit_config/config/simple_assembly.srdf`

This file owns:

- the `arm` planning group
- the joint list in the group
- named group states

If the task is:

- add/remove joints from planning
- create named states
- fix planning-group mismatch

edit this file first.

## Launch Entry Points

### Kiosk-only backend

- `src/cove_kiosk_bridge/launch/cove_kiosk.launch.py`

### Main robot launch

- `src/simple_assembly_moveit_config/launch/robot.launch.py`

### Visible website-to-RViz simulation

- `src/simple_assembly_moveit_config/launch/sim_kiosk.launch.py`

Use `sim_kiosk.launch.py` when testing website-driven simulated motion.

## Run Helpers

Primary helper:

- `scripts/ubuntu/run_cove_kiosk.sh`

Modes:

- `sim`
- `rviz-sim`
- `full`
- `smoke-test`

If another agent needs reproducible execution commands, start here.

## Common Edit Recipes

## Task: change kiosk text, visuals, or browser behavior

Edit:

- `Cove Matcha Kiosk - standalone.html`

Then sync:

- `python3 scripts/update_kiosk_bundle.py`

Affected served copy:

- `src/cove_kiosk_bridge/web/index.html`

## Task: add or change API fields

Edit:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/kiosk_bridge_node.py`

Then update:

- frontend JS in `Cove Matcha Kiosk - standalone.html`

## Task: change robot motion sequence

Edit:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/motion_controller.py`

Specifically:

- `JOINT_NAMES`
- `WAYPOINTS`
- `POURING_SEQUENCE`
- `MOVING_SEQUENCE`

## Task: connect website flow to real robot instead of fake hardware

Primary edit points:

- `src/simple_assembly_moveit_config/config/simple_assembly.ros2_control.xacro`
- `src/simple_assembly_moveit_config/config/simple_assembly.urdf.xacro`
- `src/simple_assembly_moveit_config/launch/robot.launch.py`
- `src/damiao_driver/...`

Likely work:

- ensure real hardware mode is selected
- ensure correct serial/device params are passed
- ensure real controller interfaces are active
- verify `arm_controller` action server is backed by actual hardware

## Task: move from fixed waypoints to perception or dynamic targets

Primary edit point:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/motion_controller.py`

Likely supporting files:

- `src/cove_vision/...`
- MoveIt config under `src/simple_assembly_moveit_config/config`

## Task: make frontend updates push-based instead of polling

Primary edit point:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/kiosk_bridge_node.py`

Secondary edit point:

- `Cove Matcha Kiosk - standalone.html`

Likely change:

- add websocket or SSE endpoint
- remove/set lower reliance on periodic `/api/state` polling

## Task: add a physical gripper or dispenser controller

Primary edit points:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/motion_controller.py`
- `src/simple_assembly_moveit_config/config/ros2_controllers.yaml`
- `src/simple_assembly_moveit_config/config/moveit_controllers.yaml`
- possibly `src/simple_assembly_moveit_config/config/simple_assembly.srdf`

## Integration Risks / Gotchas

## 1. Port conflicts

The kiosk bridge binds an HTTP port.

If multiple stacks are running, the bridge can fail with:

- `OSError: [Errno 98] Address already in use`

When that happens, the browser may be talking to an older backend than the current RViz stack.

## 2. Frontend source vs served copy

Do not edit only:

- `src/cove_kiosk_bridge/web/index.html`

without also considering:

- `Cove Matcha Kiosk - standalone.html`
- `scripts/update_kiosk_bundle.py`

The bundle sync path matters.

## 3. Joint-count mismatches

The MoveIt/ros2_control arm group is 7-DOF.

If another agent changes joints in:

- `motion_controller.py`
- `ros2_controllers.yaml`
- `simple_assembly.srdf`

they must stay aligned.

## 4. Real-vs-sim confusion

The repo now supports both fake and real hardware.

Before debugging motion:

- confirm which launch file is being used
- confirm whether `use_fake_hardware` is true or false

## 5. RViz interactive markers are not the control surface

The website triggers backend trajectories.

RViz is a visualization target for robot state and planning visualization, not the primary control surface.

## Recommended Commands

## Build

```bash
scripts/build_workspace.sh --packages-select simple_assembly cove_kiosk_bridge simple_assembly_moveit_config
```

## Kiosk API smoke test

```bash
scripts/ubuntu/run_cove_kiosk.sh smoke-test --build
```

## Kiosk-only timer sim

```bash
scripts/ubuntu/run_cove_kiosk.sh sim --build
```

## Visible website-to-RViz simulation

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build
```

Alternate port:

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build --port 8092
```

## Suggested Next Work For Real Robot Integration

1. Replace fixed pick/place waypoints with calibrated robot poses from real-world measurements.
2. Add explicit gripper/dispenser actuation instead of simulated sleep-only gripper steps.
3. Pass real hardware connection parameters through launch rather than hardcoding them in xacro.
4. Add backend telemetry for controller state, action status, and hardware faults.
5. Add stale-process cleanup or port-management logic to the run helper.
6. Add a websocket event stream so the UI no longer polls state every second.

## Related Docs

- `docs/cove_kiosk_ros_integration.md`
- `docs/cove_kiosk_operator_quickstart.md`
