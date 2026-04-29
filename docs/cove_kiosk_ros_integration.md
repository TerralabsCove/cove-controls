# COVE Kiosk ROS Integration

## Summary

This repository now supports a website-to-Python-script control flow for the COVE kiosk.

The frontend kiosk page can now:

- submit orders to a ROS-backed kiosk bridge
- start the Python script that moves the arm
- show live script activity on the kiosk frontend

This is not just a timer-based UI demo anymore. A kiosk order now starts the configured arm movement script. The bundled dummy script is a placeholder until the final arm script is ready.

## What Was Accomplished

### 1. Frontend integration with ROS-backed state

The kiosk HTML was converted from a local fake queue/timer demo into a backend-driven UI.

Current frontend behavior:

- fetches state from `/api/state`
- submits orders to `/api/orders`
- reacts to backend queue and arm-script phase updates
- no longer acts as the source of truth for order execution

Relevant files:

- `Cove Matcha Kiosk - standalone.html`
- `scripts/update_kiosk_bundle.py`
- `src/cove_kiosk_bridge/web/index.html`

### 2. ROS kiosk bridge package

A new ROS 2 Python package was added:

- `src/cove_kiosk_bridge`

This package provides:

- HTTP API for the kiosk UI
- queue ownership and order lifecycle management
- slot assignment
- operator fault/reset actions
- starting the configured Python arm movement script
- tracking script stdout/status output

Key file:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/kiosk_bridge_node.py`

### 3. Script-driven motion path

A script execution path was added:

- website
- kiosk bridge
- configured Python arm script
- script stdout parsed into frontend state

This lets the kiosk submit an order from an iPad while the backend runs the same Python movement script family used for arm bring-up.

### 4. Fake hardware support

The robot description was updated so the same robot can run in either:

- real hardware mode using `damiao_driver/DamiaoHardwareInterface`
- simulation mode using `mock_components/GenericSystem`

Key files:

- `src/simple_assembly_moveit_config/config/simple_assembly.ros2_control.xacro`
- `src/simple_assembly_moveit_config/config/simple_assembly.urdf.xacro`

### 5. Dedicated simulation launch

A dedicated launch file was added for visible website-to-RViz simulation:

- `src/simple_assembly_moveit_config/launch/sim_kiosk.launch.py`

This launches:

- robot state publisher
- MoveIt `move_group`
- RViz
- `ros2_control_node`
- controller spawners
- kiosk bridge

### 6. Motion backend replacement

The kiosk bridge no longer owns arm motion directly.

Instead, it now starts a configured Python script and tracks its stdout.

Default dummy script:

- `src/cove_kiosk_bridge/scripts/dummy_matcha_motion.py`

The final arm script can replace it through the `motion_script_command` ROS parameter or the `MOTION_SCRIPT_COMMAND` environment variable in `scripts/pi/run_cove_kiosk.sh`.

Key file:

- `src/cove_kiosk_bridge/cove_kiosk_bridge/motion_controller.py`

### 7. One-command runner

A run helper was added:

- `scripts/ubuntu/run_cove_kiosk.sh`

Important modes:

- `sim`: kiosk-only backend with timer simulation
- `rviz-sim`: full visible simulation in RViz
- `full`: full robot stack launch
- `smoke-test`: backend/API validation

## Technical Implementation

## System architecture

```text
Kiosk Website
  -> HTTP POST /api/orders
  -> cove_kiosk_bridge
  -> configured Python arm script
  -> stdout JSON/plain-text status lines
  -> HTTP GET /api/state
  -> kiosk frontend status display
```

## Motion execution model

The bridge executes one configured script per order.

Current phases:

- `received`
- `pouring`
- `moving`
- `arrived`

Current waypoint flow:

- `home`
- `pre_pick`
- `pick`
- `lift`
- `transit`
- `pre_place`
- `place`
- `retreat`
- `home`

The backend watches the real arm script stdout for these words and maps them to
the kiosk state:

- `fetching` -> order/script has started
- `pouring` -> preparation phase
- `moving` -> arm is moving toward pickup
- `done` -> order completed

Plain text output is enough. JSON lines are also supported if the script emits
fields such as `state`, `phase`, `status`, `step`, `message`, or `progress`:

```json
{"phase":"pouring","step":"move pick","message":"Lowering arm to pickup pose","progress":0.36}
```

## Fake vs real hardware

Simulation mode:

- `mock_components/GenericSystem`

Real hardware mode:

- `damiao_driver/DamiaoHardwareInterface`

The xacro now accepts a `use_fake_hardware` argument so the same MoveIt package can serve both workflows.

## Important packages and files

### Kiosk bridge

- `src/cove_kiosk_bridge/package.xml`
- `src/cove_kiosk_bridge/setup.py`
- `src/cove_kiosk_bridge/cove_kiosk_bridge/kiosk_bridge_node.py`
- `src/cove_kiosk_bridge/cove_kiosk_bridge/motion_controller.py`

### MoveIt / simulation

- `src/simple_assembly_moveit_config/config/simple_assembly.ros2_control.xacro`
- `src/simple_assembly_moveit_config/config/simple_assembly.urdf.xacro`
- `src/simple_assembly_moveit_config/config/ros2_controllers.yaml`
- `src/simple_assembly_moveit_config/launch/sim_kiosk.launch.py`

### Frontend sync

- `Cove Matcha Kiosk - standalone.html`
- `scripts/update_kiosk_bundle.py`
- `src/cove_kiosk_bridge/web/index.html`

### Runtime helper

- `scripts/ubuntu/run_cove_kiosk.sh`

## How To Make It Work

## Prerequisites

You need:

- ROS 2 Jazzy installed
- the workspace built
- RViz available

## Build

```bash
scripts/build_workspace.sh --packages-select simple_assembly cove_kiosk_bridge simple_assembly_moveit_config
```

## Start the visible simulation

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build
```

Default URL:

- `http://localhost:8080`

If port `8080` is already in use:

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build --port 8092
```

Then use:

- `http://localhost:8092`

## How To Verify It

### In the browser

Open the kiosk URL and submit an order.

### In RViz

Watch the main 3D viewport.

What you should expect:

- the robot model changes pose over time
- the movement follows the kiosk order lifecycle
- this is visible in the robot model itself

What you should not expect:

- the website does not drive RViz interactive markers
- the robot is not meant to be dragged manually by the site

The site triggers trajectories. RViz visualizes the resulting robot state.

### In logs

You should see messages like:

- `Moving arm to 'home'.`
- `Moving arm to 'pre_pick'.`
- `Moving arm to 'pick'.`
- `Goal reached, success!`

## Recommended Workflow

For quick API-only testing:

```bash
scripts/ubuntu/run_cove_kiosk.sh smoke-test --build
```

For frontend-only kiosk testing:

```bash
scripts/ubuntu/run_cove_kiosk.sh sim --build
```

For visible ROS simulation:

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build
```

## Troubleshooting

### Nothing moves in RViz

Check these first:

1. Make sure you are on the correct kiosk URL.
2. Make sure the kiosk bridge actually started and did not fail to bind its HTTP port.
3. Make sure you submitted an order.
4. Watch the latest RViz window, not an older stale one.

### RViz opens but the website is talking to the wrong backend

This usually happens when port `8080` is already occupied by an older kiosk process.

Use a clean port:

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --port 8092
```

Then browse to:

- `http://localhost:8092`

### The robot model is visible but the motion is not obvious

In RViz, make sure:

- `RobotModel` display is enabled
- `MotionPlanning` display is enabled
- the main 3D camera is centered on the robot
- the fixed frame is `root`

### Controller manager warnings about overruns

These were observed during simulation. They do not block the visible website-to-RViz flow, but they indicate the environment is not running with ideal timing.

### Missing collision geometry warnings

The URDF currently has visual geometry on links without collision geometry. This does not block visible simulation, but it reduces planning-scene fidelity.

## Current Limitations

- The robot uses fixed joint waypoints, not perception-driven planning.
- The frontend polls backend state; it does not use websocket push.
- The URDF collision model is incomplete.
- There can be port conflicts if multiple kiosk stacks are left running.

## Next Recommended Improvements

1. Add websocket state push for the kiosk frontend.
2. Add a stale-process cleanup helper for kiosk ports.
3. Improve RViz launch defaults and camera framing.
4. Add proper collision geometry to the robot description.
5. Add action/result diagnostics to the operator panel.
