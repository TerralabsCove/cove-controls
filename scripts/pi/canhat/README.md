# CAN HAT One Motor Bring-Up

Start the one-motor SocketCAN driver on the Pi:

```bash
./scripts/pi/canhat/one_motor_driver.sh
```

The standalone driver publishes status only until a command topic message is
received. Starting it or calling `enable_motor.sh` should not send a hidden
zero-velocity or zero-position command.

Then, from another Pi terminal, enable/disable or do a short velocity spin:

```bash
./scripts/pi/canhat/enable_motor.sh
./scripts/pi/canhat/spin_motor.sh
./scripts/pi/canhat/disable_motor.sh
```

Defaults:

- `CAN_INTERFACE=can1`
- `MOTOR_SPEC=revolute_7_0:DM4340:0x01:0x11`
- `VEL=0.4`
- `DURATION=2.0`

Override values inline, for example:

```bash
VEL=0.2 DURATION=1.0 ./scripts/pi/canhat/spin_motor.sh
```

## CAN HAT MoveIt

These scripts use the SocketCAN ros2_control hardware interface instead of the
original serial `damiao_driver` config:

```bash
./scripts/pi/canhat/simple_assembly_moveit.sh
./scripts/pi/canhat/tracking_moveit.sh
./scripts/pi/canhat/plan_to_tag.sh
./scripts/pi/canhat/plan_to_tag_execute.sh
```

Before using the CAN HAT MoveIt scripts, put the arm in the straight-line zero
pose and write that pose into the DM motors:

```bash
./scripts/pi/canhat/zero_straight_position.sh
```

The CAN HAT ros2_control configs use the motor-stored zero directly
(`capture_zero_on_activate=false`) instead of capturing a software offset on
each launch. This is intentionally only enabled in the CAN HAT configs.

The CAN HAT MoveIt config defaults to:

- `can_interface=can1`
- `control_mode=position` mapped internally to DM `POS_VEL_MODE`
- `switch_mode_on_activate=true`
- `capture_zero_on_activate=false`
- `suppress_initial_writes=true`
- full-arm IDs `0x01..0x07` with feedback IDs `0x11..0x17`

On startup, the CAN HAT hardware interface seeds command positions from live
motor feedback and suppresses DM control writes until a non-hold command
arrives. Launching `plan_to_tag.sh`, `tracking_moveit.sh`, or
`simple_assembly_moveit.sh` should initialize/enable/read feedback, but should
not command the arm to zero by itself.

The original `scripts/pi/simple` and `scripts/pi/full` scripts still use the
serial-compatible MoveIt configs.

## Waypoint Recording

Run this while the CAN HAT robot launch is publishing `/joint_states` and TF:

```bash
./scripts/pi/canhat/record_waypoints.sh
```

If you want to manually move the arm and record joint positions without
enabling the motors, use the read-only wrapper instead:

```bash
./scripts/pi/canhat/record_waypoints_readonly.sh
```

It starts the CAN HAT driver in `control_mode=status` with `auto_enable=false`,
`switch_mode_on_start=false`, and `disable_on_shutdown=false`, then starts the
CAN HAT robot_state_publisher so TF is available. It records the latest motor
joint positions plus `root -> camera_optical_frame` pose to a timestamped file
like `recorded_waypoints/canhat_joint_waypoints_YYYYmmdd_HHMMSS.jsonl`, and
updates `recorded_waypoints/canhat_joint_waypoints_latest.jsonl`.

Each time you press Enter, the recorder appends a waypoint. Type text before
pressing Enter to save that text as the waypoint comment. Type `q`, `quit`, or
`exit` to stop. Comments are only commands during replay if they begin with
`!`, for example `!magnet on` or `!magnet off`.

By default it records transforms from `root` to both `wrist_link` and
`camera_optical_frame`. Override the file or frames inline:

```bash
OUTPUT=/tmp/arm_path.jsonl ./scripts/pi/canhat/record_waypoints.sh --target-frame wrist_link
```

## Recorded Path Replay

Start the CAN HAT ros2_control robot in one terminal:

```bash
./scripts/pi/canhat/tracking_moveit.sh
```

Then replay the latest read-only joint path in another terminal:

```bash
./scripts/pi/canhat/run_recorded_path.sh
```

By default this reads
`recorded_waypoints/canhat_joint_waypoints_latest.jsonl` and executes all valid
joint-position waypoints in file order. The runner pauses before each waypoint
so you can review before pressing Enter. `!magnet on` and `!magnet off`
comments execute after that waypoint's move; other comments are ignored by the
runner.

Override the file, sequence, or speed inline:

```bash
WAYPOINT_FILE=recorded_waypoints/canhat_waypoints.jsonl WAYPOINT_SEQUENCE=7,6,5,6,7 MOVE_DURATION=6.0 ./scripts/pi/canhat/run_recorded_path.sh
```

## End-Effector Path Replay

For pose-only JSONL files captured with `record_waypoints_readonly.sh`, use:

```bash
./scripts/pi/canhat/run_eef_path.sh recorded_waypoints/canhat_eef_waypoints.jsonl
```

This sends each recorded Cartesian pose through MoveIt `/move_action` using the
recorded end-effector frame, usually `camera_optical_frame`. It pauses before
each JSONL line so you can approve the move. Comments are treated as comments
unless they start with `!`; supported commands are `!magnet on` and
`!magnet off`, which execute after that line's pose move.

## Raw CAN Diagnostics

These scripts do not use ROS. They are for checking which DM motor IDs are
actually responding on the CAN HAT:

```bash
./scripts/pi/canhat/raw_scan_ids.sh
./scripts/pi/canhat/raw_enable_disable_all_ids.sh
```

Defaults are `CAN_INTERFACE=can1` and IDs `0x01..0x07`. Override them inline:

```bash
START_ID=1 END_ID=7 ./scripts/pi/canhat/raw_scan_ids.sh
ACTION=cycle LEAVE_ENABLED=0 ./scripts/pi/canhat/raw_enable_disable_all_ids.sh
```

`raw_scan_ids.sh` only sends DM status requests. `raw_enable_disable_all_ids.sh`
can run `ACTION=cycle`, `ACTION=enable`, or `ACTION=disable`; the default cycle
disables, enables, requests status, then disables again.

If a scan reports zero raw capture lines and the CAN state is `ERROR-PASSIVE`,
the Pi sent frames but nothing on that bus ACKed them.
