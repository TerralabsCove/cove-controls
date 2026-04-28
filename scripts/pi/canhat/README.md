# CAN HAT One Motor Bring-Up

Start the one-motor SocketCAN driver on the Pi:

```bash
./scripts/pi/canhat/one_motor_driver.sh
```

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
- full-arm IDs `0x01..0x07` with feedback IDs `0x11..0x17`

The original `scripts/pi/simple` and `scripts/pi/full` scripts still use the
serial-compatible MoveIt configs.

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
