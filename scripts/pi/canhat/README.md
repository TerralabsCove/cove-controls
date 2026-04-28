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

The CAN HAT MoveIt config defaults to:

- `can_interface=can1`
- `control_mode=position` mapped internally to DM `POS_VEL_MODE`
- `switch_mode_on_activate=true`
- `revolute_7_0=DM4340:0x01:0x11` for the current small bench motor

The original `scripts/pi/simple` and `scripts/pi/full` scripts still use the
serial-compatible MoveIt configs.
