# damiao_socketcan_driver

ROS 2 Damiao/DM motor driver using native Linux SocketCAN for the CAN HAT.

This package ports the protocol used by `damiao_driver` away from the serial
USB-CAN adapter framing and onto raw CAN frames through SocketCAN.

Known-good test default:

```bash
ros2 launch damiao_socketcan_driver damiao_socketcan_launch.py
```

The launch file defaults to:

- `can_interface:=can1`
- `motors:=revolute_7_0:DM10010:0x01:0x11`
- `control_mode:=status`
- `auto_enable:=false`

The standalone `motors` spec also accepts an optional fifth field for logical
joint direction:

```bash
motors:=revolute_1_0:DM4340:0x01:0x11:-1
```

Enable or disable the configured motors through services:

```bash
ros2 service call /damiao_socketcan_driver/enable_motors std_srvs/srv/Trigger
ros2 service call /damiao_socketcan_driver/disable_motors std_srvs/srv/Trigger
```

Command topics match the serial driver node:

- `cmd_vel_motors` for `control_mode:=velocity`
- `cmd_pos_vel` for `control_mode:=pos_vel`
- `cmd_mit` for `control_mode:=mit`

For ros2_control, use plugin:

```xml
<plugin>damiao_socketcan_driver/DamiaoSocketCanHardwareInterface</plugin>
<param name="can_interface">can1</param>
<param name="control_mode">pos_vel</param>
```

Joint parameters are the same as the existing driver:

```xml
<param name="motor_type">DM10010</param>
<param name="slave_id">0x01</param>
<param name="master_id">0x11</param>
<param name="direction">-1</param>
```

`direction` is optional and defaults to `1`. A value of `-1` makes ROS joint
position, velocity, effort, and commands use the opposite sign from the raw DM
motor encoder.
