# CAN HAT One Motor RViz

Run on the Ubuntu/RViz machine:

```bash
./scripts/ubuntu/canhat/one_motor_rviz.sh
```

The RViz model uses the joint name `revolute_7_0`, matching the known working
DM motor `0x01` on the CAN HAT driver. When the Pi publishes `/joint_states`,
the yellow marker on the one-motor bot should rotate in RViz.

To inspect incoming joint states:

```bash
./scripts/ubuntu/canhat/watch_joint.sh
```

## CAN HAT MoveIt RViz

Use these on the Ubuntu/RViz machine with the CAN HAT ros2_control variants:

```bash
./scripts/ubuntu/canhat/simple_assembly_moveit_rviz.sh
./scripts/ubuntu/canhat/tracking_moveit_rviz.sh
./scripts/ubuntu/canhat/plan_to_tag_rviz.sh
```

The old `scripts/ubuntu/simple` and `scripts/ubuntu/full` RViz scripts are left
unchanged for the serial-compatible MoveIt configs.

Stored States uses the MoveIt warehouse SQLite backend. If RViz still reports
that it is not connected to a database, install the backend on the Ubuntu/RViz
machine:

```bash
sudo apt install ros-jazzy-warehouse-ros-sqlite
```

The default database path is `~/.ros/simple_assembly_tracking_warehouse.sqlite`.
Override it for CAN HAT scripts with `MOVEIT_WAREHOUSE_DATABASE_PATH=/path/file.sqlite`.

## Waypoint Recording

If `/joint_states` and TF are visible on the Ubuntu/RViz machine, you can record
the same JSONL waypoint file from Ubuntu:

```bash
./scripts/ubuntu/canhat/record_waypoints.sh
```

Press Enter to append a waypoint, or type a comment and press Enter. The default
output is `recorded_waypoints/canhat_waypoints.jsonl`; override with `OUTPUT=...`
if needed.
