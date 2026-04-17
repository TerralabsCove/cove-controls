#!/usr/bin/env python3
"""
Motion sequence demo for COVE 6-DOF arm.

Uses pymoveit2 to connect to the already-running move_group node.
MoveIt2 must be running first: bash ~/ros2_ws/launch_moveit.sh

To tune poses, edit the POSES dict below. Find joint values by dragging
the arm in RViz2's MotionPlanning -> Joints tab.

Run:
  python3 pickup_sequence.py
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
import rclpy.logging
import threading
import time
import sys

JOINT_NAMES = [
    "revolute_1_0",  # base rotation (X joint, +/-180)
    "revolute_2_0",  # shoulder (Z joint, +/-110)
    "revolute_3_0",  # elbow (X joint, +/-180)
    "revolute_4_0",  # forearm (Z joint, +/-110)
    "revolute_5_0",  # wrist pitch (X joint, +/-180)
    "revolute_6_0",  # wrist roll (Z joint, +/-110)
]

# --- EDIT THESE POSES ---
# Each pose is [j1, j2, j3, j4, j5, j6] in radians.
POSES = {
    "home":       [0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
    "ready":      [0.0,   -1.5,    0.05,  -0.20,  -0.43,   0.58],
    "pickup":     [0.0,   -1.0,    0.5,   -0.8,    0.0,    0.0],
    "look_left":  [1.0,   -0.8,    0.3,   -0.5,   -0.2,    0.0],
    "look_right": [-1.0,  -0.8,    0.3,   -0.5,    0.2,    0.0],
    "reach_up":   [0.0,   -0.3,   -0.2,   -0.1,   -0.5,    0.0],
    "fold":       [0.0,   -1.8,    1.2,   -1.5,    0.8,    0.0],
}

SEQUENCE = [
    "ready",
    "look_left",
    "reach_up",
    "look_right",
    "pickup",
    "fold",
    "home",
]


def main():
    rclpy.init()
    node = Node("pickup_sequence")

    # Suppress pymoveit2's noisy "joint states not available" warnings
    rclpy.logging.set_logger_level(
        "pickup_sequence", rclpy.logging.LoggingSeverity.ERROR
    )

    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name="root",
        end_effector_name="part_1_8",
        group_name="arm",
        callback_group=callback_group,
    )
    moveit2.pipeline_id = "pilz_industrial_motion_planner"
    moveit2.planner_id = "PTP"
    moveit2.max_velocity = 0.1
    moveit2.max_acceleration = 0.1
    moveit2.planning_time = 2.0

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    print("Waiting for move_group...", end="", flush=True)
    time.sleep(3.0)
    timeout = time.time() + 10.0
    while moveit2.joint_state is None:
        if time.time() > timeout:
            print(" TIMEOUT — is MoveIt2 running?")
            sys.exit(1)
        time.sleep(0.2)
    print(" ready.\n")

    # Restore normal logging for execution phase
    rclpy.logging.set_logger_level(
        "pickup_sequence", rclpy.logging.LoggingSeverity.WARN
    )

    success_count = 0
    for i, pose_name in enumerate(SEQUENCE, 1):
        joint_positions = POSES[pose_name]
        print(f"  [{i}/{len(SEQUENCE)}] {pose_name}...", end=" ", flush=True)

        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()

        if moveit2.motion_suceeded:
            print("ok")
            success_count += 1
        else:
            print("FAILED")

        time.sleep(0.5)

    print(f"\nDone: {success_count}/{len(SEQUENCE)} moves succeeded.")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
