#!/usr/bin/env python3
"""
Pick and Place demo for the COVE 6-DOF arm.

Simulates picking an object from one location and placing it at another.
Since we have no gripper yet, gripper open/close are simulated with pauses.

The sequence:
  1. Home          — start position, arm upright
  2. Pre-pick      — approach above the pick location
  3. Pick          — lower to the object
  4. [grip]        — close gripper (simulated)
  5. Lift          — raise with object
  6. Transit       — carry to place location (base rotates)
  7. Pre-place     — approach above the place location
  8. Place         — lower to the surface
  9. [release]     — open gripper (simulated)
  10. Retreat      — back away from placed object
  11. Home         — return to start

MoveIt2 must be running: bash ~/ros2_ws/launch_moveit.sh

Run:
  python3 pick_and_place.py
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
    "revolute_1_0",  # base rotation
    "revolute_2_0",  # shoulder
    "revolute_3_0",  # elbow
    "revolute_4_0",  # forearm
    "revolute_5_0",  # wrist pitch
    "revolute_6_0",  # wrist roll
]

# --- PICK AND PLACE WAYPOINTS ---
# All values in radians. Tune these to match your workspace.
#
#                     j1     j2     j3     j4     j5     j6
#                    base  shldr  elbow  farm   wpitch wroll
WAYPOINTS = {
    "home":       [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0],

    # --- PICK SIDE (base at 0 rad, straight ahead) ---
    "pre_pick":   [ 0.0,  -1.0,   0.3,  -0.5,  -0.3,   0.0],
    "pick":       [ 0.0,  -1.4,   0.6,  -0.8,  -0.3,   0.0],
    "lift":       [ 0.0,  -0.8,   0.3,  -0.4,  -0.3,   0.0],

    # --- TRANSIT (rotate base to place side) ---
    "transit":    [ 1.2,  -0.6,   0.2,  -0.3,  -0.2,   0.0],

    # --- PLACE SIDE (base at ~1.2 rad, rotated left) ---
    "pre_place":  [ 1.2,  -1.0,   0.3,  -0.5,  -0.3,   0.0],
    "place":      [ 1.2,  -1.4,   0.6,  -0.8,  -0.3,   0.0],
    "retreat":    [ 1.2,  -0.6,   0.2,  -0.3,  -0.2,   0.0],
}

# The full pick-and-place sequence with actions
SEQUENCE = [
    ("move",    "home"),
    ("move",    "pre_pick"),
    ("move",    "pick"),
    ("gripper", "close"),       # grab the object
    ("move",    "lift"),
    ("move",    "transit"),
    ("move",    "pre_place"),
    ("move",    "place"),
    ("gripper", "open"),        # release the object
    ("move",    "retreat"),
    ("move",    "home"),
]


def gripper_action(action: str):
    """Simulate gripper open/close. Replace with real gripper control later."""
    if action == "close":
        print("  [gripper] closing...", end=" ", flush=True)
        time.sleep(0.8)
        print("gripped")
    elif action == "open":
        print("  [gripper] opening...", end=" ", flush=True)
        time.sleep(0.8)
        print("released")


def main():
    rclpy.init()
    node = Node("pick_and_place")

    rclpy.logging.set_logger_level(
        "pick_and_place", rclpy.logging.LoggingSeverity.ERROR
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
    time.sleep(.1)
    timeout = time.time() + 10.0
    while moveit2.joint_state is None:
        if time.time() > timeout:
            print(" TIMEOUT — is MoveIt2 running?")
            sys.exit(1)
        time.sleep(0.2)
    print(" ready.\n")

    rclpy.logging.set_logger_level(
        "pick_and_place", rclpy.logging.LoggingSeverity.WARN
    )

    print("=== PICK AND PLACE ===\n")

    move_count = 0
    fail_count = 0

    for action_type, target in SEQUENCE:
        if action_type == "gripper":
            gripper_action(target)
            continue

        joint_positions = WAYPOINTS[target]
        move_count += 1
        print(f"  [{move_count}] {target}...", end=" ", flush=True)

        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()

        if moveit2.motion_suceeded:
            print("ok")
        else:
            print("FAILED")
            fail_count += 1

        time.sleep(0.01)

    print(f"\n=== COMPLETE: {move_count - fail_count}/{move_count} moves ===")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
