#!/usr/bin/env python3
"""Move to a saved waypoint, toggle the magnet, and return home.

Sequence:
  waypoint -> magnet on -> home -> wait -> waypoint -> wait -> magnet off -> home

The waypoint file is intended to be captured from show_eef_pose.sh / tf2_echo.
The parser uses the last Translation and Rotation entries it finds.
"""

import math
import re
import subprocess
from enum import Enum
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker


JOINT_NAMES = [
    "revolute_1_0",
    "revolute_2_0",
    "revolute_3_0",
    "revolute_4_0",
    "revolute_5_0",
    "revolute_6_0",
    "revolute_7_0",
]


class State(Enum):
    GO_WAYPOINT_PICK = "go_waypoint_pick"
    MAGNET_ON = "magnet_on"
    GO_HOME_LOADED = "go_home_loaded"
    WAIT_HOME = "wait_home"
    GO_WAYPOINT_DROP = "go_waypoint_drop"
    WAIT_WAYPOINT = "wait_waypoint"
    MAGNET_OFF = "magnet_off"
    GO_HOME_DONE = "go_home_done"
    DONE = "done"
    FAILED = "failed"


class ContinuousPath(Node):
    def __init__(self) -> None:
        super().__init__("continuous_path")

        self.fixed_frame = str(self.declare_parameter("fixed_frame", "root").value)
        self.camera_frame = str(
            self.declare_parameter("camera_frame", "camera_optical_frame").value
        )
        self.group_name = str(self.declare_parameter("group_name", "arm").value)
        self.move_action = str(
            self.declare_parameter("move_action", "/move_action").value
        )
        self.waypoint_file = str(self.declare_parameter("waypoint_file", "").value)
        self.goal_tolerance = float(
            self.declare_parameter("goal_tolerance", 0.025).value
        )
        self.orientation_tolerance = float(
            self.declare_parameter("orientation_tolerance", 0.12).value
        )
        self.constrain_orientation = bool(
            self.declare_parameter("constrain_orientation", True).value
        )
        self.joint_tolerance = float(
            self.declare_parameter("joint_tolerance", 0.05).value
        )
        self.velocity_scale = float(
            self.declare_parameter("velocity_scale", 0.25).value
        )
        self.acceleration_scale = float(
            self.declare_parameter("acceleration_scale", 0.25).value
        )
        self.wait_after_home = float(
            self.declare_parameter("wait_after_home", 1.0).value
        )
        self.wait_before_release = float(
            self.declare_parameter("wait_before_release", 1.0).value
        )
        self.min_z = float(self.declare_parameter("min_z", 0.0565).value)
        self.gpio_chip = str(self.declare_parameter("gpio_chip", "gpiochip4").value)
        self.gpio_line = int(self.declare_parameter("gpio_line", 17).value)

        if not self.waypoint_file:
            raise RuntimeError("waypoint_file parameter is required")

        self.waypoint = self._load_waypoint(Path(self.waypoint_file))
        if self.waypoint.position.z < self.min_z:
            raise RuntimeError(
                f"Waypoint z={self.waypoint.position.z:.4f} below min_z={self.min_z:.4f}"
            )

        self.move_group_client = ActionClient(self, MoveGroup, self.move_action)
        self.marker_pub = self.create_publisher(Marker, "/waypoints/markers", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self.latest_joint_state = None
        self.state = State.GO_WAYPOINT_PICK
        self.busy = False
        self.started = False
        self.magnet_proc: subprocess.Popen | None = None
        self._next_state_after_motion = State.FAILED

        q = self.waypoint.orientation
        self.get_logger().info(
            "ContinuousPath loaded waypoint "
            f"pos=({self.waypoint.position.x:.3f}, {self.waypoint.position.y:.3f}, "
            f"{self.waypoint.position.z:.3f}) quat=({q.x:.3f}, {q.y:.3f}, "
            f"{q.z:.3f}, {q.w:.3f}) from {self.waypoint_file}"
        )

        self._publish_waypoint_marker()
        self.create_timer(0.5, self._tick)

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _tick(self) -> None:
        if self.busy:
            return
        if not self.started:
            if not self.move_group_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info("Waiting for MoveGroup...")
                return
            if self.latest_joint_state is None:
                self.get_logger().info("Waiting for /joint_states...")
                return
            self.started = True
            self._magnet_off()
            self.get_logger().info("Starting waypoint/magnet/home sequence.")

        if self.state == State.GO_WAYPOINT_PICK:
            self.get_logger().info("-> waypoint, then magnet ON")
            self._send_pose_goal(self.waypoint, after=State.MAGNET_ON)

        elif self.state == State.MAGNET_ON:
            self.get_logger().info("-> magnet ON")
            self._magnet_on()
            self.state = State.GO_HOME_LOADED

        elif self.state == State.GO_HOME_LOADED:
            self.get_logger().info("-> home with magnet ON")
            self._send_home_goal(after=State.WAIT_HOME)

        elif self.state == State.WAIT_HOME:
            self.get_logger().info(f"-> wait {self.wait_after_home:.1f}s at home")
            self._wait(self.wait_after_home, State.GO_WAYPOINT_DROP)

        elif self.state == State.GO_WAYPOINT_DROP:
            self.get_logger().info("-> waypoint again")
            self._send_pose_goal(self.waypoint, after=State.WAIT_WAYPOINT)

        elif self.state == State.WAIT_WAYPOINT:
            self.get_logger().info(
                f"-> wait {self.wait_before_release:.1f}s at waypoint before release"
            )
            self._wait(self.wait_before_release, State.MAGNET_OFF)

        elif self.state == State.MAGNET_OFF:
            self.get_logger().info("-> magnet OFF")
            self._magnet_off()
            self.state = State.GO_HOME_DONE

        elif self.state == State.GO_HOME_DONE:
            self.get_logger().info("-> final home")
            self._send_home_goal(after=State.DONE)

        elif self.state == State.DONE:
            self.get_logger().info("Sequence complete.")
            self.state = State.FAILED

        elif self.state == State.FAILED:
            pass

    def _send_pose_goal(self, target: Pose, after: State) -> None:
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.goal_tolerance]

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(target)

        pc = PositionConstraint()
        pc.header.frame_id = self.fixed_frame
        pc.header.stamp = self.get_clock().now().to_msg()
        pc.link_name = self.camera_frame
        pc.constraint_region = region
        pc.weight = 1.0

        constraints = Constraints()
        constraints.name = "saved_waypoint"
        constraints.position_constraints.append(pc)

        if self.constrain_orientation:
            oc = OrientationConstraint()
            oc.header.frame_id = self.fixed_frame
            oc.header.stamp = self.get_clock().now().to_msg()
            oc.link_name = self.camera_frame
            oc.orientation = target.orientation
            oc.absolute_x_axis_tolerance = self.orientation_tolerance
            oc.absolute_y_axis_tolerance = self.orientation_tolerance
            oc.absolute_z_axis_tolerance = self.orientation_tolerance
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

        self._send_request(constraints, after)

    def _send_home_goal(self, after: State) -> None:
        constraints = Constraints()
        constraints.name = "home"
        for name in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = 0.0
            jc.tolerance_above = self.joint_tolerance
            jc.tolerance_below = self.joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        self._send_request(constraints, after)

    def _send_request(self, constraints: Constraints, after: State) -> None:
        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveGroup not available")
            self.state = State.FAILED
            return

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.velocity_scale
        req.max_acceleration_scaling_factor = self.acceleration_scale
        req.goal_constraints.append(constraints)
        if self.latest_joint_state is not None:
            req.start_state.joint_state = self.latest_joint_state
            req.start_state.is_diff = True

        options = PlanningOptions()
        options.plan_only = False
        options.planning_scene_diff.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = options

        self.busy = True
        self._next_state_after_motion = after
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("MoveGroup rejected the goal")
            self.busy = False
            self.state = State.FAILED
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Motion failed (code={code})")
            self.busy = False
            self.state = State.FAILED
            return
        self.busy = False
        self.state = self._next_state_after_motion

    def _wait(self, seconds: float, after: State) -> None:
        self.busy = True
        timer = self.create_timer(seconds, lambda: self._after_wait(timer, after))

    def _after_wait(self, timer, after: State) -> None:
        timer.cancel()
        self.busy = False
        self.state = after

    def _magnet_on(self) -> None:
        if self.magnet_proc is not None:
            return
        self._drive_gpio_low()
        try:
            self.magnet_proc = subprocess.Popen(
                ["gpioset", "--mode=signal", self.gpio_chip, f"{self.gpio_line}=1"]
            )
        except FileNotFoundError:
            self.get_logger().error("gpioset not found - install gpiod")
            self.state = State.FAILED

    def _magnet_off(self) -> None:
        if self.magnet_proc is not None:
            self.magnet_proc.terminate()
            try:
                self.magnet_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.magnet_proc.kill()
            self.magnet_proc = None
        self._drive_gpio_low()
        self.get_logger().info("Magnet OFF.")

    def _drive_gpio_low(self) -> None:
        try:
            subprocess.run(
                ["gpioset", self.gpio_chip, f"{self.gpio_line}=0"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.get_logger().error("gpioset not found - install gpiod")
            self.state = State.FAILED

    def _publish_waypoint_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoint"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.waypoint
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _load_waypoint(self, path: Path) -> Pose:
        if not path.exists():
            raise RuntimeError(f"Waypoint file not found: {path}")

        position = None
        quat = None
        rpy_rad = None
        rpy_deg = None

        for line in path.read_text(encoding="utf-8").splitlines():
            values = self._bracket_values(line)
            if values is None:
                continue
            if "Translation" in line and len(values) >= 3:
                position = values[:3]
            elif "Quaternion" in line and len(values) >= 4:
                quat = values[:4]
            elif "RPY (radian)" in line and len(values) >= 3:
                rpy_rad = values[:3]
            elif "RPY (degree)" in line and len(values) >= 3:
                rpy_deg = values[:3]

        if position is None:
            raise RuntimeError(
                f"{path} is missing a Translation line. Capture with show_eef_pose.sh "
                "and save at least one full pose sample."
            )

        if quat is None:
            if rpy_rad is not None:
                quat = self._quat_from_rpy(*rpy_rad)
            elif rpy_deg is not None:
                quat = self._quat_from_rpy(*(math.radians(v) for v in rpy_deg))
            else:
                self.get_logger().warn(
                    f"{path} has no Rotation line; using identity orientation"
                )
                quat = [0.0, 0.0, 0.0, 1.0]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation = self._normalize_quat(quat)
        return pose

    def _bracket_values(self, line: str) -> list[float] | None:
        match = re.search(r"\[([^\]]+)\]", line)
        if match is None:
            return None
        return [
            float(v)
            for v in re.findall(
                r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", match.group(1)
            )
        ]

    def _quat_from_rpy(self, roll: float, pitch: float, yaw: float) -> list[float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]

    def _normalize_quat(self, values: list[float]) -> Quaternion:
        x, y, z, w = values[:4]
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            raise RuntimeError("Waypoint orientation quaternion has zero length")
        q = Quaternion()
        q.x = x / norm
        q.y = y / norm
        q.z = z / norm
        q.w = w / norm
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContinuousPath()
    try:
        rclpy.spin(node)
    finally:
        node._magnet_off()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
