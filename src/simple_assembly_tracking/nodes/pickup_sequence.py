#!/usr/bin/env python3
"""Scan for an AprilTag, approach it, magnet-grab, lift, return home.

State machine:
  GO_SCAN     → move to a low scan pose
  SCAN        → rotate base joint in steps, look for tag TF after each step
  APPROACH    → plan to a Cartesian point above the detected tag
  DESCEND     → lower onto the tag
  MAGNET_ON   → energise GPIO17, dwell briefly so the magnet seats
  LIFT        → raise by lift_height
  GO_HOME     → joint-space goal of all zeros
  DONE        → idle, magnet stays on (release manually with /pickup/release)
"""

import math
import subprocess
from enum import Enum

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
    MotionPlanRequest, PlanningOptions, PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener


JOINT_NAMES = [
    "revolute_1_0", "revolute_2_0", "revolute_3_0",
    "revolute_4_0", "revolute_5_0", "revolute_6_0", "revolute_7_0",
]


class State(Enum):
    GO_SCAN = "go_scan"
    SCAN = "scan"
    APPROACH = "approach"
    DESCEND = "descend"
    MAGNET_ON = "magnet_on"
    LIFT = "lift"
    GO_HOME = "go_home"
    DONE = "done"
    FAILED = "failed"


class PickupSequence(Node):
    def __init__(self) -> None:
        super().__init__("pickup_sequence")

        # --- frames / planning ---
        self.fixed_frame = str(self.declare_parameter("fixed_frame", "root").value)
        self.camera_frame = str(
            self.declare_parameter("camera_frame", "camera_optical_frame").value
        )
        self.tag_frame = str(self.declare_parameter("tag_frame", "tag_0").value)
        self.group_name = str(self.declare_parameter("group_name", "arm").value)
        self.move_action = str(
            self.declare_parameter("move_action", "/move_action").value
        )
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.04).value)
        self.joint_tolerance = float(
            self.declare_parameter("joint_tolerance", 0.05).value
        )
        self.velocity_scale = float(self.declare_parameter("velocity_scale", 0.25).value)
        self.acceleration_scale = float(
            self.declare_parameter("acceleration_scale", 0.25).value
        )

        # --- scan pose (camera position in fixed frame) ---
        self.scan_x = float(self.declare_parameter("scan_x", 0.25).value)
        self.scan_y = float(self.declare_parameter("scan_y", 0.0).value)
        self.scan_z = float(self.declare_parameter("scan_z", 0.18).value)

        # --- scan sweep ---
        self.scan_step_deg = float(self.declare_parameter("scan_step_deg", 30.0).value)
        self.scan_max_deg = float(self.declare_parameter("scan_max_deg", 180.0).value)
        self.scan_dwell = float(self.declare_parameter("scan_dwell", 0.8).value)

        # --- approach / descend / lift offsets ---
        self.approach_above = float(
            self.declare_parameter("approach_above", 0.08).value
        )
        self.descend_offset = float(
            self.declare_parameter("descend_offset", 0.02).value
        )
        self.lift_height = float(self.declare_parameter("lift_height", 0.10).value)

        # --- magnet ---
        self.gpio_chip = str(self.declare_parameter("gpio_chip", "gpiochip4").value)
        self.gpio_line = int(self.declare_parameter("gpio_line", 17).value)
        self.magnet_dwell = float(self.declare_parameter("magnet_dwell", 1.0).value)

        # --- safety ---
        self.min_z = float(self.declare_parameter("min_z", 0.0565).value)

        # --- runtime ---
        self.move_group_client = ActionClient(self, MoveGroup, self.move_action)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(Empty, "/pickup/start", self._on_start, 10)
        self.create_subscription(Empty, "/pickup/release", self._on_release, 10)

        self.latest_joint_state = None
        self.state = State.GO_SCAN
        self.scan_index = 0
        self.scan_target_deg = 0.0
        self.tag_pose = None  # (x, y, z) in fixed frame
        self.magnet_proc: subprocess.Popen | None = None
        self.busy = False
        self._auto_started = False

        self.get_logger().info(
            "PickupSequence ready. Publish to /pickup/start to begin "
            "(or it auto-starts when MoveGroup + joint_states are up). "
            "Publish to /pickup/release to drop the magnet."
        )

        self.create_timer(0.5, self._tick)

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _on_start(self, _msg: Empty) -> None:
        self.get_logger().info("Manual start trigger.")
        self.state = State.GO_SCAN
        self._auto_started = True
        self.busy = False

    def _on_release(self, _msg: Empty) -> None:
        self._magnet_off()

    # ── Tick ──────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self.busy:
            return
        if not self._auto_started:
            if (
                self.move_group_client.wait_for_server(timeout_sec=0.1)
                and self.latest_joint_state is not None
            ):
                self.get_logger().info("MoveGroup + joint_states up — starting.")
                self._auto_started = True
            else:
                return

        if self.state == State.GO_SCAN:
            self.get_logger().info(f"→ scan pose ({self.scan_x:.2f}, {self.scan_y:.2f}, {self.scan_z:.2f})")
            self._send_position_goal(self.scan_x, self.scan_y, self.scan_z, after=State.SCAN)

        elif self.state == State.SCAN:
            if self._tag_visible():
                self._capture_tag()
                if self.tag_pose is not None:
                    self.state = State.APPROACH
                    return
            angle = self._next_scan_angle()
            if angle is None:
                self.get_logger().error("Scan exhausted, no tag found.")
                self.state = State.GO_HOME
                return
            self.get_logger().info(f"→ scan rotate revolute_1_0 = {math.degrees(angle):.1f}°")
            self._send_joint_goal({"revolute_1_0": angle}, after=State.SCAN, dwell=self.scan_dwell)

        elif self.state == State.APPROACH:
            x, y, z = self.tag_pose
            tz = max(z + self.approach_above, self.min_z)
            self.get_logger().info(f"→ approach above tag ({x:.3f}, {y:.3f}, {tz:.3f})")
            self._send_position_goal(x, y, tz, after=State.DESCEND)

        elif self.state == State.DESCEND:
            x, y, z = self.tag_pose
            tz = max(z + self.descend_offset, self.min_z)
            self.get_logger().info(f"→ descend onto tag ({x:.3f}, {y:.3f}, {tz:.3f})")
            self._send_position_goal(x, y, tz, after=State.MAGNET_ON)

        elif self.state == State.MAGNET_ON:
            self.get_logger().info("→ magnet ON")
            self._magnet_on()
            self.busy = True
            timer = self.create_timer(self.magnet_dwell, lambda: self._after_magnet(timer))

        elif self.state == State.LIFT:
            x, y, z = self.tag_pose
            tz = max(z + self.lift_height, self.min_z)
            self.get_logger().info(f"→ lift to ({x:.3f}, {y:.3f}, {tz:.3f})")
            self._send_position_goal(x, y, tz, after=State.GO_HOME)

        elif self.state == State.GO_HOME:
            self.get_logger().info("→ home (all joints 0)")
            self._send_joint_goal({n: 0.0 for n in JOINT_NAMES}, after=State.DONE)

        elif self.state == State.DONE:
            self.get_logger().info("✓ Sequence complete. Magnet still energised — /pickup/release to drop.")
            self.state = State.FAILED  # park in a no-op state

        elif self.state == State.FAILED:
            pass

    def _after_magnet(self, timer) -> None:
        timer.cancel()
        self.busy = False
        self.state = State.LIFT

    # ── Scan helpers ──────────────────────────────────────────────────────────

    def _next_scan_angle(self) -> float | None:
        """Return the next revolute_1_0 angle to try, or None when exhausted."""
        # Sweep pattern: 0, +step, -step, +2*step, -2*step, ...
        if self.scan_index == 0:
            self.scan_index = 1
            return 0.0
        max_steps = int(self.scan_max_deg // self.scan_step_deg)
        half = (self.scan_index + 1) // 2
        if half > max_steps:
            return None
        sign = 1 if self.scan_index % 2 == 1 else -1
        self.scan_index += 1
        return sign * math.radians(half * self.scan_step_deg)

    def _tag_visible(self) -> bool:
        return self.tf_buffer.can_transform(
            self.fixed_frame, self.tag_frame, rclpy.time.Time()
        )

    def _capture_tag(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.tag_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().warn(f"Tag TF lookup failed: {exc}")
            return
        t = tf.transform.translation
        if t.z < self.min_z:
            self.get_logger().warn(
                f"Tag at z={t.z:.3f} below min_z={self.min_z:.3f} — refusing to grab"
            )
            return
        self.tag_pose = (t.x, t.y, t.z)
        self.get_logger().info(
            f"Captured tag at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})"
        )

    # ── Magnet ────────────────────────────────────────────────────────────────

    def _magnet_on(self) -> None:
        if self.magnet_proc is not None:
            return
        try:
            self.magnet_proc = subprocess.Popen(
                ["gpioset", "--mode=signal", self.gpio_chip, f"{self.gpio_line}=1"]
            )
        except FileNotFoundError:
            self.get_logger().error("gpioset not found — install gpiod (`sudo apt install gpiod`)")
            self.state = State.FAILED

    def _magnet_off(self) -> None:
        if self.magnet_proc is None:
            return
        self.magnet_proc.terminate()
        try:
            self.magnet_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.magnet_proc.kill()
        self.magnet_proc = None
        self.get_logger().info("Magnet released.")

    # ── MoveGroup goal builders ───────────────────────────────────────────────

    def _send_position_goal(self, x: float, y: float, z: float, after: State, dwell: float = 0.0) -> None:
        if z < self.min_z:
            self.get_logger().error(f"Refusing goal z={z:.3f} below min_z={self.min_z:.3f}")
            self.state = State.FAILED
            return

        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.w = 1.0

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
        constraints.position_constraints.append(pc)
        self._send_request(constraints, after, dwell)

    def _send_joint_goal(self, joints: dict, after: State, dwell: float = 0.0) -> None:
        constraints = Constraints()
        for name, value in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = self.joint_tolerance
            jc.tolerance_below = self.joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        self._send_request(constraints, after, dwell)

    def _send_request(self, constraints: Constraints, after: State, dwell: float) -> None:
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
        self._dwell_after_motion = dwell

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
        if self._dwell_after_motion > 0:
            timer = self.create_timer(
                self._dwell_after_motion, lambda: self._after_dwell(timer)
            )
        else:
            self.busy = False
            self.state = self._next_state_after_motion

    def _after_dwell(self, timer) -> None:
        timer.cancel()
        self.busy = False
        self.state = self._next_state_after_motion


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickupSequence()
    try:
        rclpy.spin(node)
    finally:
        node._magnet_off()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
