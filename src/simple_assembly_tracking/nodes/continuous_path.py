#!/usr/bin/env python3
"""Continuously cycle through Cartesian waypoints — no manual approval, loops forever."""

# ── Edit waypoints here (same coords as waypoint_runner.py) ───────────────────
# Each entry: (name, x, y, z) in fixed frame (root), metres
WAYPOINTS = [
    ("location_1", -0.118,  0.205, 0.418),
    ("location_2", -0.140,  0.150, 0.346),
]
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, MoveItErrorCodes,
    MotionPlanRequest, PlanningOptions, PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker


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
        self.goal_tolerance = float(
            self.declare_parameter("goal_tolerance", 0.04).value
        )
        self.min_z = float(self.declare_parameter("min_z", 0.0565).value)
        self.velocity_scale = float(
            self.declare_parameter("velocity_scale", 0.3).value
        )
        self.acceleration_scale = float(
            self.declare_parameter("acceleration_scale", 0.3).value
        )
        self.dwell = float(self.declare_parameter("dwell", 0.5).value)
        self.max_loops = int(self.declare_parameter("max_loops", 0).value)  # 0 = forever

        for name, _, _, z in WAYPOINTS:
            if z < self.min_z:
                raise RuntimeError(
                    f"Waypoint '{name}' z={z:.4f} below min_z={self.min_z:.4f}"
                )

        self.move_group_client = ActionClient(self, MoveGroup, self.move_action)
        self.marker_pub = self.create_publisher(Marker, "/waypoints/markers", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self.latest_joint_state = None
        self.current_index = 0
        self.loops_completed = 0

        self.get_logger().info(
            f"ContinuousPath: {len(WAYPOINTS)} waypoints, "
            f"{'forever' if self.max_loops == 0 else f'{self.max_loops} loops'}, "
            f"dwell={self.dwell:.2f}s, vel={self.velocity_scale:.2f}"
        )

        self._publish_markers()
        self.create_timer(2.0, self._kick_off_once)
        self._kicked = False

    def _kick_off_once(self) -> None:
        if self._kicked:
            return
        if not self.move_group_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("Waiting for MoveGroup…")
            return
        if self.latest_joint_state is None:
            self.get_logger().info("Waiting for /joint_states…")
            return
        self._kicked = True
        self._send_current()

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _send_current(self) -> None:
        name, x, y, z = WAYPOINTS[self.current_index]
        self.get_logger().info(
            f"→ {self.current_index + 1}/{len(WAYPOINTS)} '{name}' "
            f"({x:.3f}, {y:.3f}, {z:.3f})"
        )

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

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.velocity_scale
        req.max_acceleration_scaling_factor = self.acceleration_scale
        constraints = Constraints()
        constraints.name = name
        constraints.position_constraints.append(pc)
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

        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("MoveGroup rejected goal — retrying in 2s")
            self.create_timer(2.0, lambda: self._send_current())
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        code = result.error_code.val
        name = WAYPOINTS[self.current_index][0]

        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"'{name}' failed (code={code}) — retrying after dwell"
            )
        else:
            self.get_logger().info(
                f"✓ '{name}' in {result.planning_time:.2f}s"
            )
            self.current_index = (self.current_index + 1) % len(WAYPOINTS)
            if self.current_index == 0:
                self.loops_completed += 1
                if self.max_loops and self.loops_completed >= self.max_loops:
                    self.get_logger().info(
                        f"Completed {self.loops_completed} loop(s) — stopping"
                    )
                    return

        self._publish_markers()
        timer = self.create_timer(self.dwell, lambda: self._fire_next(timer))

    def _fire_next(self, timer) -> None:
        timer.cancel()
        self._send_current()

    def _publish_markers(self) -> None:
        for i, (name, x, y, z) in enumerate(WAYPOINTS):
            sphere = Marker()
            sphere.header.frame_id = self.fixed_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "waypoints"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
            if i == self.current_index:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (0.1, 1.0, 0.1, 1.0)
            else:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (0.7, 0.7, 0.7, 0.5)
            self.marker_pub.publish(sphere)

            label = Marker()
            label.header.frame_id = self.fixed_frame
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "waypoint_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.06
            label.pose.orientation.w = 1.0
            label.scale.z = 0.04
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = f"{i + 1}:{name}"
            self.marker_pub.publish(label)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContinuousPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
