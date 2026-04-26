#!/usr/bin/env python3
"""Step through predefined Cartesian waypoints one at a time, waiting for manual approval."""

# ── Edit waypoints here ────────────────────────────────────────────────────────
# Each entry: (name, x, y, z)  in the fixed frame (root), metres
WAYPOINTS = [
    ("location_1", -0.118,  0.205, 0.418),
    ("location_2", -0.140,  0.150, 0.346),
]
# ──────────────────────────────────────────────────────────────────────────────

from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, MoveItErrorCodes,
    MotionPlanRequest, PlanningOptions, PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, Marker,
)


class WaypointRunner(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_runner")

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
        self.min_z = float(
            self.declare_parameter("min_z", 0.0565).value
        )
        self.velocity_scale = float(
            self.declare_parameter("velocity_scale", 0.3).value
        )
        self.acceleration_scale = float(
            self.declare_parameter("acceleration_scale", 0.3).value
        )
        self.button_x = float(self.declare_parameter("button_x", 0.35).value)
        self.button_y = float(self.declare_parameter("button_y", -0.25).value)
        self.button_z = float(self.declare_parameter("button_z", 0.60).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group_client = ActionClient(self, MoveGroup, self.move_action)
        self.button_server = InteractiveMarkerServer(self, "waypoint_runner_buttons")

        self.waypoint_marker_pub = self.create_publisher(
            Marker, "/waypoints/markers", 10
        )

        self.latest_joint_state = None
        self.current_index = 0
        self.moving = False

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(Empty, "/waypoints/execute", self._on_execute, 10)

        self._make_button()
        self._publish_waypoint_markers()

        self.get_logger().info(
            f"WaypointRunner ready — {len(WAYPOINTS)} waypoints loaded. "
            f"Click EXECUTE in RViz or: ros2 topic pub /waypoints/execute std_msgs/msg/Empty '{{}}'"
        )
        self._log_current()

    # ── Subscriptions ──────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _on_execute(self, _msg: Empty) -> None:
        self._execute_current("topic")

    def _on_button_feedback(self, feedback) -> None:
        if feedback.event_type == feedback.BUTTON_CLICK:
            self._execute_current("rviz_button")

    # ── Execution ──────────────────────────────────────────────────────────────

    def _execute_current(self, source: str) -> None:
        if self.moving:
            self.get_logger().warn("Already moving — wait for current motion to finish")
            return
        if not WAYPOINTS:
            self.get_logger().error("No waypoints defined")
            return

        name, x, y, z = WAYPOINTS[self.current_index]

        if z < self.min_z:
            self.get_logger().error(
                f"Waypoint '{name}' z={z:.4f} is below min_z={self.min_z:.4f} "
                "(height of revolute_1_0) — refusing to execute"
            )
            return

        self.get_logger().info(
            f"{source}: executing waypoint {self.current_index + 1}/{len(WAYPOINTS)} "
            f"'{name}' → ({x:.3f}, {y:.3f}, {z:.3f})"
        )

        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"MoveGroup not available: {self.move_action}")
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

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.velocity_scale
        req.max_acceleration_scaling_factor = self.acceleration_scale
        goal_constraints = Constraints()
        goal_constraints.name = name
        goal_constraints.position_constraints.append(pc)
        req.goal_constraints.append(goal_constraints)
        if self.latest_joint_state is not None:
            req.start_state.joint_state = self.latest_joint_state
            req.start_state.is_diff = True

        options = PlanningOptions()
        options.plan_only = False
        options.planning_scene_diff.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = options

        self.moving = True
        self._update_button(label="MOVING…", color=(0.6, 0.6, 0.0))
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup rejected the goal")
            self.moving = False
            self._make_button()
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self.moving = False
        result = future.result().result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Motion failed: error_code={code} — staying on waypoint {self.current_index + 1}"
            )
            self._make_button()
            return

        name = WAYPOINTS[self.current_index][0]
        self.get_logger().info(
            f"Reached '{name}' in {result.planning_time:.2f}s"
        )

        self.current_index = (self.current_index + 1) % len(WAYPOINTS)
        self._make_button()
        self._publish_waypoint_markers()
        self._log_current()

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _log_current(self) -> None:
        name, x, y, z = WAYPOINTS[self.current_index]
        self.get_logger().info(
            f"Next: waypoint {self.current_index + 1}/{len(WAYPOINTS)} "
            f"'{name}' ({x:.3f}, {y:.3f}, {z:.3f}) — click EXECUTE or publish /waypoints/execute"
        )

    def _make_button(self) -> None:
        name, x, y, z = WAYPOINTS[self.current_index]
        label = f"EXECUTE\n{self.current_index + 1}/{len(WAYPOINTS)} {name}"
        color = (0.05, 0.65, 0.20)
        self._insert_button(label=label, color=color)
        self.button_server.applyChanges()

    def _update_button(self, label: str, color) -> None:
        self._insert_button(label=label, color=color)
        self.button_server.applyChanges()

    def _insert_button(self, label: str, color) -> None:
        button = InteractiveMarker()
        button.header.frame_id = self.fixed_frame
        button.name = "execute_waypoint"
        button.description = label
        button.scale = 0.16
        button.pose.position.x = self.button_x
        button.pose.position.y = self.button_y
        button.pose.position.z = self.button_z
        button.pose.orientation.w = 1.0

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 0.24
        box.scale.y = 0.08
        box.scale.z = 0.04
        box.color.r = color[0]
        box.color.g = color[1]
        box.color.b = color[2]
        box.color.a = 0.95

        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z = 0.07
        text.scale.z = 0.030
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = label

        control = InteractiveMarkerControl()
        control.name = "click"
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.extend([box, text])
        button.controls.append(control)

        self.button_server.insert(button, feedback_callback=self._on_button_feedback)

    def _publish_waypoint_markers(self) -> None:
        for i, (name, x, y, z) in enumerate(WAYPOINTS):
            marker = Marker()
            marker.header.frame_id = self.fixed_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04

            if i == self.current_index:
                # Current target: bright green
                marker.color.r = 0.1
                marker.color.g = 1.0
                marker.color.b = 0.1
                marker.color.a = 1.0
            else:
                # Other waypoints: dim white
                marker.color.r = 0.7
                marker.color.g = 0.7
                marker.color.b = 0.7
                marker.color.a = 0.5

            # Label
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
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            label.text = f"{i + 1}:{name}"

            self.waypoint_marker_pub.publish(marker)
            self.waypoint_marker_pub.publish(label)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointRunner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
