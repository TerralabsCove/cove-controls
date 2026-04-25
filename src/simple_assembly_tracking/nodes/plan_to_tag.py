#!/usr/bin/env python3
"""Trigger a conservative MoveIt plan toward the currently tracked AprilTag."""

import math

from interactive_markers import InteractiveMarkerServer
import rclpy
from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, MoveItErrorCodes
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, PositionConstraint
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class PlanToTag(Node):
    def __init__(self) -> None:
        super().__init__("plan_to_tag")

        self.fixed_frame = str(self.declare_parameter("fixed_frame", "root").value)
        self.tag_frame = str(self.declare_parameter("tag_frame", "tag_0").value)
        self.camera_frame = str(
            self.declare_parameter("camera_frame", "camera_optical_frame").value
        )
        self.target_link = str(
            self.declare_parameter("target_link", "camera_optical_frame").value
        )
        self.group_name = str(self.declare_parameter("group_name", "arm").value)
        self.move_action = str(
            self.declare_parameter("move_action", "/move_action").value
        )
        self.execute = bool(self.declare_parameter("execute", False).value)
        self.approach_distance = float(
            self.declare_parameter("approach_distance", 0.20).value
        )
        self.goal_tolerance = float(
            self.declare_parameter("goal_tolerance", 0.04).value
        )
        self.allowed_planning_time = float(
            self.declare_parameter("allowed_planning_time", 5.0).value
        )
        self.velocity_scale = float(
            self.declare_parameter("velocity_scale", 0.10).value
        )
        self.acceleration_scale = float(
            self.declare_parameter("acceleration_scale", 0.10).value
        )
        self.workspace_radius = float(
            self.declare_parameter("workspace_radius", 2.0).value
        )
        self.button_x = float(self.declare_parameter("button_x", 0.35).value)
        self.button_y = float(self.declare_parameter("button_y", -0.25).value)
        self.button_z = float(self.declare_parameter("button_z", 0.45).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group = ActionClient(self, MoveGroup, self.move_action)
        self.button_server = InteractiveMarkerServer(self, "plan_to_tag_button")

        self.display_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        self.goal_marker_pub = self.create_publisher(
            Marker, "/apriltag/plan_goal_marker", 10
        )
        self.create_subscription(Empty, "/apriltag/plan_to_tag", self.on_trigger, 10)
        self.create_subscription(
            PointStamped,
            "/apriltag/plan_to_tag_point",
            self.on_point_trigger,
            10,
        )
        self._make_button()

        mode = "plan+execute" if self.execute else "plan-only"
        self.get_logger().info(
            "PlanToTag ready "
            f"mode={mode} group={self.group_name} target_link={self.target_link} "
            f"tag={self.tag_frame} trigger=/apriltag/plan_to_tag rviz_button=/plan_to_tag_button"
        )

    def on_trigger(self, _msg: Empty) -> None:
        self.plan_to_tag("topic")

    def on_point_trigger(self, _msg: PointStamped) -> None:
        self.plan_to_tag("rviz")

    def on_button_feedback(self, feedback) -> None:
        if feedback.event_type == feedback.BUTTON_CLICK:
            self.plan_to_tag("rviz_button")

    def _make_button(self) -> None:
        button = InteractiveMarker()
        button.header.frame_id = self.fixed_frame
        button.name = "plan_to_tag"
        button.description = "PLAN TO TAG"
        button.scale = 0.16
        button.pose.position.x = self.button_x
        button.pose.position.y = self.button_y
        button.pose.position.z = self.button_z
        button.pose.orientation.w = 1.0

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 0.20
        box.scale.y = 0.08
        box.scale.z = 0.04
        box.color.r = 0.05
        box.color.g = 0.65
        box.color.b = 0.20
        box.color.a = 0.95

        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.z = 0.07
        label.scale.z = 0.035
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = "PLAN TO TAG"

        control = InteractiveMarkerControl()
        control.name = "click"
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.extend([box, label])
        button.controls.append(control)

        self.button_server.insert(button, feedback_callback=self.on_button_feedback)
        self.button_server.applyChanges()

    def plan_to_tag(self, source: str) -> None:
        tag_tf = self._lookup(self.fixed_frame, self.tag_frame)
        camera_tf = self._lookup(self.fixed_frame, self.camera_frame)
        if tag_tf is None or camera_tf is None:
            return

        tag = tag_tf.transform.translation
        camera = camera_tf.transform.translation
        dx = camera.x - tag.x
        dy = camera.y - tag.y
        dz = camera.z - tag.z
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-6:
            self.get_logger().warn("Camera and tag are at the same TF point; cannot choose approach side")
            return

        scale = self.approach_distance / norm
        target = Pose()
        target.position.x = tag.x + dx * scale
        target.position.y = tag.y + dy * scale
        target.position.z = tag.z + dz * scale
        target.orientation.w = 1.0

        self._publish_goal_marker(target)
        self.get_logger().info(
            f"{source} trigger: planning {self.target_link} to "
            f"({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f}) "
            f"in {self.fixed_frame}; tag range={norm:.3f}m"
        )

        if not self.move_group.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"MoveGroup action server not available: {self.move_action}")
            return

        goal = MoveGroup.Goal()
        goal.request = self._motion_plan_request(target)
        goal.planning_options = self._planning_options()

        future = self.move_group.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _lookup(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Missing TF {target_frame} -> {source_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

    def _motion_plan_request(self, target: Pose) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = self.allowed_planning_time
        request.max_velocity_scaling_factor = self.velocity_scale
        request.max_acceleration_scaling_factor = self.acceleration_scale
        request.workspace_parameters.header.frame_id = self.fixed_frame
        request.workspace_parameters.min_corner.x = -self.workspace_radius
        request.workspace_parameters.min_corner.y = -self.workspace_radius
        request.workspace_parameters.min_corner.z = -self.workspace_radius
        request.workspace_parameters.max_corner.x = self.workspace_radius
        request.workspace_parameters.max_corner.y = self.workspace_radius
        request.workspace_parameters.max_corner.z = self.workspace_radius

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.goal_tolerance]

        position = PositionConstraint()
        position.header.frame_id = self.fixed_frame
        position.link_name = self.target_link
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(target)
        position.weight = 1.0

        constraints = Constraints()
        constraints.name = f"{self.tag_frame}_approach"
        constraints.position_constraints.append(position)
        request.goal_constraints.append(constraints)
        return request

    def _planning_options(self) -> PlanningOptions:
        options = PlanningOptions()
        options.plan_only = not self.execute
        options.look_around = False
        options.replan = False
        options.planning_scene_diff.is_diff = True
        return options

    def _publish_goal_marker(self, target: Pose) -> None:
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "plan_to_tag"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = target
        marker.scale.x = self.goal_tolerance * 2.0
        marker.scale.y = self.goal_tolerance * 2.0
        marker.scale.z = self.goal_tolerance * 2.0
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.85
        self.goal_marker_pub.publish(marker)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup rejected the plan-to-tag request")
            return

        self.get_logger().info("MoveGroup accepted the plan-to-tag request")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveGroup failed plan-to-tag request: error_code={code}"
            )
            return

        display = DisplayTrajectory()
        display.model_id = "simple_assembly_tracking"
        display.trajectory_start = result.trajectory_start
        display.trajectory.append(result.planned_trajectory)
        self.display_pub.publish(display)
        action = "executed" if self.execute else "planned"
        self.get_logger().info(
            f"MoveGroup {action} path in {result.planning_time:.2f}s; published /display_planned_path"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanToTag()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
