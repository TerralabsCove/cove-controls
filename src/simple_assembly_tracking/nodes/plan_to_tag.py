#!/usr/bin/env python3
"""Move the arm's camera to a captured AprilTag using MoveIt."""

import math

from geometry_msgs.msg import PointStamped, Pose
from interactive_markers import InteractiveMarkerServer
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, MoveItErrorCodes,
    MotionPlanRequest, PlanningOptions, PositionConstraint, RobotState,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        self.group_name = str(self.declare_parameter("group_name", "arm").value)
        self.move_action = str(
            self.declare_parameter("move_action", "/move_action").value
        )
        self.execute = bool(self.declare_parameter("execute", False).value)
        self.approach_distance = float(
            self.declare_parameter("approach_distance", 0.0).value
        )
        self.tag_size = float(self.declare_parameter("tag_size", 0.162).value)
        self.goal_tolerance = float(
            self.declare_parameter("goal_tolerance", 0.04).value
        )
        self.velocity_scale = float(
            self.declare_parameter("velocity_scale", 0.3).value
        )
        self.acceleration_scale = float(
            self.declare_parameter("acceleration_scale", 0.3).value
        )
        self.button_x = float(self.declare_parameter("button_x", 0.35).value)
        self.button_y = float(self.declare_parameter("button_y", -0.25).value)
        self.button_z = float(self.declare_parameter("button_z", 0.45).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group_client = ActionClient(self, MoveGroup, self.move_action)
        self.button_server = InteractiveMarkerServer(self, "plan_to_tag_button")

        # RViz preview topics — used when execute=False to show ghost robot
        self.rviz_start_pub = self.create_publisher(
            Empty, "/rviz/moveit/update_start_state", 10
        )
        self.rviz_goal_pub = self.create_publisher(
            RobotState, "/rviz/moveit/update_custom_goal_state", 10
        )
        self.captured_marker_pub = self.create_publisher(
            Marker, "/apriltag/captured_tag_marker", 10
        )

        self.captured_tag_pose = None
        self.captured_camera_target = None
        self.latest_joint_state = None

        self.create_subscription(Empty, "/apriltag/plan_to_tag", self.on_trigger, 10)
        self.create_subscription(Empty, "/apriltag/capture_tag", self.on_capture, 10)
        self.create_subscription(
            Empty, "/apriltag/plan_captured_tag", self.on_plan_captured, 10
        )
        self.create_subscription(
            PointStamped, "/apriltag/plan_to_tag_point", self.on_point_trigger, 10
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self._make_buttons()

        mode = "execute" if self.execute else "plan-preview"
        self.get_logger().info(
            f"PlanToTag ready mode={mode} group={self.group_name} "
            f"camera={self.camera_frame} approach_distance={self.approach_distance:.3f}m "
            f"tag={self.tag_frame} capture=/apriltag/capture_tag "
            "plan=/apriltag/plan_captured_tag rviz_buttons=/plan_to_tag_button"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def on_trigger(self, _msg: Empty) -> None:
        self.plan_to_tag("topic")

    def on_capture(self, _msg: Empty) -> None:
        self.capture_tag("topic")

    def on_plan_captured(self, _msg: Empty) -> None:
        self.plan_captured("topic")

    def on_point_trigger(self, _msg: PointStamped) -> None:
        self.capture_tag("rviz")

    def on_button_feedback(self, feedback) -> None:
        if feedback.event_type != feedback.BUTTON_CLICK:
            return
        if feedback.marker_name == "capture_tag":
            self.capture_tag("rviz_button")
        elif feedback.marker_name == "plan_captured":
            self.plan_captured("rviz_button")

    def _make_buttons(self) -> None:
        self._insert_button(
            name="capture_tag",
            description="CAPTURE TAG",
            y_offset=0.0,
            color=(0.10, 0.40, 1.0),
        )
        self._insert_button(
            name="plan_captured",
            description="PLAN CAPTURED",
            y_offset=-0.14,
            color=(0.05, 0.65, 0.20),
        )
        self.button_server.applyChanges()

    def _insert_button(
        self, name: str, description: str, y_offset: float, color
    ) -> None:
        button = InteractiveMarker()
        button.header.frame_id = self.fixed_frame
        button.name = name
        button.description = description
        button.scale = 0.16
        button.pose.position.x = self.button_x
        button.pose.position.y = self.button_y + y_offset
        button.pose.position.z = self.button_z
        button.pose.orientation.w = 1.0

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 0.20
        box.scale.y = 0.08
        box.scale.z = 0.04
        box.color.r = color[0]
        box.color.g = color[1]
        box.color.b = color[2]
        box.color.a = 0.95

        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.z = 0.07
        label.scale.z = 0.035
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = description

        control = InteractiveMarkerControl()
        control.name = "click"
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.extend([box, label])
        button.controls.append(control)

        self.button_server.insert(button, feedback_callback=self.on_button_feedback)

    def plan_to_tag(self, source: str) -> None:
        if self.capture_tag(source):
            self.plan_captured(source)

    def capture_tag(self, source: str) -> bool:
        tag_tf = self._lookup(self.fixed_frame, self.tag_frame)
        if tag_tf is None:
            return False

        tag = tag_tf.transform.translation

        captured_tag = Pose()
        captured_tag.position.x = tag.x
        captured_tag.position.y = tag.y
        captured_tag.position.z = tag.z
        captured_tag.orientation = tag_tf.transform.rotation

        # Default target: camera goes to the tag's XYZ position
        target = Pose()
        target.position.x = tag.x
        target.position.y = tag.y
        target.position.z = tag.z
        target.orientation.w = 1.0

        if self.approach_distance > 0.0:
            camera_tf = self._lookup(self.fixed_frame, self.camera_frame)
            if camera_tf is not None:
                cam = camera_tf.transform.translation
                dx = cam.x - tag.x
                dy = cam.y - tag.y
                dz = cam.z - tag.z
                norm = math.sqrt(dx * dx + dy * dy + dz * dz)
                if norm > 1e-6:
                    scale = self.approach_distance / norm
                    target.position.x = tag.x + dx * scale
                    target.position.y = tag.y + dy * scale
                    target.position.z = tag.z + dz * scale

        self.captured_tag_pose = captured_tag
        self.captured_camera_target = target
        self._publish_captured_marker()
        self.get_logger().info(
            f"{source} capture: {self.tag_frame} at "
            f"({tag.x:.3f}, {tag.y:.3f}, {tag.z:.3f}) in {self.fixed_frame}"
        )
        return True

    def plan_captured(self, source: str) -> None:
        if self.captured_camera_target is None:
            self.get_logger().warn("No captured tag. Press CAPTURE TAG first.")
            return

        self._publish_captured_marker()

        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"MoveGroup not available: {self.move_action}")
            return

        target = self.captured_camera_target
        plan_only = not self.execute

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.goal_tolerance]

        center = Pose()
        center.position = target.position
        center.orientation.w = 1.0

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(center)

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
        goal_constraints.name = "tag_position"
        goal_constraints.position_constraints.append(pc)
        req.goal_constraints.append(goal_constraints)
        if self.latest_joint_state is not None:
            req.start_state.joint_state = self.latest_joint_state
            req.start_state.is_diff = True

        options = PlanningOptions()
        options.plan_only = plan_only
        options.planning_scene_diff.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = options

        action = "plan" if plan_only else "execute"
        self.get_logger().info(
            f"{source} {action}: moving {self.camera_frame} to "
            f"({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})"
        )
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, plan_only=plan_only)
        )

    def _on_goal_response(self, future, plan_only: bool = False) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup rejected the goal")
            return
        self.get_logger().info("MoveGroup accepted goal")
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, plan_only=plan_only)
        )

    def _on_result(self, future, plan_only: bool = False) -> None:
        result = future.result().result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveGroup failed: error_code={code}")
            return

        action = "planned" if plan_only else "executed"
        self.get_logger().info(f"MoveGroup {action} in {result.planning_time:.2f}s")

        if plan_only:
            self._push_goal_state_to_rviz(result)

    def _push_goal_state_to_rviz(self, result) -> None:
        traj = result.planned_trajectory.joint_trajectory
        if not traj.points:
            return
        last = traj.points[-1]
        js = JointState()
        js.name = list(traj.joint_names)
        js.position = list(last.positions)
        goal_state = RobotState()
        goal_state.joint_state = js
        goal_state.is_diff = False
        self.rviz_start_pub.publish(Empty())
        self.rviz_goal_pub.publish(goal_state)

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

    def _publish_captured_marker(self) -> None:
        if self.captured_tag_pose is None:
            return
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "captured_tag"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self.captured_tag_pose
        marker.scale.x = self.tag_size * 1.5
        marker.scale.y = self.tag_size
        marker.scale.z = self.tag_size * 0.6
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 0.75
        self.captured_marker_pub.publish(marker)


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
