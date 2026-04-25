#!/usr/bin/env python3
"""Trigger a conservative MoveIt plan toward the currently tracked AprilTag."""

import math

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PointStamped, Pose
from interactive_markers import InteractiveMarkerServer
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, PositionConstraint
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


ARM_JOINTS = [
    "revolute_1_0",
    "revolute_2_0",
    "revolute_3_0",
    "revolute_4_0",
    "revolute_5_0",
    "revolute_6_0",
    "revolute_7_0",
]


class PlanToTag(Node):
    def __init__(self) -> None:
        super().__init__("plan_to_tag")

        self.fixed_frame = str(self.declare_parameter("fixed_frame", "root").value)
        self.tag_frame = str(self.declare_parameter("tag_frame", "tag_0").value)
        self.camera_frame = str(
            self.declare_parameter("camera_frame", "camera_optical_frame").value
        )
        self.target_link = str(
            self.declare_parameter("target_link", "wrist_link").value
        )
        self.group_name = str(self.declare_parameter("group_name", "arm").value)
        self.move_action = str(
            self.declare_parameter("move_action", "/move_action").value
        )
        self.ik_service = str(
            self.declare_parameter("ik_service", "/compute_ik").value
        )
        self.execute = bool(self.declare_parameter("execute", False).value)
        self.arm_joint_names = list(
            self.declare_parameter("arm_joint_names", ARM_JOINTS).value
        )
        self.approach_distance = float(
            self.declare_parameter("approach_distance", 0.20).value
        )
        self.proxy_plan_step = float(
            self.declare_parameter("proxy_plan_step", 0.10).value
        )
        self.max_plan_step = float(
            self.declare_parameter("max_plan_step", self.proxy_plan_step).value
        )
        self.ik_search_samples = int(
            self.declare_parameter("ik_search_samples", 20).value
        )
        self.tag_size = float(self.declare_parameter("tag_size", 0.162).value)
        self.goal_tolerance = float(
            self.declare_parameter("goal_tolerance", 0.04).value
        )
        self.allowed_planning_time = float(
            self.declare_parameter("allowed_planning_time", 5.0).value
        )
        self.ik_timeout = float(self.declare_parameter("ik_timeout", 1.0).value)
        self.joint_goal_tolerance = float(
            self.declare_parameter("joint_goal_tolerance", 0.015).value
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
        self.ik_client = self.create_client(GetPositionIK, self.ik_service)
        self.button_server = InteractiveMarkerServer(self, "plan_to_tag_button")

        self.display_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        self.goal_marker_pub = self.create_publisher(
            Marker, "/apriltag/plan_goal_marker", 10
        )
        self.captured_marker_pub = self.create_publisher(
            Marker, "/apriltag/captured_tag_marker", 10
        )
        self.captured_tag_pose = None
        self.captured_camera_start_pose = None
        self.captured_target_pose = None
        self.captured_camera_target_pose = None
        self.captured_desired_camera_pose = None
        self.captured_plan_step = None
        self.captured_range = None
        self.captured_target_link_orientation = None
        self.captured_target_to_camera_offset = None
        self.latest_joint_state = None

        self.create_subscription(Empty, "/apriltag/plan_to_tag", self.on_trigger, 10)
        self.create_subscription(Empty, "/apriltag/capture_tag", self.on_capture, 10)
        self.create_subscription(
            Empty, "/apriltag/plan_captured_tag", self.on_plan_captured, 10
        )
        self.create_subscription(
            PointStamped,
            "/apriltag/plan_to_tag_point",
            self.on_point_trigger,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self._make_buttons()

        mode = "plan+execute" if self.execute else "plan-only"
        self.get_logger().info(
            "PlanToTag ready "
            f"mode={mode} group={self.group_name} target_link={self.target_link} "
            f"proxy_plan_step={self.proxy_plan_step:.3f}m max_plan_step={self.max_plan_step:.3f}m "
            f"tag={self.tag_frame} capture=/apriltag/capture_tag plan=/apriltag/plan_captured_tag "
            "rviz_buttons=/plan_to_tag_button"
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

    def _insert_button(self, name: str, description: str, y_offset: float, color) -> None:
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
        captured = self.capture_tag(source)
        if captured:
            self.plan_captured(source)

    def capture_tag(self, source: str) -> bool:
        tag_tf = self._lookup(self.fixed_frame, self.tag_frame)
        camera_tf = self._lookup(self.fixed_frame, self.camera_frame)
        target_link_tf = None
        target_to_camera_tf = None
        if self.target_link != self.camera_frame:
            target_link_tf = self._lookup(self.fixed_frame, self.target_link)
            target_to_camera_tf = self._lookup(self.target_link, self.camera_frame)

        if (
            tag_tf is None
            or camera_tf is None
            or (self.target_link != self.camera_frame and target_link_tf is None)
            or (self.target_link != self.camera_frame and target_to_camera_tf is None)
        ):
            return False

        tag = tag_tf.transform.translation
        camera = camera_tf.transform.translation
        dx = camera.x - tag.x
        dy = camera.y - tag.y
        dz = camera.z - tag.z
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-6:
            self.get_logger().warn("Camera and tag are at the same TF point; cannot choose approach side")
            return False

        scale = self.approach_distance / norm
        captured_tag = Pose()
        captured_tag.position.x = tag.x
        captured_tag.position.y = tag.y
        captured_tag.position.z = tag.z
        captured_tag.orientation = tag_tf.transform.rotation

        camera_target = Pose()
        camera_target.position.x = tag.x + dx * scale
        camera_target.position.y = tag.y + dy * scale
        camera_target.position.z = tag.z + dz * scale
        camera_target.orientation = camera_tf.transform.rotation
        desired_camera_target = camera_target

        camera_start = Pose()
        camera_start.position.x = camera.x
        camera_start.position.y = camera.y
        camera_start.position.z = camera.z
        camera_start.orientation = camera_tf.transform.rotation

        target_orientation = camera_tf.transform.rotation
        target_offset = None
        if self.target_link != self.camera_frame:
            target_orientation = target_link_tf.transform.rotation
            target_offset = target_to_camera_tf.transform.translation

        full_step = self._pose_distance(camera_start, desired_camera_target)
        search_step = self._search_step(full_step)
        camera_target = self._interpolate_pose(camera_start, desired_camera_target, search_step)
        target = self._target_pose_from_camera_pose(camera_target, target_orientation, target_offset)

        self.captured_tag_pose = captured_tag
        self.captured_camera_start_pose = camera_start
        self.captured_target_pose = target
        self.captured_camera_target_pose = camera_target
        self.captured_desired_camera_pose = desired_camera_target
        self.captured_plan_step = search_step
        self.captured_range = norm
        self.captured_target_link_orientation = target_orientation
        self.captured_target_to_camera_offset = target_offset
        self._publish_captured_marker()
        self._clear_goal_marker()
        self.get_logger().info(
            f"{source} capture: froze {self.tag_frame} at "
            f"({captured_tag.position.x:.3f}, {captured_tag.position.y:.3f}, {captured_tag.position.z:.3f}) "
            f"in {self.fixed_frame}; range={norm:.3f}m; "
            f"proxy_step={search_step:.3f}m full_approach_step={full_step:.3f}m"
        )
        return True

    def plan_captured(self, source: str) -> None:
        if self.captured_desired_camera_pose is None or self.captured_camera_start_pose is None:
            self.get_logger().warn("No captured tag pose yet. Press CAPTURE TAG first.")
            return

        self._publish_captured_marker()
        self._clear_goal_marker()
        self.get_logger().info(
            f"{source} plan: searching closest plannable proxy {self.target_link} target "
            f"in {self.fixed_frame}; captured range={self.captured_range:.3f}m "
            f"proxy_step={self.captured_plan_step:.3f}m"
        )

        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"MoveIt IK service not available: {self.ik_service}")
            return

        candidates = self._ik_candidates()
        if not candidates:
            self.get_logger().warn("No IK candidates generated for captured tag")
            return

        self._try_ik_candidates(candidates, source, 0)

    def _try_ik_candidates(self, candidates, source: str, index: int) -> None:
        if index >= len(candidates):
            self.get_logger().error(
                "MoveIt could not find any plannable proxy pose toward the captured tag"
            )
            return

        camera_target, target, step = candidates[index]
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.target_link
        request.ik_request.pose_stamped.header.frame_id = self.fixed_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = target
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout = self._duration_msg(self.ik_timeout)
        if self.latest_joint_state is not None:
            request.ik_request.robot_state.joint_state = self.latest_joint_state
            request.ik_request.robot_state.is_diff = True

        future = self.ik_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_ik_result(
                result,
                source,
                candidates,
                index,
                camera_target,
                target,
                step,
            )
        )

    def _on_ik_result(
        self,
        future,
        source: str,
        candidates,
        index: int,
        camera_target: Pose,
        target: Pose,
        step: float,
    ) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"MoveIt IK request failed: {exc}")
            return

        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            if index == 0:
                self.get_logger().warn(
                    f"Ideal captured pose is not IK-solvable; backing off from step={step:.3f}m"
                )
            self._try_ik_candidates(candidates, source, index + 1)
            return

        joints = self._selected_joint_positions(result.solution.joint_state)
        if len(joints) != len(self.arm_joint_names):
            found = ", ".join(name for name, _ in joints) or "none"
            expected = ", ".join(self.arm_joint_names)
            self.get_logger().warn(
                f"MoveIt IK returned incomplete arm joint state for step={step:.3f}m. "
                f"found=[{found}] expected=[{expected}]; trying next candidate"
            )
            self._try_ik_candidates(candidates, source, index + 1)
            return

        self.get_logger().info(
            f"MoveIt IK solved candidate step={step:.3f}m after {index + 1} attempt(s); "
            "checking MoveGroup plan"
        )
        self._send_joint_goal(
            joints,
            source=source,
            candidates=candidates,
            index=index,
            camera_target=camera_target,
            target=target,
            step=step,
        )

    def _send_joint_goal(
        self,
        joints,
        source: str = "",
        candidates=None,
        index: int = 0,
        camera_target: Pose = None,
        target: Pose = None,
        step: float = 0.0,
    ) -> None:
        if not self.move_group.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"MoveGroup action server not available: {self.move_action}")
            return

        goal = MoveGroup.Goal()
        goal.request = self._joint_motion_plan_request(joints)
        goal.planning_options = self._planning_options()

        future = self.move_group.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._on_goal_response(
                result,
                source=source,
                candidates=candidates,
                index=index,
                camera_target=camera_target,
                target=target,
                step=step,
            )
        )

    def _selected_joint_positions(self, joint_state: JointState):
        by_name = dict(zip(joint_state.name, joint_state.position))
        return [
            (name, by_name[name])
            for name in self.arm_joint_names
            if name in by_name
        ]

    def _duration_msg(self, seconds: float) -> DurationMsg:
        whole = int(seconds)
        msg = DurationMsg()
        msg.sec = whole
        msg.nanosec = int((seconds - whole) * 1_000_000_000)
        return msg

    def _ik_candidates(self):
        full_step = self._pose_distance(
            self.captured_camera_start_pose,
            self.captured_desired_camera_pose,
        )
        search_step = self._search_step(full_step)
        samples = max(self.ik_search_samples, 1)
        candidates = []
        seen = set()
        for index in range(samples + 1):
            fraction = 1.0 - (index / samples)
            step = search_step * fraction
            key = round(step, 5)
            if key in seen:
                continue
            seen.add(key)
            camera_target = self._interpolate_pose(
                self.captured_camera_start_pose,
                self.captured_desired_camera_pose,
                step,
            )
            target = self._target_pose_from_camera_pose(
                camera_target,
                self.captured_target_link_orientation,
                self.captured_target_to_camera_offset,
            )
            candidates.append((camera_target, target, step))
        return candidates

    def _search_step(self, full_step: float) -> float:
        max_step = max(self.max_plan_step, 0.0)
        if max_step <= 0.0:
            return full_step
        return min(full_step, max_step)

    def _pose_distance(self, start: Pose, end: Pose) -> float:
        dx = end.position.x - start.position.x
        dy = end.position.y - start.position.y
        dz = end.position.z - start.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _interpolate_pose(self, start: Pose, end: Pose, step: float) -> Pose:
        full_step = self._pose_distance(start, end)
        if full_step < 1e-6:
            return end

        ratio = max(0.0, min(step / full_step, 1.0))
        pose = Pose()
        pose.position.x = start.position.x + (end.position.x - start.position.x) * ratio
        pose.position.y = start.position.y + (end.position.y - start.position.y) * ratio
        pose.position.z = start.position.z + (end.position.z - start.position.z) * ratio
        pose.orientation = end.orientation
        return pose

    def _target_pose_from_camera_pose(self, camera_pose: Pose, target_orientation, offset) -> Pose:
        if self.target_link == self.camera_frame or offset is None:
            return camera_pose

        target = Pose()
        target.orientation = target_orientation
        ox, oy, oz = self._rotate_vector(target.orientation, offset)
        target.position.x = camera_pose.position.x - ox
        target.position.y = camera_pose.position.y - oy
        target.position.z = camera_pose.position.z - oz
        return target

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

    def _base_motion_plan_request(self) -> MotionPlanRequest:
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
        if self.latest_joint_state is not None:
            request.start_state.joint_state = self.latest_joint_state
            request.start_state.is_diff = True
        return request

    def _joint_motion_plan_request(self, joints) -> MotionPlanRequest:
        request = self._base_motion_plan_request()
        constraints = Constraints()
        constraints.name = f"{self.tag_frame}_captured_ik"
        for name, position in joints:
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = self.joint_goal_tolerance
            joint.tolerance_below = self.joint_goal_tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints.append(constraints)
        return request

    def _motion_plan_request(self, target: Pose) -> MotionPlanRequest:
        request = self._base_motion_plan_request()

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.goal_tolerance]

        position = PositionConstraint()
        position.header.frame_id = self.fixed_frame
        position.link_name = self.target_link
        position.target_point_offset = self._target_point_offset()
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(target)
        position.weight = 1.0

        constraints = Constraints()
        constraints.name = f"{self.tag_frame}_approach"
        constraints.position_constraints.append(position)
        request.goal_constraints.append(constraints)
        return request

    def _rotate_vector(self, quaternion, vector):
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        vx = vector.x
        vy = vector.y
        vz = vector.z

        uvx = qy * vz - qz * vy
        uvy = qz * vx - qx * vz
        uvz = qx * vy - qy * vx
        uuvx = qy * uvz - qz * uvy
        uuvy = qz * uvx - qx * uvz
        uuvz = qx * uvy - qy * uvx

        return (
            vx + 2.0 * (qw * uvx + uuvx),
            vy + 2.0 * (qw * uvy + uuvy),
            vz + 2.0 * (qw * uvz + uuvz),
        )

    def _target_point_offset(self):
        offset = PositionConstraint().target_point_offset
        if self.target_link == self.camera_frame:
            return offset

        transform = self._lookup(self.target_link, self.camera_frame)
        if transform is None:
            self.get_logger().warn(
                f"Could not resolve {self.target_link} -> {self.camera_frame}; "
                "planning with zero target_point_offset"
            )
            return offset

        offset.x = transform.transform.translation.x
        offset.y = transform.transform.translation.y
        offset.z = transform.transform.translation.z
        return offset

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

    def _clear_goal_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "plan_to_tag"
        marker.id = 1
        marker.action = Marker.DELETE
        self.goal_marker_pub.publish(marker)

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

    def _on_goal_response(
        self,
        future,
        source: str = "",
        candidates=None,
        index: int = 0,
        camera_target: Pose = None,
        target: Pose = None,
        step: float = 0.0,
    ) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            if candidates is not None:
                self.get_logger().warn(
                    f"MoveGroup rejected candidate step={step:.3f}m; trying next candidate"
                )
                self._try_ik_candidates(candidates, source, index + 1)
            else:
                self.get_logger().error("MoveGroup rejected the plan-to-tag request")
            return

        self.get_logger().info("MoveGroup accepted the plan-to-tag request")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_result(
                result,
                source=source,
                candidates=candidates,
                index=index,
                camera_target=camera_target,
                target=target,
                step=step,
            )
        )

    def _on_result(
        self,
        future,
        source: str = "",
        candidates=None,
        index: int = 0,
        camera_target: Pose = None,
        target: Pose = None,
        step: float = 0.0,
    ) -> None:
        result = future.result().result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            if candidates is not None:
                self.get_logger().warn(
                    f"MoveGroup failed candidate step={step:.3f}m: error_code={code}; "
                    "trying next candidate"
                )
                self._try_ik_candidates(candidates, source, index + 1)
            else:
                self.get_logger().error(
                    f"MoveGroup failed plan-to-tag request: error_code={code}"
                )
            return

        if camera_target is not None and target is not None:
            self.captured_camera_target_pose = camera_target
            self.captured_target_pose = target
            self.captured_plan_step = step
            self._publish_goal_marker(camera_target)
            self.get_logger().info(
                f"MoveIt selected closest plannable target at step={step:.3f}m "
                f"after {index + 1} candidate attempt(s)"
            )

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
