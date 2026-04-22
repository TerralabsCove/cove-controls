#!/usr/bin/env python3
"""AprilTag visual servo tracker for the simple_assembly arm."""

import math

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ALL_JOINTS = [
    "revolute_1_0",
    "revolute_2_0",
    "revolute_3_0",
    "revolute_4_0",
    "revolute_5_0",
    "revolute_6_0",
    "revolute_7_0",
]

LIMITS = {
    "revolute_1_0": (-1.9199, 1.9199),
    "revolute_2_0": (-math.pi, math.pi),
    "revolute_3_0": (-1.9199, 1.9199),
    "revolute_4_0": (-math.pi, math.pi),
    "revolute_5_0": (-1.9199, 1.9199),
    "revolute_6_0": (-math.pi, math.pi),
    "revolute_7_0": (-1.9199, 1.9199),
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class TagTracker(Node):
    def __init__(self):
        super().__init__("tag_tracker")

        self.image_width = float(self.declare_parameter("image_width", 640.0).value)
        self.image_height = float(self.declare_parameter("image_height", 480.0).value)
        self.cx = self.image_width * 0.5
        self.cy = self.image_height * 0.5

        self.target_tag_id = int(self.declare_parameter("tag_id", 0).value)
        self.track_any_tag_if_missing = bool(
            self.declare_parameter("track_any_tag_if_missing", True).value
        )

        self.pan_joint = str(self.declare_parameter("pan_joint", "revolute_1_0").value)
        self.tilt_joint = str(self.declare_parameter("tilt_joint", "revolute_2_0").value)
        self.kp_pan = float(self.declare_parameter("kp_pan", 0.0015).value)
        self.kp_tilt = float(self.declare_parameter("kp_tilt", 0.0015).value)
        self.pan_sign = float(self.declare_parameter("pan_sign", 1.0).value)
        self.tilt_sign = float(self.declare_parameter("tilt_sign", -1.0).value)
        self.deadband_px = float(self.declare_parameter("deadband_px", 25.0).value)
        self.traj_duration_s = float(
            self.declare_parameter("trajectory_duration", 0.15).value
        )

        if self.pan_joint not in ALL_JOINTS:
            raise ValueError(f"pan_joint must be one of {ALL_JOINTS}: {self.pan_joint}")
        if self.tilt_joint not in ALL_JOINTS:
            raise ValueError(f"tilt_joint must be one of {ALL_JOINTS}: {self.tilt_joint}")

        self.positions: dict[str, float] = {j: 0.0 for j in ALL_JOINTS}
        self.joint_states_received = False

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.create_subscription(
            AprilTagDetectionArray, "/detections", self.on_detection, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10
        )

        self.get_logger().info(
            "TagTracker ready "
            f"tag_id={self.target_tag_id} "
            f"kp_pan={self.kp_pan} "
            f"kp_tilt={self.kp_tilt} "
            f"deadband={self.deadband_px}px"
        )

    def on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.positions:
                self.positions[name] = pos
        self.joint_states_received = True

    def on_detection(self, msg: AprilTagDetectionArray):
        if not self.joint_states_received:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=2.0)
            return

        if not msg.detections:
            return

        det = self._select_detection(msg.detections)
        if det is None:
            return

        tag_x = det.centre.x
        tag_y = det.centre.y
        err_x = tag_x - self.cx
        err_y = tag_y - self.cy

        if math.hypot(err_x, err_y) < self.deadband_px:
            return

        pan_cmd = clamp(
            self.positions[self.pan_joint] + self.pan_sign * self.kp_pan * err_x,
            *LIMITS[self.pan_joint],
        )
        tilt_cmd = clamp(
            self.positions[self.tilt_joint] + self.tilt_sign * self.kp_tilt * err_y,
            *LIMITS[self.tilt_joint],
        )

        self.get_logger().info(
            f"tag({tag_x:.0f},{tag_y:.0f})  err({err_x:+.0f},{err_y:+.0f})  "
            f"pan->{pan_cmd:.3f}  tilt->{tilt_cmd:.3f}",
            throttle_duration_sec=0.25,
        )

        self._send_trajectory(pan_cmd, tilt_cmd)

    def _select_detection(self, detections):
        selected = next(
            (d for d in detections if self._detection_id(d) == self.target_tag_id),
            None,
        )
        if selected is not None:
            return selected
        if self.track_any_tag_if_missing:
            return detections[0]
        return None

    @staticmethod
    def _detection_id(detection):
        tag_id = detection.id
        if isinstance(tag_id, (list, tuple)):
            return int(tag_id[0]) if tag_id else None
        return int(tag_id)

    def _send_trajectory(self, pan: float, tilt: float):
        positions = dict(self.positions)
        positions[self.pan_joint] = pan
        positions[self.tilt_joint] = tilt

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ALL_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = [positions[joint] for joint in ALL_JOINTS]
        pt.velocities = [0.0] * 7

        ns = int((self.traj_duration_s % 1) * 1e9)
        pt.time_from_start = Duration(
            sec=int(self.traj_duration_s), nanosec=ns
        )
        traj.points = [pt]
        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = TagTracker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
