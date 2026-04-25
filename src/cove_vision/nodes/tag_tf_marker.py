#!/usr/bin/env python3
"""Publish generic tracked-object markers at AprilTag TF frames."""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class TagTfMarker(Node):
    def __init__(self) -> None:
        super().__init__("tag_tf_marker")

        self.source_frame = str(
            self.declare_parameter("source_frame", "camera_optical_frame").value
        )
        self.tag_frames = list(
            self.declare_parameter(
                "tag_frames",
                [f"tag_{tag_id}" for tag_id in range(21)],
            ).value
        )
        self.tag_size = float(self.declare_parameter("tag_size", 0.162).value)
        self.lookup_timeout = float(self.declare_parameter("lookup_timeout", 0.05).value)
        self.marker_lifetime = float(
            self.declare_parameter("marker_lifetime", 1.5).value
        )
        self.fixed_frame = str(self.declare_parameter("fixed_frame", "root").value)
        self.hold_duration = float(
            self.declare_parameter("hold_duration", self.marker_lifetime).value
        )
        self.object_width = float(
            self.declare_parameter("object_width", self.tag_size * 1.5).value
        )
        self.object_height = float(
            self.declare_parameter("object_height", self.tag_size * 1.0).value
        )
        self.object_depth = float(
            self.declare_parameter("object_depth", self.tag_size * 0.6).value
        )
        self.show_tag_face = bool(
            self.declare_parameter("show_tag_face", False).value
        )
        self.show_label = bool(self.declare_parameter("show_label", False).value)
        self.show_dummy_object = bool(
            self.declare_parameter("show_dummy_object", True).value
        )
        self.dummy_frame = str(self.declare_parameter("dummy_frame", "root").value)
        self.detections_topic = str(
            self.declare_parameter("detections_topic", "/detections").value
        )

        self.active_tag_frames = {}
        self.last_logged_tag_ids = ()
        self.last_logged_missing_tf = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.on_detections,
            10,
        )
        self.pub = self.create_publisher(MarkerArray, "/apriltag/tag_marker", 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info(
            f"Publishing generic tracked-object markers on /apriltag/tag_marker from live detections on {self.detections_topic}; fixed_frame={self.fixed_frame}"
        )

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        now_ns = self.get_clock().now().nanoseconds
        valid_frames = set(self.tag_frames)
        detected_tag_ids = []
        for detection in msg.detections:
            tag_frame = f"tag_{int(detection.id)}"
            if tag_frame in valid_frames:
                self.active_tag_frames[tag_frame] = now_ns
                detected_tag_ids.append(int(detection.id))

        detected_tag_ids = tuple(sorted(detected_tag_ids))
        if detected_tag_ids and detected_tag_ids != self.last_logged_tag_ids:
            self.get_logger().info(
                f"Detected AprilTag ids: {', '.join(str(tag_id) for tag_id in detected_tag_ids)}"
            )
        self.last_logged_tag_ids = detected_tag_ids

    def publish_marker(self) -> None:
        markers = []
        if self.show_dummy_object:
            markers.append(self._dummy_marker())

        now_ns = self.get_clock().now().nanoseconds
        max_age_ns = int(self.hold_duration * 1e9)
        active_frames = [
            tag_frame
            for tag_frame, last_seen_ns in self.active_tag_frames.items()
            if now_ns - last_seen_ns <= max_age_ns
        ]
        self.active_tag_frames = {
            tag_frame: self.active_tag_frames[tag_frame] for tag_frame in active_frames
        }

        for index, tag_frame in enumerate(sorted(active_frames)):
            marker_set = self._markers_for_transform(index, tag_frame)
            if marker_set:
                markers.extend(marker_set)

        if len(markers) == (1 if self.show_dummy_object else 0):
            self.get_logger().warn(
                "No active AprilTag detections yet",
                throttle_duration_sec=2.0,
            )

        if markers:
            self.pub.publish(MarkerArray(markers=markers))

    def _dummy_marker(self):
        stamp = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = self.dummy_frame
        marker.header.stamp = stamp
        marker.ns = "tracked_object_debug"
        marker.id = 9990
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.35
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.14
        marker.scale.y = 0.10
        marker.scale.z = 0.08
        marker.color.r = 0.1
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.frame_locked = True
        return marker

    def _markers_for_transform(self, index, tag_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                tag_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.lookup_timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Detected {tag_frame}, but RViz marker cannot resolve TF {self.fixed_frame} -> {tag_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return []

        stamp = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=self.marker_lifetime).to_msg()

        body = Marker()
        body.header.frame_id = self.fixed_frame
        body.header.stamp = stamp
        body.ns = "tracked_object"
        body.id = index * 3
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = transform.transform.translation.x
        body.pose.position.y = transform.transform.translation.y
        body.pose.position.z = (
            transform.transform.translation.z - self.object_depth * 0.5
        )
        body.pose.orientation = transform.transform.rotation
        body.scale.x = self.object_width
        body.scale.y = self.object_height
        body.scale.z = self.object_depth
        body.color.r = 1.0
        body.color.g = 0.55
        body.color.b = 0.15
        body.color.a = 0.55
        body.frame_locked = True
        body.lifetime = lifetime

        markers = [body]

        if self.show_tag_face:
            tag_face = Marker()
            tag_face.header.frame_id = self.fixed_frame
            tag_face.header.stamp = stamp
            tag_face.ns = "apriltag"
            tag_face.id = index * 3 + 1
            tag_face.type = Marker.CUBE
            tag_face.action = Marker.ADD
            tag_face.pose.position.x = transform.transform.translation.x
            tag_face.pose.position.y = transform.transform.translation.y
            tag_face.pose.position.z = transform.transform.translation.z
            tag_face.pose.orientation = transform.transform.rotation
            tag_face.scale.x = self.tag_size
            tag_face.scale.y = self.tag_size
            tag_face.scale.z = 0.01
            tag_face.color.r = 0.0
            tag_face.color.g = 1.0
            tag_face.color.b = 0.2
            tag_face.color.a = 0.9
            tag_face.frame_locked = True
            tag_face.lifetime = lifetime
            markers.append(tag_face)

        if self.show_label:
            label = Marker()
            label.header.frame_id = self.fixed_frame
            label.header.stamp = stamp
            label.ns = "apriltag"
            label.id = index * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = transform.transform.translation.x
            label.pose.position.y = transform.transform.translation.y
            label.pose.position.z = transform.transform.translation.z + self.tag_size * 0.7
            label.pose.orientation.w = 1.0
            label.scale.z = 0.06
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = tag_frame
            label.frame_locked = True
            label.lifetime = lifetime
            markers.append(label)

        return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagTfMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
