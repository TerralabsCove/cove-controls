#!/usr/bin/env python3
"""Publish generic tracked-object markers at AprilTag TF frames."""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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
            self.declare_parameter("marker_lifetime", 0.25).value
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, "/apriltag/tag_marker", 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info(
            f"Publishing generic tracked-object markers on /apriltag/tag_marker from TF frames under {self.source_frame}"
        )

    def publish_marker(self) -> None:
        markers = []
        if self.show_dummy_object:
            markers.append(self._dummy_marker())
        first_error = None
        for index, tag_frame in enumerate(self.tag_frames):
            try:
                self.tf_buffer.lookup_transform(
                    self.source_frame,
                    str(tag_frame),
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.lookup_timeout),
                )
            except TransformException as exc:
                if first_error is None:
                    first_error = exc
                continue
            markers.extend(self._markers_for_transform(index, str(tag_frame)))

        if not markers:
            self.get_logger().warn(
                f"No configured AprilTag TF frames found yet: {first_error}",
                throttle_duration_sec=2.0,
            )
            return

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
        stamp = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=self.marker_lifetime).to_msg()

        body = Marker()
        body.header.frame_id = tag_frame
        body.header.stamp = stamp
        body.ns = "tracked_object"
        body.id = index * 3
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.z = -self.object_depth * 0.5
        body.pose.orientation.w = 1.0
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
            tag_face.header.frame_id = tag_frame
            tag_face.header.stamp = stamp
            tag_face.ns = "apriltag"
            tag_face.id = index * 3 + 1
            tag_face.type = Marker.CUBE
            tag_face.action = Marker.ADD
            tag_face.pose.orientation.w = 1.0
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
            label.header.frame_id = tag_frame
            label.header.stamp = stamp
            label.ns = "apriltag"
            label.id = index * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.z = self.tag_size * 0.7
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
