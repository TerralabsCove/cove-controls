#!/usr/bin/env python3
"""Publish an RViz marker at an AprilTag TF frame."""

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, "/apriltag/tag_marker", 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info(
            f"Publishing /apriltag/tag_marker from TF frames under {self.source_frame}"
        )

    def publish_marker(self) -> None:
        markers = []
        first_error = None
        for index, tag_frame in enumerate(self.tag_frames):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.source_frame,
                    str(tag_frame),
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.lookup_timeout),
                )
            except TransformException as exc:
                if first_error is None:
                    first_error = exc
                continue
            markers.extend(self._markers_for_transform(index, str(tag_frame), transform))

        if not markers:
            self.get_logger().warn(
                f"No configured AprilTag TF frames found yet: {first_error}",
                throttle_duration_sec=2.0,
            )
            return

        self.pub.publish(MarkerArray(markers=markers))

    def _markers_for_transform(self, index, tag_frame, transform):
        stamp = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = self.source_frame
        marker.header.stamp = stamp
        marker.ns = "apriltag"
        marker.id = index * 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = transform.transform.translation.x
        marker.pose.position.y = transform.transform.translation.y
        marker.pose.position.z = transform.transform.translation.z
        marker.pose.orientation = transform.transform.rotation
        marker.scale.x = self.tag_size
        marker.scale.y = self.tag_size
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.85

        label = Marker()
        label.header.frame_id = self.source_frame
        label.header.stamp = stamp
        label.ns = "apriltag"
        label.id = index * 2 + 1
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

        return [marker, label]


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
