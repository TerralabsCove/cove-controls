#!/usr/bin/env python3
"""Publish robot_description on a latched topic for RViz and joint_state_publisher."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionTopicPublisher(Node):
    def __init__(self) -> None:
        super().__init__("robot_description_topic")

        self.robot_description = str(
            self.declare_parameter("robot_description", "").value
        )
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, "/robot_description", qos)
        self.timer = self.create_timer(1.0, self.publish_description)
        self.publish_description()

    def publish_description(self) -> None:
        if not self.robot_description:
            self.get_logger().warn("robot_description is empty", throttle_duration_sec=5.0)
            return
        msg = String()
        msg.data = self.robot_description
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionTopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
