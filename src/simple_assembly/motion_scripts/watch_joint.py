#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class WatchJoint(Node):
    def __init__(self):
        super().__init__('watch_joint')
        self.last_pos = None
        self.last_time = None
        self.create_subscription(JointState, '/joint_states', self.cb, 10)

    def cb(self, msg):
        if not msg.position:
            return

        # pick joint 0 for now
        pos = msg.position[0]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is not None:
            dt = t - self.last_time
            dp = pos - self.last_pos
            print(f"dt={dt:7.4f}  pos={pos: .5f}  dpos={dp: .5f}")

        self.last_time = t
        self.last_pos = pos

def main():
    rclpy.init()
    node = WatchJoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()