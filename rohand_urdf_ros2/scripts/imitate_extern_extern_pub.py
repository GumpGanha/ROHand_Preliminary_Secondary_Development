from rohand_selfdef_interface.msg import JointStateWithoutStamp

import rclpy
from rclpy.node import Node
import time
import math

class ExternExternalJointStatePublisher(Node):
    def __init__(self):
        super().__init__("external_external_joint_publisher")
        self.pub = self.create_publisher(JointStateWithoutStamp, "/external_external_joint_states", 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)  # 10Hz发布

    def publish_joint_state(self):
        msg = JointStateWithoutStamp()

        msg.name = [
            'if_slider_link',
            'mf_slider_link',
            'rf_slider_link',
            'lf_slider_link',
            'th_slider_link',
            'th_root_link'
        ]

        t = self.get_clock().now().nanoseconds / 1e9
        msg.position = [
            math.sin(t) * 0.5,  # if_slider_link
            math.cos(t) * 0.5,  # mf_slider_link
            math.sin(t+1) * 0.5, # rf_slider_link
            math.cos(t+1) * 0.5, # lf_slider_link
            math.sin(t+2) * 0.3, # th_slider_link
            math.cos(t+2) * 0.3  # th_root_link
        ]
        msg.velocity = [] 
        msg.effort = []
        self.pub.publish(msg)

        self.get_logger().info(f"Publish Message : {dict(zip(msg.name, msg.position))}")


def main():
    rclpy.init()
    node = ExternExternalJointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



