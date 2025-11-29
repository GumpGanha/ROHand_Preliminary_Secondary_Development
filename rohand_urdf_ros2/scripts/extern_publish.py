from rohand_selfdef_interface.msg import JointStateWithoutStamp

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# import time
# import math


class ExternalJointStatePublisher(Node):
    def __init__(self):
        super().__init__("external_joint_publisher")
        self.pub = self.create_publisher(JointState, "/external_joint_states", 10)
        # self.timer = self.create_timer(0.1, self.publish_joint_state)  # 10Hz发布

        self.suber_extern_JointStateWithoutTimeStamp = self.create_subscription(
            JointStateWithoutStamp,
            '/external_external_joint_states',
            self.sub_extern_JointStateWithoutTimeStamp_callback,
            10
        )

    def sub_extern_JointStateWithoutTimeStamp_callback(self,msg_withoutJointState):

        self.get_logger().info("Receive Message! transmit ...")
        msg_JointState = JointState()

        msg_JointState.name = msg_withoutJointState.name
        msg_JointState.position = msg_withoutJointState.position
        msg_JointState.velocity = msg_withoutJointState.velocity
        msg_JointState.effort = msg_withoutJointState.effort

        self.pub.publish(msg_JointState)

        


    # def publish_joint_state(self):
    #     msg = JointState()
    #     msg.header.stamp = self.get_clock().now().to_msg()

    #     msg.name = [
    #         'if_slider_link',
    #         'mf_slider_link',
    #         'rf_slider_link',
    #         'lf_slider_link',
    #         'th_slider_link',
    #         'th_root_link'
    #     ]

    #     t = self.get_clock().now().nanoseconds / 1e9
    #     msg.position = [
    #         math.sin(t) * 0.5,  # if_slider_link
    #         math.cos(t) * 0.5,  # mf_slider_link
    #         math.sin(t+1) * 0.5, # rf_slider_link
    #         math.cos(t+1) * 0.5, # lf_slider_link
    #         math.sin(t+2) * 0.3, # th_slider_link
    #         math.cos(t+2) * 0.3  # th_root_link
    #     ]
    #     self.pub.publish(msg)

def main():
    rclpy.init()
    node = ExternalJointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
