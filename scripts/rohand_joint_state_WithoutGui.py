# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# Copyright (c) 2025, CRP developers.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import sys
import rclpy
from sensor_msgs.msg import JointState
from joint_state_publisher.joint_state_publisher import JointStatePublisher

# ROHand目标关节列表
ROHAND_SLIDER_LIST = [
    'if_slider_link',
    'mf_slider_link',
    'rf_slider_link',
    'lf_slider_link',
    'th_slider_link',
    'th_root_link'
]


class ExternalJointSyncNode:
    def __init__(self, urdf_file=None):

        # Create an instance of JointStatePublisher (core Publisher)
        self.jsp = JointStatePublisher(urdf_file)

        # Subscribe to the topic on external joint conditions
        self.subscribed_topic = "/external_joint_states"
        self.joint_state_sub = self.jsp.create_subscription(
            JointState,
            self.subscribed_topic,
            self.external_joint_state_callback,
            rclpy.qos.QoSProfile(depth=10)
        )
        
        # Log prompt
        self.jsp.get_logger().info(f"Subscribed to the external joint topic {self.subscribed_topic}")
        self.jsp.get_logger().info("Start synchronizing joint data and release...")

    def external_joint_state_callback(self, msg: JointState):
        """External joint data callback: Update the joint position of JSP"""
        for idx, joint_name in enumerate(msg.name):
            # Only process the joints in the target joint list
            if joint_name in ROHAND_SLIDER_LIST and joint_name in self.jsp.free_joints:
                if idx < len(msg.position):
                    # Get Joints Config # free_joints's reference
                    joint = self.jsp.free_joints[joint_name]
                    # Check Joints Limits
                    new_pos = max(joint['min'], min(joint['max'], msg.position[idx]))
                    # Update Joints Position
                    joint['position'] = new_pos
                    self.jsp.get_logger().debug(f"Update Joints {joint_name}: {new_pos:.3f}")


def main():
    rclpy.init() 

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file', help='URDF file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    sync_node = ExternalJointSyncNode(parsed_args.urdf_file)

    rclpy.spin(sync_node.jsp) 

    sync_node.jsp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
