#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from zed_interfaces.srv import set_pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class ZED2InitialPose:

    def __init__(self, service_name='/zed/set_pose'):
        # Wait zed service
        rospy.wait_for_service(service_name)
        self.set_pose = rospy.ServiceProxy(service_name, set_pose)
        # Initialize initalpose subscriber
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.pose_estimate)

    def pose_estimate(self, data):
        # Extract pose end Euler angles
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        rpy = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        # Type: zed_interfaces/set_pose
        # Args: x y z R P Y
        try:
            resp = self.set_pose(position.x, position.y, position.z, rpy[0], rpy[1], rpy[2])
            if resp.done:
                rospy.loginfo("New ZED pose xyz=({x}, {y}, {z}) rpy={rpy}".format(x=position.x, y=position.y, z=position.z, rpy=rpy))
            else:
                rospy.logerr("Invalid ZED pose")
        except rospy.ServiceException as error:
            rospy.loginfo("Service call failed: {error}".format(error=error))


def zed_to_initialpose():
    rospy.init_node('zed_to_initialpose')
    rospy.loginfo("Waiting ZED service")
    # Load zed converter
    ZED2InitialPose()
    # Node converter ready
    rospy.loginfo("ZED to initialpose ready")
    # Wait node until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    zed_to_initialpose()
# EOF
