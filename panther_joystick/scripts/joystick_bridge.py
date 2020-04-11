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
import os
import netifaces as ni
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from sound_play.msg import SoundRequest
# Local imports
from button import Button
from audio_controller import AudioController

# Definition start stop LED effect
enable_effect = None
status = True
# Publisher LED controller
pub = None


def joy_callback(data):
    #global audio_controller
    global status
    # Check if start and stop led effect button is pressed
    if enable_effect.update(data.buttons):
        pub.publish(status)
        if status:
            rospy.loginfo("Effects ENABLE")
        else:
            rospy.loginfo("Effects DISABLE")
        # Update status
        status = not status

def joystick_bridge():
    joy_topic = rospy.get_param("~joy", "joy")
    # Inizialize led effect controller
    global enable_effect
    button_led = rospy.get_param("~led/enable")
    enable_effect = Button(button_led)
    status = rospy.get_param("~led/topic")
    rospy.loginfo("* LED -> ON/OFF[%d] - Param:%s"%(button_led, status))
    # Initialize audio controller
    audio_controller = AudioController(joy_topic)
    # Initialize led status controller
    global pub
    pub = rospy.Publisher(status, Bool, queue_size=10)
    # Launch Joystick reader
    # rospy.Subscriber(joy_topic, Joy, joy_callback)
    # Print start node
    rospy.loginfo("... %s running!"%rospy.get_name())
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # Print exit message
    rospy.loginfo("%s Stopped!"%rospy.get_name())


if __name__ == '__main__':
    try:
        # Initialzie ROS python node
        rospy.init_node('joystick_bridge', anonymous=True)
        # Start joystick bridge
        joystick_bridge()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
# EOF

