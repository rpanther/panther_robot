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

# ROS libraries
import rospy
from std_msgs.msg import Bool, Int8
# buttons
from .button import Buttons


class TopicButton:
    """
    Convert a button pressed to counter topic
    """
    def __init__(self, numbers, topic, max_value=10):
        self.counter = 0
        self.max_value = max_value
        # Load button reader
        self.buttons = Buttons(numbers)
        # Load topic output
        self.pub = rospy.Publisher(topic, Int8, queue_size=10)

    def update(self, buttons):
        # Update status button
        self.buttons.update(buttons)
        # publish if pressed
        if self.buttons:
            rospy.logdebug("{buttons} max={max_value}".format(buttons=self.buttons, max_value=self.max_value))
            self.pub.publish(self.counter)
            # Update status
            self.counter += 1
            # reset counter
            if self.counter > self.max_value:
                self.counter = 0


class BoolButton:
    """
    Convert a button pressed to boolean topic
    """
    def __init__(self, numbers, topic, status=True):
        self.status = status
        # Load button reader
        self.buttons = Buttons(numbers)
        # Load topic output
        self.pub = rospy.Publisher(topic, Bool, queue_size=10)

    def update(self, buttons):
        # Update status button
        self.buttons.update(buttons)
        # publish if pressed
        if self.buttons:
            rospy.logdebug("{buttons} status={status}".format(buttons=self.buttons, status=self.status))
            self.pub.publish(self.status)
            # Update status
            self.status = not self.status
# EOF
