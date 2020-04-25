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


class TimeButton:
    """
    Convert a button pressed to timer topic
    """
    def __init__(self, numbers, topic, topic_timer="", time=3.5):
        # Load button reader
        self.buttons = Buttons(numbers)
        # Initialize time
        self.time = time
        # status
        self.status = True
        # Load topic output
        self.pub = rospy.Publisher(topic, Bool, queue_size=10)
        # Set topic timer
        if topic_timer:
            # Load topic output
            self.topic_timer = rospy.Publisher(topic_timer, Int8, queue_size=10)
        self.timer = True if topic_timer else False
        self._old_count = 0

    def update(self, buttons):
        # Update status button
        self.buttons.update(buttons)
        # Check status button
        if self.buttons:
            rospy.logdebug("{buttons} Start timer {time}".format(buttons=self.buttons, time=self.time))
            # Reset counter
            self._old_count = 0
        # Send a topic message every second
        if self.timer and self.status and int(self.buttons.time) > self._old_count:
            self._old_count = int(self.buttons.time)
            # Publish a message
            rospy.logdebug("{buttons} Message={time}".format(buttons=self.buttons, time=int(self.buttons.time)))
            # Publish status
            self.pub.publish(int(self.buttons.time))
        # update status
        if self.buttons.time >= self.time:
            if self.status:
                self.status = False
                rospy.logdebug("{buttons} time={time}".format(buttons=self.buttons, time=self.buttons.time))
                # Publish status
                self.pub.publish(self.status)
        else:
            self.status = True



class CounterButton:
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
