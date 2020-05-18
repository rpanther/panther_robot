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
from sensor_msgs.msg import Joy
# Topic manager
from .TopicButton import BoolButton, CounterButton, TimeButton
from .ServiceButton import ServiceButton


class ButtonManager:
    """
    Read status button and for each button pressed publish a message
    """
    def __init__(self, joy_topic, name="buttons"):
        self.buttons = {}
        # Get list of buttons
        buttons = rospy.get_param("~{name}".format(name=name), {})
        # Load buttons
        if buttons:
            rospy.loginfo("Buttons list:")
        for name, config in buttons.items():
            if 'buttons' not in config:
                rospy.logwarn("Check {name} button (Miss button)".format(name=name))
                continue
            if not ('topic' in config or 'service' in config):
                rospy.logwarn("Check {name} button (Miss topic or service)".format(name=name))
                continue
            # Load type of button
            type_button = config.get('type', 'bool')
            list_buttons = config['buttons']
            # Initialzie buttons
            if type_button == 'bool':
                status = config.get('status', True)
                topic = config['topic']
                rospy.loginfo("{list_buttons} {name}. topic={topic} - status={status}".format(list_buttons=list_buttons, name=name, topic=topic, status=status))
                # Add buttons in list
                self.buttons[name] = BoolButton(list_buttons, topic, status=status)
            elif type_button == 'counter':
                max_value = config.get('max', 10)
                topic = config['topic']
                rospy.loginfo("{list_buttons} {name}. topic={topic} - max={max}".format(list_buttons=list_buttons, name=name, topic=topic, max=max_value))
                # Add buttons in list
                self.buttons[name] = CounterButton(list_buttons, topic, max_value=max_value)
            elif type_button == 'time':
                time = config.get('time', 3.5)
                topic_timer = config.get('topic_time', "")
                topic = config['topic']
                rospy.loginfo("{list_buttons} {name}. topic={topic} - time={time}".format(list_buttons=list_buttons, name=name, topic=topic, time=time))
                # Add buttons in list
                self.buttons[name] = TimeButton(list_buttons, topic, time=time, topic_timer=topic_timer)
            elif type_button == 'service':
                request = config.get('request', {})
                service = config['service']
                service_class = config.get('class', "")
                rospy.loginfo("{list_buttons} {name}. service={service}".format(list_buttons=list_buttons, name=name, service=service))
                # Add buttons in list
                self.buttons[name] = ServiceButton(list_buttons, service, request, service_class=service_class)
            else:
                rospy.logerr("{name}: {type} not in list!".format(name=name, type=type_button))
        # Launch Joystick reader
        rospy.Subscriber(joy_topic, Joy, self.joy_callback)

    def joy_callback(self, data):
        # Update status buttons
        for button in self.buttons.values():
            button.update(data.buttons)
# EOF
