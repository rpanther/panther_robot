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
from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback


class ForceFeedback:

    def __init__(self):
        # Load force feedback parameters
        joy_feedback = rospy.get_param("~joy_ff", {})
        joy_feedback_topic = joy_feedback.get("topic", "joy/set_feedback")
        self.intensity = joy_feedback.get("intensity", 1.0)
        self.enable = joy_feedback.get("enable", True)
        # Force feedback info:
        rospy.loginfo("FF {topic} intensity={intensity} enable={enable}".format(topic=joy_feedback_topic, intensity=self.intensity, enable=self.enable))
        self.pub = rospy.Publisher(joy_feedback_topic, JoyFeedbackArray, queue_size=10)

    def stop(self, event):
        left = JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=0)
        right = JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=1, intensity=0)
        feed = JoyFeedbackArray([left, right])
        self.pub.publish(feed)

    def feedback(self, intensity=0.5, time=0.2):
        if not self.enable:
            return
        # Force feedback for all motors
        left = JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=self.intensity)
        right = JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=1, intensity=self.intensity)
        feed = JoyFeedbackArray([left, right])
        self.pub.publish(feed)
        # Set a time to stop the vibration
        rospy.Timer(rospy.Duration(time), self.stop, oneshot=True)
# EOF
