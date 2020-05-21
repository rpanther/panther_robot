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

from threading import Thread
# ROS libraries
import rospy
import genpy.message
import rosservice
# buttons
from .button import Buttons, TimeButtons


class ServiceButton:

    def __init__(self, numbers, service, request, time=0):
        self.service = service
        # Load request
        self.request = request
        # Load button reader
        self.buttons = TimeButtons(numbers, timeout=time)
        # Service classes
        self.service_proxy = None
        self.service_class = None
        # Start Service client
        self._thread = Thread(target=self._init_service, args=[])
        self._thread.start()

    def _init_service(self):
        try:
            rospy.wait_for_service(self.service)
            # Extract service class by name
            self.service_class = rosservice.get_service_class_by_name(self.service)
            self.service_proxy = rospy.ServiceProxy(self.service, self.service_class)
            rospy.loginfo("Initialized {service}".format(service=self.service))
        except rospy.ServiceException, error:
            rospy.logerr("Service call failed: {error}".format(error=error))
        except rosservice.ROSServiceException, error:
            rospy.logerr("Service call failed: {error}".format(error=error))
        except rospy.ROSException, error:
            rospy.loginfo("Service error")

    def update(self, buttons):
        # Update status button
        self.buttons.update(buttons)
        # publish if pressed
        if not self.buttons:
            return
        rospy.logdebug("{buttons}".format(buttons=self.buttons))
        # Call service
        if self.service_proxy is None and self.service_class is None:
            rospy.logerr("Service {service} not initializated".format(service=self.service))
            return
        # Make service message
        service_class = self.service_class._request_class()
        try:
            genpy.message.fill_message_args(service_class, [self.request])
        except genpy.MessageException, error:
            rospy.logerr("Message in {service}: {error}".format(service=self.service, error=error))
            return
        # Run  service proxy
        try:
            res = self.service_proxy(service_class)
            rospy.loginfo("Output {service} {res}".format(service=self.service, res=res.return_))
        except rospy.ServiceException, error:
            rospy.logerr("Service call failed: {error}".format(error=error))
            # Restart initialization thread
            self._thread.join()
            if not self._thread.is_alive():
                self._thread = Thread(target=self._init_service, args=[])
                self._thread.start()
# EOF
