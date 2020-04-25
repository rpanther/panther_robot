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


class Buttons:
    """
    Button class definition
    """

    class ButtonException(Exception):
        pass

    def __init__(self, numbers):
        numbers = [numbers] if isinstance(numbers, int) else numbers
        self.numbers = numbers
        self._state = False
        self._old_state = False
        self._click = False
        self.status = "release"
        self._start = 0
    
    def update(self, buttons):
        self._click = False
        # Read button
        for num in self.numbers:
            if num >= len(buttons):
                raise Button.ButtonException("Button [{num}] not in list".format(num=num))
        # Read button
        self._state = all([buttons[num] for num in self.numbers])
        self.status = "pressed" if self._state else "released" 
        # Check edge
        if self._state and not self._old_state:
            self._click = True
            # Initialize timer
            self._start = rospy.get_time()
        # Update status
        self._old_state = self._state

    @property
    def time(self):
        if self.status != "pressed":
            return 0
        return rospy.get_time() - self._start

    def __nonzero__(self):
        return self._click

    def __str__(self):
        return str(self.numbers)
    
    def __repr__(self):
        return "{status} {click}".format(status=self.status, click=self._click)
# EOF
