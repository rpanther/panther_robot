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
import actionlib
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequestAction
# buttons
from button import Button


class AudioController:
    """
    Audio controller with play/stop and next song
    """
    def __init__(self, rospy, enable, play_stop, next, global_path, audiolist):
        self.rospy = rospy
        self.enable_button = Button(enable)
        self.play_stop = Button(play_stop)
        self.next = Button(next)
        self.sound_client = SoundClient()
        ac = actionlib.SimpleActionClient('sound_play', SoundRequestAction)
        ac.wait_for_server()
        self.global_path = global_path
        self.audiolist = audiolist
        self.selected = 0
        self.enable = False
        self.song_init = ''
        self.song_exit = ''
        # Load configuration audio
        if self.rospy.has_param('~audio/song/init'):
            self.song_init = self.rospy.get_param("~audio/song/init")
        if self.rospy.has_param('~audio/song/exit'):
            self.song_exit = self.rospy.get_param("~audio/song/exit")
        self.rospy.loginfo("Init:%s - Exit:%s"%(self.song_init, self.song_exit))
        
                
    def startAudio(self):
        self.rospy.loginfo("Play %s"%self.audiolist[self.selected])
        self.sound_client.playWave(self.global_path + "/" + self.audiolist[self.selected])
        
    def init(self):
        #self.rospy.loginfo("Sound controller status[%d]"%self.enable)
        if self.enable:
            if self.song_init != '':
                # Start Hello robot
                self.sound_client.playWave(self.global_path + "/" + self.song_init)
        else:
            if self.song_exit != '':
                # Start Exit robot
                self.sound_client.playWave(self.global_path + "/" + self.song_exit)            
        
    def update(self, buttons):
        if self.enable:
            if self.play_stop.update(buttons):
                self.startAudio()
            if self.next.update(buttons):
                # Increase counter
                self.selected += 1
                if self.selected >= len(self.audiolist):
                    self.selected = 0
                # Start new audio
                self.startAudio()
        if self.enable_button.update(buttons):
            # update status
            self.enable = not self.enable
            # Launch audio start/stop
            self.init()
# EOF
