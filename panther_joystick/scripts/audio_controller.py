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

import os
# ROS libraries
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequestAction
# buttons
from buttons import Button


def info_song(song, name):
    if os.path.isfile(song):
        rospy.loginfo("Song {name}: {song}".format(name=name, song=song))
    else:
        rospy.logwarn("{name} song {song} does not exist!".format(name=name, song=song))


class AudioController:
    """
    Audio controller with play/stop and next song
    """
    def __init__(self, joy_topic, sound_client="sound_play", audio="audio", speech="speech"):
        self.ntrack = 0
        self.nMessage = 0
        self.enable = False
        self.joy_topic = joy_topic
        self.sound_client_name = sound_client
        # Load music buttons
        self.button_enable = Button(rospy.get_param("~{audio}/enable".format(audio=audio), 0))
        self.play_stop = Button(rospy.get_param("~{audio}/start".format(audio=audio), 1))
        self.next_song = Button(rospy.get_param("~{audio}/next".format(audio=audio), 2))
        rospy.loginfo("Audio controller:")
        rospy.loginfo(" [{enable}] Enable - [{start}] Start/Stop - [{next}] Next".format(enable=self.button_enable,
                                                                                         start=self.play_stop,
                                                                                         next=self.next_song))
        # Load speech button
        self.button_speech = Button(rospy.get_param("~{speech}/button".format(speech=speech), 3))
        rospy.loginfo(" [{button}] speech".format(button=self.button_speech))
        # Load music
        path = rospy.get_param("~{audio}/path".format(audio=audio), "{home}/Music".format(home=os.environ['HOME']))
        # Lists all files in the current directory
        # Selected only the wav files
        self.audiolist = ["{path}/{song}".format(path=path, song=item) for item in os.listdir(path) if item.endswith('.wav')]
        if self.audiolist:
            rospy.loginfo("Audio loaded from {path}: ".format(path=path))
            for idx, item in enumerate(self.audiolist):
                rospy.loginfo(" {idx}. {item}".format(idx=idx, item=os.path.basename(item)))
        else:
            rospy.logwarn("Music list in {path} empty".format(path=path))
        # Load init and exit songs
        song_init = rospy.get_param("~{audio}/song/init".format(audio=audio), "")
        self.song_init = "{path}/{song}".format(path=path, song=song_init) 
        info_song(self.song_init, "init")
        song_exit = rospy.get_param("~{audio}/song/exit".format(audio=audio), "")
        self.song_exit = "{path}/{song}".format(path=path, song=song_exit) 
        info_song(self.song_exit, "exit")
        # Load texts
        self.texts = rospy.get_param("~{speech}/text".format(speech=speech), [])
        if self.texts:
            rospy.loginfo("Texts in list: ")
            for idx, text in enumerate(self.texts):
                rospy.loginfo(" {idx}. {text}".format(idx=idx, text=text))
        else:
            rospy.loginfo("No texts in list!")
        # Enable sound client and wait server
        self.sound_client = SoundClient()
        self.ac = actionlib.SimpleActionClient(self.sound_client_name, SoundRequestAction)
        self.active = False
        # Launch Joystick reader
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)

    def start(self, timeout=rospy.Duration()):
        # Print start node
        rospy.loginfo("Waiting {client} ...".format(client=self.sound_client_name))
        self.ac.wait_for_server(timeout)
        self.active = True
        # Print start node
        rospy.loginfo("... {client} connected!".format(client=self.sound_client_name))
        # Stop all other sounds
        self.sound_client.stopAll()

    def _startAudio(self, song):
        if os.path.isfile(song):
            if self.active:
                rospy.loginfo("Play {song}".format(song=song))
                self.sound_client.playWave(song)
            else:
                rospy.logerr("[{server}] Server".format(server=self.active))

    def joy_callback(self, data):
        # Update status buttons
        self.play_stop.update(data.buttons)
        self.button_enable.update(data.buttons)
        self.next_song.update(data.buttons)
        self.button_speech.update(data.buttons)
        # Check status button
        if self.button_enable:
            # update status
            self.enable = not self.enable
            # Launch audio start/stop
            if self.enable:
                rospy.loginfo("Audio enabled")
                self._startAudio(self.song_init)
            else:
                rospy.loginfo("Audio Disabled")
                self._startAudio(self.song_exit)
        # Player control
        if self.enable:
            # Check play/stop
            if self.play_stop:
                if self.ntrack < len(self.audiolist):
                    self._startAudio(self.audiolist[self.ntrack])
                return
            # Check next song
            if self.next_song:
                # Update counter
                self.ntrack += 1
                if self.ntrack >= len(self.audiolist):
                    self.ntrack = 0
                # Start a new audio
                if self.ntrack < len(self.audiolist):
                    self._startAudio(self.audiolist[self.ntrack])
                return
            # Start speech
            if self.button_speech:
                text = self.texts[self.nMessage]
                rospy.loginfo("speech: {text}".format(text=text))
                # Counter update
                self.nMessage += 1
                if self.nMessage >= len(self.texts):
                    self.nMessage = 0
                # Send voice sound
                sound = self.sound_client.voiceSound(text)
                # sound = self.sound_client.builtinSound(SoundRequest.NEEDS_PLUGGING)
                sound.play()
# EOF
