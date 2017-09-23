#!/usr/bin/env python
import rospy
import os
from sensor_msgs.msg import Joy
from sound_play.libsoundplay import SoundClient

global_path = '/home/develop/Music'
audio_files = []
# Button class definition
class Button:
    def __init__(self, num):
        self.num = num
        self.old_state = False
    
    def update(self, buttons):
        if buttons[self.num - 1] and not self.old_state:
            self.old_state = buttons[self.num - 1]
            return True
        self.old_state = buttons[self.num - 1]
        return False
# Audio controller with play/stop and next song
class AudioController:
    def __init__(self, rospy, play_stop, next, sound_client, audiolist):
        self.rospy = rospy
        self.play_stop = Button(play_stop)
        self.next = Button(next)
        self.sound_client = sound_client
        self.audiolist = audiolist
        self.selected = 0
        
    def startAudio(self):
        global global_path
        self.rospy.loginfo("Play %s"%self.audiolist[self.selected])
        self.sound_client.playWave(global_path + "/" + self.audiolist[self.selected])
        
    def update(self, buttons):
        if self.play_stop.update(buttons):
            self.startAudio()
        if self.next.update(buttons):
            # Increase counter
            self.selected += 1
            if self.selected >= len(self.audiolist):
                self.selected = 0
            # Start new audio
            self.startAudio()

# Definition sound_client controller
sound_client = None
# Definition type of effect
audio_controller = None

def callback(data):
    #rospy.loginfo(data.buttons)
    global audio_controller
    # Update audio controller
    audio_controller.update(data.buttons)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def joystick_bridge():
    global global_path
    # Initialzie ROS python node
    rospy.init_node('joystick_bridge', anonymous=True)
    
    # Lists all files in the current directory
    # Selected only the wav files
    global audio_files
    audio_files = [item for item in os.listdir(global_path) if item.endswith('.wav')]
    rospy.loginfo(audio_files)
    
    # Initialize the sound player cliend
    global sound_client
    sound_client = SoundClient()
    rospy.sleep(2)
    # Initialize button controller
    # Launch Joystick reader
    rospy.Subscriber("joy", Joy, callback)
    # Print start node
    rospy.loginfo("Node started")
    # Initialize audio controller
    global audio_controller
    audio_controller = AudioController(rospy, 5, 6, sound_client, audio_files)
    
    sound_client.playWave(global_path + "/" + 'controller/R2D2-init.wav')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joystick_bridge()
