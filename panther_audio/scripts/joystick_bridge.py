#!/usr/bin/env python
import rospy
import os
from sensor_msgs.msg import Joy
from sound_play.libsoundplay import SoundClient

# Button class definition
class Button:
    def __init__(self, num):
        self.num = num
        self.state = False
        self.old_state = False
    
    def update(self, buttons):
        self.state = buttons[self.num - 1]
        if self.state and not self.old_state:
            self.old_state = self.state
            return True
        self.old_state = self.state
        return False

# Audio controller with play/stop and next song
class AudioController:
    def __init__(self, rospy, play_stop, next, sound_client, global_path, audiolist):
        self.rospy = rospy
        self.play_stop = Button(play_stop)
        self.next = Button(next)
        self.sound_client = sound_client
        self.global_path = global_path
        self.audiolist = audiolist
        self.selected = 0
        
    def startAudio(self):
        self.rospy.loginfo("Play %s"%self.audiolist[self.selected])
        self.sound_client.playWave(self.global_path + "/" + self.audiolist[self.selected])
        
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

# Definition type of effect
audio_controller = None
# Definition start stop LED effect
led_controller = None
status = True

def callback(data):
    global audio_controller
    global led_controller
    global status
    # Update audio controller
    audio_controller.update(data.buttons)
    # Check if start and stop led effect button is pressed
    if led_controller.update(data.buttons):
        param = rospy.get_param("~led_controller/param")
        rospy.set_param(param, status)
        if status:
            rospy.loginfo("LED ON")
        else:
            rospy.loginfo("LED OFF")
        status = not status
    
def joystick_bridge():
    # Initialzie ROS python node
    rospy.init_node('joystick_bridge', anonymous=True)
    # Read Music global path
    global_path = rospy.get_param("~audio_path", os.environ['HOME'] + "/Music")
    # Lists all files in the current directory
    # Selected only the wav files
    audio_files = [item for item in os.listdir(global_path) if item.endswith('.wav')]
    rospy.loginfo("Audio loaded from %s:"%global_path)
    counter = 1
    for item in audio_files:
        rospy.loginfo(" %d. %s"%(counter, item))
        counter += 1
    rospy.loginfo("Wait %s load audio controller..."%rospy.get_name())
    # Initialize the sound player cliend
    sound_client = SoundClient()
    rospy.sleep(2)
    # Launch Joystick reader
    rospy.Subscriber("joy", Joy, callback)
    # Initialize audio controller
    global audio_controller
    button_start = rospy.get_param("~audio_controller/start")
    button_next = rospy.get_param("~audio_controller/next")
    audio_controller = AudioController(rospy, button_start, button_next, sound_client, global_path, audio_files)
    rospy.loginfo("* Audio -> Start/Stop[%d] - Next[%d]"%(button_start, button_next))
    # Inizialize led effect controller
    global led_controller
    button_led = rospy.get_param("~led_controller/button")
    led_controller = Button(button_led)
    rospy.loginfo("* LED -> ON/OFF[%d] - Param:%s"%(button_led, rospy.get_param("~led_controller/param")))
    # Print start node
    rospy.loginfo("... %s running!"%rospy.get_name())
    # Start Hello robot
    sound_client.playWave(global_path + "/" + 'controller/R2D2-init.wav')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # Print exit message
    rospy.loginfo("%s Stopped!"%rospy.get_name())

if __name__ == '__main__':
    joystick_bridge()
