#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import Bool
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
    def __init__(self, rospy, enable, play_stop, next, global_path, audiolist):
        self.rospy = rospy
        self.enable_button = Button(enable)
        self.play_stop = Button(play_stop)
        self.next = Button(next)
        self.sound_client = SoundClient()
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

# Definition type of effect
audio_controller = None
# Definition start stop LED effect
enable_effect = None
status = True
# Publisher LED controller
pub = None

def callback(data):
    global audio_controller
    global led_controller
    global status
    # Update audio controller
    audio_controller.update(data.buttons)
    # Check if start and stop led effect button is pressed
    if enable_effect.update(data.buttons):
        pub.publish(status)
        if status:
            rospy.loginfo("Effects ENABLE")
        else:
            rospy.loginfo("Effects DISABLE")
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
    # Initialize audio controller
    global audio_controller
    button_enable = rospy.get_param("~audio/enable")
    button_start = rospy.get_param("~audio/start")
    button_next = rospy.get_param("~audio/next")
    audio_topic = rospy.get_param("~audio/joy", "joy")
    audio_controller = AudioController(rospy, button_enable, button_start, button_next, global_path, audio_files)
    rospy.loginfo("* Audio -> Enable[%d] - Start/Stop[%d] - Next[%d] - topic:%s"%(button_enable, button_start, button_next, audio_topic))
    # Inizialize led effect controller
    global enable_effect
    button_led = rospy.get_param("~led/enable")
    enable_effect = Button(button_led)
    status = rospy.get_param("~led/topic")
    rospy.loginfo("* LED -> ON/OFF[%d] - Param:%s"%(button_led, status))
    # Wait load audio controller
    rospy.sleep(2)
    # Launch Joystick reader
    rospy.Subscriber(audio_topic, Joy, callback)
    # Initialize led status controller
    global pub
    pub = rospy.Publisher(status, Bool, queue_size=10)
    # Print start node
    rospy.loginfo("... %s running!"%rospy.get_name())
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # Print exit message
    rospy.loginfo("%s Stopped!"%rospy.get_name())

if __name__ == '__main__':
    joystick_bridge()
