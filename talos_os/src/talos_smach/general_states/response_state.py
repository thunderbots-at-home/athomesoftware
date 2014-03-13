########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#     Response state, awaits for a response before going further and saves the result in instance.response
#
########################################################################
# Imports

import roslib; roslib.load_manifest('sound_play'); roslib.load_manifest('talos_speech')
# It doesn't work any other way, please explain someone?
import talos_speech_listener
from talos_speech_listener.speech_listener import SpeechListener

import rospy
import smach
import smach_ros

from std_msgs.msg import String

############################### CLASS DEF ##########################
class ResponseState(smach.State):

    def __init__(self):
        self.counter = 0
        self.response = "NoResponse"
        smach.State.__init__(self, outcomes=["AwaitingResponse", "ResponseReceived"])
        self.waiting_for_response = True
        self.first_entry = True
        self.got_a_word = False
        

    def response_callback(self, data):
        if not self.first_entry and len(data.data) > 0:
            rospy.loginfo("GOT A RESPONSE!!!!: %s", data.data)
            self.response = data.data
            self.waiting_for_response = False
            SpeechListener.stop_recognizer()
            self.got_a_word = True
            rospy.loginfo("LOL GOT SOMETHING WOTO")
            rospy.sleep(10)

    # Should start the recognizer service and get the first callback.
    def execute(self, userdata):

        if self.first_entry:
            self.subscriber = rospy.Subscriber('recognizer/output',String, self.response_callback)
            self.waiting_for_response = True
            self.first_entry = False
            SpeechListener.start_recognizer()

        if (self.got_a_word):
            #self.first_entry = True
            return "ResponseReceived"
        
        return "AwaitingResponse"
        
        
