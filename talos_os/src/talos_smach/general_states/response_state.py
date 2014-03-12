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
class AwaitingResponseState(smach.State):

    def __init__(self):
        self.counter = 0
        self.response = "NoResponse"
        smach.State.__init__(self, outcomes=["AwaitingResponse", "ResponseReceived"]
        self.started_listening
        self.subscriber = rospy.Subscriber('recognizer/output', self.response_callback, String)
        self.waiting_for_response = False

    def response_callback(self, data):
        rospy.loginfo("Receiving things.. hmm")
        if self.waiting_for_response and len(data.data) > 0:
            # Call the stop listening service
            rospy.loginfo("GOT A RESPONSE!!!!: %s", data.data)
            self.response = data.data
            self.waiting_for_response = False
            SpeechListener.stop_recognizer()
        else:
            rospy.loginfo("Sorry, I wasn't listening")   

    # Should start the recognizer service and get the first callback.
    def execute(self, userdata):

        if (self.response != "NoResponse"):
            return "ResponseReceived"

        if not self.waiting_for_response:
            self.waiting_for_response = True 
            SpeechListener.start_recognizer()
        
        return "AwaitingResponse"
        
        
