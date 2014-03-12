########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#     The questioning state listens for a response, given a question to ask. eg. state(question)->answer. 
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

from sound_play.msg import SoundRequest
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest
from talos_speech.srv import ListenForAny
from talos_speech.srv import ListenForAll

############################################ CLASS DEF ##############################################
class QuestioningState(smach.State):

    def __init__(self, question, response=[]):
        self.question = question
        self.response = response
        smach.State.__init__(self, outcomes=["ResponseReceived", "NoResponseGiven"])
        self.counter = 0
        self.attempts = 3
        self.response = "None"
        self.waiting_for_response = False  

        # Subscribe to recognizer/output
        self.subscriber = rospy.Subscriber('recognizer/output', String, self.response_callback)

    def response_callback(self, data):
        if self.waiting_for_response and len(data.data) > 0:
            # Call the stop listening service
            rospy.loginfo("GOT A RESPONSE!!!!: %s", data.data)
            self.response = data.data
            self.waiting_for_response = False
        else:
            rospy.loginfo("Sorry, I wasn't listening")

    def get_response(self):
        self.waiting_for_response = True
        # Start
        #Code to do response here, subscribes to /output topic. 
        return self.waiting_for_response

    # Asks the user a question via the sound-play microphone node
    def execute(self, userdata):

        rospy.loginfo("Asking question: %s", self.question)
        SpeechListener.say("What is your name?")
        rospy.loginfo("Sending sound request...")
        rospy.sleep(4)   

        # lol fuk i brokez it ## HEAP OVERFLOW 
        while (self.get_response()):
            rospy.sleep(2)
            rospy.loginfo("Waiting for response...")
#
        SpeechListener.say("Is " + self.response + " what you mean? Yes or No")
 #       rospy.sleep(4)       

        # do confirmation after
           
        return "NoResponseGiven"


