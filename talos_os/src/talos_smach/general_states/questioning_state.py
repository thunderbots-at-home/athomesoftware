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


from threading import Thread
import rospy
import smach
import smach_ros

from sound_play.msg import SoundRequest
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest
from talos_speech.srv import ListenFor
from talos_speech.srv import ListenForAll

############################################ CLASS DEF ##############################################
class QuestioningState(smach.State):

    def __init__(self, question, response=[]):
        self.question = question
        smach.State.__init__(self, outcomes=["QuestionAsked", "AwaitingResponse", "ResponseReceived"])
        self.counter = 0
        self.waiting_for_response = True
        self.first_entry = True
        self.got_a_word = False
        self.response = "Response"

    def say_service(self, words):
        try:
            say = rospy.ServiceProxy('say', ListenFor)
            void = say(words)
        except rospy.ServiceException, e:
            print "Service call failed %s" %e

    def response_callback(self, data):
        if not self.first_entry and len(data.data) > 0:
            self.response = data.data
            self.got_a_word = True

    def execute(self, userdata):
        if self.first_entry:
            rospy.loginfo("BEFORE")         
            self.say_service(self.question) 
            #rospy.loginfo("AFTER")
            rospy.sleep(3)       
            self.subscriber = rospy.Subscriber('recognizer/output',String, self.response_callback)
            self.waiting_for_response = True
            self.first_entry = False
            SpeechListener.start_recognizer()
            return "QuestionAsked"
        if (self.got_a_word):
            self.first_entry = True
            self.waiting_for_response = False
            SpeechListener.stop_recognizer()
            rospy.logdebug("Response Received: %s", self.response)
            return "ResponseReceived"
        
        return "AwaitingResponse"
           


