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
        smach.State.__init__(self, outcomes=["QuestionAsked", "AwaitingResponse", "ResponseReceived", "ConfirmingResponse", "ConfirmedYes", "ConfirmedNo"])
        self.counter = 0
        self.waiting_for_response = True
        self.first_entry = True
        self.got_a_word = False
        self.response = "Response"
        self.response_received = False
        self.confirming_response = False
        self.confirmation_sm = smach.StateMachine
        with self.confirmation_sm:
                self.confirmation_sm = smach.StateMachine(outcomes=["CommandDetected", "NoCommandDetected"])
                transitions = {}
                transitions["CommandDetected"] = "CommandDetected"
                transitions["NoCommandDetected"] = "NoCommandDetected"
                self.confirmation_sm.add(ListeningState(["yes", "no"], transitions)
        self.confirmation_outcome = "NoneOutcome"
        self.confirming_response_first_entry = True


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
            self.first_entry = False
            # Elements for response
            self.waiting_for_response = True
            SpeechListener.start_recognizer()
            return "QuestionAsked"
        elif (self.got_a_word):
            self.first_entry = True
            self.waiting_for_response = False
            SpeechListener.stop_recognizer()
            rospy.logwarn("Response Received: %s", self.response)
            rospy.logwarn("lolok")
            self.response_received = True
            return "ResponseReceived"
        elif (self.response_received):
            # Confirm response
            rospy.loginfo("Confirming response")
            SpeechListener.say_service("Please confirm: Did you say %s", self.response)
            self.confirming_response = True
            return "ConfirmingResponse"
        elif (self.confirming_response and self.confirming_response_first_entry):
            # Do a listening for state that returns 
            self.confirming_response_first_entry = False
            self.confirmation_outcome = self.confirmation_sm.execute()

            if (self.confirmation_outcome = "CommandDetected"):
                return "ConfirmedYes"
            elif (self.confirmation_outcome ="NoCommandDetected"):
                return "ConfirmedNo"


           


