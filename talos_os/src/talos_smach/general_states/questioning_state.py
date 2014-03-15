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

from listening_state import ListeningState

############################################ CLASS DEF ##############################################
class QuestioningState(smach.State):

    def __init__(self, question):
        self.CURRENT_MODE = 0
        self.QUESTION_ASK_MODE = 1
        self.CONFIRM_ANSWER_MODE = 3
        self.AWAITING_RESPONSE_MODE = 2
        self.AWAITING_CONFIRM_RESPONSE_MODE = 4

        # Default behaviour
        self.CURRENT_MODE = self.QUESTION_ASK_MODE
        smach.State.__init__(self, outcomes=["AwaitingResponse", "ResponseReceived", "ConfirmedYes", "ConfirmedNo"])
        self.question = question
        self.counter = 0
        self.got_a_word = False

        #self.question = question
        #self.counter = 0
        #self.got_a_word = False
        #self.response = "Response"
        #self.response_received = False
        #self.confirming_response = False
        #self.confirming_response_first_entry = True

    def say_service(self, words):
        try:
            say = rospy.ServiceProxy('say', ListenFor)
            void = say(words)
        except rospy.ServiceException, e:
            print "Service call failed %s" %e

    def response_callback(self, data):
        if (self.CURRENT_MODE == self.AWAITING_RESPONSE_MODE && len(data.data) > 0:
            self.response = data.data
            self.got_a_word = True

    # Restores the values because things may have been modified, and the question may have to  be asked again thus start fresh. 
    def reset_questioning_variables():
        self.got_a_word = False
        self.counter = 0
        self.response = "NoResponse"
        self.CURRENT_MODE = self.QUESTION_ASK_MODE

    def execute(self, userdata):
        if (self.CURRENT_MODE == self.QUESTION_ASK_MODE):
            rospy.logdebug("MODE: QUESTION ASK MODE)
            rospy.loginfo("UNDEFINED BEHAVIOUR BUG")
            self.say_service(self.question)
            rospy.sleep(3)
            self.subscriber = rospy.Subscriber('recognizer/output', String, self.response_callback)
            self.CURRENT_MODE = self.AWAITING_RESPONSE_MODE
            SpeechListener.start_recognizer()
            return "AwaitingResponse"
        
        if (self.CURRENT_MODE == self.AWAITING_RESPONSE_MODE):
            rospy.logdebug("MODE: AWAITING_RESPONSE_MODE")
            if (self.got_a_word):
                self.CURRENT_MODE == self.CONFIRM_ANSWER_MODE
                SpeechListener.stop_recognizer()
                self.got_a_word = False
                return "ResponseReceived"

        if (self.CURRENT_MODE == self.CONFIRM_ANSWER_MODE):
            rospy.logdebug("MODE: CONFIRM_ANSWER_MODE")
            rospy.loginfo("Did you say %s, please say yes or no", self.response)
            rospy.say_service("Did you say " + self.response + " please say yes or no")
            rospy.sleep(3)
            self.CURRENT_MODE == self.AWAITING_CONFIRM_RESPONSE
            SpeechListener.start_recognizer()
            return "AwaitingResponse"

        if (self.CURRENT_MODE == self.AWAITING_CONFIRM_RESPONSE):
            rospy.logdebug("MODE: AWAITING CONFIRM RESPONSE MODE")
            if (self.got_a_word):
                if (self.response == "yes"):
                    return "ConfirmedYes"
                if (self.response == "no"):
                    self.reset_questioning_variables()
                    return "ConfirmedNo"

        #if self.first_entry:
        #    rospy.loginfo("BEFORE")         
        #    self.say_service(self.question) 
        #    #rospy.loginfo("AFTER")
        #    rospy.sleep(3)       
        #    self.subscriber = rospy.Subscriber('recognizer/output',String, self.response_callback)
        #    self.first_entry = False
        #    # Elements for response
        #    self.waiting_for_response = True
        #    SpeechListener.start_recognizer()
        #    return "QuestionAsked"
        #elif (self.got_a_word and self.waiting_for_response):
        #    SpeechListener.stop_recognizer()
        #    rospy.logwarn("Response Received: %s", self.response)
        #    rospy.logwarn("lolok")
        #    self.response_received = True
        #    return "ResponseReceived"
        #elif (self.response_received):
        #    rospy.loginfo("Confirming response")
        #    SpeechListener.say_service("Please confirm: Did you say %s", self.response)
        #    SpeechListener.start_recognizer()
        #    self.confirming_response = True
        #    return "ConfirmingResponse"
        #elif (self.confirming_response and self.confirming_response_first_entry):
        #    self.confirming_response_first_entry = True
        #    listening_state = ListeningState(["yes", "no"])
        #    while (listening_state.execute() == "NoCommandDetected"):
        #        rospy.logdebug("Waiting for confirmation")

        #    if (listening_state.word_heard == "yes"):
        #        rospy.logdebug("Confirmed YES")
        #        return "ConfirmedYes"
        #    elif (self.confirmation_outcome =="no"):
        #        rospy.logdebug("Confirmed NO")
        #        return "ConfirmedNo"

        #    SpeechListener.stop_recognizer()

        return "AwaitingResponse"


           


