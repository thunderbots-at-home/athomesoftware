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
        smach.State.__init__(self, outcomes=["QuestionAsked", "QuestionFailed"])
        self.counter = 0

    def say_service(self, words):
        try:
            say = rospy.ServiceProxy('say', ListenFor)
            void = say(words)
        except rospy.ServiceException, e:
            print "Service call failed %s" %e

    def execute(self, userdata):

# this state needs to stay active while speech listener is executing. wtf. im going to have to make it a service used in another process
        self.say_service(self.question)        
           
        return "QuestionAsked"


