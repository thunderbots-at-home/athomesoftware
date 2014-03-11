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

import roslib; roslib.load_manifest('sound_play')
import rospy
import smach
import smach_ros

from sound_play.msg import SoundRequest



############################################ CLASS DEF ##############################################
class QuestioningState(smach.State):

    def __init__(self, question):
        self.question = question
        self.response = "NoResponseGiven"
        smach.State.__init__(self, outcomes=["ResponseReceived", "NoResponseGiven"])
        self.counter = 0
        self.attempts = 3


    # Asks the user a question via the sound-play microphone node
    def execute(self, userdata):

        publisher = rospy.Publisher('robotsound', SoundRequest)
        # Publish a sound_play/SoundRequest
        # 3 attempts for asking a question
        for attempt in range(self.attempts):
            #
