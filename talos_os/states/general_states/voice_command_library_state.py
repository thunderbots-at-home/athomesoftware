########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#    The voice command library takes a dictionary of
#    utterance:state_machine pairs. On hearing an utterance, 
#    the state machine is executed. When the state machine finishes
#    control returns to the initial standby state where the user must
#    access the voice command libray by first saying "OK talos" or
#    something similar.
#
########################################################################
# Imports

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from talos_speech.srv import ListenFor

############################### CLASS DEF ##############################
class VoiceCommandLibraryState(smach.State):

    def __init__(self, utterances):
        self.utterances = utterances
        smach.State.__init__(self, outcomes=["WaitingForCommand", "CommandComplete"])
        self.counter = 0

        rospy.loginfo("##############################################")
        rospy.loginfo("#VoiceCommandLibraryState: Commands available#")
        rospy.loginfo("##############################################")
        for key in self.utterances:
            rospy.loginfo("# VoiceCommandLibraryState: %s", key)

        rospy.loginfo("##############################################")

    def execute(self, userdata):
        print "NotImplemented"

        
