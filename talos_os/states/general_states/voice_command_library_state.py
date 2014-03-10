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
from talos_speech.srv import ListenForAll

############################### CLASS DEF ##############################
class VoiceCommandLibraryState(smach.State):

    def __init__(self, utterances):
        self.utterances = utterances
        smach.State.__init__(self, outcomes=["WaitingForCommand", "CommandComplete"])
        self.counter = 0
        self.state_machine = None

        #TODO Add the utterances and their states to the overall state 
        # machine with this one as the parent. 
        # Each one of the utterances values can 

        rospy.loginfo("##############################################")
        rospy.loginfo("#VoiceCommandLibraryState: Commands available#")
        rospy.loginfo("##############################################")
        
        for key in self.utterances:
            rospy.loginfo("# VoiceCommandLibraryState: %s", key)

        rospy.loginfo("##############################################")

    # Calls the service listen_for_all on the utterance keys
    # depending on which one is a match, then call that state machine
    # and when that state machine is done executing then return
    # back to main
    def execute(self, userdata):

        
        try:

            rospy.loginfo("Checking for utterances...")
            listen_for_all = rospy.ServiceProxy('listen_for_all', ListenForAll)

            response = listen_for_all(self.utterances.keys())

            
            # get the corresponding state machine
            if response is not None:
                self.state_machine = self.utterances[response]


        print "NotImplemented"

        
