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

        #TODO Add the utterances and their states to the overall state 
        # machine with this one as the parent. 
        # Each one of the utterances values can 

        rospy.loginfo("##############################################")
        rospy.loginfo("#VoiceCommandLibraryState: Commands available#")
        rospy.loginfo("##############################################")
        
        outcomes = []
        # The default outcomes
        outcomes.append("WaitingForCommand")
        outcomes.append("CommandTimeout")

        self.default_transitions = {}
        self.default_transitions["WaitingForCommand"] = "VoiceCommandLibraryState"
        self.default_transitions["CommandTimeout"] = "InitialStandbyState"

        self.utterances = utterances

        for name, state_machine in self.utterances.iteritems():
            # Register the commands as transitions
            self.add_command(name, state_machine)
            # lets this state know it has the other state machine as an outcome
            outcomes.append(name)
            rospy.loginfo("# VoiceCommandLibraryState: %s", name)

        rospy.loginfo("##############################################")

        smach.State.__init__(self, outcomes)
        self.counter = 0


    # TODO CHANGE THIS SO THAT IT TRANSITIONS  TO PROPER STATE
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
        except rospy.ServiceException, e:
            print "Service call failed %s" %e


    # Adds a command to the voice library
    def add_command(self, name, state_machine):

        # Set all transitions back to the InitialStandbyState
        # All paths lead to rome/home? :D
        outcomes = state_machine.get_registered_outcomes()
        transitions = {} 
        for outcome in outcomes:
            transitions[outcome] = "InitialStandbyState"

        smach.StateMachine.add(name, state_machine, transitions)
        

