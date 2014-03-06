#!/usr/bin/env python
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
########################### Import Libraries #######################

import roslib
import rospy
import smach
import smach_ros

############################ Import states #########################

from states.startup_state import StartupState
from states.command_standby_state import CommandStandbyState
from states.follower_command_standby_state import FollowerCommandStandbyState
from states.remembering_user_state import RememberingUserState
from states.tracking_unidentified_state import TrackingUnidentifiedUserState

################################ MAIN ##############################

def main():
    rospy.init_node('talos_main_state_machine')
    
    #Create a Smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # Open the container
    with sm:
        print 'nothing here'

        # Transitions = {Outcome:NextState}
        #  STARTUP STATE
        smach.StateMachine.add('StartupState', StartupState(),
                                transitions={"NoCommandDetected":'StartupState', "CommandDetected":'CommandStandbyState'})

        # COMMAND STANDBY STATE
        smach.StateMachine.add('CommandStandbyState', CommandStandbyState(),
                                transitions={"RememberMeCommandDetected":'RememberingUserState', "NoCommandDetected":'CommandStandbyState'})

        # REMEMBER ME STATE
        smach.StateMachine.add('RememberingUserState', RememberingUserState(),
                                transitions=
 {"FailedToRemember":'FailedStatePrompt',    "UserRemembered":'TrackingUnidentifiedUserState',
 "UnsuccessfulAttempt":'RememberingUserState'})

        # FAILED STATE PROMPT
        smach.StateMachine.add('FailedStatePrompt', FailedStatePrompt(),
                                transitions={"FailedStatePrompt":'FailedStatePrompt'})

        # TRACKING UNIDENTIFIED USER STATE
        smach.StateMachine.add('TrackingUnidentifiedUserState', TrackingUnidentifiedUserState(), transitions={'TrackingFailed':'FailedStatePrompt', 'IdentifiedUser':'FollowerCommandStandbyState', 'ContinueTracking':'TrackingUnidentifiedUserState'}
        
        # FOLLOWER COMMAND STANDBY STATE
        smach.Statemachine.add('FollowerCommandStandbyState', FollowerCommandStandbyState(), transitions={"FollowMeCommandDetected":"FollowingState", "RestartCommandDetected":"FailedStatePrompt", "ContinueStandby":"FollowerCommandStandbyState"})

        # FOLLOWING STATE
        smach.StateMachine.add("FollowingState", FollowingState(), transitions={

        # FAILED TRACKING STATE

        # NO USER DETECTED STATE

        # TRACKING WRONG USER STATE

        # OCCLUDED STATE

        # DONE NOOBS

########################### Start Main #############################

if __name__ == '__main__':
    main()
