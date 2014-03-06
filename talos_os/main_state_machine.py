#!/usr/bin/env python
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
########################### Import Libraries #######################

import roslib
import rospy
import smach
import smach_ros

############################ Import states #########################
#Replaced by __all__ inside __init__
from states import *

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
        smach.StateMachine.add("FollowingState", FollowingState(), transitions={"ContinueFollowing":"FollowingState", "FollowingFailed":"FailedStatePrompt", "NoUserDetected":"NoUserDetectedState"})

        # NO USER DETECTED STATE
        smach.StateMachine.add("NoUserDetectedState", NoUserDetectedState(), transitions={"UserDetected":"FollowingState", "UserOffScreen":"UserOffScreenState", "UserOccluded":"OccludedState", "TrackingWrongUser":"TrackingWrongUserState", "FailedToFindUser":"FailedStatePrompt"})

        # FAILED TRACKING STATE
        smach.StateMachine.add("FailedTrackingState", FailedTrackingState(), transitions={"FailedTracking":"FailedStatePrompt"})

        # TRACKING WRONG USER STATE
        smach.StateMachine.add("TrackingWrongUserState", TrackingWrongUserState(), transitions={"TrackingCorrected":"FollowingState", "TrackingUncorrected":"TrackingWrongUserState", "TrackingFailed":"FailedTrackingState", "AttemptingToRetrack":"TrackingWrongUserState"})

        # OCCLUDED STATE
        smach.StateMachine.add("OccludedState", OccludedState(), transitions={"WaitingForUser":"OccludedState", "PositioningForUser":"PositioningForUserState", "FailedToFindUser":"FailedTrackingState"})

        # POSITIONING FOR USER STATE
        smach.StateMachine.add("PositioningForUserState", PositioningForUserState(), transitions={"UserDetected":"FollowingState", "PositioningForUser":"PositioningForUserState", "PositioningForUserFailed":"FailedTrackingState"})

        # DONE NOOBS

########################### Start Main #############################

if __name__ == '__main__':
    main()
