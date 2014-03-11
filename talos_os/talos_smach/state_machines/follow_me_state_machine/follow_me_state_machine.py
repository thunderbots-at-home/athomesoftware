########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#     The follow me algorithm used in the RoboCup@Home competition for
#     Thunderbots@Home team coded as a state machine
#
########################################################################
# Imports

import roslib
import rospy
import smach
import smach_ros

# Follower state imports
from talos_smach.state_machines.follow_me_state_machine.occluded_state import OccludedState
from talos_smach.state_machines.follow_me_state_machine.positioning_for_user_state import PositioningForUserState
from talos_smach.state_machines.follow_me_state_machine.remembering_user_state import RememberingUserState
from talos_smach.state_machines.follow_me_state_machine.user_off_screen_state import UserOffScreenState
from talos_smach.state_machines.follow_me_state_machine.no_user_detected_state import NoUserDetectedState
from talos_smach.state_machines.follow_me_state_machine.following_state import FollowingState
from talos_smach.state_machines.follow_me_state_machine.tracking_unidentified_state import TrackingUnidentifiedUserState
from talos_smach.state_machines.follow_me_state_machine.tracking_wrong_user_state import TrackingWrongUserState

############################### CLASS DEF ##############################
class FollowMeStateMachine(smach.StateMachine):

    ## When done, it should return the state "InitialStandbyState"
    def __init__(self):
        outcomes = "InitialStandbyState"
        super(FollowMeStateMachine, self).__init__(outcomes)
        
        # REMEMBER ME STATE	
        smach.StateMachine.add('RememberingUserState', RememberingUserState(),transitions={"FailedToRemember":'FailedStatePrompt',    "UserRemembered":'TrackingUnidentifiedUserState',"UnsuccessfulAttempt":'RememberingUserState'})

        # TRACKING UNIDENTIFIED USER STATE
        smach.StateMachine.add('TrackingUnidentifiedUserState', TrackingUnidentifiedUserState(), transitions={'TrackingFailed':'FailedStatePrompt', 'IdentifiedUser':'FollowerCommandStandbyState', 'ContinueTracking':'TrackingUnidentifiedUserState'})

            # FOLLOWING STATE
        smach.StateMachine.add("FollowingState", FollowingState(), transitions={"ContinueFollowing":"FollowingState", "FollowingFailed":"FailedStatePrompt", "NoUserDetected":"NoUserDetectedState"})

            # NO USER DETECTED STATE
        super.StateMachine.add("NoUserDetectedState", NoUserDetectedState(), transitions={"UserDetected":"FollowingState", "UserOffScreen":"UserOffScreenState", "UserOccluded":"OccludedState", "TrackingWrongUser":"TrackingWrongUserState", "FailedToFindUser":"FailedStatePrompt"})

            # FAILED TRACKING STATE
        super.StateMachine.add("FailedTrackingState", FailedTrackingState(), transitions={"FailedTracking":"FailedStatePrompt"})

            # TRACKING WRONG USER STATE
        super.StateMachine.add("TrackingWrongUserState", TrackingWrongUserState(), transitions={"TrackingCorrected":"FollowingState", "TrackingUncorrected":"TrackingWrongUserState", "TrackingFailed":"FailedTrackingState", "AttemptingToRetrack":"TrackingWrongUserState"})

            # OCCLUDED STATE
        super.StateMachine.add("OccludedState", OccludedState(), transitions={"WaitingForUser":"OccludedState", "PositioningForUser":"PositioningForUserState", "FailedToFindUser":"FailedTrackingState"})

            # POSITIONING FOR USER STATE
        super.StateMachine.add("PositioningForUserState", PositioningForUserState(), transitions={"UserDetected":"FollowingState", "PositioningForUser":"PositioningForUserState", "PositioningForUserFailed":"FailedTrackingState"})

            # USER OFF SCREEN STATE
        super.StateMachine.add("UserOffScreenState", UserOffScreenState(), transitions={"UserDetected":"FollowingState", "UserOffScreenStill":"UserOffScreenState", "FailedToFindUser":"FailedTrackingState"})


