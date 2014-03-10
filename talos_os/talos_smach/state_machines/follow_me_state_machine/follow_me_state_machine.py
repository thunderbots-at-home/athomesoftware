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

############################### CLASS DEF ##############################
class FollowMeStateMachine(smach.StateMachine):

    ## When done, it should return the state "InitialStandbyState"
    def __init__(self):
        outcomes = "InitialStandbyState"
        super().__init__(outcomes)
        
        # REMEMBER ME STATE
        super.StateMachine.add('RememberingUserState', RememberingUserState(),transitions=
 {"FailedToRemember":'FailedStatePrompt',    "UserRemembered":'TrackingUnidentifiedUserState',
 "UnsuccessfulAttempt":'RememberingUserState'})

        # TRACKING UNIDENTIFIED USER STATE
        super.StateMachine.add('TrackingUnidentifiedUserState', TrackingUnidentifiedUserState(), transitions={'TrackingFailed':'FailedStatePrompt', 'IdentifiedUser':'FollowerCommandStandbyState', 'ContinueTracking':'TrackingUnidentifiedUserState'})

        # FOLLOWER COMMAND STANDBY STATE
        super.StateMachine.add('FollowerCommandStandbyState', FollowerCommandStandbyState(), transitions={'FollowMeCommandDetected':'FollowingState','RestartCommandDetected':'FailedStatePrompt', 'ContinueStandby':'FollowerCommandStandbyState'})

            # FOLLOWING STATE
        super.StateMachine.add("FollowingState", FollowingState(), transitions={"ContinueFollowing":"FollowingState", "FollowingFailed":"FailedStatePrompt", "NoUserDetected":"NoUserDetectedState"})

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


