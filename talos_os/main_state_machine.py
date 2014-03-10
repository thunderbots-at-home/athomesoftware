#!/usr/bin/env python
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
########################### Import Libraries #######################

import roslib
import rospy
import smach
import smach_ros


############################ Import states #########################
# DEPRECATED
#from talos_smach.command_standby_state import CommandStandbyState
#from talos_smach.failed_state_prompt import FailedStatePrompt
#from talos_smach.follower_command_standby_state import FollowerCommandStandbyState
#from talos_smach.remembering_user_state import RememberingUserState
#from talos_smach.startup_state import StartupState
#from talos_smach.tracking_unidentified_state import TrackingUnidentifiedUserState

# DEPRECATED
#Follower algorithm includes
#from talos_smach.follow_states.failed_tracking_state import FailedTrackingState
#from talos_smach.follow_states.occluded_state import OccludedState
#from talos_smach.follow_states.tracking_wrong_user_state import TrackingWrongUserState
#from talos_smach.follow_states.user_off_screen_state import UserOffScreenState
#from talos_smach.follow_states.no_user_detected_state import NoUserDetectedState
#from talos_smach.follow_states.following_state import FollowingState
#from talos_smach.follow_states.positioning_for_user_state import PositioningForUserState

# General state includes
from talos_smach.general_states.listening_state import ListeningState
from talos_smach.general_states.voice_command_library_state import VoiceCommandLibraryState
from talos_smach.state_machines.follow_me_state_machine.follow_me_state_machine import FollowingStateMachine
################################ MAIN ##############################


class MainStateMachine:
    
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])

    # Adds the initial startup states to talos
    def add_startup_states(self):

        with self.sm:

            # STARTUP STATE
            smach.StateMachine.add('InitialStandbyState', ListeningState("ok"),transitions={"NoCommandDetected":'InitialStandbyState', "CommandDetected":'CommandStandbyState'})

            # This is where to add the state machines to the voice library.
            utterances = {}
            utterances["follow"] = FollowMestateMachine()

            # VOICE COMMAND LIBRARY STATE
            # Adding the voice command library state
            voice_command_lib = VoiceCommandLibraryState(utterances)
            # Do not do this directly. Instead, add them to an utterances list.
            #follow_me_sm = FollowMeStateMachine()
            #voice_command_lib.add_command("FollowMeStateMachine", follow_me_sm)
            smach.StateMachine.add('VoiceCommandLibraryState', voice_command_lib, voice_command_lib.default_transitions)
 
            # DEPRECATED
            # Replaced by VoiceCommandLibraryState
            # COMMAND STANDBY STATE
            #smach.StateMachine.add('CommandStandbyState', CommandStandbyState(),transitions={"RememberMeCommandDetected":'RememberingUserState', "NoCommandDetected":'CommandStandbyState'})

            # DEPRECATED
            # FAILED STATE PROMPT
            # smach.StateMachine.add('FailedStatePrompt', FailedStatePrompt(),transitions={"FailedStatePrompt":'FailedStatePrompt'})


    # DEPRECATED
    # Adds the following states to the general robot bringup SM
    #def add_follow_me_states(self):

    #    with self.sm:


            # DEPRECATED
            # Add the follower state machine as a voice command for the voice command lib.
            # REMEMBER ME STATE
        #    smach.StateMachine.add('RememberingUserState', RememberingUserState(),transitions=
 #{"FailedToRemember":'FailedStatePrompt',    "UserRemembered":'TrackingUnidentifiedUserState',
 #"UnsuccessfulAttempt":'RememberingUserState'})

            # TRACKING UNIDENTIFIED USER STATE
  #          smach.StateMachine.add('TrackingUnidentifiedUserState', TrackingUnidentifiedUserState(), transitions={'TrackingFailed':'FailedStatePrompt', 'IdentifiedUser':'FollowerCommandStandbyState', 'ContinueTracking':'TrackingUnidentifiedUserState'})

            # FOLLOWER COMMAND STANDBY STATE
  #          smach.StateMachine.add('FollowerCommandStandbyState', FollowerCommandStandbyState(), transitions={'FollowMeCommandDetected':'FollowingState','RestartCommandDetected':'FailedStatePrompt', 'ContinueStandby':'FollowerCommandStandbyState'})

            # FOLLOWING STATE
   #         smach.StateMachine.add("FollowingState", FollowingState(), transitions={"ContinueFollowing":"FollowingState", "FollowingFailed":"FailedStatePrompt", "NoUserDetected":"NoUserDetectedState"})

            # NO USER DETECTED STATE
    #        smach.StateMachine.add("NoUserDetectedState", NoUserDetectedState(), transitions={"UserDetected":"FollowingState", "UserOffScreen":"UserOffScreenState", "UserOccluded":"OccludedState", "TrackingWrongUser":"TrackingWrongUserState", "FailedToFindUser":"FailedStatePrompt"})

            # FAILED TRACKING STATE
     #       smach.StateMachine.add("FailedTrackingState", FailedTrackingState(), transitions={"FailedTracking":"FailedStatePrompt"})

            # TRACKING WRONG USER STATE
      #      smach.StateMachine.add("TrackingWrongUserState", TrackingWrongUserState(), transitions={"TrackingCorrected":"FollowingState", "TrackingUncorrected":"TrackingWrongUserState", "TrackingFailed":"FailedTrackingState", "AttemptingToRetrack":"TrackingWrongUserState"})

            # OCCLUDED STATE
       #     smach.StateMachine.add("OccludedState", OccludedState(), transitions={"WaitingForUser":"OccludedState", "PositioningForUser":"PositioningForUserState", "FailedToFindUser":"FailedTrackingState"})

            # POSITIONING FOR USER STATE
        #    smach.StateMachine.add("PositioningForUserState", PositioningForUserState(), transitions={"UserDetected":"FollowingState", "PositioningForUser":"PositioningForUserState", "PositioningForUserFailed":"FailedTrackingState"})

            # USER OFF SCREEN STATE
         #   smach.StateMachine.add("UserOffScreenState", UserOffScreenState(), transitions={"UserDetected":"FollowingState", "UserOffScreenStill":"UserOffScreenState", "FailedToFindUser":"FailedTrackingState"})


def main():
    rospy.init_node('talos_main_state_machine')
    
    #Create a Smach
    sm = MainStateMachine()
    sm.add_startup_states()
    
    #sm.add_follow_me_states()
    outcome = sm.sm.execute()

########################### Start Main #############################

if __name__ == '__main__':
    main()
