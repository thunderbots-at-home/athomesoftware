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

    def __init__(self):
        outcomes = "InitialStandbyState"
        super().__init__(outcomes)
        
        # Add all of the sub states corresponding to following
        
