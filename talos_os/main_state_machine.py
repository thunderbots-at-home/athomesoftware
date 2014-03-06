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
        # Adding the states to the container
        #smach.StateMachine.add('Add a State



########################### Start Main #############################

if __name__ == '__main__':
    main()
