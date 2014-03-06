#!/usr/bin/env python
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

# Include the initialization state
from startup_state import StartupState

# The states are included as imports, this is the main state machine container. 

def main():
    rospy.init_node('talos_main_state_machine')
    
    #Create a Smach
    sm = smach.StateMachine(outcomes=['robot success', 'robot failure'])

    # Open the container
    with sm:
        print 'nothing here'
        # Adding the states to the container
        #smach.StateMachine.add('Add a State

if __name__ == '__main__':
    main()
