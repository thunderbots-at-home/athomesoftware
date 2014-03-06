## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class StartupState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["NoCommandDetected", "CommandDetected"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Standing by for Ok, Talos"")

        # TODO listen for speech to text commands that tell Talos "Ok, Talos" has been detected and set this variable true.        
        ok_talos_detected = 0

        if (ok_talos_detected)
            return "CommandDetected"
        else
            return "NoCommandDetected"


