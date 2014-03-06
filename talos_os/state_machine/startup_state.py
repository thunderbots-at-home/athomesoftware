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
        rospy.loginfo("Executing StartUp State")
        return 'NoCommandDetected'


