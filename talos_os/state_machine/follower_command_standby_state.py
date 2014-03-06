## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class FollowerCommandStandby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["FollowMeCommandDetected", "RestartCommandDetected", "ContinueStandby"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Waiting for Follow Orders")

        continue_standby = 1

        if (continue_standby)
            return "ContinueStandby"
        else
            return "RestartCommandDetected"


