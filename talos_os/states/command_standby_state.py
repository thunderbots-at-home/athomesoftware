## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class CommandStandbyState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["RememberMeCommandDetected", "NoCommandDetected"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Standing by for voice command")

        # TODO listen for speech to text commands that tell Talos "Ok, Talos" has been detected and set this variable true.

        # This can be done by implementing a subscriber under the main's node. eg. GlobalStateMachineNode.subscriber.subscribeTo(text)        
        remember_me_detected = 0

        if (remember_me_detected):
            return "RememberMeCommandDetected"
        else:
            return "NoCommandDetected"


