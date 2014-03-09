## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib; roslib.load_manifest('talos_speech')
import rospy
import smach
import smach_ros

from talos_speech.srv import ListenFor


#TODO This entire design can be refactored into a state that accepts a parameter to listen for something before proceeding. It won't be done on this iteration, but later. 
#TODO It would make for a nice robot scripting language. 
class CommandStandbyState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["RememberMeCommandDetected", "NoCommandDetected"])
        self.counter = 0
        rospy.loginfo("Standing by for voice command")
        rospy.wait_for_service('listen_for')

    def execute(self, userdata):

        response = ListenForResponse()
        # TODO listen for speech to text commands that tell Talos "Ok, Talos" has been detected and set this variable true.
        try:
            rospy.loginfo("Checking if I've heard: Rememeber")
            listen_for = rospy.ServiceProxy('listen_for', ListenFor)
            response = listen_for("remember")
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

        # This can be done by implementing a subscriber under the main's node. eg. GlobalStateMachineNode.subscriber.subscribeTo(text)        
        if (response.result == 1):
            return "RememberMeCommandDetected"
        else:
            return "NoCommandDetected"


