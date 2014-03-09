## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

from std_msgs.msg import String
from talos_speech.srv import ListenFor

################################ DEVELOPER README #########################
# On system startup, what should the robot be doing?
# The robot should bring up all of the necessary components (Eg. its interfaces [speech, visual, etc], and its funcionality, and its sensors.)
# This can be achieved by roslaunching the thunderbot_bringup.
###########################################################################
class StartupState(smach.State):

############################### ON STATE INIT #############################
# The state machine assumes that these nodes below are initializes by 
# running 'roslaunch talos_navigation talos_bringup.launch'

# Nodes:
#    JointStatePublisher
#    RobotStatePublisher
#    Rviz
#    hector_mapping
#    StaticTransformPublisher
#    move_base (from nav_stack)

# Sensors:
#    Kinect
#    Hokuyo Laser
#    Microphone
#    Speakers
    
    def __init__(self):
        smach.State.__init__(self, outcomes=["NoCommandDetected", "CommandDetected"])
        self.counter = 0
        rospy.loginfo("Robot is starting up")

################################ ON STATE EXECUTION #######################
# Info:
#
# On state execution, the robot should listen for the "Ok, Talos" command
# given by the speech to text node. If this has occured, then the next state
# will happen.
#
# 1. The execute command will keep being called until leaving the state
# 2. The listen_for service call is called every time
# 3. the listen_for returns true when it has heard the utterance
# 4. When the call returns true, response will be 1, and the state
# 5. will stop.  
#
    def execute(self, userdata):
        #TODO Is it possible to change the rate of execution calls? Hmm
        rospy.wait_for_service('listen_for')
        response = ListenFor()
        # Note to self: services must be called with the nodespace node/servicename
        # Calls the speech listener, and tells it to start listening for the ok command
        try:
            rospy.loginfo("Checking if I've heard: OK")
            listen_for = rospy.ServiceProxy('listen_for', ListenFor)
            response = listen_for("ok")

            if (response.result == 1):
                return "CommandDetected"
            else:
                return "NoCommandDetected"
        except rospy.ServiceException, e:
                print "Service call failed: %s" %e

