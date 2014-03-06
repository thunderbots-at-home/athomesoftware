## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros


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
#    TODO: Add microphone/speakers launch

# Controllers:
#    PS3 controller


    def __init__(self):
        smach.State.__init__(self, outcomes=["NoCommandDetected", "CommandDetected"])
        self.counter = 0

################################ ON STATE EXECUTION #######################
# On state execution, the robot should listen for the "Ok, Talos" command
# given by the speech to text node. If this has occured, then the next state
# will happen.

    def execute(self, userdata):
        rospy.loginfo("Standing by for Ok, Talos")

        ok_talos_detected = 0

        if (ok_talos_detected):
            return "CommandDetected"
        else:
            return "NoCommandDetected"


