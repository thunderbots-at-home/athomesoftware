## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

from std_msgs.msg import String

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
    
    ok_talos_detected = False    
    

    def __init__(self):
        smach.State.__init__(self, outcomes=["NoCommandDetected", "CommandDetected"])
        self.counter = 0
        rospy.loginfo("Robot is starting up")
        ## Initialize a ROS node and subscribe to a topic
        # Subscribes to the output topic produced by recognizer.py (Pocketsphinx)
        rospy.Subscriber("output", String, self.speech_callback)

    def speech_callback(data):
        rospy.loginfo(rospy.get_name() + " I heard %s", data.data)
        if (data.data == "Ok Talos"):
            ok_talos_detected = True
        else:
            ok_talos_detected = False

################################ ON STATE EXECUTION #######################
# On state execution, the robot should listen for the "Ok, Talos" command
# given by the speech to text node. If this has occured, then the next state
# will happen.
    def execute(self, userdata):
        rospy.loginfo("Standing by for Ok, Talos")

        if (ok_talos_detected):
            return "CommandDetected"
        else:
            return "NoCommandDetected"
