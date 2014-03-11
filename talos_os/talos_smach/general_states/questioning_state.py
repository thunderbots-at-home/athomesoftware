########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#     The questioning state listens for a response, given a question to ask. eg. state(question)->answer. 
#
########################################################################
# Imports

import roslib; roslib.load_manifest('sound_play')
import rospy
import smach
import smach_ros

from sound_play.msg import SoundRequest
from std_msgs.msg import String
from std_srvs.srv import Empty
from talos_speech.srv import ListenForAny
from talos_speech.srv import ListenForAll

############################################ CLASS DEF ##############################################
class QuestioningState(smach.State):

    def __init__(self, question, response=[]):
        self.question = question
        self.response = response
        smach.State.__init__(self, outcomes=["ResponseReceived", "NoResponseGiven"])
        self.counter = 0
        self.attempts = 3


    # Asks the user a question via the sound-play microphone node
    def execute(self, userdata):

        publisher = rospy.Publisher('robotsound', SoundRequest)
        sound_req = SoundRequest()

        # Publish a sound_play/SoundRequest
        # 3 attempts for asking a question
        for attempt in range(self.attempts):
            #
            sound_req.arg = self.question
            sound_req.command = 1
 
            rospy.loginfo("Asking question: %s", self.question)
            publisher.publish(sound_req)
            # Listen for the next thing said by calling the listen_for_any service   
            try:
                listen_for_any = rospy.ServiceProxy('listen_for_any', ListenForAny)
                response = listen_for_any()

                # Do a confirmation check

                sound_req.arg = "Is " + response.result + " what you mean? Yes or No"
                sound_req.command = 1
                publisher.publish(sound_req)
                
                listen_for_any_two = rospy.ServiceProxy('listen_for_any', ListenForAny)
                response_two = listen_for_any()

                if (response_two.result == "yes"):
                    ## Return that result
                    self.response.append(response_two.result)
                    return "ResponseReceived"

            except rospy.ServiceException, e:
                print "Service call failed %s" %e

        return "NoResponseGiven"


