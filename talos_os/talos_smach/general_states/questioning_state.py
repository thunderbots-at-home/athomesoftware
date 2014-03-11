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

import roslib
import rospy
import smach
import smach_ros



############################################ CLASS DEF ##############################################
class QuestioningState(smach.State):

    def __init__(self, question):
        
