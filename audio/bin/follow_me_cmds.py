#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import rospy
import math

from audio.msg import FRClientGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class follow_me_cmds:
    attempts = 0
    ask_for_name = False
    person_name = "no name"
    def __init__(self):
        self.pub_kill_ = rospy.Publisher('kill', String)
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2

        #Subscribe to the microphone input. 
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        rospy.sleep(1)
        rospy.sleep(2)
        rospy.sleep(2)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            #pub_.publish(self.msg)
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        namelist = ['james', 'john', 'robert', 'michael', 'william', 'mary', 'patricia', 'linda', 'barbara', 'elizabeth']
	## Check if robot is ready
	## Check if robot remembers person
	## Tell robot to remember person
	## Issue following command
        if (msg.data.find("stop") > -1:
            self.pub_kill_.publish(String("kill"))
        if (msg.data.find("kill") > -1:
            self.pub_kill_.publish(String("kill"))
        if (msg.data.find("follow me") > -1:
            self.pub_kill_.publish(String("run"))
        rospy.sleep(1)
        if msg.data.find("are you ready") > -1:
            pass
            #self.msg.data = "REDY BRO" ##TODO Add code that confirms readiness here
        if msg.data.find("remember me") > -1:
            follow_me_cmds.ask_for_name = True
        elif follow_me_cmds.ask_for_name == True and msg.data in namelist:
            rospy.sleep(1)

            pub = rospy.Publisher('fr_order', FRClientGoal)
            # Here we would call on the face recognition node to start learning.
            # Create the FRClientGoal message.
            fr_cmd = FRClientGoal()
            fr_cmd.order_id = 2
            fr_cmd.order_argument = msg.data
            pub.publish(fr_cmd)
            person_name = msg.data
            follow_me_cmds.ask_for_name = False
        elif self.attempts < 3 and follow_me_cmds.ask_for_name == True:
            rospy.sleep(1)
            follow_me_cmds.attempts += 1
            #Take the next speech input and compare it against a list of possible names.
            #self.msg.data = "REMEMBER ME BRU" ##TODO Add code that remembers person here
        if msg.data.find("follow me") > -1:
           pass
            #self.msg.data = "Follow YOU? NAH!" ##TODO Add following node/execution here

        if msg.data.find("confirm my identity") > -1:
            #self.msg.data = "CONFIRMD" ##TODO check identity of person here
            pub = rospy.Publisher('fr_order', FRClientGoal)
            fr_cmd = FRClientGoal()
            fr_cmd.order_id = 1
            fr_cmd.order_argument = follow_me_cmds.person_name
            pub.publish(fr_cmd)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        #self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('follow_me_cmds')
    try:
        follow_me_cmds()
    except:
        pass

