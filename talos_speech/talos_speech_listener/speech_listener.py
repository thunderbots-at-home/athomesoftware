## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
############################### IMPORTS ############################

import roslib
import rospy

from std_msgs.msg import String

########################### DEVELOEPR README #######################

# 1. The speech listener class works ontop of the recognizer.py from
# pocketsphinx package for ros.

# 2. Its intended use is the .listenFor()
# function which will return false until the callback from
# pocketsphinx returns the matched string. 

# 3.This is useful instead of putting a subscripter 
# in every state/class/ for the smach state machine

heard_words = False
words_listened_for = "listening_for_nothing"

def text_callback(data):

    rospy.loginfo(rospy.get_name() + "I heard %s", data.data)
    # TODO Only does direct comparison, should be a "similarity" comparison
    if (data.data == words_listened_for):
        heard_words = True

    ## Only on state change should the heard_words go to false
    ## Or else the thread could skip/stop/lock/block/jump over the true and into a pool of ggnore

def listen_for(request):
    # TODO Code for listening and setting words listened for
    

def main():

    rospy.init_node("speech_listener")
    rospy.loginfo(rospy.get_name() + "Started speech listener")
    rospy.Subscriber("output", String, text_callback)
    service = rospy.Service('listen_for', listen_for, 

if __name__ == "__main__":
    main()
