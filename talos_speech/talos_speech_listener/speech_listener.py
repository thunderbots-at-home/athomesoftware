## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
############################### IMPORTS ############################
#!/usr/bin/env python


import roslib
import rospy

from std_msgs.msg import String
from talos_speech.srv import ListenFor
from std_srv.srv import Empty

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
listening = False

def text_callback(data):

    rospy.loginfo(rospy.get_name() + "I heard %s", data.data)
    # TODO Only does direct comparison, should be a "similarity" comparison
    if (data.data == words_listened_for):
        heard_words = True

    ## Only on state change should the heard_words go to false
    ## Or else the thread could skip/stop/lock/block/jump over the true and into a pool of ggnore
def listen_for(request):
    # TODO Code for listening and setting words listened for
    if not listening:
        listening = True
        words_listened_for = request.words
        rospy.loginfo("Listening for the phrase: %s", heard_words)
        # Start the recognizer
        try:
            start = rospy.ServiceProxy('start', Empty)
            response = start()
            rospy.loginfo("Starting recognizer service")
        except rospy.ServiceExecution, e:
            print "Service call failed %s" %e
    #Otherwise, we're already listening so check the heard_words flag
    else:
        if (heard_words):
            # Heard the utterance
            # Reset flags
            words_listened_for = "no_words_listend_for"
            heard_words = False
            listening = False
            return 1    
        else:
            # Have not heard anything
            return 0

def main():

    rospy.init_node("speech_listener")
    rospy.loginfo(rospy.get_name() + ": Started speech listener")
    rospy.Subscriber("output", String, text_callback)
    service = rospy.Service('listen_for', ListenFor, listen_for)
    # On startup, do not listen for anything
    # Call recognizer stop
    try:
        stop = rospy.ServiceProxy('stop', Empty)
        response = stop()
        rospy.loginfo("Stopping recognizer service")
    except rospy.ServiceExecution, e:
        print "Service call failed %s" %e
        

    rospy.spin()
    
if __name__ == "__main__":
    main()
