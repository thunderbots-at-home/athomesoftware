#!/usr/bin/env python

## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
############################### IMPORTS ############################


import roslib; roslib.load_manifest('talos_speech')
import rospy

from std_msgs.msg import String
from std_srvs.srv import Empty
from talos_speech.srv import ListenFor
from talos_speech.srv import ListenForAll

########################### DEVELOEPR README #######################

# 1. The speech listener class works ontop of the recognizer.py from
# pocketsphinx package for ros.

# 2. Its intended use is the .listenFor()
# function which will return false until the callback from
# pocketsphinx returns the matched string. 

# 3.This is useful instead of putting a subscripter 
# in every state/class/ for the smach state machine

class SpeechListener:

    def __init__(self):

        self.heard_word = False
        self.listening = False
        self.words_listened_for = []
        self.last_word_heard = "No words heard"

    # Checks the list of words for a match
    def listen_for_words_callback(self, data):
        rospy.loginfo(rospy.get_name() + ": I heard %s", data.data)
        
        if data.data in self.words_listened_for:
            self.heard_word = True
            rospy.loginfo(rospy.get_name() + ": Word '%s' has been heard", data.data)
            self.stop_listening()
        else:
            rospy.loginfo("Data: %s", data.data)
            for words in words_listened_for:
                rospy.loginfo("Word: %s", words)

    #DEPRECATED
    # For more generality use listen for words
    #def listen_for_word_callback(self, data):
    #
    #    rospy.loginfo(rospy.get_name() + "I heard %s", data.data)
    #    # TODO Only does direct comparison, should be a "similarity" comparison
    #    if (data.data == self.word_listened_for):
    #        self.heard_word = True
    #        rospy.loginfo(rospy.get_name() + ": Words have been heard")
    #        self.stop_listening()

    ## Only on state change should the heard_words go to false
    ## Or else the thread could skip/stop/lock/block/jump over the true and into a pool of ggnore

    #DEPRECATED
    # Use listen_for_all for more generality it accepts an array of words
    #def listen_for(self, request):
        # TODO Code for listening and setting words listened for
        # The "and not self.heard_words" covers the case when the robot
        # has just heard the word. 
    #    if not self.listening and not self.heard_word:
            # Start the recognizer
    #        self.start_listening()
    #        self.word_listened_for = request.words
    #        rospy.loginfo("Listening for the phrase: %s", self.word_listened_for)
    #    else:
    #        if (self.heard_word):
            # Heard the utterance
            # Reset flags
    #            self.word_listened_for = "no_word_listend_for"
    #            self.heard_words = False
    #           self.listening = False
    #           return 1    
    #        else:
            # Have not heard anything
    #            return 0
    #    return 0

    # Tells recognizer/output to stop producing values it hears
    # Listens for all the words at once in the request.words

    def listen_for_all(self, request):
        if not self.listening and not self.heard_word:
            self.words_listened_for = request.words
            self.start_listening()
            self.words_listened_for = request.words
            rospy.loginfo("Listening for an array of phrases")
        else:
            if (self.heard_word):
                self.words_listened_for = []
                self.heard_word = False
                self.listening = False
                return self.last_word_heard
            
        return None

    def stop_listening(self):
        try:
                # Once the words have been heard, no need to continue listening, shut down the listening. 
            stop = rospy.ServiceProxy('recognizer/stop', Empty)
            response = stop()
        except rospy.ServiceException, e:
            print "Service call failed %s" %e
            
        self.listening = False

    # Tells recognizer/output to start producing values it hears
    def start_listening(self):
        try:
            # Start listening to the recognizer callbacks
            start = rospy.ServiceProxy('recognizer/start', Empty)
            response = start()
        except rospy.ServiceException, e:
            print "Service call failed %s" %e

        # At the end of the function, because if the service call fails
        # it is not good to have this set to true
        self.heard_words = False
        self.listening = True


def main():

    listener = SpeechListener()
    rospy.init_node("speech_listener")
    rospy.loginfo(rospy.get_name() + ": Started speech listener")
    rospy.Subscriber("recognizer/output", String, listener.listen_for_words_callback)
    service = rospy.Service('listen_for_all', ListenForAll, listener.listen_for_all)
   
    try:
        # On startup, do not listen for anything
        listener.stop_listening()
        rospy.loginfo("Stopping recognizer/output service. This node will activate it again when asked to listen for something.")
    except rospy.ServiceException, e:
        print "Service call failed %s" %e
        
    rospy.spin()
    
if __name__ == "__main__":
    main()
