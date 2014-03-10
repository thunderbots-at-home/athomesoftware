########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
########################### DEVELOPER README ###########################
# Info:
#    
#    The listening state is an abstraction that puts the robot into a #    pause until it has heard the phrase or utterance in the parameter #    to the class. Once heard, it will move forward onto the next state #    and before that, execute some given function. 
#
########################################################################
class ListeningState:


    def __init__(self, utterance):
        self.utterance = utterance







