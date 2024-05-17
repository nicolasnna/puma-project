#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool

class ControlReverse():
    '''
    Class for reverse controler
    '''
    def __init__(self):
        rospy.init_node('control_reverse', anonymous=True)
        rospy.Subscriber('control_reverse/activate', Bool, self._reverseCallback)
        
        # Set GPIO
        GPIO.setmode(GPIO.BOARD)
        
        self._reversePin = 13
        self._stateReverse = False
        GPIO.setup(self._reversePin, GPIO.OUT, initial=False)
        
        
    def _reverseCallback(self, data_received):
        if data_received.data != self._stateReverse :
            self._stateReverse = data_received.data
            GPIO.output(self._reversePin, self._stateReverse)
            rospy.loginfo("Cambiando estado de reversa a %s", self._stateReverse)
        
    def deactivateReverse(self):
        rospy.logwarn("Reversa desactivada")
        GPIO.output(self._reversePin, False)
        
        
if __name__ == "__main__":
    control_reverse = ControlReverse()
    rate = rospy.Rate(10)
    
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except: 
        control_reverse.deactivateReverse()