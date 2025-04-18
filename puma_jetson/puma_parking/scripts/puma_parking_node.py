#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool
import atexit

class ControlBrakeElectric():
    '''
    Class for control brake electric puma
    '''
    def __init__(self):
        rospy.init_node('puma_parking_node')
        rospy.Subscriber("/puma/control/parking", Bool, self._brakeCallback)
        
        # Set GPIO
        GPIO.setmode(GPIO.BOARD)
        
        self._brakeElectricPin = rospy.get_param("parking_pin", 15)
        self._stateBrakeElectric = False
        GPIO.setup(self._brakeElectricPin, GPIO.OUT, initial=False)
        atexit.register(GPIO.cleanup)
        
    def _brakeCallback(self, data_received):
        if data_received.data != self._stateBrakeElectric:
            self._stateBrakeElectric = data_received.data 
            GPIO.output(self._brakeElectricPin, self._stateBrakeElectric)
            rospy.loginfo("Cambiado estado de frenos electricos a %s", self._stateBrakeElectric)
            
    def deactivateBrake(self):
        rospy.logwarn("Frenos desactivados")
        GPIO.output(self._brakeElectricPin, False)

if __name__ == "__main__":
    brake_control = ControlBrakeElectric()
    rate = rospy.Rate(1)
    
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except:
        brake_control.deactivateBrake()