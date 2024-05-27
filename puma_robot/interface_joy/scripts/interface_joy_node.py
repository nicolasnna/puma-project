#!/usr/bin/env python3
from interface_joy.interface_joy import *
import rospy

interface_joy = InterfaceJoy()

if __name__ == "__main__":
    try:
        
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            # rospy.loginfo("Valor de lt: %s valor de lr: %s", interface_joy.lt_left, interface_joy.lt_right)
            # rospy.loginfo("boton A: %s, boton B: %s, botton LB: %s",interface_joy.A_button,interface_joy.B_button,interface_joy.LB_button)
            interface_joy.sendDataControl()
            rate.sleep()
            
    except:
        rospy.logerr("Nodo no ejecutado")