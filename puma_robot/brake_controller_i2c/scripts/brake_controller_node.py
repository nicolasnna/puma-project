#!/usr/bin/env python3
from brake_controller.BrakeController import *

ARDUINO_ADDR = 0x8
BUS = 0

node_brake_controller = BrakeController() 

if __name__ == '__main__':
    try:
        node_brake_controller.config_i2c_arduino(ARDUINO_ADDR,BUS)
        result = node_brake_controller.config_arduino_initial()
        
        if result == 1:
            rospy.loginfo("Listo para el ajuste de posicion...")
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                node_brake_controller.publish_info_brake()
                node_brake_controller.set_config_motor_runtime()
                rate.sleep()
        else:
            rospy.logerr("Error al cargar los ajustes")
            
    except:
        rospy.logerr("No se ejecuta el nodo brake_controller")