#!/usr/bin/env python
import rospy
from controller_lib.wheel_controller import WheelController
from controller_lib.direction_controller import DirectionController

try:
  if __name__ == "__main__":
    rospy.init_node('puma_gazebo_controller')
    
    wheel_controller = WheelController()
    direction_controller = DirectionController()
    
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
      wheel_controller.publish_velocity()
      direction_controller.publish_position()
      rate.sleep()
      
except:
  rospy.logerr("Nodo 'puma_gazebo_controller' desactivado!!")