#!/usr/bin/env python3
import rospy
from puma_controller.velocity_linear_controller import VelocityLinearController

if __name__ == '__main__':
  rospy.init_node('velocity_lineal_controller', anonymous=False)
  try:
    velocity_linear_controller = VelocityLinearController()
  except Exception as e:
    rospy.logerr("Error al instanciar la clase VelocityLinearController: %s", e)
  
  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    velocity_linear_controller.velocity_publish()
    rate.sleep()