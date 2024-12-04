#!/usr/bin/env python3
import rospy
from puma_controller.puma_controller import PumaController

if __name__ == '__main__':
  rospy.init_node('puma_controller', anonymous=False)
  puma_controller = PumaController()

  rate = rospy.Rate(30)
  
  while not rospy.is_shutdown():
    try:
      puma_controller.manage_control()
      rate.sleep()
    except rospy.ROSInterruptException:
      rospy.logwarn("Nodo interrumpido.")
      break
