#!/usr/bin/env python
from puma_joy.cfg import PumaJoyConfig
import rospy
from puma_joy.puma_interface_joy import PumaInterfaceJoy

if __name__ == "__main__":
  rospy.init_node('puma_joy')
  joy_controller = PumaInterfaceJoy()
  
  rate = rospy.Rate(30)
  
  while not rospy.is_shutdown():
    try:
      joy_controller.run_joy()
      rate.sleep()
    except rospy.ROSInterruptException:
      rospy.logwarn(f"Nodo {rospy.get_name()} interrumpido.")
      break