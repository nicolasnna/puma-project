#!/usr/bin/env python3
import rospy
from puma_controller.puma_controller import PumaController

if __name__ == '__main__':
  rospy.init_node('puma_controller', anonymous=False)
  try:
    puma_controller = PumaController()
  except Exception as e:
    rospy.logerr("Error al instanciar la clase PumaController: %s", e)
  
  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    puma_controller.velocity_publish()
    rate.sleep()