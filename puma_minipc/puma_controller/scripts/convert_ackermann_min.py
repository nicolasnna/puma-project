#!/usr/bin/env python3
import rospy
from puma_controller.puma_ackermann_min import CmdVelToAckermannMin

if __name__ == '__main__':
  rospy.init_node('convert_ackermann_node', anonymous=False)
  try:
    convert_ackermann = CmdVelToAckermannMin()
  except:
    rospy.logerr("Error al instanciar la clase CmdVelToAckermannMin")
  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    convert_ackermann.publish_ackermann()
    rate.sleep()
  