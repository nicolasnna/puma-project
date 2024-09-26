#!/usr/bin/env python3
import rospy
from puma_controller.puma_ackermann_controller import CmdVelToAckermann

if __name__ == '__main__':
  rospy.init_node('convert_ackermann_node', anonymous=False)
  try:
    convert_ackermann = CmdVelToAckermann()
  except:
    rospy.logerr("Error al instanciar la clase CmdVelToAckermann")

  while not rospy.is_shutdown():
    convert_ackermann.publish_ackermann()
    rospy.spin()
  