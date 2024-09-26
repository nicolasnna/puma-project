#!/usr/bin/env python3
import rospy
from puma_brake_jetson.brake_controller import BrakeController

if __name__ == "__main__":
  rospy.init_node("brake_controller")
  topic_switch = "/puma/brake/switch_status"
  topic_brake = "/puma/brake"
  brake_controller = BrakeController(pinDir=36, pinStep=37, topic_switch=topic_switch, topic_brake=topic_brake)
  
  rate = rospy.Rate(30)
  try:
    while not rospy.is_shutdown():
      brake_controller.executeBrake()
      rate.sleep()
  except:
    rospy.logwarn("Node brake_controller is stopping")