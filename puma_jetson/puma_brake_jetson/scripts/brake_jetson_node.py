#!/usr/bin/env python3
import rospy
from puma_brake_jetson.brake_controller import BrakeController

if __name__ == "__main__":
  rospy.init_node("brake_controller")
  # Get params
  topic_switch_a = rospy.get_param('switch_topic_a', '/puma/brake/switch_a')
  topic_switch_b = rospy.get_param('switch_topic_b', '/puma/brake/switch_b')
  
  topic_brake_front = rospy.get_param('topic_brake_front',"/puma/brake/front_wheels")
  topic_brake_rear = rospy.get_param('topic_brake_rear',"/puma/brake/rear_wheels")
  
  extra_steps_front = rospy.get_param('extra_steps_front', 200)
  extra_steps_rear = rospy.get_param('extra_steps_rear', 200)
  
  brake_controller_front = BrakeController(
    pinDir=37, 
    pinStep=36, 
    topic_switch=topic_switch_a, 
    topic_brake=topic_brake_front, 
    step_extra=extra_steps_front
  )
  
  brake_controller_rear = BrakeController(
    pinDir=35, 
    pinStep=33, 
    topic_switch=topic_switch_b,
    topic_brake=topic_brake_rear, 
    step_extra=extra_steps_rear
  )
  
  try:
    rospy.spin()
      
  except:
    rospy.logwarn("Node brake_controller is stopping")