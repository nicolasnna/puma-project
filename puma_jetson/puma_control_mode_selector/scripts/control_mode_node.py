#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

current_mode = "none"

def change_mode_callback(msg):
  global current_mode
  current_mode = msg.data

if __name__ == "__main__":
  rospy.init_node("puma_control_selector")
  
  rospy.Subscriber("puma/control/change_mode", String, change_mode_callback)
  mode_publisher = rospy.Publisher("puma/control/current_mode", String, queue_size=3)
  
  while not rospy.is_shutdown():
    try:
      msg = String(data=current_mode)
      mode_publisher.publish(msg)
      rospy.Rate(15).sleep()
    except rospy.ROSInterruptException:
      rospy.logwarn(f"Nodo {rospy.get_name()} interrumpido.")
      break