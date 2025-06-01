#!/usr/bin/env python3
import rospy
from puma_msgs.msg import Log

def send_log_message(message, level):
  log_pub = rospy.Publisher('/puma/logs/add_log', Log, queue_size=4)
  msg = Log()
  msg.node = rospy.get_name()
  msg.level = level
  msg.content = message
  if level == 0:
    rospy.loginfo(message)
  elif level == 1:
    rospy.logwarn(message)
  elif level == 2:
    rospy.logerr(message)
  log_pub.publish(msg)