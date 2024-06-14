#!/usr/bin/env python3
import rospy
from puma_gps_to_enu.frame_map_odom import FrameMapOdom

if __name__ == '__main__':
  rospy.init_node('enu_to_map_frame_node', anonymous=False)
  try:
    frame_map_odom = FrameMapOdom()
  except Exception as e:
    rospy.logerr("Error a instanciar la clase 'frame_map': ", e)
    
  while not rospy.is_shutdown():
    rospy.spin()