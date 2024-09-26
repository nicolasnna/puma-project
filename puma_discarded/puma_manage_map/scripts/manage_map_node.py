#!/usr/bin/env python3
import rospy
from puma_manage_map.manage_map import ManageMap

if __name__ == "__main__":
  rospy.init_node('manage_map_node')
  
  try:
    manage_map = ManageMap()
  except:
    rospy.logwarn("Error al instanciar la clase 'ManageMap'.")
    
  while not rospy.is_shutdown():
    rospy.spin()