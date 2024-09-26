#!/usr/bin/env python3
import rospy
from puma_manage_map.change_map import ChangeMap

if __name__ == "__main__":
  rospy.init_node('change_map_node')
  
  try:
    change_map = ChangeMap()
  except Exception as e:
    rospy.logwarn("Error al instanciar la clase 'ManageMap'. :%s", e)
    
  i = 0
  while not rospy.is_shutdown():
    change_map.check_quadrants()
    # if i>= 30:
    #   rospy.loginfo('limit x: %f , %f', change_map.x_left_limit, change_map.x_right_limit)
    #   rospy.loginfo('limit y: %f , %f', change_map.y_up_limit, change_map.y_down_limit)
    #   i = 0
    # i += 1
    rospy.Rate(30).sleep()