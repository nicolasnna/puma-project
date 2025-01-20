import rospy
from puma_nav_manager.waypoints_list import WaypointList

if __name__ == "__main__":
  rospy.init_node('nav_manager_node')
  waypoints_list = WaypointList()