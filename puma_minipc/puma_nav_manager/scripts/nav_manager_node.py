import rospy
from puma_nav_manager import *

if __name__ == "__main__":
  rospy.init_node('nav_manager_node')
  
  waypoints_manager = WaypointManager("/puma/navigation")
  server_import_export = ImportExportPlanServer("/puma/navigation")
  
  
  rospy.loginfo('nodo controlador de navegación iniciado')
  
  while not rospy.is_shutdown():
    waypoints_manager.publish_status()
    rospy.Rate(5).sleep()