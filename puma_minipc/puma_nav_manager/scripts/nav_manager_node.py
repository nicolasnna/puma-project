import rospy
from puma_nav_manager.waypoints_manager import WaypointManager
from puma_nav_manager.plan_manager import PlanManager

if __name__ == "__main__":
  rospy.init_node('nav_manager_node')
  waypoints_manager = WaypointManager("/puma/navigation/waypoints")
  plan_manager = PlanManager("/puma/navigation/plan")
  
  rospy.loginfo('nodo controlador de navegación iniciado')
  
  while not rospy.is_shutdown():
    waypoints_manager.publish_status()
    plan_manager.publish_status()
    rospy.Rate(5).sleep()