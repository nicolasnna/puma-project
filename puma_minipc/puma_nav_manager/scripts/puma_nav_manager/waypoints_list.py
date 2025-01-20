import rospy
from puma_msgs.msg import Waypoint, WaypointNav
from std_msgs.msg import Bool

class WaypointList:
  def __init__(self):
    self._waypoints = []
    self.waypoints_list_pub = rospy.Publisher('/puma/navigation/waypoints_list', WaypointNav, queue_size=10)
    self.waypoints_remaining_pub = rospy.Publisher('/puma/navigation/remaining_waypoints', WaypointNav, queue_size=5)
    self.start_subscriber()
    
  def start_subscriber(self):
    self.add_waypoint_sub = rospy.Subscriber('/puma/navigation/add_waypoint', Waypoint, self.add_waypoint_callback)
    self.waypoint_achieved_sub = rospy.Subscriber('/puma/navigation/waypoint_archieved', Waypoint, self.achieved_waypoint_callback)
  
  def 
  
  def add_waypoint_callback(self, waypoint):
    self._waypoints.append(waypoint)
    
  def get_waypoints(self):
    return self._waypoints
    
  def publish_waypoints_list(self):
    waypoints_nav_msg = WaypointNav()
    waypoints_nav_msg.waypoints = self._waypoints
    waypoints_nav_msg.header.stamp = rospy.Time.now()
    self.waypoints_list_pub.publish(waypoints_nav_msg)