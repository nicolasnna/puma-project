import rospy
from puma_msgs.msg import Waypoint, WaypointNav
from std_msgs.msg import Bool, Empty

class WaypointManager:
  def __init__(self, ns):
    self._waypoints = []
    self._waypoints_completed = []
    self._waypoints_remaining = []
    self._in_navigation = Bool()
    self.init_publishers(ns)
    self.start_subscriber(ns)
    
  def init_publishers(self, ns):
    self.waypoints_list_pub = rospy.Publisher(ns+'/waypoints_info', WaypointNav, queue_size=10)
    self.waypoints_remaining_pub = rospy.Publisher(ns+'/remaining', WaypointNav, queue_size=5)
    self.waypoints_completed_pub = rospy.Publisher(ns+'/completed', WaypointNav, queue_size=5)
    self.in_navigation_pub = rospy.Publisher(ns+'/in_navigation', Bool, queue_size=5)
    
  def start_subscriber(self, ns):
    self.add_waypoint_sub = rospy.Subscriber(ns+'/add', Waypoint, self.add_waypoint_callback)
    self.waypoint_achieved_sub = rospy.Subscriber(ns+'/achieved', Waypoint, self.achieved_waypoint_callback)
    self.clear_waypoints_sub = rospy.Subscriber(ns+'/clear', Empty, self.clear_waypoints)
    self.restart_waypoints_sub = rospy.Subscriber(ns+'/restart', Empty, self.restart_waypoints)
    self.set_in_navigation_sub = rospy.Subscriber(ns+'/set_in_navigation', Bool, self.set_in_navigation)
    
  def restart_waypoints(self, empty):
    rospy.loginfo('Reiniciando waypoints')
    self._waypoints_remaining = self._waypoints[:]
    self._waypoints_completed = []
  
  def clear_waypoints(self, empty):
    rospy.loginfo('Limpiando waypoints')
    self._waypoints = []
    self._waypoints_completed = []
    self._waypoints_remaining = []
  
  def achieved_waypoint_callback(self, waypoint):
    try:
      rospy.loginfo('Waypoint alcanzado, actualizando variables')
      self._waypoints_remaining.remove(waypoint)
      self._waypoints_completed.append(waypoint)
    except ValueError:
      rospy.logwarn('Waypoint no encontrado en la lista de waypoints')
  
  def add_waypoint_callback(self, waypoint):
    if not self._in_navigation.data:
      rospy.loginfo('Añadiendo waypoint')
      self._waypoints.append(waypoint)
      self._waypoints_remaining.append(waypoint)
    
  def set_in_navigation(self, bool):
    self._in_navigation = bool
    
  def get_waypoints(self):
    return self._waypoints
    
  def _publish_waypoints(self, publisher, waypoints):
    waypoints_msg = WaypointNav()
    waypoints_msg.waypoints = waypoints
    waypoints_msg.header.stamp = rospy.Time.now()
    publisher.publish(waypoints_msg)
    
  def publish_status(self):
    self._publish_waypoints(self.waypoints_list_pub, self._waypoints)
    self._publish_waypoints(self.waypoints_remaining_pub, self._waypoints_remaining)
    self._publish_waypoints(self.waypoints_completed_pub, self._waypoints_completed)
    self.in_navigation_pub.publish(self._in_navigation)