import rospy
import actionlib
from puma_msgs.msg import Waypoint, WaypointNav
from puma_nav_manager.msg import WaypointsManagerAction, WaypointsManagerFeedback, WaypointsManagerResult

class WaypointManager:
  def __init__(self, ns):
    self._srv = actionlib.SimpleActionServer(ns + '/waypoints_manager', WaypointsManagerAction, self.execute_srv_cb, False)
    self._srv.start()
    rospy.loginfo('Servidor de waypoints iniciado')
    
    self._waypoints = []
    self._waypoints_completed = []
    self._waypoints_remaining = []
    self.init_publishers(ns)
    self.start_subscriber(ns)
    
  def init_publishers(self, ns):
    self.waypoints_list_pub = rospy.Publisher(ns+'/waypoints_list', WaypointNav, queue_size=5)
    self.waypoints_remaining_pub = rospy.Publisher(ns+'/waypoints_remained', WaypointNav, queue_size=5)
    self.waypoints_completed_pub = rospy.Publisher(ns+'/waypoints_completed', WaypointNav, queue_size=5)
    
  def start_subscriber(self, ns):
    self.set_waypoints_sub = rospy.Subscriber(ns+'/set_waypoints', WaypointNav, self.set_waypoints)
    
  def set_waypoints(self, waypoints):
    rospy.loginfo('Estableciendo waypoints')
    self._waypoints = waypoints.waypoints
    self._waypoints_remaining = waypoints.waypoints[:]
    self._waypoints_completed = []
    
  def _publish_waypoints(self, publisher, waypoints):
    waypoints_msg = WaypointNav()
    waypoints_msg.waypoints = waypoints
    waypoints_msg.header.stamp = rospy.Time.now()
    publisher.publish(waypoints_msg)
    
  def execute_add(self, waypoint: Waypoint):
    result = WaypointsManagerResult()
    
    self._waypoints.append(waypoint)
    self._waypoints_remaining.append(waypoint)
    
    result.success = True
    result.message = 'Waypoint añadido con éxito'
    return result
  
  def execute_restart(self):
    result = WaypointsManagerResult()
    
    self._waypoints_remaining = self._waypoints[:]
    self._waypoints_completed.clear()
    
    result.success = True
    result.message = 'Waypoints reiniciados con éxito'
    return result
  
  def execute_clear(self):
    result = WaypointsManagerResult()
    
    self._waypoints.clear()
    self._waypoints_completed.clear()
    self._waypoints_remaining.clear()
    
    result.success = True
    result.message = 'Waypoints limpiados con éxito'
    return result
  
  def execute_achieved(self, waypoint: Waypoint):
    result = WaypointsManagerResult()
    
    try:
      self._waypoints_remaining.remove(waypoint)
      self._waypoints_completed.append(waypoint)
      
      result.success = True
      result.message = 'Waypoint alcanzado con éxito'
      
    except ValueError:
      result.success = False
      result.message = 'Waypoint no encontrado en la lista de waypoints'
    return result
  
  def execute_set(self, waypoints: WaypointNav):
    result = WaypointsManagerResult()
    
    self._waypoints = waypoints.waypoints
    self._waypoints_remaining = waypoints.waypoints[:]
    self._waypoints_completed.clear()
    
    result.success = True
    result.message = 'Waypoints establecidos con éxito'
    return result
    
  def execute_srv_cb(self, goal):
    if goal.action == 'add':
      result = self.execute_add(goal.waypoint)
    elif goal.action == 'clear':
      result = self.execute_clear()
    elif goal.action == 'restart':
      result = self.execute_restart()
    elif goal.action == 'achieved':
      result = self.execute_achieved(goal.waypoint)
    elif goal.action == 'set':
      result = self.execute_set(goal.waypoints)
    else:
      result = WaypointsManagerResult()
      result.success = False
      result.message = 'Acción no válida'
    
    self._srv.set_succeeded(result)
    
  def publish_status(self):
    self._publish_waypoints(self.waypoints_list_pub, self._waypoints)
    self._publish_waypoints(self.waypoints_remaining_pub, self._waypoints_remaining)
    self._publish_waypoints(self.waypoints_completed_pub, self._waypoints_completed)