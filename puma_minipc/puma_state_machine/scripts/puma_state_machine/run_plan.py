import rospy
import smach
import actionlib 
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty, String
from puma_msgs.msg import Log, WaypointNav, StatusArduino

class RunPlan(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success', 'plan_configuration'], input_keys=['plan_configuration_info'])
    
    self.limit_time_status_mb = 0.5
    self.publisher()
    
  def publisher(self):
    self.log_pub = rospy.Publisher('/puma/logs/add_log', Log, queue_size=2)
    ''' Waypoints '''
    self.restart_waypoints_pub = rospy.Publisher('/puma/navigation/waypoints/restart', Empty, queue_size=2)
    ''' Modo de control '''
    self.mode_selector_pub = rospy.Publisher('/puma/control/change_mode', String, queue_size=2)
  
  def start_subscriber(self):
    ns_topic = rospy.get_param('~ns_topic', '')
    self.stop_sub = rospy.Subscriber(ns_topic + '/plan_stop', Empty, self.stop_plan_callback)
    self.arduino_sub = rospy.Subscriber('/puma/arduino/status', StatusArduino, self.arduino_callback)
    self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
  
  def end_subscriber(self):
    if self.stop_sub is not None:
      self.stop_sub.unregister()
  
  def send_log(self, msg, level):
    log = Log()
    log.level = level
    log.content = msg
    log.node = rospy.get_name()+"/run_plan"
    self.log_pub.publish(log)
    if level == 0:
      rospy.loginfo(msg)
    elif level == 1:
      rospy.logwarn(msg)
    elif level == 2:
      rospy.logerr(msg)
      
  def stop_plan_callback(self, msg):
    self.is_aborted = True
    self.client.cancel_all_goals()
    rospy.sleep(0.1)
    self.send_log("La navegación ha sido interrumpida por una señal de parada. Volviendo al modo de configuración de plan.", 2)
  
  def arduino_callback(self, msg):
    if msg.control.security_signal and not self.is_aborted:
      self.is_aborted = True
      self.client.cancel_all_goals()
      rospy.sleep(0.1)
      self.send_log("La navegación ha sido interrumpida por señal de seguridad detectada en arduino Mega. Volviendo al modo de configuración de plan.", 2)
      
  def move_base_status_callback(self, msg):
    self.last_time_status_mb = rospy.get_time()
  
  def check_move_base_status(self):
    if (rospy.get_time() - self.last_time_status_mb) > self.limit_time_status_mb:
      self.is_aborted = True
      self.client.cancel_all_goals()
      rospy.sleep(0.1) 
      self.send_log("La navegación ha sido interrumpida por un error en el move_base. Volviendo al modo de selección de rutas.",2)
  
  def check_mode_control(self):
    try:
      mode_msg = rospy.wait_for_message('/puma/control/current_mode', String, timeout=5)
    except Exception as e:
      self.send_log(f"No se ha podido obtener el modo actual por el error: {e}. Volviendo a la configuración de planes.", 1)
      return False
    
    if mode_msg.data != 'navegacion':
      self.send_log(f"No se ha podido cambiar al modo de navegación (modo actual {mode_msg.data}). Volviendo a la configuración de planes.", 1)
      return False
    
    return True
  
  def check_waypoints_exist(self):
    try:
      waypoints_msg = rospy.wait_for_message('/puma/navigation/waypoints/waypoints_info', WaypointNav, timeout=5)
    except Exception as e:
      self.send_log(f"No se han cargado correctamente los waypoints por el error: {e}.", 1)
      return False, None
    
    rospy.loginfo(waypoints_msg)
    
    if len(waypoints_msg.waypoints) == 0:
      self.send_log("No se tienen waypoints almacenados. Regresando a la configuración del plan", 1)
      return False, None
    
    return True, waypoints_msg
  
  def get_goal_from_waypoint(self, waypoint):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = waypoint.x
    goal.target_pose.pose.position.y = waypoint.y
    quaternions = tf.transformations.quaternion_from_euler(0, 0, waypoint.yaw)
    goal.target_pose.pose.orientation.x = quaternions[0]
    goal.target_pose.pose.orientation.y = quaternions[1]
    goal.target_pose.pose.orientation.z = quaternions[2]
    goal.target_pose.pose.orientation.w = quaternions[3]
    return goal
  
  def execute(self, ud):
    rospy.loginfo('----- Estado ejecución de plan -----')
    self.send_log("Iniciando en el estado de ejecución de plan de navegación.", 0)
    ''' Variables '''
    self.is_aborted = False
    ''' Definir modo navegacion en el control puma '''
    self.mode_selector_pub.publish(String(data='navegacion'))
    rospy.sleep(0.3)
    
    ''' Comprobar si se ha cambiado al modo de navegación '''
    if not self.check_mode_control():
      return 'plan_configuration'
    
    ''' Abrir cliente move_base '''
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()
    rospy.loginfo("Conectado con move_base...")
    ''' Iniciar suscripciones '''
    self.start_subscriber()
    
    ''' Reiniciar waypoints logs '''
    self.restart_waypoints_pub.publish(Empty()) 
    
    ''' Revisar waypoints subidos '''
    exist_waypoints, waypoints_msg = self.check_waypoints_exist()
    if not exist_waypoints:
      return 'plan_configuration'

    
    ''' Ejecutar plan en move_base '''
    try:
      last_waypoint = waypoints_msg.waypoints[-1]
      goal = self.get_goal_from_waypoint(last_waypoint)
      is_complete = False
      self.send_log("Ejecutando el plan de navegación...", 0)
      self.client.send_goal(goal)
      while not is_complete:
        self.check_move_base_status()
        is_complete = self.client.wait_for_result(rospy.Duration(1))
        if self.is_aborted:
          break
      
    except Exception as e:
      self.client.cancel_all_goals()
      rospy.sleep(0.1)
      self.mode_selector_pub.publish(String(data='idle'))
      self.send_log(f"No se ha podido efectuar el plan por el error: {e}. Volviendo a configuración de planes", 1)
      return 'plan_configuration'
    
    ''' Comprobar si se ha completado el plan '''
    self.end_subscriber()
    if self.is_aborted:
      return 'plan_configuration'
    
    return 'success'