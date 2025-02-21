import rospy
import smach
import actionlib 
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty, String
from puma_msgs.msg import StatusArduino
from puma_state_machine.utils import create_and_publish_log, check_mode_control_navegacion, get_goal_from_waypoint, check_and_get_waypoints
from std_srvs.srv import Empty as EmptySrv
from puma_nav_manager.msg import WaypointsManagerAction, WaypointsManagerGoal
from puma_state_machine.msg import StateMachineAction, StateMachineResult
from puma_robot_status.msg import RobotStatisticsAction, RobotStatisticsGoal

class RunPlan(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success', 'plan_configuration'], input_keys=['plan_configuration_info'])
    
    self.limit_time_status_mb = 0.5
    self.publisher()
    
  def publisher(self):
    ''' Waypoints '''
    self.client_waypoints = actionlib.SimpleActionClient('/puma/navigation/waypoints_manager', WaypointsManagerAction)
    self.client_statistics = actionlib.SimpleActionClient('/puma/statistics', RobotStatisticsAction)
    ''' Modo de control '''
    self.mode_selector_pub = rospy.Publisher('/puma/control/change_mode', String, queue_size=2)
  
  def start_subscriber(self):
    ns_topic = rospy.get_param('~ns_topic', 'state_machine')
    self._srv = actionlib.SimpleActionServer(ns_topic, StateMachineAction, self.execute_srv_cb, False)
    self._srv.start()
    
    self.stop_sub = rospy.Subscriber(ns_topic + '/plan_stop', Empty, self.stop_plan_callback)
    self.arduino_sub = rospy.Subscriber('/puma/arduino/status', StatusArduino, self.arduino_callback)
    self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
  
  def end_subscriber(self):
    if self.stop_sub is not None:
      self.stop_sub.unregister()
      self.arduino_sub.unregister()
      self.move_base_status_sub.unregister()
  
  def send_log(self, msg, level):
    create_and_publish_log(msg, level, 'run_plan')
      
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
    if len(msg.status_list) > 0:
      self.status_move_base = msg.status_list[0].status
  
  def check_move_base_status(self):
    if (rospy.get_time() - self.last_time_status_mb) > self.limit_time_status_mb:
      self.is_aborted = True
      self.client.cancel_all_goals()
      rospy.sleep(0.1) 
      self.send_log("La navegación ha sido interrumpida por un error en el move_base. Volviendo al modo de selección de rutas.",2)
      
  def reset_progress_wp(self):
    try:
      self.client_waypoints.wait_for_server(rospy.Duration(5))
      goal = WaypointsManagerGoal()
      goal.action = 'reset'
      self.client_waypoints.send_goal(goal)
      self.client_waypoints.wait_for_result(rospy.Duration(5))
    except Exception as e:
      rospy.logerr(f"Error al reiniciar waypoints: {e}")
    
  def update_progress_wp(self, waypoint):
    try:
      self.client_waypoints.wait_for_server(rospy.Duration(5))
      goal = WaypointsManagerGoal()
      goal.action = 'update'
      goal.waypoint = waypoint
      self.client_waypoints.send_goal(goal)
      self.client_waypoints.wait_for_result(rospy.Duration(5))
    except Exception as e:
      rospy.logerr(f"Error al actualizar waypoints: {e}")
    
  def start_statistics(self):
    try: 
      self.client_statistics.wait_for_server(rospy.Duration(5))
      goal = RobotStatisticsGoal()
      goal.action = 'start'
      goal.type = 'navigation'
      self.client_statistics.send_goal(goal)
      self.client_statistics.wait_for_result(rospy.Duration(5))
    except Exception as e:
      rospy.logerr(f"Error al iniciar estadísticas: {e}")
      
  def stop_statistics(self):
    try: 
      self.client_statistics.wait_for_server(rospy.Duration(5))
      goal = RobotStatisticsGoal()
      goal.action = 'stop'
      goal.type = 'navigation'
      self.client_statistics.send_goal(goal)
      self.client_statistics.wait_for_result(rospy.Duration(5))
    except Exception as e:
      rospy.logerr(f"Error al detener estadísticas: {e}")
  
  def execute(self, ud):
    rospy.loginfo('----- Estado ejecución de plan -----')
    self.send_log("Iniciando en el estado de ejecución de plan de navegación.", 0)
    ''' Variables '''
    self.is_aborted = False
    self.status_move_base = 0

    self.mode_selector_pub.publish(String(data='navegacion'))
    rospy.sleep(0.3)
    
    if not check_mode_control_navegacion('navegacion', self.send_log):
      self.send_log("No se ha podido cambiar al modo de navegación. Volviendo a configuración de planes.", 1)
      return 'plan_configuration'
    
    ''' Abrir cliente move_base '''
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()
    rospy.loginfo("Conectado con move_base...")
    ''' Iniciar suscripciones '''
    self.start_subscriber()
    
    ''' Reiniciar waypoints '''
    self.reset_progress_wp()
    
    ''' Limpiar costmaps '''
    rospy.wait_for_service('/move_base/clear_costmaps')
    service_clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', EmptySrv)
    
    ''' Revisar waypoints subidos '''
    exist_waypoints, waypoints_msg = check_and_get_waypoints(self.send_log)
    if not exist_waypoints:
      return 'plan_configuration'

    ''' Iniciar estadísticas '''
    self.start_statistics()

    ''' Ejecutar plan en move_base '''
    try:
      for waypoint in waypoints_msg.waypoints:
        try: 
          service_clear_costmaps()
        except rospy.ServiceException as e:
          rospy.logerr(f"Error al limpiar los costmaps: {e}")
        rospy.sleep(0.2)
        
        goal = get_goal_from_waypoint(waypoint)
        is_complete = False
        self.send_log("Ejecutando el plan de navegación...", 0)
        self.client.send_goal(goal)
        rospy.sleep(0.2)
        while not is_complete:
          if self.is_aborted:
            break
          self.check_move_base_status()
          is_complete = self.client.wait_for_result(rospy.Duration(1))
        if self.is_aborted:
          break
        rospy.sleep(0.3)
        if self.status_move_base == 4:
          self.send_log(f"Waypoint ({round(waypoint.x,2), round(waypoint.y,2)}) ha sido abortado por el sistema de navegación. Revisar estado del robot. Volviendo a configuración de plan.", 1)
          self.is_aborted = True
          break
        if is_complete:
          self.send_log(f"Waypoint ({round(waypoint.x,2), round(waypoint.y,2)}) completado.", 0)
          self.update_progress_wp(waypoint)
      
    except Exception as e:
      self.client.cancel_all_goals()
      rospy.sleep(0.1)
      self.send_log(f"No se ha podido efectuar el plan por el error: {e}. Volviendo a configuración de planes", 1)
      self.is_aborted = True
    
    ''' Detener estadísticas '''
    self.stop_statistics()
    
    self.mode_selector_pub.publish(String(data='idle'))
    self.end_subscriber()

    return 'success' if not self.is_aborted else 'plan_configuration'
  
  def execute_srv_cb(self, goal):
    result = StateMachineResult()
    
    result.success = True
    if goal.action == 'stop':
      self.stop_plan_callback(Empty())
    else: 
      result.success = False
      result.message = "Acción no reconocida"
    
    self._srv.set_succeeded(result)