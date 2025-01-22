import rospy
import smach
import actionlib 
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty, String
from puma_msgs.msg import StatusArduino
from puma_state_machine.utils import create_and_publish_log, check_mode_control_navegacion, get_goal_from_waypoint, check_and_get_waypoints

class RunPlanCustom(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success', 'plan_configuration'], input_keys=['plan_configuration_info'])
    
    self.limit_time_status_mb = 0.5
    self.publisher()
    
  def publisher(self):
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
  
  def check_move_base_status(self):
    if (rospy.get_time() - self.last_time_status_mb) > self.limit_time_status_mb:
      self.is_aborted = True
      self.client.cancel_all_goals()
      rospy.sleep(0.1) 
      self.send_log("La navegación ha sido interrumpida por un error en el move_base. Volviendo al modo de selección de rutas.",2)
  
  def execute(self, ud):
    rospy.loginfo('----- Estado ejecución de plan personalizado -----')
    self.send_log("Iniciando en el estado de ejecución de plan de navegación.", 0)
    ''' Variables '''
    self.is_aborted = False
    ''' Definir modo navegacion en el control puma '''
    self.mode_selector_pub.publish(String(data='navegacion'))
    rospy.sleep(0.3)
    
    ''' Comprobar si se ha cambiado al modo de navegación '''
    if not check_mode_control_navegacion('navegacion', self.send_log):
      return 'plan_configuration'
    
    ''' Abrir cliente move_base '''
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()
    rospy.loginfo("Conectado con move_base...")
    ''' Iniciar suscripciones '''
    self.start_subscriber()
    
    ''' Revisar waypoints subidos '''
    exist_waypoints, waypoints_msg = check_and_get_waypoints(self.send_log)
    if not exist_waypoints:
      return 'plan_configuration'

    ''' Ejecutar los planes '''
    try:
      for loops in range(ud.plan_configuration_info['repeat']):
        ''' Reiniciar waypoints logs '''
        self.restart_waypoints_pub.publish(Empty()) 
        
        last_waypoint = waypoints_msg.waypoints[-1]
        goal = get_goal_from_waypoint(last_waypoint)
        is_complete = False
        self.send_log("Ejecutando el plan de navegación...", 0)
        self.client.send_goal(goal)
        while not is_complete:
          self.check_move_base_status()
          is_complete = self.client.wait_for_result(rospy.Duration(1))
          if self.is_aborted:
            break
          
        rospy.sleep(rospy.Duration(ud.plan_configuration_info['minutes_between_repeats']*60))
        if self.is_aborted:
          break
      self.send_log(f"Plan de navegación completado con {loops+1} vueltas.", 0)
      
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