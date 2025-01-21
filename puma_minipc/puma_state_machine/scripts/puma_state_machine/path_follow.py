#!/usr/bin/env python3
import rospy
import smach
import actionlib
import signal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from puma_msgs.msg import WaypointNav, StatusArduino, Log, Waypoint
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, String
import tf
import tf.transformations

class PathFollow(smach.State):
  """ Smach test of path follow """
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','aborted'])
    ns_topic = rospy.get_param('~ns_topic','')
    
    ''' Obtener parametros '''
    self.limit_time_status_mb = 0.5
    self.frame_id = rospy.get_param('~goal_frame_id', 'map')
    self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
    self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
    self.duration = rospy.get_param('~wait_duration', 4.0)
    change_mode_topic = rospy.get_param('change_mode_topic', '/puma/control/change_mode')

    ''' Publicadores '''
    self.mode_selector_pub = rospy.Publisher(change_mode_topic, String, queue_size=3)
    self.log_pub = rospy.Publisher('/puma/logs/add_log',  Log, queue_size=3)
    self.restart_waypoints_pub = rospy.Publisher('/puma/navigation/waypoints/restart', Empty, queue_size=3)

    
  def start_subscriber(self):
    ns_topic = rospy.get_param('~ns_topic','')
    self.stop_sub = rospy.Subscriber(ns_topic + "/plan_stop", Empty, self.stop_plan_callback)
    self.arduino_sub = rospy.Subscriber('/puma/arduino/status', StatusArduino, self.arduino_callback)
    self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
    
  def end_subscriber(self):
    self.stop_sub.unregister()
    self.arduino_sub.unregister()
    self.move_base_status_sub.unregister()
    
  def move_base_status_callback(self, msg):
    self.last_time_status_mb = rospy.get_time()
    
  def check_move_base_status(self):
    if (rospy.get_time() - self.last_time_status_mb) > self.limit_time_status_mb:
      rospy.logwarn("-> Navegacion interrumpida puma_state_machine - PATH FOLLOW - debido a error en move_base.")
      self.is_aborted = True
      self.client.cancel_all_goals()
      rospy.sleep(0.1) 
      rospy.logwarn("--- Plan abortado por error en move_base ---")
      self.send_log("La navegación ha sido interrumpida por un error en el move_base. Volviendo al modo de selección de rutas.",2)
    
  def send_log(self, msg, level):
    log = Log()
    log.level = level
    log.node = 'puma_state_machine/path_follow'
    log.content = msg
    self.log_pub.publish(log)
    
  def arduino_callback(self, msg): 
    if msg.control.security_signal and not self.is_aborted:
      rospy.logwarn("-> Navegacion interrumpida puma_state_machine - PATH FOLLOW - debido a señal de seguridad en el arduino.")
      self.is_aborted = True
      self.client.cancel_all_goals()
      rospy.sleep(0.1)
      rospy.logwarn("--- Plan abortado por error en el arduino ---")
      self.send_log("La navegación ha sido interrumpida por señal de seguridad detectada en arduino Mega. Volviendo al modo de selección de rutas.",2)
    
  def stop_plan_callback(self, msg):
    rospy.loginfo("-> Recibido comando de PARAR.")
    self.is_aborted = True
    self.client.cancel_all_goals()
    rospy.sleep(0.1) 
    rospy.logwarn("--- Plan abortado por el usuario ---")

  def execute(self, userdata):
    rospy.loginfo('------------------------------')
    rospy.loginfo('----- Estado Path Follow -----')
    rospy.loginfo('------------------------------')
    self.send_log("Iniciando el estado de seguimiento de ruta (puma_state_machine).",0)

    ''' Iniciar variables'''
    self.last_time_status_mb = rospy.Time.now()
    self.is_aborted = self.is_completed = False
    ''' Activar modo autonomo '''
    mode_selector_msg = String()
    mode_selector_msg.data = 'navegacion'
    self.mode_selector_pub.publish(mode_selector_msg)
    rospy.sleep(0.1)
    ''' Abrir cliente move_base '''
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Conectando con move_base...")
    self.client.wait_for_server()
    ''' Iniciar suscripciones '''
    self.start_subscriber()
    
    ''' Reiniciar waypoints por si acaso '''
    self.restart_waypoints_pub.publish(Empty())
    
    ''' Obtener waypoints actuales '''
    try:
      waypoints_msg = rospy.wait_for_message('/puma/navigation/waypoints/waypoints_info', WaypointNav, rospy.Duration(5))
      # waypoints = waypoints_msg.waypoints
    except rospy.ROSException:
      rospy.logwarn('Tiempo de espera excedido mientras se esperaban los waypoints.')
      return 'aborted'
    
    ''' Validar si existe waypoints '''
    if len(waypoints_msg.waypoints) == 0:
      rospy.logwarn('No se tiene waypoints almacenados.')
      return 'aborted'
    

    try:
      rospy.loginfo_throttle(10, "-> Para cancelar el destino: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = self.frame_id
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose.orientation.w = 1.0
      self.client.send_goal(goal)
      self.client.wait_for_result()

    except Exception as e:
      self.client.cancel_all_goals()
      rospy.sleep(0.1) 
      mode_selector_msg = String()
      mode_selector_msg.data = 'idle'
      self.mode_selector_pub.publish(mode_selector_msg)
      rospy.logwarn("-> Navegacion interrumpida puma_state_machine - PATH FOLLOW - debido error imprevisto %s.",e)
      
    self.end_subscriber()
    
    if self.is_aborted:
      self.send_log("La navegación ha sido interrumpida por el usuario.",1)
      return 'aborted'
    
    self.send_log("La navegación ha sido completada con éxito.",0)
    return 'success'