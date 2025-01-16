#!/usr/bin/env python3
import rospy
import smach
import actionlib
import signal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from puma_msgs.msg import GoalGpsNavInfo, StatusArduino, Log
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, String
import tf
import math

class PathFollow(smach.State):
  """ Smach test of path follow """
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','aborted'], input_keys=['waypoints','path_plan', 'gps_nav', 'gps_index'], output_keys=['waypoints'])
    ns_topic = rospy.get_param('~ns_topic','')
    
    ''' Obtener parametros '''
    self.frame_id = rospy.get_param('~goal_frame_id', 'map')
    self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
    self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
    self.duration = rospy.get_param('~wait_duration', 4.0)
    self.distance_tolerance = rospy.get_param('~distance_tolerance', 1.2)
    self.log_pub = rospy.Publisher('/puma/logs/add_log',  Log, queue_size=3)

    ''' Publicadores '''
    change_mode_topic = rospy.get_param('change_mode_topic', '/puma/control/change_mode')
    self.pose_array_planned = rospy.Publisher(ns_topic+'/path_planned', PoseArray, queue_size=3)
    self.pose_array_completed = rospy.Publisher(ns_topic+'/path_completed', PoseArray, queue_size=3)
    self.mode_selector_pub = rospy.Publisher(change_mode_topic, String, queue_size=3)
    self.nav_gps_info_pub = rospy.Publisher(ns_topic + '/gps_nav_info', GoalGpsNavInfo, queue_size=3)
    self.waypoints_global_planer_pub = rospy.Publisher('/move_base/PumaHybridAStarPlanner/set_waypoints', PoseArray, queue_size=1)
    
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
      rospy.logwarn("-> Navegacion interrumpida puma_waypoints - PATH FOLLOW - debido a error en move_base.")
      self.is_aborted = True
      self.client.cancel_all_goals()
      rospy.sleep(0.1) 
      rospy.logwarn("--- Plan abortado por error en move_base ---")
      self.send_log("La navegación ha sido interrumpida por un error en el move_base. Volviendo al modo de selección de rutas.",2)
    
  def send_log(self, msg, level):
    log = Log()
    log.level = level
    log.node = 'puma_waypoints/path_follow'
    log.content = msg
    self.log_pub.publish(log)
    
  def arduino_callback(self, msg): 
    if msg.control.security_signal and not self.is_aborted:
      rospy.logwarn("-> Navegacion interrumpida puma_waypoints - PATH FOLLOW - debido a señal de seguridad en el arduino.")
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
    self.send_log("Iniciando el estado de seguimiento de ruta (puma_waypoints).",0)

    ''' Iniciar variables'''
    self.last_time_status_mb = rospy.Time.now()
    self.limit_time_status_mb = 0.5
    path_planned = userdata.path_plan
    path_complete = PoseArray()
    path_complete.header.frame_id = path_planned.header.frame_id
    self.is_aborted = False
    ''' Activar modo autonomo '''
    mode_selector_msg = String()
    mode_selector_msg.data = 'navegacion'
    self.mode_selector_pub.publish(mode_selector_msg)
    ''' Abrir cliente move_base '''
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Conectando con move_base...")
    self.client.wait_for_server()
    rospy.loginfo('Conectado con move_base. Iniciando tf listener.')
    self.listener = tf.TransformListener()
    ''' Iniciar suscripciones '''
    self.start_subscriber()
    
    ''' Validar si existe waypoints '''
    if len(userdata.waypoints) == 0:
      rospy.logwarn('No se tiene waypoints almacenados.')
      return 'aborted'
      
    ''' Ejecutar waypoints '''
    index_waypoints = 0
    copy_gps_info = userdata.gps_nav if userdata.gps_nav else GoalGpsNavInfo()

    try:
      
      waypoint = userdata.waypoints[index_waypoints]
      ''' Publicar destino uno por uno '''
      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = self.frame_id
      goal.target_pose.pose.position = waypoint.pose.pose.position
      goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
      
      msg_to_move_base = PoseArray()
      msg_to_move_base.header.frame_id = self.frame_id
      for point in userdata.waypoints:
        msg_to_move_base.poses.append(point.pose.pose)
        
      self.waypoints_global_planer_pub.publish(msg_to_move_base)
      rospy.sleep(0.1)
      self.waypoints_global_planer_pub.publish(msg_to_move_base)
      
      self.client.send_goal(goal)
      rospy.loginfo_throttle(10, "-> Para cancelar el destino: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
      self.client.wait_for_result()

    except Exception as e:
      self.client.cancel_all_goals()
      rospy.sleep(0.1) 
      mode_selector_msg = String()
      mode_selector_msg.data = 'idle'
      self.mode_selector_pub.publish(mode_selector_msg)
      rospy.logwarn("-> Navegacion interrumpida puma_waypoints - PATH FOLLOW - debido error imprevisto %s.",e)
      
    self.end_subscriber()
    if self.is_aborted:
      self.send_log("La navegación ha sido interrumpida por el usuario.",1)
    else:
      self.send_log("La navegación ha sido completada con éxito.",0)
    return 'aborted' if self.is_aborted else 'success'