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
      while not rospy.is_shutdown() and not self.is_aborted and index_waypoints < len(userdata.waypoints):
        waypoint = userdata.waypoints[index_waypoints]
        ''' Publicar destino uno por uno '''
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position = waypoint.pose.pose.position
        goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
        goal_pos_x = waypoint.pose.pose.position.x
        goal_pos_y =  waypoint.pose.pose.position.y
        rospy.loginfo('Ejecutando move_base goal a la position(x,y, theta): %.3f, %.3s, %.3s', 
                      goal_pos_x, goal_pos_y, waypoint.pose.pose.orientation.z)
        rospy.loginfo_throttle(10, "-> Para cancelar el destino: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        ''' Actualizar informacion del waypoints gps '''
        if (len(userdata.gps_nav.goals) > 0 and index_waypoints > 0 ):
          if userdata.gps_index[index_waypoints-1]:
            anterior_index = copy_gps_info.index_to 
            copy_gps_info.index_from = anterior_index
            copy_gps_info.index_to = anterior_index + 1
            copy_gps_info.next_goal = userdata.gps_nav.goals[anterior_index]
        self.nav_gps_info_pub.publish(copy_gps_info)
          
        ''' Enviar destino '''
        if index_waypoints != 0:
          self.client.cancel_all_goals()
          rospy.sleep(0.1) 
        self.client.send_goal(goal)
        if waypoint == userdata.waypoints[-1]: 
          ''' Esperar en caso de ser el ultimo destino '''
          rospy.loginfo('Esperando en el ultimo destino...')
          self.send_log("Esperando la llegada al último destino de la ruta.",0)
          self.client.wait_for_result()
          if (len(userdata.gps_nav.goals) > 0):
            copy_gps_info.index_from = len(userdata.gps_nav.goals)
            copy_gps_info.index_to = len(userdata.gps_nav.goals)
            self.nav_gps_info_pub.publish(copy_gps_info)
        else:
          ''' Bucle para comprobar distancia al destino '''
          distance = 999
          while(distance > self.distance_tolerance):
            self.check_move_base_status()
            if self.is_aborted:
              return 'aborted'
            self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, rospy.Time(0), rospy.Duration(self.duration))
            trans,rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, rospy.Time(0))
            distance = math.sqrt(pow(goal_pos_x-trans[0],2) + pow(goal_pos_y-trans[1],2))
        
        ''' Asegurarse de terminar la ultima orden '''
        self.client.cancel_all_goals()
        rospy.sleep(0.1)
        ''' Actualizar waypoints completados y planificados '''
        path_complete.poses.append(path_planned.poses[0])
        path_planned.poses.pop(0)
        rospy.loginfo('Completado del destino (x, y, theta): %.3f, %.3s, %.3s', 
                      goal_pos_x, goal_pos_y, waypoint.pose.pose.orientation.z)
        self.pose_array_completed.publish(path_complete)
        self.pose_array_planned.publish(path_planned)
        ''' Actualizar index '''
        index_waypoints += 1
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