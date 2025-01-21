#!/usr/bin/env python3
import rospy
import smach
import rospkg
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from puma_msgs.msg import GoalGpsArray, GoalGpsNavInfo, GoalGps, Log, WaypointNav, Waypoint
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pathlib import Path
from puma_waypoints.utils import calc_goal_from_gps, calculate_bearing_from_xy, yaw_to_quaternion
import tf.transformations

class PathSelect(smach.State):
  """ Smach state for path select to waypoints """
  def __init__(self):
    smach.State.__init__(self, outcomes=['path_follow_mode', 'charge_mode'])
    # Get params
    ns_topic = rospy.get_param('~ns_topic','')
    ''' Publicadores para visualizacion '''
    self.pose_array_publisher = rospy.Publisher(ns_topic + '/path_planned', PoseArray, queue_size=4)
    self.pose_array_completed = rospy.Publisher(ns_topic + '/path_completed', PoseArray, queue_size=4)
    self.nav_gps_info_pub = rospy.Publisher(ns_topic + '/gps_nav_info', GoalGpsNavInfo, queue_size=3)
    self.log_pub = rospy.Publisher('/puma/logs/add_log',  Log, queue_size=3)
    self.waypoints_add_pub = rospy.Publisher('/puma/navigation/waypoints/add', Waypoint, queue_size=3)
    self.waypoints_clear_pub = rospy.Publisher('/puma/navigation/waypoints/clear', Empty, queue_size=3)
    
    ''' Variables '''
    self.output_file_path = rospkg.RosPack().get_path('puma_waypoints') + "/saved_path"
    self.waypoints = []
    self.activate_mode = False
    
  def start_subscriber(self):
    add_pose_topic = rospy.get_param('~add_pose_topic', '/initialpose')
    ns_topic = rospy.get_param('~ns_topic','')
    ''' Creacion de suscriptores '''
    self.reset_sub = rospy.Subscriber(ns_topic + "/plan_reset", Empty, self.plan_reset_callback)
    self.ready_sub = rospy.Subscriber(ns_topic + "/plan_ready", Empty, self.plan_ready_callback)
    self.charge_mode_sub = rospy.Subscriber(ns_topic + "/run_charge_mode", Empty, self.charge_mode_callback)
    self.goal_from_gps_sub = rospy.Subscriber(ns_topic + "/planned_goal_gps", GoalGpsArray, self.plan_from_gps_callback)
    self.goal_2d_sub = rospy.Subscriber(add_pose_topic, PoseWithCovarianceStamped, self.add_pose_2d_callback)
    self.waypoints_list_sub_ = rospy.Subscriber('/puma/navigation/waypoints/waypoints_info', WaypointNav, self.waypoints_list_callback)
  
  def end_subscriber(self):
    ''' Terminar suscriptores '''
    if self.reset_sub is not None:
      self.reset_sub.unregister()
      self.ready_sub.unregister()
      self.charge_mode_sub.unregister()
      self.goal_from_gps_sub.unregister()
      self.goal_2d_sub.unregister()
    
  def send_log(self, msg, level):
    log = Log()
    log.level = level
    log.node = 'puma_waypoints/path_select'
    log.content = msg
    self.log_pub.publish(log)
    
  def plan_reset_callback(self, empty):
    rospy.loginfo("-> Recibido el comando para limpieza de waypoints.")
    self.initialize_path_waypoints()
    self.waypoints_clear_pub.publish(Empty())
    self.send_log("Limpieza de waypoints realizada.",0)
  
  def plan_ready_callback(self, empty):
    rospy.loginfo("-> Recibido el comando de empezar.")
    if len(self.waypoints) != 0:
      self.path_ready = True
    else: 
      rospy.logwarn("--> Waypoints vacio, agrege un destino antes de empezar.")
      
  def waypoints_list_callback(self, waypoint):
    self.waypoints = waypoint.waypoints
      
  def charge_mode_callback(self, empty):
    rospy.loginfo("-> Recibido el comando para el modo carga.")
    self.charge_mode = True
    
  def plan_from_gps_callback(self, data):
    rospy.loginfo("-> Recibido arreglo de GPS waypoints.")
    if len(data.data) != 0:
      distance_limit = rospy.get_param("~distance_limit_points", 10.0)
      self.gps_mode = True
      gps_topic = rospy.get_param("~gps_topic",'/puma/sensors/gps/fix')
      rospy.loginfo("--> Obteniendo el valor del gps del robot.")
      gps_current = rospy.wait_for_message(gps_topic, NavSatFix)
      odometry_topic = rospy.get_param("~odometry_topic", '/puma/odometry/filtered')
      rospy.loginfo("--> Obteniendo la odometria.")
      pos_current = rospy.wait_for_message(odometry_topic, Odometry)
      
      self.nav_info_msgs.index_from = 0
      self.nav_info_msgs.index_to = 1
      self.nav_info_msgs.start.latitude = gps_current.latitude
      self.nav_info_msgs.start.longitude = gps_current.longitude
      orientation = pos_current.pose.pose.orientation
      quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
      _, _, yaw_ = tf.transformations.euler_from_quaternion(quaternion)
      self.nav_info_msgs.start.yaw = yaw_
      
      x_prev = pos_current.pose.pose.position.x
      y_prev = pos_current.pose.pose.position.y
      rospy.loginfo("--> Transformando waypoints.")
      # Revisar por cada destino
      for LatLonGoal in data.data:
        PoseGoal = PoseWithCovarianceStamped()
        PoseGoal.header.frame_id = 'map'
        
        GpsGoalInfo = GoalGps()
        GpsGoalInfo.latitude = LatLonGoal.latitude
        GpsGoalInfo.longitude = LatLonGoal.longitude
        
        x, y = calc_goal_from_gps(gps_current.latitude, gps_current.longitude, LatLonGoal.latitude, LatLonGoal.longitude)
        yaw = calculate_bearing_from_xy(x_prev, y_prev, x, y)
        x_rot, y_rot, z_rot, w_rot = yaw_to_quaternion(yaw)
        GpsGoalInfo.yaw = yaw
        
        # Si la distancia supera el limite del mapa global
        distance_between_points = math.sqrt(x**2 + y**2)
        current_x = pos_current.pose.pose.position.x
        current_y = pos_current.pose.pose.position.y
        
        while (distance_between_points > distance_limit):
          x_inter = current_x + distance_limit * math.cos(math.radians(yaw))
          y_inter = current_y + distance_limit * math.sin(math.radians(yaw))
          
          PoseInterGoal = PoseWithCovarianceStamped()
          PoseInterGoal.header.frame_id = 'map'
          PoseInterGoal.pose.pose.position.x = x_inter
          PoseInterGoal.pose.pose.position.y = y_inter
          PoseInterGoal.pose.pose.orientation.x = x_rot
          PoseInterGoal.pose.pose.orientation.y = y_rot
          PoseInterGoal.pose.pose.orientation.z = z_rot
          PoseInterGoal.pose.pose.orientation.w = w_rot
          self.waypoints.append(PoseInterGoal)
          self.nav_gps_index.append(False)
          
          distance_between_points -= distance_limit
          current_x = x_inter
          current_y = y_inter
        
        new_x = x + pos_current.pose.pose.position.x
        new_y = y + pos_current.pose.pose.position.y
        
        PoseGoal.pose.pose.position.x = new_x
        PoseGoal.pose.pose.position.y = new_y
        PoseGoal.pose.pose.orientation.x = x_rot
        PoseGoal.pose.pose.orientation.y = y_rot
        PoseGoal.pose.pose.orientation.z = z_rot
        PoseGoal.pose.pose.orientation.w = w_rot
        
        self.nav_info_msgs.goals.append(GpsGoalInfo)
        self.nav_gps_index.append(True)
        if LatLonGoal == data.data[0]:
          self.nav_info_msgs.next_goal = GpsGoalInfo
          
        self.waypoints.append(PoseGoal)
        x_prev = new_x
        y_prev = new_y
      self.gps_mode = False
      rospy.loginfo("--> Waypoints actualizado con los destinos desde GPS.")
      self.nav_gps_info_pub.publish(self.nav_info_msgs)
      self.send_log("Waypoints actualizados con los nuevos destinos desde GPS.",0)
    else:
      rospy.logwarn("--> GpsArray vacio.")
  
  def add_pose_2d_callback(self, pose):
    rospy.loginfo("-> Recibido el waypoint x: %.3f, y: %.3f", pose.pose.pose.position.x, pose.pose.pose.position.y)
    
    orientation = pose.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
  
    waypoint = Waypoint()
    waypoint.x = pose.pose.pose.position.x
    waypoint.y = pose.pose.pose.position.y
    waypoint.yaw = yaw
    
    self.waypoints_add_pub.publish(waypoint)
    self.send_log(f"Añadido el waypoint local en x: {round(pose.pose.pose.position.x,2)} e y: {round(pose.pose.pose.position.y,2)}, yaw: {yaw}",0)
    
  def initialize_path_waypoints(self):
    """ Initialize or reset path waypoints """
    self.waypoints = []
    rospy.loginfo("El arreglo de rutas ha sido limpiado.")

  def publish_waypoints_rviz(self):
    pose_array = PoseArray()
    pose_array.header.frame_id = 'map'
    pose_array.header.stamp = rospy.Time.now()
    
    for waypoint in self.waypoints:
      pose = PoseStamped()
      pose.header.frame_id = 'map'
      pose.pose.position.x = waypoint.x
      pose.pose.position.y = waypoint.y
      pose.pose.position.z = 0
      
      quaternion = tf.transformations.quaternion_from_euler(0, 0, waypoint.yaw)
      pose.pose.orientation.x = quaternion[0]
      pose.pose.orientation.y = quaternion[1]
      pose.pose.orientation.z = quaternion[2]
      pose.pose.orientation.w = quaternion[3]
      
      pose_array.poses.append(pose.pose)
    
    self.pose_array_publisher.publish(pose_array)
    
  def execute(self, userdata):
    """ Generate path waypoints """
    rospy.loginfo('------------------------------')
    rospy.loginfo('----- Estado Path Select -----')
    rospy.loginfo('------------------------------')
    self.send_log("Iniciando el estado de selección de ruta (puma_waypoints).",0)
    self.start_subscriber()
    self.initialize_path_waypoints()
    self.path_ready = False
    self.charge_mode = False
    self.gps_mode = False
    
    try:
      while not self.path_ready and not self.charge_mode and not rospy.is_shutdown():
        self.publish_waypoints_rviz()
        rospy.Rate(10).sleep()
    except rospy.exceptions.ROSInterruptException:
      self.send_log("Interrupcion de ROS en la 'selección de ruta' en puma_waypoints.",2)
      rospy.logwarn("-> Cerrando puma_waypoints -- PATH_SELECT.")
      
    ''' Enviar datos al siguiente estado '''
    self.end_subscriber()
    
    if self.charge_mode:
      return 'charge_mode'
    return 'path_follow_mode'