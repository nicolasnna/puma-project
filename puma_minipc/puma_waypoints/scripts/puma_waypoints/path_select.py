#!/usr/bin/env python3
import rospy
import smach
import rospkg
import tf
import json
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from puma_msgs.msg import GoalGpsArray, GoalGpsNavInfo, GoalGps
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pathlib import Path
from puma_waypoints.utils import calc_goal_from_gps, calculate_bearing_from_xy, yaw_to_quaternion
import tf.transformations

class PathSelect(smach.State):
  """ Smach state for path select to waypoints """
  def __init__(self):
    smach.State.__init__(self, outcomes=['path_follow_mode', 'charge_mode'], output_keys=['waypoints','path_plan','gps_nav', 'gps_index'], input_keys=['waypoints','gps_nav'])
    # Get params
    ns_topic = rospy.get_param('~ns_topic','')
    ''' Publicadores para visualizacion '''
    self.pose_array_publisher = rospy.Publisher(ns_topic + '/path_planned', PoseArray, queue_size=4)
    self.pose_array_completed = rospy.Publisher(ns_topic + '/path_completed', PoseArray, queue_size=4)
    self.nav_gps_info_pub = rospy.Publisher(ns_topic + '/gps_nav_info', GoalGpsNavInfo, queue_size=3)
    
    ''' Variables '''
    self.output_file_path = rospkg.RosPack().get_path('puma_waypoints') + "/saved_path"
    self.waypoints = []
    self.activate_mode = False
    
  def start_subscriber(self):
    add_pose_topic = rospy.get_param('~add_pose_topic', '/initialpose')
    ns_topic = rospy.get_param('~ns_topic','')
    ''' Creacion de suscriptores '''
    self.reset_sub = rospy.Subscriber(ns_topic + "/plan_reset", Empty, self.plan_reset_callback)
    self.upload_sub = rospy.Subscriber(ns_topic + "/plan_upload", String, self.plan_upload_callback)
    self.save_sub = rospy.Subscriber(ns_topic + "/plan_save", String, self.plan_save_callback)
    self.ready_sub = rospy.Subscriber(ns_topic + "/plan_ready", Empty, self.plan_ready_callback)
    self.charge_mode_sub = rospy.Subscriber(ns_topic + "/run_charge_mode", Empty, self.charge_mode_callback)
    self.goal_from_gps_sub = rospy.Subscriber(ns_topic + "/planned_goal_gps", GoalGpsArray, self.plan_from_gps_callback)
    self.goal_2d_sub = rospy.Subscriber(add_pose_topic, PoseWithCovarianceStamped, self.add_pose_2d_callback)
    
  def end_subscriber(self):
    ''' Terminar suscriptores '''
    if self.reset_sub is not None:
      self.reset_sub.unregister()
      self.upload_sub.unregister()
      self.save_sub.unregister()
      self.ready_sub.unregister()
      self.charge_mode_sub.unregister()
      self.goal_from_gps_sub.unregister()
      self.goal_2d_sub.unregister()
    
  def plan_reset_callback(self, empty):
    rospy.loginfo("-> Recibido el comando para limpieza de waypoints.")
    self.initialize_path_waypoints()
    
  def plan_upload_callback(self, name):
    rospy.loginfo("-> Recibido el comando para cargar waypoints desde json.")
    path_file = self.output_file_path + '/' + name.data + '.json'
    if Path(path_file).is_file():
      with open(path_file, 'r') as file:
        self.initialize_path_waypoints() 
        data = json.load(file)
        for position in data:
          pose_plan = PoseWithCovarianceStamped()
          pose_plan.pose.pose.position.x    = position["pos_x"]
          pose_plan.pose.pose.position.y    = position["pos_y"]
          pose_plan.pose.pose.position.z    = position["pos_z"]
          pose_plan.pose.pose.orientation.x = position["rot_x"]
          pose_plan.pose.pose.orientation.y = position["rot_y"]
          pose_plan.pose.pose.orientation.z = position["rot_z"]
          pose_plan.pose.pose.orientation.w = position["rot_w"]
          self.waypoints.append(pose_plan)
        self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray(self.waypoints))
        self.path_selected = True
        rospy.loginfo("--> Cargado el plan: '%s'", name.data)
    else:
      rospy.logwarn("--> No se ha encontrado el archivo %s", path_file)
    
  def plan_save_callback(self, name):
    rospy.loginfo("-> Recibido el comando para guardar waypoints en json.")
    path_file = self.output_file_path + '/' + name.data + '.json'
    if len(self.waypoints) != 0:
      array_waypoint = []
      for points in self.waypoints:
        dict_point = {}
        dict_point.update({"pos_x": points.pose.pose.position.x})
        dict_point.update({"pos_y": points.pose.pose.position.y})
        dict_point.update({"pos_z": points.pose.pose.position.z})
        dict_point.update({"rot_x": points.pose.pose.orientation.x})
        dict_point.update({"rot_y": points.pose.pose.orientation.y})
        dict_point.update({"rot_z": points.pose.pose.orientation.z})
        dict_point.update({"rot_w": points.pose.pose.orientation.w})
        array_waypoint.append(dict_point)
      with open(path_file, 'w') as file:
        json.dump({array_waypoint}, file)
      rospy.loginfo("--> Guardado el plan: '%s'", name.data)
      
  def plan_ready_callback(self, empty):
    rospy.loginfo("-> Recibido el comando de empezar.")
    if len(self.waypoints) != 0:
      self.path_ready = True
    else: 
      rospy.logwarn("--> Waypoints vacio, agrege un destino antes de empezar.")
      
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
    else:
      rospy.logwarn("--> GpsArray vacio.")
  
  def add_pose_2d_callback(self, pose):
    if not self.gps_mode:
      rospy.loginfo("-> Recibido el waypoint x: %.3f, y: %.3f", pose.pose.pose.position.x, pose.pose.pose.position.y)
      distance_limit = rospy.get_param("~distance_limit_points", 10.0)
      if (len(self.waypoints) == 0):
        odometry_topic = rospy.get_param("~odometry_topic", '/puma/odometry/filtered')
        rospy.loginfo("--> Obteniendo la odometria.")
        pos_current = rospy.wait_for_message(odometry_topic, Odometry)
        x_current = pos_current.pose.pose.position.x
        y_current = pos_current.pose.pose.position.y
        distance_between_points = math.sqrt(
          (pose.pose.pose.position.x - x_current)**2 +
          (pose.pose.pose.position.y - y_current)**2)

        yaw = calculate_bearing_from_xy(x_current, y_current, pose.pose.pose.position.x, pose.pose.pose.position.y)
        x_qua, y_qua, z_qua, w_qua = yaw_to_quaternion(yaw)
        while distance_between_points > distance_limit:
          x_inter = x_current + distance_limit * math.cos(math.radians(yaw))
          y_inter = y_current + distance_limit * math.sin(math.radians(yaw))
          
          poseInter = PoseWithCovarianceStamped()
          poseInter.header.frame_id = 'map'
          poseInter.pose.pose.position.x = x_inter
          poseInter.pose.pose.position.y = y_inter
          poseInter.pose.pose.orientation.x = x_qua
          poseInter.pose.pose.orientation.y = y_qua
          poseInter.pose.pose.orientation.z = z_qua
          poseInter.pose.pose.orientation.w = w_qua
          
          self.waypoints.append(self.convert_frame_pose(poseInter,'map'))
          distance_between_points -= distance_limit 
          x_current = x_inter
          y_current = y_inter
          
      self.waypoints.append(self.convert_frame_pose(pose,'map'))
    
  def initialize_path_waypoints(self):
    """ Initialize or reset path waypoints """
    self.waypoints = []
    self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray([]))
    self.pose_array_completed.publish(self.convert_poseCov_to_poseArray([]))
    rospy.loginfo("El arreglo de rutas ha sido limpiado.")
    
  def convert_frame_pose(self, waypoint, target_frame):
    """ Convert frame of PoseWithCovariance to target_frame """
    if waypoint.header.frame_id == target_frame:
      # Already in correct frame
      return waypoint
    if not hasattr(self.convert_frame_pose, 'listener'):
      self.convert_frame_pose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
      self.convert_frame_pose.listener.waitForTransform(
        target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0)
      )
      pose = self.convert_frame_pose.listener.transformPose(target_frame, tmp)
      ret = PoseWithCovarianceStamped()
      ret.header.frame_id = target_frame
      ret.pose.pose = pose.pose
      return ret
    except:
      rospy.logwarn("No se puede transformar pose a %s frame", target_frame)
      exit()
      
  def convert_poseCov_to_poseArray(self, waypoints):
    """ Convert array of pose with covariance to pose array for visualization in rviz """
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses
    
  def execute(self, userdata):
    """ Generate path waypoints """
    rospy.loginfo('------------------------------')
    rospy.loginfo('----- Estado Path Select -----')
    rospy.loginfo('------------------------------')
    self.nav_info_msgs = GoalGpsNavInfo()
    self.nav_gps_index = []
    self.start_subscriber()
    
    if "waypoints" in userdata:
      self.waypoints = userdata.waypoints
      self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray(self.waypoints))
      self.pose_array_completed.publish(self.convert_poseCov_to_poseArray([]))
    else:
      self.initialize_path_waypoints()
    self.path_ready = False
    self.charge_mode = False
    self.gps_mode = False
    self.arrayPoseGoalFromGps = []
    
    try:
      while not self.path_ready and not self.charge_mode and not rospy.is_shutdown():
        self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray(self.waypoints))
        rospy.Rate(30).sleep()
    except rospy.exceptions.ROSInterruptException:
      rospy.logwarn("-> Cerrando puma_waypoints -- PATH_SELECT.")
      
    ''' Enviar datos al siguiente estado '''
    userdata.path_plan = self.convert_poseCov_to_poseArray(self.waypoints)
    userdata.waypoints = self.waypoints
    userdata.gps_nav = self.nav_info_msgs
    userdata.gps_index = self.nav_gps_index
    self.end_subscriber()
    
    if self.charge_mode:
      return 'charge_mode'
    return 'path_follow_mode'