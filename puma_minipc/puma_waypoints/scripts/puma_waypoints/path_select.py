#!/usr/bin/env python3
import rospy
import smach
import rospkg
import tf
import json
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from puma_waypoints_msgs.msg import GoalGpsArray
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pathlib import Path
from puma_waypoints.utils import calc_goal_from_gps, calculate_bearing_from_xy, yaw_to_quaternion

class PathSelect(smach.State):
  """ Smach state for path select to waypoints """
  def __init__(self):
    smach.State.__init__(self, outcomes=['path_follow_mode', 'charge_mode'], output_keys=['waypoints','path_plan','aborted'], input_keys=['waypoints'])
    # Get params
    add_pose_topic = rospy.get_param('~add_pose_topic', '/initialpose')
    ns_topic = rospy.get_param('~ns_topic','')
    # Publisher for visualization
    self.pose_array_publisher = rospy.Publisher(ns_topic + '/path_planned', PoseArray, queue_size=4)
    self.pose_array_completed = rospy.Publisher(ns_topic + '/path_completed', PoseArray, queue_size=4)

    # Variables
    self.output_file_path = rospkg.RosPack().get_path('puma_waypoints') + "/saved_path"
    self.waypoints = []
    self.activate_mode = False
    
    # Subscriber control
    rospy.Subscriber(ns_topic + "/plan_reset", Empty, self.plan_reset_callback)
    rospy.Subscriber(ns_topic + "/plan_upload", String, self.plan_upload_callback)
    rospy.Subscriber(ns_topic + "/plan_save", String, self.plan_save_callback)
    rospy.Subscriber(ns_topic + "/plan_ready", Empty, self.plan_ready_callback)
    rospy.Subscriber(ns_topic + "/run_charge_mode", Empty, self.charge_mode_callback)
    rospy.Subscriber(ns_topic + "/planned_goal_gps", GoalGpsArray, self.plan_from_gps_callback)
    rospy.Subscriber(add_pose_topic, PoseWithCovarianceStamped, self.add_pose_2d_callback)
    
  def plan_reset_callback(self, empty):
    if self.activate_mode:
      rospy.loginfo("-> Recibido el comando para limpieza de waypoints.")
      self.initialize_path_waypoints()
    
  def plan_upload_callback(self, name):
    if self.activate_mode:
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
    if self.activate_mode:
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
    if self.activate_mode:
      rospy.loginfo("-> Recibido el comando de empezar.")
      if len(self.waypoints) != 0:
        self.path_ready = True
      else: 
        rospy.logwarn("--> Waypoints vacio, agrege un destino antes de empezar.")
      
  def charge_mode_callback(self, empty):
    if self.activate_mode:
      rospy.loginfo("-> Recibido el comando para el modo carga.")
      self.charge_mode = True
    
  def plan_from_gps_callback(self, data):
    if self.activate_mode:
      rospy.loginfo("-> Recibido arreglo de GPS waypoints.")
      if len(data.data) != 0:
        self.gps_mode = True
        rospy.loginfo("--> Obteniendo el valor del gps del robot.")
        gps_current = rospy.wait_for_message('/puma/sensors/gps/fix', NavSatFix)
        rospy.loginfo("--> Obteniendo la odometria.")
        pos_current = rospy.wait_for_message('/puma/odometry/filtered', Odometry)
        x_prev = pos_current.pose.pose.position.x
        y_prev = pos_current.pose.pose.position.y
        rospy.loginfo("--> Transformando waypoints.")
        for LatLonGoal in data.data:
          PoseGoal = PoseWithCovarianceStamped()
          PoseGoal.header.frame_id = 'map'
          
          x, y = calc_goal_from_gps(gps_current.latitude, gps_current.longitude, LatLonGoal.latitude, LatLonGoal.longitude)
          yaw = calculate_bearing_from_xy(x_prev, y_prev, x, y)
          x_rot, y_rot, z_rot, w_rot = yaw_to_quaternion(yaw)
          
          new_x = x + pos_current.pose.pose.position.x
          new_y = y + pos_current.pose.pose.position.y
          
          PoseGoal.pose.pose.position.x = x + pos_current.pose.pose.position.x
          PoseGoal.pose.pose.position.y = y + pos_current.pose.pose.position.y
          PoseGoal.pose.pose.orientation.x = x_rot
          PoseGoal.pose.pose.orientation.y = y_rot
          PoseGoal.pose.pose.orientation.z = z_rot
          PoseGoal.pose.pose.orientation.w = w_rot
          
          self.waypoints.append(PoseGoal)
          x_prev = new_x
          y_prev = new_y
        self.gps_mode = False
        rospy.loginfo("--> Waypoints actualizado con los destinos desde GPS.")
      else:
        rospy.logwarn("--> GpsArray vacio.")
  
  def add_pose_2d_callback(self, pose):
    if self.activate_mode:
      if not self.gps_mode:
        rospy.loginfo("-> Recibido el waypoint x: %.3f, y: %.3f", pose.pose.pose.position.x, pose.pose.pose.position.y)
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
    rospy.loginfo('-------------------------------')
    rospy.loginfo('---- Iniciando Path Select ----')
    rospy.loginfo('-------------------------------')
    self.activate_mode = True
    
    if "waypoints" in userdata:
      self.waypoints = userdata.waypoints
      self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray(self.waypoints))
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
      
    # Finish while
    userdata.path_plan = self.convert_poseCov_to_poseArray(self.waypoints)
    userdata.waypoints = self.waypoints
    self.activate_mode = False
    
    if self.charge_mode:
      return 'charge_mode'
    return 'path_follow_mode'