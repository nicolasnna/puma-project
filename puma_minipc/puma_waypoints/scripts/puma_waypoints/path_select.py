#!/usr/bin/env python3
import rospy
import smach
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from puma_waypoints_msgs.msg import GoalGpsArray
from geographiclib.geodesic import Geodesic
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math
import rospkg
import csv
import tf
import numpy as np

def calc_goal_from_gps(origin_lat, origin_long, goal_lat, goal_long):
  # Source: https://github.com/danielsnider/gps_goal/blob/master/src/gps_goal/gps_goal.py
  geod = Geodesic.WGS84
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
  hypotenuse = distance = g['s12']
  azimuth = g['azi1']
  
  azimuth = math.radians(azimuth)
  
  y = adjacent = math.cos(azimuth) * hypotenuse
  x = opposite = math.sin(azimuth) * hypotenuse
  
  return x,y

def calculate_bearing_from_xy(x1, y1, x2, y2):
    """
    Calcular el bearing entre dos puntos en coordenadas (x, y).
    
    Parámetros:
    x1, y1 -- coordenadas del punto origen
    x2, y2 -- coordenadas del punto destino
    
    Retorna:
    Bearing en grados (entre 0° y 360°)
    """
    delta_x = x2 - x1
    delta_y = y2 - y1

    # Calcular el ángulo en radianes
    bearing_rad = math.atan2(delta_y, delta_x)

    # Convertir a grados y normalizar entre 0-360°
    bearing_deg = (math.degrees(bearing_rad) + 360) % 360

    return bearing_deg

def yaw_to_quaternion(yaw):
    """
    Convert a yaw angle (in degrees) to a quaternion.
    
    Parameters:
    yaw -- yaw angle in degrees
    
    Returns:
    (w, x, y, z) -- tuple representing the quaternion
    """
    # Convert yaw from degrees to radians
    yaw = math.radians(yaw)
    
    # Calculate the quaternion components
    w = math.cos(yaw / 2)
    x = 0.0
    y = 0.0
    z = math.sin(yaw / 2)
    
    return (x, y, z, w)

class PathSelect(smach.State):
  """ Smach state for path select to waypoints """
  def __init__(self):
    smach.State.__init__(self, outcomes=['success', 'charge_mode'], output_keys=['waypoints','path_plan','aborted'], input_keys=['waypoints'])
    self.ns = '/waypoints_select/'
    self.ns_robot = '/puma/waypoints'
    # Get params
    self.add_pose_topic = rospy.get_param(self.ns+'add_pose_topic', '/initialpose')
    pose_array_topic = rospy.get_param(self.ns+'pose_array_topic', self.ns_robot+'/path_planned')
    # Publisher for visualization
    self.pose_array_publisher = rospy.Publisher(pose_array_topic, PoseArray, queue_size=1)
    self.pose_array_completed = rospy.Publisher(self.ns_robot+'/path_completed',PoseArray,queue_size=1)

    # Variables
    self.output_file_path = rospkg.RosPack().get_path('puma_waypoints') + "/saved_path"
    self.waypoints = []
    
    # Start thread for reset messages
    def wait_for_reset_plan():
      """ Function for reset path """
      try:
        while not rospy.is_shutdown():
          reset_msg = rospy.wait_for_message(self.ns_robot+'/plan_reset', Empty)
          rospy.loginfo("Received command for reset waypoints")
          self.initialize_path_waypoints()
          rospy.sleep(3)
      except rospy.ROSInterruptException:
        rospy.logwarn("Close wait for reset thread")
        
    reset_thread = threading.Thread(target=wait_for_reset_plan, daemon=True )
    reset_thread.start()
        
  def initialize_path_waypoints(self):
    """ Initialize or reset path waypoints """
    self.waypoints = []
    self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray([]))
    self.pose_array_completed.publish(self.convert_poseCov_to_poseArray([]))
    rospy.loginfo("Path array has been reseted")
    
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
      rospy.logwarn("Can't transform pose to %s frame", target_frame)
      exit()
      
  def convert_poseCov_to_poseArray(self, waypoints):
    """ Convert array of pose with covariance to pose array for visualization in rviz """
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param(self.ns+'/goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses
    
  def execute(self, userdata):
    """ Generate path waypoints """
    if "waypoints" in userdata:
      self.waypoints = userdata.waypoints
      self.path_selected = True
      self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray(self.waypoints))
    else:
      self.initialize_path_waypoints()
      self.path_selected = False
    self.path_ready = False
    self.charge_mode = False
    self.gps_mode = False
    self.arrayPoseGoalFromGps = []
    
    # Load saved path
    def wait_for_upload_path():
      """ Function for load path from csv """
      try:
        file_to_upload = rospy.wait_for_message(self.ns_robot+'/plan_upload', String)
        rospy.loginfo("Received file name for upload plan: '%s'",file_to_upload.data)
        path_file = self.output_file_path + '/' + file_to_upload.data + '.csv'
        with open(path_file, 'r') as file:
          self.initialize_path_waypoints() # Reset plan
          read_csv = csv.reader(file, delimiter = ",")
          for row in read_csv:
            print(row)
            pose_plan = PoseWithCovarianceStamped()
            pose_plan.pose.pose.position.x    = float(row[0])
            pose_plan.pose.pose.position.y    = float(row[1])
            pose_plan.pose.pose.position.z    = float(row[2])
            pose_plan.pose.pose.orientation.x = float(row[3])
            pose_plan.pose.pose.orientation.y = float(row[4])
            pose_plan.pose.pose.orientation.z = float(row[5])
            pose_plan.pose.pose.orientation.w = float(row[6])
            self.waypoints.append(pose_plan)
          self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray(self.waypoints))
        self.path_selected = True
      except rospy.ROSInterruptException:
        rospy.logwarn("Close wait for upload thread")
      except Exception as e:
        rospy.logwarn("Cannot upload or find file '%s' for path waypoints", file_to_upload)
        rospy.logwarn("Error: %s",e)
    # Thread for waiting upload path
    wait_for_upload_thread = threading.Thread(target=wait_for_upload_path, daemon=True)
    wait_for_upload_thread.start()
    
    # Save current path
    def wait_for_save_path():
      """ Function for save actual path from csv """
      try:
        file_to_save = rospy.wait_for_message(self.ns_robot+'/plan_save', String)
        rospy.loginfo("Received file name for save current plan: '%s'",file_to_save.data)
        path_file = self.output_file_path + '/' + file_to_save.data + '.csv'
        with open(path_file, 'w') as file:
          write_csv = csv.writer(file, delimiter=",")
          for pose_plan in self.waypoints:
            write_csv.writerow([
              str(pose_plan.pose.pose.position.x),
              str(pose_plan.pose.pose.position.y),
              str(pose_plan.pose.pose.position.z),
              str(pose_plan.pose.pose.orientation.x),
              str(pose_plan.pose.pose.orientation.y),
              str(pose_plan.pose.pose.orientation.z),
              str(pose_plan.pose.pose.orientation.w)
            ])
      except rospy.ROSInterruptException:
        rospy.logwarn("Close wait for save thread")
    # Thread for waiting save path
    wait_for_save_thread = threading.Thread(target=wait_for_save_path, daemon=True)
    wait_for_save_thread.start()
    
    # Wait for path ready
    def wait_for_path_ready():
      """ complete path and ready """
      try:
        rospy.wait_for_message(self.ns_robot+'/plan_ready', Empty)
        rospy.loginfo("Received path READY message")
        self.path_ready = True
      except rospy.ROSInterruptException:
        rospy.logwarn("Close wait for ready thread")
    # Thread for path ready
    wait_for_ready_thread = threading.Thread(target=wait_for_path_ready, daemon=True)
    wait_for_ready_thread.start()
      
    # Wait for charge mode
    # Thread for charge car mode
    def wait_for_charge_mode():
      rospy.wait_for_message(self.ns_robot+'/run_charge_mode',Empty)
      rospy.loginfo("Change to charge car mode")
      self.charge_mode = True
    wait_for_charge_thread = threading.Thread(target=wait_for_charge_mode, daemon=True)
    wait_for_charge_thread.start()
    
    # wait for gps goal
    # Thread for gps goal
    def wait_for_goal_gps():
      try:
        gps_goals = rospy.wait_for_message(self.ns_robot+'/planned_goal_gps', GoalGpsArray)
        self.gps_mode = True
        rospy.loginfo("Get goal from gps data")
        rospy.loginfo("Wait for current gps fix value")
        gps_current = rospy.wait_for_message('/puma/sensors/gps/fix', NavSatFix)
        rospy.loginfo("Wait for local position")
        pos_current = rospy.wait_for_message('/puma/odometry/filtered', Odometry)
        #prev = {"latitude": gps_current.latitude ,"longitude": gps_current.longitude}
        x_prev = pos_current.pose.pose.position.x
        y_prev = pos_current.pose.pose.position.y
        for LatLonGoal in gps_goals.data:
          PoseGoal = PoseWithCovarianceStamped()
          PoseGoal.header.frame_id = 'map'
          #try:)
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
        rospy.loginfo("Transform from global goal to local goal is complete")
      except rospy.ROSInterruptException:
        rospy.logwarn("Close wait for goal gps thread")
      
    wait_for_goal_gps_thread = threading.Thread(target=wait_for_goal_gps, daemon=True)
    wait_for_goal_gps_thread.start()
    
    while not (self.path_ready and self.path_selected) and not self.charge_mode and not rospy.is_shutdown():
      if not self.gps_mode:
        try:
          pose = rospy.wait_for_message(self.add_pose_topic, PoseWithCovarianceStamped, timeout=1)
        except rospy.ROSException:
          continue
        except Exception as e:
          raise e
        self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray([]))
        rospy.loginfo("Recieved new waypoint to x: %.3f, y: %.3f", pose.pose.pose.position.x, pose.pose.pose.position.y)
        # Transform to pose "map"
        self.waypoints.append(self.convert_frame_pose(pose,'map'))

      self.pose_array_publisher.publish(self.convert_poseCov_to_poseArray(self.waypoints))
      self.path_selected = True
      rospy.Rate(60).sleep()
      
    # Finish while
    userdata.path_plan = self.convert_poseCov_to_poseArray(self.waypoints)
    userdata.waypoints = self.waypoints
    if self.charge_mode:
      return 'charge_mode'
    return 'success'