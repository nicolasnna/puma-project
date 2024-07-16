#!/usr/bin/env python3
import rospy
import smach
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from std_msgs.msg import Empty, String
import rospkg
import csv
import tf

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
    
    while not (self.path_ready and self.path_selected) and not self.charge_mode and not rospy.is_shutdown():
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
      
    # Finish while
    userdata.path_plan = self.convert_poseCov_to_poseArray(self.waypoints)
    userdata.waypoints = self.waypoints
    if self.charge_mode:
      return 'charge_mode'
    return 'success'