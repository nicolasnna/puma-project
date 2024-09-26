#!/usr/bin/env python3
import rospy
import smach
import threading
import time
import actionlib
import roslaunch
import math
import dynamic_reconfigure.client
from std_msgs.msg import Empty, Bool
from actionlib_msgs.msg import GoalID
from apriltag_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, Twist

class ChargeMode(smach.State):
  ''' State for charge car mode '''
  def __init__(self):
    smach.State.__init__(self, outcomes=['finish_charge'], input_keys=['waypoints'], output_keys=['waypoints'])
    self.ns_robot = '/puma/waypoints'
    self.ns = '/waypoints_charge_mode/'
    enable_topic = rospy.get_param(self.ns+'enable_topic', "/puma/tag_detector/enable")
    self.enable_pub = rospy.Publisher(enable_topic, Bool, queue_size=10)
    self.cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  
  def init_apriltag(self):
    # Configure roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
    roslaunch.configure_logging(uuid)
    self.launch = roslaunch.scriptapi.ROSLaunch()
    cli_apriltag = ['puma_tag_detector', 'apriltag_detection.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_apriltag)
    self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file1)
    self.launch.start()
    
  def init_pid(self):
    ''' Initial values for pid controller '''
    self.dis2camera = 10
    self.dis2tag_goal = rospy.get_param(self.ns+'dis2tag_goal', 1.0)
    self.integral_error = 0.0
    self.prev_error = 0
    self.pid_kp = rospy.get_param(self.ns+'pid/kp', 0.1)
    self.pid_ki = rospy.get_param(self.ns+'pid/ki', 0.01)
    self.pid_kd = rospy.get_param(self.ns+'pid/kd', 0.05)
    self.current_time = rospy.get_time()
    self.max_vel = rospy.get_param(self.ns+'max_vel', 0.4)
  
  def compute_pid_position(self):
    ''' Compute value pid and publish cmd_vel '''
    error = self.dis2camera - self.dis2tag_goal
    self.integral_error = error*self.diff_time + self.integral_error
    derivative = (error - self.prev_error) / self.diff_time
    # Get pid result
    output_pid = self.pid_kp * error + self.pid_ki * self.integral_error + self.pid_kd * derivative
    # Save prev error
    self.prev_error = error
    # Validate limits of output
    vel_x = max(min(output_pid, self.max_vel), -self.max_vel)
    self.publish_cmd_vel(vel_x)
    
  def publish_cmd_vel(self, linear_vel_x):
    vel_msg = Twist()
    vel_msg.linear.x = linear_vel_x
    vel_msg.angular.z = 0
    self.cmdvel_pub.publish(vel_msg)
  
  def callback_dis2camera(self, pos_received):
    ''' Calculate distance to tag according to pos received'''
    x = pos_received.pose.position.x
    y = pos_received.pose.position.y
    self.diff_time = rospy.get_time() - self.current_time
    self.current_time = rospy.get_time()
    self.dis2camera = math.sqrt( x**2 + y**2)
  
  def activate_transform_tag(self, is_activate):
    ''' Change enable mode for transform tag '''
    msg_enable = Bool()
    msg_enable.data = is_activate
    self.enable_pub.publish(msg_enable)
  
  def execute(self, ud):
    rospy.loginfo("Enter in charge mode")
    self.init_pid()
    self.init_apriltag()
    self.activate_transform_tag(is_activate=True)
    self.stop = False
    detections = []
    client_movebase = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Thread for stop mode
    def wait_for_stop():
      rospy.wait_for_message('/puma/waypoints/plan_stop', Empty)
      rospy.loginfo("Stoping charge mode...")
      stop_msg = GoalID()
      rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish(stop_msg)
      self.stop = True
    wait_for_stop_thread = threading.Thread(target=wait_for_stop, daemon=True)
    wait_for_stop_thread.start()
    
    # Wait for detections
    rospy.loginfo("-> waiting for apriltag detections...")
    while len(detections) == 0:
      msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray,timeout=3)
      self.activate_transform_tag(is_activate=True)
      if len(msg.detections)>0:
        rospy.loginfo("-> Is detected apriltag")
        detections = msg.detections
      time.sleep(0.5)
      rospy.loginfo("-> waiting for apriltag detections...")
    
    # Reconfigure TEB tolerance
    rospy.loginfo("-> Change teb tolerance params")
    teb_for_charge = rospy.get_param(self.ns+'teb_for_charge', {})
    client_teb = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS", timeout=30)
    client_teb.update_configuration(teb_for_charge)
    
    # Get goal according to tag
    rospy.loginfo("-> Waiting for goal tag")
    id_tag = detections[0].id[0]
    # Create Subscriber for tag
    rospy.loginfo("-> Create subscriptor")
    rospy.Subscriber('/puma/tag_detector/pose2camera/tag_'+str(id_tag),PoseStamped, self.callback_dis2camera)
    goal_tag = rospy.wait_for_message('/puma/tag_detector/goal/tag_'+str(id_tag), MoveBaseGoal)
    
    # Run to goal nearby position and same orientation
    rospy.loginfo("-> Connecting to move_base...")
    client_movebase.wait_for_server()
    rospy.loginfo("-> Conection sucessful")
    client_movebase.send_goal(goal_tag)
    rospy.loginfo("-> Sending to goal")
    client_movebase.wait_for_result()
    rospy.loginfo("-> Goal completed!")
    time.sleep(0.5)
    
    # Run straight to tag
    rospy.loginfo("-> Switch to PID mode for parking...")
    while not rospy.is_shutdown() and not self.stop and (self.dis2camera > self.dis2tag_goal):
      rospy.loginfo("--> Remaining distance: %.3f mts...", self.dis2camera-self.dis2tag_goal)
      self.compute_pid_position()
      time.sleep(0.1)
    self.publish_cmd_vel(0)
    rospy.loginfo("-> Completed parking!!")
    
    # Reconfigure TEB tolerance
    rospy.loginfo("-> Change teb tolerance params to origin")
    teb_origin = rospy.get_param(self.ns+'teb_origin', {})
    client_teb = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS", timeout=30)
    client_teb.update_configuration(teb_origin)
    
    self.activate_transform_tag(False)
    self.launch.stop()
    return 'finish_charge'