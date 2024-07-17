#!/usr/bin/env python3
import rospy
import smach
import threading
import time
import actionlib
import roslaunch
from std_msgs.msg import Empty, Bool
from actionlib_msgs.msg import GoalID
from apriltag_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import dynamic_reconfigure.client

class ChargeMode(smach.State):
  ''' State for charge car mode '''
  def __init__(self):
    smach.State.__init__(self, outcomes=['finish_charge'], input_keys=['waypoints'], output_keys=['waypoints'])
    self.ns_robot = '/puma/waypoints'
    self.ns = '/waypoints_charge_mode/'
    enable_topic = rospy.get_param(self.ns+'enable_topic', "/puma/tag_detector/enable")
    self.enable_pub = rospy.Publisher(enable_topic, Bool, queue_size=10)
  
  def init_apriltag(self):
    # Configure roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
    roslaunch.configure_logging(uuid)
    self.launch = roslaunch.scriptapi.ROSLaunch()
    cli_apriltag = ['puma_tag_detector', 'apriltag_detection.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_apriltag)
    self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file1)
    self.launch.start()
  
  def activate_transform_tag(self, is_activate):
    ''' Change enable mode for transform tag '''
    msg_enable = Bool()
    msg_enable.data = is_activate
    self.enable_pub.publish(msg_enable)
  
  def execute(self, ud):
    rospy.loginfo("Enter in charge mode")
    self.init_apriltag()
    self.activate_transform_tag(is_activate=True)
    detections = []
    client_movebase = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Thread for stop mode
    def wait_for_stop():
      rospy.wait_for_message('/puma/waypoints/plan_stop', Empty)
      rospy.loginfo("Stoping charge mode...")
      stop_msg = GoalID()
      rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish(stop_msg)
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
    goal_tag = rospy.wait_for_message('/puma/tag_detector/goal/tag_'+str(id_tag), MoveBaseGoal)
    
    # Run to goal
    rospy.loginfo("-> Connecting to move_base...")
    client_movebase.wait_for_server()
    rospy.loginfo("-> Conection sucessful")
    client_movebase.send_goal(goal_tag)
    rospy.loginfo("-> Sending to goal")
    client_movebase.wait_for_result()
    rospy.loginfo("-> Goal completed!")
    
    # Reconfigure TEB tolerance
    rospy.loginfo("-> Change teb tolerance params to origin")
    teb_origin = rospy.get_param(self.ns+'teb_origin', {})
    client_teb = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS", timeout=30)
    client_teb.update_configuration(teb_origin)
    
    self.activate_transform_tag(False)
    self.launch.stop()
    return 'finish_charge'