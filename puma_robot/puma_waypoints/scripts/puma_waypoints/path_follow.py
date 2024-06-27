#!/usr/bin/env python3
import rospy
import smach
import threading
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty
import tf
import time
import math

class PathFollow(smach.State):
  """ Smach test of path follow """
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','aborted'], input_keys=['waypoints','path_plan'], output_keys=['waypoints'])
    self.ns = '~waypoints'
    self.ns_robot = '/puma/waypoints'
    
    # Get params 
    self.frame_id = rospy.get_param(self.ns+'/goal_frame_id', 'map')
    self.odom_frame_id = rospy.get_param(self.ns+'/odom_frame_id', 'odom')
    self.base_frame_id = rospy.get_param(self.ns+'/base_frame_id', 'base_link')
    self.duration = rospy.get_param(self.ns+'/wait_duration', 0.0)
    
    # Get a move_base action client
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Connecting to move_base...")
    self.client.wait_for_server()
    rospy.loginfo('Connected to move_base.')
    rospy.loginfo('Starting a tf listener.')
    self.listener = tf.TransformListener()
    self.distance_tolerance = rospy.get_param(self.ns+'/waypoiint_distance_tolerance', 1.2)
    
    # Publisher 
    self.pose_array_planned = rospy.Publisher(self.ns_robot+'/path_planned',PoseArray,queue_size=1)
    self.pose_array_completed = rospy.Publisher(self.ns_robot+'/path_completed',PoseArray,queue_size=1)
    
    self.is_aborted = False
    
  def execute(self, userdata):
    
    # Aborted current plan
    def wait_for_stop_plan():
      rospy.wait_for_message('/puma/waypoints/plan_stop', Empty)
      rospy.loginfo('####################################')
      rospy.loginfo('##### PLAN IS ABORTED FOR USER #####')
      rospy.loginfo('####################################')
      stop_msg = GoalID()
      rospy.Publisher('/move_base/cancel',GoalID,queue_size=1).publish(stop_msg)
      self.is_aborted = True
    # Init thread
    stop_thread = threading.Thread(target=wait_for_stop_plan)
    stop_thread.start()
    
    # Execute waypoints each in sequence
    for waypoint in userdata.waypoints:
      # If dont have waypoints
      if userdata.waypoints == []:
        rospy.logwarn('The waypoint queue has been reset.')
        #return 'aborted'
        break
      # Publish the next waypoint as goal
      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = self.frame_id
      goal.target_pose.pose.position = waypoint.pose.pose.position
      goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
      goal_pos_x = waypoint.pose.pose.position.x
      goal_pos_y =  waypoint.pose.pose.position.y
      rospy.loginfo('Executing move_base goal to position(x,y): %s,%s', 
                    goal_pos_x,
                    goal_pos_y)
      rospy.loginfo('With orientation in Z: %s', waypoint.pose.pose.orientation.z)
      rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
      
      # Send goal
      self.client.send_goal(goal)
      if waypoint == userdata.waypoints[-1]:
        self.client.wait_for_result()
      else:
        # Loop wich detect when robot is near to a certain GOAL point
        distance = 10
        while(distance > self.distance_tolerance):
          if self.is_aborted:
            return 'aborted'
          now = rospy.Time.now()
          self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
          trans,rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
          distance = math.sqrt(pow(goal_pos_x-trans[0],2) + pow(goal_pos_y-trans[1],2))
          #rospy.loginfo("DISTANCE: %.5f", distance)
        
    return 'success'