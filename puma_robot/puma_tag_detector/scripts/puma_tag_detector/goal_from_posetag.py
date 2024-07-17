#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal
import tf
import math

class GoalFromPoseTag:
  ''' Transform pose tag to goal '''
  def __init__(self, tag_name):
    rospy.Subscriber('/puma/tag_detector/pose'+tag_name, PoseStamped, self.pose_from_tag)
    self.goal_pub = rospy.Publisher('/puma/tag_detector/goal'+tag_name, MoveBaseGoal, queue_size=1)
    
  def pose_from_tag(self, pose_received):
    goal = MoveBaseGoal()
    
    quaternion = (pose_received.pose.orientation.x, 
                  pose_received.pose.orientation.y, 
                  pose_received.pose.orientation.z, 
                  pose_received.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    result = tf.transformations.quaternion_from_euler(0, 0, euler[2]+math.pi/2)
    
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = pose_received.pose.position.x - 6*math.cos(euler[2]+math.pi/2)
    goal.target_pose.pose.position.y = pose_received.pose.position.y - 6*math.sin(euler[2]+math.pi/2)
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = result[0]
    goal.target_pose.pose.orientation.y = result[1]
    goal.target_pose.pose.orientation.z = result[2]
    goal.target_pose.pose.orientation.w = result[3]
    
    self.goal_pub.publish(goal)