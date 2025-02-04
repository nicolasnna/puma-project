#!/usr/bin/env python3  
import rospy
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math

def set_pose_callback(msg):
  try:
    rospy.wait_for_service('/set_pose', timeout=5)
    set_pose = rospy.ServiceProxy('/set_pose', SetPose)
    set_pose(msg)
  except rospy.ServiceException as e:
    rospy.logerr(f"Service call failed: {e}")
    
def odom_cb(msg):
  global odom
  odom = msg
    
def change_angle_degree_cb(msg):
  global odom
  newPose = PoseWithCovarianceStamped()
  newPose.pose.pose.position.x = odom.pose.pose.position.x
  newPose.pose.pose.position.y = odom.pose.pose.position.y
  newPose.pose.pose.position.z = odom.pose.pose.position.z
  newPose.pose.pose.orientation.x = 0
  newPose.pose.pose.orientation.y = 0
  newPose.pose.pose.orientation.z = math.sin(math.radians(-msg.data)/2)
  newPose.pose.pose.orientation.w = math.cos(math.radians(-msg.data)/2)
  newPose.header.stamp = rospy.Time.now()
  newPose.header.frame_id = 'odom'
  try:
    rospy.wait_for_service('/set_pose', timeout=5)
    set_pose = rospy.ServiceProxy('/set_pose', SetPose)
    set_pose(newPose)
  except rospy.ServiceException as e:
    rospy.logerr(f"Service call failed: {e}")
  
    
if __name__ == '__main__':
  rospy.init_node('manage_localization_ekf')
  rospy.Subscriber('/puma/localization/set_new_pose', PoseWithCovarianceStamped, set_pose_callback)
  rospy.Subscriber('/puma/localization/change_angle_degree', Float64, change_angle_degree_cb)
  rospy.Subscriber('/puma/localization/ekf_odometry', Odometry, odom_cb)
  
  rospy.spin()