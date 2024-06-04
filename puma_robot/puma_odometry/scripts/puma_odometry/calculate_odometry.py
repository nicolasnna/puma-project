#!/usr/bin/env python3
import rospy
from puma_odometry.pulse_velocity_converter import PulseToVelocityConverter
import tf2_ros
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class CalculateOdometry():
  '''
  Calculate odometry and frame odom
  '''
  def __init__(self):
    # Get params
    self.wheels_base = rospy.get_param('wheels_base', 1.1) # in meters
    self.frame_id = rospy.get_param('frame_id', 'odom')
    self.child_frame_id = rospy.get_param('child_frame_id', 'base_link')
    
    # Variables
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0
    
    self.last_time = rospy.Time.now()
    
    self.velocity_converter = PulseToVelocityConverter()
    self.odom_pub = rospy.Publisher("puma/odom", Odometry, queue_size=10)
    self.odom_broadcaster = tf2_ros.TransformBroadcaster()
    
  def calculate_odometry(self):
    '''
    Calculate odometry
    '''
    current_time = rospy.Time.now()
    dt = (current_time - self.last_time).to_sec()
    
    self.vx = self.velocity_converter.get_lineal_velocity()
    delta_x = self.vx * dt
    self.x += delta_x
    
    self.publish_transform(current_time)
    self.publish_odometry(current_time)
    
    self.last_time = current_time
    
  def publish_transform(self, current_time):
    '''
    Create tf transform and send
    '''
    t = TransformStamped()
    
    t.header.stamp = current_time
    t.header.frame_id = self.frame_id
    t.child_frame_id = self.child_frame_id
    
    t.transform.translation.x = self.x
    t.transform.translation.y = self.y
    t.transform.translation.z = 0.0
    
    t.transform.rotation = tf.transformations.quaternion_from_euler(0, 0, self.theta)
    
    self.odom_broadcaster.sendTransform(t)
    
  def publish_odometry(self, current_time):
    '''
    Create odometry msg and send
    '''
    odom = Odometry()
    
    odom.header.stamp = current_time
    odom.header.frame_id = self.frame_id
    
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.position.z = 0.0
    
    q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    
    odom.child_frame_id = self.child_frame_id
    odom.twist.twist.linear.x = self.vx
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.angular.z = 0.0
    
    self.odom_pub.publish(odom)