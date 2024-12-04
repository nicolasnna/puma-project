#!/usr/bin/env python3
import rospy
from puma_odometry.pulse_velocity_converter import PulseToVelocityConverter
import tf2_ros
import tf
from nav_msgs.msg import Odometry
from puma_msgs.msg import StatusArduino
from geometry_msgs.msg import TransformStamped
import math
from std_msgs.msg import Bool

class CalculateOdometry():
  '''
  Calculate odometry and frame odom
  '''
  def __init__(self):    
    # Get angle steering
    rospy.Subscriber('/puma/arduino/status', StatusArduino, self._arduino_status_callback)
    rospy.Subscriber('/puma/control/reverse', Bool, self._reverse_callback)
    # Get params
    self.wheels_base = rospy.get_param('~wheels_base', 1.1) # in meters
    self.frame_id = rospy.get_param('~frame_id', 'odom')
    self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
    self.direction_zero = rospy.get_param('~direction_zero', 395)
    self.publish_frame = rospy.get_param('~publish_frame', False)
    # Variables
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0
    
    self.angle_direction = 0
    self.is_reverse = False
    self.last_time = rospy.Time.now()
    self.vx = 0.0
    
    self.velocity_converter = PulseToVelocityConverter()
    self.odom_pub = rospy.Publisher('puma/odom', Odometry, queue_size=10)
    self.odom_broadcaster = tf2_ros.TransformBroadcaster()
    
  def _reverse_callback(self, data_received):
    self.is_reverse = data_received.data
    
  def _arduino_status_callback(self, data_received):
    '''
    Callback for arduino status. Calculate angle direction in rads
    '''
    analog_direction = data_received.current_position_dir
    diff_direction = self.direction_zero - analog_direction
    # if Diff + -> Right
    # if Diff - -> left
    self.angle_direction = diff_direction/1024 * 2 * math.pi
    
  def calculate_odometry(self):
    '''
    Calculate odometry
    '''
    current_time = rospy.Time.now()
    dt = (current_time - self.last_time).to_sec()
    
    self.vx = self.velocity_converter.get_lineal_velocity()
    if self.is_reverse:
      self.vx = -self.vx
    # Angular velocity based in angle direction
    # This is based in Kinemmatic bicycle model
    if self.angle_direction != 0:
      turning_radius = self.wheels_base/ math.tan(self.angle_direction)
      self.angular_velocity = self.vx / turning_radius
    else: 
      self.angular_velocity = 0
    
    # Update position and rotation
    delta_x = self.vx * dt * math.cos(self.theta)
    delta_y = self.vx * dt * math.sin(self.theta)
    delta_theta = self.angular_velocity * dt
    self.x += delta_x
    self.y += delta_y
    self.theta += delta_theta

    if self.publish_frame:
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
    
    q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
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
    odom.twist.twist.angular.z = self.angular_velocity
    
    self.odom_pub.publish(odom)