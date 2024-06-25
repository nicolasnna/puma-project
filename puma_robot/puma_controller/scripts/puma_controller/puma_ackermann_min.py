#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from puma_arduino_msgs.msg import StatusArduino
import numpy as np


class CmdVelToAckermannMin():
  def __init__(self):
    # Initialization of parameters
    self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
    self.wheel_base = rospy.get_param('~wheel_base', 1.15)  # Wheelbase in meters

    # Subscribers
    rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)
    rospy.Subscriber('/puma/arduino/status', StatusArduino, self.status_callback)
    
    # Publishers
    self.ackermann_pub = rospy.Publisher('puma/control/ackermann/command', AckermannDriveStamped, queue_size=10)
    
    # Calculate max radius
    self.ang_max = np.deg2rad(15)
    self.radius_max = self.wheel_base/np.arctan(self.ang_max)  # 4.4912
    #rospy.loginfo("Radius max: %f", self.radius_max)
    self.speed_acker = 0.0
    self.A2R = 0.006135742
    self.position_zero = 395
    self.vel = 0.0
    self.current_angle = 0.0
    self.steering_angle = 0.0
    
  def status_callback(self, status_data):
    analog_angle = status_data.current_position_dir
    self.current_angle = (analog_angle - self.position_zero) * self.A2R
    
  def cmd_vel_callback(self, data_received):
    """
    Processes data received from cmd_vel and updates necessary variables
    """
    self.vel = data_received.linear.x
    self.steering_angle = data_received.angular.z
  
  
  def publish_ackermann(self):
    # Messages
    ackermann_msg = AckermannDriveStamped()  
    ackermann_msg.header.stamp = rospy.Time.now()
    ackermann_msg.header.frame_id = 'odom'
    ackermann_msg.drive.steering_angle = self.steering_angle
    
    diff_angle_1 = abs(self.steering_angle - self.current_angle)
    diff_angle_2 = abs(self.current_angle - self.steering_angle)
    
    if ( diff_angle_1 > np.deg2rad(20) or diff_angle_2> np.deg2rad(20)):
      ackermann_msg.drive.speed = 0.0
    elif( diff_angle_1 > np.deg2rad(12) or diff_angle_2> np.deg2rad(12)):
      ackermann_msg.drive.speed = self.vel * 0.2
    elif( diff_angle_1 > np.deg2rad(7) or diff_angle_2> np.deg2rad(7)):
      ackermann_msg.drive.speed = self.vel * 0.5
    elif( diff_angle_1 > np.deg2rad(2) or diff_angle_2> np.deg2rad(2)):
      ackermann_msg.drive.speed = self.vel * 0.8
    else: 
      ackermann_msg.drive.speed = self.vel

    
    self.ackermann_pub.publish(ackermann_msg)