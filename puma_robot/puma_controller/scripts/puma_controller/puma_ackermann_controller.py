#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class CmdVelToAckermann():
  def __init__(self):
    # Initialization of parameters
    self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
    self.wheel_base = rospy.get_param('~wheel_base', 1.15)  # Wheelbase in meters

    # Subscribers
    rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)
    
    # Publishers
    self.ackermann_pub = rospy.Publisher('puma/control/ackermann/command', AckermannDriveStamped, queue_size=10)
    
  def cmd_vel_callback(self, data_received):
    """
    Processes data received from cmd_vel and updates necessary variables
    """
    linear_velocity = data_received.linear.x
    angular_velocity = data_received.angular.z
    self.steering_angle = self.calculate_steering_angle(linear_velocity, angular_velocity)

    # Messages
    ackermann_msg = AckermannDriveStamped()  
    ackermann_msg.header.stamp = rospy.Time.now()
    ackermann_msg.header.frame_id = 'odom'
    ackermann_msg.drive.steering_angle = self.steering_angle
    ackermann_msg.drive.speed = linear_velocity 
    
    self.ackermann_pub.publish(ackermann_msg)
  
  def calculate_steering_angle(self, linear_velocity, angular_velocity):
    """
    Calculates the steering angle and wheel input
    """
    # Calculate turning radius
    if angular_velocity == 0 or linear_velocity == 0:
      return 0
    else:
      radius = linear_velocity / angular_velocity
    # Calculate steering angle
    steering_angle = math.atan(self.wheel_base / radius)
   
    return steering_angle
  
    