#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from puma_brake_msgs.msg import BrakeCmd
from puma_direction_msgs.msg import DirectionCmd
from std_msgs.msg import Bool, Int16
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry

class PumaController():
  def __init__(self):
    ns = '/velocity_puma'
    # Get params
    accelerator_topic = rospy.get_param(ns+'/accelerator_topic', 'puma/accelerator/command')
    brake_topic = rospy.get_param(ns+'/brake_topic', 'puma/brake/command')
    reverse_topic = rospy.get_param(ns+'/revese_topic', 'puma/reverse/command')
    ackermann_topic = rospy.get_param(ns+'/ackermann_topic', 'puma/control/ackermann/command')
    direction_topic = rospy.get_param(ns+'/direction_topic', 'puma/direction/command')
    self.max_value_brake = rospy.get_param(ns+'/max_value_brake', 1000)
    self.min_value_brake = rospy.get_param(ns+'/min_value_brake', 0)
    
    # Subscribers
    rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_callback)
    rospy.Subscriber('/puma/joy/diagnostic', DiagnosticStatus, self.diagnostic_joy_callback)
    rospy.Subscriber('/puma/odometry/filtered', Odometry, self.odometry_callback)

    # Publishers
    self.reverse_pub = rospy.Publisher(reverse_topic, Bool, queue_size=10)
    self.brake_pub = rospy.Publisher(brake_topic, BrakeCmd, queue_size=10)
    self.accel_pub = rospy.Publisher(accelerator_topic, Int16, queue_size=10)
    self.direction_pub = rospy.Publisher(direction_topic, DirectionCmd, queue_size=5)
    self.diagnostic_pub = rospy.Publisher('/puma/control/controller/diagnostic', DiagnosticStatus, queue_size=5)
    
    # Variable
    self.reverse_msg = Bool(False)
    self.brake_msg = BrakeCmd()
    self.accel_msg = Int16()
    self.direction_msg = DirectionCmd()
    self.direction_msg.activate = False
    self.brake_value = 0
    self.vel_linear = 0
    self.enable_joy = False
    self.diagnostic_msg = DiagnosticStatus()
    self.diagnostic_msg.name = 'Puma controller node'
    self.diagnostic_msg.level = 0
    self.diagnostic_msg.message = 'Is works controller between ackerman and puma'
  
  def odometry_callback(self, odom):
    """ Get current velocity and calculate break value"""
    pass
  
  def diagnostic_joy_callback(self, status):
    '''
    Callback diagnostic joy status
    '''
    if status.level == 0:
      self.enable_joy = True
      self.diagnostic_msg.level = 1
      self.diagnostic_msg.message = 'Controller doesnt work, use joy'
    else: 
      self.enable_joy = False
      self.diagnostic_msg.level = 0
      self.diagnostic_msg.message = 'Controller works between ackerman and puma'
  
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    self.vel_linear = acker_data.drive.speed
    self.reverse_msg.data = self.vel_linear < 0 
    self.brake_value = 1000 if self.vel_linear == 0 else 0
    self.brake_msg.position = self.brake_value
    self.direction_msg.angle = acker_data.drive.steering_angle
    
    self.accel_msg.data = int(self.linear_converter_pwm(abs(self.vel_linear), 28, 100, 0.01, 9.8))

  def linear_converter_pwm(self, input_value, pwm_min, pwm_max, speed_min, speed_max):
    """
    Converts linear velocity to a PWM signal for motor control
    """
    return (pwm_max - pwm_min) / (speed_max - speed_min) * (input_value - speed_min) + pwm_min
  
  def velocity_publish(self):
    '''
    Puublish velocity linear control periodically
    '''
    self.diagnostic_pub.publish(self.diagnostic_msg)
    if not self.enable_joy:
      self.accel_pub.publish(self.accel_msg)
      self.brake_pub.publish(self.brake_msg)
      self.reverse_pub.publish(self.reverse_msg)
      self.direction_pub.publish(self.direction_msg)