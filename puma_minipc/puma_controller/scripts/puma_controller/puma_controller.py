#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from puma_brake_msgs.msg import BrakeCmd
from puma_direction_msgs.msg import DirectionCmd
from std_msgs.msg import Bool, Int16, String
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry

class PumaController():
  def __init__(self):
    ns = '/puma_controller'
    # Get params
    accelerator_topic = rospy.get_param(ns+'/accelerator_topic', 'puma/accelerator/command')
    parking_topic = rospy.get_param(ns+'/parking_topic', 'puma/parking/command')
    reverse_topic = rospy.get_param(ns+'/revese_topic', 'puma/reverse/command')
    ackermann_topic = rospy.get_param(ns+'/ackermann_topic', 'puma/control/ackermann/command')
    direction_topic = rospy.get_param(ns+'/direction_topic', 'puma/direction/command')
    self.range_accel_converter = rospy.get_param(ns+'/range_accel_converter', [25,100])
    self.range_vel_converter = rospy.get_param(ns+'/range_vel_converter', [0.01, 9.8])
    
    # Subscribers
    rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_callback)

    rospy.Subscriber('/puma/odometry/filtered', Odometry, self.odometry_callback)
    rospy.Subscriber('/puma/mode_selector', String, self.selector_mode_callback)

    # Publishers
    self.reverse_pub = rospy.Publisher(reverse_topic, Bool, queue_size=10)
    self.parking_pub = rospy.Publisher(parking_topic, Bool, queue_size=10)
    self.accel_pub = rospy.Publisher(accelerator_topic, Int16, queue_size=10)
    self.direction_pub = rospy.Publisher(direction_topic, DirectionCmd, queue_size=5)
    self.diagnostic_pub = rospy.Publisher('/puma/control/controller/diagnostic', DiagnosticStatus, queue_size=5)
    
    # Variable
    self.reverse_msg = Bool(False)
    self.brake_msg = BrakeCmd()
    self.brake_msg.activate_brake = False
    self.parking_msg = Bool()
    self.parking_msg.data = False
    self.accel_msg = Int16()
    self.direction_msg = DirectionCmd()
    self.direction_msg.activate = False
    self.vel_linear = 0
    self.mode_puma = 'autonomous'
    self.diagnostic_msg = DiagnosticStatus()
    self.diagnostic_msg.name = 'Puma controller node'
    self.diagnostic_msg.level = 0
    self.diagnostic_msg.message = 'Is works controller between ackerman and puma'
  
  def selector_mode_callback(self, mode):
    self.mode_puma = mode.data
    rospy.loginfo("Received "+ mode.data + " mode...")
    if mode.data == "autonomous":
      self.diagnostic_msg.level = 0
      self.diagnostic_msg.message = 'Controller works between ackerman and puma'
    else:
      self.diagnostic_msg.level = 1
      self.diagnostic_msg.message = 'Controller doesnt works'
  
  def odometry_callback(self, odom):
    """ Get current velocity and calculate break value"""
    pass
  
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    self.vel_linear = acker_data.drive.speed
    self.reverse_msg.data = self.vel_linear < 0 
    self.brake_msg.activate_brake = True if self.vel_linear == 0 else False
    self.direction_msg.angle = acker_data.drive.steering_angle
    self.direction_msg.activate = True
    
    self.accel_msg.data = int(self.linear_converter_pwm(
      abs(self.vel_linear), 
      self.range_accel_converter[0], 
      self.range_accel_converter[1], 
      self.range_vel_converter[0], 
      self.range_vel_converter[1]))

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
    if self.mode_puma == "autonomous":
      self.accel_pub.publish(self.accel_msg)
      self.parking_pub.publish(self.parking_msg)
      self.reverse_pub.publish(self.reverse_msg)
      self.direction_pub.publish(self.direction_msg)