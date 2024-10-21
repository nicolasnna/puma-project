#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from puma_brake_msgs.msg import BrakeCmd
from puma_direction_msgs.msg import DirectionCmd
from std_msgs.msg import Bool, Int16, String
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from puma_controller.pid_antiwindup import PIDAntiWindUp
import time

class PumaController():
  def __init__(self):
    ns = '/puma_controller'
    # Get params
    accelerator_topic = rospy.get_param(ns+'/accelerator_topic', 'puma/accelerator/command')
    parking_topic = rospy.get_param(ns+'/parking_topic', 'puma/parking/command')
    reverse_topic = rospy.get_param(ns+'/revese_topic', 'puma/reverse/command')
    ackermann_topic = rospy.get_param(ns+'/ackermann_topic', 'puma/control/ackermann/command')
    direction_topic = rospy.get_param(ns+'/direction_topic', 'puma/direction/command')
    brake_topic = rospy.get_param(ns+'/brake_topic', 'puma/brake/command')
    self.range_accel_converter = rospy.get_param(ns+'/range_accel_converter', [23,100])
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
    self.brake_pub = rospy.Publisher(brake_topic, BrakeCmd, queue_size=5)
    
    # Variable
    self.vel_linear = 0
    self.angle = 0
    self.vel_linear_odometry = 0
    self.mode_puma = 'autonomous'
    self.reverse_msg = Bool(False)
    self.brake_msg = BrakeCmd(activate_brake=False)
    self.parking_msg = Bool(data=False)
    self.accel_msg = Int16(0)
    self.direction_msg = DirectionCmd(activate=False)
    self.diagnostic_msg = DiagnosticStatus(
        name='Puma controller node', 
        level=0, 
        message='Is works controller between ackerman and puma'
    )
    
    self.pid = PIDAntiWindUp(kp=2, ki=0.1, kd=0.05, min_value=self.range_accel_converter[0], max_value=self.range_accel_converter[1], anti_windup=True)

    self.last_time_odometry = 0
    self.last_time_ackermann = 0

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
    self.last_time_odometry = time.time()
    self.vel_linear_odometry = odom.twist.twist.linear.x
  
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    self.last_time_ackermann = time.time()
    
    self.vel_linear = acker_data.drive.speed
    self.angle = acker_data.drive.steering_angle

      
      # self.accel_msg.data = int(self.linear_converter_pwm(
      #   abs(self.vel_linear), 
      #   self.range_accel_converter[0], 
      #   self.range_accel_converter[1], 
      #   self.range_vel_converter[0], 
      #   self.range_vel_converter[1]))
      
  def calculate_control_inputs(self):
    diagnostic = DiagnosticStatus(
      name='Puma controller node', 
      level=0, 
      message='Not received odometry or ackermann msgs'
      )
    if self.mode_puma == "autonomous":
      accel_msg = Int16(int(self.pid.update(self.vel_linear, self.vel_linear_odometry)))
      rospy.loginfo("PWM calculado: %d", accel_msg.data)
      reverse_msg = Bool(self.vel_linear < 0) 
      brake_msg = BrakeCmd(activate_brake=(self.vel_linear == 0))
      direction_msg = DirectionCmd(angle=self.angle, activate=True)
      
      current_time = time.time()
      if current_time-self.last_time_odometry > 0.2 or current_time-self.last_time_ackermann > 0.2:
        reverse_msg = Bool(False)
        accel_msg = Int16(0)
        brake_msg = BrakeCmd(activate_brake=False)
        direction_msg = DirectionCmd(angle=0, activate=False)
        diagnostic.level = 1
      
      self.accel_pub.publish(accel_msg)
      self.reverse_pub.publish(reverse_msg)
      self.direction_pub.publish(direction_msg)
      self.brake_pub.publish(brake_msg)
    
    self.diagnostic_pub.publish(self.diagnostic_msg if diagnostic.level == 0 else diagnostic)

  def linear_converter_pwm(self, input_value, pwm_min, pwm_max, speed_min, speed_max):
    """
    Converts linear velocity to a PWM signal for motor control
    """
    return (pwm_max - pwm_min) / (speed_max - speed_min) * (input_value - speed_min) + pwm_min
  
