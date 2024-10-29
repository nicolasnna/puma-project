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
    self.range_accel_converter = rospy.get_param(ns+'/range_accel_converter', [22,100])
  
    
    # Subscribers
    rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_callback)
    rospy.Subscriber('/puma/odometry/filtered', Odometry, self.odometry_callback)
    rospy.Subscriber('/puma/control/current_mode', String, self.selector_mode_callback)

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
    self.diagnostic_msg = DiagnosticStatus(
        name='Puma controller node', 
        level=0, 
        message='Is works controller between ackerman and puma'
    )
    
    self.pid = PIDAntiWindUp(
      kp=0.3, 
      ki=0.2, 
      kd=0.05, 
      min_value=self.range_accel_converter[0], 
      max_value=self.range_accel_converter[1]
    )

    self.last_time_odometry = 0
    self.last_time_ackermann = 0
    self.is_change_reverse = False

  def selector_mode_callback(self, mode):
    self.mode_puma = mode.data
    #rospy.loginfo("Received "+ mode.data + " mode...")
    if mode.data == "autonomous":
      self.diagnostic_msg.level = 0
      self.diagnostic_msg.message = 'Controller works between ackerman and puma'
    else:
      self.diagnostic_msg.level = 1
      self.diagnostic_msg.message = 'Controller doesnt works'
  
  def odometry_callback(self, odom):
    """ Get current velocity and calculate break value"""
    self.last_time_odometry = time.time()
    self.vel_linear_odometry = round(odom.twist.twist.linear.x,4)
  
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    self.last_time_ackermann = time.time()
    
    self.vel_linear = round(acker_data.drive.speed,3)
    self.angle = acker_data.drive.steering_angle
    
    self.is_change_reverse = (self.vel_linear > 0 and self.vel_linear_odometry < 0.3) or (self.vel_linear < 0 and self.vel_linear_odometry > 0.3)


  def calculate_control_inputs(self):
    diagnostic = DiagnosticStatus(
      name='Puma controller node', 
      level=0, 
      message='Not received odometry or ackermann msgs'
      )
    if self.mode_puma == "autonomous":
      if self.vel_linear == 0:
        self.pid.clean_acumulative_error()
      
      # if self.is_change_reverse:
      #   accel_msg = Int16(0)
      #   brake_msg = BrakeCmd(activate_brake=True)
      #   self.pid.clean_acumulative_error()
      #   rospy.loginfo("Cambiando sentido de aceleracion: %d", accel_msg.data)
      # else:
      accel_msg = Int16(int(self.pid.update(abs(self.vel_linear), abs(self.vel_linear_odometry)))) if self.vel_linear != 0 else Int16(self.range_accel_converter[0])
      brake_msg = BrakeCmd(activate_brake=(self.vel_linear == 0))
      #rospy.loginfo("PWM calculado: %d", accel_msg.data)
      
      reverse_msg = Bool(self.vel_linear < 0) 
      direction_msg = DirectionCmd(angle=self.angle, activate=True)
      
      current_time = time.time()
      if current_time-self.last_time_odometry > 0.2 or current_time-self.last_time_ackermann > 0.2:
        reverse_msg = Bool(False)
        accel_msg = Int16(0)
        brake_msg = BrakeCmd(activate_brake=False)
        direction_msg = DirectionCmd(angle=0, activate=False)
        diagnostic.level = 1
        self.pid.clean_acumulative_error()
      
      self.accel_pub.publish(accel_msg)
      self.reverse_pub.publish(reverse_msg)
      self.direction_pub.publish(direction_msg)
      self.brake_pub.publish(brake_msg)
    
    self.diagnostic_pub.publish(self.diagnostic_msg if diagnostic.level == 0 else diagnostic)
