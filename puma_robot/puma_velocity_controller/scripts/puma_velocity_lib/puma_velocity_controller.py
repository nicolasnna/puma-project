#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Bool
from puma_direction_msgs.msg import DirectionCmd
from puma_arduino_msgs.msg import StatusArduino
from puma_brake_msgs.msg import BrakeCmd

class PumaVelocityController():
  def __init__(self):
    
    # Params
    self.cmd_vel_topic = rospy.get_param('cmd_vel_topic', 'cmd_vel')
    self.angle_limit = rospy.get_param('angle_limit', 30.0) # Grados
    self.max_steering_angle = self.angle_limit/180 * math.pi
    self.status_topic = rospy.get_param('status_arduino', 'status_arduino')
    self.wheel_base = rospy.get_param('wheel_base', 1.15)
    
    rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_callback) 
    rospy.Subscriber(self.status_topic, StatusArduino, self._status_arduino_callback)
    
    # Publishers
    self.rear_wheels_pub = rospy.Publisher('/accel_puma/value', Int16, queue_size=5)
    self.steering_front_pub = rospy.Publisher('/control_dir/dir_data', DirectionCmd, queue_size=5)
    self.brake_wheels_pub = rospy.Publisher('brake_controller/data_control', BrakeCmd, queue_size=4)
    self.reverse_pub = rospy.Publisher('control_reverse/activate', Bool, queue_size=5)
    
    self.rear_wheels_msg = Int16()
    self.steering_msg = DirectionCmd()
    self.brake_wheels_msg = BrakeCmd()
    self.reverse_msg = Bool()
    # Variable
    self.CONST_VEL_TRANSFORM = rospy.get_param('const_vel_transform',100.0 / 10.0) # Transform m/s to analog read to pwm
    self.position_steering = float("nan")
    self.CONST_RAD_VALUE = 2*math.pi/1024 
    self.ZERO_POSITION_VALUE = rospy.get_param('zero_position_sensor',392)
    self.DIF_LIMIT_VALUE = int(self.angle_limit/360 * 1024)
    self.RIGHT_LIMIT_VALUE = self.ZERO_POSITION_VALUE - self.DIF_LIMIT_VALUE
    self.LEFT_LIMIT_VALUE = self.ZERO_POSITION_VALUE + self.DIF_LIMIT_VALUE

    self.range_extra_value = 3
    
    self.change_steering = False
    self.steering_angle = 0
    self.input_wheels = 0

  def _status_arduino_callback(self, status_received):
    '''
    Processing data from status arduino
    '''
    self.position_steering = status_received.current_position_dir
    # Evaluate if need change steering
    angle_value = int(self.steering_angle/self.CONST_RAD_VALUE) + self.ZERO_POSITION_VALUE
    if (self.position_steering < angle_value - self.range_extra_value) or (self.position_steering > angle_value + self.range_extra_value):
      self.change_steering = True
    else: 
      self.change_steering = False
    
  def _cmd_vel_callback(self, data_received):
    '''
    Processing data received from cmd_vel
    '''
    linear_velocity = data_received.linear.x
    angular_velocity = data_received.angular.z
    if linear_velocity > 0:
      self.steering_angle, self.input_wheels = self.calculate_angles_velocities_output(linear_velocity, angular_velocity)
      self.brake_wheels_msg.position = 0
      if self.reverse_msg.data:
        rospy.loginfo("Quitando modo reversa!!")
      self.reverse_msg.data = False
    elif linear_velocity == 0:
      self.brake_wheels_msg.position = 1000
      if self.reverse_msg.data:
        rospy.loginfo("Quitando modo reversa!!")
      self.reverse_msg.data = False
    else:
      self.steering_angle, self.input_wheels = self.calculate_angles_velocities_output(linear_velocity, angular_velocity)
      self.brake_wheels_msg.position = 0
      if not self.reverse_msg.data:
        rospy.loginfo("Cambiando a modo reversa!!")
      self.reverse_msg.data = True
      
  def control_wheels(self):
    '''
    Control wheels
    '''
    # Change velocity if have changes in steering angle
    if not self.change_steering:
      self.rear_wheels_msg.data = int(self.input_wheels)
    else: 
      self.rear_wheels_msg.data = int(5)
    self.rear_wheels_pub.publish(self.rear_wheels_msg)
  
  def control_steering(self):
    '''
    Control steering angle 
    '''
    self.steering_msg.finish_calibration = True
    
    # Convert rad to int 1024
    angle_value = int(self.steering_angle/self.CONST_RAD_VALUE) + self.ZERO_POSITION_VALUE
    # Evaluate value
    if angle_value > self.LEFT_LIMIT_VALUE:
      angle_value = self.LEFT_LIMIT_VALUE
    elif angle_value < self.RIGHT_LIMIT_VALUE:
      angle_value = self.RIGHT_LIMIT_VALUE

    # Comparate with current position
    if not math.isnan(self.position_steering):
      if angle_value < self.position_steering-self.range_extra_value:
        self.steering_msg.range = -10
        self.steering_msg.activate = True
        self.steering_front_pub.publish(self.steering_msg)
      elif angle_value > self.position_steering+self.range_extra_value:
        self.steering_msg.range = 10
        self.steering_msg.activate = True
        self.steering_front_pub.publish(self.steering_msg)
      else:
        self.steering_msg.activate = False
        self.steering_front_pub.publish(self.steering_msg)
       
    
  def calculate_angles_velocities_output(self, linear_velocity, angular_velocity):
    '''
    Calculate steering angle and input wheels
    '''
    # Calculate radius
    if angular_velocity == 0 or linear_velocity == 0:
      radius = float('inf')
    else:
      radius = linear_velocity / angular_velocity  
      
    steering_angle = math.atan(self.wheel_base / radius)
    # Limit angle max
    if steering_angle > self.max_steering_angle:
      steering_angle = self.max_steering_angle
    elif steering_angle < -self.max_steering_angle:
      steering_angle = -self.max_steering_angle
  
    # Calculate inputs in wheels
    input_wheels = abs(linear_velocity * self.CONST_VEL_TRANSFORM) 
    
    return steering_angle, input_wheels
  
  def control_puma(self):
    '''
    Publish control periodically
    '''
    self.control_steering()
    self.control_wheels()
    self.brake_wheels_pub.publish(self.brake_wheels_msg)
    self.reverse_pub.publish(self.reverse_msg)