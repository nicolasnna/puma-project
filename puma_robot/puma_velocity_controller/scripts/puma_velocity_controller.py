#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from control_dir_msgs.msg import dir_data
from init_puma.msg import status_arduino

class PumaVelocityController():
  def __init__(self):
    self.name = ''
    
    rospy.init_node('puma_vel_control')
    
    # Params
    self.cmd_vel_topic = rospy.get_param('cmd_vel_topic', 'cmd_vel')
    self.max_steering_angle = rospy.get_param('max_steering_angle', 0.7854) # En rad
    self.status_topic = rospy.get_param('status_arduino', 'status_arduino')
    self.wheel_base = rospy.get_param('wheel_base', 1.15)
    
    rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_callback) 
    rospy.Subscriber(self.status_topic, status_arduino, self._status_arduino_callback)
    
    # Publishers
    self.rear_wheels_pub = rospy.Publisher(self.name +'/accel_puma/value', Int16, queue_size=5)
    self.steering_front_pub = rospy.Publisher(self.name + '/control_dir/dir_data', dir_data, queue_size=5)
    
    self.rear_wheels_msg = Int16()
    self.steering_msg = dir_data()
    # Variable
    self.CONST_VEL_TRANSFORM = 100.0 / 10.0 # Transform m/s to analog read to pwm
    self.position_steering = float("nan")
    self.ready_accel = False
    self.ZERO_POSITION_VALUE = 392
    self.CONST_RAD_VALUE = 2*math.pi/1024 
    #self.RIGHT_LIMIT_VALUE = 263 
    #self.LEFT_LIMIT_VALUE = 521
    self.RIGHT_LIMIT_VALUE = 392 - 87
    self.LEFT_LIMIT_VALUE = 392 + 87

    self.steering_angle = 0
    self.input_wheels = 0

  def _status_arduino_callback(self, status_received):
    '''
    Processing data from status arduino
    '''
    self.position_steering = status_received.current_position_dir
    
  def _cmd_vel_callback(self, data_received):
    '''
    Processing data received from cmd_vel
    '''
    linear_velocity = data_received.linear.x
    angular_velocity = data_received.angular.z
    if linear_velocity >= 0:
      self.steering_angle, self.input_wheels = self.calculate_angles_velocities_output(linear_velocity, angular_velocity)
    else:
      rospy.logwarn("El robot no esta configurado para ir hacia atras")
      
  def control_wheels(self, input_wheels):
    '''
    Control wheels
    '''
    if self.ready_accel:
      self.rear_wheels_msg.data = int(input_wheels)
      self.rear_wheels_pub.publish(self.rear_wheels_msg)
  
  def control_steering(self, steering_angle):
    '''
    Control steering angle 
    '''
    self.steering_msg.finish_calibration = True
    
    # Convert rad to int 1024
    angle_value = int(steering_angle/self.CONST_RAD_VALUE) + self.ZERO_POSITION_VALUE
    # Evaluate value
    if angle_value > self.LEFT_LIMIT_VALUE:
      angle_value = self.LEFT_LIMIT_VALUE
    elif angle_value < self.RIGHT_LIMIT_VALUE:
      angle_value = self.RIGHT_LIMIT_VALUE

    # Comparate with current position
    if not math.isnan(self.position_steering):
      if angle_value < self.position_steering-3:
        self.steering_msg.range = -10
        self.steering_msg.activate = True
        self.steering_front_pub.publish(self.steering_msg)
      elif angle_value > self.position_steering+3:
        self.steering_msg.range = 10
        self.steering_msg.activate = True
        self.ready_accel = False
        self.steering_front_pub.publish(self.steering_msg)
      else:
        self.steering_msg.activate = False
        self.steering_front_pub.publish(self.steering_msg)
        self.ready_accel = True
    
  def calculate_angles_velocities_output(self, linear_velocity, angular_velocity):
    '''
    Calculate steering angle and input wheels
    '''
    if angular_velocity == 0 or linear_velocity == 0:
      radius = float('inf')
    else:
      radius = linear_velocity / angular_velocity  
      
    steering_angle = math.atan(self.wheel_base / radius)
    
    
    if steering_angle > self.max_steering_angle:
      steering_angle = self.max_steering_angle
    elif steering_angle < -self.max_steering_angle:
      steering_angle = -self.max_steering_angle
  
    input_wheels = linear_velocity * self.CONST_VEL_TRANSFORM
    
    return steering_angle, input_wheels
  
  def publish_msg(self):
    self.control_steering(self.steering_angle)
    self.control_wheels(self.input_wheels)
  
try: 
  if __name__ == "__main__":
    puma_velocity_control = PumaVelocityController()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
      
      puma_velocity_control.publish_msg()
      rate.sleep()
except:
  rospy.logerr("Nodo 'puma_vel_control' desactivado!!!")