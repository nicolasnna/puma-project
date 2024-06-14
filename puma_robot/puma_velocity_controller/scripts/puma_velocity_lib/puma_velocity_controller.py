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
    # Initialization of parameters
    self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
    self.angle_limit = rospy.get_param('~angle_limit', 30.0)  # Angle limit in degrees
    self.max_steering_angle = math.radians(self.angle_limit)  # Convert angle limit to radians
    self.status_topic = rospy.get_param('~status_arduino', 'puma/arduino/status')
    self.wheel_base = rospy.get_param('~wheel_base', 1.15)  # Wheelbase in meters

    # Subscribers
    rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)
    rospy.Subscriber(self.status_topic, StatusArduino, self.status_arduino_callback)

    # Publishers
    self.rear_wheels_pub = rospy.Publisher('puma/accelerator/command', Int16, queue_size=5)
    self.steering_front_pub = rospy.Publisher('puma/direction/command', DirectionCmd, queue_size=5)
    self.brake_wheels_pub = rospy.Publisher('puma/brake/command', BrakeCmd, queue_size=4)
    self.reverse_pub = rospy.Publisher('puma/reverse/command', Bool, queue_size=5)
  
    # Parameters and variables for conversion
    self.CONST_VEL_TRANSFORM = rospy.get_param('~const_vel_transform', 10.0)
    self.position_steering = float("nan")
    self.CONST_RAD_VALUE = 2 * math.pi / 1024
    self.ZERO_POSITION_VALUE = rospy.get_param('~zero_position_sensor', 395)
    self.DIF_LIMIT_VALUE = int(self.angle_limit / 360 * 1024)
    self.RIGHT_LIMIT_VALUE = self.ZERO_POSITION_VALUE - self.DIF_LIMIT_VALUE
    self.LEFT_LIMIT_VALUE = self.ZERO_POSITION_VALUE + self.DIF_LIMIT_VALUE
        
    # Extra variables
    self.range_extra_value = 3
    self.change_steering = False
    self.steering_angle = 0
    self.input_wheels = 0
    
    # Messages
    self.rear_wheels_msg = Int16()
    self.steering_msg = DirectionCmd()
    self.brake_wheels_msg = BrakeCmd()
    self.reverse_msg = Bool()
    
  def status_arduino_callback(self, status_received):
    """
    Processes data received from Arduino and determines if steering angle needs adjustment
    """
    self.position_steering = status_received.current_position_dir
    # Evaluate if need change steering
    angle_value = int(self.steering_angle/self.CONST_RAD_VALUE) + self.ZERO_POSITION_VALUE
      
    self.change_steering = (self.position_steering < angle_value - self.range_extra_value or 
                            self.position_steering > angle_value + self.range_extra_value)
    
  def cmd_vel_callback(self, data_received):
    """
    Processes data received from cmd_vel and updates necessary variables
    """
    linear_velocity = data_received.linear.x
    angular_velocity = -data_received.angular.z
    self.steering_angle, self.input_wheels = self.calculate_angles_velocities_output(linear_velocity, angular_velocity)
    
    self.brake_wheels_msg.position = 0 if linear_velocity != 0 else 1000
    self.reverse_msg.data = linear_velocity < 0
    
    if self.reverse_msg.data:
      rospy.loginfo("Reverse mode!!")

  def control_wheels(self):
    """
    Controls the speed of the rear wheels
    """
    # Change velocity if have changes in steering angle
    self.rear_wheels_msg.data = int(self.input_wheels)
    #self.rear_wheels_msg.data = int(self.input_wheels if not self.change_steering else 30)
    self.rear_wheels_pub.publish(self.rear_wheels_msg)
  
  def control_steering(self):
    """
    Controls the steering angle of the front wheels
    """
    self.steering_msg.finish_calibration = True
    
    # Convert rad to int 1024
    angle_value = int(self.steering_angle / self.CONST_RAD_VALUE) + self.ZERO_POSITION_VALUE
    # Evaluate value
    angle_value = max(min(angle_value, self.LEFT_LIMIT_VALUE), self.RIGHT_LIMIT_VALUE)
    
    # Comparate with current position
    if not math.isnan(self.position_steering):
      if angle_value < self.position_steering - self.range_extra_value:
          self.steering_msg.range = -10
          self.steering_msg.activate = True
      elif angle_value > self.position_steering + self.range_extra_value:
          self.steering_msg.range = 10
          self.steering_msg.activate = True
      else:
          self.steering_msg.activate = False
      self.steering_front_pub.publish(self.steering_msg)

    
  def calculate_angles_velocities_output(self, linear_velocity, angular_velocity):
    """
    Calculates the steering angle and wheel input
    """
    # Calculate turning radius
    if angular_velocity == 0 or linear_velocity == 0:
      radius = float('inf')
    else:
      radius = linear_velocity / angular_velocity
    # Calculate steering angle
    steering_angle = math.atan(self.wheel_base / radius)
    steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
    
    # Calculate wheel input speed
    input_wheels = self.linear_converter_pwm(abs(linear_velocity), 28,100,0.01,9.8) if linear_velocity != 0 else 0

    return steering_angle, input_wheels
  
  def linear_converter_pwm(self, input_value, pwm_min, pwm_max, speed_min, speed_max):
    """
    Converts linear velocity to a PWM signal for motor control
    """
    return (pwm_max - pwm_min) / (speed_max - speed_min) * (input_value - speed_min) + pwm_min
     
  def control_puma(self):
    """
    Periodically publishes control commands
    """
    self.control_steering()
    self.control_wheels()
    self.brake_wheels_pub.publish(self.brake_wheels_msg)
    self.reverse_pub.publish(self.reverse_msg)