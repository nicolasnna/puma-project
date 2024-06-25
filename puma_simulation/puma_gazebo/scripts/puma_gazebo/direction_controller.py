#!/usr/bin/env python
import rospy
from puma_direction_msgs.msg import DirectionCmd
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Bool
from diagnostic_msgs.msg import DiagnosticStatus
import numpy as np

class DirectionController():
  '''
  A considerar: el giro a la derecha resulta en un valor menor en el sensor de posicion, y la izquiera un valor mayor.
  Es decir, opuesto al movimiento
  '''
  def __init__(self):
    # Publisher
    self._pub_dir_left = rospy.Publisher('/dir_left_controller/command', Float64, queue_size=5)
    self._pub_dir_right = rospy.Publisher('/dir_right_controller/command', Float64, queue_size=5)
    
    # Subscriber
    rospy.Subscriber('puma/direction/command', DirectionCmd, self.dir_callback)
    rospy.Subscriber('puma/control/ackermann/command', AckermannDriveStamped, self.ackermann_callback)
    rospy.Subscriber('puma/reverse/command', Bool, self.reverse_callback)
    rospy.Subscriber('puma/joy/diagnostic', DiagnosticStatus, self.diagnostic_joy)
    
    # Variable
    self.value_offset = rospy.get_param('direction_value_offset', 0.0)
    # Dir is config in the same orientation
    self.dir_value = Float64()
    self.dir_value.data = self.value_offset
    
    self._activate_dir = False
    self._orientation_dir = 0.0
    self.ZERO_POSITION_VALUE = 395
    self.CONST_RAD_VALUE = 2*np.pi/1024 
    # Limit absolutely
    # self.RIGHT_LIMIT_VALUE =  (263 - self.ZERO_POSITION_VALUE) 
    # self.LEFT_LIMIT_VALUE = (521 - self.ZERO_POSITION_VALUE) 
    # 30 grados limit
    self.RIGHT_LIMIT_VALUE =  (310 - self.ZERO_POSITION_VALUE) 
    self.LEFT_LIMIT_VALUE = (480 - self.ZERO_POSITION_VALUE) 
    
    self.current_position = 0
    self.current_enable = False
    self.current_angle = 0.0
    self.angle_goal = 0
    self.tolerance = 0.05
    
    self.is_reverse = False
    self.enable_joy = False
    
  def diagnostic_joy(self, status):
    '''
    Callback from status of joy
    '''
    if status.level == 0:
      self.enable_joy = True
    else:
      self.enable_joy = False
    
  def ackermann_callback(self, acker_data):
    '''
    Callback from ackermann drive controller callback
    '''
    self.angle_goal = acker_data.drive.steering_angle
    
  def reverse_callback(self, reverse_data):
    '''
    Callback from reverse node
    '''
    self.is_reverse = reverse_data.data
    
  def direction_to_angle(self):
    '''
    Move direction to specific angle
    '''
    current_angle_analog = self.current_angle / self.CONST_RAD_VALUE
    
    # if not self.is_reverse:
    # For forward drive
    is_diff_angle_positiv = (self.current_angle*(1+self.tolerance) < self.angle_goal) and (current_angle_analog <= self.LEFT_LIMIT_VALUE)
    is_diff_angle_negativ = (self.current_angle*(1-self.tolerance) > self.angle_goal) and (current_angle_analog >= self.RIGHT_LIMIT_VALUE)
    
    if is_diff_angle_positiv:
      self.dir_value.data = self.dir_value.data + (7*self.CONST_RAD_VALUE/3)
      #rospy.loginfo("Increment direction to right")
    elif is_diff_angle_negativ:
      self.dir_value.data = self.dir_value.data - (7*self.CONST_RAD_VALUE/3)
      #rospy.loginfo("Increment direction to left")
    
    self.current_position = int((self.dir_value.data)/self.CONST_RAD_VALUE + self.ZERO_POSITION_VALUE)
    self.current_angle = self.dir_value.data

      
  def dir_callback(self, data_received):
    '''
    Callback from dir controller remote topic
    ''' 
    self._orientation_dir = data_received.range
    self._activate_dir = data_received.activate
    self.control_position_remote()
    
  def control_position_remote(self):
    '''
    Control dir position with remote
    '''
    if self.enable_joy:
      if self._activate_dir:
        current_angle_analog = self.current_angle / self.CONST_RAD_VALUE
        if (self._orientation_dir > 0 ) and (current_angle_analog >= self.RIGHT_LIMIT_VALUE):
          self.dir_value.data = self.dir_value.data - (7*self.CONST_RAD_VALUE/3) + self.value_offset
            
        elif (self._orientation_dir < 0) and (current_angle_analog <= self.LEFT_LIMIT_VALUE):
          self.dir_value.data = self.dir_value.data + (7*self.CONST_RAD_VALUE/3) + self.value_offset
      
      self.current_position = int(-1*(self.dir_value.data-self.value_offset)/self.CONST_RAD_VALUE + self.ZERO_POSITION_VALUE)
      self.current_angle = self.dir_value.data
      self.current_enable = self._activate_dir 
    
  def publish_position(self):
    '''
    Send msg to direction 
    '''
    if not self.enable_joy:
      self.direction_to_angle()
    self._pub_dir_left.publish(self.dir_value)
    self._pub_dir_right.publish(self.dir_value)
    