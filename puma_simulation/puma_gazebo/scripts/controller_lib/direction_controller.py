#!/usr/bin/env python
import rospy
from control_dir_msgs.msg import dir_data
from std_msgs.msg import Float64
import time
import numpy as np

class DirectionController():
  def __init__(self):
    # Publisher
    self._pub_dir_left = rospy.Publisher('/dir_left_controller/command', Float64, queue_size=5)
    self._pub_dir_right = rospy.Publisher('/dir_right_controller/command', Float64, queue_size=5)
    
    # Subscriber
    rospy.Subscriber('control_dir/dir_data', dir_data, self.__dir_callback)
    
    # Variable
    # Dir is config in the same orientation
    self.dir_value = Float64()
    self.dir_value.data = 0.0
    
    self._activate_dir = False
    self._orientation_dir = 0.0
    self.ZERO_POSITION_VALUE = 392
    self.CONST_RAD_VALUE = 2*np.pi/1024 
    self.RIGHT_LIMIT_VALUE =  (263 - self.ZERO_POSITION_VALUE) * self.CONST_RAD_VALUE
    self.LEFT_LIMIT_VALUE = (521 - self.ZERO_POSITION_VALUE) * self.CONST_RAD_VALUE
    
    self.current_position = 0
    self.current_enable = False
    
  def __dir_callback(self, data_received):
    '''
    Callback from dir controller topic
    '''
    self._orientation_dir = data_received.range
    self._activate_dir = data_received.activate
    self.__control_position()
    
  def __control_position(self):
    '''
    Control dir position
    '''
    if self._activate_dir:
      
      if (self._orientation_dir > 0 ) and (self.dir_value.data >= self.RIGHT_LIMIT_VALUE):
        self.dir_value.data = self.dir_value.data - self.CONST_RAD_VALUE/3
          
      elif (self._orientation_dir < 0) and (self.dir_value.data <= self.LEFT_LIMIT_VALUE):
        self.dir_value.data = self.dir_value.data + self.CONST_RAD_VALUE/3
    
    self.current_position = int(-1*self.dir_value.data/self.CONST_RAD_VALUE + self.ZERO_POSITION_VALUE)
    self.current_enable = self._activate_dir 
    
  def publish_position(self):
    '''
    Send msg to direction 
    '''
    #rospy.loginfo("Valor en radianes de la direccion: %s",self.dir_value.data)
    #rospy.loginfo("limite derecho: %s limite izquierdo: %s", self.RIGHT_LIMIT_VALUE, self.LEFT_LIMIT_VALUE)
    self._pub_dir_left.publish(self.dir_value)
    self._pub_dir_right.publish(self.dir_value)