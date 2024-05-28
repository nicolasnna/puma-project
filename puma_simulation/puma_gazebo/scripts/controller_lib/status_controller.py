#!/usr/bin/env python
import rospy
from init_puma.msg import status_arduino

class StatusController():
  def __init__(self):
    
    self._status_pub = rospy.Publisher('status_arduino', status_arduino, queue_size=5)
    self.status_msg = status_arduino()
    
    # default values
    self.status_msg.topic_brake = '/brake_controller/data_control'
    self.status_msg.position_brake = 0
    self.status_msg.is_move_brake = False
    
    self.status_msg.topic_dir = 'control_dir/dir_data'
    self.status_msg.current_position_dir = 394
    self.status_msg.enable_dir = False
    self.status_msg.is_limit_right_dir = False
    self.status_msg.is_limit_left_dir = False
    
    self.status_msg.topic_accel = 'accel_puma/value'
    self.status_msg.pwm_accel = 0
    self.status_msg.voltage_accel = 0.0
    
  def set_status_accel(self, pwm_accel, voltage):
    '''
    Set accel puma status
    '''
    self.status_msg.pwm_accel = int(pwm_accel)
    self.status_msg.voltage_accel = float(voltage)
    
  def set_status_direction(self, current_position: int, enable: bool, limit_right:bool, limit_left:bool):
    '''
    Set direction puma status
    '''
    self.status_msg.current_position_dir = current_position
    self.status_msg.enable_dir = enable
    self.status_msg.is_limit_right_dir = limit_right
    self.status_msg.is_limit_left_dir = limit_left
    
  def set_status_brake(self, position, is_move):
    '''
    Set brake puma status
    '''
    self.status_msg.position_brake = position
    self.status_msg.is_move_brake = is_move
    
  def send_msg(self):
    '''
    Send status_arduino simulation
    '''
    self._status_pub.publish(self.status_msg)