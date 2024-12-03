#!/usr/bin/env python
import rospy
from puma_msgs.msg import StatusArduino

class ArduinoController():
  def __init__(self):
    
    self._status_pub = rospy.Publisher('puma/arduino/status', StatusArduino, queue_size=5)
    self.status_msg = StatusArduino()
    
    # default values
    self.status_msg.topic_brake = 'puma/brake/command'
    self.status_msg.activate_brake = False
    
    self.status_msg.topic_dir = 'puma/direction/command'
    self.status_msg.current_position_dir = 395
    self.status_msg.enable_dir = False
    self.status_msg.is_limit_right_dir = False
    self.status_msg.is_limit_left_dir = False
    
    self.status_msg.topic_accel = "puma/accelerator/command"
    self.status_msg.pwm_accel = 43
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
    
  def set_status_brake(self, activate):
    '''
    Set brake puma status
    '''
    self.status_msg.activate_brake = activate
    
  def send_msg(self):
    '''
    Send status_arduino simulation
    '''
    self._status_pub.publish(self.status_msg)