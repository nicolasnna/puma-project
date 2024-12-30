#!/usr/bin/env python
import rospy
from puma_msgs.msg import StatusArduino
from std_msgs.msg import Bool

class ArduinoController():
  def __init__(self):
    
    self._status_pub = rospy.Publisher('puma/arduino/status', StatusArduino, queue_size=5)
    self.status_msg = StatusArduino()
    rospy.Subscriber('simulation/secure_signal', Bool, self.secure_signal_callback)
    
    # default values
    self.status_msg.brake.topic = 'puma/control/brake'
    self.status_msg.brake.activate = False
    
    self.status_msg.direction.topic = 'puma/control/direction'
    self.status_msg.direction.analog_value = 420
    self.status_msg.direction.enable = False
    self.status_msg.direction.is_limit_right = False
    self.status_msg.direction.is_limit_left = False
    
    self.status_msg.accelerator.topic = "puma/control/accelerator"
    self.status_msg.accelerator.pwm = 43
    self.status_msg.accelerator.voltage_out = 0.0
    
    self.status_msg.control.security_signal = False
    
  def secure_signal_callback(self, msg):
    self.status_msg.control.security_signal = msg.data
    
  def set_status_accel(self, pwm_accel, voltage):
    '''
    Set accel puma status
    '''
    self.status_msg.accelerator.pwm = int(pwm_accel)
    self.status_msg.accelerator.voltage_out = float(voltage)
    
  def set_status_direction(self, current_position: int, enable: bool, limit_right:bool, limit_left:bool):
    '''
    Set direction puma status
    '''
    self.status_msg.direction.analog_value = current_position
    self.status_msg.direction.enable = enable
    self.status_msg.direction.is_limit_right = limit_right
    self.status_msg.direction.is_limit_left = limit_left
    
  def set_status_brake(self, activate):
    '''
    Set brake puma status
    '''
    self.status_msg.brake.activate = activate
    
  def send_msg(self):
    '''
    Send status_arduino simulation
    '''
    self._status_pub.publish(self.status_msg)