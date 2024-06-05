#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int16, Bool
from puma_brake_msgs.msg import BrakeCmd

class WheelController():
  def __init__(self):
    # Publisher
    self._pub_wheel_left = rospy.Publisher('/wheel_left_controller/command', Float64, queue_size=5)  
    self._pub_wheel_right = rospy.Publisher('/wheel_right_controller/command', Float64, queue_size=5)  
    #Subscriber
    rospy.Subscriber('puma/accelerator/command', Int16, self.__accel_callback)
    rospy.Subscriber('puma/reverse/command', Bool, self.__reverse_callback)
    
    rospy.Subscriber('puma/parking/command', Bool, self.__brake_electric_callback)
    rospy.Subscriber('puma/brake/command', BrakeCmd, self.__brake_callback)
    
    # Variable
    # Para delante izq (-) der (+)
    # Para atras izq (+) der (-)
    self.wheel_value = [Float64(), Float64()]
    self._activate_reverse = False
    self._activate_brake_electric = False
    self._activate_brake = False
    
    self.accel_value = 0.0
    
  def __brake_callback(self, data_received):
    '''
    set brake
    '''
    if data_received.position > 500:
      self._activate_brake = True
    else:
      self._activate_brake = False
  
  def __brake_electric_callback(self, data_received):
    '''
    set Brake electrict 
    '''
    self._activate_brake_electric = data_received.data
    
  def __reverse_callback(self, data_received):
    '''
    Set _activate_reverse 
    '''
    self._activate_reverse = data_received.data
    
  def __accel_callback(self, data_received):
    '''
    Value received from interface joy about acceleration
    '''
    self.accel_value = data_received.data
    if not self._activate_brake_electric and not self._activate_brake:
      if self._activate_reverse:
        self.wheel_value[0].data = data_received.data*1.0/10.0
        self.wheel_value[1].data = -data_received.data*1.0/10.0
      else:
        self.wheel_value[0].data = -data_received.data*1.0/10.0
        self.wheel_value[1].data = data_received.data*1.0/10.0
    
    else:
      self.wheel_value[0].data = 0
      self.wheel_value[1].data = 0
  
  def publish_velocity(self):
    '''
    Send msg for control wheels
    '''
    self._pub_wheel_left.publish(self.wheel_value[0])
    self._pub_wheel_right.publish(self.wheel_value[1])
  