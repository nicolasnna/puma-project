#!/usr/bin/env python
import rospy
from puma_arduino_msgs.msg import StatusTachometer
from std_msgs.msg import Float64
import math 

class TachometerController():
  '''
  Tachometer simulator
  '''
  def __init__(self):
    # Rospy
    self.tachometer_pub = rospy.Publisher("puma/sensors/tachometer", StatusTachometer, queue_size=10)
    rospy.Subscriber("/wheel_right_controller/command", Float64, self._velocity_callback)
    
    self.limit_time = rospy.get_param('time_tachometer', 1000)
    self.wheels_diameter = rospy.get_param('wheels_diameter', 0.53)
    calibrate_max_rpm = rospy.get_param('calibrate_max_rpm', 1654)
    calibrate_max_velocity = rospy.get_param('calibrate_max_velocity', 22)
    
    self.pulses = 0
    
    # Calculate
    CONSTANT_METER_TO_INCH = 39.370
    calibrate_rpm_wheel_max = calibrate_max_velocity / (self.wheels_diameter * CONSTANT_METER_TO_INCH * math.pi * 60 / 63360)
    
    self.transmission_ratio = round(calibrate_max_rpm/ calibrate_rpm_wheel_max, 2)
    
    self.last_time = rospy.Time.now()
    
  def _velocity_callback(self, data_received):
    '''
    Velocity Callback, convert linear velocity to pulses
    '''
    velocity = data_received.data
    rpm_wheels = velocity / (self.wheels_diameter * math.pi) * 60
    time = self.limit_time / 1000

    self.pulses = int(rpm_wheels * self.transmission_ratio * time / 60)
    
  def publish_tachometer(self):
    '''
    Create StatusTachometer and send
    '''
    current_time = rospy.Time.now()
    
    tachometer_msgs = StatusTachometer()
    tachometer_msgs.time_millis = self.limit_time
    tachometer_msgs.pulsos = self.pulses
    
    diff_time = current_time - self.last_time
    if diff_time.to_sec() >= (self.limit_time/1000):
      self.tachometer_pub.publish(tachometer_msgs)