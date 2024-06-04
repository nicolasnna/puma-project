#!/usr/bin/env python3
import rospy
from puma_arduino_msgs.msg import StatusTachometer
from puma_odometry.kalman_filter import KalmanFilter
import math

class PulseToVelocityConverter():
  '''
  Pulse of tachometer to lineal velocity converter
  '''
  def __init__(self):
    rospy.Subscriber('puma/sensors/tacometer', StatusTachometer, self._tachometer_callback)
    
    # Get params for calibrate
    _calibrate_max_rpm = rospy.get_param('calibrate_max_rpm_motor', 1654) #
    _calibrate_max_velocity = rospy.get_param('calibrate_max_velocity', 22)  # Max velocity in mph
    self._wheels_diameter = rospy.get_param('wheels_diameter', 0.53) # in meters
    self._kalman_q_noise = rospy.get_param('kalman_q_noise', 7e-4)
    self._kalman_r_noise = rospy.get_param('kalman_r_noise', 0.001)
    # Init kalman filter
    self.filter = KalmanFilter(self._kalman_q_noise, self._kalman_r_noise)
    
    # Calculate
    CONSTANT_METER_TO_INCH = 39.3701
    _calibrate_rpm_wheel_max = _calibrate_max_velocity / (self._wheels_diameter * CONSTANT_METER_TO_INCH * math.pi * 60 / 63360)
    
    self.transmission_ratio = round(_calibrate_max_rpm/ _calibrate_rpm_wheel_max, 2)
    self.rpm_wheels = 0
    self.lineal_velocity = 0

  def _tachometer_callback(self, data_received):
    '''
    Callback from tachometer
    '''
    pulses = data_received.pulsos
    time = data_received.time_millis / 1000 # Convert to seconds
    
    pulses_filtered = self.filter.filtrar(pulses)
    self.rpm_wheels = (pulses_filtered * 60 / time) / self.transmission_ratio
    self.lineal_velocity = round(self.rpm_wheels * self._wheels_diameter * math.pi / 60, 2)
    
  def get_lineal_velocity(self):
    '''
    Return lineal velocity
    '''
    return self.lineal_velocity