#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import MagneticField, Imu
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import statistics
import time

class Bno08xDriver():
  def __init__(self):
    # Get params
    ns='/bno08x_driver'
    topic_imu = rospy.get_param(ns+'/imu_topic', 'puma/sensors/imu/raw')
    topic_mag = rospy.get_param(ns+'/magnetic_topic', 'puma/sensors/imu/magnetic')
    topic_diagnostic = rospy.get_param(ns+'/diagnostic_topic', 'puma/sensors/imu/diagnostic')
    self.frame = rospy.get_param(ns+'/frame_imu','imu_link')
    
    # Publishers
    self.imu_pub = rospy.Publisher(topic_imu, Imu, queue_size=10)
    self.mag_pub = rospy.Publisher(topic_mag, MagneticField, queue_size=10)
    self.diagnostic_pub = rospy.Publisher(topic_diagnostic, DiagnosticStatus, queue_size=10)
    
    i2c = busio.I2C(board.SCL_1, board.SDA_1) 
    self.bno = BNO08X_I2C(i2c, address=0x4b) # BNO080 (0x4b) BNO085 (0x4a)
    # Init calibration
    #self.bno.begin_calibration()
    # Activate feature
    self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
    self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    
  def measurement_sensor(self):
    '''
    Measurement data of BNO08X
    '''
    imu_msg = Imu()
    mag_msg = MagneticField()
    diagnostic_msg = DiagnosticStatus()
    key_value = KeyValue()
    total_error = 0 
      
    # --- IMU RAW DATA --- #
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = self.frame
    
    key_value.key = "acceleration linear"
    try:
      accel_x, accel_y, accel_z = self.bno.linear_acceleration
      imu_msg.linear_acceleration.x = accel_x
      imu_msg.linear_acceleration.y = accel_y
      imu_msg.linear_acceleration.z = accel_z
      key_value.value = "works"
    except:
      rospy.logwarn_once("Error al obtener las acceleraciones lineales")
      key_value.value = "not works"
      total_error += 1
    diagnostic_msg.values.append(key_value)
    
    key_value.key = "gyroscope"
    try:
      gyro_x, gyro_y, gyro_z = self.bno.gyro
      imu_msg.angular_velocity.x = gyro_x
      imu_msg.angular_velocity.y = gyro_y
      imu_msg.angular_velocity.z = gyro_z
      key_value.value = "works"
    except:
      rospy.logwarn_once("Error al obtener valores del gyroscopio")
      key_value.value = "not works"
      total_error += 1
    diagnostic_msg.values.append(key_value)
    
    key_value.key = "orientation"
    try:
      quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
      imu_msg.orientation.w = quat_i
      imu_msg.orientation.x = quat_j
      imu_msg.orientation.y = quat_k
      imu_msg.orientation.z = quat_real
      key_value.value = "works"
    except:
      rospy.logwarn_once("Error al obtener los quaterniones")
      key_value.value = "not works"
      total_error += 1
    diagnostic_msg.values.append(key_value)
    
    imu_msg.orientation_covariance[0] = 0
    imu_msg.linear_acceleration_covariance[0] = 0
    imu_msg.angular_velocity_covariance[0] = 0
    
    # --- Magnetic data --- #
    key_value.key = "magnetic"
    try:
      mag_x, mag_y, mag_z = self.bno.magnetic
      mag_msg.header.stamp = rospy.Time.now()
      mag_msg.header.frame_id = self.frame
      mag_msg.magnetic_field.x = mag_x
      mag_msg.magnetic_field.y = mag_y
      mag_msg.magnetic_field.z = mag_z
      mag_msg.magnetic_field_covariance[0] = 0
      key_value.value = "works"
    except:
      rospy.logwarn_once("Error al obtener el valor del magnetometro")
      key_value.value = "not works"
      total_error += 1
    diagnostic_msg.values.append(key_value)
    
    # --- Diagnostic data --- #
    diagnostic_msg.level = 0
    diagnostic_msg.name = "bno08x IMU"
    diagnostic_msg.message = "Imu is running"
    
    if total_error >= 4:
      # --- Diagnostic data --- #
      diagnostic_msg.level = 2
      diagnostic_msg.name = "bno08x IMU"
      diagnostic_msg.message = "Imu doesn't work"
    
    self.publish_data(imu_msg, mag_msg, diagnostic_msg)
      
  def publish_data(self, imu_msg, mag_msg, diagnostic_msg):
    self.imu_pub.publish(imu_msg)
    self.mag_pub.publish(mag_msg)
    self.diagnostic_pub.publish(diagnostic_msg)