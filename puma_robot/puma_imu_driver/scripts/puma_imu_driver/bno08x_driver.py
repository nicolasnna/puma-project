#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import MagneticField, Imu
from diagnostic_msgs.msg import DiagnosticStatus
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

class Bno08xDriver():
  def __init__(self):
    # Publishers
    self.imu_pub = rospy.Publisher('puma/sensors/imu/raw', Imu, queue_size=10)
    self.mag_pub = rospy.Publisher('puma/sensors/imu/magnetic', MagneticField, queue_size=10)
    self.diagnostic_pub = rospy.Publisher('puma/sensors/imu/diagnostic', DiagnosticStatus, queue_size=10)
    self.frame = rospy.get_param('~imu/frame','imu1_link')
    
    i2c = busio.I2C(board.SCL_1, board.SDA_1) 
    self.bno = BNO08X_I2C(i2c, address=0x4b) # BNO080 (0x4b) BNO085 (0x4a)

    self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
    self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    
    self.acceleration_offset = [0.0, 0.0, 0.0]
    
  def calibrate_acceleration(self, with_gravity=True):
    '''
    Calibrate acceleration linear 
    '''
    accel_x_array = []
    accel_y_array = []
    accel_z_array = []
    
    for i in range(0,1000):
      accel_x, accel_y, accel_z = self.bno.acceleration
      accel_x_array.append(accel_x)
      accel_y_array.append(accel_y)
      accel_z_array.append(accel_z if not with_gravity else 0)
         
    self.acceleration_offset = [statistics.mean(accel_x_array), 
                                statistics.mean(accel_y_array), 
                                statistics.mean(accel_z_array)]
    
  def measurement_sensor(self):
    '''
    Measurement data of BNO08X
    '''
    imu_msg = Imu()
    mag_msg = MagneticField()
    diagnostic_msg = DiagnosticStatus()
    try: 
      # --- IMU RAW DATA --- #
      imu_msg.header.stamp = rospy.Time.now()
      imu_msg.header.frame_id = self.frame
      
      accel_x, accel_y, accel_z = self.bno.acceleration
      imu_msg.linear_acceleration.x = accel_x - self.acceleration_offset[0]
      imu_msg.linear_acceleration.y = accel_y - self.acceleration_offset[1]
      imu_msg.linear_acceleration.z = accel_z - self.acceleration_offset[2]
      
      gyro_x, gyro_y, gyro_z = self.bno.gyro
      imu_msg.angular_velocity.x = gyro_x
      imu_msg.angular_velocity.y = gyro_y
      imu_msg.angular_velocity.z = gyro_z
      
      quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
      imu_msg.orientation.w = quat_i
      imu_msg.orientation.x = quat_j
      imu_msg.orientation.y = quat_k
      imu_msg.orientation.z = quat_real
      
      imu_msg.orientation_covariance[0] = 0
      imu_msg.linear_acceleration_covariance[0] = 0
      imu_msg.angular_velocity_covariance[0] = 0
      
      # --- Magnetic data --- #
      mag_x, mag_y, mag_z = self.bno.magnetic
      mag_msg.header.stamp = rospy.Time.now()
      mag_msg.header.frame_id = self.frame
      mag_msg.magnetic_field.x = mag_x
      mag_msg.magnetic_field.y = mag_y
      mag_msg.magnetic_field.z = mag_z
      mag_msg.magnetic_field_covariance[0] = 0
      
      # --- Diagnostic data --- #
      diagnostic_msg.level = 0
      diagnostic_msg.name = "bno08x IMU"
      diagnostic_msg.message = "Imu is running"
    except: 
      # --- Diagnostic data --- #
      diagnostic_msg.level = 2
      diagnostic_msg.name = "bno08x IMU"
      diagnostic_msg.message = "Imu doesn't work"
    finally:
      self.publish_data(imu_msg, mag_msg, diagnostic_msg)
      
  def publish_data(self, imu_msg, mag_msg, diagnostic_msg):
    self.imu_pub.publish(imu_msg)
    self.mag_pub.publish(mag_msg)
    self.diagnostic_pub.publish(diagnostic_msg)