#!/usr/bin/env python
import rospy
from icm20948 import ICM20948
from sensor_msgs.msg import Imu, MagneticField
from smbus2 import SMBus
import math

if __name__ == "__main__":
  rospy.init_node("jetson_imu_icm20948")
  imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
  mag_pub = rospy.Publisher('mag', MagneticField, queue_size=10)
  
  imu = ICM20948(i2c_bus=SMBus(0))
  
  while not rospy.is_shutdown():
    mx, my, mz = imu.read_magnetometer_data()
    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
    
    imu_msg = Imu()
    mag_msg = MagneticField()
    imu_msg.header.stamp = mag_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = mag_msg.header.frame_id = "imu_link"
    imu_msg.orientation.x = 0
    imu_msg.orientation.y = 0
    imu_msg.orientation.z = 0
    imu_msg.orientation.w = 1
    imu_msg.angular_velocity.x = gx * math.pi / 180
    imu_msg.angular_velocity.y = gy * math.pi / 180
    imu_msg.angular_velocity.z = gz * math.pi / 180
    imu_msg.linear_acceleration.x = ax * 9.80665
    imu_msg.linear_acceleration.y = ay * 9.80665
    imu_msg.linear_acceleration.z = az * 9.80665
    
    mag_msg.magnetic_field.x = mx * 1e-6
    mag_msg.magnetic_field.y = my * 1e-6
    mag_msg.magnetic_field.z = mz * 1e-6
    
    imu_pub.publish(imu_msg)
    mag_pub.publish(mag_msg)
    rospy.Rate(100).sleep()