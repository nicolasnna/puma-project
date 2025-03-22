#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Imu, MagneticField
from puma_msgs.msg import ImuData

A_G = 9.8067
DEG_TO_RAD = 0.017453292519943295

def raw_icm20948_callback(msg: ImuData):
  global imu_pub, mag_pub
  imu_msg = Imu()
  mag_msg = MagneticField()
  imu_msg.header.frame_id = mag_msg.header.frame_id = 'imu_link'
  imu_msg.header.stamp = mag_msg.header.stamp = rospy.Time.now()
  imu_msg.linear_acceleration.x = msg.acc_x_g * A_G
  imu_msg.linear_acceleration.y = msg.acc_y_g * A_G
  imu_msg.linear_acceleration.z = msg.acc_z_g * A_G
  imu_msg.angular_velocity.x = msg.gyro_x_deg * DEG_TO_RAD
  imu_msg.angular_velocity.y = msg.gyro_y_deg * DEG_TO_RAD
  imu_msg.angular_velocity.z = msg.gyro_z_deg * DEG_TO_RAD
  mag_msg.magnetic_field.x = msg.mag_x_uT
  mag_msg.magnetic_field.y = msg.mag_y_uT
  mag_msg.magnetic_field.z = msg.mag_z_uT
  imu_pub.publish(imu_msg)
  mag_pub.publish(mag_msg)
  

def main():
  rospy.init_node('translate_icm20948_arduino')
  rospy.Subscriber('puma/sensors/icm20948/raw', ImuData, raw_icm20948_callback)
  global imu_pub, mag_pub
  imu_pub = rospy.Publisher('puma/sensors/icm20948/imu', Imu, queue_size=10)
  mag_pub = rospy.Publisher('puma/sensors/icm20948/mag', MagneticField, queue_size=10)
  #mag_pub = rospy.Publisher('/mag', MagneticField, queue_size=10)

  rospy.spin()
  
if __name__ == '__main__':
  main()