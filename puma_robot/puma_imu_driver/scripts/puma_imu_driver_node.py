#!/usr/bin/env python3
import rospy
from puma_imu_driver.bno08x_driver import Bno08xDriver

if __name__ == "__main__":
  try:
    rospy.init_node('puma_imu')
    imu_driver = Bno08xDriver()
    
    frequency = rospy.get_param('~imu/frequency', 60)
    with_gravity = rospy.get_param('~imu/with_gravity', True)
    
    rospy.loginfo("Iniciando calibración de IMU")
    imu_driver.calibrate_acceleration(with_gravity)
    rospy.loginfo("Acceleracion lineal y velocidad angular calibrados")
    
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
      imu_driver.measurement_sensor()
      rate.sleep()
    
  except:
    rospy.logwarn("Nodo " + rospy.get_caller_id + " se ha desactivado!!")