#!/usr/bin/env python3
import rospy
from puma_imu_driver.bno08x_driver import Bno08xDriver

if __name__ == "__main__":
  try:
    rospy.init_node('puma_imu')
    imu_driver = Bno08xDriver()
    
    rospy.loginfo("Iniciando calibración de IMU")
    imu_driver.calibrate_acceleration_velocity()
    rospy.loginfo("Acceleracion lineal y velocidad angular calibrados")
    
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
      imu_driver.measurement_sensor()
      rate.sleep()
    
  except:
    rospy.logwarn("Nodo " + rospy.get_caller_id + " se ha desactivado!!")