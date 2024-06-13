#!/usr/bin/env pyhton3
import rospy
from puma_gps_to_enu.gps_to_enu import GpsToEnu

if __name__ == '__main__':
  rospy.init_node('gps_to_enu_node', anonymous=False)
  try: 
    gps_to_enu = GpsToEnu()
  except Exception as e:
    rospy.logerr("Error al instanciar las clases: ", e)
  
  while not rospy.is_shutdown():
    rospy.spin()
  