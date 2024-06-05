#!/usr/bin/env python
import rospy
from puma_velocity_lib.puma_velocity_controller import PumaVelocityController
  
try: 
  if __name__ == "__main__":
    
    rospy.init_node('puma_controller')
    
    puma_velocity_control = PumaVelocityController()
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
      puma_velocity_control.control_puma()
      rate.sleep()
      
      
except:
  rospy.logerr("Nodo 'puma_controller' desactivado!!!")