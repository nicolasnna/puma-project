#!/usr/bin/env python3
import rospy
from puma_controller.puma_velocity_controller import PumaVelocityController
  
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