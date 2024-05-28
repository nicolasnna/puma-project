#!/usr/bin/env python
import rospy
from controller_lib.wheel_controller import WheelController
from controller_lib.direction_controller import DirectionController
from controller_lib.status_controller import StatusController

try:
  if __name__ == "__main__":
    rospy.init_node('puma_gazebo_controller')
    
    wheel_controller = WheelController()
    direction_controller = DirectionController()
    status_controller = StatusController()
    
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
      # Set evalue status Accel
      pwm_value = wheel_controller.accel_value
      voltage = 5.0/256 * (pwm_value + 43)
      status_controller.set_status_accel(pwm_value,voltage)
      # Set evaluate status dir
      dir_position = direction_controller.current_position
      enable_dir = direction_controller.current_enable
      status_controller.set_status_direction(dir_position,enable_dir,False,False)
      # Send msg
      wheel_controller.publish_velocity()
      direction_controller.publish_position()
      status_controller.send_msg()

      rate.sleep()
      
except Exception as e:
  rospy.logerr("Nodo 'puma_gazebo_controller' desactivado!!: %s", e)