#!/usr/bin/env python
import rospy
from puma_fake_driver.wheel_controller import WheelController
from puma_fake_driver.direction_controller import DirectionController
from puma_fake_driver.arduino_controller import ArduinoController
from puma_fake_driver.tachometer_controller import TachometerController
from std_msgs.msg import Float32

if __name__ == "__main__":
  try:
    rospy.init_node('puma_gazebo_controller')
    
    wheel_controller = WheelController()
    direction_controller = DirectionController()
    arduino_controlller = ArduinoController()
    tachometer_controller = TachometerController()
    
    battery_pub = rospy.Publisher('/puma/sensors/battery/raw_72v', Float32, queue_size=10)
    
    battery_msg = Float32()
    battery_msg.data = 70.0
    rate = rospy.Rate(30)
      
    while not rospy.is_shutdown():
      # Set evalue status Accel
      pwm_value = wheel_controller.accel_value+43
      voltage = 5.0/256 * (pwm_value)
      arduino_controlller.set_status_accel(pwm_value,voltage)
      # Set evaluate status dir
      dir_position = direction_controller.current_position
      enable_dir = direction_controller.current_enable
      activate_brake = wheel_controller._activate_brake
      arduino_controlller.set_status_brake(activate_brake)
      arduino_controlller.set_status_direction(dir_position,enable_dir,False,False)
      # Send msg
      wheel_controller.publish_velocity()
      direction_controller.publish_position()
      arduino_controlller.send_msg()
      tachometer_controller.publish_tachometer()
      
      battery_pub.publish(battery_msg)
      
      rate.sleep()
      
  except Exception as e:
    rospy.logwarn("Nodo 'puma_gazebo_controller' desactivado!!: %s", e)