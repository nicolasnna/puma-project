#!/usr/bin/env python3
import rospy
from puma_bringup_jetson.node_supervisor_launch import NodeSupervisorLaunch
from puma_msgs.msg import StatusArduino
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
import rospkg

if __name__ == '__main__':
  rospy.init_node('supervisor_manager_node')
  
  path = rospkg.RosPack().get_path('puma_bringup_jetson')
  
  nodes_to_superviser = [
    {
      'node_name': 'arduino_mega',
      'launch_file': f"{path}/launch/core/arduino.launch",
      'monitor_topic': '/puma/arduino/status',
      'type_topic': StatusArduino,
      'timeout': 4,
      'times_respawn': 3
    },
    {
      'node_name': 'puma_control_mode',
      'launch_file': f"{path}/launch/core/control_mode.launch",
      'monitor_topic': '/puma/control/current_mode',
      'type_topic': String,
      'timeout': 3,
      'times_respawn': 2
    },
    {
      'node_name': 'puma_joy',
      'launch_file': f"{path}/launch/core/joy.launch",
    },
    {
      'node_name': 'puma_reverse',
      'launch_file': f"{path}/launch/core/reverse.launch",
    },
    {
      'node_name': 'puma_parking',
      'launch_file': f"{path}/launch/core/parking.launch",
    },
    {
      'node_name': 'puma_ublox',
      'launch_file': f"{path}/launch/core/gps_ublox.launch",
      'monitor_topic': '/puma/sensors/gps/fix',
      'type_topic': NavSatFix,
      'timeout': 4,
      'times_respawn': 2
    },
    {
      'node_name': 'puma_controller',
      'launch_file': f"{path}/launch/core/controller.launch",
    },
    {
      'node_name': 'jetson_system_monitor',
      'launch_file': f"{path}/launch/core/jetson_monitor.launch",
      'monitor_topic': '/puma/jetson/cpu_usage',
      'type_topic': Float32,
      'timeout': 5,
      'times_respawn': 2
    },
    {
      'node_name': 'puma_realsense_rear',
      'launch_file': f"{path}/launch/core/realsense.launch",
    },
    {
      'node_name': 'puma_odom_visual_rear',
      'launch_file': f"{path}/launch/core/odom_rtabmap.launch",
    },
    {
      'node_name': 'filter_imu_realsense_rear',
      'launch_file': f"{path}/launch/core/filter_imu_realsense.launch",
    }
  ]

  supervisors = []
  for node in nodes_to_superviser:
    if 'monitor_topic' in node:
      supervisor = NodeSupervisorLaunch(
        node_name=node['node_name'],
        launch_file=node['launch_file'],
        monitor_topic=node['monitor_topic'],
        type_topic=node['type_topic'],
        timeout=node['timeout'],
        times_respawn=node['times_respawn']
      )
    else:
      supervisor = NodeSupervisorLaunch(
        node_name=node['node_name'],
        launch_file=node['launch_file']
      )
    supervisors.append(supervisor)

  try:
    rospy.spin()
  except:
    for supervisor in supervisors:
      supervisor.stop_supervisor()
    