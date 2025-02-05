#!/usr/bin/env python3
import rospy
from puma_bringup.node_supervisor_launch import NodeSupervisorLaunch
import rospkg
from puma_msgs.msg import LogArray

if __name__ == '__main__':
  rospy.init_node('supervisor_minipc_node')
  
  path = rospkg.RosPack().get_path('puma_bringup')
  path_state = rospkg.RosPack().get_path('puma_state_machine')
  
  nodes_to_superviser = [
    {
      'node_name': 'arduino_nano',
      'launch_file': f"{path}/launch/core/arduino.launch",
    },
    {
      'node_name': 'minipc_system_monitor',
      'launch_file': f"{path}/launch/core/minipc_monitor.launch",
    },
    {
      'node_name': 'realsense_front',
      'launch_file': f"{path}/launch/core/realsense_camera.launch",
    },
    {
      'node_name': 'filter_imu_realsense_front',
      'launch_file': f"{path}/launch/core/imu_filter_realsense.launch",
    },
    {
      'node_name': 'odom_visual_front',
      'launch_file': f"{path}/launch/core/visual_odometry.launch",
    },
    {
      'node_name': 'puma_localization',
      'launch_file': f"{path}/launch/core/localization_bringup.launch",
    },
    {
      'node_name': 'puma_navigation',
      'launch_file': f"{path}/launch/core/puma_navigation_bringup.launch",
    },
    {
      'node_name': 'web_interface',
      'launch_file': f"{path}/launch/core/web_interface.launch",
      'monitor_topic': '/puma/logs/logs',
      'type_topic': LogArray,
      'timeout': 10,
      'times_respawn': 3
    },
    {
      'node_name': 'navigation_manager',
      'launch_file': f"{path}/launch/core/nav_manager.launch",
    },
    {
      'node_name': 'state_machine',
      'launch_file': f"{path_state}/launch/state_machine_v2.launch",
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