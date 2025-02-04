#!/usr/bin/env python3
import rospy
from puma_bringup_jetson.node_supervisor import NodeSupervisor
from puma_msgs.msg import StatusArduino
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix

if __name__ == '__main__':
    rospy.init_node('supervisor_manager_node')
    
    nodes_to_supervise = [
      {
        'node_name': 'arduino_mega',
        'node_command': 'roslaunch puma_bringup_jetson arduino.launch',
        'topic_name': '/puma/arduino/status',
        'type_topic': StatusArduino,
        'timeout': 4,
        'times_respawn': 3
      },
      {
        'node_name': 'puma_control_mode',
        'node_command': 'roslaunch puma_bringup_jetson control_mode.launch',
        'topic_name': '/puma/control/current_mode',
        'type_topic': String,
        'timeout': 2,
        'times_respawn': 5
      },
      {
        'node_name': 'puma_joy',
        'node_command': 'roslaunch puma_bringup_jetson joy.launch',
      },
      {
        'node_name': 'puma_reverse',
        'node_command': 'roslaunch puma_bringup_jetson reverse.launch',
      },
      {
        'node_name': 'puma_parking',
        'node_command': 'roslaunch puma_bringup_jetson parking.launch',
      },
      {
        'node_name': 'puma_ublox',
        'node_command': 'roslaunch puma_bringup_jetson gps_ublox.launch',
        'topic_name': '/puma/sensors/gps/fix',
        'type_topic': NavSatFix,
        'timeout': 4,
        'times_respawn': 5
      },
      {
        'node_name': 'jetson_system_monitor',
        'node_command': 'roslaunch puma_bringup_jetson jetson_monitor.launch',
        'topic_name': '/puma/jetson/cpu_usage',
        'type_topic': Float32,
        'timeout': 10,
        'times_respawn': 2
      },
      {
        'node_name': 'puma_controller',
        'node_command': 'roslaunch puma_bringup_jetson controller.launch',
      }
    ]

    supervisors = []
    for node in nodes_to_supervise:
      if 'topic_name' in node:
        supervisor = NodeSupervisor(
          node_name=node['node_name'],
          node_command=node['node_command'],
          topic_name=node['topic_name'],
          type_topic=node['type_topic'],
          timeout=node['timeout'],
          times_respawn=node['times_respawn']
        )
      else:
        supervisor = NodeSupervisor(
          node_name=node['node_name'],
          node_command=node['node_command']
        )
      supervisors.append(supervisor)

    try:
      rospy.spin()
    except:
      for supervisor in supervisors:
        supervisor.stop_supervisor()
    