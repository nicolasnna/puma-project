#!/usr/bin/env python3
import rospy
import requests
import json
from sensor_msgs.msg import CompressedImage, NavSatFix, BatteryState
from puma_msgs.msg import StatusArduino, WaypointNav, Waypoint
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from smach_msgs.msg import SmachContainerStatus
from puma_web_interface.utils import *
import base64
from datetime import datetime

def realsense_front_cb(data: CompressedImage):
  global realsense_front
  image = base64.b64encode(data.data).decode('utf-8')
  realsense_front = { "data": {"image": image} }
    
def realsense_rear_cb(data: CompressedImage):
  global realsense_rear
  image = base64.b64encode(data.data).decode('utf-8')
  realsense_rear = { "data": {"image": image} }
    
def gps_cb(data: NavSatFix):
  global gps
  data_to_send = {"latitude": data.latitude, "longitude": data.longitude, "altitude": data.altitude}
  gps = { "data": data_to_send }

def arduino_status_cb(data: StatusArduino):
  global arduino_status
  data_to_send = {
    "brake": data.brake.activate, 
    "steering_degree": data.direction.degree_angle, 
    "steering_radian": data.direction.radian_angle,
    "voltage_accel": data.accelerator.voltage_out,
    "pwm": data.accelerator.pwm,
    "security_signal": data.control.security_signal}
  arduino_status = { "data": data_to_send }
    
def mode_control_cb(data: String):
  global control_mode
  data_to_send = {"mode": data.data}
  control_mode = { "data": data_to_send }
    
def odometry_cb(data: Odometry):
  global odometry
  data_to_send = {
    "pos_x": data.pose.pose.position.x,
    "pos_y": data.pose.pose.position.y,
    "pos_z": data.pose.pose.position.z,
    "qua_x": data.pose.pose.orientation.x,
    "qua_y": data.pose.pose.orientation.y,
    "qua_z": data.pose.pose.orientation.z,
    "qua_w": data.pose.pose.orientation.w,
    "linear_x": data.twist.twist.linear.x,
    "angular_z": data.twist.twist.angular.z
  }
  odometry = { "data": data_to_send, "created_at": datetime.now().isoformat()}
    
def state_machine_cb(data: SmachContainerStatus):
  global state_machine
  data_to_send = {"active_state": data.active_states[0]}
  state_machine = { "data": data_to_send }
    
def battery_cb(data: BatteryState):
  global battery
  battery = {"data": {"voltage": data.voltage, "percentage": data.percentage}}
  
def waypoint_to_dict(waypoint: Waypoint):
  return {
    "x": waypoint.x,
    "y": waypoint.y,
    "yaw": waypoint.yaw,
    "latitude": waypoint.latitude,
    "longitude": waypoint.longitude,
  }
  
def wp_completed_cb(data: WaypointNav):
  global wp_completed
  waypoints = [waypoint_to_dict(wp) for wp in data.waypoints]
  wp_completed = {"data": {"waypoints": waypoints}}

def wp_remained_cb(data: WaypointNav):
  global wp_remained
  waypoints = [waypoint_to_dict(wp) for wp in data.waypoints]
  wp_remained = {"data": {"waypoints": waypoints}}

def wp_list_cb(data: WaypointNav):
  global wp_list
  waypoints = [waypoint_to_dict(wp) for wp in data.waypoints]
  wp_list = {"data": {"waypoints": waypoints}}
    
def setting_up():
  rospy.Subscriber("/puma/sensors/camera_front/color/image_raw/compressed", CompressedImage, realsense_front_cb)
  rospy.Subscriber("/puma/sensors/camera_rear/color/image_raw/compressed", CompressedImage, realsense_rear_cb)
  rospy.Subscriber("/puma/sensors/gps/fix", NavSatFix, gps_cb)
  rospy.Subscriber("/puma/arduino/status", StatusArduino, arduino_status_cb)
  rospy.Subscriber("/puma/control/current_mode", String, mode_control_cb)
  rospy.Subscriber("/puma/localization/ekf_odometry", Odometry, odometry_cb)
  rospy.Subscriber("/puma/smach/container_status", SmachContainerStatus, state_machine_cb)
  rospy.Subscriber("/puma/sensors/battery/status", BatteryState, battery_cb)
  rospy.Subscriber("/puma/navigation/waypoints_completed", WaypointNav, wp_completed_cb )
  rospy.Subscriber("/puma/navigation/waypoints_remained", WaypointNav, wp_remained_cb)
  rospy.Subscriber("/puma/navigation/waypoints_list", WaypointNav, wp_list_cb)
  
  
def get_msg_to_send():
  global realsense_front, realsense_rear, gps, arduino_status, control_mode, odometry, state_machine, battery, wp_completed, wp_remained, wp_list
  
  data = {}
  
  if realsense_front:
    data["realsense_front"] = realsense_front
  if realsense_rear:
    data["realsense_rear"] = realsense_rear
  if gps:
    data["gps"] = gps
  if arduino_status:
    data["arduino_status"] = arduino_status
  if control_mode:
    data["control_mode"] = control_mode
  if odometry:
    data["odometry"] = odometry
  if state_machine:
    data["state_machine"] = state_machine
  if battery:
    data["battery"] = battery
  if wp_completed:
    data["waypoint_completed"] = wp_completed
  if wp_remained:
    data["waypoint_remained"] = wp_remained
  if wp_list:
    data["waypoint_list"] = wp_list
  
  return {"data": data}


def run_send():
  global headers, BACKEND_URL
  msg_to_send = get_msg_to_send()
  
  try: 
    res = requests.post(BACKEND_URL+"/robot/status", data=json.dumps(msg_to_send), headers=headers)
    if res.status_code != 200:
      rospy.logwarn_throttle(10, f"Error al enviar datos: {res.status_code} - {res.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn_throttle(10, f"Error al enviar datos: {e}")

if __name__ == "__main__":
  rospy.init_node("send_robot_status_backend")
  rospy.loginfo(f"Empezando {rospy.get_name()} node")
  global BACKEND_URL, headers
  BACKEND_URL = rospy.get_param('~backend_url',"http://localhost:8000")
  # Creación de variables globales
  global realsense_front, realsense_rear, gps, arduino_status, control_mode, odometry, state_machine, battery, wp_completed, wp_remained, wp_list
  # Inicialización de variables
  realsense_front = realsense_rear = None 
  gps = arduino_status = control_mode = odometry = state_machine = battery = None
  wp_completed = wp_remained = wp_list = None
  
  rospy.loginfo("Obteniendo token")
  try: 
    token = get_token(BACKEND_URL)
  except Exception as e:
    rospy.logwarn(f"{rospy.get_name()} -> Error al obtener token: {e}")
  while not token:
    rospy.loginfo(f"{rospy.get_name()} - Token no encontrado, esperando 3 segundos")
    rospy.sleep(3)
    token = get_token(BACKEND_URL)
  
  bearer_token = f"Bearer {str(token)}"
  headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token}

  setting_up()
  
  while not rospy.is_shutdown():
    run_send()
    rospy.Rate(2).sleep()