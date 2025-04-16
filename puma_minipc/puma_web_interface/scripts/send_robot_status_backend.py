#!/usr/bin/env python3
import rospy
import requests
import json
from sensor_msgs.msg import CompressedImage, NavSatFix, BatteryState
from puma_msgs.msg import StatusArduino, WaypointNav, StatusArduinoRelay
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from smach_msgs.msg import SmachContainerStatus
from puma_web_interface.utils import *
import base64
from datetime import datetime
import time

DATA_TO_SEND = {
  "realsense_front": None,
  "realsense_rear": None, 
  "gps": None, 
  "arduino_status": None, 
  "control_mode": None, 
  "odometry": None, 
  "state_machine": None, 
  "battery": None, 
  "waypoint_completed": None, 
  "waypoint_remained": None, 
  "waypoint_list": None, 
  "arduino_relay": None,
  "nvr_cam1": None,
  "nvr_cam2": None,
  "nvr_cam3": None,
  "realsense_front_depth": None,
  "realsense_rear_depth": None,
}

def realsense_front_cb(data: CompressedImage):
  image = base64.b64encode(data.data).decode('utf-8')
  DATA_TO_SEND["realsense_front"] = { "data": {"image": image} }
    
def realsense_rear_cb(data: CompressedImage):
  image = base64.b64encode(data.data).decode('utf-8')
  DATA_TO_SEND["realsense_rear"] = { "data": {"image": image} }
    
def gps_cb(data: NavSatFix):
  gps = {"latitude": data.latitude, "longitude": data.longitude, "altitude": data.altitude}
  DATA_TO_SEND["gps"] = { "data": gps }

def arduino_status_cb(data: StatusArduino):
  status = {
    "brake": data.brake.activate, 
    "steering_degree": data.direction.degree_angle, 
    "steering_radian": data.direction.radian_angle,
    "voltage_accel": data.accelerator.voltage_out,
    "pwm": data.accelerator.pwm,
    "security_signal": data.control.security_signal}
  DATA_TO_SEND["arduino_status"] = { "data": status }
    
def mode_control_cb(data: String):
  control = {"mode": data.data}
  DATA_TO_SEND["control_mode"] = { "data": control }
    
def odometry_cb(data: Odometry):
  odometry = {
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
  DATA_TO_SEND["odometry"] = { "data": odometry, "created_at": datetime.now().isoformat()}
    
def state_machine_cb(data: SmachContainerStatus):
  state_machine = {"active_state": data.active_states[0]}
  DATA_TO_SEND["state_machine"] = { "data": state_machine }
    
def battery_cb(data: BatteryState):
  DATA_TO_SEND["battery"] = {"data": {"voltage": data.voltage, "percentage": data.percentage}}
  
def wp_completed_cb(data: WaypointNav):
  waypoints = [waypoint_to_dict(wp) for wp in data.waypoints]
  DATA_TO_SEND["waypoint_completed"] = {"data": {"waypoints": waypoints}}

def wp_remained_cb(data: WaypointNav):
  waypoints = [waypoint_to_dict(wp) for wp in data.waypoints]
  DATA_TO_SEND["waypoint_remained"] = {"data": {"waypoints": waypoints}}

def wp_list_cb(data: WaypointNav):
  waypoints = [waypoint_to_dict(wp) for wp in data.waypoints]
  DATA_TO_SEND["waypoint_list"] = {"data": {"waypoints": waypoints}}

def arduino_relay_cb(data: StatusArduinoRelay):
  data_send = {
    "charge_connection": data.charge_connection,
    "front_lights": data.lights_front
  }
  DATA_TO_SEND["arduino_relay"] = {"data": data_send}
  
def nvr_camera0_cb(data: CompressedImage):
  image = base64.b64encode(data.data).decode('utf-8')
  DATA_TO_SEND["nvr_cam1"] = { "data": {"image": image} }
  
def nvr_camera1_cb(data: CompressedImage):
  image = base64.b64encode(data.data).decode('utf-8')
  DATA_TO_SEND["nvr_cam2"] = { "data": {"image": image} }
  
def nvr_camera2_cb(data: CompressedImage):
  image = base64.b64encode(data.data).decode('utf-8')
  DATA_TO_SEND["nvr_cam3"] = { "data": {"image": image} }
  
def realsense_front_depth_cb(data: CompressedImage):
  image = base64.b64encode(data.data).decode('utf-8')
  DATA_TO_SEND["realsense_front_depth"] = { "data": {"image": image} }
  
def realsense_rear_depth_cb(data: CompressedImage):
  image = base64.b64encode(data.data).decode('utf-8')
  DATA_TO_SEND["realsense_rear_depth"] = { "data": {"image": image} }
    
def setting_up():
  topics_config = [
    {"topic": "/puma/sensors/camera_front/color/image_raw/compressed", "type": CompressedImage, "cb" : realsense_front_cb},
    {"topic": "/puma/sensors/camera_rear/color/image_raw/compressed", "type": CompressedImage, "cb" : realsense_rear_cb},
    {"topic": "/puma/sensors/gps/fix", "type": NavSatFix, "cb" : gps_cb},
    {"topic": "/puma/arduino/status", "type": StatusArduino, "cb" : arduino_status_cb},
    {"topic": "/puma/control/current_mode", "type": String, "cb" : mode_control_cb},
    {"topic": "/puma/localization/ekf_odometry", "type": Odometry, "cb" : odometry_cb},
    {"topic": "/puma/smach/container_status", "type": SmachContainerStatus, "cb" : state_machine_cb},
    {"topic": "/puma/sensors/battery/status", "type": BatteryState, "cb" : battery_cb},
    {"topic": "/puma/navigation/waypoints_completed", "type": WaypointNav, "cb" : wp_completed_cb },
    {"topic": "/puma/navigation/waypoints_remained", "type": WaypointNav, "cb" : wp_remained_cb},
    {"topic": "/puma/navigation/waypoints_list", "type": WaypointNav, "cb" : wp_list_cb},
    {"topic": "/puma/arduino/status_relay", "type": StatusArduinoRelay, "cb" : arduino_relay_cb},
    {"topic": "/puma/nvr/camera0/image_raw/compressed", "type": CompressedImage, "cb": nvr_camera0_cb},
    {"topic": "/puma/nvr/camera1/image_raw/compressed", "type": CompressedImage, "cb": nvr_camera1_cb},
    {"topic": "/puma/nvr/camera2/image_raw/compressed", "type": CompressedImage, "cb": nvr_camera2_cb},
    {"topic": "/puma/sensors/camera_front/depth/image_rect_raw/compressed", "type": CompressedImage, "cb": realsense_front_depth_cb},
    {"topic": "/puma/sensors/camera_rear/depth/image_rect_raw/compressed", "type": CompressedImage, "cb": realsense_rear_depth_cb},
  ]
  
  for topic in topics_config:
    rospy.Subscriber(topic["topic"], topic["type"], topic["cb"])
  
def get_msg_to_send():
  data = {}
  
  for key, value in DATA_TO_SEND.items():
    if value is not None:
      data[key] = value
  
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
  
  headers = None
  while not headers:
    rospy.loginfo(f"{rospy.get_name()} -> Buscando token en /puma/web/auth_token.")
    try: 
      bearer_token: String = rospy.wait_for_message("/puma/web/auth_token", String, timeout=10)
      headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token.data}
    except Exception as e:
      rospy.logwarn(f"{rospy.get_name()} -> Error al obtener token: {e}")
  rospy.loginfo(f"{rospy.get_name()} -> Token recibido, ejecutando nodo")

  setting_up()
  
  while not rospy.is_shutdown():
    run_send()
    time.sleep(1)