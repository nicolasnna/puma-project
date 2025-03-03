#!/usr/bin/env python3
import rospy
import requests
import json
from sensor_msgs.msg import CompressedImage, NavSatFix
from puma_msgs.msg import StatusArduino
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from smach_msgs.msg import SmachContainerStatus
from puma_web_interface.utils import get_token
import base64
from datetime import datetime



def callback_camera(data: CompressedImage):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["camera_front"]> 1:
    image = base64.b64encode(data.data).decode('utf-8')
    dataToSend= { "data": {"image": image} , "created_at": datetime.now().isoformat()}
    
    try:
      response = requests.post(BACKEND_URL+"/database/latest-data/realsense_front", data=json.dumps(dataToSend), headers=headers, timeout=5)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["camera_front"] = time_now

def callback_camera_rear(data: CompressedImage):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["camera_rear"]> 1:
    image = base64.b64encode(data.data).decode('utf-8')
    dataToSend= { "data": {"image": image} , "created_at": datetime.now().isoformat()}
    
    try:
      response = requests.post(BACKEND_URL+"/database/latest-data/realsense_rear", data=json.dumps(dataToSend), headers=headers, timeout=5)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["camera_rear"] = time_now
    
def callback_gps(data: NavSatFix):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["gps"] > 2:
    data_to_send = {"latitude": data.latitude, "longitude": data.longitude, "altitude": data.altitude}
    msg_to_send = { "data": data_to_send, "created_at": datetime.now().isoformat()}
    try:
      response = requests.post(BACKEND_URL+"/database/latest-data/gps", data=json.dumps(msg_to_send), headers=headers, timeout=5)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["gps"] = time_now

def arduino_status_cb(data: StatusArduino):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["arduino_status"] > 2:
    data_to_send = {
      "brake": data.brake.activate, 
      "steering_degree": data.direction.degree_angle, 
      "steering_radian": data.direction.radian_angle,
      "voltage_accel": data.accelerator.voltage_out,
      "pwm": data.accelerator.pwm,
      "security_signal": data.control.security_signal}
    msg_to_send = { "data": data_to_send, "created_at": datetime.now().isoformat()}
    try:
      response = requests.post(BACKEND_URL+"/database/latest-data/arduino_status", data=json.dumps(msg_to_send), headers=headers, timeout=5)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["arduino_status"] = time_now
    
def mode_control_cb(data: String):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["control_mode"] > 2:
    # rospy.loginfo("Sending data to backend mode")
    data_to_send = {"mode": data.data}
    msg_to_send = { "data": data_to_send, "created_at": datetime.now().isoformat()}
    try:
      response = requests.post(BACKEND_URL+"/database/latest-data/control_mode", data=json.dumps(msg_to_send), headers=headers, timeout=5)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["control_mode"] = time_now
    
def odometry_cb(data: Odometry):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["odometry"] > 1:
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
    msg_to_send = { "data": data_to_send, "created_at": datetime.now().isoformat()}
    try:
      response = requests.post(BACKEND_URL+"/database/latest-data/odometry", data=json.dumps(msg_to_send), headers=headers)
      if response.status_code == 200:
        # rospy.loginfo("Data sent to backend odometry")
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["odometry"] = time_now
    
def state_machine_cb(data: SmachContainerStatus):
  global times, headers, previus_sm, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["state_machine"] > 3:
    if previus_sm == data.active_states[0]:
      return
    data_to_send = {
      "active_state": data.active_states[0],
    }
    msg_to_send = { "data": data_to_send, "created_at": datetime.now().isoformat()}
    try:
      response = requests.post(BACKEND_URL+"/database/latest-data/state_machine", data=json.dumps(msg_to_send), headers=headers)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["state_machine"] = time_now


def setting_up():
  rospy.Subscriber("/puma/sensors/camera_front/color/image_raw/compressed", CompressedImage, callback_camera)
  rospy.Subscriber("/puma/sensors/camera_rear/color/image_raw/compressed", CompressedImage, callback_camera_rear)
  rospy.Subscriber("/puma/sensors/gps/fix", NavSatFix, callback_gps)
  rospy.Subscriber("/puma/arduino/status", StatusArduino, arduino_status_cb)
  rospy.Subscriber("/puma/control/current_mode", String, mode_control_cb)
  rospy.Subscriber("/puma/localization/ekf_odometry", Odometry, odometry_cb)
  rospy.Subscriber("/puma/smach/container_status", SmachContainerStatus, state_machine_cb)
  rospy.spin()

if __name__ == "__main__":
  rospy.init_node("post_data_backend")
  rospy.loginfo("Starting post_data_backend node")
  global BACKEND_URL
  BACKEND_URL = rospy.get_param('~backend_url',"http://localhost:8000")
  
  time_now = rospy.Time.now().to_sec()
  global times, headers, previus_sm
  times = { 
          "camera_front": time_now,
          "camera_rear": time_now,
          "gps": time_now,
          "arduino_status": time_now,
          "control_mode": time_now,
          "odometry": time_now,
          "state_machine": time_now
          }
  previus_sm = ""
  rospy.loginfo("Getting token")
  token = get_token(BACKEND_URL)
  while not token:
    rospy.loginfo("Token no encontrado, esperando 3 segundos")
    rospy.sleep(3)
    token = get_token(BACKEND_URL)
  
  bearer_token = f"Bearer {str(token)}"
  headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token}

  setting_up()