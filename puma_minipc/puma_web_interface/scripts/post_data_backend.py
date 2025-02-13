#!/usr/bin/env python3
import rospy
import requests
import json
from sensor_msgs.msg import CompressedImage, NavSatFix
from puma_msgs.msg import StatusArduino
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from smach_msgs.msg import SmachContainerStatus
import base64

def get_token():
  global BACKEND_URL
  headers = { 'Content-Type': 'application/x-www-form-urlencoded'}
  body = { 'username': 'admin', 'password': 'admin'}
  try:
    response = requests.post(BACKEND_URL+"/auth/login", headers=headers, data=body, timeout=5)
    if response.status_code == 200:
      return response.json()
    else:
      rospy.logwarn(f"Error al obtener token: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Error al obtener token: {e}")

def callback_camera(data: CompressedImage):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["camera_front"]> 1:
    image = base64.b64encode(data.data).decode('utf-8')
    dataToSend= {"camera": "realsense-front", "data": image}
    # rospy.loginfo("Sending data to backend camera")
    try:
      response = requests.post(BACKEND_URL+"/db/camera", data=json.dumps(dataToSend), headers=headers, timeout=5)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["camera_front"] = time_now
    
def callback_gps(data: NavSatFix):
  global times, headers, BACKEND_URL
  time_now = rospy.Time.now().to_sec()
  if time_now - times["gps"] > 2:
    dataToSend = {"latitude": data.latitude, "longitude": data.longitude, "altitude": data.altitude}
    # rospy.loginfo("Sending data to backend gps")
    try:
      response = requests.post(BACKEND_URL+"/db/gps", data=json.dumps(dataToSend), headers=headers, timeout=5)
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
    dataToSend = {
      "brake": data.brake.activate, 
      "steering_degree": data.direction.degree_angle, 
      "steering_radian": data.direction.radian_angle,
      "voltage_accel": data.accelerator.voltage_out,
      "pwm": data.accelerator.pwm,
      "security_signal": data.control.security_signal}
    # rospy.loginfo("Sending data to backend arduino")
    try:
      response = requests.post(BACKEND_URL+"/db/arduino_status", data=json.dumps(dataToSend), headers=headers, timeout=5)
      if response.status_code == 200:
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["arduino_status"] = time_now
    
def mode_control_cb(data: String):
  global times, headers, BACKEND_URL
  dataToSend = {"mode": data.data}
  time_now = rospy.Time.now().to_sec()
  if time_now - times["control_mode"] > 2:
    # rospy.loginfo("Sending data to backend mode")
    try:
      response = requests.post(BACKEND_URL+"/db/control_mode", data=json.dumps(dataToSend), headers=headers, timeout=5)
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
    dataToSend = {
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
    try:
      response = requests.post(BACKEND_URL+"/db/odometry", data=json.dumps(dataToSend), headers=headers)
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
    dataToSend = {
      "active_state": data.active_states[0],
    }
    try:
      response = requests.post(BACKEND_URL+"/db/state_machine", data=json.dumps(dataToSend), headers=headers)
      if response.status_code == 200:
        rospy.loginfo("Data sent to backend state machine")
        pass
      else:
        rospy.logwarn(f"Error al enviar datos: {response.status_code} - {response.text}")
    except requests.exceptions.RequestException as e:
      rospy.logwarn(f"Error al enviar datos: {e}")
    times["state_machine"] = time_now


def setting_up():
  rospy.Subscriber("/puma/sensors/camera_front/color/image_raw/compressed", CompressedImage, callback_camera)
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
          "gps": time_now,
          "arduino_status": time_now,
          "control_mode": time_now,
          "odometry": time_now,
          "state_machine": time_now
          }
  previus_sm = ""
  rospy.loginfo("Getting token")
  token = get_token()
  bearer_token = f"Bearer {str(token['access_token'])}"
  headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token}

  setting_up()