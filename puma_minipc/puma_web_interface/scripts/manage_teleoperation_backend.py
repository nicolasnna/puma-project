#!/usr/bin/env python3
import rospy
import requests
from datetime import datetime
from puma_web_interface.utils import get_token
from std_msgs.msg import String
from puma_msgs.msg import WebTeleop
import time

def get_control_mode(msg):
  global current_mode
  current_mode = msg.data

def check_teleop_cmd():
  global headers, BACKEND_URL
  try:
    response = requests.get(BACKEND_URL+"/robot/teleop_cmd", headers=headers, timeout=3)
    if response.status_code == 200:
      return response.json()
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"manage_teleoperation_backend -> Error al actualizar datos: {e}")


def check_and_use_teleop_cmd():
  global latest_command, initial_configuration
  try:
    res = check_teleop_cmd()
    if res:
      if initial_configuration:
        latest_command = res
        initial_configuration = False
      else:
        if latest_command != res:
          msg_teleop = WebTeleop()
          msg_teleop.accel_value = min(int(res['accel_value']), 24)
          msg_teleop.angle_degree = float(res['angle_degree'])
          msg_teleop.brake = bool(res['brake'])
          msg_teleop.reverse = bool(res['reverse'])
          msg_teleop.parking = bool(res['parking'])
          teleop_pub.publish(msg_teleop)
          rospy.loginfo(f"Teleop command: {msg_teleop}")
          latest_command = res
          
  except Exception as e:
    rospy.logwarn(f"Error: {e}")


if __name__ == "__main__":
  rospy.init_node("get_teleop_backend")
  rospy.loginfo(f"Empezando {rospy.get_name()} node")
  global headers, BACKEND_URL, current_mode, latest_command, initial_configuration
  BACKEND_URL = rospy.get_param('~backend_url',"http://localhost:8000")
  initial_configuration = True
  current_mode = ''
  rospy.Subscriber("puma/control/current_mode", String, get_control_mode)
  teleop_pub = rospy.Publisher("puma/web/teleop", WebTeleop, queue_size=3)

  token = None
  while not token:
    rospy.loginfo("Esperando 3 segundos para la solicitud del token de autenticacion.")
    time.sleep(3)
    try: 
      token = get_token(BACKEND_URL)
    except Exception as e:
      rospy.logwarn(f"{rospy.get_name()} -> Error al obtener token: {e}")
  
  bearer_token = f"Bearer {str(token)}"
  headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token}

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    if current_mode == "web":
      check_and_use_teleop_cmd()
    else:
      initial_configuration = True
    rate.sleep()
      