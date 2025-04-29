#!/usr/bin/env python3
import rospy
import requests
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
          msg_teleop.accel_value = int(res['accel_value'])
          msg_teleop.angle_degree = float(res['angle_degree'])
          msg_teleop.enable_direction = bool(res['enable_direction'])
          msg_teleop.brake = bool(res['brake'])
          msg_teleop.reverse = bool(res['reverse'])
          msg_teleop.parking = bool(res['parking'])
          teleop_pub.publish(msg_teleop)
          # rospy.loginfo(f"Teleop command: {msg_teleop}")
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

  headers = None
  while not headers:
    rospy.loginfo(f"{rospy.get_name()} -> Buscando token en /puma/web/auth_token.")
    try: 
      bearer_token: String = rospy.wait_for_message("/puma/web/auth_token", String, timeout=10)
      headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token.data}
    except Exception as e:
      rospy.logwarn(f"{rospy.get_name()} -> Error al obtener token: {e}")
      time.sleep(10)
  rospy.loginfo(f"{rospy.get_name()} -> Token recibido, ejecutando nodo")

  rate = rospy.Rate(20)
  while not rospy.is_shutdown():
    if current_mode == "web":
      check_and_use_teleop_cmd()
    else:
      initial_configuration = True
    rate.sleep()
      