#!/usr/bin/env python3
import rospy
import requests
from datetime import datetime
from puma_web_interface.translate_command import translate_command
from puma_web_interface.utils import get_token
import json


def get_remain_command_robot():
  global headers, BACKEND_URL
  try:
    response = requests.get(BACKEND_URL+"/db/command_robot/planned", headers=headers, timeout=5)
    if response.status_code == 200:
      return response
    else:
      rospy.logwarn(f"Error al obtener datos: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Error al obtener datos: {e}")
    
def update_complete_command(body):
  global headers, BACKEND_URL
  try:
    response = requests.post(BACKEND_URL+f"/db/command_robot/complete", headers=headers, data=json.dumps(body),timeout=5)
    if response.status_code == 200:
      return response
    else:
      rospy.logwarn(f"Error al actualizar datos: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Error al actualizar datos: {e}")
    
def check_and_send_remain_commands():
  global completed_commands, intial_configuration
  res = get_remain_command_robot()
  if res and res.content:
    try:
      res_cmd = res.json()
      for cmd in res_cmd:
        if cmd not in completed_commands:
          if not intial_configuration:
            update_complete_command(cmd)
          else:
            time = datetime.fromisoformat(cmd['updated_at'])
            diff_time = datetime.now() - time
            
            # rospy.loginfo(f"comando efectuado hace {diff_time.seconds/60} minutos")
            if diff_time.seconds/60 < 305:
              translate_command[cmd['type']](cmd['cmd'])
              # Process the command here
              update_complete_command(cmd)
              completed_commands.append(cmd)
          intial_configuration = True
    except ValueError:
      rospy.logwarn("Error: Response content is not valid JSON")
      

if __name__ == "__main__":
    rospy.init_node("get_data_backend")
    rospy.loginfo("Node get_data_backend started")
    global BACKEND_URL
    BACKEND_URL = rospy.get_param('~backend_url',"http://localhost:8000")
    rospy.loginfo("Backend: "+BACKEND_URL)
    try:
      global headers, completed_commands, intial_configuration
      intial_configuration = False
      completed_commands = []
      token = get_token(BACKEND_URL)
      
      if token:
        bearer_token = f"Bearer {str(token)}"
        headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token}
      
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
          check_and_send_remain_commands()
          rate.sleep()
      else:
        rospy.logwarn("No se pudo obtener el token de autenticación")
      
    except Exception as e:
      rospy.logwarn(f"Error: {e}")