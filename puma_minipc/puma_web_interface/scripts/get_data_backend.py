#!/usr/bin/env python3
import rospy
import requests
from puma_web_interface.translate_command import translate_command
from puma_web_interface.utils import *
import json
from datetime import datetime

def get_remain_command_robot():
  global headers, BACKEND_URL
  try:
    response = requests.get(BACKEND_URL+"/robot/command_robot", headers=headers, timeout=5)
    if response.status_code == 200:
      return response
    else:
      rospy.logwarn(f"get_data_backebd -> Error al obtener datos: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"get_data_backebd -> Error al obtener datos: {e}")
    
def update_complete_command(body):
  global headers, BACKEND_URL
  try:
    response = requests.post(BACKEND_URL+f"/db/command_robot/complete", headers=headers, data=json.dumps(body),timeout=5)
    if response.status_code == 200:
      return response
    else:
      rospy.logwarn(f"get_data_backebd -> Error al actualizar datos: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"get_data_backebd -> Error al actualizar datos: {e}")
    
def check_and_send_remain_commands():
  global completed_commands, intial_configuration
  res = get_remain_command_robot()
  if not intial_configuration:
    intial_configuration = True
    return  # update_complete_command(cmd)
  if res:
    try:
      res_cmd = res.json()
      
      if isinstance(res_cmd, list) and res_cmd:  # Si es una lista y no está vacía
        for cmd in res_cmd:
          if cmd not in completed_commands:
            # time = datetime.fromisoformat(cmd['updated_at'])
            # if time.tzinfo is None:
            #   time = time.replace(tzinfo=ZoneInfo("Chile/Continental"))
            # diff_time = time_chile_now() - time
            # rospy.loginfo(f"comando {cmd} enviado hace {diff_time} sgs")
            # rospy.loginfo(f"Tiempo actual {time_chile_now()}")
            
            # if diff_time.total_seconds() < 5*60:
            try:
              if translate_command[cmd['type']](cmd['cmd']):
                rospy.loginfo(f"Comando {cmd['type']} enviado")
              else:
                rospy.logwarn(f"Error al enviar comando {cmd['type']}")
            except Exception as e:
              rospy.logwarn(f"Error al enviar comando {cmd['type']}: {e}")
            completed_commands.append(cmd)

              
    except ValueError:
      rospy.logwarn("get_data_backebd -> Error: Response content is not valid JSON")
      
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
      while not token:
        rospy.loginfo("Token no encontrado, esperando 3 segundos")
        rospy.sleep(3)
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
      rospy.logwarn(f"get_data_backebd -> Error: {e}")