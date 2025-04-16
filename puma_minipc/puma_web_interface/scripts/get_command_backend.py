#!/usr/bin/env python3
import rospy
import requests
from puma_web_interface.translate_command import translate_command
from puma_web_interface.utils import *
import time
from datetime import datetime
from std_msgs.msg import String

def get_remain_command_robot():
  global headers, BACKEND_URL
  try:
    response = requests.get(BACKEND_URL+"/robot/command_robot", headers=headers, timeout=5)
    if response.status_code == 200:
      return response
    else:
      rospy.logwarn(f"get_data_backebd -> Error al obtener datos: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"get_data_backend -> Error al obtener datos: {e}")
    
def update_complete_command(id):
  global headers, BACKEND_URL
  try:
    response = requests.put(BACKEND_URL+f"/robot/command_robot/{id}", headers=headers,timeout=5)
    if response.status_code == 200:
      return response
    else:
      rospy.logwarn(f"get_data_backend -> Error al actualizar datos: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"get_data_backend -> Error al actualizar datos: {e}")
    
def check_and_send_remain_commands():
  global completed_commands, intial_configuration
  res = get_remain_command_robot()
  ''' Primera lectura marca como completado todos los comandos anteriores del encendido '''
  if not intial_configuration:
    intial_configuration = True
    try: 
      clear_completed_commands_db()
      rospy.loginfo("Comandos de sesiones anteriores limpiados de la base de datos")
    except Exception as e:
      rospy.logwarn(f"Error al marcar comandos anteriores como completados: {e}")
    return  # update_complete_command(cmd)
  ''' Uso normal de los comandos'''
  if res:
    try:
      res_cmd = res.json()
      if isinstance(res_cmd, list) and len(res_cmd)>0:  # Si es una lista y no está vacía
        for cmd in res_cmd:
          # if cmd not in completed_commands:
          ''' Comprobar si el comando fue enviado hace menos de 5 minutos '''
          # rospy.loginfo(cmd)
          time = datetime.fromisoformat(cmd['updated_at'])
          if time.tzinfo is None:
            time = time.replace(tzinfo=ZoneInfo("Chile/Continental"))
          diff_time = time_chile_now() - time
          rospy.loginfo(f"comando {cmd} enviado hace {diff_time} sgs")
          rospy.loginfo(f"Tiempo actual {time_chile_now()}")
          
          if diff_time.total_seconds() < 5*60:
            try:
              if translate_command[cmd['type']](cmd['cmd']):
                rospy.loginfo(f"Comando {cmd['type']} enviado")
                update_complete_command(cmd['id'])
              else:
                rospy.logwarn(f"Error al enviar comando {cmd['type']}")
            except Exception as e:
              rospy.logwarn(f"Error al enviar comando {cmd['type']}: {e}")
              # completed_commands.append(cmd)

              
    except ValueError:
      rospy.logwarn("get_data_backend -> Error: Response content is not valid JSON")
      
def clear_completed_commands_db():
  global headers, BACKEND_URL
  try:
    response = requests.delete(BACKEND_URL+"/robot/command_robot/complete", headers=headers, timeout=5)
    if response.status_code == 200:
      return response
    else:
      rospy.logwarn(f"get_data_backend -> Error al limpiar datos: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"get_data_backend -> Error al limpiar datos: {e}")
      
if __name__ == "__main__":
    rospy.init_node("get_data_backend")
    rospy.loginfo(f"Node {rospy.get_name()} started")
    global BACKEND_URL
    BACKEND_URL = rospy.get_param('~backend_url',"http://localhost:8000")
    rospy.loginfo("Backend: "+BACKEND_URL)
    global headers, completed_commands, intial_configuration
    intial_configuration = False
    completed_commands = []
    
    headers = None
    while not headers:
      rospy.loginfo(f"{rospy.get_name()} -> Buscando token en /puma/web/auth_token.")
      try: 
        bearer_token: String = rospy.wait_for_message("/puma/web/auth_token", String, timeout=10)
        headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token.data}
      except Exception as e:
        rospy.logwarn(f"{rospy.get_name()} -> Error al obtener token: {e}")
    rospy.loginfo(f"{rospy.get_name()} -> Token recibido, ejecutando nodo")
    
    while not rospy.is_shutdown():
      check_and_send_remain_commands()
      time.sleep(3)
