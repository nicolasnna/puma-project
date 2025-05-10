#!/usr/bin/env python3
import rospy
import actionlib
import requests
from puma_msgs.msg import Log
from puma_ip_devices.msg import SpeakerManagerAction, SpeakerManagerGoal, SpeakerManagerResult
from puma_ip_devices.utils import send_log_message

def send_to_speaker(url):
  try:
    rs = requests.get(url, timeout=5)
    if rs.status_code == 200:
      send_log_message(f"Se ha realizado la petición al parlante correctamente.")
      return True
  except requests.exceptions.RequestException as e:
    send_log_message(f"Error al obtener datos al realizar peticion al parlante: {e}")
  return False

def play_speaker(goal: SpeakerManagerGoal):
  ip = rospy.get_param('~ip_speaker', '10.42.0.95')
  url = f"http://{ip}/api/play?action=start"
  result = SpeakerManagerResult()
  if not goal.filename:
    result.message = 'Error: Es necesario indicar el nombre del archivo'
    result.success = False
    return result
  
  url = url + f"&file={goal.filename}"
  
  if goal.mode == SpeakerManagerGoal.MODE_PLAY_REPEAT:
    url = url + f"&mode={goal.mode}"
    if not goal.repeats or goal.repeats == 0:
      result.message = f'Error: Para el modo {goal.mode} requiere indicar repeticiones mayores a 0'
      result.success = False
      return result
    url = url + f"&count={goal.repeats}"
    
  elif goal.mode == SpeakerManagerGoal.MODE_PLAY_TIME:
    url = url + f"&mode={goal.mode}"
    if not goal.time_seconds or goal.time_seconds == 0:
      result.message = f'Error: Para el modo {goal.mode} requiere indicar el tiempo de sonido a 0'
      result.success = False
      return result
    url = url + f"&count={goal.time_seconds}"
  
  if goal.volume and goal.volume >= 1 and goal.volume <= 100:
    url = url + f"&volume={goal.volume}"
  
  is_work = send_to_speaker(url)
  
  result.success = is_work
  result.message = "Se ha realizado la petición al parlante correctamente" if is_work else "No se ha logrado realizar la petición al parlante"
  return result

def stop_speaker():
  ip = rospy.get_param('~ip_speaker', '10.42.0.95')
  url = f"http://{ip}/api/play?action=stop"
  result = SpeakerManagerResult()
  is_work = send_to_speaker(url)
  
  result.success = is_work
  result.message = "Se ha realizado la petición de parada al parlante" if is_work else "No se ha logrado realizar la petición al parlante"
  return result

def speaker_srv_cb(goal: SpeakerManagerGoal):
  result = SpeakerManagerResult()
  
  if goal.action == SpeakerManagerGoal.ACTION_PLAY:
    result = play_speaker()
  elif goal.action == SpeakerManagerGoal.ACTION_STOP:
    result = stop_speaker()
  else:
    result.success = False
    result.message = "Comando 'action' no soportado"
    srv.set_aborted(result)
    return False
  
  srv.set_succeeded(result)

def main():
  rospy.init_node('speaker_manager_node')
  global srv
  srv = actionlib.SimpleActionServer('puma/speaker', SpeakerManagerAction, execute_cb=speaker_srv_cb,auto_start=False)
  srv.start()
  send_log_message(f"{rospy.get_name()} esta listo para recibir comandos.", 0)
  rospy.spin()

if __name__ == "__main__":
  main()