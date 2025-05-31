#/usr/bin/env python3
import rospy
import actionlib
from puma_ip_devices.msg import RebootPtzAction, RebootPtzResult, RebootPtzGoal
import requests
from requests.auth import HTTPDigestAuth
from puma_ip_devices.utils import send_log_message

def send_reboot_request():
  ip = rospy.get_param("ip_ptz", "10.42.0.103")
  url = f"http://{ip}/cgi-bin/magicBox.cgi?action=reboot"
  user = rospy.get_param("user_ptz", "admin")
  password = rospy.get_param("password_ptz", "smartbot2023")
  auth = HTTPDigestAuth(user, password)
  try: 
    r = requests.get(url, auth=auth, timeout=10)
    if r.status_code == 200:
      send_log_message("Acción de reinicio efectuada correctamente.", 0)
      return True
    send_log_message(f"Ha fallado en la acción de reinicio: {r.status_code}", 1)
    return False
  except Exception as e:
    send_log_message(f"Error al enviar la solicitud de reinicio: {e}", 2)
    return False

def execute_srv(goal: RebootPtzGoal):
  global server
  result = RebootPtzResult()
  
  if goal.action == RebootPtzGoal.REBOOT:
    send_log_message("Reiniciando PTZ...", 0)
    # Simulate reboot action
    if send_reboot_request():
      result.success = True
      result.message = "PTZ reiniciado correctamente."
    else:
      result.success = False
      result.message = "No se ha logrado efectuar el reinicio de la cámara ptz."
  else:
    result.success = False
    result.message = "Accion invalida. Solo REBOOT es soportado."
    
  server.set_succeeded(result)

def main():
  rospy.init_node('reboot_ptz_node')
  global server
  server = actionlib.SimpleActionServer('puma/nvr/reboot_ptz', RebootPtzAction, execute_cb=execute_srv, auto_start=False)
  server.start()
  send_log_message(f"{rospy.get_name()} esta listo para recibir comandos.", 0)
  rospy.spin()
  
if __name__ == "__main__":
  main()