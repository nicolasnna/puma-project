import rospy
import requests
from puma_robot_status.msg import LoggerManagerAction, LoggerManagerGoal
from std_msgs.msg import String
import actionlib
import json
import time

def convert_log_to_dict(log):
  return {
    "date": log.date_text,
    "level": log.level,
    "node": log.node,
    "content": log.content
  }

def get_logs(client):
  try:
    client.wait_for_server(rospy.Duration(3))
    goal = LoggerManagerGoal()
    goal.action = LoggerManagerGoal.GET_LOG_AND_CLEAN
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(3))
    result = client.get_result()
    return result.log_array.logs
  except Exception as e:
    rospy.logwarn(f"Error al conectar con el servidor: {e}")
    return []
  
def send_logs_to_backend(logs, headers, backend_url):
  try:
    # rospy.loginfo(f"logs: {json.dumps(logs)}")
    res = requests.post(backend_url+"/robot/log/multiple", headers=headers, data=json.dumps(logs), timeout=5)
    if res.status_code != 200:
      rospy.logwarn(f"Error al enviar datos: {res.status_code} - {res.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Error al enviar datos: {e}")

def main():
  rospy.init_node("update_logs_backend")
  rospy.loginfo(f"Empezando {rospy.get_name()} node")
  # global headers, BACKEND_URL, client_log
  BACKEND_URL = rospy.get_param('~backend_url',"http://localhost:8000")
  rospy.loginfo(f"{rospy.get_name()} -> Backend: "+BACKEND_URL)
  client_log = actionlib.SimpleActionClient('/puma/logs', LoggerManagerAction)
  
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
    logs_msgs = get_logs(client_log)
    if isinstance(logs_msgs, list) and len(logs_msgs) > 0:
      logArray = [convert_log_to_dict(log) for log in logs_msgs]
      send_logs_to_backend({"logs":logArray}, headers, BACKEND_URL)
    time.sleep(5)

if __name__ == "__main__":
  main()