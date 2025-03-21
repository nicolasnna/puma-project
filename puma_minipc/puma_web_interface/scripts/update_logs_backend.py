import rospy
import requests
from puma_web_interface.utils import get_token
from puma_robot_status.msg import LoggerManagerAction, LoggerManagerGoal
import actionlib
import json

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
    return client.get_result().log_array
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
  rospy.loginfo("Backend: "+BACKEND_URL)
  client_log = actionlib.SimpleActionClient('/puma/logs', LoggerManagerAction)
  
  token = get_token(BACKEND_URL)
  while not token:
    rospy.loginfo(f"{rospy.get_name()} - Token no encontrado, esperando 3 segundos")
    rospy.sleep(3)
    token = get_token(BACKEND_URL)
    
  bearer_token = f"Bearer {str(token)}"
  headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token}
  
  while not rospy.is_shutdown():
    logs_msgs = get_logs(client_log)
    if isinstance(logs_msgs.logs, list) and len(logs_msgs.logs) > 0:
      logArray = [convert_log_to_dict(log) for log in logs_msgs.logs]
      send_logs_to_backend({"logs":logArray}, headers, BACKEND_URL)
    rospy.sleep(5)

if __name__ == "__main__":
  main()