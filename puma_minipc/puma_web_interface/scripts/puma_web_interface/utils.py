#!/usr/bin/env python3
import rospy
import requests
from datetime import datetime
from puma_msgs.msg import Log
try:
  from zoneinfo import ZoneInfo
except ImportError:
  from backports.zoneinfo import ZoneInfo

add_log_pub = rospy.Publisher("/puma/logs/add_log", Log, queue_size=4)

def get_token(BACKEND_URL):
  headers = { 'Content-Type': 'application/x-www-form-urlencoded'}
  body = { 'username': 'puma', 'password': 'puma2023'}
  try:
    response = requests.post(BACKEND_URL+"/auth/login", headers=headers, data=body, timeout=5)
    if response.status_code == 200:
      return response.json()['access_token']
    else:
      rospy.logwarn(f"Error al obtener token: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Error al obtener token: {e}")
    
def time_chile_now():
  return datetime.now(ZoneInfo("Chile/Continental"))

def send_log_msg(content,level):
  """
  Envía un mensaje de log al topic /puma/logs/add_log
  """
  node_name = rospy.get_name()
  log = Log()
  log.content = content
  log.node = node_name
  log.level = level
  text = f"{node_name} - {content}"
  if level == 0:
    rospy.loginfo(text)
  elif level == 1:
    rospy.logwarn(text)
  elif level == 2:
    rospy.logerr(text)
  add_log_pub.publish(log)