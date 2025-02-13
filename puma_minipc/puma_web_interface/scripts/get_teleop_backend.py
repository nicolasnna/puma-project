#!/usr/bin/env python3
import rospy
import requests
from datetime import datetime
from puma_web_interface.utils import get_token
import json

def check_teleop_cmd():
  global headers, BACKEND_URL
  try:
    response = requests.get(BACKEND_URL+"/db/teleop", headers=headers, timeout=1)
    rospy.loginfo(response.json())
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Error al actualizar datos: {e}")

if __name__ == "__main__":
  rospy.init_node("get_teleop_backend")
  rospy.loginfo("Node get_teleop_backend started")
  global headers, BACKEND_URL
  BACKEND_URL = rospy.get_param('~backend_url',"http://localhost:8000")
  try: 
    token = get_token(BACKEND_URL)
    
    if token:
      bearer_token = f"Bearer {str(token)}"
      headers = { 'Content-Type': 'application/json', 'Authorization': bearer_token}
    
      rate = rospy.Rate(20)
      while not rospy.is_shutdown():
        check_teleop_cmd()
        rate.sleep()
      
  except Exception as e:
    rospy.logwarn(f"Error: {e}")