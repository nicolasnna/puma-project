#!/usr/bin/env python3
import rospy
import requests
from datetime import datetime
try:
  from zoneinfo import ZoneInfo
except ImportError:
  from backports.zoneinfo import ZoneInfo

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