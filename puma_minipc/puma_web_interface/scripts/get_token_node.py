#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time
from puma_web_interface.utils import get_token

def main():
  rospy.init_node("get_token_api_node")
  BACKEND_URL = rospy.get_param("~backend_url", "http://localhost:8000")
  username = rospy.get_param("~username", "admin")
  password = rospy.get_param("~password", "admin")
  
  token = None
  while not token:
    rospy.loginfo(f"{rospy.get_name()} -> Esperando 5 segundos para la solicitud del token de autenticacion.")
    time.sleep(5)
    try: 
      token = get_token(BACKEND_URL, username, password)
    except Exception as e:
      rospy.logwarn(f"{rospy.get_name()} -> Error al obtener token: {e}")
  rospy.loginfo(f"{rospy.get_name()} -> Token recibido, publicando en el topic /puma/web/auth_token")
  bearer_token = f"Bearer {str(token)}"
  
  token_pub = rospy.Publisher("/puma/web/auth_token", String, queue_size=2)
  rate = rospy.Rate(0.5)
  token_msg = String()
  token_msg.data = bearer_token
  try: 
    while not rospy.is_shutdown():
      token_pub.publish(token_msg)
      rate.sleep()
  except:
    pass
  

if __name__ == "__main__":
  main()