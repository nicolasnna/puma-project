#!/usr/bin/env python3
import rospy
import actionlib
import requests
from puma_msgs.msg import Log
from puma_ip_devices.msg import SpeakerManagerAction, SpeakerManagerGoal, SpeakerManagerResult
from puma_ip_devices.utils import send_log_message

def play_speaker(goal: SpeakerManagerGoal):
  ip = rospy.get_param('~ip_speaker', '10.42.0.95')
  result = SpeakerManagerResult()
  if not goal.filename:
    result.message = 'Error: Es necesario indicar el nombre del archivo'
    result.success = False
  
  

def stop_speaker():
  ip = rospy.get_param('~ip_speaker', '10.42.0.95')
  pass

def speaker_srv_cb(goal: SpeakerManagerGoal):
  result = SpeakerManagerResult()
  
  if goal.action == SpeakerManagerGoal.ACTION_PLAY:
    play_speaker()
  elif goal.action == SpeakerManagerGoal.ACTION_STOP:
    stop_speaker()
  else:
    result.success = False
    result.message = "Comando 'action' no soportado"
    srv.set_aborted(result)
  

def main():
  rospy.init_node('speaker_manager_node')
  global srv
  srv = actionlib.SimpleActionServer('puma/speaker', SpeakerManagerAction, execute_cb=speaker_srv_cb,auto_start=False)
  srv.start()
  send_log_message(f"{rospy.get_name()} esta listo para recibir comandos.", 0)
  rospy.spin()

if __name__ == "__main__":
  main()