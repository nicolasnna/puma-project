#!/usr/bin/env python3
import rospy
import subprocess
import actionlib
from puma_system_monitor.msg import ServicesManagerAction, ServicesManagerGoal, ServicesManagerResult

def call_service_command(cmd, service_name):
  process = subprocess.Popen(
    ["sudo", "systemctl", cmd, f"{service_name}.service"],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )
  try:
    output, error = process.communicate(timeout=30)
  except subprocess.TimeoutExpired:
    process.kill()
    output, error = process.communicate()
  
  return output.decode("utf-8"), error.decode("utf-8"), process.returncode
  
def action_server_cb(goal : ServicesManagerGoal):
  result = ServicesManagerResult()

  if not goal.command or not goal.service_name:
    result.success = False
    result.message ="Error: No se ha detectado el comando o nombre del servicio"
    srv.set_aborted(result)
    
  out, err, code = call_service_command(goal.command, goal.service_name)
  if code == 0:
    result.success = True
    result.message = f"Servicio {goal.command} {goal.service_name} efectuado con exito"
    srv.set_succeeded(result)
    return 
  
  result.success = False
  result.message = f"Error al ejecutar el servicio {goal.command} {goal.service_name}: {err}"
  srv.set_aborted(result)
  
def main():
  rospy.init_node("services_manager_node")
  global srv
  srv = actionlib.SimpleActionServer('/services_manager', ServicesManagerAction, action_server_cb, False)
  srv.start()
  
  rospy.spin()

if __name__ == "__main__":
  main()