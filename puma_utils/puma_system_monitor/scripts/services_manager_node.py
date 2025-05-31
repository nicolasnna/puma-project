#!/usr/bin/env python3
import rospy
import subprocess
import actionlib
from puma_system_monitor.msg import ServicesManagerAction, ServicesManagerGoal, ServicesManagerResult
from puma_msgs.msg import ServiceInfo

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

def get_all_services_raw():
  systemctl_list = subprocess.Popen(
    ["systemctl", "list-unit-files", "--type=service", "--no-pager"],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )
  grep = subprocess.Popen(
    ["grep", "-E", "puma-.*\\.service"],
    stdin=systemctl_list.stdout,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )
  try:
    out, err = grep.communicate(timeout=30)
  except subprocess.TimeoutExpired:
    grep.kill()
    out, err = grep.communicate()
  
  return out.decode("utf-8"), err.decode("utf-8"), grep.returncode

def get_all_services():
  out, err, code = get_all_services_raw()
  if code != 0:
    return [], err, code
  array = out.split()
  is_jetson = rospy.get_param("~is_jetson_nano", False)
  columns = 2 if is_jetson else 3
  total_services = len(array) // columns # name state default
  services = []
  for i in range(0,total_services):
    service = ServiceInfo()
    service.service_name = array[i*columns].split(".")[0]
    if service.service_name == "puma-jetson":
      continue
    service.state = array[i*columns + 1]
    service.default = array[i*columns + (columns - 1)]
    services.append(service)
  return services, err, code
  
  
def action_server_cb(goal : ServicesManagerGoal):
  result = ServicesManagerResult()
  rospy.loginfo("Recibido solicitud para servicios")

  if goal.command == ServicesManagerGoal.GET_ALL_SERVICES_CMD:
    rospy.loginfo("Detectado comando para obtener todos los servicios")
    out, err, code = get_all_services()
    result.success = code == 0
    result.message = "Información de servicios obtenidos" if code == 0 else f"Error al obtener servicios: {err}"
    result.services = out
    srv.set_succeeded(result)
    return

  if not goal.command or not goal.service_name:
    result.success = False
    result.message ="Error: No se ha detectado el comando o nombre del servicio"
    srv.set_aborted(result)
    return
    
  if goal.command not in [
    ServicesManagerGoal.START_CMD,
    ServicesManagerGoal.STOP_CMD,
    ServicesManagerGoal.RESTART_CMD,
    ServicesManagerGoal.RELOAD_CMD,
    ServicesManagerGoal.STATUS_CMD,
    ServicesManagerGoal.ENABLE_CMD,
    ServicesManagerGoal.DISABLE_CMD
    ]:
    return srv.set_aborted(result, f"Error: Comando {goal.command} no soportado")
  
  out, err, code = call_service_command(goal.command, goal.service_name)
  if code == 0:
    result.success = True
    result.message = f"Servicio {goal.command} {goal.service_name} efectuado con exito: {out}"
    srv.set_succeeded(result)
    return 
  
  result.success = False
  result.message = f"Error al ejecutar el servicio {goal.command} {goal.service_name}: {code} {err}"
  srv.set_aborted(result)
  
def main():
  rospy.init_node("services_manager_node")
  global srv
  ns = rospy.get_param('~ns_service', '')
  srv = actionlib.SimpleActionServer(ns+'/services_manager', ServicesManagerAction, action_server_cb, False)
  srv.start()
  
  rospy.spin()

if __name__ == "__main__":
  main()