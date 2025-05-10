#!/usr/bin/env python3
import rospy
import actionlib
from puma_robot_status.msg import ChargeManagerAction, ChargeManagerGoal, ChargeManagerResult
from puma_msgs.msg import StatusArduinoRelay, Log
from std_msgs.msg import Bool

CHARGE_CONNECTION = "puma/control/charge_connection"

def send_log_message(message, level):
  global log_pub
  msg = Log()
  msg.node = rospy.get_name()
  msg.level = level
  msg.content = message
  if level == 0:
    rospy.loginfo(message)
  elif level == 1:
    rospy.logwarn(message)
  elif level == 2:
    rospy.logerr(message)
  log_pub.publish(msg)

def get_status_relay():
  try:
    check = rospy.wait_for_message('/puma/arduino/status_relay', StatusArduinoRelay, timeout=5)
    return check
  except:
    send_log_message("Tiempo de espera excedido para revisar el estado del relay.", 2)
    return False

def rutine_charge_connection(value: bool):
  msg = Bool()
  msg.data = value
  charge_rele_pub.publish(msg)
  rospy.sleep(0.5)
  check: StatusArduinoRelay | False = get_status_relay()
  if check is False:
    return False
  if check.charge_connection == value:
    send_log_message("Se ha efectuado el cambio del cargador correctamente al estado " + str(value) + ".", 0)
    return False
  
  send_log_message("No se ha logrado efecuar el cambio en el estado del cargador.", 2)
  return True

def charge_srv_cb(goal: ChargeManagerGoal):
  result = ChargeManagerResult()
  
  if goal.action == ChargeManagerGoal.ACTION_CONNECT:
    is_realize = rutine_charge_connection(True)
    result.success = is_realize
    result.message = "Se ha activado el cargador correctamente." if is_realize else "No se ha logrado cambiar el estado del cargador"
  elif goal.action == ChargeManagerGoal.ACTION_DISCONNECT:
    is_realized = rutine_charge_connection(False)
    result.success = is_realized
    result.message = "Se ha desactivado el cargador correctamente." if is_realized else "No se ha logrado cambiar el estado del cargador"
  else:
    result.success = False
    result.message = "Action no soportado"
    srv.set_aborted(result)
    return
  
  srv.set_succeeded(result)

def main():
  rospy.init_node('charge_manager_node')
  global srv, charge_rele_pub
  
  charge_rele_pub = rospy.Publisher(CHARGE_CONNECTION, Bool, queue_size=2)
  
  srv = actionlib.SimpleActionServer('/puma/control/charge', ChargeManagerAction, execute_cb=charge_srv_cb, auto_start=False)
  srv.start()
  
  rospy.spin()

if __name__ == "__main__":
  main()