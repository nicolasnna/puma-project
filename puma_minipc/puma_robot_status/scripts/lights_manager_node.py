#/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import actionlib
from puma_robot_status.msg import LightsManagerAction, LightsManagerGoal, LightsManagerResult
from puma_msgs.msg import StatusArduinoRelay, Log

LIGHT_FRONT = "puma/control/lights_front"
LIGHT_SECURITY = "puma/control/security_lights"

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
  except:
    send_log_message("Tiempo de espera excedido para revisar el estado del relay.", 2)
    return False
  return check

def rutine_security_lights():
  global security_pub
  ''' Proceso 1 - apagar rele '''
  msg = Bool()
  msg.data = False
  security_pub.publish(msg)
  rospy.sleep(0.4)
  check = get_status_relay()
  if check is False:
    return False
  if check.security_lights:
    send_log_message("El accionador de la luz de seguridad no se ha realizado correctamente.", 1)
    return False
  rospy.sleep(0.2)
  ''' Proceso 2 - prender rele '''
  msg = Bool()
  msg.data = True
  security_pub.publish(msg)
  rospy.sleep(0.4)
  check = get_status_relay()
  if check is False:
    return False
  if not check.security_lights:
    send_log_message("El accionador de la luz de seguridad no se ha realizado correctamente.", 1)
    return False
  ''' Completado proceso de cambio de modo '''
  send_log_message("Se ha cambiado el modo de la luz de seguridad correctamente.", 0)
  return True

def rutine_front_lights_activate():
  global front_pub
  msg = Bool()
  msg.data = True
  front_pub.publish(msg)
  rospy.sleep(0.5)
  check = get_status_relay()
  if check is False:
    return False
  if not check.lights_front:
    send_log_message("No se ha efectuado la activación de la luz frontal.", 1)
    return False
  send_log_message("Se ha activado la luz frontal correctamente.", 0)
  return True

def rutine_front_lights_deactivate():
  global front_pub
  msg = Bool()
  msg.data = False
  front_pub.publish(msg)
  rospy.sleep(0.5)
  check = get_status_relay()
  if check is False:
    return False
  if check.lights_front:
    send_log_message("No se ha efectuado la desactivación de la luz frontal.", 1)
    return False
  send_log_message("Se ha desactivado la luz frontal correctamente.", 0)
  return True

def execute_srv(goal: LightsManagerGoal):
  global server
  result = LightsManagerResult()
  
  if goal.action == "security":
    state = rutine_security_lights()
    result.success = state
    result.message = "Se ha cambiado el modo de la luz de seguridad correctamente." if state else "No se ha logrado cambiar el modo de la luz de seguridad."
  elif goal.action == "front_activate":
    state = rutine_front_lights_activate()
    result.success = state
    result.message = "Se ha activado la luz frontal correctamente." if state else "No se ha logrado activar la luz frontal."
  elif goal.action == "front_deactivate":
    state = rutine_front_lights_deactivate()
    result.success = state
    result.message = "Se ha desactivado la luz frontal correctamente." if state else "No se ha logrado desactivar la luz frontal."
  else:
    result.success = False
    result.message = "Accion invalida. Solo ACTION_SECURITY_SIGNAL, ACTION_FRONT_ACTIVATE y ACTION_FRONT_DEACTIVATE son soportadas."
  
  server.set_succeeded(result)

def main():
  rospy.init_node('lights_manager_node')
  global server, log_pub, security_pub, front_pub
  log_pub = rospy.Publisher('/puma/logs/add_log', Log, queue_size=10)
  security_pub = rospy.Publisher(LIGHT_SECURITY, Bool, queue_size=10)
  front_pub = rospy.Publisher(LIGHT_FRONT, Bool, queue_size=10)
  server = actionlib.SimpleActionServer('/puma/control/lights', LightsManagerAction, execute_cb=execute_srv, auto_start=False)
  server.start()
  send_log_message("Nodo de control de luces iniciado.", 0)
  
  rospy.spin()
  
if __name__ == "__main__":
  main()