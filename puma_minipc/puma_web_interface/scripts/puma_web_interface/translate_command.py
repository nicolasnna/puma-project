import rospy
from std_msgs.msg import Empty, String, Float64
from puma_msgs.msg import ConfigurationStateMachine, WaypointNav, Waypoint, Log
from puma_state_machine.msg import StateMachineAction, StateMachineGoal
from smach_msgs.msg import SmachContainerStatus
from puma_nav_manager.msg import LocalizationManagerAction, LocalizationManagerGoal
from puma_robot_status.msg import LightsManagerAction, LightsManagerGoal, ChargeManagerAction, ChargeManagerGoal
from puma_system_monitor.msg import ServicesManagerAction, ServicesManagerGoal
from puma_ip_devices.msg import SpeakerManagerAction, SpeakerManagerGoal, RebootPtzAction, RebootPtzGoal
from puma_web_interface.utils import send_log_msg, send_latest_data
import actionlib

# ------ Publishers ------
change_mode_pub = rospy.Publisher("/puma/control/change_mode", String, queue_size=1)
# ------ Client action ------
client_sm_plan_config = actionlib.SimpleActionClient('/puma/state_machine/plan_configuration', StateMachineAction)
client_sm_run_plan = actionlib.SimpleActionClient('/puma/state_machine/run_plan', StateMachineAction)
client_sm_run_plan_custom = actionlib.SimpleActionClient('/puma/state_machine/run_plan_custom', StateMachineAction)
client_localization_manager = actionlib.SimpleActionClient('/puma/localization/manager', LocalizationManagerAction)
client_lights_manager = actionlib.SimpleActionClient('/puma/control/lights', LightsManagerAction)
client_service_jetson_manager = actionlib.SimpleActionClient('/puma/jetson/services_manager', ServicesManagerAction)
client_service_minipc_manager = actionlib.SimpleActionClient('/puma/minipc/services_manager', ServicesManagerAction)
client_charge_manager = actionlib.SimpleActionClient('/puma/control/charge', ChargeManagerAction)
client_speaker = actionlib.SimpleActionClient('/puma/speaker', SpeakerManagerAction)
client_reboot_ptz = actionlib.SimpleActionClient('/puma/nvr/reboot_ptz', RebootPtzAction)

def action_state_machine(client,goal):
  try:
    client.wait_for_server(rospy.Duration(5))
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5)):
      return True
    return False
  except Exception as e:
    send_log_msg(f"Error al conectar con el servidor: {e}", 1)
    return False
  
def state_smach():
  msg = rospy.wait_for_message("/puma/smach/container_status", SmachContainerStatus, timeout=5)
  if not msg:
    return None
  return msg.active_states[0]

# ------ Functions ------
def start_plan_fn(cmd):
  send_log_msg("Detectado comando para iniciar plan", 0)
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.START_PLAN
  return action_state_machine(client_sm_plan_config, goal)

def stop_plan_fn(cmd):
  send_log_msg("Detectado comando para detener plan", 0)
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.STOP_PLAN
  state = state_smach()
  if state == "RUN_PLAN":
    return action_state_machine(client_sm_run_plan, goal)
  elif state == "RUN_PLAN_CUSTOM":
    return action_state_machine(client_sm_run_plan_custom, goal)
  return False
  
def clear_plan_fn(cmd):
  send_log_msg("Detectado comando para limpiar plan", 0)
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.CLEAR_PLAN
  return action_state_machine(client_sm_plan_config, goal)
  
def save_plan_fn(cmd):
  send_log_msg(f"Detectado comando para guardar plan {cmd['name']}", 0)
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.SAVE_PLAN
  goal.file_name = String(cmd['name'])
  return action_state_machine(client_sm_plan_config, goal)
  
def load_plan_fn(cmd):
  send_log_msg(f"Detectado comando para cargar plan {cmd['name']}", 0)
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.LOAD_PLAN
  goal.file_name = String(cmd['name'])
  return action_state_machine(client_sm_plan_config, goal)

def configuration_plan_fn(cmd):
  send_log_msg(f"Detectado comando para configurar plan {cmd['plan_to_load']} con {cmd['nro_repeats']} repeticiones cada {cmd['minutes_between_repeats']} minutos", 0)
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  config = ConfigurationStateMachine()
  config.plan_to_load = cmd['plan_to_load']
  config.load_plan_from = cmd['load_plan_from']
  config.nro_repeats = cmd['nro_repeats']
  config.minutes_between_repeats = cmd['minutes_between_repeats']
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.CONFIG_PLAN
  goal.configuration_plan = config
  return action_state_machine(client_sm_plan_config, goal)
  
def add_waypoints_fn(waypoints):
  send_log_msg(f"Detectado comando para agregar waypoints {waypoints}", 0)
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  waypoints_msg = WaypointNav()
  for waypoint in waypoints['waypoints']:
    send_log_msg(f"- Agregando waypoint {waypoint}", 0)
    waypoints_msg.waypoints.append(Waypoint(latitude=waypoint['latitude'], longitude=waypoint['longitude'], yaw=waypoint['yaw']))
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.ADD_WEB
  goal.waypoint_nav = waypoints_msg
  return action_state_machine(client_sm_plan_config, goal)
  
def change_angle_degree_fn(cmd):
  send_log_msg(f"Detectado comando para cambiar orientación {cmd}", 0)
  try:
    client_localization_manager.wait_for_server(rospy.Duration(5))
    goal = LocalizationManagerGoal()
    goal.action = LocalizationManagerGoal.ACTION_CHANGE_ANGLE
    goal.angle_deg = Float64(cmd["angle"])
    client_localization_manager.send_goal(goal)
    if client_localization_manager.wait_for_result(rospy.Duration(5)):
      return True
    return False
  except Exception as e:
    send_log_msg(f"Error al conectar con el servidor: {e}", 1)
    return False
  
def change_mode_fn(cmd):
  send_log_msg(f"Detectado comando para cambiar modo {cmd}", 0)
  change_mode_pub.publish(String(cmd["mode"]))
  return True
  
def change_front_lights_fn(cmd):
  send_log_msg(f"Detectado comando para cambiar luces frontales {cmd}", 0)
  try:
    client_lights_manager.wait_for_server(rospy.Duration(5))
    goal = LightsManagerGoal()
    goal.action = LightsManagerGoal.ACTION_FRONT_ACTIVATE if cmd["state"] else LightsManagerGoal.ACTION_FRONT_DEACTIVATE
    client_lights_manager.send_goal(goal)
    client_lights_manager.wait_for_result(rospy.Duration(5))
    result = client_lights_manager.get_result()
    return result.success
  except Exception as e:
    send_log_msg(f"Error al ejecutar la acción: {e}", 1)
    return False

def change_security_lights_fn(cmd):
  send_log_msg(f"Detectado comando para cambiar el modo de la baliza de seguridad", 0)
  try:
    client_lights_manager.wait_for_server(rospy.Duration(5))
    goal = LightsManagerGoal()
    goal.action = LightsManagerGoal.ACTION_SECURITY_SIGNAL
    client_lights_manager.send_goal(goal)
    client_lights_manager.wait_for_result(rospy.Duration(5))
    result = client_lights_manager.get_result()
    if result.success:
      send_log_msg("Baliza de seguridad activada correctamente", 0)
    return result.success
  except Exception as e:
    send_log_msg(f"Error al ejecutar la acción: {e}", 1)
    return False
  

def get_services_from_client(client: StateMachineAction):
  client.wait_for_server(rospy.Duration(5))
  goal = ServicesManagerGoal()
  goal.command = ServicesManagerGoal.GET_ALL_SERVICES_CMD
  client.send_goal(goal)
  if client.wait_for_result(rospy.Duration(10)):
    result = client.get_result()
    if result.success:
      send_log_msg("Servicios obtenidos correctamente", 0)
      if result.services:
        send_log_msg(f"Servicios: {[s.service_name for s in result.services]}", 0)
    return result.services
  return []
  
def update_services_state_fn(cmd):
  send_log_msg("Detectado comando para actualizar la información de los servicios", 0)
  
  try: 
    srvs_jetson_raw = get_services_from_client(client_service_jetson_manager)
    srvs_minipc_raw = get_services_from_client(client_service_minipc_manager)
    
    srvs_jetson = []
    for i in srvs_jetson_raw:
      srvs_jetson.append({"service_name": i.service_name, "state": i.state, "default": i.default})
    srvs_minipc = []
    for i in srvs_minipc_raw:
      srvs_minipc.append({"service_name": i.service_name, "state": i.state, "default": i.default})
      
    send_latest_data("services_jetson", {"data": srvs_jetson})
    send_latest_data("services_minipc", {"data": srvs_minipc})
    
  except Exception as e:
    send_log_msg(f"Error al actualizar los servicios del robot: {e}", 1)
    return False
  
  send_log_msg("Se ha actualizado la información de los servicios del robot en la base de datos", 0)
  return True
  
def change_service_state_fn(cmd): 
  """
  cmd: { action: str, service_name: str, source: str }  
  """
  send_log_msg("Detectado comando para cambiar el estado de los servicios", 0)
  
  if cmd["source"] == "jetson":
    client = client_service_jetson_manager
  elif cmd["source"] == "minipc":
    client = client_service_minipc_manager
  else:
    send_log_msg(f"Error: fuente de servicio no válida {cmd['source']}", 1)
    return False
  
  try:
    client.wait_for_server(rospy.Duration(5))
    goal = ServicesManagerGoal()
    if cmd["action"] == 'enabled':
      goal.command = 'enable'
    elif cmd["action"] == 'disabled':
      goal.command = 'disable'
    else:
      goal.command = cmd["action"]
    goal.service_name = cmd["service_name"]
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5)):
      result = client.get_result()
      send_log_msg(f"Resultado de la acción del servicio {cmd['service_name']}: {result.message}", 0)
      return result.success
  except Exception as e:
    send_log_msg(f"Error al conectar con el servidor: {e}", 1)
  return False

def change_charge_connector_state_fn(cmd):
  send_log_msg("Detectado comando para cambiar el estado el conector de carga", 0)
  
  try:
    client_charge_manager.wait_for_server(rospy.Duration(5))
    goal = ChargeManagerGoal()
    goal.action = cmd["action"]
    client_charge_manager.send_goal(goal)
    if client_charge_manager.wait_for_result(rospy.Duration(5)):
      result = client_charge_manager.get_result()
      send_log_msg(f"Resultado de la acción del conector de carga: {result.message}", 0)
      return result.success
  except Exception as e:
    send_log_msg(f"Error al conectar con el servidor: {e}", 1)
  return False
  
def execute_speaker_fn(cmd):
  send_log_msg("Detectado comando para usar el parlante ip", 0)
  
  try: 
    client_speaker.wait_for_server(rospy.Duration(5))
    goal = SpeakerManagerGoal()
    goal.action = cmd["action"]
    if cmd["action"] == "play":
      goal.filename = cmd["filename"]
      goal.mode = cmd["mode"]
      goal.volume = cmd["volume"]
      if cmd["time_seconds"] is not None:
        goal.time_seconds = cmd["time_seconds"]
      if cmd["repeats"]:
        goal.repeats = cmd["repeats"]
    client_speaker.send_goal(goal)
    if client_speaker.wait_for_result(rospy.Duration(5)):
      result = client_speaker.get_result()
      send_log_msg(f"Resultado de la acción del parlante: {result.message}", 0)
      return result.success
  except Exception as e:
    send_log_msg(f"Error al conectar con el servidor: {e}", 1)
  return False

def reboot_ptz_fn(cmd):
  send_log_msg("Detectado comando para reinicio de la cámara ptz", 0)
  try:
    client_reboot_ptz.wait_for_server(rospy.Duration(5))
    goal = RebootPtzGoal()
    goal.action = cmd["action"]
    client_reboot_ptz.send_goal(goal)
    if client_reboot_ptz.wait_for_result(rospy.Duration(5)):
      result = client_reboot_ptz.get_result()
      send_log_msg(f"Resultado del reinicio de la cámara ptz: {result.message}", 0)
      return result.success
  except Exception as e:
    send_log_msg(f"Error al conectar con el servidor para la ptz: {e}", 1)
  return False

translate_command = {
  "start_plan": start_plan_fn,
  "stop_plan": stop_plan_fn,
  "clear_plan": clear_plan_fn,
  "save_plan": save_plan_fn,
  "load_plan": load_plan_fn,
  "config_plan": configuration_plan_fn,
  "upload_waypoints_to_robot": add_waypoints_fn,
  "change_angle_robot": change_angle_degree_fn,
  "change_mode_robot": change_mode_fn,
  "change_front_lights": change_front_lights_fn,
  "change_security_lights": change_security_lights_fn,
  "update_services_state": update_services_state_fn,
  "change_service_state": change_service_state_fn,
  "change_charge_connector_state": change_charge_connector_state_fn,
  "execute_speaker": execute_speaker_fn,
  "reboot_ptz": reboot_ptz_fn
}
