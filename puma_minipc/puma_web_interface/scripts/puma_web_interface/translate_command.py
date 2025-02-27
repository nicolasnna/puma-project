import rospy
from std_msgs.msg import Empty, String, Float64
from puma_msgs.msg import ConfigurationStateMachine, WaypointNav, Waypoint
from puma_state_machine.msg import StateMachineAction, StateMachineGoal
from smach_msgs.msg import SmachContainerStatus
from puma_nav_manager.msg import LocalizationManagerAction, LocalizationManagerGoal
import actionlib

# ------ Publishers ------
start_plan_pub = rospy.Publisher("/puma/state_machine/start_plan", Empty, queue_size=1)
stop_plan_pub = rospy.Publisher("/puma/state_machine/plan_stop", Empty, queue_size=1)
clear_plan_pub = rospy.Publisher("/puma/state_machine/clear_plan", Empty, queue_size=1)
save_plan_pub = rospy.Publisher("/puma/state_machine/save_plan", String, queue_size=1)
load_plan_pub = rospy.Publisher("/puma/state_machine/load_plan", String, queue_size=1)
configuration_plan_pub = rospy.Publisher("/puma/state_machine/configuration_cmd", ConfigurationStateMachine, queue_size=1)
add_waypoints_pub = rospy.Publisher("/puma/state_machine/add_waypoints_web", WaypointNav, queue_size=1)
set_orientation_localization_pub = rospy.Publisher("/puma/localization/change_angle_degree", Float64, queue_size=1)
change_mode_pub = rospy.Publisher("/puma/control/change_mode", String, queue_size=1)
# ------ Client action ------
client_sm_plan_config = actionlib.SimpleActionClient('/puma/state_machine/plan_configuration', StateMachineAction)
client_sm_run_plan = actionlib.SimpleActionClient('/puma/state_machine/run_plan', StateMachineAction)
client_sm_run_plan_custom = actionlib.SimpleActionClient('/puma/state_machine/run_plan_custom', StateMachineAction)
client_localization_manager = actionlib.SimpleActionClient('/puma/localization/manager', LocalizationManagerAction)

def action_state_machine(client,goal):
  try:
    client.wait_for_server(rospy.Duration(5))
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5)):
      return True
    return False
  except Exception as e:
    rospy.logwarn(f"Error al conectar con el servidor: {e}")
    return False
  
def state_smach():
  msg = rospy.wait_for_message("/puma/smach/container_status", SmachContainerStatus, timeout=5)
  if not msg:
    return None
  return msg.active_states[0]

# ------ Functions ------
def start_plan_fn(cmd):
  rospy.loginfo("Detectado comando para iniciar plan")
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.START_PLAN
  return action_state_machine(client_sm_plan_config, goal)

def stop_plan_fn(cmd):
  rospy.loginfo("Detectado comando para detener plan")
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.STOP_PLAN
  state = state_smach()
  if state == "RUN_PLAN":
    return action_state_machine(client_sm_run_plan, goal)
  elif state == "RUN_PLAN_CUSTOM":
    return action_state_machine(client_sm_run_plan_custom, goal)
  return False
  
def clear_plan_fn(cmd):
  rospy.loginfo("Detectado comando para limpiar plan")
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.CLEAR_PLAN
  return action_state_machine(client_sm_plan_config, goal)
  
def save_plan_fn(cmd):
  rospy.loginfo(f"Detectado comando para guardar plan {cmd['name']}")
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.SAVE_PLAN
  goal.file_name = String(cmd['name'])
  return action_state_machine(client_sm_plan_config, goal)
  
def load_plan_fn(cmd):
  rospy.loginfo(f"Detectado comando para cargar plan {cmd['name']}")
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.LOAD_PLAN
  goal.file_name = String(cmd['name'])
  return action_state_machine(client_sm_plan_config, goal)

def configuration_plan_fn(cmd):
  rospy.loginfo(f"Detectado comando para configurar plan {cmd['plan_to_load']} con {cmd['nro_repeats']} repeticiones cada {cmd['minutes_between_repeats']} minutos")
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
  rospy.loginfo(f"Detectado comando para agregar waypoints {waypoints}")
  state = state_smach()
  if state != "PLAN_CONFIGURATION":
    return False
  waypoints_msg = WaypointNav()
  for waypoint in waypoints['waypoints']:
    rospy.loginfo(f"- Adding waypoint {waypoint}")
    waypoints_msg.waypoints.append(Waypoint(latitude=waypoint['latitude'], longitude=waypoint['longitude'], yaw=waypoint['yaw']))
  goal = StateMachineGoal()
  goal.action = StateMachineGoal.ADD_WEB
  goal.waypoint_nav = waypoints_msg
  return action_state_machine(client_sm_plan_config, goal)
  
def change_angle_degree_fn(cmd):
  rospy.loginfo(f"Detectado comando para cambiar orientación {cmd}")
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
    rospy.logwarn(f"Error al conectar con el servidor: {e}")
    return False
  
def change_mode_fn(cmd):
  rospy.loginfo(f"Detectado comando para cambiar modo {cmd}")
  change_mode_pub.publish(String(cmd["mode"]))
  
translate_command = {
  "start_plan": start_plan_fn,
  "stop_plan": stop_plan_fn,
  "clear_plan": clear_plan_fn,
  "save_plan": save_plan_fn,
  "load_plan": load_plan_fn,
  "config_plan": configuration_plan_fn,
  "upload_waypoints_to_robot": add_waypoints_fn,
  "change_angle_robot": change_angle_degree_fn,
  "change_mode_robot": change_mode_fn
}
