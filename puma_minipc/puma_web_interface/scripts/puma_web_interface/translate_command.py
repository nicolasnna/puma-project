import rospy
from std_msgs.msg import Empty, String, Float64
from puma_msgs.msg import ConfigurationStateMachine, WaypointNav, Waypoint

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

# ------ Functions ------
def start_plan_fn(cmd):
  rospy.loginfo("Detectado comando para iniciar plan")
  start_plan_pub.publish(Empty())

def stop_plan_fn(cmd):
  rospy.loginfo("Detectado comando para detener plan")
  stop_plan_pub.publish(Empty())

def clear_plan_fn(cmd):
  rospy.loginfo("Detectado comando para limpiar plan")
  clear_plan_pub.publish(Empty())
  
def save_plan_fn(cmd):
  rospy.loginfo(f"Detectado comando para guardar plan {cmd['name']}")
  save_plan_pub.publish(String(cmd['name']))
  
def load_plan_fn(cmd):
  rospy.loginfo(f"Detectado comando para cargar plan {cmd['name']}")
  load_plan_pub.publish(String(cmd['name']))

def configuration_plan_fn(cmd):
  rospy.loginfo(f"Detectado comando para configurar plan {cmd['plan_to_load']} con {cmd['nro_repeats']} repeticiones cada {cmd['minutes_between_repeats']} minutos")
  config = ConfigurationStateMachine()
  config.plan_to_load = cmd['plan_to_load']
  config.load_plan_from = cmd['load_plan_from']
  config.nro_repeats = cmd['nro_repeats']
  config.minutes_between_repeats = cmd['minutes_between_repeats']
  configuration_plan_pub.publish(config)
  
def add_waypoints_fn(waypoints):
  rospy.loginfo(f"Detectado comando para agregar waypoints {waypoints}")
  waypoints_msg = WaypointNav()
  for waypoint in waypoints['waypoints']:
    rospy.loginfo(f"- Adding waypoint {waypoint}")
    waypoints_msg.waypoints.append(Waypoint(latitude=waypoint['latitude'], longitude=waypoint['longitude'], yaw=waypoint['yaw']))
  add_waypoints_pub.publish(waypoints_msg)
  
def change_angle_degree_fn(cmd):
  rospy.loginfo(f"Detectado comando para cambiar orientación {cmd}")
  set_orientation_localization_pub.publish(Float64(cmd["angle"]))
  
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
