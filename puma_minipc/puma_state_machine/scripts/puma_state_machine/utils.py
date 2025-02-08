#!/usr/bin/env python3
import math
import rospy
import tf
from typing import Tuple
from geographiclib.geodesic import Geodesic
from puma_msgs.msg import Log, WaypointNav, Waypoint
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import Path


def calc_goal_from_gps(origin_lat, origin_long, goal_lat, goal_long):
  # Source: https://github.com/danielsnider/gps_goal/blob/master/src/gps_goal/gps_goal.py
  '''
  Convencion ENU (East, North, Up) X apunta al este e Y apunta al Norte
  '''
  geod = Geodesic.WGS84
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
  hypotenuse = distance = g['s12']
  azimuth = g['azi1']

  azimuth = math.radians(azimuth)


  # y = adjacent = math.cos(azimuth) * hypotenuse # ESTE
  # x = opposite = math.sin(azimuth) * hypotenuse # NORTE
  
  x = math.cos(azimuth) * hypotenuse  # ESTE
  y = math.sin(azimuth) * hypotenuse # NORTE

  return x,y

def calculate_bearing_from_xy(x1, y1, x2, y2):
  """
  Calcular el bearing entre dos puntos en coordenadas (x, y).
  
  Parámetros:
  x1, y1 -- coordenadas del punto origen
  x2, y2 -- coordenadas del punto destino
  
  Retorna:
  Bearing en grados (entre 0° y 360°)
  """
  delta_x = x2 - x1
  delta_y = y2 - y1

  # Calcular el ángulo en radianes
  bearing_rad = math.atan2(delta_y, delta_x)

  # Convertir a grados y normalizar entre 0-360°
  bearing_deg = (math.degrees(bearing_rad) + 360) % 360

  return bearing_deg

def yaw_to_quaternion(yaw):
  """
  Convert a yaw angle (in degrees) to a quaternion.
  
  Parameters:
  yaw -- yaw angle in degrees
  
  Returns:
  (x, y, z, w) -- tuple representing the quaternion
  """
  # Convert yaw from degrees to radians
  yaw = math.radians(yaw)
  
  # Calculate the quaternion components
  w = math.cos(yaw / 2)
  x = 0.0
  y = 0.0
  z = math.sin(yaw / 2)
  
  return (x, y, z, w)


def create_and_publish_log(msg, level, name_state):
  log = Log()
  log.level = level
  log.content = msg
  log.node = rospy.get_name()+"/"+name_state
  rospy.Publisher('/puma/logs/add_log', Log, queue_size=2).publish(log)
  if level == 0:
    rospy.loginfo(msg)
  elif level == 1:
    rospy.logwarn(msg)
  elif level == 2:
    rospy.logerr(msg)
    
def check_mode_control_navegacion(mode, send_log):
  try:
    mode_msg = rospy.wait_for_message('/puma/control/current_mode', String, timeout=5)
  except Exception as e:
    send_log(f"No se ha podido obtener el modo actual por el error: {e}. Volviendo a la configuración de planes.", 1)
    return False
  
  if mode_msg.data != 'navegacion':
    send_log(f"No se ha podido cambiar al modo de navegación (modo actual {mode_msg.data}). Volviendo a la configuración de planes.", 1)
    return False
  
  return True

def get_goal_from_waypoint(waypoint: Waypoint):
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = "map"
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = waypoint.x
  goal.target_pose.pose.position.y = waypoint.y
  quaternions = tf.transformations.quaternion_from_euler(0, 0, waypoint.yaw)
  goal.target_pose.pose.orientation.x = quaternions[0]
  goal.target_pose.pose.orientation.y = quaternions[1]
  goal.target_pose.pose.orientation.z = quaternions[2]
  goal.target_pose.pose.orientation.w = quaternions[3]
  return goal


def check_and_get_waypoints(send_log) -> Tuple[bool, WaypointNav]:
  try:
    waypoints_msg = rospy.wait_for_message('/puma/navigation/waypoints/waypoints_info', WaypointNav, timeout=5)
  except Exception as e:
    send_log(f"No se han cargado correctamente los waypoints por el error: {e}.", 1)
    return False, None
  
  
  if len(waypoints_msg.waypoints) == 0:
    send_log("No se tienen waypoints almacenados", 1)
    return False, None
  
  return True, waypoints_msg

def check_plan_load_file():
  try:
    plan_calculated = rospy.wait_for_message('/puma/navigation/plan/plan_info', Path, timeout=5)
    if plan_calculated is not None and len(plan_calculated.poses) > 0:
      return True
    else:
      rospy.logwarn("No se ha cargado correctamente el plan.")
      return False
  except Exception as e:
    rospy.logwarn(f"No se ha podido revisar el ultimo plan empleado: {e}")
    return False