#!/usr/bin/env python3
import math
import rospy
import tf
from typing import Tuple
from puma_msgs.msg import Log, WaypointNav, Waypoint
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import Path, Odometry
from geonav_transform import geonav_conversions as gc
from sensor_msgs.msg import NavSatFix

def get_xy_robot():
  odom_topic = rospy.get_param("~odom_topic",'/puma/localization/ekf_odometry')
  try: 
    odom_robot = rospy.wait_for_message(odom_topic, Odometry, timeout=5)
    pos_x = odom_robot.pose.pose.position.x
    pos_y = odom_robot.pose.pose.position.y
    return pos_x, pos_y
  except Exception as e:
    rospy.logerr(f"No se ha podido obtener la posición actual del robot: {e}")
    return None, None
  
def get_lat_lon_robot():
    gps_topic = rospy.get_param("~gps_topic",'/puma/sensors/gps/fix')
    try:
      gps_robot = rospy.wait_for_message(gps_topic, NavSatFix, timeout=5)
      lat = gps_robot.latitude
      lon = gps_robot.longitude
      return lat, lon
    except Exception as e:
      rospy.logerr(f"No se ha podido obtener la posición global actual del robot: {e}")
      return None, None
    
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
  '''
  Crea un mensaje de log y lo publica en el tópico /puma/logs/add_log
  '''
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
  '''
  Crea un mensaje de goal de move_base a partir de un waypoint.
  '''
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
    waypoints_msg = rospy.wait_for_message('/puma/navigation/waypoints_list', WaypointNav, timeout=5)
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
  
def get_xy_based_on_lat_long(lat_origin,lon_origin,lat,lon):
    '''
    Convierte coordenadas geográficas (latitud, longitud) a coordenadas cartesianas (x, y) en metros.'''
    
    xg2, yg2 = gc.ll2xy(lat,lon,lat_origin,lon_origin)
    # utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    # xa,ya = axy.ll2xy(lat,lon,olat,olon)

    # rospy.loginfo("#########  "+name+"  ###########")  
    # rospy.loginfo("LAT COORDINATES ==>"+str(lat)+","+str(lon))  
    # rospy.loginfo("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))
    # rospy.loginfo("COORDINATES AXY==>"+str(xa)+","+str(ya))
    # rospy.loginfo("COORDINATES UTM==>"+str(utmx)+","+str(utmy))

    return xg2, yg2
