#!/usr/bin/env python3
import math
from geographiclib.geodesic import Geodesic

def calc_goal_from_gps(origin_lat, origin_long, goal_lat, goal_long):
  # Source: https://github.com/danielsnider/gps_goal/blob/master/src/gps_goal/gps_goal.py
  geod = Geodesic.WGS84
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
  hypotenuse = distance = g['s12']
  azimuth = g['azi1']

  azimuth = math.radians(azimuth)

  y = adjacent = math.cos(azimuth) * hypotenuse
  x = opposite = math.sin(azimuth) * hypotenuse

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

