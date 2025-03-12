#/usr/bin/env python3
import rospy
import actionlib
from puma_robot_status.msg import RobotStatisticsAction, RobotStatisticsResult, RobotStatisticsGoal
from sensor_msgs.msg import BatteryState, NavSatFix, CompressedImage
from nav_msgs.msg import Odometry
from puma_msgs.msg import StatusArduino, WaypointNav, Waypoint
from std_msgs.msg import String
import base64
import json
import rospkg
import requests
from datetime import datetime
from pathlib import Path
try:
  from zoneinfo import ZoneInfo
except ImportError:
  from backports.zoneinfo import ZoneInfo
  
def time_chile_now():
  return datetime.now(ZoneInfo("Chile/Continental")).strftime("%Y-%m-%d %H:%M:%S")

def waypoint_to_dict(waypoint: Waypoint):
  return {
    "x": waypoint.x,
    "y": waypoint.y,
    "yaw": waypoint.yaw,
    "latitude": waypoint.latitude,
    "longitude": waypoint.longitude,
  }

def execute_cb(goal: RobotStatisticsGoal):
  global run_statistics, server, type_statistics
  result = RobotStatisticsResult()
  is_simulated = rospy.get_param('~is_simulated', True)

  type_statistics = goal.type

  if goal.action == 'start':
    if not run_statistics:
      subscriber(is_simulated)
    result.success = True if not run_statistics else False
    result.message = 'Ha empezado la recolección de estadísticas' if not run_statistics else 'Ya se está recolectando estadísticas'
    run_statistics = True
  elif goal.action == 'stop':
    if run_statistics:
      if save_statistics():
        end_subscriber()
        result.success = True
        result.message = 'Estadísticas guardadas y recolección detenida'
        run_statistics = False
      else:
        result.success = False
        result.message = 'Error al guardar las estadísticas'
    else:
      result.success = False
      result.message = 'No se estaba recolectando estadísticas'
  else:
    result.success = False
    result.message = 'Acción no reconocida'
  server.set_succeeded(result)
  
def statistics_manager():
  global run_statistics, statistics, battery, gps, angle, vel_x, distance, front_color, rear_color, front_depth, rear_depth, last_time_odom
  if run_statistics:
    statistics.append({
      "battery": battery,
      "gps": gps,
      "angle": angle,
      "vel_x": vel_x,
      "distance": distance,
      "front_color": front_color,
      "rear_color": rear_color,
      "front_depth": front_depth,
      "rear_depth": rear_depth,
      "wp_completed": wp_completed,
      "wp_remained": wp_remained,
      "wp_list": wp_list,
      "control_mode": control_mode,
      "time": time_chile_now()
    })
  else:
    statistics = []

def battery_cb(msg: BatteryState):
  global battery
  battery = msg.percentage
  
def gps_cb(msg: NavSatFix):
  global gps
  gps = {"latitude": msg.latitude, "longitude": msg.longitude, "altitude": msg.altitude}

def arduino_cb(msg: StatusArduino):
  global angle
  angle = msg.direction.degree_angle

def odom_cb(msg: Odometry):
  global vel_x, distance, last_time_odom
  current_time = msg.header.stamp
  if last_time_odom is None:
    last_time_odom = current_time
  else:
    distance += vel_x * (current_time - last_time_odom).to_sec()
    last_time_odom = current_time
  vel_x = msg.twist.twist.linear.x

def rs_front_color_cb(msg: CompressedImage):
  global front_color
  front_color = base64.b64encode(msg.data).decode('utf-8')
  
def rs_rear_color_cb(msg: CompressedImage):
  global rear_color
  rear_color = base64.b64encode(msg.data).decode('utf-8')

def rs_front_depth_cb(msg: CompressedImage):
  global front_depth
  front_depth = base64.b64encode(msg.data).decode('utf-8')

def rs_rear_depth_cb(msg: CompressedImage):
  global rear_depth
  rear_depth = base64.b64encode(msg.data).decode('utf-8')
  
def wp_completed_cb(msg: WaypointNav):
  global wp_completed
  wp_completed = [waypoint_to_dict(wp) for wp in msg.waypoints]

def wp_remained_cb(msg: WaypointNav):
  global wp_remained
  wp_remained = [waypoint_to_dict(wp) for wp in msg.waypoints]

def wp_list_cb(msg: WaypointNav):
  global wp_list
  wp_list = [waypoint_to_dict(wp) for wp in msg.waypoints]
  
def mode_control_cb(msg: String):
  global control_mode
  control_mode = msg.data
  
def subscriber(is_simulated: bool):
  global battery_sub, gps_sub, odom, status_arduino, rs_front_color, rs_rear_color, rs_front_depth, rs_rear_depth, wp_complete_sub, wp_remained_sub, wp_list_sub, control_mode_sub
  init_values()
  
  battery_sub = rospy.Subscriber('/puma/sensors/battery/status', BatteryState, battery_cb)
  gps_sub = rospy.Subscriber('/puma/sensors/gps/fix', NavSatFix, gps_cb)
  odom = rospy.Subscriber('/puma/localization/ekf_odometry', Odometry, odom_cb)
  status_arduino = rospy.Subscriber('/puma/arduino/status', StatusArduino, arduino_cb)
  
  ''' Navegación '''
  wp_complete_sub = rospy.Subscriber("/puma/navigation/waypoints_completed", WaypointNav, wp_completed_cb )
  wp_remained_sub = rospy.Subscriber("/puma/navigation/waypoints_remained", WaypointNav, wp_remained_cb)
  wp_list_sub = rospy.Subscriber("/puma/navigation/waypoints_list", WaypointNav, wp_list_cb)
  control_mode_sub = rospy.Subscriber("/puma/control/current_mode", String, mode_control_cb)
  
  ''' Cámaras '''
  rs_front_color = rospy.Subscriber('/puma/sensors/camera_front/color/image_raw/compressed', CompressedImage, rs_front_color_cb)
  rs_rear_color = rospy.Subscriber('/puma/sensors/camera_rear/color/image_raw/compressed', CompressedImage, rs_rear_color_cb)
  if is_simulated:
    rs_front_depth = rospy.Subscriber('/puma/sensors/camera_front/aligned_depth_to_color/image_raw/compressed', CompressedImage, rs_front_depth_cb)
    rs_rear_depth = rospy.Subscriber('/puma/sensors/camera_rear/aligned_depth_to_color/image_raw/compressed', CompressedImage, rs_rear_depth_cb)
  else:
    rs_front_depth = rospy.Subscriber('/puma/sensors/camera_front/depth/image_rect_raw/compressed', CompressedImage, rs_front_depth_cb)
    rs_rear_depth = rospy.Subscriber('/puma/sensors/camera_rear/depth/image_rect_raw/compressed', CompressedImage, rs_rear_depth_cb)
  
def end_subscriber():
  global battery_sub, gps_sub, odom, status_arduino, rs_front_color, rs_rear_color, rs_front_depth, rs_rear_depth, is_run_statistics, wp_complete_sub, wp_remained_sub, wp_list_sub, control_mode_sub, start_at, end_at
  try:
    battery_sub.unregister()
    gps_sub.unregister()
    odom.unregister()
    status_arduino.unregister()
    rs_front_color.unregister()
    rs_rear_color.unregister()
    rs_front_depth.unregister()
    rs_rear_depth.unregister()
    wp_complete_sub.unregister()
    wp_remained_sub.unregister()
    wp_list_sub.unregister()
    control_mode_sub.unregister()
  except Exception as e:
    rospy.logerr(f'Error al detener los subscriptores: {e}')
  is_run_statistics = False
  start_at = end_at = None
  
def save_statistics():
  global type_statistics, statistics, end_at
  dir_local = rospkg.RosPack().get_path('puma_robot_status') + '/tmp/statistics'
  statistics_manager()
  name_file = f'{type_statistics} - {str(time_chile_now())}'
  end_at = time_chile_now()
  data_export = {"type": type_statistics, "data": statistics, "start_at": start_at, "end_at": end_at, "name": name_file}
  json_data = json.dumps(data_export)
  
  try:
    file_path = f'{dir_local}/{name_file}.json'
    with open(file_path, 'w') as file:
      json.dump(data_export, file, indent=2, sort_keys=True)
    rospy.loginfo(f'Estadísticas guardadas en {file_path}')
  except Exception as e:
    rospy.logerr(f'Error al guardar las estadísticas: {e}')
    return False
    
  if rospy.get_param('~upload_db', False):
    backend_url = rospy.get_param('~backend_url', 'http://localhost:8000')
    token = get_token(backend_url)
    while not token:
      rospy.logwarn('No se ha obtenido el token, reintentando en 3 seg...')
      rospy.Rate(3).sleep()
      token = get_token(backend_url)
    headers = { 'Content-Type': 'application/json', 'Authorization': f'Bearer {str(token)}' }
    try:
      res = requests.post(backend_url+'/database/statistics', headers=headers, data=json_data, timeout=5)
      if res.status_code != 200:
        rospy.logwarn(f'Error al subir las estadísticas a la base de datos: {res.status_code} - {res.text}')
        return False
      return True
    except rospy.ServiceException as e:
      rospy.logerr(f'Error al subir las estadísticas a la base de datos')
      return False
  return True

def get_token(BACKEND_URL):
  headers = { 'Content-Type': 'application/x-www-form-urlencoded'}
  body = { 'username': 'admin', 'password': 'admin'}
  try:
    response = requests.post(BACKEND_URL+"/auth/login", headers=headers, data=body, timeout=5)
    if response.status_code == 200:
      return response.json()['access_token']
    else:
      rospy.logwarn(f"Error al obtener token: {response.status_code} - {response.text}")
  except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Error al obtener token: {e}")
    
def init_values():
  global battery, gps, angle, vel_x, distance, front_color, rear_color, front_depth, rear_depth, last_time_odom, wp_completed, wp_remained, wp_list, control_mode, start_at
  battery = None
  gps = None
  angle = None
  vel_x = None
  distance = 0
  front_color = None
  rear_color = None
  front_depth = None
  rear_depth = None
  last_time_odom = None
  wp_completed = None
  wp_remained = None
  wp_list = None
  control_mode = None
  start_at = time_chile_now()
    
def main():
  rospy.init_node('robot_statistics_node')
  global server, statistics, run_statistics, start_at, end_at
  statistics = []
  run_statistics = False
  start_at = end_at = None
  init_values()
  server = actionlib.SimpleActionServer('/puma/statistics', RobotStatisticsAction, execute_cb=execute_cb, auto_start=False)
  server.start()
  rospy.loginfo("Iniciando servidor para el manejo de estadísticas")
  # Crear directorio para guardar los archivos
  project_root = Path(__file__).resolve().parent.parent  # Sube a puma_robot_status
  tmp_dir = project_root / "tmp" / "statistics"
  tmp_dir.mkdir(parents=True, exist_ok=True)
  
  while not rospy.is_shutdown():
    statistics_manager()
    rospy.Rate(1).sleep()
    
if __name__ == "__main__":
  main()