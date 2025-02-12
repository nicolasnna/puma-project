#!/usr/bin/env python3
import rospy
import smach
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from puma_msgs.msg import Log, WaypointNav, Waypoint, ConfigurationStateMachine
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry
from puma_state_machine.utils import *
import tf

class PlanConfiguration(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['run_plan', 'run_plan_custom', 'run_parking_station_charge'], input_keys=[], output_keys=['plan_configuration_info'])
    self.configuration_info = {'name_plan': '', 'repeat': 0, 'minutes_between_repeats': 0}
    self.publishers()

  def publishers(self):
    # files manager
    self.file_plan_clear_pub = rospy.Publisher('/puma/navigation/files/clear_plan', Empty, queue_size=2)
    self.file_import_local_pub = rospy.Publisher('/puma/navigation/files/import_plan_local', String, queue_size=2)
    self.file_export_local_pub = rospy.Publisher('/puma/navigation/files/export_plan_local', String, queue_size=2)
    self.file_set_plan_pub = rospy.Publisher('/puma/navigation/files/set_plan', Path, queue_size=2)
    # waypoints manager
    self.waypoints_clear_pub = rospy.Publisher('/puma/navigation/waypoints/clear', Empty, queue_size=2)
    self.waypoints_add_pub = rospy.Publisher('/puma/navigation/waypoints/add', Waypoint, queue_size=2)
    self.waypoints_set_pub = rospy.Publisher('/puma/navigation/waypoints/set', WaypointNav, queue_size=2)
    # rviz
    ns_topic = rospy.get_param('~ns_topic', '')
    self.pose_array_publisher = rospy.Publisher(ns_topic + '/path_planned', PoseArray, queue_size=4)

  def start_subscriber(self):
    add_pose_topic = rospy.get_param('~add_pose_topic', '/initialpose')
    ns_topic = rospy.get_param('~ns_topic', '')
    self.clear_plan_sub = rospy.Subscriber(ns_topic + '/clear_plan', Empty, self.clear_plan_cb)
    self.start_plan_sub = rospy.Subscriber(ns_topic + '/start_plan', Empty, self.start_plan_cb)
    self.add_pose_rviz_sub = rospy.Subscriber(add_pose_topic, PoseWithCovarianceStamped, self.add_pose_rviz_cb)
    self.add_waypoints_web_sub = rospy.Subscriber(ns_topic + '/add_waypoints_web', WaypointNav, self.add_from_waypoints_web_cb)
    self.configuration_cmd_sub = rospy.Subscriber(ns_topic + '/configuration_cmd', ConfigurationStateMachine, self.configuration_cmd_cb)
    self.save_plan_sub = rospy.Subscriber(ns_topic + '/save_plan', String, self.save_plan_cb)
    self.load_plan_sub = rospy.Subscriber(ns_topic + '/load_plan', String, self.load_plan_cb)
    self.charge_mode_sub = rospy.Subscriber(ns_topic + "/run_charge_mode", Empty, self.charge_mode_cb)

  def charge_mode_cb(self, msg):
    rospy.loginfo("-> Recibido comando de carga.")
    self.charge_mode_on = True
    
  def end_subscriber(self):
    if self.clear_plan_sub is not None:
      self.clear_plan_sub.unregister()
      self.start_plan_sub.unregister()
      self.add_pose_rviz_sub.unregister()
  
  def send_log(self, msg, level):
    create_and_publish_log(msg, level, 'plan_configuration')
    
  def clear_plan_cb(self, msg):
    self.file_plan_clear_pub.publish(Empty())
    self.waypoints_clear_pub.publish(Empty())
    # Comprobar si se ha limpiado el plan
    try: 
      file_plan = rospy.wait_for_message('/puma/navigation/files/plan_selected', Path, timeout=5)
      waypoint_plan = rospy.wait_for_message('/puma/navigation/waypoints/waypoints_info', WaypointNav, timeout=5)
      if len(file_plan.poses) == 0 and len(waypoint_plan.waypoints) == 0:
        self.send_log("Plan y waypoints limpiados.", 0)
      else:
        self.send_log("No se ha limpiado correctamente los planes cargados.", 1)
    except Exception as e:
      self.send_log(f"No se ha limpiado correctamente los planes cargados por el error: {e}.", 1)
  
  def start_plan_cb(self, msg):
    try:
      file_plan = rospy.wait_for_message('/puma/navigation/files/plan_selected', Path, timeout=5)
      waypoint_plan = rospy.wait_for_message('/puma/navigation/waypoints/waypoints_info', WaypointNav, timeout=5)
      if file_plan is not None and len(file_plan.poses) > 0:
        self.send_log("Se ha detectado un plan cargado.", 0)
      if waypoint_plan is not None and len(waypoint_plan.waypoints) > 0:
        self.send_log("Waypoints definidos correctamente.", 0)
        self.is_plan_ready = True

    except Exception as e:
      self.send_log(f"No se ha cargado correctamente los planes por el error: {e}.", 1)
  
  def save_plan_cb(self, msg):
    exist_waypoints, _ = check_and_get_waypoints(self.send_log)
    
    if exist_waypoints:
      try:
        plan_calculated = rospy.wait_for_message('/puma/navigation/plan/plan_info', Path, timeout=5)
        self.file_plan_clear_pub.publish(Empty())
        rospy.sleep(0.2)
        if plan_calculated is not None and len(plan_calculated.poses) > 0:
          self.file_set_plan_pub.publish(plan_calculated)
          rospy.sleep(0.2)
          if check_plan_load_file():
            self.file_export_local_pub.publish(msg)
        
      except Exception as e:
        self.send_log(f"No se ha podido revisar el ultimo plan empleado: {e}", 1)
        
  def load_plan_cb(self, msg):
    self.file_plan_clear_pub.publish(Empty())
    rospy.sleep(0.2)
    self.file_import_local_pub.publish(msg)
    rospy.sleep(0.2)
    try:
      plan_calculated = rospy.wait_for_message('/puma/navigation/plan/plan_info', Path, timeout=5)
      if plan_calculated is not None and len(plan_calculated.poses) > 0:
        self.send_log(f"Plan con el nombre '{msg.data}' cargado con exito", 0)
      else:
        self.send_log(f"No se ha cargado el plan {msg.data} correctamente.", 1)
    except Exception as e:
      self.send_log(f"No se ha podido comprobar el plan cargado: {e}.",1)
  
  def add_pose_rviz_cb(self, pose):
    rospy.loginfo("-> Recibido el waypoint x: %.3f, y: %.3f", pose.pose.pose.position.x, pose.pose.pose.position.y)
    
    orientation = pose.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    waypoint = Waypoint()
    waypoint.x = pose.pose.pose.position.x
    waypoint.y = pose.pose.pose.position.y
    waypoint.yaw = euler[2] # yaw
    
    self.waypoints_add_pub.publish(waypoint)
    rospy.sleep(0.2)
    try:
      waypoints_msgs = rospy.wait_for_message('/puma/navigation/waypoints/waypoints_info', WaypointNav, timeout=5)
      if len(waypoints_msgs.waypoints) > 0 and waypoints_msgs.waypoints[-1] == waypoint:
        self.send_log("Waypoint añadido correctamente.", 0)
      else:
        self.send_log("No se ha añadido correctamente el waypoint.", 1)
    except Exception as e:
      self.send_log(f"No se ha logrado revisar el estado de los waypoints {e}.", 1)
    
  def add_from_waypoints_web_cb(self, msg):
    rospy.loginfo("-> Recibido arreglo de waypoints desde la web.")
    waypoints = msg.waypoints
    if len(waypoints) > 0:
      try:
        gps_topic = rospy.get_param("~gps_topic",'/puma/sensors/gps/fix')
        odom_topic = rospy.get_param("~odom_topic",'/puma/localization/ekf_odometry')
        gps_robot = rospy.wait_for_message(gps_topic, NavSatFix, timeout=5)
        odom_robot = rospy.wait_for_message(odom_topic, Odometry, timeout=5)
      except Exception as e:
        self.send_log(f"No se ha podido obtener la posición actual del robot: {e}.", 1)
        return
      
      pos_x = odom_robot.pose.pose.position.x
      pos_y = odom_robot.pose.pose.position.y
      latitude_rbt = gps_robot.latitude
      longitude_rbt = gps_robot.longitude
      new_waypoints = WaypointNav()
      
      for waypoint in waypoints:
        new_waypoint = Waypoint()
        x, y = calc_goal_from_gps(latitude_rbt, longitude_rbt, waypoint.latitude, waypoint.longitude)
        x2, y2 = get_xy_based_on_lat_long(latitude_rbt, longitude_rbt, waypoint.latitude, waypoint.longitude)
        rospy.loginfo(f"-> Waypoint en distintos metodos: x: {x}, y: {y}, x2: {x2}, y2: {y2}")
        new_waypoint.x = y + pos_x
        new_waypoint.y = x + pos_y
        new_waypoint.yaw = -waypoint.yaw * math.pi / 180
        new_waypoint.latitude = waypoint.latitude
        new_waypoint.longitude = waypoint.longitude
        new_waypoints.waypoints.append(new_waypoint)
        
      self.waypoints_set_pub.publish(new_waypoints)
      self.publish_waypoints_rviz(new_waypoints.waypoints)
      rospy.sleep(0.2)
      try: 
        is_correct, _ = check_and_get_waypoints(self.send_log)
        if is_correct:
          self.send_log("Waypoints ajustados a 2d y añadidos correctamente.", 0)
      except Exception as e:
        self.send_log(f"No se ha podido comprobar los waypoints añadidos: {e}.", 1)
    
  def configuration_cmd_cb(self, msg):
    rospy.loginfo("-> Recibido el comando de configuración: %s", msg.plan_to_load)
    if msg.plan_to_load != '':
      if msg.load_plan_from == 0:
        self.file_import_local_pub.publish(String(msg.plan_to_load))
      rospy.sleep(0.2)
      try:
        current_plan_loader = rospy.wait_for_message('/puma/navigation/files/plan_selected', Path, timeout=5)
        if current_plan_loader is not None and len(current_plan_loader.poses) > 0:
          self.configuration_info = {
            'name_plan': msg.plan_to_load,
            'repeat': msg.nro_repeats,
            'minutes_between_repeats': msg.minutes_between_repeats,
          }  
          self.send_log(f"Plan con el nombre '{msg.plan_to_load}' cargado con exito", 0)
        else:
          self.send_log(f"No se ha cargado el plan correctamente.", 1)
      except Exception as e:
        self.send_log(f"No se ha podido comprobar el plan cargado: {e}.",1)
        
    else:
      self.send_log("No se ha especificado un plan a cargar.", 1)
  
  def publish_waypoints_rviz(self, waypoints):
    pose_array = PoseArray()
    pose_array.header.frame_id = 'map'
    pose_array.header.stamp = rospy.Time.now()
    
    for waypoint in waypoints:
      pose = PoseStamped()
      pose.header.frame_id = 'map'
      pose.pose.position.x = waypoint.x
      pose.pose.position.y = waypoint.y
      pose.pose.position.z = 0
      
      quaternion = tf.transformations.quaternion_from_euler(0, 0, waypoint.yaw)
      pose.pose.orientation.x = quaternion[0]
      pose.pose.orientation.y = quaternion[1]
      pose.pose.orientation.z = quaternion[2]
      pose.pose.orientation.w = quaternion[3]
      
      pose_array.poses.append(pose.pose)
    
    self.pose_array_publisher.publish(pose_array)
  
  def execute(self, userdata):
    """ Configura el plan de navegación """
    rospy.loginfo('----- Estado Configuración de plan -----')
    self.send_log("Iniciando en el estado de configuración de plan de navegación.", 0)
    
    self.charge_mode_on = False
    self.is_plan_ready = False
    self.start_subscriber()
    
    while not self.is_plan_ready and not rospy.is_shutdown() and not self.charge_mode_on:
      rospy.Rate(1).sleep()
      
    self.end_subscriber()
    
    if self.charge_mode_on:
      return 'run_parking_station_charge'
    
    userdata.plan_configuration_info = self.configuration_info
    if self.configuration_info['repeat'] > 0:
      return 'run_plan_custom'
    return 'run_plan'