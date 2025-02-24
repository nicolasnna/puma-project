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
from puma_nav_manager.msg import ImportExportPlanAction, ImportExportPlanGoal, WaypointsManagerAction, WaypointsManagerGoal
import actionlib
import tf
from puma_state_machine.msg import StateMachineAction, StateMachineResult, StateMachineGoal

class PlanConfiguration(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['run_plan', 'run_plan_custom', 'run_parking_station_charge'], input_keys=[], output_keys=['plan_configuration_info'])
    self.configuration_info = {'name_plan': '', 'repeat': 0, 'minutes_between_repeats': 0}
    self.publishers()
    ns_topic = rospy.get_param('~ns_topic', 'state_machine')
    self._srv = actionlib.SimpleActionServer(ns_topic + '/plan_configuration', StateMachineAction, self.execute_srv_cb, auto_start=False)
    self._srv.start()

  def publishers(self):
    ''' Servidores '''
    self.client_files = actionlib.SimpleActionClient('/puma/navigation/files_manager', ImportExportPlanAction)
    self.client_waypoints = actionlib.SimpleActionClient('/puma/navigation/waypoints_manager', WaypointsManagerAction)
    
    ''' Rviz '''
    ns_topic = rospy.get_param('~ns_topic', '')
    self.pose_array_publisher = rospy.Publisher(ns_topic + '/path_planned', PoseArray, queue_size=4)

  def start_subscriber(self):
    ''' Servidores '''
    ns_topic = rospy.get_param('~ns_topic', 'state_machine')
    
    ''' Suscriptores '''
    add_pose_topic = rospy.get_param('~add_pose_topic', '/initialpose')
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
      # self._srv.shutdown()
      self.clear_plan_sub.unregister()
      self.start_plan_sub.unregister()
      self.add_pose_rviz_sub.unregister()
      self.add_waypoints_web_sub.unregister()
      self.configuration_cmd_sub.unregister()
      self.save_plan_sub.unregister()
      self.load_plan_sub.unregister()
      self.charge_mode_sub.unregister()
  
  def send_log(self, msg, level):
    create_and_publish_log(msg, level, 'plan_configuration')
    
  def clear_plan_cb(self, msg):
    try: 
      self.client_waypoints.wait_for_server(rospy.Duration(5))
      action = WaypointsManagerGoal()
      action.action = "clear"
      self.client_waypoints.send_goal(action)
      res = self.client_waypoints.wait_for_result(rospy.Duration(10))
      if res: 
        self.send_log("Waypoints limpiados correctamente.", 0)
        self.configuration_info = {'name_plan': '', 'repeat': 0, 'minutes_between_repeats': 0}
      else:
        self.send_log("No se ha podido limpiar correctamente los waypoints.", 1)
    except Exception as e:
      self.send_log(f"No se ha limpiado correctamente los planes cargados por el error: {e}.", 1)
  
  def start_plan_cb(self, msg):
    try:
      waypoint_plan = rospy.wait_for_message('/puma/navigation/waypoints_list', WaypointNav, timeout=5)
      if waypoint_plan is not None and len(waypoint_plan.waypoints) > 0:
        self.send_log("Waypoints definidos correctamente. Entrando en el modo navegación.", 0)
        self.is_plan_ready = True

    except Exception as e:
      self.send_log(f"No se ha podido comprobar los waypoints  por el error: {e}.", 1)
  
  def save_plan_cb(self, msg):
    exist_waypoints, _ = check_and_get_waypoints(self.send_log)
    rospy.loginfo("-> Recibido el comando de guardar plan: %s", msg.data)

    if exist_waypoints:
      try:
        self.client_files.wait_for_server(rospy.Duration(5))
        action = ImportExportPlanGoal()
        action.action = "export"
        action.file_name = msg.data
        self.client_files.send_goal(action)
        res = self.client_files.wait_for_result(rospy.Duration(10))
        if res:
          self.send_log(f"Plan guardado con el nombre '{msg.data}' correctamente.", 0)
        else: 
          self.send_log(f"No se ha podido guardar el plan con el nombre '{msg.data}'.", 1)
      except Exception as e:
        self.send_log(f"No se ha podido revisar el ultimo plan empleado: {e}", 1)
    else: 
      self.send_log("No se han definido waypoints para guardar.", 1)
        
  def load_plan_cb(self, msg):
    try:
      self.client_files.wait_for_server(rospy.Duration(5))
      action = ImportExportPlanGoal()
      action.action = "import"
      action.file_name = msg.data
      self.client_files.send_goal(action)
      res = self.client_files.wait_for_result(rospy.Duration(10))
      if res:
        self.send_log(f"Plan con el nombre '{msg.data}' cargado correctamente.", 0)
      else:
        self.send_log(f"No se ha podido cargar el plan con el nombre '{msg.data}'.", 1)
        
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
    
    try:
      self.client_waypoints.wait_for_server(rospy.Duration(5))
      action = WaypointsManagerGoal()
      action.action = "add"
      action.waypoint = waypoint
      self.client_waypoints.send_goal(action)
      res = self.client_waypoints.wait_for_result(rospy.Duration(10))
      if res:
        self.send_log(f"Waypoint ({round(waypoint.x,2), round(waypoint.y,2)}) añadido correctamente.", 0)
      else:
        self.send_log(f"No se ha podido añadir el waypoint ({round(waypoint.x,2), round(waypoint.y,2)}).", 1)
    except Exception as e:
      self.send_log(f"No se ha podido añadir el waypoint ({round(waypoint.x,2), round(waypoint.y,2)}) por el error: {e}.", 1)
    
  def add_from_waypoints_web_cb(self, msg):
    rospy.loginfo("-> Recibido arreglo de waypoints desde la web.")
    waypoints = msg.waypoints
    if len(waypoints) > 0:
      try:
        pos_x, pos_y = get_xy_robot()
        latitude_rbt, longitude_rbt = get_lat_lon_robot()
      except Exception as e:
        self.send_log(f"No se ha podido obtener la posición actual del robot: {e}.", 1)
        return
  
      if pos_x is None or pos_y is None or latitude_rbt is None or longitude_rbt is None:
        self.send_log("No se ha podido obtener la posición actual del robot. No se puede continuar con la estimación de global -> local.", 1)
        return
      
      new_waypoints = WaypointNav()
      
      for waypoint in waypoints:
        new_waypoint = Waypoint()
        x2, y2 = get_xy_based_on_lat_long(latitude_rbt, longitude_rbt, waypoint.latitude, waypoint.longitude)
        new_waypoint.x = x2 + pos_x
        new_waypoint.y = y2 + pos_y
        new_waypoint.yaw = -waypoint.yaw * math.pi / 180
        new_waypoint.latitude = waypoint.latitude
        new_waypoint.longitude = waypoint.longitude
        new_waypoints.waypoints.append(new_waypoint)
        
      try:      
        self.client_waypoints.wait_for_server(rospy.Duration(5))
        action = WaypointsManagerGoal()
        action.action = "set"
        action.waypoint_nav = new_waypoints
        self.client_waypoints.send_goal(action)
        res = self.client_waypoints.wait_for_result(rospy.Duration(10))
        if res:
          self.publish_waypoints_rviz(new_waypoints.waypoints)
          self.send_log("Waypoints ajustados a 2d y añadidos correctamente.", 0)
        else:
          self.send_log("No se han podido añadir los waypoints desde la web.", 1)
      except Exception as e:
        self.send_log(f"No se ha podido comprobar los waypoints añadidos: {e}.", 1)
    
  def configuration_cmd_cb(self, msg):
    rospy.loginfo("-> Recibido el comando de configuración: %s", msg.plan_to_load)
    if msg.plan_to_load != '':
      try:
        self.load_plan_cb(String(msg.plan_to_load))
        repeats = 1 if msg.nro_repeats < 1 else msg.nro_repeats
        self.configuration_info = {
          'name_plan': msg.plan_to_load,
          'repeat': repeats,
          'minutes_between_repeats': msg.minutes_between_repeats,
        }  
        self.send_log(f"Plan con el nombre '{msg.plan_to_load}' cargado y configuración para el plan custom ha sido cargado con exito", 0)
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
    if self.configuration_info['name_plan'] != '':
      return 'run_plan_custom'
    return 'run_plan'
  
  
  def execute_srv_cb(self, goal):
    result = StateMachineResult()
    try:
      rospy.loginfo("-> Recibido el comando de configuración: %s", goal.action)
      result.success = True
      
      if goal.action == StateMachineGoal.START_PLAN:
        self.start_plan_cb(Empty())
      elif goal.action == StateMachineGoal.CLEAR_PLAN:
        self.clear_plan_cb(Empty())
      elif goal.action == StateMachineGoal.ADD_WEB:
        self.add_from_waypoints_web_cb(goal.waypoint_nav)
      elif goal.action == StateMachineGoal.SAVE_PLAN:
        self.save_plan_cb(goal.file_name)
      elif goal.action == StateMachineGoal.LOAD_PLAN:
        self.load_plan_cb(goal.file_name)
      elif goal.action == StateMachineGoal.CONFIG_PLAN:
        self.configuration_cmd_cb(goal.configuration_plan)
      else:
        rospy.loginfo("-> Acción no reconocida.")
        result.success = False
        result.message = "Acción no reconocida."
      
      self._srv.set_succeeded(result)
    except Exception as e:
      result.success = False
      result.message = f"Error: {e}"
      rospy.logerr(f"Error: {e}")
      self._srv.set_aborted(result)
  