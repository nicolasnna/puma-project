#!/usr/bin/env python3
import rospy
import smach
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from puma_msgs.msg import GoalGpsArray, GoalGpsNavInfo, GoalGps, Log, WaypointNav, Waypoint, ConfigurationStateMachine
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pathlib import Path
from puma_state_machine.utils import calc_goal_from_gps, calculate_bearing_from_xy, yaw_to_quaternion
import tf

class PlanConfiguration(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['run_plan', 'run_plan_schedule'], input_keys=[], output_keys=['plan_configuration_info'])
    
    self.publishers()

  def publishers(self):
    
    # Logs
    self.log_pub = rospy.Publisher('/puma/logs/add_log', Log, queue_size=2)
    # files manager
    self.file_plan_clear_pub = rospy.Publisher('/puma/navigation/files/clear_plan', Empty, queue_size=2)
    self.file_import_local_pub = rospy.Publisher('/puma/navigation/files/import_plan_local', String, queue_size=2)
    # waypoints manager
    self.waypoints_clear_pub = rospy.Publisher('/puma/navigation/waypoints/clear', Empty, queue_size=2)
    self.waypoints_add_pub = rospy.Publisher('/puma/navigation/waypoints/add', Waypoint, queue_size=2)

  def start_subscriber(self):
    add_pose_topic = rospy.get_param('add_pose_topic', '/initialpose')
    ns_topic = rospy.get_param('ns_topic', '/ns')
    self.clear_plan_sub = rospy.Subscriber(ns_topic + '/clear_plan', Empty, self.clear_plan_cb)
    self.start_plan_sub = rospy.Subscriber(ns_topic + '/start_plan', Empty, self.start_plan_cb)
    self.add_pose_rviz_sub = rospy.Subscriber(add_pose_topic, PoseWithCovarianceStamped, self.add_pose_rviz_cb)
    self.configuration_cmd_sub = rospy.Subscriber(ns_topic + '/configuration_cmd', ConfigurationStateMachine, self.configuration_cmd_cb)
    
  def end_subscriber(self):
    if self.clear_plan_sub is not None:
      self.clear_plan_sub.unregister()
      self.start_plan_sub.unregister()
      self.add_pose_rviz_sub.unregister()
  
  def send_log(self, msg, level):
    log = Log()
    log.level = level
    log.content = msg
    log.node = rospy.get_name()+"/plan_configuration"
    self.log_pub.publish(log)
    if level == 0:
      rospy.loginfo(msg)
    elif level == 1:
      rospy.logwarn(msg)
    elif level == 2:
      rospy.logerr(msg)
    
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
      if file_plan is not None and len(file_plan.poses) > 0 and waypoint_plan is not None and len(waypoint_plan.waypoints) > 0:
        self.send_log("Plan cargado correctamente.", 0)
        self.is_plan_ready = True

    except Exception as e:
      self.send_log(f"No se ha cargado correctamente los planes por el error: {e}.", 1)
    
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
      if len(waypoints_msgs.waypoints) > 0 and waypoints_msgs[-1] == waypoint:
        self.send_log("Waypoint añadido correctamente.", 0)
      else:
        self.send_log("No se ha añadido correctamente el waypoint.", 1)
    except Exception as e:
      self.send_log(f"No se ha logrado revisar el estado de los waypoints {e}.", 1)
    
  def configuration_cmd_cb(self, msg):
    if msg.plan_to_load != '':
      if msg.load_from_plan == 0:
        self.file_import_local_pub.publish(String(msg.plan_to_load))

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
          self.send_log("No se ha cargado el plan correctamente.", 1)
      except Exception as e:
        self.send_log(f"No se ha podido comprobar el plan cargado: {e}.",1)
        
    else:
      self.send_log("No se ha especificado un plan a cargar.", 1)
    
  def execute(self, userdata):
    """ Configura el plan de navegación """
    rospy.loginfo('----- Estado Configuración de plan -----')
    self.send_log("Iniciando en el estado de configuración de plan de navegación.", 0)
    
    self.is_plan_ready = False
    self.configuration_info = {'name_plan': '', 'repeat': 0, 'minutes_between_repeats': 0}
    self.start_subscriber()
    
    while not self.is_plan_ready and not rospy.is_shutdown():
      rospy.Rate(1).sleep()
      
    self.end_subscriber()
    
    userdata.plan_configuration_info = self.configuration_info
    if self.configuration_info['repeat'] > 0:
      return 'run_plan_schedule'
    return 'run_plan'