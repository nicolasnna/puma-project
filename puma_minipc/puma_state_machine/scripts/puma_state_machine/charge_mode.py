#!/usr/bin/env python3
import rospy
import smach
import time
import actionlib
import roslaunch
import math
from std_msgs.msg import Empty, Bool
from apriltag_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, Twist

class ChargeMode(smach.State):
  ''' State for charge car mode '''
  def __init__(self):
    smach.State.__init__(self, outcomes=['finish_charge'], input_keys=['waypoints'], output_keys=['waypoints'])
    enable_topic = rospy.get_param('~enable_topic', "/puma/tag_detector/enable")
    self.enable_pub = rospy.Publisher(enable_topic, Bool, queue_size=3)
    self.cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
  
  def start_subscriber(self):
    ns_topic = rospy.get_param('ns_topic', '')
    self.stop_plan = rospy.Subscriber(ns_topic+'/plan_stop', Empty, self.stop_plan_callback)
    
  def end_subscriber(self):
    self.stop_plan.unregister()
    
  def stop_plan_callback(self):
    rospy.loginfo("-> Recibido comando de detencion.")
    self.client_movebase.cancel_all_goals()
    self.stop = True
  
  def init_apriltag(self):
    # Configure roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
    roslaunch.configure_logging(uuid)
    self.launch = roslaunch.scriptapi.ROSLaunch()
    cli_apriltag = ['puma_tag_detector', 'apriltag_detection.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_apriltag)
    self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file1)
    self.launch.start()
    
  def init_pid(self):
    ''' Initial values for pid controller '''
    self.dis2camera = 10
    self.dis2tag_goal = rospy.get_param('~dis2tag_goal', 1.0)
    self.integral_error = 0.0
    self.prev_error = 0
    self.pid_kp = rospy.get_param('~pid/kp', 0.1)
    self.pid_ki = rospy.get_param('~pid/ki', 0.01)
    self.pid_kd = rospy.get_param('~pid/kd', 0.05)
    self.current_time = rospy.get_time()
    self.max_vel = rospy.get_param('~max_vel', 0.4)
  
  def compute_pid_position(self):
    ''' Compute value pid and publish cmd_vel '''
    error = self.dis2camera - self.dis2tag_goal
    self.integral_error = error*self.diff_time + self.integral_error
    derivative = (error - self.prev_error) / self.diff_time
    # Get pid result
    output_pid = self.pid_kp * error + self.pid_ki * self.integral_error + self.pid_kd * derivative
    # Save prev error
    self.prev_error = error
    # Validate limits of output
    vel_x = max(min(output_pid, self.max_vel), -self.max_vel)
    self.publish_cmd_vel(vel_x)
    
  def publish_cmd_vel(self, linear_vel_x):
    vel_msg = Twist()
    vel_msg.linear.x = linear_vel_x
    vel_msg.angular.z = 0
    self.cmdvel_pub.publish(vel_msg)
  
  def callback_dis2camera(self, pos_received):
    ''' Calculate distance to tag according to pos received'''
    x = pos_received.pose.position.x
    y = pos_received.pose.position.y
    self.diff_time = rospy.get_time() - self.current_time
    self.current_time = rospy.get_time()
    self.dis2camera = math.sqrt( x**2 + y**2)
  
  def activate_transform_tag(self, is_activate):
    ''' Change enable mode for transform tag '''
    msg_enable = Bool()
    msg_enable.data = is_activate
    self.enable_pub.publish(msg_enable)
  
  def execute(self, ud):
    rospy.loginfo("Enter in charge mode")
    self.init_pid()
    self.init_apriltag()
    self.activate_transform_tag(is_activate=True)
    self.stop = False
    self.client_movebase = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.start_subscriber()
    detections = []
    
    try:
      ''' Esperando la deteccion de apriltag '''
      while len(detections) == 0 and not self.stop:
        rospy.loginfo_throttle(5,"-> Esperando por la deteccion de apriltag...")
        msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray,timeout=3)
        if len(msg.detections)>0:
          rospy.loginfo("-> Se ha detectado un apriltag")
          detections = msg.detections
        time.sleep(0.5)
      
      if not self.stop:
        ''' Procesando deteccion '''
        id_tag = detections[0].id[0]
        rospy.loginfo("-> Crear subscriptor del tag")
        tag_sub = rospy.Subscriber('/puma/tag_detector/pose2camera/tag_'+str(id_tag),PoseStamped, self.callback_dis2camera)
        goal_tag = rospy.wait_for_message('/puma/tag_detector/goal/tag_'+str(id_tag), MoveBaseGoal)
      
      if not self.stop:
        ''' Acercandose al tag con move_base '''
        rospy.loginfo("-> Conectando a move_base...")
        self.client_movebase.wait_for_server()
        rospy.loginfo("-> Conexión realizada. Enviando destino...")
        self.client_movebase.send_goal(goal_tag)
        self.client_movebase.wait_for_result()
        rospy.loginfo("-> Destino alcanzado!")
        time.sleep(0.5)
      
      ''' Moverse en linea recta al tag '''
      if not self.stop:
        rospy.loginfo("-> Cambiando a PID para estacionar...")
        while not rospy.is_shutdown() and not self.stop and (self.dis2camera > self.dis2tag_goal):
          rospy.loginfo_throttle(4,"--> Distancia restante: %.3f mts...", self.dis2camera-self.dis2tag_goal)
          self.compute_pid_position()
          rospy.Rate(30).sleep()
        self.publish_cmd_vel(0)
        rospy.loginfo("-> Estacionamiento completo!!")
    except:
      rospy.logwarn("-> Cerrando puma_waypoints -- CHARGE_MODE.")
    
    if tag_sub is not None:
      tag_sub.unregister()
    self.activate_transform_tag(False)
    self.launch.stop()
    self.end_subscriber()
    return 'finish_charge'