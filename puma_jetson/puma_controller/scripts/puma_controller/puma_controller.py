#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from puma_msgs.msg import DirectionCmd, WebControl, Log
from std_msgs.msg import Bool, Int16, String
from nav_msgs.msg import Odometry
from puma_controller.pid_antiwindup import PIDAntiWindUp
from geometry_msgs.msg import Twist;
import time

class PumaController():
  def __init__(self):
    ns = '/puma_controller'
    # Get params
    accelerator_topic = rospy.get_param(ns+'/accelerator_topic', 'puma/accelerator/command')
    parking_topic = rospy.get_param(ns+'/parking_topic', 'puma/parking/command')
    reverse_topic = rospy.get_param(ns+'/revese_topic', 'puma/reverse/command')
    ackermann_topic = rospy.get_param(ns+'/ackermann_topic', 'puma/control/ackermann/command')
    direction_topic = rospy.get_param(ns+'/direction_topic', 'puma/direction/command')
    brake_topic = rospy.get_param(ns+'/brake_topic', 'puma/brake/command')
    self.range_accel_converter = rospy.get_param(ns+'/range_accel_converter', [22,35])
    self.connect_to_ackermann_converter = rospy.get_param(ns+'/connect_to_ackermann_converter', False)
  
    # Subscribers
    rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_callback)
    rospy.Subscriber('/puma/odometry/filtered', Odometry, self.odometry_callback)
    rospy.Subscriber('/puma/control/current_mode', String, self.selector_mode_callback)
    rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)

    # Publishers
    self.reverse_pub = rospy.Publisher(reverse_topic, Bool, queue_size=10)
    self.parking_pub = rospy.Publisher(parking_topic, Bool, queue_size=10)
    self.accel_pub = rospy.Publisher(accelerator_topic, Int16, queue_size=10)
    self.direction_pub = rospy.Publisher(direction_topic, DirectionCmd, queue_size=5)
    self.brake_pub = rospy.Publisher(brake_topic, Bool, queue_size=5)
    self.logs_pub = rospy.Publisher('puma/logs/add_log', Log, queue_size=2)
    # Variable
    self.vel_linear = 0
    self.angle = 0
    self.vel_linear_odometry = 0
    self.mode_puma = ''
    
    self.pid = PIDAntiWindUp(
      kp=rospy.get_param('~kp', 0.3), 
      ki=rospy.get_param('~ki', 0.2), 
      kd=rospy.get_param('~kd', 0.05), 
      min_value=self.range_accel_converter[0], 
      max_value=self.range_accel_converter[1]
    )

    self.last_time_odometry = self.last_time_ackermann = self.last_time_publish = 0
    self.is_change_reverse = False
    
    self.publish_log(0, "Iniciando controlador robot puma. Recordar definir el modo de control.")

  def publish_log(self, level, content):
    log_msg = Log()
    log_msg.level =level
    log_msg.node = rospy.get_name()
    log_msg.content = content
    time.sleep(0.1)
    self.logs_pub.publish(log_msg)

  def selector_mode_callback(self, mode):
    if self.mode_puma != mode.data:
      if mode.data == "navegacion" or mode.data == "web":
        log_content = "Modo navegacion detectado, ejecutando control puma con PID." if mode.data == "navegacion" else "Modo web detectado, ejecutando control puma con datos recibidos de la web."
        self.publish_log(0, log_content)
      if self.mode_puma == "navegacion" or self.mode_puma == "web" :
        log_content = "Saliendo del modo navegacion. Desactivando control puma con PID." if self.mode_puma == "navegacion" else "Saliendo del modo web. Desactivando control a partir de la web."
        self.publish_log(1, log_content)
    self.mode_puma = mode.data
  
  def odometry_callback(self, odom):
    """ Get current velocity and calculate break value"""
    self.last_time_odometry = time.time()
    self.vel_linear_odometry = round(odom.twist.twist.linear.x,4)
    
  def cmdvel_callback(self, data):
    if not self.connect_to_ackermann_converter:
      self.vel_linear = round(data.linear.x,3)
      self.angle = round(data.angular.z,3) 
      
      self.is_change_reverse = (self.vel_linear > 0 and self.vel_linear_odometry < 0.3) or (self.vel_linear < 0 and self.vel_linear_odometry > 0.3)
  
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    if self.connect_to_ackermann_converter:
      self.last_time_ackermann = time.time()
      
      self.vel_linear = round(acker_data.drive.speed,3)
      self.angle = round(acker_data.drive.steering_angle,3)
      
      self.is_change_reverse = (self.vel_linear > 0 and self.vel_linear_odometry < 0.3) or (self.vel_linear < 0 and self.vel_linear_odometry > 0.3)

  def publish_msg_control(self, accelerator, reverse, direction, brake):
    '''
    Publica mensaje de control
    '''
    accel_msg = Int16(int(accelerator))
    brake_msg = Bool(brake)
    reverse_msg = Bool(reverse)
    direction_msg = DirectionCmd(angle=direction["angle"], activate=direction["activate"])
    
    self.accel_pub.publish(accel_msg)
    self.reverse_pub.publish(reverse_msg)
    self.direction_pub.publish(direction_msg)
    self.brake_pub.publish(brake_msg)

  def control_navegacion(self):
    '''
    Control para el modo navegación
    '''
    if self.vel_linear == 0:
      self.pid.clean_acumulative_error()
        
    accel_value = self.pid.update(abs(self.vel_linear), abs(self.vel_linear_odometry)) if self.vel_linear != 0 else self.range_accel_converter[0]
    brake_value = self.vel_linear == 0
    reverse_value = self.vel_linear < 0 
    direction_value = {"angle": self.angle, "activate": True}
    
    current_time = time.time()
    if current_time-self.last_time_odometry > 0.2 :
      reverse_value = False
      accel_value = 0
      brake_value = True
      direction_value['activate'] = False
      self.pid.clean_acumulative_error()
    
    if self.connect_to_ackermann_converter and current_time - self.last_time_ackermann > 0.3:
      reverse_value = False
      accel_value = 0
      brake_value = True
      direction_value['activate'] = False
      self.pid.clean_acumulative_error()
    
    self.publish_msg_control(
      accelerator=accel_value, 
      reverse=reverse_value, 
      direction=direction_value, 
      brake=brake_value)
    
  def control_web(self):
    ''' 
    Control del robot a partir de la web
    '''

  def calculate_control_inputs(self):
    '''
    Calcula y publica si debe, 
    '''
    if self.mode_puma == "navegacion":
      self.control_navegacion()
    elif self.mode_puma == "web":
      self.control_web()
    else: 
      time_now = time.time()
      if time_now - self.last_time_publish > 10: 
        self.publish_log(1, "El controlador no esta publicando datos, a la espera del cambio de modo. Se repite el mensaje cada 10 seg.")
        self.last_time_publish = time_now