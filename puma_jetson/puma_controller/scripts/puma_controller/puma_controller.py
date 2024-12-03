#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from puma_msgs.msg import DirectionCmd, WebControl, Log
from std_msgs.msg import Bool, Int16, String
from nav_msgs.msg import Odometry
from puma_controller.pid_antiwindup import PIDAntiWindUp
from geometry_msgs.msg import Twist;
import signal
import time
import math

class PumaController():
  def __init__(self):
    # Get params
    accelerator_topic = rospy.get_param('~accelerator_topic', 'puma/accelerator/command')
    parking_topic = rospy.get_param('~parking_topic', 'puma/parking/command')
    reverse_topic = rospy.get_param('~revese_topic', 'puma/reverse/command')
    ackermann_topic = rospy.get_param('~ackermann_topic', 'puma/control/ackermann/command')
    direction_topic = rospy.get_param('~direction_topic', 'puma/direction/command')
    brake_topic = rospy.get_param('~brake_topic', 'puma/brake/command')
    self.range_accel_converter = rospy.get_param('~range_accel_converter', [22,35])
    self.connect_to_ackermann_converter = rospy.get_param('~connect_to_ackermann_converter', False)
    self.limit_angle_degree = rospy.get_param('~limit_angle_degree', 45)
  
    # Subscribers
    rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_callback)
    rospy.Subscriber('/puma/odometry/filtered', Odometry, self.odometry_callback)
    rospy.Subscriber('/puma/control/current_mode', String, self.selector_mode_callback)
    rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)
    rospy.Subscriber('/puma/web/control_command', WebControl, self.web_command_callback)

    # Publishers
    self.reverse_pub = rospy.Publisher(reverse_topic, Bool, queue_size=10)
    self.parking_pub = rospy.Publisher(parking_topic, Bool, queue_size=10)
    self.accel_pub = rospy.Publisher(accelerator_topic, Int16, queue_size=10)
    self.direction_pub = rospy.Publisher(direction_topic, DirectionCmd, queue_size=5)
    self.brake_pub = rospy.Publisher(brake_topic, Bool, queue_size=5)
    self.logs_pub = rospy.Publisher('puma/logs/add_log', Log, queue_size=2)
    # Variable
    self.vel_linear = 0
    self.vel_linear_odometry = 0
    self.angle = 0
    """ Commands for web control """
    self.web_accel = 0
    self.web_angle = 0
    self.web_brake = False
    self.web_reverse = False
    
    self.mode_puma = ''
    
    self.pid = PIDAntiWindUp(
      kp=rospy.get_param('~kp', 0.3), 
      ki=rospy.get_param('~ki', 0.2), 
      kd=rospy.get_param('~kd', 0.05), 
      min_value=self.range_accel_converter[0], 
      max_value=self.range_accel_converter[1]
    )

    self.last_time_odometry = self.last_time_ackermann = self.last_time_publish = 0
    self.last_time_web = 0
    self.last_error_web = self.last_error_odometry = self.last_error_ackermann = 0
    self.last_time_mode = 0
    self.is_change_reverse = False
    
    self.publish_log(0, "Iniciando controlador robot puma. Recordar definir el modo de control.")
    
    # Registrar el manejador de señales
    signal.signal(signal.SIGINT, self.shutdown_hook)
    signal.signal(signal.SIGTERM, self.shutdown_hook)

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
    if not self.connect_to_ackermann_converter and self.mode_puma == "navegacion":
      self.vel_linear = round(data.linear.x,3)
      self.angle = max(min(round(data.angular.z,3), math.radians(self.limit_angle_degree)), math.radians(-self.limit_angle_degree)) 
      
      self.is_change_reverse = (self.vel_linear > 0 and self.vel_linear_odometry < 0.3) or (self.vel_linear < 0 and self.vel_linear_odometry > 0.3)
      
  def web_command_callback(self, data):
    if self.mode_puma == "web":
      self.web_accel = max(min(data.accel_value, self.range_accel_converter[1]), 0)
      self.web_angle = max(min(data.angle_degree, self.limit_angle_degree), -self.limit_angle_degree)
      self.web_brake = data.brake
      self.web_reverse = data.reverse
      self.last_time_web = time.time()
  
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    if self.connect_to_ackermann_converter and self.mode_puma == "navegacion":
      self.last_time_ackermann = time.time()
      
      self.vel_linear = round(acker_data.drive.speed,3)
      self.angle = max(min(round(acker_data.drive.steering_angle,3), math.radians(self.limit_angle_degree)), math.radians(-self.limit_angle_degree)) 
      
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
      if current_time-self.last_error_odometry > 5:
        self.publish_log(2, "Error al emplear control PID, a pasado mas de 0.2 seg desde el ultimo dato recibído por la odometria.")
        self.last_error_odometry = current_time
      reverse_value = False
      accel_value = 0
      brake_value = True
      direction_value['activate'] = False
      self.pid.clean_acumulative_error()
    
    if self.connect_to_ackermann_converter and current_time - self.last_time_ackermann > 0.3:
      if current_time-self.last_time_ackermann > 5:
        self.publish_log(2, "Error al emplear control PID con convertidor ackermann, a pasado mas de 0.3 seg desde el ultimo dato.")
        self.last_error_odometry = current_time
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
    current_time = time.time()
    if current_time - self.last_time_web < 0.5:
      accel_value = self.web_accel
      direction_value = {"angle": math.radians(self.web_angle), "activate":True}
      brake_value = self.web_brake
      reverse_value = self.web_reverse
    else:
      if current_time - self.last_error_web > 5:
        self.publish_log(2, "Error al emplear el control por la web, ha pasado mas de 0.5 seg. desde el último valor recibido.")
        self.last_error_web = current_time
      accel_value = 0
      direction_value = {"angle": 0, "activate":False}
      brake_value = True
      reverse_value = False
      
    self.publish_msg_control(
      accelerator=accel_value, 
      reverse=reverse_value, 
      direction=direction_value, 
      brake=brake_value)


  def calculate_control_inputs(self):
    '''
    Calcula y publica si debe, 
    '''
    if self.mode_puma == "navegacion":
      self.control_navegacion()
    elif self.mode_puma == "web":
      self.control_web()
    elif self.mode_puma != "joystick":
      self.publish_msg_control(accelerator=0, reverse=False, direction={"angle":0, "activate": False}, brake=True)
      time_now = time.time()
      if time_now - self.last_time_mode > 5: 
        self.publish_log(2, "No hay un modo de control válido, se define el robot en modo 'idle'.")
        self.last_time_mode = time_now
    else: 
      time_now = time.time()
      if time_now - self.last_time_publish > 10: 
        self.publish_log(1, "El controlador no esta publicando datos, a la espera del cambio de modo. Se repite el mensaje cada 10 seg.")
        self.last_time_publish = time_now
        
  def shutdown_hook(self, signum, frame):
    rospy.logwarn(f"Cerrando el nodo {rospy.get_name()}, publicando en el tópico...")
    self.publish_log(2, "Cerrando nodo...")
    for _ in range(5):  # Intentar publicar 5 veces
      self.publish_msg_control(accelerator=0, reverse=False, direction={"angle": 0, "activate": False}, brake=True)
      rospy.sleep(0.1)
    self.publish_log(2, "Nodo cerrado.")
    rospy.signal_shutdown("Nodo cerrado")
