#!/usr/bin/env python3
import rospy
import signal
import math
from ackermann_msgs.msg import AckermannDriveStamped
from puma_msgs.msg import DirectionCmd, WebTeleop, Log
from std_msgs.msg import Bool, Int16, String
from nav_msgs.msg import Odometry
from puma_controller.pid_antiwindup import PIDAntiWindUp
from geometry_msgs.msg import Twist;

class NodeConfig:
  _instance = None
  
  @staticmethod
  def get_instance():
    if NodeConfig._instance is None:
      NodeConfig()
    return NodeConfig._instance
  
  def __init__(self):
    if NodeConfig._instance is not None:
      raise Exception("Esta clase es un Singleton. Usa get_instance() para obtener la instancia.")
    
    NodeConfig._instance = self
    self.range_accel_converter = rospy.get_param('~range_accel_converter', [22, 35])
    self.connect_to_ackermann_converter = rospy.get_param('~connect_to_ackermann_converter', False)
    self.limit_angle_degree = rospy.get_param('~limit_angle_degree', 45)
    self.kp = rospy.get_param('~kp', 0.3)
    self.ki = rospy.get_param('~ki', 0.2)
    self.kd = rospy.get_param('~kd', 0.05)

class LogPublisher:
  """Clase responsable de publicar logs en el sistema."""
  def __init__(self, publisher):
    self.publisher = publisher

  def publish(self, level, content):
    log_msg = Log(level=level, node=rospy.get_name(), content=content)
    self.publisher.publish(log_msg)

class ControlPublisher:
  """Clase para encapsular la publicación de mensajes de control."""
  def __init__(self, reverse_pub, parking_pub, accel_pub, direction_pub, brake_pub):
    self.reverse_pub = reverse_pub
    self.parking_pub = parking_pub
    self.accel_pub = accel_pub
    self.direction_pub = direction_pub
    self.brake_pub = brake_pub

  def publish(self, accelerator, reverse, direction, brake):
    self.accel_pub.publish(Int16(int(accelerator)))
    self.reverse_pub.publish(Bool(reverse))
    self.direction_pub.publish(DirectionCmd(angle=direction["angle"], activate=direction["activate"]))
    self.brake_pub.publish(Bool(brake))

class PumaController:
  def __init__(self):
    self.config = NodeConfig.get_instance()
    self.setup_publishers_and_subscribers()
    self.log_publisher = LogPublisher(self.logs_pub)
    self.control_publisher = ControlPublisher(
      self.reverse_pub, self.parking_pub, self.accel_pub, self.direction_pub, self.brake_pub
    )
    self.pid = PIDAntiWindUp(
      kp=self.config.kp, 
      ki=self.config.ki, 
      kd=self.config.kd, 
      min_value=self.config.range_accel_converter[0], 
      max_value=self.config.range_accel_converter[1]
    )

    self.initialize_state_variables()
    self.register_signal_handlers()
    self.log_publisher.publish(0, "Iniciando controlador robot puma. Recordar definir el modo de control.")

  def register_signal_handlers(self):
    """Registra manejadores de señales para un apagado seguro."""
    signal.signal(signal.SIGINT, self.shutdown_hook)
    signal.signal(signal.SIGTERM, self.shutdown_hook)

  def setup_publishers_and_subscribers(self):
    """Inicializa todos los publishers y subscribers."""
    self.reverse_pub = rospy.Publisher('/puma/control/reverse', Bool, queue_size=10)
    self.parking_pub = rospy.Publisher('/puma/control/parking', Bool, queue_size=10)
    self.accel_pub = rospy.Publisher('/puma/control/accelerator', Int16, queue_size=10)
    self.direction_pub = rospy.Publisher('/puma/control/direction', DirectionCmd, queue_size=5)
    self.brake_pub = rospy.Publisher('/puma/control/brake', Bool, queue_size=5)
    self.logs_pub = rospy.Publisher('/puma/logs/add_log', Log, queue_size=10)

    if self.config.connect_to_ackermann_converter:
      rospy.Subscriber('/puma/control/ackermann', AckermannDriveStamped, self.ackermann_callback)
    rospy.Subscriber('/puma/odometry/filtered', Odometry, self.odometry_callback)
    rospy.Subscriber('/puma/control/current_mode', String, self.selector_mode_callback)
    rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)
    rospy.Subscriber('/puma/web/teleop', WebTeleop, self.web_command_callback)
    
  def initialize_state_variables(self):
    """Inicializa las variables de estado de la clase."""
    self.vel_linear = 0
    self.vel_linear_odometry = 0
    self.angle = 0
    self.web_accel = 0
    self.web_angle = 0
    self.web_brake = False
    self.web_reverse = False
    self.mode_puma = ''
    self.last_time_msg = {key: 0 for key in ["odometry", "ackermann", "web"]}
    self.last_time_log_error = {key: rospy.get_time() for key in ["web", "odometry", "ackermann", "mode", "joystick"]}
    self.last_time_log_error["joystick"] = 0
    self.time_between_msg = {
      "odometry": 0.3,
      "ackermann": 0.3,
      "web": 0.5
    }
    self.time_between_log = {
      "odometry": 3,
      "ackermann": 3,
      "web": 5,
      "joystick": 10,
      "mode": 10
    }
    self.is_change_reverse = False

  def manage_send_error_log(self, key):
    """ Manejador de logs de estado. 
      :param key: odometry, ackermann, web, mode, joystick
    """
    text = {
      "odometry": f"Error al emplear control PID, a pasado mas de {self.time_between_msg['odometry']} seg desde el ultimo dato recibido por la odometria.",
      "ackermann": f"Error al emplear control PID con convertidor ackermann, a pasado mas de {self.time_between_msg['ackermann']} seg desde el ultimo dato.",
      "web": f"Error al emplear el control por la web, ha pasado mas de {self.time_between_msg['web']} seg. desde el ultimo valor recibido.",
      "mode": "No hay un modo de control valido, se define el robot en modo 'idle'.",
      "joystick": f"El controlador no esta publicando datos, ya que se encuentra en modo joystick. Se repite el mensaje cada {self.time_between_log['joystick']} seg."
    }
    time_now = rospy.get_time()
    
    if time_now - self.last_time_log_error[key] > self.time_between_log[key]:
      self.log_publisher.publish(2, text[key])
      self.last_time_log_error[key] = time_now
    

  def selector_mode_callback(self, mode):
    text = {
      "enter_navegacion": "Modo navegacion detectado, ejecutando control puma con PID.",
      "enter_web": "Modo web detectado, ejecutando control puma con datos recibidos de la web.",
      "out_navegacion": "Saliendo del modo navegacion. Desactivando control puma con PID.",
      "out_web": "Saliendo del modo web. Desactivando control a partir de la web."
    }
    
    if self.mode_puma == "navegacion" and mode.data != "navegacion":
      self.log_publisher.publish(1, text["out_navegacion"])
    if self.mode_puma == "web" and mode.data != "web":
      self.log_publisher.publish(1, text["out_web"])
    if mode.data == "navegacion" and self.mode_puma != "navegacion":
      self.log_publisher.publish(0, text["enter_navegacion"])
    if mode.data == "web" and self.mode_puma != "web":
      self.log_publisher.publish(0, text["enter_web"])
    
    self.mode_puma = mode.data
  
  def odometry_callback(self, odom):
    """ Get current velocity and calculate break value"""
    self.last_time_msg["odometry"] = rospy.get_time()
    self.vel_linear_odometry = round(odom.twist.twist.linear.x,3)
    
  def cmdvel_callback(self, data):
    if not self.config.connect_to_ackermann_converter and self.mode_puma == "navegacion":
      self.vel_linear = round(data.linear.x,3)
      self.angle = self.clamp_angle(data.angular.z)
      self.is_change_reverse = self.should_change_reverse()
      
  def web_command_callback(self, data):
    ''' MODIFICAR PARA SIMPLIFICAR '''
    if self.mode_puma == "web":
      self.web_accel = max(min(data.accel_value, self.config.range_accel_converter[1]), 0)
      self.web_angle = self.clamp_angle(math.radians(data.angle_degree))
      self.web_brake = data.brake
      self.web_reverse = data.reverse
      self.last_time_msg["web"]= rospy.get_time()

      
  def control_web(self):
    ''' 
    Control del robot a partir de la web
    '''
    current_time = rospy.get_time()
    
    if current_time - self.last_time_msg["web"] < self.time_between_msg["web"]:
      self.control_publisher.publish(
        accelerator=self.web_accel, 
        reverse=self.web_reverse, 
        direction={"angle": self.web_angle, "activate": True}, 
        brake=self.web_brake
      )
    else:
      self.manage_send_error_log("web")
      self.publish_idle()
  
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    if self.config.connect_to_ackermann_converter and self.mode_puma == "navegacion":
      self.last_time_msg["ackermann"] = rospy.get_time()
      self.vel_linear = round(acker_data.drive.speed,3)
      self.angle = self.clamp_angle(acker_data.drive.steering_angle)
      self.is_change_reverse = self.should_change_reverse()
      
  def clamp_angle(self, angle):
    """Limita el ángulo dentro del rango permitido. angle debe ser radianes."""
    return round(max(
      min(angle, 
          math.radians(self.config.limit_angle_degree)), 
      math.radians(-self.config.limit_angle_degree)),3)
  
  def should_change_reverse(self):
    """
    Determina si es necesario cambiar a marcha atrás (reverse)
    en función de las velocidades actuales.
    :return: True si debe cambiar a reversa, False de lo contrario.
    """
    return (self.vel_linear > 0 and self.vel_linear_odometry < 0.3) or (self.vel_linear < 0 and self.vel_linear_odometry > 0.3)

  def publish_idle(self):
    """Publica comandos para dejar el robot en estado inactivo."""
    self.control_publisher.publish(
        accelerator=0, reverse=False,
        direction={"angle": 0, "activate": False}, brake=True)

  def control_navegacion(self):
    '''
    Control para el modo navegación
    '''
    current_time = rospy.get_time()
    
    if current_time - self.last_time_msg["odometry"] < self.time_between_msg["odometry"]:
      if self.vel_linear == 0:
        accel_value = self.config.range_accel_converter[0]
        self.pid.clean_acumulative_error()
      else:
        accel_value = self.pid.update(abs(self.vel_linear), abs(self.vel_linear_odometry))
      
      # Comprobacion si usa conversor ackermann
      if self.config.connect_to_ackermann_converter and current_time - self.last_time_msg["ackermann"] > self.time_between_msg["ackermann"]:
        self.publish_idle()
        self.pid.clean_acumulative_error()
        self.manage_send_error_log("ackermann")
        
      else:
        self.control_publisher.publish(
          accelerator=accel_value,
          reverse=self.vel_linear < 0,
          direction={"angle": self.angle, "activate": True},
          brake=self.vel_linear == 0
        )
    else:
      self.publish_idle()
      self.pid.clean_acumulative_error()
      self.manage_send_error_log("odometry")
    
  def manage_control(self):
    '''
    Calcula y publica si debe, 
    '''
    if self.mode_puma == "navegacion":
      self.control_navegacion()
    elif self.mode_puma == "web":
      self.control_web()
    elif self.mode_puma == "joystick":
      self.manage_send_error_log("joystick")
    else: 
      self.publish_idle()
      self.manage_send_error_log("mode")
        
  def shutdown_hook(self, signum, frame):
    """ Manejador de señal para apagar el nodo de manera segura. """
    rospy.logwarn(f"Cerrando el nodo {rospy.get_name()}, publicando en el tópico...")
    self.log_publisher.publish(2, "Cerrando nodo...")
    try:
      for _ in range(5):
          self.publish_idle()
          rospy.sleep(0.1)
    except Exception as e:
      rospy.logerr(f"Error al intentar apagar el nodo: {e}")
    self.log_publisher.publish(2, "Nodo cerrado.")
    rospy.signal_shutdown("Nodo cerrado")
