#!/usr/bin/env python3
import rospy
import signal
import math
from ackermann_msgs.msg import AckermannDriveStamped
from puma_msgs.msg import DirectionCmd, WebTeleop, StatusArduino, Log
from std_msgs.msg import Bool, Int16, String
from nav_msgs.msg import Odometry
from puma_controller.pid_antiwindup import PidAccelerator, PidAngle
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from puma_controller.utils import NodeConfig, LogPublisher, ControlPublisher

class PumaController:
  def __init__(self):
    self.config = NodeConfig.get_instance()
    self.initialize_state_variables()
    self.register_signal_handlers()
    self.setup_publishers_and_subscribers()
    self.log_publisher = LogPublisher(self.logs_pub)
    self.control_publisher = ControlPublisher(
      self.reverse_pub, self.parking_pub, self.accel_pub, self.direction_pub, self.brake_pub
    )
    self.pid = PidAccelerator(
      name="pid_navigation",
      kp=self.config.kp, 
      ki=self.config.ki, 
      kd=self.config.kd, 
      min_value=self.config.range_accel_converter[0], 
      max_value=self.config.range_accel_converter[1],
      max_value_initial=self.config.limit_accel_initial
    )
    self.pid_web_accel = PidAccelerator(
      name="pid_web_accel",
      kp=0.5,
      ki=0.3,
      kd=0.005,
      min_value=12,
      max_value=35,
      max_value_initial=22
    )
    self.pid_web_angle = PidAngle(
      name="pid_web_angle",
      kp=0.55,
      ki=0.3,
      kd=0.005,
      min_value=math.radians(-45),
      max_value=math.radians(45),
      max_value_initial=0,
      disable_final_check=True
    )

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
    rospy.Subscriber('/puma/localization/ekf_odometry', Odometry, self.odometry_callback)
    rospy.Subscriber('/puma/control/current_mode', String, self.selector_mode_callback)
    rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)
    rospy.Subscriber('/puma/web/teleop', WebTeleop, self.web_command_callback)
    rospy.Subscriber('/puma/arduino/status', StatusArduino, self.arduino_status_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
    
  def initialize_state_variables(self):
    """Inicializa las variables de estado de la clase."""
    self.signal_secure = False
    self.vel_linear = 0
    self.vel_linear_odometry = 0
    self.angle = 0
    self.web = {"accel": 0, "angle": 0, "brake": False, "reverse": False, "parking": False, "enable_direction": False}
    self.web_pid_output = {"accel": 18, "angle": 0}
    self.mode_puma = ''
    self.last_time_msg = {key: 0 for key in ["odometry", "ackermann", "web", "pid_control", "move_base"]}
    self.last_time_log_error = {key: rospy.get_time() for key in ["web", "odometry", "ackermann", "mode", "joystick", "secure"]}
    self.last_time_log_error["joystick"] = 0
    self.time_between_msg = {
      "odometry": 0.3,
      "ackermann": 0.3,
      "web": 0.8,
      "pid_control": 2,
      "move_base": 0.5
    }
    self.time_between_log = {
      "odometry": 3,
      "ackermann": 3,
      "web": 5,
      "joystick": 10,
      "mode": 10,
      "secure": 10,
    }
    self.is_change_reverse = False

  def manage_send_error_log(self, key, with_time=True):
    """ Manejador de logs de estado. 
      :param key: odometry, ackermann, web, mode, joystick, secure, pid_control, move_base
    """
    text = {
      "odometry": f"Error al emplear control PID, a pasado mas de {self.time_between_msg['odometry']} seg desde el ultimo dato recibido por la odometria.",
      "ackermann": f"Error al emplear control PID con convertidor ackermann, a pasado mas de {self.time_between_msg['ackermann']} seg desde el ultimo dato.",
      "web": f"Error al emplear el control por la web, ha pasado mas de {self.time_between_msg['web']} seg. desde el ultimo valor recibido.",
      "mode": "No hay un modo de control valido, se define el robot en modo 'idle'.",
      "joystick": f"El controlador no esta publicando datos, ya que se encuentra en modo joystick. Se repite el mensaje cada {self.time_between_log['joystick']} seg.",
      "secure": "Se ha activado la señal de seguridad, el robot se encuentra en modo seguro.",
      "pid_control": f"Error en el control de velocidad por PID, ha pasado mas de {self.time_between_msg['pid_control']} segundos desde que se llego al límite inicial en el comando de velocidad y aún no se ha logrado producir movimiento. Limpiando PID.",
      "move_base": f"Error en el uso de move_base para la navegación, ha pasado mas de {self.time_between_msg['move_base']} seg desde el ultimo estado recibido."
    }
    if with_time:
      time_now = rospy.get_time()
      if time_now - self.last_time_log_error[key] > self.time_between_log[key]:
        self.log_publisher.publish(2, text[key])
        self.last_time_log_error[key] = time_now
    else:
      self.log_publisher.publish(2, text[key])
    

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
    
  def move_base_status_callback(self, _):
    self.last_time_msg["move_base"] = rospy.get_time()
  
  def arduino_status_callback(self, arduino_msg):
    secure_received = arduino_msg.control.security_signal
    if (self.signal_secure != secure_received):
      self.log_publisher.publish(1, "Señal de seguridad recibida. Ajustando control a seguro." if secure_received else "Señal de seguridad desactivada. Ajustando control modo normal.") 
    self.signal_secure = secure_received
  
  def odometry_callback(self, odom):
    """ Get current velocity and calculate break value"""
    self.last_time_msg["odometry"] = rospy.get_time()
    self.vel_linear_odometry = round(odom.twist.twist.linear.x,2)
    
  def cmdvel_callback(self, data):
    if not self.config.connect_to_ackermann_converter and self.mode_puma == "navegacion":
      self.vel_linear = round(data.linear.x,2)
      self.angle = self.clamp_angle(data.angular.z)
      self.is_change_reverse = self.should_change_reverse()
      
  def web_command_callback(self, data):
    ''' MODIFICAR PARA SIMPLIFICAR '''
    if self.mode_puma == "web":
      self.web["accel"] = max(min(data.accel_value, self.config.range_accel_converter[1]), 0)
      self.web["angle"] = self.clamp_angle(math.radians(data.angle_degree))
      self.web["enable_direction"] = data.enable_direction
      self.web["brake"] = data.brake
      self.web["parking"] = data.parking
      self.web["reverse"] = data.reverse
      self.last_time_msg["web"]= rospy.get_time()
  
  def control_web(self):
    ''' Control del robot a partir de la web '''
    current_time = rospy.get_time()
    
    if current_time - self.last_time_msg["web"] < self.time_between_msg["web"] and not self.signal_secure:
      self.web_pid_output["accel"] = self.pid_web_accel.update(self.web["accel"], self.web_pid_output["accel"])
      
      if self.web["enable_direction"]:
        self.web_pid_output["angle"] = self.pid_web_angle.update(self.web["angle"], self.web_pid_output["angle"])
      
      self.control_publisher.publish(
        accelerator=round(self.web_pid_output["accel"]), 
        reverse=self.web["reverse"], 
        direction={"angle": self.web_pid_output["angle"], "activate": self.web["enable_direction"]}, 
        brake=self.web["brake"],
        parking=self.web["parking"]
      )
      rospy.Rate(10).sleep()
    else:
      self.manage_send_error_log("web" if not self.signal_secure else "secure")
      self.publish_idle()
  
  def ackermann_callback(self, acker_data):
    ''' Get velocity lineal of ackermann converter '''
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
    self.vel_linear = self.web["accel"] = 0 # Limpiar el ultimo comando recibido
    self.control_publisher.publish(
      accelerator=0, 
      reverse=False,
      direction={"angle": 0, "activate": False}, 
      brake=True, 
      parking=True
    )

  def control_navegacion(self):
    '''
    Control para el modo navegación
    '''
    current_time = rospy.get_time()
    
    if current_time - self.last_time_msg["odometry"] < self.time_between_msg["odometry"] and not self.signal_secure:
      if self.vel_linear == 0 or self.signal_secure:
        accel_value = self.config.range_accel_converter[0]
        self.pid.clean_acumulative_error()
      else:
        accel_value = self.pid.update(abs(self.vel_linear), abs(self.vel_linear_odometry))
      
      if accel_value == self.config.limit_accel_initial and self.vel_linear_odometry < 0.1:
        self.last_time_msg["pid_control"] = current_time if self.last_time_msg["pid_control"] == 0 else self.last_time_msg["pid_control"]
        if current_time - self.last_time_msg["pid_control"] > self.time_between_msg["pid_control"]:
          self.manage_send_error_log("pid_control", False)
          self.pid.clean_acumulative_error()
          self.last_time_msg["pid_control"] = current_time
      else :
        self.last_time_msg["pid_control"] = 0
      
      # Comprobacion si usa conversor ackermann
      if self.config.connect_to_ackermann_converter:
        if current_time - self.last_time_msg["ackermann"] > self.time_between_msg["ackermann"]:
          self.stop_control_navegacion("ackermann")
        
      else:
        self.control_publisher.publish(
          accelerator=accel_value,
          reverse=self.vel_linear < 0,
          direction={"angle": self.angle, "activate": True},
          brake=self.vel_linear == 0,
          parking=False
        )
    else:
      self.stop_control_navegacion("odometry" if not self.signal_secure else "secure")
    
  def is_move_base_active(self):
    return rospy.get_time() - self.last_time_msg["move_base"] < self.time_between_msg["move_base"]
  
  def stop_control_navegacion(self, error_key, with_time = True):
    self.publish_idle()
    self.pid.clean_acumulative_error()
    self.manage_send_error_log(error_key, with_time)
    
  def manage_control(self):
    '''
    Calcula y publica si debe, 
    '''
    try:
      if self.mode_puma == "navegacion":
        if self.is_move_base_active():
          self.control_navegacion()
        else:
          self.stop_control_navegacion("move_base", False)
      elif self.mode_puma == "web":
        self.control_web()
      elif self.mode_puma == "joystick":
        self.manage_send_error_log("joystick")
      else: 
        self.publish_idle()
        self.manage_send_error_log("mode")
    except Exception as e:
      rospy.logerr(f"Error en el controlador: {e}")
      self.log_publisher.publish(2, f"Error en el controlador: {e}")
      self.publish_idle()
      
  def shutdown_hook(self, _, __):
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
