#!/usr/bin/env python3
import rospy
import math
from puma_msgs.msg import DirectionCmd, Log
from std_msgs.msg import Bool, String, Int16
from sensor_msgs.msg import Joy
from puma_joy.utils_joy import *
from diagnostic_msgs.msg import DiagnosticArray
import signal

class PumaInterfaceJoy:
  """ Interfaz puma joy """
  def __init__(self):
    joy_sub_topic = rospy.get_param('~joy_topic', 'joy')
    self.accel_range = [
      rospy.get_param('~min_accel', 0),
      rospy.get_param('~max_accel', 10)
    ]
    self.angle_range = [
      round(math.radians(rospy.get_param('~angle_min_degree', -45)),3),
      round(math.radians(rospy.get_param('~angle_max_degree', 45)),3)
    ]
    axes_index = {
      'x_left'  : rospy.get_param('~x_left_index', 0),
      'y_left'  : rospy.get_param('~y_left_index', 1),
      'lt_left' : rospy.get_param('~lt_left_index', 2),
      'x_right' : rospy.get_param('~x_right_index', 3),
      'y_right' : rospy.get_param('~y_right_index', 4),
      'rt_right': rospy.get_param('~rt_right_index', 5),
    }
    
    buttons_index = {
      'A'     : rospy.get_param('~a_button_index', 0),
      'B'     : rospy.get_param('~b_button_index', 1),
      'X'     : rospy.get_param('~x_button_index', 2),
      'Y'     : rospy.get_param('~y_button_index', 3),
      'LB'    : rospy.get_param('~lb_button_index', 4),
      'RB'    : rospy.get_param('~rb_button_index', 5),
      'back'  : rospy.get_param('~back_button_index', 6),
      'start' : rospy.get_param('~start_button_index', 7)
    }
    
    self.publishers = {
      'brake': rospy.Publisher('puma/control/brake', Bool, queue_size=5),
      'direction': rospy.Publisher('puma/control/direction', DirectionCmd, queue_size=5),
      'accelerator': rospy.Publisher('puma/control/accelerator', Int16, queue_size=5),
      'reverse': rospy.Publisher('puma/control/reverse', Bool, queue_size=5),
      'parking': rospy.Publisher('puma/control/parking', Bool, queue_size=5),
      'log': rospy.Publisher('puma/logs/add_log', Log, queue_size=5),
      'mode': rospy.Publisher('puma/control/change_mode', String, queue_size=4)
    }
    
    #Subscribers
    rospy.Subscriber(joy_sub_topic, Joy, self.joy_callback)
    rospy.Subscriber('/puma/control/current_mode', String, self.mode_callback)
    rospy.Subscriber('diagnostics', DiagnosticArray, self.diagnostic_callback)
    
    self.joystick_input = JoystickInput(axes_index, buttons_index)
    self.controller = PumaJoyController(self.accel_range, self.angle_range)
    self.data_control = {}
    self.start_to_send = False
    self.joy_is_alive = False
    self.last_log = 0
    self.time_between_log = 10
    self.mode_puma = "idle"
    self.register_signal_handlers()
    
  def register_signal_handlers(self):
    """Registra manejadores de señales para un apagado seguro."""
    signal.signal(signal.SIGINT, self.shutdown_hook)
    signal.signal(signal.SIGTERM, self.shutdown_hook)
    
  def publish_log(self, level, content):
    log_msg = Log(node=rospy.get_name(), level=level, content=content)
    self.publishers["log"].publish(log_msg)
    
  def mode_callback(self, mode_msg):
    self.mode_puma = mode_msg.data
    if self.mode_puma == "joystick" and not self.joy_is_alive:
      self.publishers["mode"].publish(String("idle"))
    
  def diagnostic_callback(self, msg):
    status = msg.status
    for elements in status:
      if "joy_node" in elements.name:
        if elements.level == 0:
          self.publishers["mode"].publish(String("joystick"))
          self.joy_is_alive = True
        else:
          self.joy_is_alive = False
          self.start_to_send = False
    
        
  def joy_callback(self, joy_msg): 
    self.joystick_input.update(joy_msg)
    joystick_data = self.joystick_input.get_data()
    self.controller.process_inputs(joystick_data)
    self.data_control = self.controller.get_control()
    
    if joystick_data["start"]:
      self.start_to_send = True
      
  def publish_command(self):
    accel_msg = Int16(self.data_control["accelerator"])
    brake_msg = Bool(self.data_control["brake"])
    direction_msg = DirectionCmd(
      angle= self.data_control["direction_angle"],
      activate= self.data_control["activate_direction"])
    reverse_msg = Bool(self.data_control["reverse"])
    parking_msg = Bool(self.data_control["parking"])
    
    self.publishers["accelerator"].publish(accel_msg)
    self.publishers["brake"].publish(brake_msg)
    self.publishers["direction"].publish(direction_msg)
    self.publishers["reverse"].publish(reverse_msg)
    self.publishers["parking"].publish(parking_msg)
  
  def run_joy(self):
    time_now = rospy.get_time()
    if self.mode_puma == "joystick":
      if self.joy_is_alive and self.start_to_send:
        try: 
          self.publish_command()
        except:
          self.publish_idle()
      else:
        if time_now - self.last_log > self.time_between_log:
          self.publish_log(
            2,
            f"Se esta en modo idle, favor de encender joystick y presionar start. joy: {self.joy_is_alive} - start: {self.start_to_send}."
            )
          self.last_log = time_now
        self.publish_idle()
    
  def publish_idle(self): 
    self.publishers["accelerator"].publish(Int16(0))
    self.publishers["brake"].publish(Bool(True))
    self.publishers["direction"].publish(DirectionCmd(angle=0, activate=False))
    self.publishers["reverse"].publish(Bool(False))
    self.publishers["parking"].publish(Bool(True))
    
  def shutdown_hook(self, signum, frame):
    """ Manejador de señal para apagar el nodo de manera segura. """
    rospy.logwarn(f"Cerrando el nodo {rospy.get_name()}, publicando en el tópico...")
    self.publish_log(2, "Cerrando nodo por shutdown...")
    if self.mode_puma == "joystick":
      try:
        for _ in range(5):
          self.publish_idle()
          rospy.sleep(0.1)
      except Exception as e:
        rospy.logerr(f"Error al intentar apagar el nodo: {e}")
    rospy.signal_shutdown("Nodo cerrado")