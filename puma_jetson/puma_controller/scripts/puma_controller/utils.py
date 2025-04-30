import rospy
from puma_msgs.msg import DirectionCmd, Log
from std_msgs.msg import Bool, Int16
from dataclasses import dataclass

@dataclass
class ParamPid:
  """Clase para almacenar parámetros PID con valores por defecto."""
  kp: float
  ki: float
  kd: float
  min_value: float
  max_value: float
  max_value_initial: float
  disable_final_check: bool

  @classmethod
  def from_namespace(cls, namespace: str, defaults: dict):
    """Carga parámetros desde un namespace de ROS con valores por defecto."""
    return cls(
        kp=rospy.get_param(f"{namespace}/kp", defaults['kp']),
        ki=rospy.get_param(f"{namespace}/ki", defaults['ki']),
        kd=rospy.get_param(f"{namespace}/kd", defaults['kd']),
        min_value=rospy.get_param(f"{namespace}/min_value", defaults['min_value']),
        max_value=rospy.get_param(f"{namespace}/max_value", defaults['max_value']),
        max_value_initial=rospy.get_param(f"{namespace}/max_value_initial", defaults['max_value_initial']),
        disable_final_check=rospy.get_param(f"{namespace}/disable_final_check", defaults['disable_final_check'])
    )


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
    self.connect_to_ackermann_converter = rospy.get_param('~connect_to_ackermann_converter', False)
    self.limit_angle_degree = rospy.get_param('~limit_angle_degree', 45)
    self.time_between_directions = rospy.get_param('~time_between_directions', 0.4)
    
    self.navigation = self._load_pid_params(
      'navigation',
      {'kp': 5.0, 'ki': 2.0, 'kd': 0.1,
        'min_value': 14, 'max_value': 35,
        'max_value_initial': 23, 'disable_final_check': False}
    )
    
    self.teleop_accel = self._load_pid_params(
        'teleop/accelerator',
        {'kp': 0.4, 'ki': 0.3, 'kd': 0.005,
          'min_value': 12, 'max_value': 30,
          'max_value_initial': 23, 'disable_final_check': False}
    )

    self.teleop_angle = self._load_pid_params(
        'teleop/steering_angle',
        {'kp': 0.55, 'ki': 0.3, 'kd': 0.005,
          'min_value': -45, 'max_value': 45,
          'max_value_initial': 0, 'disable_final_check': True}
    )
    
  def _load_pid_params(self, namespace: str, defaults: dict):
    """Helper para cargar parámetros PID desde namespace."""
    return ParamPid.from_namespace(f'~{namespace}', defaults)


class LogPublisher:
  """Clase responsable de publicar logs en el sistema."""
  def __init__(self, publisher):
    self.publisher = publisher
    self.name = rospy.get_name()

  def publish(self, level, content):
    log_msg = Log(level=level, node=self.name, content=content)
    text = f"{self.name} -> {content}"
    if level == 0:
      rospy.loginfo(text)
    elif level == 1:
      rospy.logwarn(text)
    elif level == 2:
      rospy.logerr(text)
    self.publisher.publish(log_msg)
    

class ControlPublisher:
  """Clase para encapsular la publicación de mensajes de control."""
  def __init__(self, reverse_pub, parking_pub, accel_pub, direction_pub, brake_pub):
    self.reverse_pub = reverse_pub
    self.parking_pub = parking_pub
    self.accel_pub = accel_pub
    self.direction_pub = direction_pub
    self.brake_pub = brake_pub

  def publish(self, accelerator, reverse, direction, brake, parking):
    self.accel_pub.publish(Int16(int(accelerator)))
    self.reverse_pub.publish(Bool(reverse))
    self.direction_pub.publish(DirectionCmd(angle=round(direction["angle"],3), activate=direction["activate"]))
    self.brake_pub.publish(Bool(brake))
    self.parking_pub.publish(Bool(parking))