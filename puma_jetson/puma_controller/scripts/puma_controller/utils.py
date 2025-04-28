import rospy
from puma_msgs.msg import DirectionCmd, Log
from std_msgs.msg import Bool, Int16


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
    self.range_accel_converter = rospy.get_param('~range_accel_converter', [18, 35])
    self.limit_accel_initial = rospy.get_param('~limit_accel_initial', 27)
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

  def publish(self, accelerator, reverse, direction, brake, parking):
    self.accel_pub.publish(Int16(int(accelerator)))
    self.reverse_pub.publish(Bool(reverse))
    self.direction_pub.publish(DirectionCmd(angle=round(direction["angle"],3), activate=direction["activate"]))
    self.brake_pub.publish(Bool(brake))
    self.parking_pub.publish(Bool(parking))