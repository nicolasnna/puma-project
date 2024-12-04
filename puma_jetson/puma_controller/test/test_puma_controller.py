#!/usr/bin/env python3
import unittest
import rospy
import math
import subprocess
from std_msgs.msg import Bool, Int16, String
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from puma_msgs.msg import DirectionCmd, WebTeleop, Log
from geometry_msgs.msg import Twist

class TestPumaController(unittest.TestCase):
  def setUp(self):
    rospy.init_node('test_puma_controller', anonymous=True)
    
    self.node_process = None
    # Publishers for input topics
    self.ackermann_pub = rospy.Publisher('/puma/control/ackermann', AckermannDriveStamped, queue_size=10)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.web_control_pub = rospy.Publisher('/puma/web/teleop', WebTeleop, queue_size=10)
    self.mode_pub = rospy.Publisher('/puma/control/current_mode', String, queue_size=10)
    self.odometry_pub = rospy.Publisher('/puma/odometry/filtered', Odometry, queue_size=10)

    # Subscribers for output topics
    self.accel_received = None
    self.accel_sub = rospy.Subscriber('/puma/control/accelerator', Int16, self.accel_callback)
    self.direction_received = None
    self.direction_sub = rospy.Subscriber('/puma/control/direction', DirectionCmd, self.direction_callback)
    self.log_received = None
    self.log_sub = rospy.Subscriber('/puma/logs/add_log', Log, self.log_callback)
    self.brake_received = None
    self.brake_sub = rospy.Subscriber('/puma/control/brake', Bool, self.brake_callback)
    self.reverse_received = None
    self.reverse_sub = rospy.Subscriber('/puma/control/reverse', Bool, self.reverse_callback)
    
    rospy.sleep(1)  # Esperar a que se inicien las conexiones
    self.start_nodes()

  def tearDown(self):
    """Este método se ejecuta después de cada test."""
    self.stop_node()
    
  def start_nodes(self):
    self.node_process = subprocess.Popen(
      ['rosrun', 'puma_controller', 'puma_controller_node.py'],
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
    )
    rospy.sleep(2)
    
  def stop_node(self):
    if self.node_process:
      self.node_process.terminate()
      self.node_process.wait()
      self.node_process = None
      rospy.sleep(2)

  def accel_callback(self, msg):
    self.accel_received = msg.data

  def direction_callback(self, msg):
    self.direction_received = msg

  def log_callback(self, msg):
    self.log_received = msg

  def mode_callback(self, msg):
    self.mode = msg.data
    
  def brake_callback(self, msg):
    self.brake_received = msg.data
    
  def reverse_callback(self, msg):
    self.reverse_received = msg.data

  def test_received_web_mode(self):
    """ Se detecta correctamente el cambio de modo """
    mode_msg = String(data="web")
    self.mode_pub.publish(mode_msg)
    
    rospy.sleep(0.3)

    self.assertIn("Modo web detectado", self.log_received.content)

  def test_web_control_normal_command(self):
    """ Se emplea correctamente el control con los comandos de la web """
    self.test_received_web_mode()
    
    web_msg = WebTeleop(accel_value=22, angle_degree=15, brake=False, reverse=False)
    self.web_control_pub.publish(web_msg)
    
    rospy.sleep(0.2)
    # Objecto para la comparacion
    direction_expected = DirectionCmd(angle=round(math.radians(15),3), activate=True)
    direction_received = self.direction_received
    direction_received.angle = round(direction_received.angle, 3)
    # Test
    self.assertEqual(self.accel_received, 22)
    self.assertEquals(self.direction_received, direction_expected)
    self.assertEqual(self.brake_received, False)
    self.assertEqual(self.reverse_received, False)
    
  def test_web_control_after_1seg(self):
    """ Se setea los comandos en 0 luego de x tiempo sin recibir nuevos comandos """
    self.test_received_web_mode()
    
    web_msg = WebTeleop(accel_value=22, angle_degree=15, brake=False, reverse=False)
    self.web_control_pub.publish(web_msg)
    
    rospy.sleep(1)
    # Objecto para la comparacion
    direction_expected = DirectionCmd(angle=0, activate=False)

    # Test
    self.assertEqual(self.accel_received, 0)
    self.assertEquals(self.direction_received, direction_expected)
    self.assertEqual(self.brake_received, True)
    self.assertEqual(self.reverse_received, False)
    
  def test_received_navegacion_mode(self):
    """ Se detecta correctamente el cambio de modo """
    mode_msg = String(data="navegacion")
    self.mode_pub.publish(mode_msg)
    
    rospy.sleep(0.3)

    self.assertIn("Modo navegacion detectado", self.log_received.content)
    
  def test_navegacion_control_with_odometry(self):
    """" Se envia correctamente el comando con PID cuando se ha recibido la odometria """
    self.test_received_navegacion_mode()
    
    odom = Odometry()
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.3
    for _ in range(10):
      self.odometry_pub.publish(odom)
      self.cmd_vel_pub.publish(cmd_vel)
      rospy.sleep(0.1)
    
    min_accel = 22
    
    self.assertGreater(self.accel_received, min_accel)
  
  def test_navegacion_control_with_odometry_after_1seg(self):
    """" Modo seguro cuando a pasado mas de 1 segundo desde la ultima odometria """
    self.test_received_navegacion_mode()
    
    odom = Odometry()
    cmd_vel = Twist()
    self.odometry_pub.publish(odom)
    rospy.sleep(1)
    
    cmd_vel.linear.x = 0.3
    self.cmd_vel_pub.publish(cmd_vel)
    rospy.sleep(0.1)
    
    self.assertEqual(self.accel_received, 0)
    self.assertEqual(self.brake_received, True)
    
  def test_joystick_log_control(self):
    """ No envia nada cuando esta en modo joystick"""
    mode_msg = String(data="joystick")
    self.mode_pub.publish(mode_msg)
    rospy.sleep(0.3)
    
    self.assertIn("El controlador no esta publicando datos", self.log_received.content)
    
  def test_shutdown_command_idle(self):
    self.test_navegacion_control_with_odometry()
    self.stop_node()
    
    rospy.sleep(1)
    
    self.assertEqual(self.accel_received, 0)
    self.assertEqual(self.brake_received, True)


if __name__ == '__main__':
  import rostest
  rostest.rosrun('puma_controller', 'test_puma_controller', TestPumaController)
