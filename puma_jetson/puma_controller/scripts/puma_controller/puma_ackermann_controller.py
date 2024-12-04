#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from puma_msgs.msg import StatusArduino, Log
from std_msgs.msg import String
import time

class CmdVelToAckermann():
  def __init__(self):
    # Initialization of parameters
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
    self.wheel_base = rospy.get_param('~wheel_base', 1.15)  # Wheelbase in meters
    # Subscribers
    rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)
    rospy.Subscriber('/puma/arduino/status', StatusArduino, self.status_callback)
    rospy.Subscriber('/puma/control/current_mode', String, self.mode_callback)
    
    # Publishers
    self.ackermann_pub = rospy.Publisher('/puma/control/ackermann', AckermannDriveStamped, queue_size=10)
    self.logs_pub = rospy.Publisher('puma/logs/add_log', Log, queue_size=2)
    # Calculate max radius
    self.ang_max = math.radians(45)
    self.radius_max = self.wheel_base/math.atan(self.ang_max)  # 4.4912
    #rospy.loginfo("Radius max: %f", self.radius_max)
    self.speed_acker = 0.0
    self.A2R = 0.006135742
    self.position_zero = 395
    self.vel = 0.0
    self.current_angle = 0.0
    self.steering_angle = 0.0
    self.mode = ""
    
    self.log_msg = Log()
    self.log_msg.level = 0
    self.log_msg.node = rospy.get_name()
    self.log_msg.content = "Iniciando conversor ackermann. Recordar definir el modo de control."
    time.sleep(0.1)
    self.logs_pub.publish(self.log_msg)
    
  def mode_callback(self, mode):
    if self.mode != mode.data:
      if mode.data == "autonomous":
        self.log_msg.level = 0
        self.log_msg.content = "Modo autonomo detectado, ejecutando conversor ackermann."
        time.sleep(0.1)
        self.logs_pub.publish(self.log_msg)
      elif self.mode == "autonomous":
        self.log_msg.level = 1
        self.log_msg.content = "Saliendo del modo autonomo. Desactivando conversor ackermann."
        time.sleep(0.1)
        self.logs_pub.publish(self.log_msg)
    self.mode = mode.data
    
  def status_callback(self, status_data):
    analog_angle = status_data.current_position_dir
    self.current_angle = (analog_angle - self.position_zero) * self.A2R
    
  def cmd_vel_callback(self, data_received):
    """
    Processes data received from cmd_vel and updates necessary variables
    """
    linear_velocity = data_received.linear.x
    angular_velocity = data_received.angular.z
    self.steering_angle = self.calculate_steering_angle(linear_velocity, angular_velocity)
    self.vel = linear_velocity

  
  def calculate_steering_angle(self, linear_velocity, angular_velocity):
    """
    Calculates the steering angle and wheel input
    """
    # Calculate turning radius
    if angular_velocity == 0 or linear_velocity == 0:
      return 0
    else:
      radius = linear_velocity / angular_velocity
  
    #rospy.loginfo("Linear velocity: %s. Radius; %s", linear_velocity, radius)
    # Calculate steering angle
    calculate_ang = math.atan(self.wheel_base / radius)
    # Validate steering angle 
    steering_angle = self.ang_max if calculate_ang > self.ang_max else calculate_ang
    steering_angle = -self.ang_max if calculate_ang < -self.ang_max else steering_angle
  
    return steering_angle
  
  def publish_ackermann(self):
    # Messages
    if self.mode == 'autonomous':
      ackermann_msg = AckermannDriveStamped()  
      ackermann_msg.header.stamp = rospy.Time.now()
      ackermann_msg.header.frame_id = 'odom'
      ackermann_msg.drive.steering_angle = self.steering_angle
      ackermann_msg.drive.speed = self.vel
      self.ackermann_pub.publish(ackermann_msg)