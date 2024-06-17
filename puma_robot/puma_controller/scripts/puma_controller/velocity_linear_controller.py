#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from puma_brake_msgs.msg import BrakeCmd
from std_msgs.msg import Bool, Int16

class VelocityLinearController():
  def __init__(self):
    # Get params
    accelerator_topic = rospy.get_param('~accelerator_topic', 'puma/accelerator/command')
    brake_topic = rospy.get_param('~brake_topic', 'puma/brake/command')
    reverse_topic = rospy.get_param('~revese_topic', 'puma/reverse/command')
    ackermann_topic = rospy.get_param('~ackermann_topic', 'puma/control/ackermann/command')
    
    # Subscribers
    rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_callback)

    # Publishers
    self.reverse_pub = rospy.Publisher(reverse_topic, Bool, queue_size=10)
    self.brake_pub = rospy.Publisher(brake_topic, BrakeCmd, queue_size=10)
    self.accel_pub = rospy.Publisher(accelerator_topic, Int16, queue_size=10)
    
    # Variable
    self.reverse_msg = Bool(False)
    self.brake_msg = BrakeCmd()
    self.accel_msg = Int16()
    
  def ackermann_callback(self, acker_data):
    '''
    Get velocity lineal of ackermann converter
    '''
    vel_linear = acker_data.drive.speed
    self.reverse_msg.data = vel_linear < 0 
    self.brake_msg.position = 0 if vel_linear != 0 else 1000
    
    self.accel_msg.data = int(self.linear_converter_pwm(abs(vel_linear), 28, 100, 0.01, 9.8))
    
    
  def linear_converter_pwm(self, input_value, pwm_min, pwm_max, speed_min, speed_max):
    """
    Converts linear velocity to a PWM signal for motor control
    """
    return (pwm_max - pwm_min) / (speed_max - speed_min) * (input_value - speed_min) + pwm_min
  
  def velocity_publish(self):
    '''
    Puublish velocity linear control periodically
    '''
    self.accel_pub.publish(self.accel_msg)
    self.brake_pub.publish(self.brake_msg)
    self.reverse_pub.publish(self.reverse_msg)