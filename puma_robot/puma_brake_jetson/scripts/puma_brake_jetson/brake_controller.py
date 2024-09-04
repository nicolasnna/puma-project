#!usr/bin/env python3
import rospy
import time
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool, Empty
from puma_brake_msgs.msg import BrakeCmd
from diagnostic_msgs.msg import DiagnosticStatus

class BrakeController():
  def __init__(self, pinDir, pinStep, topic_switch, topic_brake):
    '''
    Initialize brake controller class
    
    Parameters
    ----------
    pinDir  : int 
      Pin for direction driver
    pinStep : int 
      Pin for step driver
    topic_switch  : str
      Topic for subscriber status switch
    topic_brake : str
      Topic base for brake
    '''
    GPIO.setmode(GPIO.BOARD)
    rospy.Subscriber(topic_switch, Bool, self.switchCallback)
    rospy.Subscriber(topic_brake+"/command", BrakeCmd, self.brakeCmdCallback)
    self.status_pub = rospy.Publisher(topic_brake+'/diagnostic', DiagnosticStatus, queue_size=4)
    self.topic_brake = topic_brake
    self.STEPPER_DIR = pinDir
    self.STEPPER_STEP = pinStep
    GPIO.setup(self.STEPPER_DIR, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(self.STEPPER_STEP, GPIO.OUT)
    
    self.switch_status = False
    self.current_time = time.time()
    self.activate = False
    self.count = 0
    self.count_max = 500
    self.is_calibrated = False
    
  def switchCallback(self, switch_data): 
    self.switch_status = switch_data.data
    self.current_time = time.time()
    
  def brakeCmdCallback(self, brake_cmd):
    self.activate = brake_cmd.activate_brake
  
  def runSpin(self, is_positive):
    if is_positive:
      GPIO.output(self.STEPPER_DIR, GPIO.HIGH)
    else:
      GPIO.output(self.STEPPER_DIR, GPIO.LOW)
        
    GPIO.output(self.STEPPER_STEP, GPIO.HIGH)
    time.sleep(0.0002)
    GPIO.output(self.STEPPER_STEP, GPIO.LOW)
    time.sleep(0.0005)
  
  def executeBrake(self):
    diff_time = time.time() - self.current_time
    msg_status = DiagnosticStatus()
    msg_status.level = 2
    msg_status.name = "brake jetson"
    msg_status.message = "It doesn't run"
    
    if not self.is_calibrated:
      msg_status.level = 0
      msg_status.message = "Brake controller is calibrating"
      rospy.loginfo("Wait for command in /start_calibration ")
      rospy.wait_for_message(self.topic_brake+"/start_calibration", Empty, timeout=3)
      rospy.loginfo("Start calibration")
      
      while not self.switch_status:
        self.runSpin(is_positive=True)  
        self.status_pub.publish(msg_status)
      rospy.loginfo("Is detected switch")
      
      for i in range(0, self.count_max):
        self.runSpin(is_positive=False)  
        self.status_pub.publish(msg_status)
      rospy.loginfo("Is set brake in zero position")
    
    elif self.is_calibrated and diff_time < 3:
      msg_status.level = 0
      msg_status.message = "Brake controller is run"
      
      while self.activate and not self.switch_status:
        self.runSpin(is_positive=True)
        self.status_pub.publish(msg_status)
      
      while not self.activate and self.count >= 1:
        self.runSpin(is_positive=False)
        self.count -= 1
        self.status_pub.publish(msg_status)
        
    elif diff_time >= 3:
      msg_status.level = 1
      msg_status.message = "Doesn't received state of switch"
      self.status_pub.publish(msg_status)