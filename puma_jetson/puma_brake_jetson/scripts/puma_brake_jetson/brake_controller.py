#!/usr/bin/env python3
import rospy
import time
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool, Empty
from puma_brake_msgs.msg import BrakeCmd
from diagnostic_msgs.msg import DiagnosticStatus

class BrakeController:
  def __init__(self, pinDir, pinStep, topic_switch, topic_brake, step_extra, count_max):
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
    step_extra  : int
      Offset for step of motor
    count_max   : int
      Count of step between zero and switch touch
    '''
    GPIO.setmode(GPIO.BOARD)
    rospy.Subscriber(topic_switch, Bool, self.switchCallback)
    rospy.Subscriber(topic_brake+"/command", BrakeCmd, self.brakeCmdCallback)
    rospy.Subscriber(self.topic_brake+"/start_calibration", Bool, self.calibrationCallback)

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
    self.STEP_EXTRA = step_extra
    self.count_max = count_max
    self.count_extra = 0
    self.is_calibrated = False
    self.run_calibrate = False
    
  def calibrationCallback(self, data):
    self.run_calibrate = data.data
    
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
    
    if not self.is_calibrated and self.run_calibrate:
      msg_status.level = 0
      msg_status.message = "Brake controller is calibrating"
      # First detect switch HIGH
      while not self.switch_status:
        self.runSpin(is_positive=True)  
        self.status_pub.publish(msg_status)
      rospy.loginfo("Is detected switch")
      # Go to zero position
      for i in range(0, self.count_max):
        self.runSpin(is_positive=False)  
        self.status_pub.publish(msg_status)
      rospy.loginfo("Is set brake in zero position")
      self.is_calibrated = True
    
    elif self.is_calibrated and diff_time < 3:
      msg_status.level = 0
      msg_status.message = "Brake controller is run"
      # Go until detect switch
      if self.activate and not self.switch_status:
        self.runSpin(is_positive=True)
        self.count += 1
      # Go until extra step
      elif self.activate and self.switch_status and self.count_extra < self.STEP_EXTRA:
        self.runSpin(is_positive=True)
        self.count += 1
        self.count_extra += 1
      # Return to zero position
      elif not self.activate and self.count >= 1:
        self.runSpin(is_positive=False)
        self.count -= 1
        
      # reset count extra
      if not self.switch_status: 
        self.count_extra = 0
    
    elif diff_time >= 3:
      msg_status.level = 1
      msg_status.message = "Doesn't received state of switch"
      
    self.status_pub.publish(msg_status)