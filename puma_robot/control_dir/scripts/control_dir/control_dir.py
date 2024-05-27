#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO #Jetson's GPIO Lib
import time
from control_dir_msgs.msg import dir_data
from std_msgs.msg import Int16


class ControlDir():
    '''
    Class for direction controler for servo motor with PWM 
    '''
    
    def __init__(self):
        # Configure ros
        rospy.init_node('control_dir', anonymous=True)
        rospy.Subscriber('control_dir/dir_data', dir_data, self.__dirDataCallback)
        rospy.Subscriber('dir_puma/status', Int16, self.__statusDirCallback)
      
        # Params from rosparam
        #12sg
        #val_max = 992
        #time_low = 0.00002
        self.val_max = rospy.get_param('value_max', 868)
        self.time_high = rospy.get_param('time_high', 0.005)
        self.time_low = rospy.get_param('time_low', 0.0000)
        self.total_loop = rospy.get_param('total_value_loop', 124)
  
        # Pin board scheme
        GPIO.setmode(GPIO.BOARD)

        # Save pin direction
        self.__right_pin = rospy.get_param('right_pin_dir', 36)
        self.__left_pin = rospy.get_param('left_pin_dir', 37)
        self.__range = 0
        self.__current_range = 0
        self.__activate_control = False
        self.__finish_calibration = False
        self.__stop_dir_right = True
        self.__stop_dir_left = True
        
        # PINES DE SALIDA
        GPIO.setup(self.__right_pin, GPIO.OUT, initial=False)
        GPIO.setup(self.__left_pin, GPIO.OUT, initial=False)
    
        self.i = 30
        self.i_max = 30
        
    def __statusDirCallback(self,data_received):
        if data_received.data >=300:
            self.__stop_dir_right = False
        else:
            self.__stop_dir_right = True
        if data_received.data <= 450:
            self.__stop_dir_left = False
        else:
            self.__stop_dir_left = True
    
    def __dirDataCallback(self,data_received):
        '''
        Callback for data dir received
        '''
        self.__range = data_received.range
        self.__activate_control = data_received.activate
        if self.__finish_calibration == False and self.__finish_calibration != data_received.finish_calibration:
            self.__finish_calibration = data_received.finish_calibration
        
    def isCalibrated(self):
        '''
        Return calibrate state
        '''
        return self.__finish_calibration
    
    def calibrateDirection(self):
        '''
        Calibrate direction
        '''
        if self.i >= self.i_max:
            rospy.loginfo("# --- En modo calibracion de direccion --- #")
        if self.__activate_control:
            if not self.__stop_dir_left:
                if self.__range > 0:
                    rospy.loginfo("# --- Calibrando direccion a la izquierda --- #")
                    for i in range( 0, int(self.total_loop/2) ):
                        GPIO.output(self.__left_pin, True)
                        time.sleep(self.time_high)
                        GPIO.output(self.__left_pin, False)
                        time.sleep(self.time_low)
                
            if not self.__stop_dir_right:
                if self.__range < 0:
                    rospy.loginfo("# --- Calibrando direccion a la derecha --- #")
                    for i in range( 0, int(self.total_loop/2) ):
                        GPIO.output(self.__right_pin, True)
                        time.sleep(self.time_high)
                        GPIO.output(self.__right_pin, False)
                        time.sleep(self.time_low)
            
        else: 
            if self.i >= self.i_max:
                rospy.loginfo("# --- Presionar B para terminar la calibracion --- #")
        # Evitar spam de log info
        self.i +=1
        if self.i>self.i_max:
            self.i = 0
            
    
    def controlDirection(self):
        '''
        Generate PWM for control direction
        '''
        if self.__activate_control and self.__range!=self.__current_range and not self.__stop_dir:
            #rospy.loginfo("#--- PWM direccion activado ---#")
            
            if not self.__stop_dir_left:
                if self.__range > 0 and self.__current_range >-self.val_max:
                    for i in range(0,self.total_loop):
                        GPIO.output(self.__left_pin, True)
                        time.sleep(self.time_high)
                        GPIO.output(self.__left_pin, False)
                        time.sleep(self.time_low)
                    self.__current_range -= self.total_loop
                    
            if not self.__stop_dir_right:
                if self.__range < 0 and self.__current_range <self.val_max:
                
                    for i in range(0,self.total_loop):
                        GPIO.output(self.__right_pin, True)
                        time.sleep(self.time_high)
                        GPIO.output(self.__right_pin, False)
                        time.sleep(self.time_low)
                    self.__current_range += self.total_loop
            if self.__stop_dir_left or self.__stop_dir_right:
                rospy.logerr("Direccion maxima alcanzada")
            #rospy.loginfo("Valor de la direccion actual: %s",self.__current_range)
            
       # else: 
            #rospy.loginfo("# --- PWM direccion desactivado - direccion actual: %s --- #", self.__current_range)