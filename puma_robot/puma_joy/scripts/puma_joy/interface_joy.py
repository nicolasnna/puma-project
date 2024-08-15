#!/usr/bin/env python3
import rospy
import numpy as np
from puma_brake_msgs.msg import BrakeCmd
from puma_direction_msgs.msg import DirectionCmd
from std_msgs.msg import Int16, Bool
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class InterfaceJoy():
    '''
    Data Interface between Joy and another msg
    '''
    # Triggers
    lt_left = 1.0
    rt_right = 1.0
    # Analog left
    x_left = 0
    y_left = 0
    # Analog right
    x_right = 0
    y_right = 0
    # Button State
    A_button = False
    B_button = False
    X_button = False
    Y_button = False
    LB_button = False 
    RB_button = False
    back_button = False
    start_button = False
    ready_send_accel = False
    
    init_send = 0
    state_transform = {"0": False, "1": True}
    def __init__(self):
        
        rospy.init_node("puma_joy_node")
        ns = '/puma_joy'
        # Get params from rosparam
        self.__sub_topic = rospy.get_param(ns+'/joy_topic','joy')
        self.__pub_topic_brake = rospy.get_param(ns+'/brake_topic','puma/brake/command')
        self.__pub_topic_dir = rospy.get_param(ns+'/dir_topic','puma/direction/command')
        self.__pub_topic_accel_puma = rospy.get_param(ns+'/accel_puma_topic','puma/accelerator/command')
        self.accel_puma_range = [rospy.get_param(ns+'/min_accel', 0), rospy.get_param(ns+'/max_accel', 10)]
        self.angle_range = np.deg2rad([rospy.get_param(ns+'/angle_min_degree', -30), rospy.get_param(ns+'/angle_max_degree', 30)])
        
        # Get index of axes and buttons
        self.__LT_LEFT_INDEX = rospy.get_param(ns+'/lt_left_index',2)
        self.__RT_RIGHT_INDEX = rospy.get_param(ns+'/rt_right_index',5)
        self.__X_LEFT_INDEX = rospy.get_param(ns+'/x_left_index', 0)
        self.__Y_LEFT_INDEX = rospy.get_param(ns+'/y_left_index', 1)    
        self.__X_RIGHT_INDEX = rospy.get_param(ns+'/x_right_index', 3)
        self.__Y_RIGHT_INDEX = rospy.get_param(ns+'/y_right_index', 4)
        
        self.__A_BUTTON_INDEX = rospy.get_param(ns+'/a_button_index', 0)
        self.__B_BUTTON_INDEX = rospy.get_param(ns+'/b_button_index', 1)
        self.__X_BUTTON_INDEX = rospy.get_param(ns+'/x_button_index', 2)
        self.__Y_BUTTON_INDEX = rospy.get_param(ns+'/y_button_index', 3)
        self.__LB_BUTTON_INDEX = rospy.get_param(ns+'/lb_button_index', 4)
        self.__RB_BUTTON_INDEX = rospy.get_param(ns+'/rb_button_index', 5)
        self.__BACK_BUTTON_INDEX = rospy.get_param(ns+'/back_button_index', 6)
        self.__START_BUTTON_INDEX = rospy.get_param(ns+'/start_button_index', 7)
        
        # Create topics
        rospy.Subscriber(self.__sub_topic, Joy, self.__subCallBack)
        rospy.Subscriber('diagnostics', DiagnosticArray, self._diagnostic_callback)
        self._publisher_brake = rospy.Publisher(self.__pub_topic_brake, BrakeCmd, queue_size=5)  # Is modifly
        self._publisher_dir = rospy.Publisher(self.__pub_topic_dir, DirectionCmd, queue_size=5)
        self._publisher_accel_puma = rospy.Publisher(self.__pub_topic_accel_puma, Int16, queue_size=5)
        self._publisher_reverse = rospy.Publisher('puma/reverse/command', Bool, queue_size=5)
        self._publisher_brake_electric = rospy.Publisher('puma/parking/command', Bool, queue_size=5)
        self._publisher_diagnostic = rospy.Publisher('puma/joy/diagnostic', DiagnosticStatus, queue_size=5)
        # Create variable to send
        self.msg_send_brake = BrakeCmd()
        self.msg_send_dir = DirectionCmd()
        self.msg_send_accel_puma = Int16()
        self.msg_send_accel_puma.data = 0 
        
        self.msg_send_reverse = Bool()
        self.msg_send_reverse.data = False
        
        self.msg_send_brake_electric = Bool()
        self.msg_send_brake_electric.data = False
        
        self.msg_diagnostic = DiagnosticStatus()
        self.msg_diagnostic.level = 0
        self.msg_diagnostic.name = 'Joy interface puma'
        self.msg_diagnostic.message = "Interface is working"
        
        self.level_joy = 2
        
        
    def __subCallBack(self, data_received):
        '''
        CallBack for received data from Joy
        '''
        #--- Start in 1.0 and end in -1.0 --#
        self.lt_left = data_received.axes[self.__LT_LEFT_INDEX]
        self.rt_right = data_received.axes[self.__RT_RIGHT_INDEX]
        
        self.x_left = data_received.axes[self.__X_LEFT_INDEX]
        self.y_left = data_received.axes[self.__Y_LEFT_INDEX]
        
        self.x_right = data_received.axes[self.__X_RIGHT_INDEX]
        self.y_right = data_received.axes[self.__Y_RIGHT_INDEX]
        
        #--- Convert from 0-1 to False-True ---#
        self.A_button = self.state_transform[str(data_received.buttons[self.__A_BUTTON_INDEX])]
        self.B_button = self.state_transform[str(data_received.buttons[self.__B_BUTTON_INDEX])]
        self.X_button = self.state_transform[str(data_received.buttons[self.__X_BUTTON_INDEX])]
        self.Y_button = self.state_transform[str(data_received.buttons[self.__Y_BUTTON_INDEX])]
        self.LB_button = self.state_transform[str(data_received.buttons[self.__LB_BUTTON_INDEX])]
        self.RB_button = self.state_transform[str(data_received.buttons[self.__RB_BUTTON_INDEX])]
        self.start_button = self.state_transform[str(data_received.buttons[self.__START_BUTTON_INDEX])]
        self.back_button = self.state_transform[str(data_received.buttons[self.__BACK_BUTTON_INDEX])]
        
        # Brake electric status
        if self.LB_button and self.RB_button:
            self.msg_send_brake_electric.data = False
        elif self.RB_button:
            self.msg_send_brake_electric.data = True
        # Reverse status
        #if self.back_button and self.start_button:
        self.msg_send_reverse.data = self.LB_button
        
        # Accelerator status
        if self.start_button:
            self.ready_send_accel = True

    def _diagnostic_callback(self, data_received):
        status = data_received.status
        
        for elements in status:
            if elements.name == 'joy_node: Joystick Driver Status':
                self.level_joy = elements.level 
        self.msg_diagnostic.level = self.level_joy
                        
    def __convertTriggerToRange(self,trigger_value, value_min, value_max):
        '''
        Convert 1.0/-1.0 to pos_min/pos_max
        '''
        conversion_result = (value_min - value_max)/(1.0+1.0) * (trigger_value-1.0) + value_min
        return (conversion_result)
        
    def sendDataControl(self):
        '''
        Publish control data and dir data
        '''
        try: 
            # --- Control brake --- #
            self.msg_send_brake.activate_brake = True if self.lt_left < 0.0 else False 
                
            # --- Control direction --- #
            self.msg_send_dir.angle = self.__convertTriggerToRange(self.x_left*-1, self.angle_range[0], self.angle_range[1])
            self.msg_send_dir.activate = self.A_button
            #self.msg_send_dir.finish_calibration = self.B_button
            
            # --- Control Accelerator puma --- #
            self.msg_send_accel_puma.data = int(self.__convertTriggerToRange(self.rt_right, self.accel_puma_range[0], self.accel_puma_range[1]))
            
            self.msg_diagnostic.message = "Interface is working"
            
            if self.level_joy != 0:
                self.msg_send_accel_puma.data = 0
                self.msg_send_dir.activate = False
                self.msg_send_brake.activate_brake = False
                self.msg_diagnostic.message = "Interface is not working"
            
        except: 
            # Cierre
            self.msg_send_brake.activate_brake = True
            self.msg_send_dir.activate = False
            self.msg_send_accel_puma.data = 0
        finally: 
            self._publisher_diagnostic.publish(self.msg_diagnostic)
            self._publisher_brake.publish(self.msg_send_brake)
            self._publisher_dir.publish(self.msg_send_dir)
            if self.ready_send_accel:
                self._publisher_accel_puma.publish(self.msg_send_accel_puma)
            self._publisher_reverse.publish(self.msg_send_reverse)
            self._publisher_brake_electric.publish(self.msg_send_brake_electric)