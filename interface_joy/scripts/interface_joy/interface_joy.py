#!/usr/bin/env python3
import rospy
from brake_controller_msgs.msg import brake_control
from control_dir_msgs.msg import dir_data
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy

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
    
    state_transform = {"0": False, "1": True}
    def __init__(self) -> None:
        
        rospy.init_node("interface_joy",anonymous=True)
        
        # Get params from rosparam
        self.__sub_topic = rospy.get_param('joy_topic','joy')
        self.__pub_topic_brake = rospy.get_param('brake_topic','brake_controller/data_control')
        self.__pub_topic_dir = rospy.get_param('dir_topic','control_dir/dir_data')
        self.__pub_topic_accel_puma = rospy.get_param('accel_puma_topic','accel_puma/value')
        self.pos_range = [rospy.get_param('pos_min', 0), rospy.get_param('pos_max',1000)]
        self.accel_puma_range = [rospy.get_param('min_accel', 0), rospy.get_param('max_accel', 10)]
        
        # Get index of axes and buttons
        self.__LT_LEFT_INDEX = rospy.get_param('lt_left_index',2)
        self.__RT_RIGHT_INDEX = rospy.get_param('rt_right_index',5)
        self.__X_LEFT_INDEX = rospy.get_param('x_left_index', 0)
        self.__Y_LEFT_INDEX = rospy.get_param('y_left_index', 1)    
        self.__X_RIGHT_INDEX = rospy.get_param('x_right_index', 3)
        self.__Y_RIGHT_INDEX = rospy.get_param('y_right_index', 4)
        
        self.__A_BUTTON_INDEX = rospy.get_param('a_button_index', 0)
        self.__B_BUTTON_INDEX = rospy.get_param('b_button_index', 1)
        self.__X_BUTTON_INDEX = rospy.get_param('x_button_index', 2)
        self.__Y_BUTTON_INDEX = rospy.get_param('y_button_index', 3)
        self.__LB_BUTTON_INDEX = rospy.get_param('lb_button_index', 4)
        self.__RB_BUTTON_INDEX = rospy.get_param('rb_button_index', 5)
        
        # Create topics
        rospy.Subscriber(self.__sub_topic, Joy, self.__subCallBack)
        self._publisher_brake = rospy.Publisher(self.__pub_topic_brake, brake_control, queue_size=5)  # Is modifly
        self._publisher_dir = rospy.Publisher(self.__pub_topic_dir, dir_data, queue_size=5)
        self._publisher_accel_puma = rospy.Publisher(self.__pub_topic_accel_puma, Int16, queue_size=5)
        # Create variable to send
        self.msg_send_brake = brake_control()
        self.msg_send_dir = dir_data()
        self.msg_send_accel_puma = Int16()
        
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
        
    def __convertTriggerToRange(self,trigger_value, value_min, value_max):
        '''
        Convert 1.0/-1.0 to pos_min/pos_max
        '''
        conversion_result = (value_min - value_max)/(1.0+1.0) * (trigger_value-1.0) + value_min
        return int(conversion_result)
        
    def sendDataControl(self):
        '''
        Publish control data and dir data
        '''
        # --- Control brake --- #
        self.msg_send_brake.position = self.__convertTriggerToRange(self.lt_left, self.pos_range[0], self.pos_range[1])
        self.msg_send_brake.button_repeat = self.LB_button
        self._publisher_brake.publish(self.msg_send_brake)
        
        # --- Control direction --- #
        self.msg_send_dir.range = int(self.x_left*100)
        self.msg_send_dir.activate = self.A_button
        self.msg_send_dir.finish_calibration = self.B_button
        self._publisher_dir.publish(self.msg_send_dir)
        
        # --- Control Accelerator puma --- #
        self.msg_send_accel_puma.data = self.__convertTriggerToRange(self.rt_right, self.accel_puma_range[0], self.accel_puma_range[1])
        self._publisher_accel_puma.publish(self.msg_send_accel_puma)