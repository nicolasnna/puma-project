#!/usr/bin/env python3
import rospy
from puma_brake_msgs.msg import BrakeData, BrakeCmd
import time
from puma_brake_i2c.i2cArduinoConection import I2cArduinoConection
from puma_brake_i2c.arduinoBrakeController import ArduinoBrakeController

class BrakeController():
    '''
    Controlador de los motores paso a paso
    '''
    def __init__(self):
        '''
        Configuracion inicial del nodo
        '''
        rospy.init_node('puma_brake_i2c_node')
        
        rospy.Subscriber('puma/brake/command', BrakeCmd, self._control_callback)
        self._pub_status = rospy.Publisher('puma/brake/status', BrakeData, queue_size=10)
        self._arduino_controller = ArduinoBrakeController()
        
        self._set_params()
        self._info_brake = BrakeData()
        self._send_data_control = False
        self._is_new_data = False
        self._repeat_cmd = False
        self.position = 0
    def _control_callback(self, data):
        '''
        Almacenar nueva posicion recibida
        '''
        self._send_data_control = True
        rango = [int(self.pos_max/3), int(self.pos_max*2/3)]
        if data.position < rango[0]:
            position_aprox = 30
        elif data.position < rango[1]:
            position_aprox = int(self.pos_max/2)
        else:
            position_aprox = self.pos_max
        
        if (self.position == position_aprox):
            self._is_new_data = False
        else:
            self.position = position_aprox
            self._is_new_data = True
        rospy.loginfo(data)
        self._repeat_cmd = data.button_repeat

    def _set_params(self):
        '''
        Ajuste de parametros de velocidad, resolucion y aceleracion
        '''
        self.velocity = rospy.get_param('vel_motor',750)
        self.acceleration = rospy.get_param('acel_motor',250)
        self.resolution = rospy.get_param('step_res', "1/8")
        self.pos_max = rospy.get_param('pos_max_motor', 0)
        
    def config_i2c_arduino(self, ADDR, BUS):
        '''
        Configuracion de la comunicacion i2c
        '''
        self._i2c_connection = I2cArduinoConection(ADDR, BUS)
        
    def config_arduino_initial(self):
        '''
        Comunicacion y configuracion inicial con Arduino.
        '''
        #--- Inicia la configuracion ---#
        rospy.loginfo("Iniciando la configuracion de los motores...")
        try:
            self._i2c_connection.send_string(self._arduino_controller.get_command_resolution(self.resolution))
            time.sleep(0.7)
            rospy.loginfo("Resolucion: %s",self._i2c_connection.read_string())
            time.sleep(0.2)
            self._i2c_connection.send_string(self._arduino_controller.get_command_acceleration(self.acceleration))
            time.sleep(0.7)
            rospy.loginfo("Aceleracion: %s",self._i2c_connection.read_string())
            time.sleep(0.2)
            self._i2c_connection.send_string(self._arduino_controller.get_command_velocity(self.velocity))
            time.sleep(0.7)
            rospy.loginfo("Velocidad: %s",self._i2c_connection.read_string())
            time.sleep(0.2)
            return 1
        except IOError:
            rospy.logerr("No se logro comunicar con el Arduino")
            return -1

    def set_config_motor_runtime(self):
        '''
        Envio de ajuste de posicion de destino del motor 
        '''
        #rospy.loginfo("New data: %s",self._is_new_data)
        if (self._send_data_control and self._is_new_data) or self._repeat_cmd:
            cmd_to_send = self._arduino_controller.get_command_position(self.position)
            self._i2c_connection.send_string(cmd_to_send)
            rospy.loginfo(cmd_to_send)
            self._send_data_control = False


    def publish_info_brake(self):
        '''
        Publicar info de configuracion del motor
        '''
        self._info_brake.name = "motor pp freno"
        self._info_brake.step_resolution = self.resolution
        self._info_brake.velocity_max = self.velocity
        self._info_brake.acceleration = self.acceleration
        self._info_brake.position_max = self.position
    
        #rospy.loginfo(self._info_brake)
        self._pub_status.publish(self._info_brake) 