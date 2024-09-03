#!/usr/bin/env python3
import rospy
class ArduinoBrakeController:
    '''
    Controlador de los motores paso a paso a partir de comandos al Arduino por i2c.
    '''
    def __init__(self):
        self.resolution_steps_dict = {
            "full": "000",
            "1/2": "100",
            "1/4": "010",
            "1/8": "110",
            "1/16": "111"
        }

    def get_command_resolution(self, resolution):
        '''
        Retorna comando de la resolucion de los micropasos para arduino en i2c.
        resolution: "full", "1/2", "1/4", "1/8" y "1/16".
        '''
        try:
            config_data = "cfg steps:" + self.resolution_steps_dict[resolution] + ","
            return config_data
        except KeyError:
            rospy.logerr("Resolucion ingresada invalida")
            return -1

    def get_command_acceleration(self, acceleration):
        '''
        Retorna comando para configurar la acceleracion del motor para arduino.
        acceleration: int.
        '''
        config_data = "cfg acel:" + str(acceleration) + ","
        return config_data
       
    def get_command_velocity(self, velocity):
        '''
        Retorna comando para configurar la velocidad maxima del motor para arduino.
        velocity: int.
        '''
        config_data = "cfg vel:" + str(velocity) + ","
        return config_data

    def get_command_position(self, position):
        '''
        Retorna comando para especificar la posicion del motor para arduino.
        position: int.
        '''
        config_data = "cfg pose:" + str(position) + ','
        return config_data

    def get_3rd_command_vel_acel_pos(self, velocity, acceleration, position):
        '''
        Retorna comando para ajustar la configuracion de los frenos.
        velocity: int, acceleration: int, position: int.
        '''
        config_data = "cfg pose:"+str(position)+",acel:"+str(acceleration)+",vel:"+str(velocity)+","
        return config_data