#!/usr/bin/env python3
from smbus2 import SMBus  

class I2cArduinoConection:
    '''
    Manejo de la conexion i2c con el Arduino.
    '''
    data_received = ""
    def __init__(self, ARDUINO_ADDR, BUS):
        self.ARDUINO_ADDR = ARDUINO_ADDR
        self.BUS = SMBus(BUS)
        
    def send_string(self, data_string):
        '''
        Convertir string en un arreglo de numeros para ser enviado.
        '''
        list_bytes = []
        for character in str(data_string):
            list_bytes.append(ord(character))
        
        self.BUS.write_i2c_block_data(self.ARDUINO_ADDR, 0, list_bytes)

        #cfg steps:110,pose:1000,acel:250,vel:500,
        #cfg steps:110,acel:250,vel:700,
        #cfg pose:200,
        #cfg pose:-200,
        #terminar en ,
    
    def read_string(self):
        '''
        Convertir arreglo de bytes recibido en string
        '''
        self.data_received = ""
 
        total_msg = int(self.BUS.read_byte(self.ARDUINO_ADDR))
        if total_msg==0:
                #print("no hay datos")
                return -1
        else:
            #print("Total de mensajes a recibir: ", total_msg)
            for i in range(0,total_msg):
                byte_received = chr(self.BUS.read_byte(self.ARDUINO_ADDR)) 
                #print(byte_received)
                
                self.data_received = str(self.data_received) + byte_received
            return self.data_received
        
    def get_data_received(self):
        '''
        Retorna el ultimo string recibido del Arduino.
        '''
        return self.data_received