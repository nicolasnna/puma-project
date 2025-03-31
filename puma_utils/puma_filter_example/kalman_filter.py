'''
Script de prueba para el analisis de datos obtenidos por el tacometro.
buen resultado = FiltroKalmanData(7e-4,0.001)
'''
#!/usr/bin/env python
import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt 
import math

class FiltroKalmanData:
    """
    Filtro de Kalman orientado a las mediciones unidimensionales
    """
    """
    P: Matriz de covarianza del error de la estimación
    mu: Estimación del estado
    Inicio con valor nulo
    """
    P = float('nan')   
    mu = float('nan')

    def __init__(self, Q, R):
        """
        Constructor:
        Q: Ruido del proceso
        R: Ruido de la medición
        """
        self.A = 1
        self.B = 0
        self.C = 1

        self.Q = Q
        self.R = R
    
    def filtrar(self,data):
        """
        Filtrado de las mediciones data:
        rssi: Mediciones data a filtrar
        return: Devuelve los valores filtrados  
        """
        u = 0
        if math.isnan(self.mu):
            self.mu = (1 / self.C) * data
            self.P = (1 / self.C) * self.R * (1 / self.C)
        else:
            #Predicciones
            pred_mu = (self.A * self.mu) + (self.B * u)
            pred_P = self.A * self.P * self.A + self.Q
            #Ganancia de Kalman
            K = pred_P * self.C * (self.C * pred_P * self.C + self.R)**(-1)
            #Actualización
            self.mu = pred_mu + K*(data - self.C*pred_mu)
            self.P = pred_P - K * self.C * pred_P
        
        return self.mu
    
    def ajustar_ruido_proceso(self,ruido):
        """
        Ajustar la varianza del ruido del proceso Q
        """
        self.Q = ruido

    def ajustar_rudio_medicion(self, ruido):
        """
        Ajustar la varianza del ruido de medición R
        """
        self.R = ruido


dirname = os.path.dirname(__file__)
path = os.path.join(dirname, 'data/')
elements = os.listdir(path)
elements.sort()

arduino_status = pd.read_csv(path + elements[1])
tacometer_data = pd.read_csv(path + elements[2])

pwm_values = []
for i in range(0, int(arduino_status['time'].size/30)):
  pwm_values.append(arduino_status['.pwm_accel'][i*30])
pwm_values.append(43)
pwm_values.append(43)

pulsos_values = tacometer_data['.pulsos']

print(len(pulsos_values))
print(len(pwm_values))

fig, axs = plt.subplots(4)
fig.suptitle('Vertically stacked subplots')
axs[0].plot(arduino_status['.pwm_accel'])
axs[1].plot(tacometer_data['.pulsos'])

filtro = FiltroKalmanData(7e-4,0.001)
values_filtered =[]
for element in tacometer_data['.pulsos']:
  values_filtered.append(filtro.filtrar(element))
axs[2].plot(values_filtered)

maximo_rpm = round(np.max(tacometer_data['.pulsos'])*2*60)
print(maximo_rpm)

maximo_rpm = round(np.max(values_filtered)*2*60)
print(maximo_rpm)
vel_maxima = 22
# Relacion pulgadas - metro
# 39.3701 - 1
# RPM = RPM_RUEDAS * RELACION DE TRANSMISION
# VELOCIDAD (MPH) = RPM_RUEDAS * D_RUEDAS (PULGADAS) * PI * 60 / 63360
d_rueda = 0.53

rpm_rueda = vel_maxima / (d_rueda*39.3701 * np.pi * 60 / 63360)  

print(rpm_rueda)
relacion_transmision = round(maximo_rpm / rpm_rueda,2)
print(relacion_transmision)

def convert_vel(x):
    rpm = (x *2 * 60) / relacion_transmision
    velocidad = round(rpm * d_rueda * np.pi / 60, 2)
    return velocidad

array_vel = map(convert_vel, values_filtered)


axs[3].plot(list(array_vel))



plt.show()