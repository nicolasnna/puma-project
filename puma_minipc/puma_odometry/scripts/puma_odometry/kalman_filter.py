#!/usr/bin/env python3
import math

class KalmanFilter:
    """
    Filtro de Kalman orientado a mediciones unidimensionales.
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
        data: Mediciones a filtrar
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