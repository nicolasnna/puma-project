import time

class PIDAntiWindUp:
  def __init__(self, kp, ki, kd, min_value = 0, max_value = 0, anti_windup = False):
    # Parametros PID
    self._kp = kp
    self._ki = ki
    self._kd = kd
    
    # Anti WindUp
    self._min_value = min_value
    self._max_value = max_value
    self._anti_windup = anti_windup
    
    self.integral = 0.0
    self.previus_error = 0.0
    self.last_time = None
  
  def update(self, setpoint, measurement): 
    current_time = time.time()
    if self.last_time is None: 
      self.last_time = current_time
    dt = current_time - self.last_time
    
    error = setpoint - measurement

    proportional = self._kp * error
    self.integral += error +dt
    if self._anti_windup:
      self.integral = max(min(self.integral, self._max_value/self._ki), self._min_value/self._ki)
      
    integral = self._ki*self.integral
    
    derivative = 0.0
    if dt>0:
      derivative = self._kd * (error - self.previus_error) / dt
    # Calcualar salida
    output = proportional + integral + derivative
    output = max(min(output, self._max_value), self._min_value)
    
    self.previus_error = error
    self.last_time = current_time
    
    return output