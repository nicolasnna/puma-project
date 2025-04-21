import time

class PIDAntiWindUp:
  def __init__(self, kp, ki, kd, min_value, max_value, max_value_initial, disable_final_check=False):
    '''
    kp: float - Constante proporcional
    ki: float - Constante integral
    kd: float - Constante derivativa
    min_value: float - Valor minimo de salida pwm
    max_value: float - Valor maximo de salida pwm
    max_value_initial: float - Valor maximo de salida pwm inicial si el robot no se encuenta en movimiento
    '''
    # Parametros PID
    self._kp = kp
    self._ki = ki
    self._kd = kd
    
    # Anti WindUp
    self._min_value = min_value
    self._max_value = max_value
    self._max_value_initial = max_value_initial
    
    self.integral = 0.0
    self.previus_error = 0.0
    self.last_time = None
    self.disable_final_check = disable_final_check
  
  def update(self, setpoint, measurement): 
    current_time = time.time()
    if self.last_time is None: 
      self.last_time = current_time
    dt = current_time - self.last_time
    
    error = setpoint - measurement

    proportional = self._kp * error
    self.integral += error * dt
    self.integral = max(min(self.integral, self._max_value/self._ki), self._min_value/self._ki)
      
    integral = self._ki*self.integral
    
    derivative = 0.0
    if dt>0:
      derivative = self._kd * (error - self.previus_error) / dt
    # Calcualar salida
    output = proportional + integral + derivative
    if not self.disable_final_check:
      if measurement < 0.1:
        output = max(min(self._max_value_initial, output),self._min_value)
      else:
        output = max(min(output, self._max_value), self._min_value)
    else:
      output = max(min(output, self._max_value), self._min_value)
      
    self.previus_error = error
    self.last_time = current_time
    
    return output
  
  def clean_acumulative_error(self):
    self.integral = 0.0
    self.previus_error = 0.0
    self.last_time = None