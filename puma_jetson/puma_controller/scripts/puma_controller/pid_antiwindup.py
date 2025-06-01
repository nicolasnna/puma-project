import time
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class PIDAntiWindUp:
  def __init__(self, name, kp, ki, kd, min_value, max_value, max_value_initial, disable_final_check=False):
    '''range_min_value
    name: string - Nombre para reconfigurar
    kp: float - Constante proporcional
    ki: float - Constante integral
    kd: float - Constante derivativa
    min_value: float - Valor minimo de salida
    max_value: float - Valor maximo de salida
    max_value_initial: float - Valor maximo de salida inicial si el robot no se encuenta en movimiento
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
    self.reconfigure = DDynamicReconfigure(name)
    self.init_ddynamic_reconfigure()
    
  def init_ddynamic_reconfigure(self):
    pass
    
  def ddynamic_cb(self, config, level):
    self._kp = config.kp
    self._ki = config.ki
    self._kd = config.kd
    self._min_value = config.min_value
    self._max_value = config.max_value
    self._max_value_initial = config.max_value_initial
    self.disable_final_check = config.disable_final_check
    return config
  
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
        output = max(min(self._max_value_initial, output, setpoint),self._min_value)
      else:
        output = max(min(output, self._max_value, setpoint), self._min_value)
    else:
      output = max(min(output, self._max_value, setpoint), self._min_value)
      
    self.previus_error = error
    self.last_time = current_time
    
    return output
  
  def clean_acumulative_error(self):
    self.integral = 0.0
    self.previus_error = 0.0
    self.last_time = None
    
class PidAccelerator(PIDAntiWindUp):
  def init_ddynamic_reconfigure(self):
    self.reconfigure.add_variable("kp", "Constante proporcional", self._kp, 0, 10.0)
    self.reconfigure.add_variable("ki", "Constante integral", self._ki, 0, 5.0)
    self.reconfigure.add_variable("kd", "Constante derivativa", self._kd, 0, 0.5)
    self.reconfigure.add_variable("min_value", "Valor minimo de salida", self._min_value, 0, 20)
    self.reconfigure.add_variable("max_value", "Valor maximo de salida", self._max_value, 20, 45)
    self.reconfigure.add_variable("max_value_initial", "Valor maximo de salida en caso de no haber cambios", self._max_value_initial, self._max_value_initial-10, self._max_value_initial+10)
    self.reconfigure.add_variable("disable_final_check", "Realizar revision final a la salida", self.disable_final_check)
    
    self.reconfigure.start(self.ddynamic_cb)
    
class PidAngle(PIDAntiWindUp):
  def init_ddynamic_reconfigure(self):
    self.reconfigure.add_variable("kp", "Constante proporcional", self._kp, 0, 5.0)
    self.reconfigure.add_variable("ki", "Constante integral", self._ki, 0, 5.0)
    self.reconfigure.add_variable("kd", "Constante derivativa", self._kd, 0, 0.5)
    self.reconfigure.add_variable("min_value", "Valor minimo de salida", self._min_value, -55, -35)
    self.reconfigure.add_variable("max_value", "Valor maximo de salida", self._max_value, 35, 55)
    self.reconfigure.add_variable("max_value_initial", "Valor maximo de salida en caso de no haber cambios", self._max_value_initial, self._max_value_initial-10, self._max_value_initial+10)
    self.reconfigure.add_variable("disable_final_check", "Realizar revision final a la salida", self.disable_final_check)
    
    self.reconfigure.start(self.ddynamic_cb)