class JoystickInput:
  """ Maneja la lectura de datos """
  def __init__(self, axes_index, buttons_index):
    self.axes_index = axes_index
    self.buttons_index = buttons_index
    self.joystick_data = {}
    
  def update(self, joy_msg):
    """ Actualiza los datos del joy """
    for name, index in self.axes_index.items():
      self.joystick_data[name] = joy_msg.axes[index]
      
    for name, index in self.buttons_index.items():
      self.joystick_data[name] = bool(joy_msg.buttons[index])
    
  def get_data(self):
    return self.joystick_data
  
class PumaJoyController:
  """ Procesamiento de datos del joy """
  def __init__(self, accel_range, angle_range):
    self.accel_range = accel_range
    self.angle_range = angle_range
    self.data_control = {}
  
  def process_inputs(self, joystick_data):
    self.data_control["accelerator"] = int(self.convert_trigger_to_range(joystick_data['rt_right'], *self.accel_range))
    self.data_control["direction_angle"] = round(self.convert_trigger_to_range(-joystick_data['x_left'], *self.angle_range),3)
    self.data_control["activate_direction"] = joystick_data['A']
    self.data_control["brake"] = joystick_data['lt_left'] < 0.0
    self.data_control["reverse"] = joystick_data['LB']
    self.data_control["parking"] = joystick_data['RB']
  
  def convert_trigger_to_range(self, trigger_value, min_value, max_value):
    return (min_value - max_value) / 2 * (trigger_value - 1 ) + min_value
  
  def get_control(self):
    return self.data_control
  