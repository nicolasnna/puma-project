import rospy
from sensor_msgs.msg import Imu, MagneticField

class ReadMagnetometer:
    def __init__(self, topic):
      self.mag_sub = rospy.Subscriber(topic, MagneticField, self._mag_callback)
      self.mx = 0
      self.my = 0
      self.mz = 0
      self._start_measurement = False
      self.array_mx = []
      self.array_my = []
      self.array_mz = []

    def _mag_callback(self, msg):
      self.mx = msg.magnetic_field.x 
      self.my = msg.magnetic_field.y 
      self.mz = msg.magnetic_field.z 
      if self._start_measurement:
        self.array_mx.append(self.mx)
        self.array_my.append(self.my)
        self.array_mz.append(self.mz)
    
    def get_latest_magnetometer_data(self):
      return self.mx, self.my, self.mz
    
    def get_magnetometer_array(self):
      return self.array_mx, self.array_my, self.array_mz
    
    def start(self):
      self._start_measurement = True
    
    def stop(self):
      self._start_measurement = False