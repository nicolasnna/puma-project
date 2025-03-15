import rospy
from puma_imu.read_topic import ReadMagnetometer
from puma_imu import utils
import numpy as np
from sensor_msgs.msg import MagneticField
import rospkg
import yaml
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


if __name__ == "__main__":
  rospy.init_node("calibrate_magnetometer")
  mag_reader = ReadMagnetometer("/mag")
  
  rospy.loginfo("Hacer movimientos en forma de 8 para calibrar el magnetometro")
  rospy.loginfo("Presionar Enter para iniciar la calibracion")
  rospy.loginfo("Presionar Ctrl+C para finalizar") 
  input(">")
  mag_reader.start()

  rospy.loginfo("Obteniendo datos...")
  rospy.loginfo("Presionar Enter para terminar la obtención de datos")
  input(">")
  mag_reader.stop()
  
  rospy.loginfo("Datos obtenidos")
  rospy.loginfo("Calculando matriz de calibración...")
  mx, my, mz = mag_reader.get_magnetometer_array()
  
  s = np.array([mx, my, mz])
  M, n, d = utils.ellipsoid_fit(s)
  
  A_1, b = utils.calculate_params_calibration_mag(M, n, d)
  
  # Calibrar cada punto
  xm_cal, ym_cal, zm_cal = utils.estimate_new_magnetometer_with_array_data(A_1, b, mx, my, mz)
  
  rospy.loginfo("Matriz de corrección:")
  rospy.loginfo(A_1)
  rospy.loginfo("Hard iron offset:")
  rospy.loginfo(b)
  
  # Guardar parametros de calibracion
  calibration_data = {
    'calibration_date': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    'hard_iron_offset': {
      'x': float(b[0, 0]),
      'y': float(b[1, 0]),
      'z': float(b[2, 0])
    },
    'soft_iron_correction': {
      'x': A_1[0, :].tolist(),
      'y': A_1[1, :].tolist(),
      'z': A_1[2, :].tolist()
    }
  }
  path = rospkg.RosPack().get_path('puma_imu') + '/config/magnetometer_calibration.yaml'
  with open(path, 'w') as f:
    yaml.dump(calibration_data, f, default_flow_style=False)
    
  rospy.loginfo(f"Archivo de calibración guardado en: {path}")
  
  
  fig = plt.figure()
  a1 = fig.add_subplot(121, projection='3d')
  a1.scatter(xm_cal, ym_cal, zm_cal, marker='o', color='g')
  a1.title.set_text('Calibrated Data')
  a2 = fig.add_subplot(122, projection='3d')
  a2.scatter(mx, my, mz, marker='o', color='r')
  a2.title.set_text('Original Data')
  
  # Crear figura con subplots
  fig2 = plt.figure(figsize=(15, 5))
  
  # Configurar los 3 subplots
  planes = [
      ('XY', mx, my, xm_cal, ym_cal),
      ('XZ', mx, mz, xm_cal, zm_cal),
      ('YZ', my, mz, ym_cal, zm_cal)
  ]
  
  for i, (title, orig_x, orig_y, cal_x, cal_y) in enumerate(planes):
      ax = fig2.add_subplot(1, 3, i+1)
      ax.scatter(orig_x, orig_y, c='r', marker='o', alpha=0.3, label='Original')
      ax.scatter(cal_x, cal_y, c='g', marker='o', alpha=0.3, label='Calibrado')
      ax.set_title(f'Plano {title}')
      ax.set_xlabel(f'Eje {title[0]}' if i != 2 else 'Eje Y')
      ax.set_ylabel(f'Eje {title[1]}')
      ax.grid(True)
      ax.legend()
      ax.axis('equal')  # Mantener relación de aspecto 1:1
  
  plt.tight_layout()
  plt.show()
  
  # rospy.loginfo(" Publicando mag corregidos tiempo real:")
  # while not rospy.is_shutdown():
  #   mx, my, mz = mag_reader.get_latest_magnetometer_data()
  #   x_off = mx - b[0, 0]
  #   y_off = my - b[1, 0]
  #   z_off = mz - b[2, 0]
    
  #   x_cal = A_1[0,0] * x_off + A_1[0,1] * y_off + A_1[0,2] * z_off
  #   y_cal = A_1[1,0] * x_off + A_1[1,1] * y_off + A_1[1,2] * z_off
  #   z_cal = A_1[2,0] * x_off + A_1[2,1] * y_off + A_1[2,2] * z_off
    
  #   msg = MagneticField()
  #   msg.header.stamp = rospy.Time.now()
  #   msg.header.frame_id = "imu_link"
  #   msg.magnetic_field.x = x_cal
  #   msg.magnetic_field.y = y_cal
  #   msg.magnetic_field.z = z_cal
  #   mag_corrected_pub.publish(msg)
  #   rospy.loginfo(msg)
  #   rospy.Rate(30).sleep()
  #   # rospy.spin()
    
  
  

  
  