# Puma IMU

## Resumen

Paquete ROS para la calibración y/o lectura de un sensor IMU ICM20948. La calibración implementada es enfocada al magnetómetro, empleando `hard iron offset` y `soft iron correction`. 

## Dependencias

- rospy
- sensor_msgs
- puma_msgs
- numpy
- scipy
- smbus2
- icm20948
- matplotlib
  
## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_imu

## Scripts

- **`calibrate_magnetometer.py`**: Script para la obtención de offset y correcciones del magnetómetro, para ello se lee los datos del sensor por el tópico `mag` con el formato `sensor_msgs/MagneticField`. Al finalizar, se crea el archivo yaml en el directorio `config` y se muestra un gráfico comparativo entre los datos medidos y corregidos. El tiempo destinado para la lectura de datos a usar es arbitrario.
  
- **`jetson_icm20948.py`**: Realiza lectura del sensor ICM20948 usando conexión I2C desde una jetson Nano, luego, publica los datos recibidos en los tópicos `imu` y `mag`. (`sensor_msgs/Imu` y `sensor_msgs/MagneticField`)
  
- **`magnetometer_corrected.py`**: Publica datos del magnetómetro calibrado, usando nuevos datos recibidos por `mag` y el archivo de calibración generado en `calibrate_magnetometer`.
  
- **`translate_icm20948_arduino.py`**: Recibe datos desde `puma/sensors/icm20948/raw` (`puma_msgs/ImuData`), los transforma en tipo `sensor_msgs/Imu` y `sensor_msgs/MagneticField`. (`puma/sensors/icm20948/imu` y `puma/sensors/icm20948/mag`) 
