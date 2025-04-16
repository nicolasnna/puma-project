# Puma Bringup Jetson

## Resumen

Paquete ROS noetic para la ejecución de los programas necesarios en la Jetson Nano para el funcionamiento del robot PUMA.

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_bringup_jetson

#### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [ublox_ros](https://github.com/KumarRobotics/ublox)
- [Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio)
- puma_reverse
- puma_parking
- puma_system_monitor
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino)

## Ejecución

Debido a que es una agrupación de paquetes para la Jetson Nano, solo se tiene un archivo tipo _launch_:

    roslaunch puma_bringup_jetson jetson_bringup.launch

## Launch

- **jetson_bringup.launch:** Lanzador del iniciador de los paquetes en la Jetson Nano para el robot PUMA. Este achivo contiene los siguientes paquetes:

  - **`rosserial_python`**: Encargado de enlazar el arduino mega conectado a la Jetson con ROS. El arduino se encarga del acelerador, control de frenos y control de dirección. Se tiene definido los siguientes parámetros para este nodo:

    - **`port`** (string, valor: "/dev/ttyUSB0")

      Puerto de conexión con el arduino.

    - **`baud`** (int, valor: 115200)

      Baudios de la conexión. Se define tanto en el nodo como en el Arduino. Se elige esta velocidad por la cantidad de datos que se transmiten entre el Arduino y la Jetson. (Con valores más bajos presenta errores al mantener la comunicación)

  - **`puma_reverse`**: Encargado de activar la reversa de los motores, tanto delantero como trasero.

  - **`puma_parking`**: Encargado de activar o desactivar los motores del robot, tanto delantero como trasero.

  - **`puma_system_monitor`**: Encargado de monitorear la temperatura y uso de recursos de la Jetson nano.

  - **`ublox_gps`**: Encargado del control del GPS usando ublox. En este nodo se cargan las configuraciones del archivo **`zed_f9p.yaml`**. Adicionalmente se realiza un _remap_ de los tópicos:

    - **`ublox/fix`** -> **`puma/sensors/gps/fix`**
    - **`ublox/fix_velocity`** -> **`puma/sensors/gps/fix_velocity`**
  
  - **`puma_joy`**: Encargado de la lectura de mando de xbox360 y control manual del robot.

  - **`control_mode`**: Encargado de identificar y publicar el modo de control actual del robot.

  - **`controller`**: Encargado de realizar el control del robot, dependiendo del modo utilizado.

  - **`imu_bringup`**: Encargado de traducir los valores de la imu ICM20948 conectado al arduino MEGA, e implementar el filtro de madgwick para mejorar el resultado de la IMU.

  - **`camera_odom`**: Encargado de activar la camara realsense trasera del robot e implementar la odometria visual con rtabmap.

## Config files

Se tiene archivos **`yaml`** en el directorio **`config`** para la configuración de los nodos:

- **zed_f9p.yaml:** Parámetros para el ublox GPS.

  - **`device`** (string, valor: "/dev/ttyACM0")

    Puerto de conexión del ublox.

  - **`frame_id`** (string, valor: "gps")

    Marco de referencia de coordenadas del GPS.

  - **`uart1`**

    - **`~baudrate`** (int, valor: 38400)

      Velocidad de conexión con el dispositivo.

  - **`config_on_startup`** (bool, valor: false)

  - **`dynamic_model`** (string, valor: "automotive")
