# Puma Robot Status

## Resumen

Paquete encargado de manejar la información sobre el robot Puma, entre el estado de la batería, logger y grabación de estadísticas del robot.

## Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [std_msgs](http://wiki.ros.org/std_msgs)
- puma_msgs
- nav_msgs
- geometry_msgs
- sensor_msgs
- actionlib
- actionlib_msgs
- message_generation

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_robot_status

## Ejecución

Se tienen 3 nodos para ejecutar con sus respectivos launch files:

    roslaunch puma_robot_status battery_status.launch
    roslaunch puma_robot_status logger.launch
    roslaunch puma_robot_status robot_statistics.launch
    roslaunch puma_robot_status robot_statistics_local.launch

- **`battery_status.launch`**: Nodo encargado de manejar el estado de la batería del robot Puma.

  - **`/puma/sensors/battery/status`** _(sensor_msgs/BatteryState)_: Publica el estado de la batería del robot Puma.

  - **`/puma/sensors/battery/raw_72v`** _(std_msgs/Float32)_: Lee el estado de la batería del robot Puma en voltios.

- **`logger.launch`**: Nodo encargado de manejar el logger del robot Puma.

  - **`/puma/logs/add_log`** _(puma_msgs/Log)_: Lee los logs del robot Puma y los almacena en un array.
  
  - **`/puma/logs/clean_logs`** _(std_msgs/Empty)_: Limpia los logs almacenados del robot Puma.
  
  - **`/puma/logs/logs`** _(puma_msgs/LogArray)_: Publica los logs del robot Puma.
  
  - **`/puma/logs`** _(puma_robot_status/LoggerManagerAction)_: Server action obtener los logs y limpiarlos del arreglo.
  
- **`robot_statistics.launch`**: Nodo encargado de manejar y grabar las estadísticas del robot. Crea un JSON con los datos grabados y los sube a la nube. Para la grabación se lee los datos de los tópicos:

  - **`/puma/sensors/battery/status`** _(sensor_msgs/BatteryState)_
  - **`/puma/sensors/gps/fix`** _(sensor_msgs/NavSatFix)_
  - **`/puma/localization/ekf_odometry`** _(nav_msgs/Odometry)_
  - **`/puma/arduino/status`** _(puma_msgs/ArduinoStatus)_
  - **`/puma/navigation/waypoints_completed`** _(puma_msgs/WaypointNav)_
  - **`/puma/navigation/waypoints_remained`** _(puma_msgs/WaypointNav)_
  - **`/puma/navigation/waypoints_list`** _(puma_msgs/WaypointNav)_
  - **`/puma/control/current_mode`** _(std_msgs/String)_
  - **`/puma/sensors/camera_front/color/image_raw/compressed`** _(sensor_msgs/CompressedImage)_
  - **`/puma/sensors/camera_rear/color/image_raw/compressed`** _(sensor_msgs/CompressedImage)_
  - **`/puma/sensors/camera_front/depth/image_rect_raw/compressed`** _(sensor_msgs/CompressedImage)_
  - **`/puma/sensors/camera_rear/depth/image_rect_raw/compressed`** _(sensor_msgs/CompressedImage)_
  
  Se utiliza el nodo a partir del server action:

    - **`/puma/statistics`** _(puma_robot_status/StatisticsManagerAction)_: Server action para grabar las estadísticas del robot Puma. Se utiliza "goal.action" `start` para empezar a grabar y `stop` para detenerlo, guardarlo y subirlo.

  Tiene parámetros de configuración:

    - **`upload_db`** (Boolean, default: false): Habilitar la subida a la nube.
    - **`backend_url`** (String, default: "http://localhost:5000"): URL del backend.
    - **`is_simulated`** (Boolean, default: false): Indica si el robot es simulado o real (Cambia ligeramente el tópico de la cámara de profundidad).

- **`robot_statistics_local.launch`**: Lanzador con la misma función que `robot_statistics.launch`, pero con la subida aun servidor local y considerando el robot simulado.