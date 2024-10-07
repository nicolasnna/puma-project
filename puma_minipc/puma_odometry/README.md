# Puma Odometry

## Resumen

Paquete ROS noetic para la estimación de la odometría de las ruedas del robot.

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_odometry

#### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [tf2_ros](http://wiki.ros.org/tf2_ros)
- puma_arduino_msgs
- [nav_msgs](http://wiki.ros.org/nav_msgs)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)

## Ejecución

Se puede ejecutar el nodo con valores por defecto con:

    rosrun puma_odometry puma_odometry_node.py

También se puede lanzar el nodo tomando un archivo de configuración con el _launch_:

    roslaunch puma_odometry puma_odometry.launch

## Launch

- **puma_odometry.launch:** Lanzador del para el nodo puma_odometry. Este lanzador cuenta con la carga de parámetros del archivo **`config/odometry_params.yaml`**.

## Config files

Se tiene un archivo **`yaml`** en el directorio **`config`** para la configuración de parámetros del nodo puma_odometry. Su contenido es el siguiente:

- **odometry_params.yaml**:

  - **`odometry`**

    Agrupación para los parámetros que se encargan de la odometría.

    - **`~wheel_base`** (float, valor: 1.1)

      Distancia entre el eje de las ruedas delanteras y traseras, para estimación de la dirección ackermann.

    - **`~frame_id`** (string, valor: "odom")

      Marco de coordenadas de referencia para la localización por odometría.

    - **`~child_frame_id`** (string, valor: "base_link")

      Marco de coordenadas base para la estimación de posición.

    - **`~direction_zero`** (int, valor: 395)

      Valor análogo del sensor de dirección cuando se encuentra en el ángulo 0. (Ambas ruedas delanteras apuntando hacia adelante)

  - **`tachometer`**

    Agrupación de parámetros encargados de la estimación de pulsos del motor a velocidad.

    - **`~calibrate_max_rpm_motor`** (int, valor: 1654)

      Valor de referencia de los rpm del motor.

    - **`~calibrate_max_velocity`** (int, valor: 22)

      Velocidad alcanzada para el rpm indicado en **`calibrate_max_rpm_motor`** en mph

    - **`~wheels_diameter`** (float, valor: 0.53)

      Valor del diámetro de las ruedas traseras en metros.

    - **`~kalman_q_noise`** (float, valor: 0.0007)

      Covarianza del ruido de proceso del filtro de kalman

    - **`~kalman_r_noise`** (float, valor: 0.001)

      Covarianza del ruido de medición del filtro de kalman
