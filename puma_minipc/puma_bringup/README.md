# Puma Bringup

## Resumen

Paquete ROS noetic para lanzar los paquetes necesarios en el minipc, para el funcionamiento del robot PUMA. Entre ellos esta:

- `move_base`
- `robot_localization`
- `realsense2_camera`
- `puma_controller`
- `rtabmap_odom`
- `rosserial_arduino`
- `puma_system_monitor`
- `puma_odometry`

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_bringup

### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [Realsense2_camera](http://wiki.ros.org/realsense2_camera)
- puma_odometry
- puma_controller
- puma_system_monitor
- [robot_localization](http://wiki.ros.org/robot_localization)
- [move_base](http://wiki.ros.org/move_base)
- [rtabmap_odom](http://wiki.ros.org/rtabmap_odom)

## Ejecución

El lanzador de todos los paquetes se realiza con el siguiente comando:

    roslaunch puma_bringup minipc_bringup.launch

Adicionalmente se tiene un lanzador para visualizar por rviz:

    roslaunch puma_bringup rviz.launch

## Launch

- **navigation_bringup.launch:** Lanzador de los nodos `puma_controller`, `convert_ackermann`, `map_server` y `move_base`. Usa los archivos de configuración ubicados en **`config/nav`** y el archivo **`navigation_params.yaml`**.

- **realsense_camera:** Ejecuta el nodo encargado de intermediar entre la camara realsense con ROS. Esta configurado para tener activo la imu interna de la cámara.

- **visual_odometry:** Ejecuta la odometria visual con `rtabmap_odom` y la camara realsense.

- **localization_bringup.launch:** Ejecuta la localización del robot mediante fusión de sensores con `robot_localization`. También ejecuta el nodo `puma_odometry`, `robot_state_publisher`, `joint_state_publisher` y `static_transform_publisher` (entre odom y map).

- **minipc_bringup.launch:** Ejecuta todos los lanzadores de arriba e incluye el nodo `serial_node` para la conexión del arduino nano a ROS y `puma_system_monitor` para el monitoreo del estado del PC.

- **rviz.launch:** Ejecuta una instancia en rviz con _displays_ configurados y preparados para el sistema.
