# Puma Nav Manager

## Resumen

Maneja los waypoints para la configuración de la navegación. Adicionalmente ofrece una interfaz para ajustar la localización con `robot_localization`, tanto para ajustar la orientación como para limpiar la posición.

## Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- puma_msgs
- nav_msgs
- geometry_msgs
- actionlib
- actionlib_msgs
- message_generation

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_nav_manager

## Scripts

- **`nav_manager_node.py`**: Ejecuta el gestor de waypoints para la navegación, incluyendo la importación y exportación de los waypoints en JSON.

  - **`/puma/navigation/files_manager`** _(puma_nav_manager/ImportExportManagerAction)_: Server action para importar y exportar los waypoints en JSON.
  - **`/puma/navigation/waypoints_manager`** _(puma_nav_manager/WaypointManagerAction)_: Server action para manejar los waypoints.
  - **`/puma/navigation/waypoints_list`** _(puma_msgs/WaypointNav)_: Publica la lista de waypoints.
  - **`/puma/navigation/waypoints_completed`** _(puma_msgs/WaypointNav)_: Publica los waypoints completados.
  - **`/puma/navigation/waypoints_remained`** _(puma_msgs/WaypointNav)_: Publica los waypoints restantes.

- **`manager_localization_node.py`**: Ejecuta el gestor de localización para ajustar la orientación y limpiar la posición.

  - **`/puma/localization/manager`** _(puma_nav_manager/LocalizationManagerAction)_: Server action para ajustar la orientación y limpiar la posición.
  - **`/puma/localization/ekf_odometry`** _(nav_msgs/Odometry)_: Lee la odometría del robot.
  - **`/puma/localization/filtered_map`** _(nav_msgs/Odometry)_: Lee la odometría del mapa (si es que está activada).
  - **`/puma/localization/change_angle_degree`** _(std_msgs/Float64)_: Cambia la orientación de la localización actual a patir de un ángulo YAW.
  - **`/puma/localization/set_new_pose`** _(geometry_msgs/PoseWithCovarianceStamped)_: Cambia la posición de la localización actual.
