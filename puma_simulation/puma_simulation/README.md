# Puma Simulation

## Resumen

Paquete con los archivos necesarios para la ejecución de la simulación del robot PUMA.

## Dependencias

- controller_manager
- joint_state_controller
- robot_state_publisher
- gazebo_ros_control
- velocity_controllers
- effort_controllers
- global_planner
- move_base

Adicionalmente emplea paquetes creados dentro del proyecto, tanto de `puma_jetson` como `puma_minipc`.

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_simulation

## Ejecución

Se ha creado diversos roslaunch para distintas pruebas. La principal es empleando lo siguiente:

    roslanch puma_simulation model_simulation.launch

Ejecuta gazebo con un modelo simplificado del robot, incluyendo sensores, control de modos del robot, y simulación del comportamiento base del robot, como los nodos fundamentales para su control.

    roslaunch puma_simulation localization_simulation.launch

Simula el sistema de localización del robot, usando `robot_localization`, además ejecuta la odometría visual con `rtabmap_ros` empleando la cámara realsense delantera y trasera.

    roslaunch puma_simulation test_navigation_simulation.launch

Ejecuta el servidor del mapa, manejo de los datos de la navegación y de la localización. Adicionalmente ejecuta el sistema de navegación usando el Stack `move_base`. Emplea planificadores local y global (`puma_local_planner` y `puma_global_planner`). También ejecuta la visualización en RVIZ.